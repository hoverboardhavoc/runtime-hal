//! Forced-fault resume validator for the detect BusFault fixup (both families).
//!
//! # Why this image exists
//!
//! The detect probe's BusFault fixup (the HAL naked entry `probe::bus_fault_entry` + `on_bus_fault`'s
//! symbol-anchored resume) is the load-bearing, host-untestable piece of runtime detection: no host or
//! emulator raises the real bus fault it recovers from. In PRODUCTION it is only ever exercised on an
//! F10x part, because the family probe faults there (reading the reserved F1x0 GPIO base). On an F1x0
//! part detect NEVER faults: the wrong-family base `0x4800_0000` is that part's own real GPIOA, and the
//! peripheral-presence sweep reads absent slots as zero rather than bus-faulting. So the family that
//! never triggers the fixup in production has no on-silicon coverage of the resume path.
//!
//! This image closes that gap. It deliberately reads a KNOWN-reserved, bus-faulting address
//! (`0x6000_0000`, the unpopulated FSMC / external-memory region on these parts) through runtime-hal's
//! public armed-probe harness, so the HAL's naked BusFault entry and the symbol-anchored resume run on
//! REAL silicon of BOTH the F103 and the F130. It records:
//!   - `faulted`  = 1 if the read bus-faulted and was caught (`probe_present` returned `None`),
//!   - `readback` = the value the read returned (garbage on a fault; meaningful only if `faulted == 0`),
//!   - `magic`    = written LAST, so a reader seeing it knows the fault was caught AND execution
//!                  RESUMED correctly (the run reached the end); a hang/HardFault would leave `magic == 0`.
//!
//! Strictly read-only on hardware: one reserved-address read, caught and resumed, with the probe-scoped
//! vector table restored afterward; no peripheral control state is touched. It busy-spins (NOT `wfi`;
//! a bare `wfi` locks GD32 SWD re-attach).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use runtime_hal::detect::probe;

/// A known-reserved address that bus-faults on both bench parts: `0x6000_0000` is the FSMC /
/// external-memory bank, which is unpopulated on the GD32F130C8 and the bench GD32F103C8, so a read
/// there raises a precise BusFault (confirmed on silicon: an SWD `mdw 0x6000_0000` errors on the F130).
const RESERVED_FAULT_ADDR: u32 = 0x6000_0000;

/// `0x46504E31` = "FPN1"; written LAST so a reader seeing it knows the whole run (fault caught +
/// resumed) completed.
const MAGIC: u32 = 0x4650_4E31;

/// The fixed-layout SWD-readable result. `#[repr(C)]`; `magic` is written LAST (0 = the run never
/// finished, i.e. the fault was NOT resumed).
#[repr(C)]
struct FaultPinResult {
    /// `MAGIC` once the run completed (fault caught AND resumed).
    magic: u32,
    /// The reserved address that was read (`RESERVED_FAULT_ADDR`), so the record is self-describing.
    forced_addr: u32,
    /// The value the armed read returned (garbage if it faulted; meaningful only if `faulted == 0`).
    readback: u32,
    /// 1 = the read bus-faulted and was caught (`probe_present` -> `None`); 0 = it read cleanly.
    faulted: u8,
    /// Padding to a 4-byte boundary so the decoder's offsets are obvious.
    _pad: [u8; 3],
}

/// Fixed RAM address of the result struct: the top of the (shrunk) RAM region reserved by `memory.x`
/// (cortex-m-rt ends RAM 256 B early so it never allocates here). The SWD reader reads this CONSTANT
/// directly; the size-optimised release ELF drops the `.symtab` an nm read would need.
const RESULT_ADDR: u32 = 0x2000_1F00;

/// Initial contents (the region is outside `.bss`, so the C runtime does not zero it): `magic = 0`
/// until the run finishes and writes `magic` LAST.
const INIT_RESULT: FaultPinResult = FaultPinResult {
    magic: 0,
    forced_addr: RESERVED_FAULT_ADDR,
    readback: 0,
    faulted: 0,
    _pad: [0; 3],
};

#[inline]
fn result_ptr() -> *mut FaultPinResult {
    RESULT_ADDR as *mut FaultPinResult
}

#[entry]
fn main() -> ! {
    // Initialise the fixed-address result region (outside .bss, not zeroed by the runtime); magic = 0
    // until the run completes.
    // SAFETY: RESULT_ADDR is reserved RAM (see memory.x); single writer, single-threaded bring-up.
    unsafe { core::ptr::write_volatile(result_ptr(), INIT_RESULT) };

    // Force the fault on a known-reserved address through the HAL's armed-probe harness. This drives
    // the exact machinery detect uses: the probe-scoped vector table (BusFault slot -> the naked
    // `bus_fault_entry`), `SHCSR.BUSFAULTENA`, then the single armed `probe_read32`. A bus fault at
    // `0x6000_0000` traps to `bus_fault_entry`, `on_bus_fault` fixes up the stacked PC via the
    // symbol-anchored resume, and `probe_present` returns `None`. If any of that is wrong on this
    // silicon (bad resume PC / clobbered callee-saved state) the core hangs or HardFaults here and
    // `magic` is never written.
    let result = probe::with_probe_vector_table(|| {
        let prev = probe::arm_busfault();
        let read = probe::probe_present(RESERVED_FAULT_ADDR);
        probe::disarm_busfault(prev);
        read
    });

    let (faulted, readback) = match result {
        None => (1u8, 0u32),
        Some(v) => (0u8, v),
    };

    // SAFETY: RESULT_ADDR is reserved RAM; single writer. Write the observations, then `magic` LAST.
    unsafe {
        let p = result_ptr();
        core::ptr::addr_of_mut!((*p).faulted).write_volatile(faulted);
        core::ptr::addr_of_mut!((*p).readback).write_volatile(readback);
        core::ptr::addr_of_mut!((*p).magic).write_volatile(MAGIC);
    }

    // Busy-spin, NOT wfi (a bare wfi with DBG_CTL0=0 locks GD32 SWD re-attach). This validator has no
    // reason to sleep; spin and stay re-attachable.
    loop {
        cortex_m::asm::nop();
    }
}
