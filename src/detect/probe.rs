//! The bus-fault-safe family probe and the peripheral-presence measurement.
//!
//! This is the load-bearing, hardware-only piece of runtime detection. It is always compiled in
//! (detection is the only path to a chip identity). It runs BEFORE any bring-up, on the reset clock
//! (IRC8M), before the production RAM vector table is installed.
//!
//! # The discriminator (spec section 4.1)
//!
//! The families diverge in where GPIOA lives and which RCU register enables its clock:
//!
//! | | F1x0 (GD32F130) | F10x (GD32F103) |
//! |---|---|---|
//! | GPIO-A clock enable register | `RCU_AHBEN` (RCU + 0x14) | `RCU_APB2EN` (RCU + 0x18) |
//! | GPIO-A clock enable bit (`PAEN`) | bit 17 | bit 2 |
//! | GPIO-A base (control register read) | `0x4800_0000` | `0x4001_0800` |
//!
//! The wrong-family base is a RESERVED region that bus-faults rather than aliasing to RAM, so a
//! fault is an unambiguous negative. The shared RCU base (`0x4002_1000`) means we can always enable a
//! GPIO clock; we just enable it in the family-correct RCU register, and the two families put `PAEN`
//! in different registers at different bits.
//!
//! # The fault-safe mechanism (spec section 3) and the PC-fixup (the MAIN RISK)
//!
//! Probing the wrong family's GPIO base raises a BusFault on Cortex-M3. The probe CATCHES that fault
//! and reads it as "this base is not present" rather than letting it hang the boot:
//!
//! - [`run`] sets `SCB.SHCSR.BUSFAULTENA` so a precise data-bus error traps to the dedicated BusFault
//!   handler rather than escalating to HardFault, and restores it afterward.
//! - Three atomics coordinate the probe and the handler: `EXPECTING_FAULT` (armed before each
//!   candidate access, disarmed after a clean read), `PROBED_ADDR` (the candidate base, so the
//!   handler can confirm `BFAR == PROBED_ADDR`), and `FAULTED` (the handler sets it so the probe
//!   learns the access faulted).
//! - **The PC fixup (the risk).** On a faulted access the handler MUST advance the stacked return PC
//!   past the faulting load so execution resumes AFTER the probe access instead of re-executing it
//!   (which would re-fault forever). The probe emits the candidate access as a FIXED-WIDTH 32-bit
//!   volatile read (`probe_read32`, `#[inline(never)]`), which lowers to a 32-bit Thumb-2 `LDR`
//!   (4 bytes), so the handler adds [`FAULT_SKIP_WIDTH`] = 4 to the stacked PC. DF-5: this is the
//!   pinned-fixed-width approach (the alternative is a trampolined re-execute). Getting this `+4`
//!   wrong is the one piece that needs careful testing on BOTH real parts; it is the explicit
//!   acceptance criterion of the DF-T6 bench test. The access is placed outside any IT block (a plain
//!   call) so the xPSR IT-state complication does not arise. **This fixup is validated ONLY on
//!   hardware**; no host/emulator raises the fault it fixes up (spec section 8.2).
//!
//! # Who owns the BusFault handler (the HAL, via a probe-scoped vector table)
//!
//! The HAL owns the BusFault handling end to end; the application defines NO `#[exception] BusFault`.
//! cortex-m-rt's flash vector table still carries whatever BusFault handler the application linked (if
//! any), but for the duration of the probe the HAL installs its OWN vector table in RAM:
//!
//! - [`with_probe_vector_table`] copies the currently-active table (from `SCB.VTOR`) into a HAL-owned,
//!   suitably aligned RAM table, OVERRIDES the BusFault slot (exception number 5, table index 5,
//!   vector offset `0x14`) with the address of the HAL-internal naked entry [`bus_fault_entry`], points
//!   `SCB.VTOR` at the RAM table (with a `dsb`/`isb` barrier), runs the probe closure, then RESTORES
//!   the prior `VTOR` (again barriered).
//! - [`bus_fault_entry`] is a NAKED function so the hardware can call it directly from the relocated
//!   table: on exception entry the core has already stacked the frame and set `LR = EXC_RETURN`, and a
//!   naked entry emits no prologue that would move `SP` before the frame pointer is captured. It
//!   captures `LR`/`MSP`/`PSP`, selects the frame per EXC_RETURN bit 2 (0 => MSP, 1 => PSP), calls
//!   `on_bus_fault` for the PC fix-up, and returns from the exception with `BX LR`.
//!
//! The swap is fully probe-scoped (installed and restored inside detection), so it never interferes
//! with a production RAM vector table the application installs later. This relocated-table mechanism
//! is implemented and host-tested for its table-build/alignment/offset logic; the bus-fault recovery
//! it drives is re-validated on real silicon by `bench-fw-detect` / `bench-fw-probe`.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::peripheral::{scb::Exception, SCB};

use super::{Family, FLASH_DENSITY_ADDR};

// --- the probe-scoped relocated vector table (HAL-owned BusFault) -----------------------------
//
// The HAL owns the BusFault handling: during detection it installs its OWN vector table in RAM whose
// BusFault slot points at the HAL-internal naked entry below, runs the probe, then restores VTOR. The
// application therefore defines NO `#[exception] BusFault`. See the module docs for the full rationale.

/// The number of `u32` entries the probe-scoped RAM vector table holds: the initial-SP word + 15
/// system-exception vectors + a generous IRQ margin. ARMv7-M has at most 240 external IRQs; copying a
/// generous prefix is cheap and keeps every handler the active table already had. 256 entries (1 KiB)
/// covers the system vectors and the IRQ lines these parts use with margin to spare.
const VECTOR_TABLE_LEN: usize = 256;

/// The exception number (and table index) of BusFault on ARMv7-M: vector offset `0x14` => word index
/// 5. This is the only slot the HAL overrides; every other entry is copied from the active table so
/// existing handlers (Reset, NMI, HardFault, ...) keep working.
const BUSFAULT_VECTOR_INDEX: usize = 5;

/// A HAL-owned RAM vector table for the probe window.
///
/// `#[repr(align(1024))]` satisfies the ARMv7-M `VTOR` alignment requirement: the table base must be
/// aligned to a power of two that is >= its byte size and >= 128 bytes. The table is 256 words = 1 KiB
/// (`VECTOR_TABLE_LEN`), so aligning the static to a full 1 KiB makes its own address a valid `VTOR`
/// base directly; [`vector_table_base`] rounds up to the same boundary as a defensive no-op (so a
/// misaligned base could never be programmed, and the indexed slots always land inside this static).
#[repr(align(1024))]
struct AlignedVectorTable {
    entries: [u32; VECTOR_TABLE_LEN],
}

/// The single probe-scoped RAM vector table instance. Written only by [`install_probe_vector_table`]
/// (single-threaded bring-up context, before any IRQs are enabled) and read by the hardware as the
/// vector table while `VTOR` points at it.
static mut PROBE_VECTOR_TABLE: AlignedVectorTable = AlignedVectorTable {
    entries: [0; VECTOR_TABLE_LEN],
};

/// The byte-size of the RAM vector table (used as the `VTOR` alignment requirement).
const VECTOR_TABLE_BYTES: u32 = (VECTOR_TABLE_LEN * core::mem::size_of::<u32>()) as u32;

/// Compute the `VTOR` base to program for the RAM table: the table's own address rounded UP to a
/// multiple of its byte size, which is the ARMv7-M requirement (base aligned to a power of two >= the
/// table byte size). Because `AlignedVectorTable` is `#[repr(align(1024))]` and the table is exactly
/// 1 KiB, the static's address is ALREADY a multiple of `VECTOR_TABLE_BYTES`, so this round-up is a
/// no-op for the real base; it is kept as a defensive guarantee that the programmed `VTOR` is always
/// table-size-aligned (the indexed slots then always land inside the static).
#[inline]
fn vector_table_base(addr: u32) -> u32 {
    let align = VECTOR_TABLE_BYTES;
    addr.wrapping_add(align - 1) & !(align - 1)
}

/// Build the probe-scoped table by copying the active table then overriding the BusFault slot.
///
/// `active_vtor` is the current `SCB.VTOR` base (the flash table cortex-m-rt linked, or whatever table
/// is active). We copy `VECTOR_TABLE_LEN` words from it so every existing handler (HardFault, NMI, the
/// IRQ lines, ...) is preserved, then write [`bus_fault_entry`]'s address into the BusFault slot. A
/// Rust fn pointer on thumbv7m already has the Thumb bit (bit 0) set, which the hardware requires for
/// an exception vector, so `bus_fault_entry as usize as u32` is the correct value to store.
///
/// # Safety
/// `active_vtor` must be a readable vector-table base (it comes from `SCB.VTOR`). Single-threaded
/// bring-up context: this is the only writer of [`PROBE_VECTOR_TABLE`], and IRQs are not enabled.
unsafe fn build_probe_vector_table(active_vtor: u32) {
    let dst = core::ptr::addr_of_mut!(PROBE_VECTOR_TABLE.entries) as *mut u32;
    let src = active_vtor as *const u32;
    // Copy the active table (system exceptions + a generous IRQ prefix). The source is the real
    // (flash) table base, so reading VECTOR_TABLE_LEN words from it is in-bounds on these parts.
    let mut i = 0usize;
    while i < VECTOR_TABLE_LEN {
        core::ptr::write_volatile(dst.add(i), core::ptr::read_volatile(src.add(i)));
        i += 1;
    }
    // Override ONLY the BusFault slot with the HAL-internal naked entry. The fn pointer carries the
    // Thumb bit; an exception vector must have bit 0 set, so store the pointer value verbatim.
    let entry = bus_fault_entry as *const () as usize as u32;
    core::ptr::write_volatile(dst.add(BUSFAULT_VECTOR_INDEX), entry);
}

/// `SCB.VTOR` address (`0xE000_ED08`), the ARMv7-M Vector Table Offset Register.
#[cfg(target_arch = "arm")]
const VTOR_ADDR: u32 = 0xE000_ED08;

/// Read `VTOR` through an inline-asm `LDR`.
///
/// Routed through `asm!` (rather than `SCB.vtor.read()`) deliberately: the install/restore of the
/// relocated table has no effect the Rust abstract machine can observe (the hardware, not the program,
/// reads `VTOR` and fetches the vector), so an optimizer that proves the program never observes the
/// intermediate `VTOR` value may delete the whole swap. Doing the access via `asm!` with a `memory`
/// clobber makes it an opaque side effect the compiler must not reorder past or eliminate.
///
/// # Safety
/// Reads a system control register; no side effect beyond the read itself.
#[cfg(target_arch = "arm")]
#[inline]
unsafe fn read_vtor() -> u32 {
    let value: u32;
    core::arch::asm!(
        "ldr {v}, [{a}]",
        v = out(reg) value,
        a = in(reg) VTOR_ADDR,
        options(nostack, preserves_flags),
    );
    value
}

/// Host stub for [`read_vtor`] (mock / non-`arm` builds). The probe never runs on the host (no real
/// bus fault is raised), so the vector-table swap is dead there; this keeps the crate compiling.
#[cfg(not(target_arch = "arm"))]
#[inline]
unsafe fn read_vtor() -> u32 {
    0
}

/// Write `VTOR` and barrier (`dsb`/`isb`) through inline asm with a `memory` clobber.
///
/// The `memory` clobber + the `dsb`/`isb` make this a hard barrier the optimizer cannot delete or move
/// the relocated-table stores or the probe across, so the swap genuinely brackets the probe on silicon
/// (see [`read_vtor`] for why a plain volatile write was not enough).
///
/// # Safety
/// Reprograms the vector-table base; the caller must ensure `base` points at a valid, suitably aligned
/// vector table for the duration it is active.
#[cfg(target_arch = "arm")]
#[inline]
unsafe fn write_vtor(base: u32) {
    core::arch::asm!(
        "str {b}, [{a}]",
        "dsb",
        "isb",
        b = in(reg) base,
        a = in(reg) VTOR_ADDR,
        options(nostack, preserves_flags),
    );
}

/// Host stub for [`write_vtor`] (mock / non-`arm` builds). See [`read_vtor`]: the swap is dead on host.
#[cfg(not(target_arch = "arm"))]
#[inline]
unsafe fn write_vtor(_base: u32) {}

/// Install the probe-scoped RAM vector table and return the prior `VTOR` to restore later.
///
/// Saves the current `VTOR`, builds the RAM table from it (copy + BusFault override), points `VTOR` at
/// the RAM table base, then `dsb`/`isb` so the core fetches subsequent vectors from the new table.
///
/// # Safety
/// Single-threaded bring-up context; mutates `VTOR` and [`PROBE_VECTOR_TABLE`]. The caller MUST pair
/// this with [`restore_vector_table`] passing the returned value before returning from detection, so
/// the swap stays probe-scoped.
unsafe fn install_probe_vector_table() -> u32 {
    let prev_vtor = read_vtor();
    build_probe_vector_table(prev_vtor);
    let base = vector_table_base(core::ptr::addr_of!(PROBE_VECTOR_TABLE) as u32);
    write_vtor(base);
    prev_vtor
}

/// Restore `VTOR` to `prev_vtor` (the value [`install_probe_vector_table`] returned), barriered, so the
/// core resumes using the prior (flash) vector table after detection.
///
/// # Safety
/// Single-threaded bring-up context; mutates `VTOR`. Pass the exact value
/// [`install_probe_vector_table`] returned.
unsafe fn restore_vector_table(prev_vtor: u32) {
    write_vtor(prev_vtor);
}

/// Run `f` with the HAL's probe-scoped vector table installed (BusFault slot -> [`bus_fault_entry`]),
/// restoring the prior `VTOR` afterward. This is what makes the BusFault handling fully HAL-owned: the
/// application defines no fault handler; the HAL relocates the table around the probe and restores it.
///
/// The install/restore is barriered (`dsb`/`isb`) on both edges and is strictly probe-scoped, so it
/// never collides with a production RAM vector table the application installs after detection.
///
/// [`run`] and [`measure_counts`] already wrap themselves in this, so [`crate::detect::detect_chip`]
/// needs nothing extra. It is exposed so a standalone probe sweep built from the lower-level helpers
/// ([`arm_busfault`] / [`probe_present`] / [`rcu_enable_sticky`]) can run under the same HAL-owned
/// BusFault entry, e.g. the bench probe-presence validator.
pub fn with_probe_vector_table<R>(f: impl FnOnce() -> R) -> R {
    // SAFETY: single-threaded bring-up context (before IRQs are enabled). We save VTOR, install the
    // HAL table, run the probe, then restore VTOR on every path (no unwinding in this no_std build).
    let prev_vtor = unsafe { install_probe_vector_table() };
    let result = f();
    // SAFETY: restoring the exact VTOR install returned, keeping the swap probe-scoped.
    unsafe { restore_vector_table(prev_vtor) };
    result
}

// --- the exact registers, addresses, and bits (spec section 4.1) ------------------------------

/// The shared RCU base (`0x4002_1000` on both families).
pub const RCU_BASE: u32 = 0x4002_1000;
/// `RCU_AHBEN` offset (F1x0 GPIO-A clock enable lives here).
const RCU_AHBEN: u32 = 0x14;
/// `RCU_APB2EN` offset (F10x GPIO-A clock enable lives here).
const RCU_APB2EN: u32 = 0x18;
/// F1x0 `RCU_AHBEN.PAEN` bit (GPIO-A clock enable).
const F1X0_PAEN_BIT: u32 = 17;
/// F10x `RCU_APB2EN.PAEN` bit (GPIO-A clock enable).
const F10X_PAEN_BIT: u32 = 2;

/// F1x0 GPIO-A base; a clean control-register read here => F1x0 family.
pub const F1X0_GPIOA_BASE: u32 = 0x4800_0000;
/// F10x GPIO-A base; a clean control-register read here => F10x family.
pub const F10X_GPIOA_BASE: u32 = 0x4001_0800;

/// The width (bytes) of the pinned probe-read instruction; the handler advances the stacked PC by
/// this on a fault. A 32-bit Thumb-2 `LDR` is 4 bytes. DF-5: this MUST match `probe_read32`'s
/// emitted instruction width or the PC-fixup resumes at the wrong place.
pub const FAULT_SKIP_WIDTH: u32 = 4;

// --- the probe <-> handler shared state -------------------------------------------------------

/// Armed by [`run`] before each candidate access, disarmed after a clean read. The handler treats a
/// fault while this is `false` as a REAL fault (not a probe access) and does NOT fix it up.
static EXPECTING_FAULT: AtomicBool = AtomicBool::new(false);
/// The candidate base currently being read; the handler confirms `BFAR == PROBED_ADDR` when
/// `BFSR.BFARVALID` is set (a precise BusFault latches the faulting address in BFAR).
static PROBED_ADDR: AtomicU32 = AtomicU32::new(0);
/// Set by the handler so the probe learns the access faulted (the family-negative signal).
static FAULTED: AtomicBool = AtomicBool::new(false);

/// The probe result: the detected family and the flash-density read (the F10x page-size input).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Detected {
    /// The family the GPIO+RCU probe resolved.
    pub family: Family,
    /// `FLASH_DENSITY[15:0]` (KiB of flash), read from `0x1FFF_F7E0`. Corroboration + the F10x
    /// `flash_page` input (spec section 4.3 / 5.2). Read after the family decision; advisory for
    /// F1x0 (constant K1).
    pub flash_kib: u16,
}

// --- the fixed-width pinned probe read (DF-5) -------------------------------------------------

/// Read a 32-bit control word at `addr` as a SINGLE fixed-width volatile load.
///
/// `#[inline(never)]` so the access is a standalone 32-bit `LDR` the handler's `+4` PC-fixup matches;
/// not inlined into a context where the compiler might fuse or re-widen it. On thumbv7m a
/// `read_volatile::<u32>` of an aligned address lowers to a 32-bit Thumb-2 `LDR` (4 bytes), which is
/// why [`FAULT_SKIP_WIDTH`] is 4. If the access faults, the handler advances the stacked PC past this
/// `LDR` and returns; the returned value is then meaningless (the caller checks `FAULTED` first).
///
/// # Safety
/// `addr` is a candidate peripheral base; the read is wrapped by the armed BusFault handler so a
/// fault on the wrong-family (reserved) base is caught instead of escalating.
#[inline(never)]
fn probe_read32(addr: u32) -> u32 {
    // SAFETY: the access is bounded by the armed fault harness (EXPECTING_FAULT + the BusFault
    // handler). A fault here is caught and turned into the family-negative signal.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Set `bit` in the RCU enable register at `RCU_BASE + reg_off` (RMW; leave the rest unchanged), the
/// single-bit set `rcu_periph_clock_enable` performs. Bare volatile RMW (no [`crate::reg`] so the
/// probe has no dependency on the descriptor / Chip; it runs before any of that exists).
#[inline]
fn rcu_set_bit(reg_off: u32, bit: u32) {
    let addr = (RCU_BASE + reg_off) as *mut u32;
    // SAFETY: the RCU base is shared and always present on both families; this is a control-register
    // RMW that enables a GPIO clock, the prerequisite for the family-correct GPIOA read.
    unsafe {
        let cur = core::ptr::read_volatile(addr);
        core::ptr::write_volatile(addr, cur | (1 << bit));
    }
}

// --- the ordered probe (DF-T4, spec section 4.2) ----------------------------------------------

/// Run the ordered GPIO+RCU family probe ONCE inside the fault-safe harness, returning the detected
/// family (or `None` if neither matched => fail safe).
///
/// Sequence (spec section 4.2):
/// 1. **F1x0 probe.** Set `RCU_AHBEN.PAEN` (bit 17). Read GPIOA control at `0x4800_0000`. A clean
///    read => F1x0. A bus-fault => not F1x0; proceed to step 2.
/// 2. **F10x probe** (only if step 1 faulted). Set `RCU_APB2EN.PAEN` (bit 2). Read GPIOA control at
///    `0x4001_0800`. A clean read => F10x. A bus-fault => NEITHER family; fail safe (`None`).
/// 3. Read the flash-density register for the F10x page-size input (corroboration; the GPIO result
///    is authoritative).
///
/// `run` installs the HAL's probe-scoped vector table (BusFault slot -> [`bus_fault_entry`]) for the
/// duration of the probe, sets `SHCSR.BUSFAULTENA` on entry, and restores both on every exit, so a
/// precise data-bus error traps to the HAL-internal BusFault handler rather than escalating to
/// HardFault, and the application defines no fault handler. It does NOT retry or loop.
pub fn run() -> Option<Detected> {
    // The vector-table swap is strictly probe-scoped: install -> probe -> restore, all inside this
    // call. The HAL's BusFault entry handles a faulted candidate read; every other vector is the
    // application's (copied from the active table).
    with_probe_vector_table(|| {
        // Enable the dedicated BusFault handler so a precise reserved-read fault traps to it (not
        // HardFault). Remember the prior state so we can restore it.
        // SAFETY: bring-up context, single core, no concurrent SCB users; we restore below.
        let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
        let bf_was_enabled = scb.is_enabled(Exception::BusFault);
        scb.enable(Exception::BusFault);

        let result = run_inner();

        // Restore the prior BUSFAULTENA state. The probe handler is strictly boot-temporary.
        if !bf_was_enabled {
            // Undo our enable so we leave SHCSR as we found it.
            scb.disable(Exception::BusFault);
        }
        // The probe leaves the shared atomics disarmed.
        EXPECTING_FAULT.store(false, Ordering::SeqCst);

        result
    })
}

/// The ordered candidate set, run with BUSFAULTENA already set.
fn run_inner() -> Option<Detected> {
    // Step 1: F1x0. Enable GPIOA's clock in the F1x0-correct RCU register, then read GPIOA control.
    rcu_set_bit(RCU_AHBEN, F1X0_PAEN_BIT);
    if probe_candidate(F1X0_GPIOA_BASE).is_some() {
        // Clean read at the F1x0 base => F1x0 family. (The known-good F130 readback is 0x682a73a3;
        // it MAY be used as an extra plausibility gate but is not load-bearing, the wrong base
        // faults rather than returning garbage, so "did not fault" is already the strong signal.)
        return Some(Detected {
            family: Family::F1x0,
            flash_kib: read_flash_density(),
        });
    }

    // Step 2: F10x (only reached if step 1 faulted). Enable GPIOA in the F10x-correct RCU register,
    // then read GPIOA control at the F10x base.
    rcu_set_bit(RCU_APB2EN, F10X_PAEN_BIT);
    if probe_candidate(F10X_GPIOA_BASE).is_some() {
        return Some(Detected {
            family: Family::F10x,
            flash_kib: read_flash_density(),
        });
    }

    // Both candidates faulted: NEITHER family matched. Fail safe (do not guess).
    None
}

/// Read one candidate GPIOA control register inside the armed fault window. Returns `Some(value)` on
/// a clean read, `None` if the access bus-faulted (the family-negative signal).
fn probe_candidate(base: u32) -> Option<u32> {
    PROBED_ADDR.store(base, Ordering::SeqCst);
    FAULTED.store(false, Ordering::SeqCst);
    // Arm: a fault between here and the disarm below is treated as a probe fault and fixed up.
    EXPECTING_FAULT.store(true, Ordering::SeqCst);
    cortex_m::asm::dsb();

    let value = probe_read32(base);

    cortex_m::asm::dsb();
    EXPECTING_FAULT.store(false, Ordering::SeqCst);

    if FAULTED.load(Ordering::SeqCst) {
        None
    } else {
        Some(value)
    }
}

// --- the peripheral-presence measurement (folded into detect_chip) ----------------------------
//
// These GENERALIZE the family probe's machinery for the peripheral-presence MEASUREMENT: instead of
// resolving F1x0-vs-F10x, MEASURE which advanced timers / ADCs a given instance actually has, per
// chip, rather than inferring counts from a family constant. They reuse the SAME shared atomics
// (EXPECTING_FAULT / PROBED_ADDR / FAULTED) and the SAME fixed-width `probe_read32` + `+4` PC-fixup
// `on_bus_fault` as `run`; no new private duplicate state. `run` itself is unchanged. The whole sweep
// runs under ONE BusFault enable (rather than per-candidate like `run`), so the SCB enable/disable is
// split out into [`arm_busfault`] / [`disarm_busfault`] and the per-access armed read is exposed as
// [`probe_present`]. `bench-fw-probe/` is the standalone validator that reports the raw sub-signals;
// the library function [`measure_counts`] is the production caller `detect_chip` uses.

/// Enable the dedicated BusFault handler (`SHCSR.BUSFAULTENA`) so a precise reserved-read fault traps
/// to the BusFault handler instead of escalating to HardFault. Returns the PRIOR enabled state so the
/// caller can restore it with [`disarm_busfault`]. A standalone sweep calls this ONCE before probing
/// all candidates and restores once at the end (whereas `run` does its own enable/restore internally).
///
/// This only arms `SHCSR.BUSFAULTENA`; it does NOT install the vector table. A caller that drives the
/// sweep from these lower-level helpers must run them inside [`with_probe_vector_table`] so the
/// HAL-internal BusFault entry is the one that fields the fault.
///
/// # Safety
/// Bring-up / single-core context only; mutates `SHCSR`. The caller must pair this with
/// [`disarm_busfault`] passing the returned value, and must run inside [`with_probe_vector_table`] so
/// a probe fault reaches the HAL-internal [`bus_fault_entry`].
pub fn arm_busfault() -> bool {
    // SAFETY: bring-up context, single core, no concurrent SCB users; the caller restores via
    // disarm_busfault.
    let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
    let was_enabled = scb.is_enabled(Exception::BusFault);
    scb.enable(Exception::BusFault);
    was_enabled
}

/// Restore `SHCSR.BUSFAULTENA` to the state [`arm_busfault`] reported (`prev`). If BusFault was not
/// enabled before the sweep, disable it again; otherwise leave it on (the caller had it on for its own
/// reasons). Also disarms the probe window so a later real fault is never mistaken for a probe access.
///
/// # Safety
/// Bring-up / single-core context only; mutates `SHCSR`. Pass the exact value [`arm_busfault`]
/// returned.
pub fn disarm_busfault(prev: bool) {
    // SAFETY: as arm_busfault; restoring the prior state.
    let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
    if !prev {
        scb.disable(Exception::BusFault);
    }
    // Leave the shared probe window disarmed (a real fault after the sweep is a genuine error).
    EXPECTING_FAULT.store(false, Ordering::SeqCst);
}

/// Fault-safe read of an ARBITRARY 32-bit address inside the armed BusFault window. Generalizes
/// `probe_candidate` (which is GPIOA-specific) for the peripheral-presence sweep: arm the shared
/// window, do the fixed-width `probe_read32`, disarm, and report `None` if the access faulted (the
/// address is absent / reserved) or `Some(value)` on a clean read.
///
/// The caller MUST have already enabled `SHCSR.BUSFAULTENA` (via [`arm_busfault`]) and be running
/// inside [`with_probe_vector_table`]; this function does NOT touch the SCB enable or the vector table,
/// so a whole candidate sweep can run under one enable/restore. It reuses the SAME `EXPECTING_FAULT` /
/// `PROBED_ADDR` / `FAULTED` atomics and the SAME `+4` `on_bus_fault` PC-fixup as `run`.
///
/// # Safety
/// `addr` is a candidate peripheral register address; the read is bounded by the armed fault harness
/// (the caller's [`arm_busfault`] + the HAL's probe-scoped [`bus_fault_entry`] -> `on_bus_fault`), so
/// a fault on an absent/reserved address is caught and reported as `None` instead of escalating.
pub fn probe_present(addr: u32) -> Option<u32> {
    PROBED_ADDR.store(addr, Ordering::SeqCst);
    FAULTED.store(false, Ordering::SeqCst);
    // Arm: a fault between here and the disarm below is treated as a probe fault and fixed up (+4).
    EXPECTING_FAULT.store(true, Ordering::SeqCst);
    cortex_m::asm::dsb();

    let value = probe_read32(addr);

    cortex_m::asm::dsb();
    EXPECTING_FAULT.store(false, Ordering::SeqCst);

    if FAULTED.load(Ordering::SeqCst) {
        None
    } else {
        Some(value)
    }
}

/// Enable `bit` in the RCU enable register at `RCU_BASE + reg_off` and report whether it STICKS (reads
/// back as 1). This is the clock-gate "stickiness" presence signal: an RCU enable bit for a peripheral
/// that does NOT exist on this part typically reads back 0 (the bit is Reserved / not implemented),
/// whereas a present peripheral's enable bit latches to 1. RMW (preserve the other clock gates), then
/// read back and test the one bit. The bit is LEFT SET, which is harmless (it only gates a clock; the
/// sweep never configures the peripheral behind it).
///
/// Reuses the same bare-volatile RMW idiom as `rcu_set_bit` (no `crate::reg` dependency; the bench
/// sweep runs before any descriptor/Chip exists), but additionally reads back and returns the result.
///
/// # Safety note
/// This only sets a clock-enable bit in the shared, always-present RCU register; it has no side effect
/// beyond gating a peripheral clock on. It NEVER touches a peripheral's own control / output state.
pub fn rcu_enable_sticky(reg_off: u32, bit: u32) -> bool {
    let addr = (RCU_BASE + reg_off) as *mut u32;
    // SAFETY: RCU base is shared and always present on both families; this is a control-register RMW
    // that enables a peripheral clock (a benign clock gate), then a read-back to test stickiness.
    unsafe {
        let cur = core::ptr::read_volatile(addr);
        core::ptr::write_volatile(addr, cur | (1 << bit));
        let readback = core::ptr::read_volatile(addr);
        (readback & (1 << bit)) != 0
    }
}

/// `RCU_APB2EN` offset (`0x18`), re-exported for the bench sweep so it can address the per-peripheral
/// clock-enable bits (TIMER0EN/TIMER7EN/ADCxEN all live in `RCU_APB2EN` on both families). The private
/// `RCU_APB2EN` constant above is the one `run` uses for the F10x GPIO-A clock; this public alias names
/// the same register for the sweep without widening the private item's visibility.
pub const RCU_APB2EN_OFFSET: u32 = RCU_APB2EN;

/// Read `FLASH_DENSITY[15:0]` (KiB of flash) from `0x1FFF_F7E0`. Always readable on these parts (the
/// flash information block needs no RCU enable and never bus-faults, spec section 4.3), so this read
/// is outside the fault window.
fn read_flash_density() -> u16 {
    // SAFETY: the flash information block is always mapped and readable; reading it has no side
    // effect and cannot fault on these parts.
    let word = unsafe { core::ptr::read_volatile(FLASH_DENSITY_ADDR as *const u32) };
    (word & 0xFFFF) as u16
}

// --- the measured advanced-timer / ADC counts -------------------------------------------------
//
// The family constant is wrong in BOTH directions on real parts: a GD32F103C8 has 1 advanced timer
// (not the family-constant 2), and a GD32F103RCT6 has 3 ADCs (not 2). So `detect_chip` MEASURES the
// per-instance presence of each advanced timer (TIMER0, TIMER7) and each ADC (ADC0, ADC1, ADC2) and
// counts the present ones. The presence test is a BENIGN scratch write-back: write a recognizable
// pattern to a side-effect-free R/W register and read it back. Validated on three boards (F103C8,
// F130C8, F103RCT6) in `bench/peripheral-probe-2026-06-18.md`.
//
// IMPORTANT (the bench's methodological finding): on this GD32 silicon the OTHER two candidate
// signals are NOT reliable presence tests. The RCU enable-bit reads back as 1 even for a Reserved
// (absent) peripheral, and an absent peripheral slot reads-as-zero rather than bus-faulting. ONLY the
// scratch write-back discriminates present from absent, so `measure_counts` relies on it.

/// The MEASURED advanced-timer and ADC instance counts (the count of candidates whose benign scratch
/// register retained a written pattern).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeasuredCounts {
    /// Number of advanced timers present (TIMER0 + TIMER7, by write-back).
    pub adv_timers: u8,
    /// Number of ADC instances present (ADC0 + ADC1 + ADC2, by write-back).
    pub adc_count: u8,
}

/// The side-effect-free scratch register offset (bytes from the peripheral base) used for the
/// write-back presence test: `TIMERx_PSC` for an advanced timer, `ADC_WDLT` for an ADC. Both sit at
/// offset `0x28` and both reset to 0 (verified against the GD32F10x Rev2.6 / GD32F1x0 Rev3.6 user
/// manuals). `TIMERx_PSC` only loads the prescaler shadow (no counter start, no output, no MOE/POEN,
/// no pin map); `ADC_WDLT` is the analog-watchdog low threshold (inert unless the watchdog is
/// separately enabled, which this never does). Both are restored to their reset value after the test.
const SCRATCH_OFFSET: u32 = 0x28;
/// The reset value the scratch register is restored to after the write-back test (0 for both PSC and
/// WDLT).
const SCRATCH_RESET: u32 = 0x0000_0000;
/// The benign test pattern written to the scratch register. `PSC` retains 16 bits and `ADC_WDLT` 12,
/// so a 12-bit-clean pattern is compared against the retained low bits.
const SCRATCH_PATTERN: u32 = 0x0000_5A5A;
/// The mask of scratch-register bits the comparison checks (the low 12 bits, which both PSC and WDLT
/// retain). `0x5A5A` fits in 12 bits, so one mask works for both.
const SCRATCH_RETAIN_MASK: u32 = 0x0000_0FFF;

/// The two advanced-timer instance bases (TIMER0, TIMER7) on both families (APB2 map).
const ADV_TIMER_BASES: [u32; 2] = [0x4001_2C00, 0x4001_3400];
/// The three ADC instance bases (ADC0, ADC1, ADC2) on both families (APB2 map).
const ADC_BASES: [u32; 3] = [0x4001_2400, 0x4001_2800, 0x4001_3C00];

/// MEASURE the advanced-timer and ADC instance counts of the running part, instead of trusting the
/// family constant. For each candidate peripheral it runs the benign scratch write-back presence test
/// (`scratch_present`) and counts the present ones.
///
/// Strictly read-only + benign-scratch-restore: it NEVER sets MOE/POEN/channel-enable/gate pins and
/// NEVER configures PWM. It installs the HAL's probe-scoped vector table (BusFault slot ->
/// [`bus_fault_entry`]) and enables the dedicated BusFault handler ONCE around the whole sweep (the
/// scratch read is done inside the armed window so an unexpected fault is caught, not escalated), then
/// restores both on exit. The application defines no fault handler.
///
/// NOT host-testable (the same reason as [`run`]: the write-back to an absent slot relies on real
/// silicon behavior no host/emulator reproduces); validated on the bench.
pub fn measure_counts() -> MeasuredCounts {
    // Probe-scoped vector-table swap around the whole sweep (install -> sweep -> restore).
    with_probe_vector_table(|| {
        let prev = arm_busfault();

        let mut adv_timers = 0u8;
        let mut i = 0;
        while i < ADV_TIMER_BASES.len() {
            if scratch_present(ADV_TIMER_BASES[i]) {
                adv_timers += 1;
            }
            i += 1;
        }

        let mut adc_count = 0u8;
        let mut j = 0;
        while j < ADC_BASES.len() {
            if scratch_present(ADC_BASES[j]) {
                adc_count += 1;
            }
            j += 1;
        }

        disarm_busfault(prev);
        MeasuredCounts {
            adv_timers,
            adc_count,
        }
    })
}

/// The benign scratch write-back presence test for one peripheral base, inside the armed BusFault
/// window. Read the scratch register first (a fault => absent), write the test pattern, read it back,
/// then RESTORE the reset value. Returns whether the retained bits read back as the pattern (a present
/// peripheral retains it; an absent / read-as-zero slot does not).
///
/// Side-effect-free: the scratch register (`TIMERx_PSC` / `ADC_WDLT`) only loads a prescaler /
/// watchdog-threshold shadow and is restored to its reset value (see [`SCRATCH_OFFSET`]).
fn scratch_present(base: u32) -> bool {
    let scratch = base + SCRATCH_OFFSET;

    // Read the scratch register inside the armed window FIRST: a fault here (a truly reserved address)
    // is caught and means absent. A clean read proves the address is present and not reserved, which
    // makes the subsequent plain write to the SAME address safe (it cannot fault if the read did not).
    if probe_present(scratch).is_none() {
        return false;
    }

    // Write the benign test pattern. The scratch read above was clean, so this store to the same
    // address cannot fault; a plain volatile write is sufficient.
    // SAFETY: `scratch` is a side-effect-free R/W register (TIMERx_PSC / ADC_WDLT) of a peripheral
    // whose scratch register just read cleanly. The write only loads a prescaler / watchdog-threshold
    // shadow; it starts nothing and enables no output. We restore the reset value below.
    unsafe {
        core::ptr::write_volatile(scratch as *mut u32, SCRATCH_PATTERN);
    }

    let readback = probe_present(scratch).unwrap_or(0);
    let retained = (readback & SCRATCH_RETAIN_MASK) == (SCRATCH_PATTERN & SCRATCH_RETAIN_MASK);

    // RESTORE the scratch register to its reset value, leaving the peripheral as we found it.
    // SAFETY: as above; restoring the documented reset value (0) of a side-effect-free register.
    unsafe {
        core::ptr::write_volatile(scratch as *mut u32, SCRATCH_RESET);
    }

    retained
}

// --- the BusFault handler body (DF-T3) --------------------------------------------------------

/// The ARMv7-M exception-frame layout: the eight words the core stacks on exception entry
/// (`r0, r1, r2, r3, r12, lr, pc, xpsr`). We need only the stacked `pc` (index 6) to fix it up. The
/// HAL-internal naked entry [`bus_fault_entry`] recovers this frame pointer from the active stack and
/// passes it down; the HAL does not depend on cortex-m-rt's `ExceptionFrame`.
const STACKED_PC_INDEX: usize = 6;

/// The HAL-internal naked BusFault entry the probe-scoped vector table points at.
///
/// The hardware calls this DIRECTLY from the relocated table on a BusFault: on entry the core has
/// already stacked the 8-word exception frame and set `LR = EXC_RETURN`. This is a NAKED function
/// (`#[unsafe(naked)]` + `naked_asm!`) so the compiler emits NO prologue, a normal `fn` could move
/// `SP` (push a frame) before we read it, which would invalidate the stack-pointer-based frame
/// recovery. The body mirrors the previously firmware-defined `#[exception] BusFault` shim (known-good
/// on silicon under the OLD mechanism), relocated into the HAL and adapted to a naked direct vector
/// target:
///
/// 1. Capture `LR` (the EXC_RETURN), `MSP`, and `PSP` up front.
/// 2. Select the frame pointer from EXC_RETURN bit 2 (0 => the frame is on MSP, 1 => on PSP). At boot
///    the probe runs on MSP.
/// 3. Call [`bus_fault_trampoline`] (a normal Rust fn) with the frame pointer; it invokes
///    `on_bus_fault` for the PC fix-up and returns whether the fault was an armed probe access.
/// 4. If handled, return from the exception with `BX LR` (the EXC_RETURN), resuming AFTER the skipped
///    probe load. If NOT handled (a real fault outside the probe window), spin: we are mid-detection
///    with no production handlers installed, so there is nothing safe to resume to.
///
/// # Safety
/// Installed only as the BusFault vector of the probe-scoped table (its address carries the Thumb bit,
/// as required for an exception vector). It must be entered by the hardware on a BusFault, not called
/// as an ordinary function.
#[cfg(target_arch = "arm")]
#[unsafe(naked)]
pub unsafe extern "C" fn bus_fault_entry() {
    core::arch::naked_asm!(
        // r0 = EXC_RETURN (LR on handler entry). Bit 2 selects MSP (0) vs PSP (1) for the frame.
        "mov  r0, lr",
        "tst  r0, #4",
        "mrs  r1, msp",
        "mrs  r2, psp",
        // r0 = selected frame pointer: MSP if bit 2 clear, else PSP.
        "ite  eq",
        "moveq r0, r1",
        "movne r0, r2",
        // Preserve EXC_RETURN across the call (callee may clobber LR); r4 is callee-saved.
        "mov  r4, lr",
        "bl   {trampoline}",
        // r0 = handled?. If zero (a real fault, not an armed probe access), spin: nothing safe to
        // resume to mid-detection.
        "cbnz r0, 1f",
        "2:",
        "b    2b",
        // Handled: return from the exception with the preserved EXC_RETURN.
        "1:",
        "bx   r4",
        trampoline = sym bus_fault_trampoline,
    )
}

/// Host stub for `bus_fault_entry` (mock / non-`arm` builds). The probe never runs on the host (no
/// real bus fault), so this is never installed or entered there; it exists only so the address-taking
/// in `build_probe_vector_table` and the public API resolve when the crate is built for the host.
///
/// # Safety
/// Never called on the host (it would `unreachable!`); present only so the public symbol and the
/// address-taking in `build_probe_vector_table` resolve in a host build.
#[cfg(not(target_arch = "arm"))]
pub unsafe extern "C" fn bus_fault_entry() {
    // The host never enters this; the family probe is hardware-only.
    unreachable!("bus_fault_entry is a hardware-only BusFault vector; never called on the host")
}

/// The normal-ABI Rust bridge the naked [`bus_fault_entry`] calls with the recovered frame pointer.
/// Kept as a separate non-naked fn so [`on_bus_fault`] (which touches statics and the SCB) runs with a
/// normal prologue; the naked entry only does the register capture + frame selection + exception
/// return that must not be perturbed by a compiler-inserted prologue.
///
/// Returns `1` if [`on_bus_fault`] fixed up an armed probe access, `0` otherwise (the entry then
/// spins).
///
/// # Safety
/// `frame` must be the stacked exception-frame pointer the naked entry recovered from the active stack.
#[cfg(target_arch = "arm")]
unsafe extern "C" fn bus_fault_trampoline(frame: *mut u32) -> u32 {
    // SAFETY: `frame` is the 8-word stacked exception frame the BusFault entry recovered.
    if on_bus_fault(frame) {
        1
    } else {
        0
    }
}

/// The BusFault handler body the HAL's probe-scoped vector entry ([`bus_fault_entry`]) delegates to.
///
/// `frame` is the stacked exception frame pointer (8 `u32` words: r0..r3, r12, lr, pc, xpsr). On a
/// PROBE fault (the access we armed) this advances the stacked PC by [`FAULT_SKIP_WIDTH`] so the
/// faulting `LDR` is skipped on return, clears the BusFault status, records `FAULTED`, and returns
/// (resuming after the probe access). On a NON-probe fault (`EXPECTING_FAULT` is `false`) it does
/// NOT fix up; it returns `false` so the entry can spin (a real bus fault outside the probe is a
/// genuine error, and detection has no production handler to escalate to).
///
/// Returns `true` if it handled (fixed up) a probe fault, `false` if the fault was not an armed probe
/// access.
///
/// # Safety
/// `frame` must be the valid stacked exception frame pointer the BusFault entry produced. The PC
/// fix-up writes the stacked PC word; an incorrect [`FAULT_SKIP_WIDTH`] resumes at the wrong address.
/// This is the DF-5 risk and is validated only on hardware (DF-T6).
// Used by `bus_fault_trampoline` (the `arm` naked entry's bridge); unreferenced in a host build.
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
pub(crate) unsafe fn on_bus_fault(frame: *mut u32) -> bool {
    if !EXPECTING_FAULT.load(Ordering::SeqCst) {
        // Not an armed probe access: a real bus fault. Do not fix up.
        return false;
    }

    // Optionally confirm the faulting address matches the candidate (BFARVALID in BFSR/CFSR). This
    // is corroboration; the EXPECTING_FAULT window is the primary gate. If BFAR is valid and does
    // not match PROBED_ADDR we still fix up (we are inside the armed window), but we clear the
    // status either way so the fault does not re-assert.
    let scb = &*SCB::PTR;
    // CFSR holds MMFSR(0..7) | BFSR(8..15) | UFSR(16..31). Clear by writing 1s back (W1C).
    let cfsr = scb.cfsr.read();
    // Clear the BusFault status bits (write-1-to-clear over the whole CFSR is the cortex-m idiom).
    scb.cfsr.write(cfsr);

    // Advance the stacked PC past the fixed-width probe load so we resume AFTER it. The stacked PC is
    // word 6 of the exception frame.
    let pc = core::ptr::read_volatile(frame.add(STACKED_PC_INDEX));
    core::ptr::write_volatile(
        frame.add(STACKED_PC_INDEX),
        pc.wrapping_add(FAULT_SKIP_WIDTH),
    );

    FAULTED.store(true, Ordering::SeqCst);
    true
}

// --- host tests for the relocated-table build/alignment logic ---------------------------------
//
// The bus-fault PROBE and the VTOR swap themselves are NOT host-testable (no host raises the fault,
// and `read_vtor`/`write_vtor`/`bus_fault_entry` are `arm`-only). These tests cover the deterministic
// table-shape logic that IS host-evaluable: the VTOR alignment round-up, the BusFault vector index /
// offset, and that the RAM table is large enough and aligned for VTOR.
#[cfg(all(test, feature = "mock"))]
mod table_tests {
    use super::*;

    #[test]
    fn busfault_vector_index_matches_offset_0x14() {
        // BusFault is exception number 5; its vector offset is 0x14 = index 5 * 4 bytes.
        assert_eq!(BUSFAULT_VECTOR_INDEX, 5);
        assert_eq!(
            BUSFAULT_VECTOR_INDEX * core::mem::size_of::<u32>(),
            0x14,
            "the BusFault slot is at byte offset 0x14 in the vector table"
        );
    }

    #[test]
    fn vector_table_is_large_enough_to_cover_the_busfault_slot() {
        // The table must hold at least the system exceptions; the BusFault slot must be in range.
        assert!(VECTOR_TABLE_LEN > BUSFAULT_VECTOR_INDEX);
        // 256 words = 1 KiB; covers the 16 system vectors plus a generous IRQ margin.
        assert_eq!(VECTOR_TABLE_BYTES, (VECTOR_TABLE_LEN * 4) as u32);
        assert!(
            VECTOR_TABLE_BYTES >= 128,
            "ARMv7-M VTOR needs >= 128-byte tables"
        );
    }

    #[test]
    fn vector_table_base_rounds_up_to_table_size() {
        let align = VECTOR_TABLE_BYTES;
        // An already-aligned address is unchanged.
        assert_eq!(vector_table_base(align), align);
        assert_eq!(vector_table_base(2 * align), 2 * align);
        assert_eq!(vector_table_base(0), 0);
        // A misaligned address rounds UP to the next multiple of the table size.
        assert_eq!(vector_table_base(1), align);
        assert_eq!(vector_table_base(align - 1), align);
        assert_eq!(vector_table_base(align + 1), 2 * align);
    }

    #[test]
    fn vector_table_base_is_a_valid_vtor_value() {
        // VTOR requires the base aligned to a power of two >= the table byte size. The round-up
        // result is always a multiple of VECTOR_TABLE_BYTES, hence aligned to it.
        for addr in [0u32, 1, 7, 0x2000_0001, 0x2000_03FF, 0x2000_0400] {
            let base = vector_table_base(addr);
            assert_eq!(
                base % VECTOR_TABLE_BYTES,
                0,
                "the programmed VTOR base must be a multiple of the table size"
            );
            assert!(
                base >= addr,
                "the round-up never moves the base below the table"
            );
        }
    }

    #[test]
    fn fault_skip_width_matches_a_32bit_thumb_ldr() {
        // The PC fix-up adds the width of the pinned 32-bit Thumb-2 LDR (4 bytes).
        assert_eq!(FAULT_SKIP_WIDTH, 4);
        // The stacked PC is word 6 of the 8-word exception frame.
        assert_eq!(STACKED_PC_INDEX, 6);
    }
}
