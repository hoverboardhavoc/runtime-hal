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
//! - **The PC fixup (the risk).** On a faulted access the handler fixes up the stacked return PC so
//!   execution resumes correctly instead of re-faulting forever. The fixup is RANGE-GATED against
//!   `probe_read32`'s code extent (DECISIONS.md #15), because pinned on silicon the stacked PC is NOT
//!   reliably the faulting load and VARIES by fault region:
//!   - **In-function** (F103: stacked PC = `ldr.w + 4`, still inside `probe_read32`): the load began,
//!     so ADVANCE the stacked PC by the decoded Thumb width of the instruction AT the stacked PC
//!     (`thumb_instr_bytes`: 4 for 32-bit Thumb-2, 2 for 16-bit), a RELATIVE one-instruction step, NOT
//!     a hardcoded constant and NOT an absolute resume address. A hardcoded skip is silicon-fragile: it
//!     desynced when the probe read lowered to a 16-bit `LDR` while the handler still skipped 4,
//!     resuming one halfword past the load into adjacent code (DECISIONS.md #15).
//!   - **Out of function** (F130 late external-region fault: stacked PC = the caller's return address,
//!     `probe_read32` already unwound): the consumer instruction there has NOT executed, so
//!     re-execution is EXACT -- leave the stacked PC UNCHANGED. The extent comes from the linker
//!     encapsulation symbols bracketing `probe_read32`'s own section (`probe_read32_extent`), so the
//!     gate is correct by construction, not by codegen luck.
//!
//! A relative advance stays SP-consistent (the hardware snapshots PC+SP together); an absolute resume
//! would fault against the wrong SP. The probe still emits the load as a single 32-bit `LDR.W`
//! (`probe_read32`, `#[inline(never)]`, its own `#[link_section]`) for a clean single access, but the
//! handler no longer DEPENDS on that width. The access is placed outside any IT block (a plain call)
//! so the xPSR IT-state complication does not arise. **The fixup's resume-on-silicon is validated
//! ONLY on hardware**; no host/emulator raises the fault it fixes up (spec section 8.2), though the
//! decode + advance + range-gate arithmetic is now host-tested.
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

use cortex_m::peripheral::SCB;

use super::{Family, FLASH_DENSITY_ADDR};

// --- the probe-scoped relocated vector table (HAL-owned BusFault) -----------------------------
//
// The HAL owns the BusFault handling: during detection it installs its OWN vector table in RAM whose
// BusFault slot points at the HAL-internal naked entry below, runs the probe, then restores VTOR. The
// application therefore defines NO `#[exception] BusFault`. See the module docs for the full rationale.

/// The number of `u32` entries the probe-scoped RAM vector table holds: the initial-SP word + the 15
/// ARMv7-M system-exception vectors (indices 1..=15, up to SysTick). The probe runs in an IRQ-LESS
/// window (no NVIC line is enabled, and the probe enables none), so ONLY the system exceptions can
/// fire; external IRQ vectors (index 16+) are unreachable and are NOT copied. 16 entries is exactly
/// the reachable set. The only slot the HAL overrides is BusFault (index 5); every other system slot
/// is copied from the active table so an unrelated system fault still reaches the application's
/// handler. Shrinking this from the former 256-word (1 KiB) table reclaims ~900 B of RAM with no
/// overlay hazard (the dropped words were never-reachable IRQ vectors), lowering the firmware stack
/// floor by that much (round-11 stack slice).
const VECTOR_TABLE_LEN: usize = 16;

/// The exception number (and table index) of BusFault on ARMv7-M: vector offset `0x14` => word index
/// 5. This is the only slot the HAL overrides; every other entry is copied from the active table so
/// existing handlers (Reset, NMI, HardFault, ...) keep working.
const BUSFAULT_VECTOR_INDEX: usize = 5;

/// The ARMv7-M `VTOR` alignment requirement for this table: the base must be aligned to a power of two
/// that is BOTH >= the table's byte size AND >= 128 bytes (the architectural floor: `VTOR[6:0]` are
/// reserved / RAZ). The table is 16 words = 64 bytes, so the 128-byte floor dominates and 128 is the
/// alignment. (128 >= 64, is a power of two, and meets the floor.)
const VTOR_ALIGN: u32 = 128;

/// A HAL-owned RAM vector table for the probe window.
///
/// `#[repr(align(128))]` satisfies [`VTOR_ALIGN`]: the table is 16 words = 64 bytes and the ARMv7-M
/// `VTOR` floor is 128 bytes, so aligning the static to 128 makes its own address a valid `VTOR` base
/// directly; [`vector_table_base`] rounds up to the same boundary as a defensive no-op (so a misaligned
/// base could never be programmed, and the indexed slots always land inside this static).
#[repr(align(128))]
struct AlignedVectorTable {
    entries: [u32; VECTOR_TABLE_LEN],
}

/// The single probe-scoped RAM vector table instance. Written only by [`install_probe_vector_table`]
/// (single-threaded bring-up context, before any IRQs are enabled) and read by the hardware as the
/// vector table while `VTOR` points at it.
static mut PROBE_VECTOR_TABLE: AlignedVectorTable = AlignedVectorTable {
    entries: [0; VECTOR_TABLE_LEN],
};

/// Compute the `VTOR` base to program for the RAM table: the table's own address rounded UP to
/// [`VTOR_ALIGN`] (128 bytes), the ARMv7-M requirement (base aligned to a power of two that is >= the
/// table byte size AND >= the 128-byte floor). Because `AlignedVectorTable` is `#[repr(align(128))]`,
/// the static's address is ALREADY a multiple of `VTOR_ALIGN`, so this round-up is a no-op for the
/// real base; it is kept as a defensive guarantee that the programmed `VTOR` is always aligned (the
/// indexed slots then always land inside the static).
#[inline]
fn vector_table_base(addr: u32) -> u32 {
    let align = VTOR_ALIGN;
    addr.wrapping_add(align - 1) & !(align - 1)
}

/// Build the probe-scoped table by copying the active table then overriding the BusFault slot.
///
/// `active_vtor` is the current `SCB.VTOR` base (the flash table cortex-m-rt linked, or whatever table
/// is active). We copy `VECTOR_TABLE_LEN` (= 16) words from it, the system-exception slots only
/// (HardFault, NMI, ...; the probe window runs with no NVIC line enabled, so IRQ slots are
/// unreachable and not carried), then write [`bus_fault_entry`]'s address into the BusFault slot. A
/// Rust fn pointer on thumbv7m already has the Thumb bit (bit 0) set, which the hardware requires for
/// an exception vector, so `bus_fault_entry as usize as u32` is the correct value to store.
///
/// # Safety
/// `active_vtor` must be a readable vector-table base (it comes from `SCB.VTOR`). Single-threaded
/// bring-up context: this is the only writer of [`PROBE_VECTOR_TABLE`], and IRQs are not enabled.
unsafe fn build_probe_vector_table(active_vtor: u32) {
    let dst = core::ptr::addr_of_mut!(PROBE_VECTOR_TABLE.entries) as *mut u32;
    let src = active_vtor as *const u32;
    // Copy the active table's 16 system-exception slots (the only vectors reachable in the
    // IRQ-less probe window). The source is the real (flash) table base, so reading
    // VECTOR_TABLE_LEN words from it is in-bounds on these parts.
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

// --- the probe <-> handler shared state -------------------------------------------------------

/// Armed by [`run`] before each candidate access, disarmed after a clean read. The handler treats a
/// fault while this is `false` as a REAL fault (not a probe access) and does NOT fix it up.
static EXPECTING_FAULT: AtomicBool = AtomicBool::new(false);
/// The candidate base currently being read; the handler confirms `BFAR == PROBED_ADDR` when
/// `BFSR.BFARVALID` is set (a precise BusFault latches the faulting address in BFAR).
static PROBED_ADDR: AtomicU32 = AtomicU32::new(0);
/// Set by the handler so the probe learns the access faulted (the family-negative signal).
static FAULTED: AtomicBool = AtomicBool::new(false);

/// The fully-populated probe result: EVERY silicon observation gathered in one pass, ready to hand to
/// [`crate::detect::synthesize`] in one shot. The family discriminator, the flash-density read, and
/// the MEASURED per-instance advanced-timer / ADC counts. There is no half-built / default value:
/// `run` does not return until all of these are known (DECISIONS.md #11).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Detected {
    /// The family the GPIO+RCU probe resolved, the detection-internal discriminator that drives
    /// `synthesize`. Reachable outside the crate only behind the `detect-internals` feature, for the
    /// in-tree detection acceptance firmware; application code derives family-shaped facts from the
    /// [`crate::Chip`] instead.
    pub family: Family,
    /// `FLASH_DENSITY[15:0]` (KiB of flash), read from `0x1FFF_F7E0`. Corroboration + the F10x
    /// `flash_page` input (spec section 4.3 / 5.2). Read after the family decision; advisory for
    /// F1x0 (constant K1).
    pub flash_kib: u16,
    /// The MEASURED number of advanced timers present (TIMER0 + TIMER7, by the benign scratch
    /// write-back), measured by [`measure_counts`] after the family decision. Never a family default
    /// (the bench proved a family constant wrong in both directions).
    pub adv_timers: u8,
    /// The MEASURED number of ADC instances present (ADC0 + ADC1 + ADC2, by the scratch write-back),
    /// measured by [`measure_counts`] after the family decision.
    pub adc_count: u8,
}

// --- the single-access probe read -------------------------------------------------------------

/// Read a 32-bit control word at `addr` as a SINGLE volatile load.
///
/// `#[inline(never)]` so the access is a standalone `LDR`, not inlined into a context where the
/// compiler might fuse or re-widen it. If the access faults, the handler advances the stacked PC past
/// this `LDR` and returns; the returned value is then meaningless (the caller checks `FAULTED` first).
///
/// The load is emitted as an explicit `ldr.w` (the `.w` qualifier FORCES the 32-bit Thumb-2 encoding)
/// so the fault is a single clean 4-byte access rather than whatever `read_volatile::<u32>` happens to
/// lower to (a 16-bit `LDR` is legal for low registers + small offsets, and the compiler DID choose it
/// at commit 3309e39). The handler no longer DEPENDS on this width, though: it decodes the faulting
/// instruction's actual width from the stacked PC ([`resume_pc_after_probe_load`]), so a 16- or 32-bit
/// lowering both resume correctly (DECISIONS.md #15). The asm block keeps volatile semantics (no
/// `pure`), reads only, no stack.
///
/// # Safety
/// `addr` is a candidate peripheral base; the read is wrapped by the armed BusFault handler so a
/// fault on the wrong-family (reserved) base is caught instead of escalating.
#[cfg(target_arch = "arm")]
#[inline(never)]
#[link_section = "probe_read32"]
fn probe_read32(addr: u32) -> u32 {
    let value: u32;
    // SAFETY: the access is bounded by the armed fault harness (EXPECTING_FAULT + the BusFault
    // handler). A fault here is caught and turned into the family-negative signal.
    unsafe {
        core::arch::asm!(
            "ldr.w {value}, [{addr}]",
            addr = in(reg) addr,
            value = out(reg) value,
            options(nostack, readonly),
        );
    }
    value
}

// `probe_read32` is placed in its OWN linker section (a valid C-identifier name), so the linker emits
// the encapsulation boundary symbols `__start_probe_read32` / `__stop_probe_read32` bracketing its
// code (both GNU ld and rust-lld generate these for such sections; the `linkme`/kernel pattern). The
// range `[__start_probe_read32, __stop_probe_read32)` is `probe_read32`'s exact code extent, which
// `on_bus_fault` range-gates the stacked-PC advance against (DECISIONS.md #15): the F103 in-function
// fault stacks a PC INSIDE this range (advance one instruction), the F130 caller-frame late fault
// stacks a PC OUTSIDE it, in `probe_present`/`probe_candidate` (leave unchanged, re-execute the
// not-yet-run consumer). Owned entirely by the HAL (the attribute + the symbol decls), no firmware
// linker-script edit.
#[cfg(target_arch = "arm")]
extern "C" {
    static __start_probe_read32: u8;
    static __stop_probe_read32: u8;
}

/// The `[lo, hi)` flash code extent of [`probe_read32`], from the linker encapsulation symbols.
#[cfg(target_arch = "arm")]
#[inline]
fn probe_read32_extent() -> (u32, u32) {
    // Linker-defined boundary symbols; only their ADDRESSES are taken (never dereferenced), and they
    // bracket `probe_read32`'s section. `addr_of!` of a static needs no `unsafe`.
    (
        core::ptr::addr_of!(__start_probe_read32) as u32,
        core::ptr::addr_of!(__stop_probe_read32) as u32,
    )
}

/// Host stub for [`probe_read32_extent`]: no linker section / boundary symbols exist on the host, and
/// the probe never runs there, so this is dead (the range-gate DECISION is host-tested directly via
/// [`resume_pc_range_gated`] with explicit bounds). Returns an empty range.
#[cfg(not(target_arch = "arm"))]
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
fn probe_read32_extent() -> (u32, u32) {
    (0, 0)
}

/// Host stub for [`probe_read32`] (mock / non-`arm` builds): the probe never runs on the host, and
/// the `ldr.w` asm does not assemble there; a plain volatile read keeps the crate compiling.
#[cfg(not(target_arch = "arm"))]
#[inline(never)]
fn probe_read32(addr: u32) -> u32 {
    // SAFETY: never executed on the host (the probe paths are silicon-only); see the arm variant.
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

/// `SHCSR.BUSFAULTENA` (ARMv7-M B3.2.13, System Handler Control and State Register @`0xE000_ED24`,
/// bit 17): the dedicated-BusFault-handler enable the probe arms around each candidate read.
const SHCSR_BUSFAULTENA: u32 = 1 << 17;

/// RMW `SHCSR.BUSFAULTENA` through the raw SCB register view, returning the PRIOR state.
///
/// Deliberately `&*SCB::PTR`, never `cortex_m::Peripherals::steal()` (DECISIONS.md #13): the HAL
/// must not consume cortex-m's one-shot `TAKEN` flag, so the application's `Peripherals::take()`
/// works regardless of whether it runs before or after `detect_chip`. This is the same raw access
/// shape the BusFault handler itself already uses ([`bus_fault_entry`]); single-core bring-up
/// context, no concurrent SHCSR user.
fn shcsr_set_busfaultena(enable: bool) -> bool {
    // SAFETY: SCB::PTR is the architectural SCB block; RMW of one bit in single-core bring-up.
    unsafe {
        let scb = &*SCB::PTR;
        let prev = scb.shcsr.read();
        if enable {
            scb.shcsr.write(prev | SHCSR_BUSFAULTENA);
        } else {
            scb.shcsr.write(prev & !SHCSR_BUSFAULTENA);
        }
        prev & SHCSR_BUSFAULTENA != 0
    }
}

/// Run the ordered GPIO+RCU family probe ONCE inside the fault-safe harness and, once a family is
/// known, MEASURE the per-instance counts, returning the fully-populated [`Detected`] (or `None` if
/// neither family matched => fail safe).
///
/// Sequence (spec section 4.2):
/// 1. **F1x0 probe.** Set `RCU_AHBEN.PAEN` (bit 17). Read GPIOA control at `0x4800_0000`. A clean
///    read => F1x0. A bus-fault => not F1x0; proceed to step 2.
/// 2. **F10x probe** (only if step 1 faulted). Set `RCU_APB2EN.PAEN` (bit 2). Read GPIOA control at
///    `0x4001_0800`. A clean read => F10x. A bus-fault => NEITHER family; fail safe (`None`).
/// 3. Read the flash-density register for the F10x page-size input (corroboration; the GPIO result
///    is authoritative), and MEASURE the advanced-timer / ADC instance counts ([`measure_counts`]).
///
/// Gathering the counts here (rather than in a separate caller step) means `run` returns a
/// fully-populated `Detected` with no half-built intermediate: every silicon observation is in hand
/// before [`crate::detect::synthesize`] runs (DECISIONS.md #11). `measure_counts` takes no family
/// argument and runs after the family discriminator, so it fits cleanly into the positive paths.
///
/// `run` installs the HAL's probe-scoped vector table (BusFault slot -> [`bus_fault_entry`]) for the
/// duration of the family probe, sets `SHCSR.BUSFAULTENA` on entry, and restores both on every exit,
/// so a precise data-bus error traps to the HAL-internal BusFault handler rather than escalating to
/// HardFault, and the application defines no fault handler. It does NOT retry or loop.
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
pub fn run() -> Option<Detected> {
    // The family-probe vector-table swap is strictly probe-scoped: install -> probe -> restore, all
    // inside this call. The HAL's BusFault entry handles a faulted candidate read; every other vector
    // is the application's (copied from the active table).
    let family = with_probe_vector_table(|| {
        // Enable the dedicated BusFault handler so a precise reserved-read fault traps to it (not
        // HardFault). Remember the prior state so we can restore it. Raw SHCSR RMW, never steal()
        // (DECISIONS.md #13: detect must not consume the one-shot TAKEN flag).
        let bf_was_enabled = shcsr_set_busfaultena(true);

        let family = probe_family();

        // Restore the prior BUSFAULTENA state. The probe handler is strictly boot-temporary.
        if !bf_was_enabled {
            // Undo our enable so we leave SHCSR as we found it.
            shcsr_set_busfaultena(false);
        }
        // The probe leaves the shared atomics disarmed.
        EXPECTING_FAULT.store(false, Ordering::SeqCst);

        family
    })?;

    // A family matched. Gather the REMAINING silicon observations so the returned `Detected` is fully
    // populated in one literal: the flash density (always-mapped, fault-free) and the MEASURED
    // per-instance counts (`measure_counts` installs its own probe-scoped harness for the sweep). No
    // half-built value escapes: every field below is its real value.
    let counts = measure_counts();
    Some(Detected {
        family,
        flash_kib: read_flash_density(),
        adv_timers: counts.adv_timers,
        adc_count: counts.adc_count,
    })
}

/// The ordered family-discriminator candidate set, run with BUSFAULTENA already set. Returns the
/// matched [`Family`] or `None` if neither candidate read cleanly (fail safe). The remaining
/// observations (flash density, measured counts) are gathered by the caller [`run`] once a family is
/// known, so this returns only the family decision.
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
fn probe_family() -> Option<Family> {
    // Step 1: F1x0. Enable GPIOA's clock in the F1x0-correct RCU register, then read GPIOA control.
    rcu_set_bit(RCU_AHBEN, F1X0_PAEN_BIT);
    if probe_candidate(F1X0_GPIOA_BASE).is_some() {
        // Clean read at the F1x0 base => F1x0 family. (The known-good F130 readback is 0x682a73a3;
        // it MAY be used as an extra plausibility gate but is not load-bearing, the wrong base
        // faults rather than returning garbage, so "did not fault" is already the strong signal.)
        return Some(Family::F1x0);
    }

    // Step 2: F10x (only reached if step 1 faulted). Enable GPIOA in the F10x-correct RCU register,
    // then read GPIOA control at the F10x base.
    rcu_set_bit(RCU_APB2EN, F10X_PAEN_BIT);
    if probe_candidate(F10X_GPIOA_BASE).is_some() {
        return Some(Family::F10x);
    }

    // Both candidates faulted: NEITHER family matched. Fail safe (do not guess).
    None
}

/// Read one candidate GPIOA control register inside the armed fault window. Returns `Some(value)` on
/// a clean read, `None` if the access bus-faulted (the family-negative signal).
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
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

// --- the peripheral-presence measurement (folded into `run`) ----------------------------------
//
// These GENERALIZE the family probe's machinery for the peripheral-presence MEASUREMENT: instead of
// resolving F1x0-vs-F10x, MEASURE which advanced timers / ADCs a given instance actually has, per
// chip, rather than inferring counts from a family constant. They reuse the SAME shared atomics
// (EXPECTING_FAULT / PROBED_ADDR / FAULTED) and the SAME `probe_read32` + width-decoding `on_bus_fault`
// PC-fixup as the family probe; no new private duplicate state. The whole sweep runs under ONE
// BusFault enable (rather than per-candidate like the family probe), so the SCB enable/disable is
// split out into [`arm_busfault`] / [`disarm_busfault`] and the per-access armed read is exposed as
// [`probe_present`]. `run` calls [`measure_counts`] once a family is known so its `Detected` carries
// the measured counts; `bench-fw-probe/` is the standalone validator that reports the raw sub-signals
// (it calls [`measure_counts`] / the lower-level helpers directly to break out each sub-signal).

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
    // Raw SHCSR RMW (never steal(), DECISIONS.md #13); the caller restores via disarm_busfault.
    shcsr_set_busfaultena(true)
}

/// Restore `SHCSR.BUSFAULTENA` to the state [`arm_busfault`] reported (`prev`). If BusFault was not
/// enabled before the sweep, disable it again; otherwise leave it on (the caller had it on for its own
/// reasons). Also disarms the probe window so a later real fault is never mistaken for a probe access.
///
/// # Safety
/// Bring-up / single-core context only; mutates `SHCSR`. Pass the exact value [`arm_busfault`]
/// returned.
pub fn disarm_busfault(prev: bool) {
    // Raw SHCSR RMW (never steal(), DECISIONS.md #13): restore the prior state.
    if !prev {
        shcsr_set_busfaultena(false);
    }
    // Leave the shared probe window disarmed (a real fault after the sweep is a genuine error).
    EXPECTING_FAULT.store(false, Ordering::SeqCst);
}

/// Fault-safe read of an ARBITRARY 32-bit address inside the armed BusFault window. Generalizes
/// `probe_candidate` (which is GPIOA-specific) for the peripheral-presence sweep: arm the shared
/// window, do the single-access `probe_read32`, disarm, and report `None` if the access faulted (the
/// address is absent / reserved) or `Some(value)` on a clean read.
///
/// The caller MUST have already enabled `SHCSR.BUSFAULTENA` (via [`arm_busfault`]) and be running
/// inside [`with_probe_vector_table`]; this function does NOT touch the SCB enable or the vector table,
/// so a whole candidate sweep can run under one enable/restore. It reuses the SAME `EXPECTING_FAULT` /
/// `PROBED_ADDR` / `FAULTED` atomics and the SAME width-decoding `on_bus_fault` PC-fixup as `run`.
///
/// # Safety
/// `addr` is a candidate peripheral register address; the read is bounded by the armed fault harness
/// (the caller's [`arm_busfault`] + the HAL's probe-scoped [`bus_fault_entry`] -> `on_bus_fault`), so
/// a fault on an absent/reserved address is caught and reported as `None` instead of escalating.
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
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
/// `RCU_APB2EN` clock-enable bit for each entry of [`ADV_TIMER_BASES`]: TIMER0EN = bit 11, TIMER7EN =
/// bit 13. Both families' advanced timers are on APB2; bit 13 is reserved on F1x0, which has no TIM8
/// at `0x4001_3400` and so still scratch-tests absent there.
const ADV_TIMER_CLOCK_BITS: [u32; 2] = [11, 13];
/// The three ADC instance bases (ADC0, ADC1, ADC2) on both families (APB2 map).
const ADC_BASES: [u32; 3] = [0x4001_2400, 0x4001_2800, 0x4001_3C00];
/// `RCU_APB2EN` clock-enable bit for each entry of [`ADC_BASES`]: ADC0EN = bit 9, ADC1EN = bit 10,
/// ADC2EN = bit 15. F1x0 has only ADC0 (bit 9); the others are reserved there and scratch-test absent.
const ADC_CLOCK_BITS: [u32; 3] = [9, 10, 15];

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
/// Each candidate's peripheral clock is enabled BEFORE its scratch test: an unclocked GD32 timer / ADC
/// ignores the write-back and reads zero, so without the clock it would be miscounted as absent (this
/// under-count is why TIMER7 went undetected on the high-density F10x part, and why a dual-ADC F10x
/// master mis-reported a single ADC). `RCU_APB2EN` is snapshotted on entry and restored on exit, so
/// the clock enables are not a lasting side effect (the application re-enables what it uses at
/// bring-up), the same leave-as-found contract `scratch_present` follows for the scratch value.
///
/// NOT host-testable (the same reason as [`run`]: the write-back to an absent slot relies on real
/// silicon behavior no host/emulator reproduces); validated on the bench.
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
pub fn measure_counts() -> MeasuredCounts {
    // Probe-scoped vector-table swap around the whole sweep (install -> sweep -> restore).
    with_probe_vector_table(|| {
        let prev = arm_busfault();

        // Snapshot APB2EN so the per-candidate clock enables below can be reverted exactly (the
        // scratch test needs each peripheral's clock ON to retain its write-back).
        let apb2en = (RCU_BASE + RCU_APB2EN) as *mut u32;
        // SAFETY: the shared RCU base is always present; this reads a control register.
        let apb2en_saved = unsafe { core::ptr::read_volatile(apb2en) };

        let mut adv_timers = 0u8;
        let mut i = 0;
        while i < ADV_TIMER_BASES.len() {
            rcu_set_bit(RCU_APB2EN, ADV_TIMER_CLOCK_BITS[i]);
            if scratch_present(ADV_TIMER_BASES[i]) {
                adv_timers += 1;
            }
            i += 1;
        }

        let mut adc_count = 0u8;
        let mut j = 0;
        while j < ADC_BASES.len() {
            rcu_set_bit(RCU_APB2EN, ADC_CLOCK_BITS[j]);
            if scratch_present(ADC_BASES[j]) {
                adc_count += 1;
            }
            j += 1;
        }

        // Restore APB2EN to its pre-probe value: the clock enables were a means to test, not a result.
        // SAFETY: as above; writing back the snapshot of a shared control register.
        unsafe {
            core::ptr::write_volatile(apb2en, apb2en_saved);
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
#[inline(never)] // O3 fault-path isolation: keep the armed-BusFault window out of the caller (main) so opt-level 3 cannot interleave its register state with the fault-skip; see round-13 F103 regression.
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

/// The byte-width of the Thumb instruction whose first halfword is `first_halfword`: 4 for a 32-bit
/// Thumb-2 encoding, 2 for a 16-bit one.
///
/// Per ARMv7-M ARM A5.1, a halfword is the FIRST halfword of a 32-bit instruction iff its top five
/// bits (`[15:11]`) are `0b11101`, `0b11110`, or `0b11111`, i.e. `(first_halfword & 0xF800) >=
/// 0xE800`; every other top-five-bit pattern is a complete 16-bit instruction. This is what lets the
/// BusFault handler resume EXACTLY on the instruction after the faulting probe load without assuming
/// how that load lowered (a `probe_read32` that compiles to a 16-bit `LDR` and one that compiles to a
/// 32-bit `LDR.W` are both handled).
// Called by `on_bus_fault` (arm) and the host tests; on a non-arm non-test build its only caller is
// the dead-code-allowed `on_bus_fault`, so allow it there too.
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
#[inline]
fn thumb_instr_bytes(first_halfword: u16) -> u32 {
    if first_halfword & 0xF800 >= 0xE800 {
        4
    } else {
        2
    }
}

/// The resume PC for a faulting probe load: `stacked_pc` advanced past the faulting instruction by its
/// decoded Thumb width ([`thumb_instr_bytes`]). Pure (no memory access), so the handler's PC-advance
/// arithmetic is host-testable; the BusFault handler reads `first_halfword` from the instruction
/// stream and calls this.
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
#[inline]
fn resume_pc_after_probe_load(stacked_pc: u32, first_halfword: u16) -> u32 {
    stacked_pc.wrapping_add(thumb_instr_bytes(first_halfword))
}

/// The range-gated stacked-PC fixup decision (DECISIONS.md #15, the round-14 correct-by-construction
/// upgrade). `[lo, hi)` is [`probe_read32`]'s code extent:
///
/// - **In-function** (`lo <= stacked_pc < hi`): the F103 real fault stacks a PC INSIDE `probe_read32`
///   (the pinned `ldr.w + 4`), so the faulting load DID begin; advance past it by the decoded Thumb
///   width so the resume does not re-fault (the read result is discarded once `FAULTED` is set).
/// - **Out of function** (`stacked_pc` outside `[lo, hi)`): the F130 late external-region fault stacks
///   the CALLER's return address (`probe_read32` already unwound). Re-execution is EXACT there: the
///   consumer instruction at the stacked PC has NOT executed, so leave the stacked PC UNCHANGED. The
///   old unconditional advance skipped that not-yet-run caller instruction (benign only because the
///   fault path guards the sole consumer); this makes it correct by construction.
///
/// Pure (no memory access), so the decision is host-testable; the handler reads `first_halfword` only
/// when in-range (so it never reads a caller instruction it will not step over).
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
#[inline]
fn resume_pc_range_gated(stacked_pc: u32, lo: u32, hi: u32, first_halfword: u16) -> u32 {
    if stacked_pc >= lo && stacked_pc < hi {
        resume_pc_after_probe_load(stacked_pc, first_halfword)
    } else {
        stacked_pc
    }
}

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
/// 1. Read `MSP`/`PSP` and select the frame pointer from EXC_RETURN (`LR`) bit 2 (0 => the frame is on
///    MSP, 1 => on PSP; at boot the probe runs on MSP) into r0, using only caller-saved r0-r2.
/// 2. Stash EXC_RETURN on the STACK (`push {r3, lr}`, r3 as 8-byte-alignment padding) across the call,
///    NOT in a callee-saved register: `on_bus_fault`'s caller-context r4-r11 must survive untouched, so
///    the entry never writes any callee-saved register (the round-13 F103 brick was `mov r4, lr`
///    leaking EXC_RETURN into the interrupted context's r4).
/// 3. Call [`bus_fault_trampoline`] (a normal Rust fn) with the frame pointer; it invokes
///    `on_bus_fault` for the PC fix-up and returns whether the fault was an armed probe access.
/// 4. Restore EXC_RETURN (`pop {r3, lr}`). If handled, return from the exception with `BX LR`, resuming
///    AFTER the skipped probe load with all callee-saved state intact. If NOT handled (a real fault
///    outside the probe window), spin: we are mid-detection with no production handlers installed, so
///    there is nothing safe to resume to.
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
        // Preserve EXC_RETURN across the call WITHOUT touching any callee-saved register: stash LR on
        // the stack. `push {{r3, lr}}` stores EXC_RETURN (LR) and pushes r3 purely as padding to keep
        // SP 8-byte aligned for the AAPCS call (r3 is caller-saved, so clobbering it is free). The
        // former `mov r4, lr` used r4 to hold EXC_RETURN across the `bl`, but r4 is callee-SAVED and is
        // NOT part of the hardware exception frame, so the interrupted context resumed with r4 =
        // EXC_RETURN (0xFFFFFFF9); code that relied on its r4 then dereferenced that value (the
        // round-13 F103 hard fault: BFAR 0xFFFFFFF9 in the VTOR-restore code). Keeping EXC_RETURN off
        // the integer registers entirely makes this entry insensitive to how the probe/caller allocate
        // r4-r11 (the #[inline(never)] on the probe fns is now hardening, not the fix).
        "push {{r3, lr}}",
        "bl   {trampoline}",
        "pop  {{r3, lr}}",
        // r0 = handled?. If zero (a real fault, not an armed probe access), spin: nothing safe to
        // resume to mid-detection.
        "cbnz r0, 1f",
        "2:",
        "b    2b",
        // Handled: return from the exception with the restored EXC_RETURN in LR (untouched callee-saved
        // state, so the interrupted context resumes exactly as stacked).
        "1:",
        "bx   lr",
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
/// PROBE fault (the access we armed) this fixes up the stacked PC, RANGE-GATED against `probe_read32`'s
/// code extent (see [`resume_pc_range_gated`] and DECISIONS.md #15): if the stacked PC is INSIDE
/// `probe_read32` (the F103 in-function case) it advances past the faulting load by that load's DECODED
/// Thumb width (2 or 4 bytes) so the `LDR` is skipped on return; if the stacked PC is OUTSIDE it (the
/// F130 caller-frame late fault, `probe_read32` already unwound) it leaves the stacked PC UNCHANGED so
/// the not-yet-run caller instruction re-executes exactly. Either way it clears the BusFault status,
/// records `FAULTED`, and returns. Only the stacked PC (word 6) is ever touched; the stacked xPSR
/// (word 7, carrying the Thumb bit) is left intact, so the exception return restores Thumb state
/// unchanged. On a NON-probe fault (`EXPECTING_FAULT` is `false`) it does NOT fix up; it returns
/// `false` so the entry can spin (a real bus fault outside the probe is a genuine error, and detection
/// has no production handler to escalate to).
///
/// Returns `true` if it handled (fixed up) a probe fault, `false` if the fault was not an armed probe
/// access.
///
/// # Safety
/// `frame` must be the valid stacked exception frame pointer the BusFault entry produced. The PC
/// fix-up is RANGE-GATED against `probe_read32`'s code extent (`probe_read32_extent`, from the linker
/// encapsulation symbols): only when the stacked PC lies INSIDE that extent does it read the
/// instruction halfword at the stacked PC and advance by the decoded width (a RELATIVE step,
/// SP-consistent); a stacked PC OUTSIDE it is left unchanged (re-execute). It does NOT assume the
/// stacked PC is the faulting instruction: pinned on silicon (DECISIONS.md #15) the stacked PC may be
/// load+width (F103, in-function) or the caller's return address (F130, a late external-region fault).
/// The resume-on-silicon is validated only on hardware (the F103 real fault on every F10x boot +
/// `bench-fw-faultpin` on the F1x0); the advance + range-gate arithmetic is host-tested.
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

    // Fix up the stacked PC (word 6), RANGE-GATED against probe_read32's code extent (DECISIONS.md
    // #15, the round-14 correct-by-construction upgrade). The stacked PC is NOT reliably the faulting
    // load, and its location VARIES by fault region (pinned on silicon):
    //   - IN-function (F103 @0x48000000, stacked PC = load+4, still inside probe_read32): the load DID
    //     begin, so ADVANCE past it by the decoded Thumb width of the instruction AT the stacked PC
    //     (RELATIVE, never an absolute address; a fixed +4 desynced when the load lowered to a 16-bit
    //     `LDR`, commit 3309e39 IACCVIOL/INVSTATE). SP-consistent because the hardware snapshots PC+SP
    //     together, and the stepped-over instruction is discarded (the read result is ignored once
    //     FAULTED is set).
    //   - OUT of function (F130 @0x60000000, late external-region fault: stacked PC = the CALLER's
    //     return address in probe_present/probe_candidate, probe_read32 already unwound): the consumer
    //     instruction there has NOT executed, so re-execution is EXACT -- leave the stacked PC UNCHANGED.
    //     The former unconditional advance skipped that not-yet-run caller instruction (benign only
    //     because the fault path guards its sole consumer); the range gate makes it correct.
    let pc = core::ptr::read_volatile(frame.add(STACKED_PC_INDEX));
    let (lo, hi) = probe_read32_extent();
    if pc >= lo && pc < hi {
        // In-function: read the faulting instruction's first halfword and advance by its decoded width.
        // SAFETY: `pc` is inside probe_read32's flash extent (readable, 2-byte aligned); masking bit 0
        // is defensive (a precise-fault stacked PC is already even).
        let first_halfword = core::ptr::read_volatile((pc & !1) as *const u16);
        core::ptr::write_volatile(
            frame.add(STACKED_PC_INDEX),
            resume_pc_range_gated(pc, lo, hi, first_halfword),
        );
    }
    // else: caller-frame late fault -- leave word 6 unchanged so the caller re-executes exactly.

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

    /// The table byte-size (16 words = 64 B), a test-only invariant helper: the VTOR base alignment
    /// ([`VTOR_ALIGN`], the 128-byte floor) exceeds it, so it is not used in the lib build.
    const VECTOR_TABLE_BYTES: u32 = (VECTOR_TABLE_LEN * core::mem::size_of::<u32>()) as u32;

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
    fn vector_table_covers_the_system_vectors_and_the_busfault_slot() {
        // The table holds exactly the reachable set in the IRQ-less probe window: the SP word + the 15
        // system exceptions (indices 0..=15). The BusFault slot (index 5) must be in range.
        assert_eq!(
            VECTOR_TABLE_LEN, 16,
            "SP word + 15 system exception vectors"
        );
        assert!(VECTOR_TABLE_LEN > BUSFAULT_VECTOR_INDEX);
        assert_eq!(VECTOR_TABLE_BYTES, (VECTOR_TABLE_LEN * 4) as u32);
        // VTOR's base alignment (not the table's own byte size) must meet the 128-byte floor and be a
        // power of two >= the table size. The table is 64 B; the 128-byte floor dominates.
        assert!(
            VTOR_ALIGN >= 128,
            "ARMv7-M VTOR base needs >= 128-byte alignment"
        );
        assert!(
            VTOR_ALIGN >= VECTOR_TABLE_BYTES,
            "alignment must be >= the table size"
        );
        assert_eq!(
            VTOR_ALIGN & (VTOR_ALIGN - 1),
            0,
            "VTOR alignment is a power of two"
        );
    }

    #[test]
    fn vector_table_base_rounds_up_to_vtor_align() {
        let align = VTOR_ALIGN;
        // An already-aligned address is unchanged.
        assert_eq!(vector_table_base(align), align);
        assert_eq!(vector_table_base(2 * align), 2 * align);
        assert_eq!(vector_table_base(0), 0);
        // A misaligned address rounds UP to the next multiple of the VTOR alignment.
        assert_eq!(vector_table_base(1), align);
        assert_eq!(vector_table_base(align - 1), align);
        assert_eq!(vector_table_base(align + 1), 2 * align);
    }

    #[test]
    fn vector_table_base_is_a_valid_vtor_value() {
        // VTOR requires the base aligned to a power of two >= the table byte size AND >= 128 (the
        // architectural floor). The round-up result is always a multiple of VTOR_ALIGN, hence aligned.
        for addr in [0u32, 1, 7, 0x2000_0001, 0x2000_03FF, 0x2000_0400] {
            let base = vector_table_base(addr);
            assert_eq!(
                base % VTOR_ALIGN,
                0,
                "the programmed VTOR base must be a multiple of the VTOR alignment"
            );
            assert!(
                base >= addr,
                "the round-up never moves the base below the table"
            );
        }
    }

    #[test]
    fn stacked_pc_is_word_six_of_the_frame() {
        // The stacked PC is word 6 of the 8-word exception frame (r0..r3, r12, lr, pc, xpsr); the
        // stacked xPSR (the Thumb bit) is word 7 and the fixup must never touch it.
        assert_eq!(STACKED_PC_INDEX, 6);
    }

    #[test]
    fn thumb_instr_bytes_decodes_16_vs_32_bit_encodings() {
        // 16-bit encodings: top five bits < 0b11101. The `6800 LDR r0,[r0]` the probe read lowered to
        // at commit 3309e39 is 16-bit; the `E7FE b.n` self-branch is 16-bit; a MOV/ADD low reg too.
        assert_eq!(
            thumb_instr_bytes(0x6800),
            2,
            "narrow LDR (the 3309e39 lowering)"
        );
        assert_eq!(thumb_instr_bytes(0xE7FE), 2, "b.n (top five bits 0b11100)");
        assert_eq!(thumb_instr_bytes(0x4608), 2, "mov r0, r1");
        assert_eq!(thumb_instr_bytes(0x0000), 2);
        assert_eq!(
            thumb_instr_bytes(0xE7FF),
            2,
            "0xE7FF is the last 16-bit halfword"
        );

        // 32-bit Thumb-2 encodings: top five bits are 0b11101 / 0b11110 / 0b11111, i.e.
        // (hw & 0xF800) >= 0xE800. The `F8D0 xxxx LDR.W` the probe read now emits is 32-bit.
        assert_eq!(
            thumb_instr_bytes(0xF8D0),
            4,
            "LDR.W (the pinned probe load)"
        );
        assert_eq!(
            thumb_instr_bytes(0xE800),
            4,
            "first 32-bit halfword (0b11101)"
        );
        assert_eq!(
            thumb_instr_bytes(0xF000),
            4,
            "0b11110 (BL / data-processing)"
        );
        assert_eq!(thumb_instr_bytes(0xF800), 4, "0b11111");
        assert_eq!(thumb_instr_bytes(0xFFFF), 4);

        // The exact 16/32 boundary: 0xE7FF is 16-bit, 0xE800 is the first 32-bit halfword.
        assert_eq!(thumb_instr_bytes(0xE7FF), 2);
        assert_eq!(thumb_instr_bytes(0xE800), 4);
    }

    #[test]
    fn resume_pc_advances_by_the_decoded_width_thumb_bit_preserved() {
        // A 32-bit LDR.W at an even PC resumes exactly 4 bytes on (the probe's current lowering); a
        // 16-bit LDR resumes 2 bytes on (the 3309e39 lowering that a fixed +4 skip mishandled). The
        // resume PC stays even in both cases (bit 0 clear), so the exception-return PC is a valid
        // instruction address and Thumb state (carried in the stacked xPSR, untouched here) is intact.
        for &pc in &[0x0800_07e2u32, 0x0800_0d02, 0x0800_0000, 0x2000_0100] {
            assert_eq!(resume_pc_after_probe_load(pc, 0xF8D0), pc + 4);
            assert_eq!(resume_pc_after_probe_load(pc, 0x6800), pc + 2);
            assert_eq!(resume_pc_after_probe_load(pc, 0xF8D0) & 1, 0);
            assert_eq!(resume_pc_after_probe_load(pc, 0x6800) & 1, 0);
        }
    }

    #[test]
    fn range_gate_advances_in_function_pc_and_leaves_caller_frame_pc() {
        // probe_read32's code extent (a stand-in range; the real bounds come from the linker symbols).
        let lo = 0x0800_0d00u32;
        let hi = 0x0800_0d20u32; // [lo, hi)
                                 // IN-function (the F103 case: stacked PC = load+4, inside probe_read32): advance by the
                                 // decoded width of the instruction at the stacked PC.
        let in_pc = 0x0800_0d06u32; // load (0d02) + 4, still inside
        assert_eq!(
            resume_pc_range_gated(in_pc, lo, hi, 0xF8D0),
            in_pc + 4,
            "in-function 32-bit instruction advances by 4"
        );
        assert_eq!(
            resume_pc_range_gated(in_pc, lo, hi, 0x6800),
            in_pc + 2,
            "in-function 16-bit instruction advances by 2"
        );
        // OUT of function (the F130 case: stacked PC = the caller's return address, past probe_read32):
        // leave the stacked PC UNCHANGED so the not-yet-run consumer re-executes exactly.
        let caller_pc = 0x0800_1a44u32; // in probe_present, outside [lo, hi)
        assert_eq!(
            resume_pc_range_gated(caller_pc, lo, hi, 0xF8D0),
            caller_pc,
            "caller-frame PC is unchanged (re-execute, do not skip)"
        );
        // A PC below the range is also out-of-function -> unchanged.
        assert_eq!(resume_pc_range_gated(lo - 2, lo, hi, 0xF8D0), lo - 2);
    }

    #[test]
    fn range_gate_boundaries_lo_inclusive_hi_exclusive() {
        let lo = 0x0800_0d00u32;
        let hi = 0x0800_0d20u32;
        // lo is inside (>= lo): advance.
        assert_eq!(resume_pc_range_gated(lo, lo, hi, 0x6800), lo + 2);
        // hi is outside (< hi is false): unchanged.
        assert_eq!(resume_pc_range_gated(hi, lo, hi, 0x6800), hi);
        // hi - 2 (last in-range halfword) is inside: advance.
        assert_eq!(resume_pc_range_gated(hi - 2, lo, hi, 0x6800), hi);
    }

    #[test]
    fn frame_model_only_pc_word_changes() {
        // Model the 8-word stacked exception frame and apply the fixup arithmetic the handler does:
        // only word 6 (PC) moves, by the decoded width; every other word (including word 7, xPSR /
        // the Thumb bit) is untouched. This is the property the desynced fixed-width skip violated by
        // overshooting the PC (and the reason the Thumb bit / return state stayed valid here).
        let faulting_pc = 0x0800_0d02u32; // a 32-bit LDR.W probe load
        let xpsr = 0x0100_0000u32; // xPSR with the Thumb bit (bit 24) set
        let mut frame: [u32; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, faulting_pc, xpsr];
        let before = frame;

        // The handler's arithmetic: read word 6, decode width from the instruction (0xF8D0 = LDR.W),
        // write the advanced PC back to word 6.
        frame[STACKED_PC_INDEX] = resume_pc_after_probe_load(frame[STACKED_PC_INDEX], 0xF8D0);

        assert_eq!(
            frame[STACKED_PC_INDEX],
            faulting_pc + 4,
            "PC advanced past the 4-byte LDR.W"
        );
        for i in 0..8 {
            if i != STACKED_PC_INDEX {
                assert_eq!(
                    frame[i], before[i],
                    "word {i} must be untouched (xPSR is word 7)"
                );
            }
        }
        assert_eq!(frame[7], xpsr, "the stacked xPSR (Thumb bit) is preserved");

        // The same frame with the 16-bit lowering resumes 2 bytes on (a fixed +4 skip would have
        // overshot to faulting_pc + 4, landing past the load's `pop {r7,pc}` into adjacent code).
        let mut narrow = before;
        narrow[STACKED_PC_INDEX] = resume_pc_after_probe_load(narrow[STACKED_PC_INDEX], 0x6800);
        assert_eq!(narrow[STACKED_PC_INDEX], faulting_pc + 2);
    }
}
