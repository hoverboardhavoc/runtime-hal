//! Peripheral-presence probe firmware: MEASURE which advanced timers / ADCs a chip actually has.
//!
//! # Why this exists
//!
//! The library's `detect_chip` MEASURES the advanced-timer / ADC instance counts (inside
//! [`probe::run`], via [`probe::measure_counts`]) rather than trusting a family CAPABILITY baseline
//! (F10x can carry up to 2 advanced timers / 2 ADCs; F1x0 up to 1 / 1): a medium-density F103C8
//! carries only 1 advanced timer (TIMER0) and 2 ADCs, while a high-density F10x (two 3-phase bridges
//! => TIMER0 + TIMER7) carries more. This firmware is the STANDALONE VALIDATOR for that measurement:
//! it MEASURES presence per instance two ways and reports the raw sub-signals next to the family
//! capability baseline ([`family_capability`]), so we can see which signal is trustworthy and whether
//! that baseline over- or under-counts. (The library relies only on the write-back signal, which this
//! firmware confirmed is the trustworthy one; see `bench/peripheral-probe-2026-06-18.md`.)
//!
//! # What it does (and the mirror of bench-fw-detect)
//!
//! It runs the bus-fault-safe ordered GPIO+RCU family probe (`probe::run`, giving the detected family
//! + the family-correct register model). Then, inside the HAL's probe-scoped vector table
//! (`probe::with_probe_vector_table`) with BusFault enabled ONCE around the whole sweep, it MEASURES
//! each candidate peripheral two independent ways and records both plus
//! the raw sub-results, classifies present/absent/disagree, writes a fixed-layout result struct to a
//! `.bss` `static mut`, writes `magic` LAST, and busy-spins (NOT `wfi`; see the idle loop), exactly the bench-fw-detect pattern:
//! the SWD reader reads the struct at its FIXED address ([`RESULT_ADDR`]) NON-HALTING, and
//! `magic` written last means the whole run completed. The struct is NOT pinned to a fixed section
//! (the bench-fw-m2 lesson: a fixed RAM-origin section collided with cortex-m-rt's RAM allocation).
//!
//! # The two independent presence signals (per candidate)
//!
//! 1. **RCU enable-bit stickiness** ([`probe::rcu_enable_sticky`]): set the peripheral's `RCU_APB2EN`
//!    clock-enable bit and read it back. A present peripheral's enable bit latches to 1; an absent
//!    peripheral's bit is Reserved and typically reads back 0. (Leaving the bit set is harmless: it
//!    only gates a clock; the sweep never configures the peripheral behind it.)
//! 2. **Fault-safe base read + benign scratch write-back** ([`probe::probe_present`]): read the
//!    peripheral's base register inside the armed BusFault window (faulted? + readback value), then
//!    write a test pattern to a SAFE, side-effect-free R/W register of that peripheral, read it back,
//!    and restore it. A present peripheral's scratch register accepts the pattern; an absent one either
//!    faulted on the base read or does not retain the pattern.
//!
//! `present = sticky AND not-faulted AND writeback-ok`. The per-candidate status byte records
//! {ABSENT, PRESENT, DISAGREE} so we can see WHICH signal disagreed, alongside the three raw
//! sub-results (sticky / faulted / writeback-ok) and the base readback.
//!
//! # SAFETY (the hard constraint): strictly read-only, FETs OFF, logic-only
//!
//! The sweep does ONLY: clock-enable + enable-bit-readback + base register read + benign
//! scratch-write-restore. The scratch registers are chosen to be side-effect-free (verified against the
//! GD32F10x Rev2.6 and GD32F1x0 Rev3.6 user manuals):
//!
//! - TIMER scratch = `TIMERx_PSC` (offset 0x28, reset 0x0000_0000): the prescaler. Writing it does NOT
//!   start the counter, enable an output, set MOE/POEN, or map a pin; it only loads the prescaler
//!   shadow. It is restored to its reset value 0 afterward. We NEVER touch CTL0/CTL1 (counter/MOE
//!   enables), CHCTL0/1 (channel config), CHCTL2 (channel-enable/CCER), or BKDT (break/dead-time/POEN).
//! - ADC scratch = `ADC_WDLT` (offset 0x28, reset 0x0000_0000): the analog-watchdog LOW threshold. It
//!   is a benign comparison threshold with NO effect unless the watchdog is separately enabled
//!   (WDEN/RWDEN in CTL0, which the sweep never sets). It is restored to 0. We NEVER touch CTL1
//!   (ADON/ADCON power-on, SWRCST software start) or any conversion-trigger bit.
//!
//! This firmware NEVER configures PWM, NEVER sets MOE/POEN/BKDT, NEVER sets a channel enable, NEVER
//! maps a gate pin. The 12-FET power stage stays off; this is logic-only.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use runtime_hal::detect::probe::{self, RCU_APB2EN_OFFSET};
// `family_capability` comes from the crate-root re-export gated behind the `detect-internals` feature
// (enabled in this crate's Cargo.toml), not the `detect` module path. It is the constant per-family
// capability baseline (the most advanced timers / ADCs a part of the family can carry) the measured
// per-instance presence is compared against.
use runtime_hal::family_capability;

/// The magic value written last once the whole run completed (distinct from bench-fw-detect's).
const MAGIC: u32 = 0x4DF7_9505;

// --- per-peripheral classification + candidate identity sentinels ------------------------------

/// Per-candidate classified presence status.
mod status {
    /// All signals say absent (or they agree the peripheral is not there).
    pub const ABSENT: u8 = 0;
    /// All three signals agree the peripheral is present (sticky AND not-faulted AND writeback-ok).
    pub const PRESENT: u8 = 1;
    /// The signals DISAGREE (e.g. the clock bit stuck but the base read faulted, or vice versa). The
    /// raw sub-results below show which signal dissented, the whole point of recording both.
    pub const DISAGREE: u8 = 2;
}

/// A stable identity code per candidate, written into each record so the SWD decode is unambiguous
/// regardless of family / array position.
mod periph_id {
    pub const TIMER0: u8 = 1;
    pub const TIMER7: u8 = 2;
    pub const ADC0: u8 = 3;
    pub const ADC1: u8 = 4;
    pub const ADC2: u8 = 5;
}

/// The kind of scratch (side-effect-free R/W) register a candidate uses, so the sweep picks the right
/// offset / pattern / restore value.
#[derive(Clone, Copy)]
enum Kind {
    /// An advanced timer: scratch = `TIMERx_PSC` (offset 0x28, reset 0). See module SAFETY note.
    Timer,
    /// An ADC: scratch = `ADC_WDLT` (offset 0x28, reset 0). See module SAFETY note.
    Adc,
}

/// A candidate peripheral to MEASURE: its identity code, base address, `RCU_APB2EN` enable bit, and
/// kind (which fixes the scratch register). All addresses + bits were verified against the GD32F10x
/// Rev2.6 and GD32F1x0 Rev3.6 user manuals (see the bench writeup for the exact manual references).
struct Candidate {
    id: u8,
    base: u32,
    /// The candidate's clock-enable bit in `RCU_APB2EN` (TIMER0EN=11, TIMER7EN=13, ADC0EN/ADCEN=9,
    /// ADC1EN=10, ADC2EN=15). On F1x0 the bit-13 (TIMER7) and bit-10/15 (ADC1/ADC2) positions are
    /// Reserved, so stickiness there is the absence signal.
    apb2en_bit: u32,
    kind: Kind,
}

/// The candidate set, probed IDENTICALLY on BOTH families so absence shows up as absence (the
/// contrast is the point). Expected on the bench:
///   - F103C8 (medium density): TIMER0 present, TIMER7 ABSENT, ADC0 + ADC1 present, ADC2 ABSENT.
///     (The family constant says adv_timers=2/adc_count=2; a C8 should MEASURE fewer, the headline.)
///   - F130C8: TIMER0 + the single ADC (ADC0) present; TIMER7, ADC1, ADC2 ABSENT.
///   - A high-density F10x (the 12-FET board, NOT tested here): TIMER0 + TIMER7 present (two 3-phase
///     bridges), and possibly ADC2 present, the contrast vs the C8.
const CANDIDATES: [Candidate; 5] = [
    // TIMER0 @ 0x4001_2C00, RCU_APB2EN bit 11 (TIMER0EN). Present on both families.
    Candidate {
        id: periph_id::TIMER0,
        base: 0x4001_2C00,
        apb2en_bit: 11,
        kind: Kind::Timer,
    },
    // TIMER7 @ 0x4001_3400, RCU_APB2EN bit 13 (TIMER7EN). Present only on higher-density F10x; the
    // slot + the APB2EN bit are Reserved on F1x0 and on a medium-density F103C8.
    Candidate {
        id: periph_id::TIMER7,
        base: 0x4001_3400,
        apb2en_bit: 13,
        kind: Kind::Timer,
    },
    // ADC0 @ 0x4001_2400, RCU_APB2EN bit 9 (ADC0EN on F10x; named ADCEN, same bit, on F1x0). Present
    // on both families (the single ADC on F1x0 lives here).
    Candidate {
        id: periph_id::ADC0,
        base: 0x4001_2400,
        apb2en_bit: 9,
        kind: Kind::Adc,
    },
    // ADC1 @ 0x4001_2800, RCU_APB2EN bit 10 (ADC1EN). Present on F103; the slot + bit are Reserved on
    // F1x0 (single ADC).
    Candidate {
        id: periph_id::ADC1,
        base: 0x4001_2800,
        apb2en_bit: 10,
        kind: Kind::Adc,
    },
    // ADC2 @ 0x4001_3C00, RCU_APB2EN bit 15 (ADC2EN). Present only on higher-density F10x; Reserved on
    // F1x0 and expected ABSENT on a medium-density F103C8.
    Candidate {
        id: periph_id::ADC2,
        base: 0x4001_3C00,
        apb2en_bit: 15,
        kind: Kind::Adc,
    },
];

/// The side-effect-free scratch-register offset + restore value per kind (see module SAFETY note).
/// `TIMERx_PSC` and `ADC_WDLT` both sit at offset 0x28 and both reset to 0, so the offset/restore are
/// the same here, but the kinds are kept distinct so the rationale (and any future divergence) is
/// explicit.
const SCRATCH_OFFSET: u32 = 0x28;
/// The reset value to restore the scratch register to after the write-back test (0 for both PSC and
/// WDLT).
const SCRATCH_RESET: u32 = 0x0000_0000;
/// The benign test pattern written to the scratch register (a recognizable, in-range value: PSC is a
/// 16-bit prescaler and WDLT a 12-bit threshold, so the low bits are what actually retain).
const SCRATCH_PATTERN: u32 = 0x0000_5A5A;
/// The mask of bits the scratch register actually retains: PSC retains 16 bits, ADC_WDLT retains 12.
/// The write-back check compares only the retained bits so a present peripheral that masks the upper
/// bits is not misread as "did not retain". `0x5A5A` fits in 12 bits, so a single mask works for both.
const SCRATCH_RETAIN_MASK: u32 = 0x0000_0FFF;

// --- the SWD-readable result struct ------------------------------------------------------------

/// One candidate's measured record. `#[repr(C)]` fixes the offsets so the SWD reader can index it.
/// Records BOTH presence signals and the raw sub-results so a disagreement shows WHICH signal
/// dissented.
#[repr(C)]
#[derive(Clone, Copy)]
struct PeriphRecord {
    /// The candidate base address probed (so the decode is self-describing).
    base: u32,
    /// The base register readback (meaningful only if `faulted == 0`).
    base_readback: u32,
    /// [`periph_id`] identity code (TIMER0/TIMER7/ADC0/ADC1/ADC2).
    id: u8,
    /// Signal (a): RCU enable-bit stickiness, 1 = the `RCU_APB2EN` bit read back as 1, else 0.
    sticky: u8,
    /// Signal (b) part 1: 1 = the base read BUS-FAULTED (absent / reserved), 0 = clean read.
    faulted: u8,
    /// Signal (b) part 2: 1 = the benign scratch write-read-restore retained the pattern, else 0.
    /// (0 if the base faulted, since the scratch write is then skipped.)
    writeback_ok: u8,
    /// The classified [`status`]: ABSENT / PRESENT / DISAGREE.
    status: u8,
}

/// The fixed-layout result. `#[repr(C)]`; `magic` is written LAST (0 = the run never completed). The
/// SWD reader reads `ProbeResult` at the fixed [`RESULT_ADDR`] and decodes by byte offset.
#[repr(C)]
struct ProbeResult {
    /// 0x4DF7_9505, written LAST = the full run completed.
    magic: u32,
    /// `FLASH_DENSITY[15:0]` at 0x1FFF_F7E0 (KiB of flash; corroborates the part).
    flash_density: u16,
    /// Detected family code: 0 = none, 1 = F1x0, 2 = F10x ([`Family`]'s wire codes).
    detected_family: u8,
    /// Non-zero only if the family probe matched NEITHER family (then the sweep is skipped).
    no_family: u8,
    /// The family CAPABILITY baseline for advanced timers (the most a part of this family can carry),
    /// recorded for the headline comparison: this is what a no-measurement family inference would
    /// report. Same field offset and same recorded value (F10x = 2, F1x0 = 1) as before; only its
    /// source changed (the constant `family_capability`, not a half-built descriptor).
    syn_adv_timers: u8,
    /// The family CAPABILITY baseline for ADC instances (the other half of the comparison; F10x = 2,
    /// F1x0 = 1).
    syn_adc_count: u8,
    /// Padding to keep the records array 4-byte aligned and the layout obvious to the decoder.
    _pad: [u8; 2],
    /// The per-candidate measured records, one per [`CANDIDATES`] entry, same order.
    records: [PeriphRecord; 5],
}

/// Fixed RAM address of the result struct: the top of the (shrunk) RAM region, reserved by `memory.x`
/// (cortex-m-rt's RAM ends 256 bytes early so it never allocates here). The SWD reader reads this
/// CONSTANT directly, no `arm-none-eabi-nm` resolution needed (the size-optimised release ELF drops
/// the `.symtab` nm reads, so a symbol-based read is unreliable; a fixed address is not).
const RESULT_ADDR: u32 = 0x2000_1F00;

/// Initial result contents, written to [`RESULT_ADDR`] at startup. The region is OUTSIDE `.bss` (above
/// cortex-m-rt's RAM), so the C runtime does NOT zero it; `main` writes this first so a reader sees
/// `magic == 0` until the run completes and writes `magic` LAST.
const INIT_RESULT: ProbeResult = ProbeResult {
    magic: 0,
    flash_density: 0,
    detected_family: 0,
    no_family: 0,
    syn_adv_timers: 0,
    syn_adc_count: 0,
    _pad: [0; 2],
    records: [PeriphRecord {
        base: 0,
        base_readback: 0,
        id: 0,
        sticky: 0,
        faulted: 0,
        writeback_ok: 0,
        status: status::ABSENT,
    }; 5],
};

// --- entry -------------------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    // Initialise the fixed-address result region (outside .bss, so not zeroed by the C runtime):
    // write the defaults first, magic = 0 until the run completes.
    // SAFETY: RESULT_ADDR is reserved RAM (see memory.x); single writer.
    unsafe { core::ptr::write_volatile(result_ptr(), INIT_RESULT) };

    // Step 1: the bus-fault-safe family probe (giving the detected family + the family-correct
    // register model). We record the family CAPABILITY baseline adv_timers/adc_count (the most a part
    // of this family can carry, the genuine constant `family_capability` fact) so the bench decode can
    // compare it against the MEASURED per-instance presence below (the headline: does the family
    // baseline match reality?). NOTE: `probe::run` also measures the per-instance counts internally;
    // this firmware re-measures each candidate below to break out the raw sub-signals, which the
    // library's count does not expose.
    let family = match probe::run() {
        Some(detected) => {
            let fam = detected.family;
            store_u8(Field::DetectedFamily, fam as u8);
            // The family capability baseline, straight from the constant family model (NOT a
            // half-built descriptor): the most advanced timers / ADCs this family can carry.
            let (max_adv_timers, max_adcs) = family_capability(fam);
            store_u8(Field::SynAdvTimers, max_adv_timers);
            store_u8(Field::SynAdcCount, max_adcs);
            Some(fam)
        }
        None => {
            // NoFamily: fail safe, do not sweep peripherals on an unknown part.
            store_u8(Field::NoFamily, 1);
            None
        }
    };

    // The density register (corroboration; always mapped, no fault). Read regardless of the sweep.
    // SAFETY: the flash information block is always mapped and readable; no side effect, no fault.
    let density =
        unsafe { core::ptr::read_volatile(runtime_hal::FLASH_DENSITY_ADDR as *const u32) };
    store_u16(Field::FlashDensity, (density & 0xFFFF) as u16);

    // Step 2: the peripheral-presence sweep, only on a known family. Enable BusFault ONCE around the
    // whole sweep (the section-3 fault-safe harness, reused via the public helpers), measure each
    // candidate two independent ways, classify, and restore BusFault.
    if family.is_some() {
        // Install the HAL's probe-scoped vector table around the whole sweep (its BusFault slot points
        // at the HAL-internal entry; VTOR is restored when the closure returns), then enable the
        // dedicated BusFault handler once for the sweep. This firmware defines no fault handler.
        probe::with_probe_vector_table(|| {
            let prev = probe::arm_busfault();

            let mut i = 0;
            while i < CANDIDATES.len() {
                let c = &CANDIDATES[i];
                let rec = measure(c);
                store_record(i, rec);
                i += 1;
            }

            // Restore BusFault to its prior state (and disarm the probe window) before idling.
            probe::disarm_busfault(prev);
        });
    }

    // Done: write the magic LAST, then idle. A reader that sees MAGIC knows every store above ran.
    store_u32(Field::Magic, MAGIC);
    // Busy-spin, NOT wfi. A core left in WFI sleep with no DBGMCU debug-low-power bits set locks out
    // SWD re-attach on these GD32F130s (the AP-write to halt the sleeping core fails after a
    // power-cycle), which presents as a permanent debug "brick" until a connect-under-reset +
    // mass-erase. This validator has no reason to sleep, so spin and stay re-attachable.
    loop {
        cortex_m::asm::nop();
    }
}

/// MEASURE one candidate two independent ways and classify. Strictly: clock-enable + enable-bit
/// readback + base read + benign scratch write-read-restore (see the module SAFETY note for why each
/// touched register is side-effect-free). Assumes BusFault is already enabled (by [`probe::arm_busfault`]).
fn measure(c: &Candidate) -> PeriphRecord {
    // Signal (a): RCU enable-bit stickiness. Set the candidate's APB2EN clock-enable bit and read it
    // back. (Harmless: a clock gate; the bit is left set, the peripheral is never configured.)
    let sticky = probe::rcu_enable_sticky(RCU_APB2EN_OFFSET, c.apb2en_bit);

    // Signal (b) part 1: fault-safe base read. A fault => absent / reserved base.
    let base_read = probe::probe_present(c.base);
    let faulted = base_read.is_none();
    let base_readback = base_read.unwrap_or(0);

    // Signal (b) part 2: benign scratch write-read-restore, ONLY if the base read was clean (no point
    // writing to a base that just faulted). Read the scratch register first (inside the armed window,
    // so a fault is caught), write the test pattern, read it back, then RESTORE the reset value. The
    // scratch register (TIMERx_PSC / ADC_WDLT) is side-effect-free per the module SAFETY note.
    let writeback_ok = if faulted { false } else { scratch_writeback(c) };

    let present = sticky && !faulted && writeback_ok;
    // Classify. If all signals agree present => PRESENT; if they agree absent (nothing latched, base
    // faulted, no write-back) => ABSENT; otherwise the signals DISAGREE (the interesting case).
    let all_absent = !sticky && faulted && !writeback_ok;
    let class = if present {
        status::PRESENT
    } else if all_absent {
        status::ABSENT
    } else {
        status::DISAGREE
    };

    PeriphRecord {
        base: c.base,
        base_readback,
        id: c.id,
        sticky: sticky as u8,
        faulted: faulted as u8,
        writeback_ok: writeback_ok as u8,
        status: class,
    }
}

/// The benign scratch write-read-restore for one candidate, inside the armed BusFault window. Returns
/// whether the retained bits of the scratch register read back as the test pattern (a present
/// peripheral retains it; an absent one does not). RESTORES the scratch register to its reset value
/// afterward, so the peripheral is left in its reset state. Side-effect-free (see module SAFETY note).
fn scratch_writeback(c: &Candidate) -> bool {
    let scratch = c.base + SCRATCH_OFFSET;

    // Read the scratch register inside the armed window FIRST (so an unexpected fault here is caught,
    // not escalated). A clean read proves the scratch address is present and not reserved, which makes
    // the subsequent plain write to the SAME address safe (it cannot fault if the read did not). If it
    // faults, treat the write-back as failed (absent) and do nothing else.
    if probe::probe_present(scratch).is_none() {
        return false;
    }

    // Write the benign test pattern. The scratch read above was clean, so this store to the same
    // address cannot fault; a plain volatile write is sufficient (no armed window needed for it).
    // SAFETY: `scratch` is a side-effect-free R/W register (TIMERx_PSC / ADC_WDLT, see module SAFETY
    // note) of a peripheral whose base AND scratch register both just read cleanly and whose clock we
    // enabled. The write only loads a prescaler / watchdog-threshold shadow; it starts nothing and
    // enables no output. We restore the reset value below.
    unsafe {
        core::ptr::write_volatile(scratch as *mut u32, SCRATCH_PATTERN);
    }

    // Read back (inside the armed window) and compare only the retained bits.
    let readback = probe::probe_present(scratch).unwrap_or(0);
    let retained = (readback & SCRATCH_RETAIN_MASK) == (SCRATCH_PATTERN & SCRATCH_RETAIN_MASK);

    // RESTORE the scratch register to its reset value, leaving the peripheral as we found it.
    // SAFETY: as above; restoring the documented reset value (0) of a side-effect-free register.
    unsafe {
        core::ptr::write_volatile(scratch as *mut u32, SCRATCH_RESET);
    }
    let _ = c.kind; // kind currently maps to the same offset/reset for both; kept for clarity.

    retained
}

// --- result-struct writers (volatile, through the raw pointer to the static) -------------------
//
// Mirrors the bench-fw-detect / bench-fw-m2 `store!` pattern: volatile stores so the optimiser cannot
// drop/reorder the writes the SWD reader depends on, and so the magic genuinely lands last.

/// The scalar fields the firmware writes (the per-candidate records use [`store_record`]).
enum Field {
    Magic,
    FlashDensity,
    DetectedFamily,
    NoFamily,
    SynAdvTimers,
    SynAdcCount,
}

#[inline]
fn result_ptr() -> *mut ProbeResult {
    RESULT_ADDR as *mut ProbeResult
}

fn store_u32(field: Field, val: u32) {
    // SAFETY: single-threaded firmware; the only writer is this path, reads are external (SWD).
    unsafe {
        let p = result_ptr();
        match field {
            Field::Magic => core::ptr::addr_of_mut!((*p).magic).write_volatile(val),
            _ => unreachable!(),
        }
    }
}

fn store_u16(field: Field, val: u16) {
    // SAFETY: as above.
    unsafe {
        let p = result_ptr();
        match field {
            Field::FlashDensity => core::ptr::addr_of_mut!((*p).flash_density).write_volatile(val),
            _ => unreachable!(),
        }
    }
}

fn store_u8(field: Field, val: u8) {
    // SAFETY: as above.
    unsafe {
        let p = result_ptr();
        match field {
            Field::DetectedFamily => {
                core::ptr::addr_of_mut!((*p).detected_family).write_volatile(val)
            }
            Field::NoFamily => core::ptr::addr_of_mut!((*p).no_family).write_volatile(val),
            Field::SynAdvTimers => core::ptr::addr_of_mut!((*p).syn_adv_timers).write_volatile(val),
            Field::SynAdcCount => core::ptr::addr_of_mut!((*p).syn_adc_count).write_volatile(val),
            _ => unreachable!(),
        }
    }
}

/// Store one measured candidate record at index `i` (volatile, field by field, so the optimiser
/// cannot drop the writes the SWD reader depends on).
fn store_record(i: usize, rec: PeriphRecord) {
    // SAFETY: single-threaded firmware; `i` is bounded by the CANDIDATES length (the caller's loop),
    // and the records array has exactly that many slots.
    unsafe {
        let p = result_ptr();
        let slot = core::ptr::addr_of_mut!((*p).records[i]);
        core::ptr::addr_of_mut!((*slot).base).write_volatile(rec.base);
        core::ptr::addr_of_mut!((*slot).base_readback).write_volatile(rec.base_readback);
        core::ptr::addr_of_mut!((*slot).id).write_volatile(rec.id);
        core::ptr::addr_of_mut!((*slot).sticky).write_volatile(rec.sticky);
        core::ptr::addr_of_mut!((*slot).faulted).write_volatile(rec.faulted);
        core::ptr::addr_of_mut!((*slot).writeback_ok).write_volatile(rec.writeback_ok);
        core::ptr::addr_of_mut!((*slot).status).write_volatile(rec.status);
    }
}
