//! On-silicon acceptance firmware for runtime family detection.
//!
//! This is THE acceptance test for the detection path. It runs the bus-fault-safe ordered GPIO+RCU
//! family probe and the family -> chip synthesis ENTIRELY through runtime-hal (its normal real-MMIO
//! `no_std` build, NOT the `mock` feature: the probe relies on a REAL bus fault, which no
//! host/emulator raises). It calls the same library primitives `detect_chip` does (`probe::run` for
//! the family discriminator, `synthesize` for the descriptor, `probe::measure_counts` for the
//! measured peripheral counts) and records the intermediate probe outcome for the human to verify.
//!
//! It writes a fixed-layout result struct ([`DetectResult`]) to a `.bss` `static mut`, writes the
//! `magic` word LAST, then idles in `wfi`, the same result-struct pattern the `coldpath` firmware uses:
//! the SWD reader resolves the struct address with `arm-none-eabi-nm`, reads it NON-HALTING, and `magic`
//! written last means the whole run completed. The struct address is NOT pinned to a fixed section
//! (a fixed RAM-origin section collided with cortex-m-rt's RAM allocation).
//!
//! # The BusFault handling (the load-bearing piece, now HAL-owned)
//!
//! The BusFault handling is owned entirely by runtime-hal: `probe::run` / `probe::measure_counts`
//! install a probe-scoped vector table whose BusFault slot points at the HAL-internal naked entry
//! `probe::bus_fault_entry`, run the probe, then restore `VTOR`. On a probe fault the lib advances the
//! stacked PC past the fixed-width probe load (the `+4` PC-fixup, DF-5) so the faulting `LDR` is
//! skipped on return; the probe then reads the recorded "faulted" flag as the family-negative signal.
//! This firmware therefore defines NO `#[exception] BusFault`; it is a thin recorder around the lib.
//!
//! # Two-board acceptance
//!
//! `adc_count` / `adv_timers` here are the MEASURED per-instance counts (via the scratch write-back),
//! not the family default, so the bench F103C8's 1 advanced timer shows up as 1 (the family default
//! would have said 2).
//!
//! - **F103 master** (dapdirect ST-Link): expect F10x. `detected_family == 2`, `f1x0_probe == 2`
//!   (BUS-FAULTED on the 0x4800_0000 AHB probe), `f10x_probe == 1` (CLEAN READ on the 0x4001_0800 APB
//!   probe), `gpioa_base == 0x4001_0800`, `gpio == ApbCrlCrh(0)`, `clock == F10xRcc(0)`,
//!   `adc_count == 2`, `adv_timers == 1` (measured: TIMER0 only on the C8), `flash_page == K1(0)`,
//!   `no_family == 0`, `magic == MAGIC`.
//! - **F130 slave** (HLA-only clone): expect F1x0. `detected_family == 1`, `f1x0_probe == 1` (CLEAN
//!   READ on 0x4800_0000; `gpioa_readback == 0x682a73a3`), the F10x probe is NOT RUN (step 1 cleared,
//!   so `f10x_probe == 0` = not-run), `gpioa_base == 0x4800_0000`, `gpio == AhbCtlAfsel(1)`,
//!   `clock == F1x0Rcu(1)`, `adc_count == 1`, `adv_timers == 1` (measured), `flash_page == K1(0)`,
//!   `no_family == 0`, `magic == MAGIC`.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use runtime_hal::detect::probe::{self, F10X_GPIOA_BASE, F1X0_GPIOA_BASE};
// `Family` / `synthesize` come from the crate-root re-export gated behind the `detect-internals`
// feature (enabled in this crate's Cargo.toml), not the `detect` module path.
use runtime_hal::{synthesize, Chip, ClockPath, Family, GpioPath, PageSize};

/// The probe-record sentinels for a per-candidate probe field (`f1x0_probe` / `f10x_probe`).
mod probe_code {
    /// The candidate was NOT RUN (e.g. the F10x probe when step 1 already matched F1x0).
    pub const NOT_RUN: u8 = 0;
    /// The candidate read CLEANLY (no bus fault) => this family.
    pub const CLEAN: u8 = 1;
    /// The candidate BUS-FAULTED => not this family.
    pub const FAULTED: u8 = 2;
}

// --- the SWD-readable result struct ------------------------------------------------------------

/// The fixed-layout DF-T6 result (spec section 8.3). `#[repr(C)]` fixes the field order/offsets so
/// the SWD reader can index by byte offset. `magic` is written LAST (0 = the run never completed /
/// the probe hung). The per-candidate probe records use [`probe_code`] sentinels.
#[repr(C)]
struct DetectResult {
    /// 0x4DF7_0B1E, written LAST = the full run completed.
    magic: u32,
    /// The GPIOA control word read at the matching family's base (expect 0x682a73a3 on the F130).
    gpioa_readback: u32,
    /// The GPIOA base the detected family resolves to (0x4800_0000 on F130, 0x4001_0800 on F103).
    gpioa_base: u32,
    /// Detected family code: 0 = none, 1 = F1x0, 2 = F10x ([`Family`]'s wire codes).
    detected_family: u8,
    /// F1x0 candidate (0x4800_0000 AHB probe): [`probe_code`] {NOT_RUN, CLEAN, FAULTED}.
    f1x0_probe: u8,
    /// F10x candidate (0x4001_0800 APB probe): [`probe_code`] {NOT_RUN, CLEAN, FAULTED}.
    f10x_probe: u8,
    /// Non-zero only if the probe matched NEITHER family (expected 0 on both bench boards).
    no_family: u8,
    /// Synthesized `gpio` selector discriminant (0 = ApbCrlCrh/F10x, 1 = AhbCtlAfsel/F1x0).
    syn_gpio: u8,
    /// Synthesized `clock` selector discriminant (0 = F10xRcc, 1 = F1x0Rcu).
    syn_clock: u8,
    /// Synthesized `adc_count` (1 = F1x0, 2 = F10x).
    syn_adc_count: u8,
    /// Synthesized `adv_timers` (1 = F1x0, 2 = F10x).
    syn_adv_timers: u8,
    /// Synthesized `flash_page` discriminant (0 = K1, 1 = K2).
    syn_flash_page: u8,
    /// `FLASH_DENSITY[15:0]` read at 0x1FFF_F7E0 (KiB of flash; the F10x page-size cross-check).
    flash_density: u16,
}

/// The magic value written last once the whole run completed.
const MAGIC: u32 = 0x4DF7_0B1E;

/// The result struct, a zero-initialised `static mut` in `.bss`. `#[no_mangle]` keeps it a findable
/// symbol so the SWD reader resolves its address via `arm-none-eabi-nm`; `magic` starts 0, so no
/// fixed section / RAM-origin placement is needed.
#[no_mangle]
static mut DETECT_RESULT: DetectResult = DetectResult {
    magic: 0,
    gpioa_readback: 0,
    gpioa_base: 0,
    detected_family: 0,
    f1x0_probe: probe_code::NOT_RUN,
    f10x_probe: probe_code::NOT_RUN,
    no_family: 0,
    syn_gpio: 0xFF,
    syn_clock: 0xFF,
    syn_adc_count: 0,
    syn_adv_timers: 0,
    syn_flash_page: 0xFF,
    flash_density: 0,
};

// --- entry -------------------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    // Runtime detection, the same primitives `detect_chip` runs, decomposed here so the intermediate
    // outcome can be recorded: the bus-fault-safe family probe, then the family -> chip synthesis,
    // then the MEASURED peripheral counts. This runs on the reset IRC8M clock. The HAL installs its
    // own probe-scoped vector table for the fault-safe reads and restores VTOR before each call
    // returns, so this firmware defines no BusFault handler.
    match probe::run() {
        Some(detected) => {
            let family = detected.family;
            // Record the per-candidate probe outcome. The probe always runs the F1x0 candidate
            // first; the F10x candidate runs ONLY if F1x0 faulted. Reconstruct that record from the
            // detected family:
            //   - family F1x0  => f1x0 CLEAN, f10x NOT-RUN (step 2 skipped).
            //   - family F10x  => f1x0 FAULTED, f10x CLEAN.
            let (f1x0_rec, f10x_rec, gpioa_base) = match family {
                Family::F1x0 => (probe_code::CLEAN, probe_code::NOT_RUN, F1X0_GPIOA_BASE),
                Family::F10x => (probe_code::FAULTED, probe_code::CLEAN, F10X_GPIOA_BASE),
            };
            store_u8(StructField::F1x0Probe, f1x0_rec);
            store_u8(StructField::F10xProbe, f10x_rec);
            store_u8(StructField::DetectedFamily, family as u8);
            store_u32(StructField::GpioaBase, gpioa_base);

            // The GPIOA control-word readback at the detected base (the F130 reads 0x682a73a3). The
            // family's GPIO clock was enabled by the probe, so this read is clean.
            // SAFETY: the detected family's GPIOA base is present on this silicon (the probe proved
            // it did not fault) and its clock is enabled.
            let readback = unsafe { core::ptr::read_volatile(gpioa_base as *const u32) };
            store_u32(StructField::GpioaReadback, readback);

            // Synthesize the family-correct descriptor (selectors + density-derived flash page), then
            // build the same Chip detect_chip returns by writing the MEASURED per-instance counts over
            // the family default. Record the synthesized selectors and the measured counts.
            let mut desc = synthesize(family, detected.flash_kib);
            let counts = probe::measure_counts();
            desc.adv_timers = counts.adv_timers;
            desc.adc_count = counts.adc_count;
            let chip = Chip::from_descriptor(desc);

            store_u8(StructField::SynGpio, gpio_code(chip.gpio()));
            store_u8(StructField::SynClock, clock_code(chip.clock()));
            store_u8(StructField::SynAdcCount, chip.adc_count()); // MEASURED
            store_u8(StructField::SynAdvTimers, chip.adv_timers()); // MEASURED
            store_u8(StructField::SynFlashPage, page_code(chip.flash_page()));

            // The density register (the F10x page-size cross-check; constant K1 on F1x0).
            // SAFETY: the flash information block is always mapped and readable; no side effect.
            let density =
                unsafe { core::ptr::read_volatile(runtime_hal::FLASH_DENSITY_ADDR as *const u32) };
            store_u16(StructField::FlashDensity, (density & 0xFFFF) as u16);
        }
        None => {
            // NoFamily (neither candidate matched): fail safe. Record no_family so the human sees the
            // probe matched nothing. detected_family stays 0.
            store_u8(StructField::NoFamily, 1);
        }
    }

    // Done: write the magic LAST, then idle. A reader that sees MAGIC knows every store above ran.
    store_u32(StructField::Magic, MAGIC);
    loop {
        cortex_m::asm::wfi();
    }
}

// --- selector-discriminant encoders ------------------------------------------------------------

#[inline]
fn gpio_code(g: GpioPath) -> u8 {
    match g {
        GpioPath::ApbCrlCrh => 0,
        GpioPath::AhbCtlAfsel => 1,
    }
}
#[inline]
fn clock_code(c: ClockPath) -> u8 {
    match c {
        ClockPath::F10xRcc => 0,
        ClockPath::F1x0Rcu => 1,
    }
}
#[inline]
fn page_code(p: PageSize) -> u8 {
    match p {
        PageSize::K1 => 0,
        PageSize::K2 => 1,
    }
}

// --- result-struct writers (volatile, through the raw pointer to the static) -------------------
//
// Mirrors the coldpath `store!` pattern: volatile stores so the optimiser cannot drop/reorder the
// writes the SWD reader depends on, and so the magic genuinely lands last.

/// The fields the firmware writes, kept as a small enum so each store names its target unambiguously.
enum StructField {
    Magic,
    GpioaReadback,
    GpioaBase,
    DetectedFamily,
    F1x0Probe,
    F10xProbe,
    NoFamily,
    SynGpio,
    SynClock,
    SynAdcCount,
    SynAdvTimers,
    SynFlashPage,
    FlashDensity,
}

#[inline]
fn result_ptr() -> *mut DetectResult {
    core::ptr::addr_of_mut!(DETECT_RESULT)
}

fn store_u32(field: StructField, val: u32) {
    // SAFETY: single-threaded firmware; the only writer is this path, reads are external (SWD).
    unsafe {
        let p = result_ptr();
        match field {
            StructField::Magic => core::ptr::addr_of_mut!((*p).magic).write_volatile(val),
            StructField::GpioaReadback => {
                core::ptr::addr_of_mut!((*p).gpioa_readback).write_volatile(val)
            }
            StructField::GpioaBase => core::ptr::addr_of_mut!((*p).gpioa_base).write_volatile(val),
            _ => unreachable!(),
        }
    }
}

fn store_u16(field: StructField, val: u16) {
    // SAFETY: as above.
    unsafe {
        let p = result_ptr();
        match field {
            StructField::FlashDensity => {
                core::ptr::addr_of_mut!((*p).flash_density).write_volatile(val)
            }
            _ => unreachable!(),
        }
    }
}

fn store_u8(field: StructField, val: u8) {
    // SAFETY: as above.
    unsafe {
        let p = result_ptr();
        match field {
            StructField::DetectedFamily => {
                core::ptr::addr_of_mut!((*p).detected_family).write_volatile(val)
            }
            StructField::F1x0Probe => core::ptr::addr_of_mut!((*p).f1x0_probe).write_volatile(val),
            StructField::F10xProbe => core::ptr::addr_of_mut!((*p).f10x_probe).write_volatile(val),
            StructField::NoFamily => core::ptr::addr_of_mut!((*p).no_family).write_volatile(val),
            StructField::SynGpio => core::ptr::addr_of_mut!((*p).syn_gpio).write_volatile(val),
            StructField::SynClock => core::ptr::addr_of_mut!((*p).syn_clock).write_volatile(val),
            StructField::SynAdcCount => {
                core::ptr::addr_of_mut!((*p).syn_adc_count).write_volatile(val)
            }
            StructField::SynAdvTimers => {
                core::ptr::addr_of_mut!((*p).syn_adv_timers).write_volatile(val)
            }
            StructField::SynFlashPage => {
                core::ptr::addr_of_mut!((*p).syn_flash_page).write_volatile(val)
            }
            _ => unreachable!(),
        }
    }
}
