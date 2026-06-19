//! Runtime heuristic detection: the ONLY way the HAL learns its chip identity.
//!
//! runtime-hal boots as ONE binary on any supported part. It does not read a chip descriptor from
//! flash and it is not built per-chip; instead it PROBES the silicon to work out what the MCU is and
//! what it can do, then synthesizes the [`McuDescriptor`] the rest of the HAL is built on. Three
//! heuristics, in order:
//!
//! 1. **Family discriminator** ([`crate::detect::probe::run`]): the two families put GPIOA at different bases on
//!    different buses (F10x APB at `0x4001_0800`, F1x0 AHB at `0x4800_0000`). Deliberately read the
//!    reserved-region GPIO base of the wrong family and it bus-faults; a clean read confirms the
//!    family. This fixes all four register-model selectors and the base-address table.
//! 2. **Peripheral-presence measurement** ([`crate::detect::probe::measure_counts`]): the advanced-timer and ADC
//!    INSTANCE counts are MEASURED, not inferred from a family constant, by a benign scratch
//!    write-back per candidate (the bench found the family constant wrong in BOTH directions, see
//!    `bench/peripheral-probe-2026-06-18.md`).
//! 3. **Flash density**: read `FLASH_DENSITY[15:0]` at `0x1FFF_F7E0` for the F10x page-size input.
//!
//! [`detect_chip`] runs all three and returns the synthesized [`Chip`] directly, or fails loud
//! ([`DetectError::NoFamily`]) if neither family matched.
//!
//! # The per-family chip-capability constants live HERE
//!
//! [`descriptor_f103`] / [`descriptor_f130`] are the single source of truth for the two parts' chip
//! descriptors (the register-model selectors and base addresses). [`synthesize`] fills a descriptor
//! from a `Family` and the density read; [`detect_chip`] then OVERWRITES the `adv_timers` /
//! `adc_count` fields with the MEASURED counts (the family constant is the fallback, not the truth).
//!
//! # What is host-testable and what is not
//!
//! The deterministic logic is host-tested: the family -> descriptor synthesis and the F10x density ->
//! page-size branch. The bus-fault PROBE itself ([`crate::detect::probe::run`] / [`crate::detect::probe::measure_counts`]) is NOT
//! host-testable: the `mock` register backend and emulators raise NO bus fault on a reserved read, so
//! the probe would simply read zero instead of faulting. The probe ordering, the fault-catch, the
//! PC-fixup, and the positive/negative interpretation are validated ONLY on hardware by the
//! `bench-fw-detect` and `bench-fw-probe` firmware.

use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::DetectError;

pub mod probe;

pub mod bringup;

/// The MCU family the probe resolves. A single family determination fixes all four register-model
/// selectors, the base-address table, and the family-default timer/ADC capability counts (which the
/// peripheral-presence measurement then refines).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    /// GD32F1x0 (e.g. F130): GPIO on AHB2 at `0x4800_0000`, the `RCU_AHBEN` GPIO clock enable.
    /// Wire code 1 (matches the `bench-fw-detect` `detected_family` sentinel).
    F1x0 = 1,
    /// GD32F10x (e.g. F103): GPIO on APB2 at `0x4001_0800`, the `RCU_APB2EN` GPIO clock enable.
    /// Wire code 2.
    F10x = 2,
}

/// TIM8 (TIMER7) base on the F10x high-density parts (APB2 advanced-timer window). Added to the
/// descriptor by [`detect_chip`] only when a second advanced timer is measured, so a part without it
/// resolves `Timer7` as [`crate::error::DescriptorError::MissingBase`].
const TIMER7_BASE: u32 = 0x4001_3400;

/// ADC1 base on the F10x dual-ADC parts (APB2 ADC window, ADC0 + 0x400). Added by [`detect_chip`]
/// only when a second ADC is measured, so single-ADC parts resolve `Adc1` as `MissingBase`.
const ADC1_BASE: u32 = 0x4001_2800;

// --- the per-family chip descriptor constants (the register-model + base-address facts) -------
//
// These are the family-correct selectors and base addresses. `adv_timers` / `adc_count` carry the
// family DEFAULT, which `detect_chip` overwrites with the MEASURED per-instance counts: the bench
// proved the default wrong in both directions (a C8 has 1 advanced timer not 2; an RCT6 has 3 ADCs
// not 2), so it is only a fallback when the measurement could not run.

/// The GD32F103 (F10x) chip descriptor (the F10x register model + APB GPIO bases). The bench F103 is
/// a medium-density part, so `flash_page` is `K1` here; `synthesize` re-derives it from the density
/// register for a high-density part.
pub const fn descriptor_f103() -> McuDescriptor {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, 0x4000_4400);
    // F10x GPIO ports: APB2, GPIOA at 0x4001_0800, 0x400 per-port stride (GPIOC = +0x800,
    // GPIOD = +0xC00, GPIOF = +0x1400). GPIOE (+0x1000) is rarely wired on these boards and is not
    // carried.
    addrs.set(PeriphLabel::Gpioa, 0x4001_0800);
    addrs.set(PeriphLabel::Gpiob, 0x4001_0C00);
    addrs.set(PeriphLabel::Gpioc, 0x4001_1000);
    addrs.set(PeriphLabel::Gpiod, 0x4001_1400);
    addrs.set(PeriphLabel::Gpiof, 0x4001_1C00);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    addrs.set(PeriphLabel::I2c0, 0x4000_5400);
    addrs.set(PeriphLabel::Spi0, 0x4001_3000); // F10x SPI0 on APB2
    addrs.set(PeriphLabel::Adc0, 0x4001_2400);
    addrs.set(PeriphLabel::Timer0, 0x4001_2C00);
    addrs.set(PeriphLabel::Fwdgt, 0x4000_3000); // free watchdog, APB1, same base both families
    addrs.set(PeriphLabel::Timer1, 0x4000_0000); // general TIMER1, APB1, same base both families
    McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Dual,
        irq: IrqLayout::F10xSeparate,
        addrs,
        flash_page: PageSize::K1,
        // Family DEFAULT (high-density F10x: TIMER0 + TIMER7, ADC0 + ADC1); `detect_chip` overwrites
        // both with the measured per-instance counts.
        adv_timers: 2,
        adc_count: 2,
    }
}

/// The GD32F130 (F1x0) chip descriptor (the F1x0 register model + AHB GPIO bases). `flash_page` is
/// the family CONSTANT `K1` (F1x0 is always 1 KiB pages).
pub const fn descriptor_f130() -> McuDescriptor {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, 0x4000_4400);
    // F1x0 GPIO ports: AHB, GPIOA at 0x4800_0000, 0x400 per-port stride (GPIOC = +0x800,
    // GPIOD = +0xC00, GPIOF = +0x1400). F1x0 has no port E (the AHBEN PEEN bit is the gap), so
    // GPIOE is never carried.
    addrs.set(PeriphLabel::Gpioa, 0x4800_0000);
    addrs.set(PeriphLabel::Gpiob, 0x4800_0400);
    addrs.set(PeriphLabel::Gpioc, 0x4800_0800);
    addrs.set(PeriphLabel::Gpiod, 0x4800_0C00);
    addrs.set(PeriphLabel::Gpiof, 0x4800_1400);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    addrs.set(PeriphLabel::I2c0, 0x4000_5400);
    addrs.set(PeriphLabel::Spi0, 0x4000_3800); // F1x0 single SPI on APB1, mapped as SPI0
    addrs.set(PeriphLabel::Adc0, 0x4001_2400);
    addrs.set(PeriphLabel::Timer0, 0x4001_2C00);
    addrs.set(PeriphLabel::Fwdgt, 0x4000_3000); // free watchdog, APB1, same base both families
    addrs.set(PeriphLabel::Timer1, 0x4000_0000); // general TIMER1, APB1, same base both families
    McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel,
        clock: ClockPath::F1x0Rcu,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        // Family DEFAULT (F1x0: single advanced TIMER0, single ADC); `detect_chip` overwrites both
        // with the measured per-instance counts.
        adv_timers: 1,
        adc_count: 1,
    }
}

// --- the family -> descriptor synthesis -------------------------------------------------------

/// The flash density register `FLASH_DENSITY` lives at `0x1FFF_F7E0`; `[15:0]` is KiB of flash.
/// LOAD-BEARING only for the F10x `flash_page` decision.
pub const FLASH_DENSITY_ADDR: u32 = 0x1FFF_F7E0;

/// The F10x flash-size threshold (KiB) above which pages are 2 KiB (`K2`). At or below it the part
/// is medium-density with 1 KiB pages (`K1`): "if flash size > 128 KiB ... K2".
pub const F10X_K2_THRESHOLD_KIB: u16 = 128;

/// Synthesize the per-family chip [`McuDescriptor`] from the detected `family` and the
/// density-register read `flash_kib`.
///
/// The selectors and the address table are CONSTANT per family ([`descriptor_f103`] /
/// [`descriptor_f130`]). `flash_page` is the one per-part input:
/// - **F1x0**: constant `K1` (always 1 KiB pages); `flash_kib` is ignored.
/// - **F10x**: density-dependent. `flash_kib > 128` => `K2` (high/extra density), else `K1`
///   (medium density).
///
/// The `adv_timers` / `adc_count` fields carry the family DEFAULT; [`detect_chip`] overwrites them
/// with the MEASURED per-instance counts. The result passes `addrs.check_ranges(gpio, clock)` by
/// construction (it reuses the family-correct bases).
pub fn synthesize(family: Family, flash_kib: u16) -> McuDescriptor {
    match family {
        Family::F1x0 => descriptor_f130(), // flash_page is the family constant K1; density unused.
        Family::F10x => {
            let mut d = descriptor_f103();
            d.flash_page = if flash_kib > F10X_K2_THRESHOLD_KIB {
                PageSize::K2
            } else {
                PageSize::K1
            };
            d
        }
    }
}

// --- the boot-flow entry ----------------------------------------------------------------------

/// Detect the MCU at runtime and return its [`Chip`] context. This is the single function the
/// firmware calls at boot to learn what silicon it is running on.
///
/// It runs the bus-fault-safe ordered GPIO+RCU family probe ([`crate::detect::probe::run`]) for the family
/// discriminator, MEASURES the advanced-timer and ADC instance counts ([`crate::detect::probe::measure_counts`])
/// rather than trusting the family constant, synthesizes the per-family [`McuDescriptor`]
/// ([`synthesize`]) with the density-derived flash page, writes the MEASURED counts into it, and
/// returns the resulting [`Chip`].
///
/// Fail-loud: if neither family matched it returns [`DetectError::NoFamily`] rather than guessing a
/// register layout (the firmware then fails safe: halt on the reset IRC8M clock, outputs untouched).
///
/// The probe runs ONCE (it does not retry or loop), BEFORE any bring-up, on the reset clock (IRC8M),
/// before any production RAM vector table the application installs later. The BusFault handling is
/// fully HAL-owned: [`probe::run`] and [`probe::measure_counts`] each install a probe-scoped vector
/// table (BusFault slot -> [`probe::bus_fault_entry`]) and restore `VTOR` before returning, so the
/// application defines NO `#[exception] BusFault`.
pub fn detect_chip() -> Result<Chip, DetectError> {
    // 1. Family discriminator. A clean GPIOA read at one family's base, a bus fault at the other's.
    let detected = match probe::run() {
        Some(d) => d,
        // Neither candidate matched: fail safe, do NOT guess.
        None => return Err(DetectError::NoFamily),
    };

    // Synthesize the family-correct descriptor (selectors + bases + density-derived flash page). Its
    // adv_timers/adc_count carry the family DEFAULT for now.
    let mut desc = synthesize(detected.family, detected.flash_kib);

    // 2. MEASURE the per-instance advanced-timer and ADC counts (the family constant is wrong in both
    //    directions on real parts) and write them over the family default.
    let counts = probe::measure_counts();
    desc.adv_timers = counts.adv_timers;
    desc.adc_count = counts.adc_count;

    // 3. A part with a SECOND advanced timer (TIM8 = TIMER7, the F10x high-density parts) carries its
    //    base so the hot path resolves it like any peripheral; one-advanced-timer parts leave it
    //    absent, so a request for it fails loud (MissingBase) instead of being faked. The base is
    //    fixed by the family at the APB2 advanced-timer window (TIM8 = 0x4001_3400 on the F10x).
    if desc.adv_timers >= 2 {
        desc.addrs.set(PeriphLabel::Timer7, TIMER7_BASE);
    }
    // Likewise a part with a SECOND ADC (the F10x dual-ADC parts) carries ADC1's base, so the dual
    // capability resolves; single-ADC parts (the F1x0 baseline) leave it absent.
    if desc.adc_count >= 2 {
        desc.addrs.set(PeriphLabel::Adc1, ADC1_BASE);
    }

    Ok(Chip::from_descriptor(desc))
}

// --- host tests (synthesis + the F10x density branch) -----------------------------------------
//
// The bus-fault probe itself is NOT tested here (no fault on a mock/emulated reserved read); it is
// validated only by the bench firmware. These tests cover the deterministic synthesis logic.
#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;

    #[test]
    fn synthesize_f1x0_equals_descriptor_f130() {
        // flash_kib is ignored for F1x0 (constant K1); try a value that WOULD trip K2 on F10x.
        assert_eq!(synthesize(Family::F1x0, 256), descriptor_f130());
        assert_eq!(synthesize(Family::F1x0, 64), descriptor_f130());
        assert_eq!(synthesize(Family::F1x0, 64).flash_page, PageSize::K1);
    }

    #[test]
    fn synthesize_f10x_64kib_equals_descriptor_f103() {
        // The bench F103 is a 64 KiB medium-density part: synthesize(F10x, 64) == descriptor_f103().
        assert_eq!(synthesize(Family::F10x, 64), descriptor_f103());
    }

    #[test]
    fn f10x_density_branch_picks_page_size() {
        // <= 128 KiB => K1 (medium density); > 128 KiB => K2 (high/extra density).
        assert_eq!(synthesize(Family::F10x, 64).flash_page, PageSize::K1);
        assert_eq!(synthesize(Family::F10x, 128).flash_page, PageSize::K1);
        assert_eq!(synthesize(Family::F10x, 129).flash_page, PageSize::K2);
        assert_eq!(synthesize(Family::F10x, 256).flash_page, PageSize::K2);
        assert_eq!(synthesize(Family::F10x, 512).flash_page, PageSize::K2);
    }

    #[test]
    fn synthesized_descriptors_pass_check_ranges() {
        // The synthesized descriptor passes the GPIO/RCU selector-vs-address invariant by
        // construction (it reuses the family-correct bases).
        for d in [
            synthesize(Family::F1x0, 64),
            synthesize(Family::F10x, 64),
            synthesize(Family::F10x, 256),
        ] {
            assert!(d.addrs.check_ranges(d.gpio, d.clock).is_ok());
        }
    }

    #[test]
    fn synthesized_f10x_selects_apb_bases_and_f130_selects_ahb() {
        // The probe-proven selectors: F10x GPIOA on APB2 at 0x4001_0800, F1x0 GPIOA on AHB at
        // 0x4800_0000. This is the family discriminator made into a descriptor.
        let f103 = synthesize(Family::F10x, 64);
        assert_eq!(f103.addrs.get(PeriphLabel::Gpioa), Some(0x4001_0800));
        assert_eq!(f103.gpio, GpioPath::ApbCrlCrh);
        assert_eq!(f103.clock, ClockPath::F10xRcc);

        let f130 = synthesize(Family::F1x0, 64);
        assert_eq!(f130.addrs.get(PeriphLabel::Gpioa), Some(0x4800_0000));
        assert_eq!(f130.gpio, GpioPath::AhbCtlAfsel);
        assert_eq!(f130.clock, ClockPath::F1x0Rcu);
    }
}
