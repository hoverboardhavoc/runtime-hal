//! Runtime heuristic detection: the ONLY way the HAL learns its chip identity.
//!
//! runtime-hal boots as ONE binary on any supported part. It does not read a chip descriptor from
//! flash and it is not built per-chip; instead it PROBES the silicon to work out what the MCU is and
//! what it can do, then synthesizes the [`McuDescriptor`] the rest of the HAL is built on. Three
//! heuristics, in order:
//!
//! 1. **Family discriminator + peripheral-presence measurement** ([`crate::detect::probe::run`]): the
//!    two families put GPIOA at different bases on different buses (F10x APB at `0x4001_0800`, F1x0 AHB
//!    at `0x4800_0000`). Deliberately read the reserved-region GPIO base of the wrong family and it
//!    bus-faults; a clean read confirms the family, which fixes all four register-model selectors and
//!    the base-address table. With the family known, `run` then MEASURES the advanced-timer and ADC
//!    INSTANCE counts (a benign scratch write-back per candidate; the bench found a family constant
//!    wrong in BOTH directions, see `bench/peripheral-probe-2026-06-18.md`) so the returned
//!    [`crate::detect::probe::Detected`] already carries the per-instance counts, not a family default.
//! 2. **Flash density**: read `FLASH_DENSITY[15:0]` at `0x1FFF_F7E0` for the F10x page-size input.
//!
//! [`crate::detect::probe::run`] gathers ALL the silicon observations up front into one
//! fully-populated [`crate::detect::probe::Detected`]; [`detect_chip`] then hands it to the single
//! [`synthesize`] constructor and returns the [`Chip`], or fails loud ([`DetectError::NoFamily`]) if
//! neither family matched.
//!
//! # The per-family constant facts live HERE
//!
//! `family_model` holds the truly-CONSTANT per-family facts: the four register-model selectors
//! (`gpio`/`clock`/`adc`/`irq`) and the base [`AddrTable`] shared by every part of that family.
//! [`synthesize`] combines that constant model with the measured/derived inputs in
//! [`crate::detect::probe::Detected`] (the per-instance counts and the flash-density-derived page
//! size, plus the count-conditional `Timer7`/`Adc1` bases) into a fully-resolved [`McuDescriptor`] in
//! ONE expression, no placeholders and no post-construction mutation (DECISIONS.md #11).
//! [`descriptor_f103`] / [`descriptor_f130`] are the fully-resolved descriptors for the two bench
//! reference parts, built through that same `synthesize` path.
//!
//! # What is host-testable and what is not
//!
//! The deterministic logic is host-tested: the [`synthesize`] construction and the F10x density ->
//! page-size branch. The bus-fault PROBE itself ([`crate::detect::probe::run`]) is NOT
//! host-testable: the `mock` register backend and emulators raise NO bus fault on a reserved read, so
//! the probe would simply read zero instead of faulting. The probe ordering, the fault-catch, the
//! PC-fixup, and the positive/negative interpretation are validated ONLY on hardware by the
//! `bench-fw-detect` and `bench-fw-probe` firmware.

use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::DetectError;

use probe::Detected;

pub mod probe;

pub mod bringup;

/// The MCU family the probe resolves. A single family determination fixes all four register-model
/// selectors and the base-address table (the `family_model` constant facts); the per-instance
/// timer/ADC counts are MEASURED separately by the probe, not inferred from the family.
///
/// The detection-internal discriminator that drives descriptor synthesis. Application code never
/// needs this (the silicon-purity principle: the HAL hands back a [`Chip`] / capability fruits, and
/// an app derives any family-shaped fact from those, e.g. `chip.clock()`).
///
/// VISIBILITY: this is a detection internal. It is NOT reachable from outside the crate in the
/// default build (the silicon-purity public API never names a chip family). It is re-exported at the
/// crate root ONLY behind the `detect-internals` Cargo feature, for the in-tree detection-acceptance
/// bench firmware (`bench-fw/detect`, `bench-fw/probe`), which is THE on-silicon test of the
/// detection path and must name the family it resolved to record the probe outcome for the human
/// reader. The `synth` inner module holds the definition; the cfg'd re-exports below set the
/// reachable visibility (Rust cannot cfg the `pub` / `pub(crate)` keyword directly).
#[cfg(feature = "detect-internals")]
pub use synth::{family_capability, synthesize, Family};
#[cfg(not(feature = "detect-internals"))]
#[allow(unused_imports)]
pub(crate) use synth::{family_capability, synthesize, Family};

/// The family discriminator + the single `Detected` -> descriptor constructor. Defined in a child
/// module so the re-export above can set the family-internal visibility per the `detect-internals`
/// feature without the items being reachable through the (public) `detect` module by default.
mod synth {
    use super::{family_model, Detected, McuDescriptor, PageSize, PeriphLabel};
    use super::{ADC1_BASE, ADC2_BASE, F10X_K2_THRESHOLD_KIB, TIMER7_BASE};

    /// The MCU family the probe resolves. A single family determination fixes all four
    /// register-model selectors and the base-address table (the constant `family_model` facts).
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

    /// Build the fully-resolved chip [`McuDescriptor`] from a single fully-populated
    /// [`Detected`] (every silicon observation gathered up front) in ONE struct-literal expression,
    /// no `mut` and no post-construction mutation (DECISIONS.md #11).
    ///
    /// The constant per-family facts (the four selectors + the base `AddrTable`) come from
    /// `family_model`; the per-part inputs all come from `detected`:
    /// - **`flash_page`**: F1x0 is always `K1` (1 KiB pages); F10x is density-dependent
    ///   (`flash_kib > 128` => `K2`, else `K1`).
    /// - **`adv_timers` / `adc_count`**: the MEASURED per-instance counts (the bench proved a family
    ///   constant wrong in both directions, so these are measured, never a family default).
    /// - **the count-conditional bases**: a second advanced timer (`Timer7`) carries its base only
    ///   when `adv_timers >= 2`, and a second ADC (`Adc1`) only when `adc_count >= 2`, applied through
    ///   the non-mutating [`crate::addr::AddrTable::with`] builder so a part without them resolves
    ///   the label as [`crate::error::DescriptorError::MissingBase`] rather than a faked base.
    ///
    /// The result passes `addrs.check_ranges(gpio, clock)` by construction (the family-correct bases).
    /// Reachable outside the crate ONLY behind the `detect-internals` feature, for the in-tree
    /// detection acceptance firmware.
    pub fn synthesize(detected: &Detected) -> McuDescriptor {
        let model = family_model(detected.family);
        McuDescriptor {
            gpio: model.gpio,
            clock: model.clock,
            adc: model.adc,
            irq: model.irq,
            addrs: model
                .addrs
                .with(detected.adv_timers >= 2, PeriphLabel::Timer7, TIMER7_BASE)
                .with(detected.adc_count >= 2, PeriphLabel::Adc1, ADC1_BASE)
                .with(detected.adc_count >= 3, PeriphLabel::Adc2, ADC2_BASE),
            flash_page: flash_page_for(detected.family, detected.flash_kib),
            // Retain the raw density (KiB) as the flash extent the FMC driver bounds writes against
            // (`Chip::flash_size_bytes` = `flash_kib * 1024`); a pure read at detect, never written.
            flash_kib: detected.flash_kib,
            adv_timers: detected.adv_timers,
            adc_count: detected.adc_count,
        }
    }

    /// The family's per-instance CAPABILITY: `(max_adv_timers, max_adcs)`, the most advanced timers /
    /// ADCs a part of `family` can carry (F10x: 2 / 2; F1x0: 1 / 1). A genuine constant family fact
    /// (read straight from `family_model`), NOT a per-part count, the actual count is MEASURED per
    /// part. The `bench-fw-probe` validator records this as the baseline its measurement is compared
    /// against. Reachable outside the crate ONLY behind the `detect-internals` feature (the only
    /// consumer; dead code without it).
    #[cfg_attr(not(feature = "detect-internals"), allow(dead_code))]
    pub fn family_capability(family: Family) -> (u8, u8) {
        let model = family_model(family);
        (model.max_adv_timers, model.max_adcs)
    }

    /// The `flash_page` for a part: F1x0 is always `K1`; F10x is density-dependent
    /// (`flash_kib > F10X_K2_THRESHOLD_KIB` => `K2`, else `K1`).
    const fn flash_page_for(family: Family, flash_kib: u16) -> PageSize {
        match family {
            Family::F1x0 => PageSize::K1,
            Family::F10x if flash_kib > F10X_K2_THRESHOLD_KIB => PageSize::K2,
            Family::F10x => PageSize::K1,
        }
    }
}

/// TIM8 (TIMER7) base on the F10x high-density parts (APB2 advanced-timer window). Carried by
/// [`synthesize`] only when a second advanced timer is measured, so a part without it resolves
/// `Timer7` as [`crate::error::DescriptorError::MissingBase`].
const TIMER7_BASE: u32 = 0x4001_3400;

/// ADC1 base on the F10x dual-ADC parts (APB2 ADC window, ADC0 + 0x400). Carried by [`synthesize`]
/// only when a second ADC is measured, so single-ADC parts resolve `Adc1` as `MissingBase`.
const ADC1_BASE: u32 = 0x4001_2800;

/// ADC2 base on the F10x high-density parts (SPL `gd32f10x_adc.h:46`: `ADC2 = ADC_BASE + 0x1800` =
/// `0x4001_3C00` - NOT contiguous with ADC1; it sits past the APB2 timer/USART block). Carried by
/// [`synthesize`] only when a THIRD ADC is measured (the probe's `ADC_BASES[2]` scratch test, e.g.
/// the 12-FET's GD32F103RC), so every other part resolves `Adc2` as `MissingBase`. Data only; no
/// driver work rides on it.
const ADC2_BASE: u32 = 0x4001_3C00;

// --- the constant per-family facts (the register-model selectors + base-address table) --------
//
// These hold ONLY the truly-constant facts shared by every part of a family: the four register-model
// selectors and the base AddrTable. The per-part facts (flash page, the MEASURED timer/ADC counts,
// and the count-conditional Timer7/Adc1 bases) are NOT here; they are inputs `synthesize` folds in
// from the probe's `Detected`. There is no placeholder field anywhere in this model.

/// The constant per-family facts: the four register-model selectors, the base [`AddrTable`], and the
/// family's per-instance CAPABILITY (the most advanced timers / ADCs a part of this family can carry).
/// Combined with the probe's [`Detected`] inputs by [`synthesize`] into a fully-resolved descriptor.
struct FamilyModel {
    gpio: GpioPath,
    clock: ClockPath,
    adc: AdcPath,
    irq: IrqLayout,
    addrs: AddrTable,
    /// The most advanced timers a part of this family can carry (F10x: up to 2, TIMER0 + TIMER7;
    /// F1x0: 1, TIMER0 only). A genuine family capability, NOT a per-part count: the actual count is
    /// MEASURED per part. The bench `bench-fw-probe` records this as the baseline the measurement is
    /// compared against, read via [`family_capability`] (only that path consumes it, so it is dead
    /// code without the `detect-internals` feature the bench enables).
    #[cfg_attr(not(feature = "detect-internals"), allow(dead_code))]
    max_adv_timers: u8,
    /// The most ADC instances a part of this family can carry (F10x: up to 2, ADC0 + ADC1; F1x0: 1,
    /// ADC0 only). A family capability, not a per-part count (the actual count is MEASURED per part).
    #[cfg_attr(not(feature = "detect-internals"), allow(dead_code))]
    max_adcs: u8,
}

/// The constant register-model selectors + base [`AddrTable`] for `family`. CONSTANT per family: it
/// holds no per-part field (no flash page, no instance counts, no count-conditional base). This is
/// the genuine "F10x vs F1x0" register-layout fact, with no placeholder anywhere.
const fn family_model(family: Family) -> FamilyModel {
    match family {
        Family::F10x => {
            let mut addrs = AddrTable::new();
            // USART0 (ST USART1) on APB2 at 0x4001_3800 on BOTH families (gd32f10x_usart.h:48 /
            // the F1x0 memory map): promoted into the family models (todo A1), killing the
            // with_usart0 copy-and-inject helper. Its default pins (PA9/PA10) are gate pins on the
            // 6-FET boards; carrying the BASE is data, not a bring-up.
            addrs.set(PeriphLabel::Usart0, 0x4001_3800);
            addrs.set(PeriphLabel::Usart1, 0x4000_4400);
            // USART2 (ST USART3) is F10x-only, on APB1 at base 0x4000_4400 + 0x400; F1x0 has no
            // USART2 (its F1x0 arm below carries no base, so it resolves MissingBase(Usart2)).
            addrs.set(PeriphLabel::Usart2, 0x4000_4800);
            // F10x GPIO ports: APB2, GPIOA at 0x4001_0800, 0x400 per-port stride (GPIOC = +0x800,
            // GPIOD = +0xC00, GPIOF = +0x1400). GPIOE (+0x1000) is rarely wired on these boards and
            // is not carried.
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
            FamilyModel {
                gpio: GpioPath::ApbCrlCrh,
                clock: ClockPath::F10xRcc,
                adc: AdcPath::Dual,
                irq: IrqLayout::F10xSeparate,
                addrs,
                max_adv_timers: 2,
                max_adcs: 2,
            }
        }
        Family::F1x0 => {
            let mut addrs = AddrTable::new();
            // USART0 at the same APB2 base as F10x (see the F10x arm's note).
            addrs.set(PeriphLabel::Usart0, 0x4001_3800);
            addrs.set(PeriphLabel::Usart1, 0x4000_4400);
            // F1x0 GPIO ports: AHB, GPIOA at 0x4800_0000, 0x400 per-port stride (GPIOC = +0x800,
            // GPIOD = +0xC00, GPIOF = +0x1400). F1x0 has no port E (the AHBEN PEEN bit is the gap),
            // so GPIOE is never carried.
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
            FamilyModel {
                gpio: GpioPath::AhbCtlAfsel,
                clock: ClockPath::F1x0Rcu,
                adc: AdcPath::Single,
                irq: IrqLayout::F1x0Grouped,
                addrs,
                max_adv_timers: 1,
                max_adcs: 1,
            }
        }
    }
}

// --- the fully-resolved descriptors for the two bench reference parts -------------------------
//
// `descriptor_f103` / `descriptor_f130` are the fully-resolved descriptors for the two bench
// reference parts, built through the same `synthesize` path from an explicit `Detected`. They are NOT
// placeholder templates: every field (including the measured counts and the count-conditional bases)
// is its real value for that specific part. They are `pub` because the host tests and the in-tree
// firmware use them as known-good fixtures.

/// The fully-resolved [`McuDescriptor`] for the bench GD32F103C8 (F10x reference part): the F10x
/// register model + APB GPIO bases, 64 KiB medium-density (`flash_page == K1`), and the MEASURED C8
/// counts `adv_timers == 1` (TIMER0 only) / `adc_count == 2` (so `Adc1`'s base is carried and `Timer7`'s
/// is not). Built through [`synthesize`], so it is self-consistent (no count without its base).
pub fn descriptor_f103() -> McuDescriptor {
    synthesize(&Detected {
        family: Family::F10x,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 2,
    })
}

/// The fully-resolved [`McuDescriptor`] for the bench GD32F130C8 (F1x0 reference part): the F1x0
/// register model + AHB GPIO bases, `flash_page == K1` (F1x0 is always 1 KiB pages), and the MEASURED
/// counts `adv_timers == 1` / `adc_count == 1` (single advanced timer, single ADC; no `Timer7`/`Adc1`
/// base). Built through [`synthesize`].
pub fn descriptor_f130() -> McuDescriptor {
    synthesize(&Detected {
        family: Family::F1x0,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

// --- the flash-density input ------------------------------------------------------------------

/// The flash density register `FLASH_DENSITY` lives at `0x1FFF_F7E0`; `[15:0]` is KiB of flash.
/// LOAD-BEARING only for the F10x `flash_page` decision.
pub const FLASH_DENSITY_ADDR: u32 = 0x1FFF_F7E0;

/// The F10x flash-size threshold (KiB) above which pages are 2 KiB (`K2`). At or below it the part
/// is medium-density with 1 KiB pages (`K1`): "if flash size > 128 KiB ... K2".
pub const F10X_K2_THRESHOLD_KIB: u16 = 128;

// `synthesize` and the `Family` discriminator live in the private `synth` child module above; the
// cfg'd re-exports there set their reachable visibility per the `detect-internals` feature.

// --- the boot-flow entry ----------------------------------------------------------------------

/// Detect the MCU at runtime and return its [`Chip`] context. This is the single function the
/// firmware calls at boot to learn what silicon it is running on.
///
/// It runs the bus-fault-safe ordered GPIO+RCU family probe and peripheral-presence measurement
/// ([`crate::detect::probe::run`]), which returns a fully-populated [`crate::detect::probe::Detected`]
/// (the family, the flash density, and the MEASURED per-instance advanced-timer / ADC counts, all
/// gathered up front), then hands it to the single [`synthesize`] constructor to build the
/// fully-resolved [`McuDescriptor`] and returns the resulting [`Chip`].
///
/// Fail-loud: if neither family matched it returns [`DetectError::NoFamily`] rather than guessing a
/// register layout (the firmware then fails safe: halt on the reset IRC8M clock, outputs untouched).
///
/// The probe runs ONCE (it does not retry or loop), BEFORE any bring-up, on the reset clock (IRC8M),
/// before any production RAM vector table the application installs later. The BusFault handling is
/// fully HAL-owned: [`probe::run`] installs a probe-scoped vector table (BusFault slot ->
/// [`probe::bus_fault_entry`]) and restores `VTOR` before returning, so the application defines NO
/// `#[exception] BusFault`.
pub fn detect_chip() -> Result<Chip, DetectError> {
    // The probe gathers EVERY silicon observation up front (family discriminator + flash density +
    // the MEASURED per-instance counts) into one fully-populated `Detected`; `synthesize` then builds
    // the fully-resolved descriptor from it in one shot. Neither family matched => fail safe, no guess.
    let detected = probe::run().ok_or(DetectError::NoFamily)?;
    Ok(Chip::from_descriptor(synthesize(&detected)))
}

// --- host tests (the single `synthesize` constructor + the F10x density branch) ---------------
//
// The bus-fault probe itself is NOT tested here (no fault on a mock/emulated reserved read); it is
// validated only by the bench firmware. These tests cover the deterministic `synthesize` logic: each
// builds a `Detected` (the probe's output) and checks the resolved descriptor.
#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;

    /// A `Detected` fixture: the probe's fully-populated output for a part.
    fn detected(family: Family, flash_kib: u16, adv_timers: u8, adc_count: u8) -> Detected {
        Detected {
            family,
            flash_kib,
            adv_timers,
            adc_count,
        }
    }

    #[test]
    fn synthesize_f1x0_equals_descriptor_f130() {
        // F1x0 page size is the family constant K1 regardless of flash_kib (256 KiB would trip K2 only
        // on F10x). flash_kib is now RETAINED as the flash extent, so a 256 KiB F1x0 differs from
        // descriptor_f130() in that field alone while staying K1; the 64 KiB bench part matches it whole.
        assert_eq!(
            synthesize(&detected(Family::F1x0, 64, 1, 1)),
            descriptor_f130()
        );
        assert_eq!(
            synthesize(&detected(Family::F1x0, 256, 1, 1)).flash_page,
            PageSize::K1
        );
    }

    #[test]
    fn synthesize_f10x_c8_baseline_equals_descriptor_f103() {
        // The bench F103C8 is a 64 KiB medium-density part measuring adv_timers == 1 / adc_count == 2;
        // synthesize of that Detected == descriptor_f103() (the same construction, by definition).
        assert_eq!(
            synthesize(&detected(Family::F10x, 64, 1, 2)),
            descriptor_f103()
        );
    }

    #[test]
    fn f10x_descriptor_resolves_usart2_base() {
        // GD Usart2 (ST USART3) is F10x-only, on APB1 at 0x4000_4800. The resolved F103 descriptor
        // carries that base.
        assert_eq!(
            descriptor_f103().addrs.resolve(PeriphLabel::Usart2),
            Ok(0x4000_4800)
        );
    }

    #[test]
    fn f1x0_descriptor_has_no_usart2() {
        // F1x0 has no USART2: resolving its base errors (MissingBase) rather than faking one.
        assert_eq!(
            descriptor_f130().addrs.resolve(PeriphLabel::Usart2),
            Err(crate::error::DescriptorError::MissingBase(
                PeriphLabel::Usart2
            ))
        );
    }

    #[test]
    fn f10x_density_branch_picks_page_size() {
        // <= 128 KiB => K1 (medium density); > 128 KiB => K2 (high/extra density). The counts do not
        // affect the page-size branch.
        assert_eq!(
            synthesize(&detected(Family::F10x, 64, 1, 2)).flash_page,
            PageSize::K1
        );
        assert_eq!(
            synthesize(&detected(Family::F10x, 128, 1, 2)).flash_page,
            PageSize::K1
        );
        assert_eq!(
            synthesize(&detected(Family::F10x, 129, 2, 3)).flash_page,
            PageSize::K2
        );
        assert_eq!(
            synthesize(&detected(Family::F10x, 256, 2, 3)).flash_page,
            PageSize::K2
        );
        assert_eq!(
            synthesize(&detected(Family::F10x, 512, 2, 3)).flash_page,
            PageSize::K2
        );
    }

    #[test]
    fn count_conditional_bases_are_present_only_at_two_or_more() {
        // Timer7's base is carried iff adv_timers >= 2; Adc1's iff adc_count >= 2. Below the
        // threshold the label is absent, so a request for it fails loud rather than faking a base.
        let single = synthesize(&detected(Family::F10x, 64, 1, 1));
        assert_eq!(single.addrs.get(PeriphLabel::Timer7), None);
        assert_eq!(single.addrs.get(PeriphLabel::Adc1), None);

        let dual = synthesize(&detected(Family::F10x, 256, 2, 2));
        assert_eq!(dual.addrs.get(PeriphLabel::Timer7), Some(TIMER7_BASE));
        assert_eq!(dual.addrs.get(PeriphLabel::Adc1), Some(ADC1_BASE));

        // The two conditions are independent: two ADCs but a single advanced timer (the C8) carries
        // Adc1 but not Timer7.
        let c8 = synthesize(&detected(Family::F10x, 64, 1, 2));
        assert_eq!(c8.addrs.get(PeriphLabel::Timer7), None);
        assert_eq!(c8.addrs.get(PeriphLabel::Adc1), Some(ADC1_BASE));
    }

    #[test]
    fn synthesize_carries_the_measured_counts() {
        // The resolved descriptor's counts are exactly the measured inputs (no family default).
        let d = synthesize(&detected(Family::F10x, 256, 2, 3));
        assert_eq!(d.adv_timers, 2);
        assert_eq!(d.adc_count, 3);
    }

    #[test]
    fn synthesized_descriptors_pass_check_ranges() {
        // The synthesized descriptor passes the GPIO/RCU selector-vs-address invariant by
        // construction (it reuses the family-correct bases).
        for d in [
            synthesize(&detected(Family::F1x0, 64, 1, 1)),
            synthesize(&detected(Family::F10x, 64, 1, 2)),
            synthesize(&detected(Family::F10x, 256, 2, 2)),
        ] {
            assert!(d.addrs.check_ranges(d.gpio, d.clock).is_ok());
        }
    }

    #[test]
    fn synthesized_f10x_selects_apb_bases_and_f130_selects_ahb() {
        // The probe-proven selectors: F10x GPIOA on APB2 at 0x4001_0800, F1x0 GPIOA on AHB at
        // 0x4800_0000. This is the family discriminator made into a descriptor.
        let f103 = synthesize(&detected(Family::F10x, 64, 1, 2));
        assert_eq!(f103.addrs.get(PeriphLabel::Gpioa), Some(0x4001_0800));
        assert_eq!(f103.gpio, GpioPath::ApbCrlCrh);
        assert_eq!(f103.clock, ClockPath::F10xRcc);

        let f130 = synthesize(&detected(Family::F1x0, 64, 1, 1));
        assert_eq!(f130.addrs.get(PeriphLabel::Gpioa), Some(0x4800_0000));
        assert_eq!(f130.gpio, GpioPath::AhbCtlAfsel);
        assert_eq!(f130.clock, ClockPath::F1x0Rcu);
    }
}
