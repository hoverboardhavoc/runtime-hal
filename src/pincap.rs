//! Per-chip pin-capability queries (R-CAP; `specs/pin-capability.md`).
//!
//! The hoverboard board-layout boot validator (its `specs/board-model.md`) checks a persisted
//! pin layout against the detected chip through a `Capabilities` trait seam; these five queries
//! are the REAL implementation behind that seam (a thin firmware adapter maps the trait onto
//! them). All five are **pure**: no register or GPIO access, nothing configured or routed.
//!
//! The `supports_rx` precedent applies ([`crate::usart_rx::supports_rx`]): every answer is
//! derived from the HAL model - the descriptor's register-model selectors, its MEASURED
//! per-instance counts, and the AF/bonding data owned by THIS module - never from a
//! consumer-side table, and a capability the HAL cannot yet express (a remapped gate set or
//! I2C pair) answers `None` rather than a hopeful yes.
//!
//! Pins are packed logical bytes (`(port << 4) | pin`, port A = 0, B = 1, C = 2, D = 3, F = 5),
//! the encoding [`crate::Chip::input_group`] / `route_*_pin` already take. A byte outside that
//! encoding simply does not exist ([`pin_exists`] answers `false`, the others follow).
//!
//! # The package story (why `pin_exists` is derived, not detected)
//!
//! The descriptor knows the FAMILY and the measured counts, not the package: bonding is
//! invisible to the core (a port's registers exist whether or not its pins are bonded), so
//! there is nothing to probe. The bonding is therefore derived per the spec's package story:
//! F10x parts that measured a second advanced timer are high-density, which starts at LQFP64
//! (GD32F103xx Datasheet Rev2.14 Tables 2-1/2-2: no 48-pin part carries TIMER7), so they get
//! the LQFP64 bonding; every other F10x part and every F1x0 part gets its family's LQFP48
//! bonding (the fleet floor), which is conservative in the REFUSE direction for the larger
//! non-fleet packages.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::GpioPath;

// --- the owned capability data (packed pin bytes; provenance in specs/pin-capability.md) ------

/// TIMER0 main channels CH0/CH1/CH2 = PA8/PA9/PA10 (default mapping, both families; the
/// silicon-proven 6-FET high-side set).
const TIMER0_CH: [u8; 3] = [0x08, 0x09, 0x0A];
/// TIMER0 complementary channels CH0N/CH1N/CH2N = PB13/PB14/PB15 (the 6-FET low-side set).
const TIMER0_CHN: [u8; 3] = [0x1D, 0x1E, 0x1F];
/// TIMER7 main channels CH0/CH1/CH2 = PC6/PC7/PC8 (F10x high-density; the 12-FET second motor).
const TIMER7_CH: [u8; 3] = [0x26, 0x27, 0x28];
/// TIMER7 complementary channels CH0N/CH1N/CH2N = PA7/PB0/PB1.
const TIMER7_CHN: [u8; 3] = [0x07, 0x10, 0x11];

/// I2C0 default pair: PB6 (SCL) / PB7 (SDA), every part of both families.
const I2C0_PAIR: (u8, u8) = (0x16, 0x17);
/// I2C1 default pair: PB10 (SCL) / PB11 (SDA), on the parts that carry a second I2C (see
/// [`i2c_pair`]'s flash gate).
const I2C1_PAIR: (u8, u8) = (0x1A, 0x1B);

/// Both datasheets' feature tables carry a second I2C exactly on the >= 64 KiB flash variants
/// (GD32F103xx Rev2.14 Tables 2-1/2-2; GD32F130xx Rev3.7 Table 2-1), so I2C1 presence is gated
/// on the descriptor's measured density.
const I2C1_MIN_FLASH_KIB: u16 = 64;

/// Whether this F10x part gets the LQFP64 bonding: a measured second advanced timer means a
/// high-density part, and GD32F103 high density starts at LQFP64 (the spec's package story).
/// F1x0 never takes this branch (its parts all measure one advanced timer, and its bonding is
/// the family's LQFP48 map regardless).
#[inline]
fn f10x_lqfp64_bonding(chip: &Chip) -> bool {
    chip.adv_timers() >= 2
}

/// Does `pin` exist (is it bonded) on the detected part? (R-CAP query a.)
///
/// Answers the BONDING fact for the derived package (module docs): the family from the
/// descriptor's GPIO register-model selector, the F10x package tier from the measured
/// advanced-timer count. Bonding is not routability: the OSC-remap pins (PD0/PD1 on F10x-48)
/// exist here even though driving them as GPIO would additionally need a remap primitive; that
/// is the configure seam's concern, not the existence question.
pub fn pin_exists(chip: &Chip, pin: u8) -> bool {
    let (port, n) = (pin >> 4, pin & 0x0F);
    match chip.gpio() {
        // F10x: LQFP48 bonds PA/PB full + PC13-15 + PD0-1 (37 GPIOs); LQFP64 (high density,
        // measured by the second advanced timer) bonds PC full + PD0-2 (51). No port F below
        // LQFP144, so F answers false.
        GpioPath::ApbCrlCrh => match port {
            0 | 1 => true,
            2 => f10x_lqfp64_bonding(chip) || n >= 13,
            3 => n <= if f10x_lqfp64_bonding(chip) { 2 } else { 1 },
            _ => false,
        },
        // F1x0: the LQFP48 bonding always (the fleet floor; no count separates its packages):
        // PA/PB full + PC13-15 + PF0/PF1 + PF6/PF7 (39 GPIOs; the datasheet pinout DOES bond
        // PF6/PF7 on LQFP48). No port D below LQFP64.
        GpioPath::AhbCtlAfsel => match port {
            0 | 1 => true,
            2 => n >= 13,
            5 => matches!(n, 0 | 1 | 6 | 7),
            _ => false,
        },
    }
}

/// Is `pin` gate-capable, i.e. an advanced-timer main or complementary channel? (R-CAP query b,
/// the board validator's denylist input: gate-capable pins refuse non-gate functions.)
///
/// Defined as membership in a known-valid complementary assignment and computed from the SAME
/// tables [`gate_set`] matches, so the two queries cannot drift: the TIMER0 set on every part,
/// plus the TIMER7 set exactly where the descriptor carries a measured second advanced timer.
pub fn gate_capable(chip: &Chip, pin: u8) -> bool {
    TIMER0_CH.contains(&pin)
        || TIMER0_CHN.contains(&pin)
        || (chip.has_advanced_timer(PeriphLabel::Timer7)
            && (TIMER7_CH.contains(&pin) || TIMER7_CHN.contains(&pin)))
}

/// Do the six pins form a known-valid complementary assignment of ONE advanced timer on this
/// chip? Returns the named timer ([`PeriphLabel::Timer0`] / [`PeriphLabel::Timer7`]). (R-CAP
/// query b's set half.)
///
/// An EXACT positional match of the default-mapping tables in channel order (`hi[i]` = CHi,
/// `lo[i]` = CHiN, so each high-side pin pairs its own complementary output): a scrambled,
/// rotated, or hi/lo-swapped set is not a valid dead-time-paired bridge assignment and answers
/// `None`. Remapped gate sets answer `None` until a remap primitive exists (the `supports_rx`
/// not-yet-expressible rule); no fleet board uses one. The TIMER7 arm exists only where the
/// descriptor carries the measured second advanced timer.
pub fn gate_set(chip: &Chip, hi: [u8; 3], lo: [u8; 3]) -> Option<PeriphLabel> {
    if hi == TIMER0_CH && lo == TIMER0_CHN {
        return Some(PeriphLabel::Timer0);
    }
    if chip.has_advanced_timer(PeriphLabel::Timer7) && hi == TIMER7_CH && lo == TIMER7_CHN {
        return Some(PeriphLabel::Timer7);
    }
    None
}

/// The ADC channel behind an analog-capable pin, if any. (R-CAP query c.)
///
/// The F1-class external-channel map, identical on both families: PA0-7 = channels 0-7,
/// PB0-1 = 8-9, PC0-5 = 10-15, gated on [`pin_exists`] (so the PC channels answer only on the
/// LQFP64-bonding parts, matching the datasheets' 10-vs-16 external-channel counts). Internal
/// channels (16/17) are not pin-backed ([`crate::adc::is_internal_channel`] owns them).
pub fn adc_channel(chip: &Chip, pin: u8) -> Option<u8> {
    if !pin_exists(chip, pin) {
        return None;
    }
    match (pin >> 4, pin & 0x0F) {
        (0, n) if n <= 7 => Some(n),
        (1, n) if n <= 1 => Some(8 + n),
        (2, n) if n <= 5 => Some(10 + n),
        _ => None,
    }
}

/// Does the (SCL, SDA) pair form a hardware-I2C instance on this chip, and which? (R-CAP
/// query d, the bus-kind derivation the hoverboard `specs/imu.md` assumes.)
///
/// Returns the instance index in the **GD zero-based numbering** (never the ST 1-based names):
/// PB6/PB7 = I2C0 = 0 on every part of both families; PB10/PB11 = I2C1 = 1 on the parts that
/// carry a second I2C (the >= 64 KiB flash variants of both families, read from the
/// descriptor's measured density - every fleet part qualifies). Direction matters (SCL, SDA):
/// a reversed pair is no instance. Remapped pairs (F10x I2C0_REMAP PB8/PB9, the F1x0 PF6/PF7
/// alternates) answer `None` until a remap primitive exists.
///
/// The index is DATA: no `PeriphLabel::I2c1` label exists (one label = one resolvable base,
/// DECISIONS #14) until an I2C1 bring-up consumer arrives; a plan naming instance 1 fails loud
/// at the bring-up seam, never silently.
pub fn i2c_pair(chip: &Chip, scl: u8, sda: u8) -> Option<u8> {
    if (scl, sda) == I2C0_PAIR {
        return Some(0);
    }
    if (scl, sda) == I2C1_PAIR && chip.descriptor().flash_kib >= I2C1_MIN_FLASH_KIB {
        return Some(1);
    }
    None
}

// --- host tests (pure logic; the fleet descriptors + the synthesized high-density part) -------

#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;
    use crate::detect::probe::Detected;
    use crate::detect::{descriptor_f103, descriptor_f130, synthesize, Family};

    /// The bench GD32F103C8 (F10x, LQFP48 bonding: one advanced timer measured).
    fn f103c8() -> Chip {
        Chip::from_descriptor(descriptor_f103())
    }

    /// The bench GD32F130C8 (F1x0, LQFP48 bonding).
    fn f130c8() -> Chip {
        Chip::from_descriptor(descriptor_f130())
    }

    /// The 12-FET GD32F103RC shape (F10x high density: 256 KiB, two advanced timers, three
    /// ADCs measured), built through the same `synthesize` path detection uses.
    fn f103rc() -> Chip {
        Chip::from_descriptor(synthesize(&Detected {
            family: Family::F10x,
            flash_kib: 256,
            adv_timers: 2,
            adc_count: 3,
        }))
    }

    /// Every byte the pin encoding can express (ports A..D, F; pins 0..15).
    fn all_encodable_pins() -> impl Iterator<Item = u8> {
        [0u8, 1, 2, 3, 5]
            .into_iter()
            .flat_map(|port| (0u8..16).map(move |n| (port << 4) | n))
    }

    #[test]
    fn bonded_pin_counts_match_the_datasheet_gpio_counts() {
        // The whole bonding model at once, against the feature tables' GPIO counts:
        // F103C8 LQFP48 = 37, F130C8 LQFP48 = 39, F103RC LQFP64 = 51.
        for (chip, want, part) in [
            (f103c8(), 37, "F103C8"),
            (f130c8(), 39, "F130C8"),
            (f103rc(), 51, "F103RC"),
        ] {
            let n = all_encodable_pins()
                .filter(|&p| pin_exists(&chip, p))
                .count();
            assert_eq!(n, want, "{part} bonded-pin count");
        }
    }

    #[test]
    fn pin_exists_family_and_package_vectors() {
        // PD0 is the F10x-48 OSC-remap pin; PF0 its F1x0 counterpart. Each family bonds its own.
        assert!(pin_exists(&f103c8(), 0x30)); // PD0 on F103
        assert!(!pin_exists(&f130c8(), 0x30)); // no port D on F1x0-48
        assert!(pin_exists(&f130c8(), 0x50)); // PF0 on F130
        assert!(!pin_exists(&f103c8(), 0x50)); // no port F on F103
                                               // The F1x0 LQFP48 bonds PF6/PF7 (the datasheet pinout), and nothing between.
        assert!(pin_exists(&f130c8(), 0x56) && pin_exists(&f130c8(), 0x57));
        assert!(!pin_exists(&f130c8(), 0x52) && !pin_exists(&f130c8(), 0x55));
        // 48-pin port C is PC13-15 only; the high-density (LQFP64) part bonds it fully + PD2.
        assert!(pin_exists(&f103c8(), 0x2D) && !pin_exists(&f103c8(), 0x2A));
        assert!(pin_exists(&f103rc(), 0x2A) && pin_exists(&f103rc(), 0x32));
        assert!(!pin_exists(&f103c8(), 0x32) && !pin_exists(&f103rc(), 0x33));
        // Encoding-invalid bytes (port E = 4, ports above F) exist nowhere.
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert!(!pin_exists(&chip, 0x40));
            assert!(!pin_exists(&chip, 0x90));
            assert!(!pin_exists(&chip, 0xFF));
        }
    }

    #[test]
    fn gate_capable_is_the_gate_tables_gated_on_timer7_presence() {
        for chip in [f103c8(), f130c8(), f103rc()] {
            for p in TIMER0_CH.iter().chain(&TIMER0_CHN) {
                assert!(gate_capable(&chip, *p), "TIMER0 pins on every part");
            }
        }
        // The TIMER7 set is gate-capable only where the second advanced timer was measured.
        for p in TIMER7_CH.iter().chain(&TIMER7_CHN) {
            assert!(gate_capable(&f103rc(), *p));
            assert!(!gate_capable(&f103c8(), *p));
            assert!(!gate_capable(&f130c8(), *p));
        }
        // A plain pin is never gate-capable.
        assert!(!gate_capable(&f103rc(), 0x04)); // PA4
    }

    #[test]
    fn gate_set_matches_exactly_and_names_the_timer() {
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert_eq!(
                gate_set(&chip, TIMER0_CH, TIMER0_CHN),
                Some(PeriphLabel::Timer0)
            );
            // Swapped halves / a scrambled slot are not a valid complementary assignment.
            assert_eq!(gate_set(&chip, TIMER0_CHN, TIMER0_CH), None);
            let mut hi = TIMER0_CH;
            hi.swap(0, 2);
            assert_eq!(gate_set(&chip, hi, TIMER0_CHN), None);
        }
        // TIMER7's set only where the descriptor carries it.
        assert_eq!(
            gate_set(&f103rc(), TIMER7_CH, TIMER7_CHN),
            Some(PeriphLabel::Timer7)
        );
        assert_eq!(gate_set(&f103c8(), TIMER7_CH, TIMER7_CHN), None);
        assert_eq!(gate_set(&f130c8(), TIMER7_CH, TIMER7_CHN), None);
    }

    #[test]
    fn adc_channels_follow_the_f1_map_gated_on_bonding() {
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert_eq!(adc_channel(&chip, 0x04), Some(4)); // PA4 (vbatt) = channel 4
            assert_eq!(adc_channel(&chip, 0x10), Some(8)); // PB0 = channel 8
            assert_eq!(adc_channel(&chip, 0x11), Some(9)); // PB1 = channel 9
            assert_eq!(adc_channel(&chip, 0x2D), None); // PC13: no channel behind it
            assert_eq!(adc_channel(&chip, 0x1C), None); // PB12: not analog
        }
        // PC0-5 = channels 10-15 exist only with the LQFP64 bonding (the 10-vs-16 count).
        assert_eq!(adc_channel(&f103rc(), 0x20), Some(10));
        assert_eq!(adc_channel(&f103rc(), 0x25), Some(15));
        assert_eq!(adc_channel(&f103c8(), 0x20), None);
        assert_eq!(adc_channel(&f130c8(), 0x20), None);
    }

    #[test]
    fn i2c_pairs_answer_the_gd_numbering() {
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert_eq!(i2c_pair(&chip, 0x16, 0x17), Some(0), "PB6/PB7 = I2C0");
            assert_eq!(i2c_pair(&chip, 0x1A, 0x1B), Some(1), "PB10/PB11 = I2C1");
            assert_eq!(i2c_pair(&chip, 0x17, 0x16), None, "reversed pair");
            assert_eq!(i2c_pair(&chip, 0x05, 0x06), None, "not an I2C pair");
            assert_eq!(
                i2c_pair(&chip, 0x18, 0x19),
                None,
                "PB8/PB9 remap: not expressible"
            );
        }
    }

    #[test]
    fn i2c1_is_absent_below_the_64k_density_floor() {
        // A GD32F103C6-shaped part (32 KiB, single I2C per the feature table): I2C0 only.
        let c6 = Chip::from_descriptor(synthesize(&Detected {
            family: Family::F10x,
            flash_kib: 32,
            adv_timers: 1,
            adc_count: 2,
        }));
        assert_eq!(i2c_pair(&c6, 0x16, 0x17), Some(0));
        assert_eq!(i2c_pair(&c6, 0x1A, 0x1B), None);
    }
}
