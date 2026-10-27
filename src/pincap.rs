//! Per-chip pin-capability queries (R-CAP; `specs/pin-capability.md`).
//!
//! The hoverboard board-layout boot validator (its `specs/board-model.md`) checks a persisted
//! pin layout against the detected chip through a `Capabilities` trait seam; these queries are the
//! REAL implementation behind that seam (a thin firmware adapter maps the trait onto them). All of
//! them are **pure**: no register or GPIO access, nothing configured or routed.
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
use crate::error::DescriptorError;
use crate::gpio::UsartAf;

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
/// I2C1 default pair: PB10 (SCL) / PB11 (SDA), on the parts that carry a second I2C. The default
/// mapping on BOTH families (GD32F103xx Datasheet Rev2.14 Table 2-7/2-8: PB10 = `I2C1_SCL`,
/// PB11 = `I2C1_SDA`, no remap field exists for I2C1 on F10x; GD32F130xx Datasheet Rev3.7
/// Table 2-10: PB10 `AF1` = `I2C1_SCL`, PB11 `AF1` = `I2C1_SDA`). The F1x0 AF number is the SAME
/// `AF1` its I2C0 pair PB6/PB7 uses, which is why [`crate::gpio::PinRole::I2cAfOpenDrain`] routes
/// both instances without an instance-dependent branch.
const I2C1_PAIR: (u8, u8) = (0x1A, 0x1B);

/// The USART pin mappings this HAL can route, and how each family reaches them.
///
/// Provenance (primary sources, per mapping):
/// - **PA2/PA3 = USART1.** GD32F130xx Datasheet Rev3.7 Table 2-9: PA2 `AF1` = `USART1_TX`, PA3
///   `AF1` = `USART1_RX`. On F10x it is USART1's default mapping (no AFIO remap). The fleet's
///   silicon-proven inter-board link pair on both families.
/// - **PB6/PB7 = USART0.** GD32F130xx Datasheet Rev3.7 Table 2-10: PB6 `AF0` = `USART0_TX`, PB7
///   `AF0` = `USART0_RX`. **F1x0 only.** On F10x these pins reach USART0 only through the global
///   `AFIO_PCF0.USART0_REMAP` field (GD32F10x User Manual Rev2.6 §7.5.9 bit 2), a remap primitive
///   [`crate::afio`] does not implement and no board exercises, so the F10x answer is `None`, not a
///   hopeful yes.
/// - **PB10/PB11 = USART2.** F10x's USART2 default mapping (no AFIO remap). **F10x only:** the
///   F1x0 has no USART2 at all, and Datasheet Rev3.7 Table 2-10 puts `I2C1_SCL`/`I2C1_SDA` on
///   those pins there instead.
///
/// **PA9/PA10 (USART0's DEFAULT mapping, Datasheet Table 2-9 `AF1`) is deliberately absent from
/// this table on BOTH families.** Those same pins are `TIMER0_CH1`/`TIMER0_CH2` (Table 2-9 `AF2`)
/// = the fleet's high-side FET gates, so [`gate_capable`] answers true and
/// [`check_pin_ownership`] refuses them for any non-gate function, permanently. Listing a mapping
/// that the ownership backstop can never allow to be configured would be a half-state; the pin
/// model and the backstop agree that this pair is not a USART pair here.
const USART_PIN_MAPPINGS: [UsartPinMapping; 3] = [
    UsartPinMapping {
        tx: 0x02, // PA2
        rx: 0x03, // PA3
        usart: PeriphLabel::Usart1,
        af: UsartAf::Af1,
        f10x: true,
        f1x0: true,
    },
    UsartPinMapping {
        tx: 0x16, // PB6
        rx: 0x17, // PB7
        usart: PeriphLabel::Usart0,
        af: UsartAf::Af0,
        // F10x needs AFIO_PCF0.USART0_REMAP, which the HAL does not implement.
        f10x: false,
        f1x0: true,
    },
    UsartPinMapping {
        tx: 0x1A, // PB10
        rx: 0x1B, // PB11
        usart: PeriphLabel::Usart2,
        // Inert: this row is F10x-only and the F10x has no per-pin AF mux, so no AFSEL nibble is
        // ever written for it. `Af1` matches the other default USART mappings rather than adding a
        // family-conditional field for a value no F10x register consumes.
        af: UsartAf::Af1,
        f10x: true,
        // No USART2 on F1x0; PB10/PB11 are the I2C1 pair there (see `I2C1_PAIR`).
        f1x0: false,
    },
];

/// One row of [`USART_PIN_MAPPINGS`]: a (TX, RX) pin pair, the USART instance it drives, the F1x0
/// AF-mux number that routes it, and which families reach it with no remap primitive.
struct UsartPinMapping {
    /// Packed TX pin byte.
    tx: u8,
    /// Packed RX pin byte.
    rx: u8,
    /// The instance this pair drives.
    usart: PeriphLabel,
    /// The F1x0 per-pin `GPIOx_AFSEL` number (inert on F10x, which has no per-pin AF mux).
    af: UsartAf,
    /// Routable on F10x with no AFIO remap field.
    f10x: bool,
    /// Routable on F1x0 through the per-pin AF mux.
    f1x0: bool,
}

/// A resolved USART pin mapping ([`usart_pins`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsartMapping {
    /// The USART instance this (TX, RX) pair drives on the detected chip.
    pub usart: PeriphLabel,
    /// The AF-mux number the configure seam writes for this mapping's pins. Meaningful on F1x0;
    /// inert on F10x, where the CRL/CRH mode/cnf nibble is the whole routing (see [`UsartAf`]).
    pub af: UsartAf,
}

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
    table_has(&TIMER0_CH, pin)
        || table_has(&TIMER0_CHN, pin)
        || (chip.has_advanced_timer(PeriphLabel::Timer7)
            && (table_has(&TIMER7_CH, pin) || table_has(&TIMER7_CHN, pin)))
}

/// Is `pin` one of the three bytes of a channel table? An open-coded comparison, deliberately,
/// and it must stay one: do NOT simplify it back to `table.contains(&pin)`.
///
/// `contains` on a byte slice is not a loop over three bytes. It dispatches through the
/// `SliceContains` specialization into `core::slice::memchr`, whose chunked word-at-a-time
/// search and alignment prologue get inlined at each of the four call sites in
/// [`gate_capable`]. That is the wrong trade for a table of three bytes on a 64 KiB part: the
/// whole scan IS three compares. Measured on the hoverboard firmware image (the consumer that
/// flashes this crate, and the only place the cost is visible, since the HAL alone links
/// nothing): -28 B of release span and -52 B of dev span against the same tree. The semantics
/// are identical byte for byte, because `contains` on a `&[u8]` is defined as
/// `iter().any(|&b| b == pin)`, and [`gate_capable`] is a cold query (board validation and
/// bring-up, never an ISR), so no hot path wants the chunked form back.
#[inline]
fn table_has(table: &[u8; 3], pin: u8) -> bool {
    table[0] == pin || table[1] == pin || table[2] == pin
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
#[cfg_attr(runtime_hal_nightly, optimize(size))]
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
/// carry a second I2C. Direction matters (SCL, SDA): a reversed pair is no instance. Remapped
/// pairs (F10x I2C0_REMAP PB8/PB9, the F1x0 PA0/PA1 and PF6/PF7 alternates) answer `None` until
/// a remap primitive exists.
///
/// Presence of instance 1 is asked of the chip ([`Chip::has_i2c`]), not re-derived here: the
/// density rule and its datasheet citations live once, at the descriptor that resolves the base
/// (`crate::detect`'s `I2C1_MIN_FLASH_KIB`). So this query and the resolvable
/// [`PeriphLabel::I2c1`] label cannot drift: whenever this answers `Some(1)`, `I2c::new` on that
/// label resolves, and whenever it answers `None` the label is `MissingBase` and bring-up fails
/// loud at the seam.
///
/// The index is the form a HAL-free consumer can carry (the hoverboard board validator states its
/// `Capabilities` seam in plain integers); a consumer that goes on to CALL [`crate::i2c::I2c::new`]
/// wants the label instead and should ask [`i2c_instance`] directly rather than mapping the index
/// back itself.
pub fn i2c_pair(chip: &Chip, scl: u8, sda: u8) -> Option<u8> {
    i2c_instance(chip, scl, sda).map(|label| match label {
        PeriphLabel::I2c1 => 1,
        _ => 0,
    })
}

/// The I2C instance a (SCL, SDA) pair drives on this chip, as the [`PeriphLabel`]
/// [`crate::i2c::I2c::new`] takes. The label form of [`i2c_pair`], and its single source of truth:
/// the index answer is derived from this one, so an index and the label a bring-up is handed for
/// the same pin pair cannot disagree.
///
/// This exists because the pair -> instance fact belongs here, in the pin model, and nowhere else.
/// Without it a caller that resolved a bus INDEX from a persisted board layout has to turn that
/// index back into a label with a local `0 => I2c0, 1 => I2c1` match, i.e. a second copy of this
/// table living in the consumer, free to drift the moment a part maps a third instance.
pub fn i2c_instance(chip: &Chip, scl: u8, sda: u8) -> Option<PeriphLabel> {
    if (scl, sda) == I2C0_PAIR {
        return Some(PeriphLabel::I2c0);
    }
    if (scl, sda) == I2C1_PAIR && chip.has_i2c(PeriphLabel::I2c1) {
        return Some(PeriphLabel::I2c1);
    }
    None
}

/// Does the (TX, RX) pair drive a USART instance on this chip, and which, through which AF?
/// (R-CAP query e, the USART sibling of [`i2c_pair`].)
///
/// The single owner of the crate's pin -> USART routing fact. [`crate::usart::Usart::new`] resolves
/// every bring-up through it, so the AF written to `GPIOx_AFSEL` is the datasheet's AF for THAT
/// pin, never a number baked into the pin role: `USART0_TX` is `AF1` at PA9 but `AF0` at PB6
/// (GD32F130xx Datasheet Rev3.7 Tables 2-9 / 2-10), which is exactly the fact a per-role constant
/// cannot express.
///
/// Direction matters (TX, RX): a reversed pair is no mapping. Family-gated per
/// [`USART_PIN_MAPPINGS`] (the F1x0-only PB6/PB7 USART0 mapping, the F10x-only PB10/PB11 USART2
/// one), and the gate-pin pair PA9/PA10 is not a mapping on either family - the module's
/// not-yet-expressible rule and the gate-pin denial, both visible here.
///
/// Answering `Some` is a ROUTING fact, not permission: [`check_pin_ownership`] is still the
/// backstop every configure call goes through.
#[cfg_attr(runtime_hal_nightly, optimize(size))]
pub fn usart_pins(chip: &Chip, tx: u8, rx: u8) -> Option<UsartMapping> {
    let f1x0 = matches!(chip.gpio(), GpioPath::AhbCtlAfsel);
    USART_PIN_MAPPINGS
        .iter()
        .find(|m| m.tx == tx && m.rx == rx && if f1x0 { m.f1x0 } else { m.f10x })
        .map(|m| UsartMapping {
            usart: m.usart,
            af: m.af,
        })
}

// --- the per-configuration ownership backstop (Layer 7) ---------------------------------------

/// What a configure call claims a pin FOR, the question [`check_pin_ownership`] answers.
///
/// Two claims, because the property being defended is one bit: a gate-capable pin drives a
/// MOSFET half-bridge, so it belongs to the advanced timer and to nothing else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinClaim {
    /// An advanced-timer complementary output: a motor gate. Only a gate-capable pin qualifies.
    Gate,
    /// Any other function (GPIO in or out, analog, USART, I2C, SPI, general-timer PWM). A
    /// gate-capable pin is refused.
    NonGate,
}

/// The Layer-7 per-configuration pin-ownership backstop: may `pin` be configured for `claim` on
/// this chip? (`hoverboard-firmware/specs/board-model.md`'s second layer, the one the boot
/// validator does not provide.)
///
/// The boot validator checks the persisted layout ONCE, as a set. This is the independent second
/// layer: every individual configure call re-asks at the HAL boundary, so a bug that bypasses the
/// validator entirely (a hardcoded pin, a mis-decoded field, a call made before the layout is
/// read) still cannot route a gate-capable pin to a non-gate function. Two layers only help if
/// they fail independently, which is why this one derives its answer from [`gate_capable`] and
/// the descriptor alone, with no input from the layout the validator checked.
///
/// The refusal is symmetric, and deliberately so: a gate claim on a pin that is NOT gate-capable
/// is equally a bug (it would write a complementary-output AF onto a pin no bridge is wired to),
/// and catching it costs nothing.
///
/// Density-dependence is the point, not an artifact: on a part with a second advanced timer
/// PB0/PB1 are TIMER7 complementary outputs, so the same pins that are legitimate ADC sense
/// inputs on a medium-density part are refused for analog use on a high-density one.
pub fn check_pin_ownership(chip: &Chip, pin: u8, claim: PinClaim) -> Result<(), DescriptorError> {
    if gate_capable(chip, pin) == matches!(claim, PinClaim::Gate) {
        Ok(())
    } else {
        Err(DescriptorError::PinOwnership)
    }
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
    fn the_ownership_backstop_refuses_a_gate_pin_for_every_other_function() {
        // The Layer-7 claim check, both directions, on both bench parts. The six TIMER0 gate
        // pins are gate-only; the pins the fleet actually configures for other functions (the
        // USART link pair PA2/PA3, the I2C pair PB6/PB7, VBATT PA4, the halls PC13/PA1/PC14,
        // the LEDs PB3/PA15/PB4, the buzzer PB9) are refused as gates.
        for chip in [f103c8(), f130c8()] {
            for gate in [0x08u8, 0x09, 0x0A, 0x1D, 0x1E, 0x1F] {
                assert_eq!(check_pin_ownership(&chip, gate, PinClaim::Gate), Ok(()));
                assert_eq!(
                    check_pin_ownership(&chip, gate, PinClaim::NonGate),
                    Err(DescriptorError::PinOwnership),
                    "gate pin {gate:#04x} claimed for a non-gate function"
                );
            }
            for other in [
                0x02u8, 0x03, 0x16, 0x17, 0x04, 0x2D, 0x01, 0x2E, 0x13, 0x0F, 0x14, 0x19,
            ] {
                assert_eq!(check_pin_ownership(&chip, other, PinClaim::NonGate), Ok(()));
                assert_eq!(
                    check_pin_ownership(&chip, other, PinClaim::Gate),
                    Err(DescriptorError::PinOwnership),
                    "non-gate pin {other:#04x} claimed as a gate"
                );
            }
        }
    }

    #[test]
    fn the_ownership_backstop_follows_the_density_gated_timer7_set() {
        // PB0 is a 6-FET phase-current sense pin (ADC channel 8) on a medium-density part and a
        // TIMER7 complementary gate on a high-density one, so the same claim flips verdict with
        // the measured second advanced timer. This is the case `motor-integration.md` records:
        // the 6-FET sense pins are gate-capable on the 12-FET part and correctly refused there.
        const PB0: u8 = 0x10;
        assert_eq!(
            check_pin_ownership(&f103c8(), PB0, PinClaim::NonGate),
            Ok(())
        );
        assert_eq!(
            check_pin_ownership(&f103rc(), PB0, PinClaim::NonGate),
            Err(DescriptorError::PinOwnership)
        );
        assert_eq!(check_pin_ownership(&f103rc(), PB0, PinClaim::Gate), Ok(()));
        assert_eq!(
            check_pin_ownership(&f103c8(), PB0, PinClaim::Gate),
            Err(DescriptorError::PinOwnership)
        );
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

    /// The label answer is the index answer's source, so the two can never disagree, and the
    /// label it hands a bring-up is the one `I2c::new` takes. Checked over the same vectors the
    /// index test uses plus the sub-floor part where instance 1 is absent.
    #[test]
    fn i2c_instance_is_the_label_form_of_the_same_answer() {
        let below = Chip::from_descriptor(synthesize(&Detected {
            family: Family::F10x,
            flash_kib: 32,
            adv_timers: 1,
            adc_count: 2,
        }));
        for chip in [f103c8(), f130c8(), f103rc(), below] {
            for (scl, sda) in [
                I2C0_PAIR,
                I2C1_PAIR,
                (0x17, 0x16), // reversed
                (0x05, 0x06), // not an I2C pair
                (0x18, 0x19), // PB8/PB9 remap: not expressible
            ] {
                let label = i2c_instance(&chip, scl, sda);
                let index = i2c_pair(&chip, scl, sda);
                // Same verdict, and where there IS one the two forms name the same instance.
                assert_eq!(label.is_some(), index.is_some());
                match (label, index) {
                    (Some(PeriphLabel::I2c0), Some(0)) | (Some(PeriphLabel::I2c1), Some(1)) => {}
                    (None, None) => {}
                    other => panic!("index/label disagree for ({scl:#04x},{sda:#04x}): {other:?}"),
                }
                // And a named label is one `I2c::new` can actually resolve a base for.
                if let Some(l) = label {
                    assert!(chip.base(l).is_ok(), "{l:?} named but its base is absent");
                }
            }
        }
    }

    #[test]
    fn usart_pins_resolve_per_family_with_the_datasheet_af() {
        // PA2/PA3 = USART1 on BOTH families, at AF1 (GD32F130xx Datasheet Rev3.7 Table 2-9).
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert_eq!(
                usart_pins(&chip, 0x02, 0x03),
                Some(UsartMapping {
                    usart: PeriphLabel::Usart1,
                    af: UsartAf::Af1
                }),
                "PA2/PA3 = USART1"
            );
            // Direction matters, exactly like `i2c_pair`.
            assert_eq!(usart_pins(&chip, 0x03, 0x02), None, "reversed pair");
            assert_eq!(usart_pins(&chip, 0x05, 0x06), None, "not a USART pair");
        }

        // PB6/PB7 = USART0 at **AF0** on F1x0 (Datasheet Rev3.7 Table 2-10: PB6 AF0 = USART0_TX,
        // PB7 AF0 = USART0_RX). This is the whole point of the mapping-owned AF: the SAME function
        // sits on AF1 at PA9 and AF0 at PB6, so no per-role constant can express both.
        assert_eq!(
            usart_pins(&f130c8(), 0x16, 0x17),
            Some(UsartMapping {
                usart: PeriphLabel::Usart0,
                af: UsartAf::Af0
            }),
            "PB6/PB7 = USART0 at AF0 on F1x0"
        );
        // On F10x the same pair needs the AFIO USART0_REMAP field the HAL does not implement.
        for chip in [f103c8(), f103rc()] {
            assert_eq!(
                usart_pins(&chip, 0x16, 0x17),
                None,
                "PB6/PB7 USART0 needs an AFIO remap on F10x: not expressible"
            );
        }

        // PB10/PB11 = USART2, F10x-only: the F1x0 has no USART2 (those pins are its I2C1 pair).
        for chip in [f103c8(), f103rc()] {
            assert_eq!(
                usart_pins(&chip, 0x1A, 0x1B).map(|m| m.usart),
                Some(PeriphLabel::Usart2)
            );
        }
        assert_eq!(usart_pins(&f130c8(), 0x1A, 0x1B), None, "no USART2 on F1x0");
    }

    #[test]
    fn usart0_default_pa9_pa10_is_no_mapping_and_stays_gate_denied() {
        // The gate-pin denial, from both layers at once. PA9/PA10 IS USART0's default mapping on
        // silicon (Datasheet Table 2-9, AF1), but those pins are TIMER0_CH1/CH2 = the fleet's
        // high-side FET gates, so: the pin model refuses to name them a USART pair at all, AND the
        // ownership backstop refuses them for any non-gate claim. Neither layer may soften.
        for chip in [f103c8(), f130c8(), f103rc()] {
            assert_eq!(
                usart_pins(&chip, 0x09, 0x0A),
                None,
                "PA9/PA10 is never a routable USART pair here"
            );
            for gate in [0x09u8, 0x0A] {
                assert!(gate_capable(&chip, gate));
                assert_eq!(
                    check_pin_ownership(&chip, gate, PinClaim::NonGate),
                    Err(DescriptorError::PinOwnership),
                    "gate pin {gate:#04x} stays denied for USART use"
                );
            }
        }
    }

    #[test]
    fn pb6_pb7_answers_both_i2c0_and_usart0_because_the_silicon_muxes_both() {
        // PB6/PB7 is a genuine AF-mux conflict on F1x0: I2C0 at AF1, USART0 at AF0 (Datasheet
        // Rev3.7 Table 2-10). Both queries answer, on their own AF, and neither invalidates the
        // other: WHICH function a board uses is the board layout's choice, not the chip's.
        let f130 = f130c8();
        assert_eq!(
            i2c_pair(&f130, 0x16, 0x17),
            Some(0),
            "PB6/PB7 = I2C0 at AF1"
        );
        assert_eq!(
            usart_pins(&f130, 0x16, 0x17).map(|m| m.af),
            Some(UsartAf::Af0),
            "PB6/PB7 = USART0 at AF0"
        );
        // Neither is gate-capable, so the ownership backstop permits either.
        for p in [0x16u8, 0x17] {
            assert_eq!(check_pin_ownership(&f130, p, PinClaim::NonGate), Ok(()));
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

    /// The capability query and the resolvable label must agree on EVERY part: whenever
    /// `i2c_pair` names an instance, that instance's `PeriphLabel` resolves to a base (so
    /// `I2c::new` on it can succeed), and whenever it declines, the label is `MissingBase` (so
    /// bring-up fails loud at the seam instead of driving a peripheral that is not there).
    #[test]
    fn i2c_pair_and_the_resolvable_label_agree_on_every_part() {
        // A 32 KiB F10x and a 32 KiB F1x0 sit below the floor; the three fleet parts sit above it.
        let below_f10x = Chip::from_descriptor(synthesize(&Detected {
            family: Family::F10x,
            flash_kib: 32,
            adv_timers: 1,
            adc_count: 2,
        }));
        let below_f1x0 = Chip::from_descriptor(synthesize(&Detected {
            family: Family::F1x0,
            flash_kib: 32,
            adv_timers: 1,
            adc_count: 1,
        }));
        for chip in [f103c8(), f130c8(), f103rc(), below_f10x, below_f1x0] {
            for (pair, label) in [
                (I2C0_PAIR, PeriphLabel::I2c0),
                (I2C1_PAIR, PeriphLabel::I2c1),
            ] {
                let named = i2c_pair(&chip, pair.0, pair.1).is_some();
                assert_eq!(
                    named,
                    chip.has_i2c(label),
                    "pincap vs the chip capability query for {label:?}"
                );
                assert_eq!(
                    named,
                    chip.base(label).is_ok(),
                    "pincap named {label:?} but its base does not resolve (or vice-versa)"
                );
            }
        }
    }

    /// The pin-ownership backstop still governs the second instance's pins: PB10/PB11 are not
    /// gate-capable on any part, so they are accepted as a non-gate claim (what `I2c::new` makes)
    /// and refused as a gate claim. The gate pins are unaffected by the new instance.
    #[test]
    fn the_ownership_backstop_still_governs_the_i2c1_pins() {
        const PB10: u8 = 0x1A;
        const PB11: u8 = 0x1B;
        for chip in [f103c8(), f130c8(), f103rc()] {
            for pin in [PB10, PB11] {
                assert!(pin_exists(&chip, pin));
                assert!(!gate_capable(&chip, pin));
                assert_eq!(check_pin_ownership(&chip, pin, PinClaim::NonGate), Ok(()));
                assert_eq!(
                    check_pin_ownership(&chip, pin, PinClaim::Gate),
                    Err(DescriptorError::PinOwnership)
                );
            }
            // And a gate pin is still refused for I2C's non-gate claim, on every part: PB13 is
            // TIMER0_CH0N, adjacent to the I2C1 pair on the same port.
            assert_eq!(
                check_pin_ownership(&chip, 0x1D, PinClaim::NonGate),
                Err(DescriptorError::PinOwnership)
            );
        }
    }
}
