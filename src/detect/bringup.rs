//! The default bring-up the firmware applies after a DETECTED chip (spec section 6).
//!
//! After [`crate::detect::detect_chip`] returns the synthesized chip, the firmware brings up the
//! minimal default: a default clock, GPIO
//! available, and one debug console. These are SILICON facts (same for any board of that family) or
//! clearly-flagged conventions; nothing here drives board hardware (no motor PWM, no buzzer).
//!
//! # The defaults (the spec's recommended open-item choices)
//!
//! - **DF-2 clock: BARE IRC8M, no PLL.** The fallback does NOT call `configure_tree` (no PLL
//!   bring-up that could fail). It stays on the reset IRC8M 8 MHz clock and brings up GPIO + USART0
//!   at 8 MHz. The USART BRR is computed for 115200 at 8 MHz, the same approach the M1 firmware used.
//!   This guarantees the core runs regardless of board wiring (crystal presence is a board fact the
//!   probe cannot determine, so the fallback never selects HXTAL).
//! - **DF-4 console pins: PA9/PA10** for USART0, as a FLAGGED CONVENTION (an assumption, not a
//!   silicon fact). PA9/PA10 is the common GD32/STM32F1 USART0 routing; a console on a guessed pin is
//!   still useful and easily corrected. [`CONSOLE_TX`] / [`CONSOLE_RX`] carry the convention and
//!   [`CONSOLE_PINS_ASSUMED`] documents it so the firmware can log the assumption.
//!
//! # USART0 base
//!
//! The synthesized descriptor (which must stay byte-for-byte equal to the per-family constants in
//! `tests/blobs.rs`) does NOT carry a USART0 base, so this helper supplies the shared APB2 USART0
//! base ([`USART0_BASE`], `0x4001_3800` on both families, `addr::ranges::USART_APB2`) by wrapping a
//! local AUGMENTED copy of the descriptor for the console bring-up only. The detected `Chip` the
//! firmware holds is unchanged.

use crate::clock::{ClockConfig, ClockSource};

// The console bring-up itself (and everything only it uses) is compiled only under the gate; see
// `apply_defaults`'s GATE-PIN GUARD.
#[cfg(any(feature = "mock", feature = "yes-console-on-pa9-pa10"))]
use {
    super::Family,
    crate::addr::PeriphLabel,
    crate::chip::Chip,
    crate::clock,
    crate::config::{Oversampling, UsartConfig, UsartFrame},
    crate::error::DescriptorError,
    crate::gpio::{self, PinRole},
    crate::usart::Usart,
};

/// The bare-IRC8M default clock (DF-2): 8 MHz, 0 wait states, no PLL. The fallback stays on the
/// reset clock rather than bringing up a PLL that could fail. `pll_mul`/prescalers are filled with
/// legal-but-unused values so [`ClockConfig::validate_for`] passes; `configure_tree` is NOT called.
pub const FALLBACK_CLOCK: ClockConfig = ClockConfig {
    sysclk_hz: 8_000_000,
    wait_states: 0,
    source: ClockSource::Irc8m,
    // Unused (we never run the PLL on the fallback), but kept legal so `validate_for` passes: a
    // multiplier of 2 and /1 prescalers are all in-range. The USART input clock derives from
    // sysclk / ahb_psc / apbx_psc = 8 MHz, the bare IRC8M speed.
    pll_mul: 2,
    ahb_psc: 1,
    apb1_psc: 1,
    apb2_psc: 1,
};

/// The shared USART0 base (APB2 at `0x4001_3800` on both families). The synthesized descriptor does
/// not carry it, so the console bring-up supplies it here.
pub const USART0_BASE: u32 = 0x4001_3800;

/// The default console baud (DF-2: a console at 8 MHz is fine for 115200).
pub const CONSOLE_BAUD: u32 = 115_200;

/// Console TX pin, `(port << 4) | pin`: PA9 (port A = 0, pin 9; the flagged DF-4 convention).
pub const CONSOLE_TX: u8 = 9;
/// Console RX pin, `(port << 4) | pin`: PA10 (port A = 0, pin 10; the flagged DF-4 convention).
pub const CONSOLE_RX: u8 = 10;

/// `true`: the console TX/RX pins (PA9/PA10) are an ASSUMED convention, not a silicon fact (DF-4).
/// The firmware should log this. Board routing can place the console elsewhere; this is the common
/// default and is easily corrected.
pub const CONSOLE_PINS_ASSUMED: bool = true;

/// Apply the section-6 default bring-up to a DETECTED chip: bare IRC8M (no clock-tree call), enable
/// the GPIOA + USART0 peripheral clocks, configure PA9/PA10 as the USART0 AF console pins, and bring
/// up USART0 at 8 MHz, 115200 8N1, oversample /16. Returns the configured [`Usart`].
///
/// The clock is left at the reset IRC8M 8 MHz; this helper does NOT call `configure_tree`. The RCU
/// base comes from the synthesized chip (it carries `Rcu` at `0x4002_1000`). The console pins are the
/// flagged PA9/PA10 convention ([`CONSOLE_PINS_ASSUMED`]); the firmware should log the assumption.
///
/// `family` is accepted for symmetry / logging; the register model is taken from `chip.clock()` (the
/// synthesized clock path), which the family already fixed.
///
/// HAL-internal (`pub(crate)`): it takes the internal [`Family`] discriminator, so it cannot be a
/// caller surface while the silicon-purity principle keeps `Family` out of the public API. It is the
/// retained section-6 default console bring-up, exercised by the host tests; nothing else in-crate
/// calls it.
///
/// **GATE-PIN GUARD (debt-paydown slice 9):** PA9/PA10 are FET gate pins on the 6-FET hoverboard
/// boards (`specs/l3.md` pin safety denies them), so this function is COMPILED OUT of every real
/// (non-mock) build unless the explicit `yes-console-on-pa9-pa10` feature opts in - the
/// wfi-lock-repro gating pattern, protective only. The mock host build keeps it (its tests pin the
/// register sequence; no real pin exists on the host).
#[cfg(any(feature = "mock", feature = "yes-console-on-pa9-pa10"))]
#[allow(dead_code)] // mock builds keep it for the host tests; nothing non-test calls it
pub(crate) fn apply_defaults(chip: &Chip, family: Family) -> Result<Usart, DescriptorError> {
    let _ = family; // the register model is carried by chip.clock(); family is for logging only.
    let path = chip.clock();
    let rcu = chip.rcu_base()?;

    // Bare IRC8M: validate the (PLL-less) config for the family, but DO NOT configure_tree. The chip
    // stays on the reset 8 MHz IRC8M clock. (validate_for catches a config illegal for the family.)
    FALLBACK_CLOCK
        .validate_for(path)
        .map_err(|_| DescriptorError::SelectorAddrMismatch)?;

    // GPIO available: enable the GPIOA port clock (the console pins live on PA).
    clock::enable_gpio_port(rcu, path, PeriphLabel::Gpioa)?;
    // USART0 peripheral clock (APB2EN bit 14 on both families).
    clock::enable_usart(rcu, path, PeriphLabel::Usart0)?;

    // Configure PA9 (TX) and PA10 (RX) as the USART0 alternate-function console pins (DF-4
    // convention). GPIOA's base comes from the synthesized descriptor.
    let gpioa = chip.base(PeriphLabel::Gpioa)?;
    let (tx_pin, rx_pin) = (CONSOLE_TX & 0x0F, CONSOLE_RX & 0x0F);
    gpio::configure_af(gpioa, chip.gpio(), tx_pin, PinRole::Tx);
    gpio::configure_af(gpioa, chip.gpio(), rx_pin, PinRole::Rx);

    // Bring up USART0 at 8 MHz / 115200 8N1 /16. The synthesized descriptor does not carry a USART0
    // base, so wrap a local augmented chip that adds USART0 = 0x4001_3800 for the bring-up only.
    let console_chip = with_usart0(chip);
    let cfg = UsartConfig {
        usart: PeriphLabel::Usart0,
        baud: CONSOLE_BAUD,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    };
    Usart::bring_up(&console_chip, &FALLBACK_CLOCK, &cfg)
}

/// A copy of `chip` with the shared USART0 base added, for the console bring-up. The detected chip
/// the firmware holds is untouched (it stays byte-for-byte equal to the per-family constant). Only
/// [`apply_defaults`] uses it (and shares its gate; see that fn's GATE-PIN GUARD).
#[cfg(any(feature = "mock", feature = "yes-console-on-pa9-pa10"))]
#[allow(dead_code)] // see apply_defaults
fn with_usart0(chip: &Chip) -> Chip {
    let mut desc = *chip.descriptor();
    desc.addrs.set(PeriphLabel::Usart0, USART0_BASE);
    Chip::from_descriptor(desc)
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;

    #[test]
    fn fallback_clock_is_bare_irc8m_8mhz_and_validates() {
        // DF-2: bare IRC8M, 8 MHz, 0 wait states, no PLL run. The config validates for both families
        // (the shared RCU register model), so `apply_defaults` would not reject it.
        assert_eq!(FALLBACK_CLOCK.sysclk_hz, 8_000_000);
        assert_eq!(FALLBACK_CLOCK.wait_states, 0);
        assert_eq!(FALLBACK_CLOCK.source, ClockSource::Irc8m);
        assert!(FALLBACK_CLOCK
            .validate_for(crate::descriptor::ClockPath::F10xRcc)
            .is_ok());
        assert!(FALLBACK_CLOCK
            .validate_for(crate::descriptor::ClockPath::F1x0Rcu)
            .is_ok());
    }

    #[test]
    fn console_pins_are_the_flagged_pa9_pa10_convention() {
        // DF-4: PA9 (TX) / PA10 (RX), flagged as an assumption.
        assert_eq!(CONSOLE_TX, 9); // PA9: port 0, pin 9
        assert_eq!(CONSOLE_RX, 10); // PA10: port 0, pin 10
        assert!(CONSOLE_PINS_ASSUMED);
        assert_eq!(CONSOLE_BAUD, 115_200);
    }

    #[test]
    fn usart0_base_is_the_shared_apb2_base() {
        // 0x4001_3800 on both families (addr::ranges::USART_APB2 low bound).
        assert_eq!(USART0_BASE, crate::addr::ranges::USART_APB2.0);
    }

    #[test]
    fn apply_defaults_brings_up_the_console_on_both_families() {
        use crate::detect::{descriptor_f103, descriptor_f130, Family};
        use crate::reg::mock;

        // apply_defaults takes the internal Family discriminator (for logging only) and brings up
        // the section-6 default IRC8M console on USART0. Exercise both families: it enables the
        // GPIOA + USART0 clocks, configures PA9/PA10, and returns Ok with the console Usart. The
        // detected chip itself is untouched (the USART0 base is added only to a local copy).
        let _g = mock::lock();

        mock::reset();
        let f130 = Chip::from_descriptor(descriptor_f130());
        assert!(apply_defaults(&f130, Family::F1x0).is_ok());
        assert!(
            f130.base(PeriphLabel::Usart0).is_err(),
            "detected chip untouched"
        );

        mock::reset();
        let f103 = Chip::from_descriptor(descriptor_f103());
        assert!(apply_defaults(&f103, Family::F10x).is_ok());
    }
}
