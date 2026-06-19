//! Error types.
//!
//! DECISIONS.md #5 splits the error space in two: a rich [`DescriptorError`] for the
//! parse/validate boundary (which field or selector failed), and small per-bus runtime
//! error types that implement the relevant `embedded-hal` / `embedded-io` `Error` traits
//! via `ErrorKind`. Neither carries an allocation in its payload.

use crate::addr::PeriphLabel;

/// Validation failures at the chip-descriptor boundary.
///
/// `#[non_exhaustive]` and additive (DECISIONS.md #5). The chip descriptor is now synthesized by
/// runtime detection ([`crate::detect`]) rather than decoded from a data blob, so the former
/// frame/CBOR-decode variants are gone; what remains are the selector-vs-address and resolution
/// invariants the synthesized descriptor and the code-level config are still checked against.
/// No allocation in the payload (the only data is a copy-`PeriphLabel`).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DescriptorError {
    /// A path selector named an address range it does not own (e.g. `ahb_ctl_afsel`
    /// paired with an APB GPIO base). See the per-path range check in [`crate::addr`].
    SelectorAddrMismatch,
    /// A selector value this build does not implement.
    UnknownSelector,
    /// The address table has no base for a label that the wiring requires.
    MissingBase(PeriphLabel),
    /// A USART wiring record failed validation.
    UsartConfig,
}

/// The runtime-detection boot entry's failure surface.
///
/// `#[non_exhaustive]` and additive (DECISIONS.md #5). Runtime heuristic detection is the only way
/// the HAL learns its chip identity, so the only way [`crate::detect::detect_chip`] can fail to
/// produce a [`crate::Chip`] is that the silicon probe matched neither known family:
///
/// - [`DetectError::NoFamily`]: the bus-fault-safe GPIO+RCU family probe matched NEITHER the F10x
///   APB-GPIO model nor the F1x0 AHB-GPIO model, so the boot fails safe (halt on the reset IRC8M
///   clock, outputs untouched) rather than guessing a register layout.
///
/// Detection is fail-loud: it never silently picks a family it could not confirm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectError {
    /// The silicon family probe matched NEITHER family. The boot fails safe; it does not guess a
    /// configuration.
    NoFamily,
}

/// Per-bus runtime error for the USART cold path (T7 maps the status bits).
///
/// Implements [`embedded_io::Error`] so the polled serial impl can surface it through the
/// `embedded-io` seam. The variants here are the ones the USART status register can raise;
/// `Other` is a catch-all that maps to `ErrorKind::Other`.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsartError {
    /// Receive overrun (a byte arrived before the previous one was read).
    Overrun,
    /// Framing error (stop bit not seen / line noise broke the frame).
    Framing,
    /// Parity error.
    Parity,
    /// An error the kind mapping does not name specifically.
    Other,
}

impl embedded_io::Error for UsartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        // embedded-io has no dedicated overrun/framing/parity kinds, so the recoverable
        // line conditions fold into `Other` per the trait's guidance.
        match self {
            UsartError::Overrun | UsartError::Framing | UsartError::Parity | UsartError::Other => {
                embedded_io::ErrorKind::Other
            }
        }
    }
}

/// Per-bus runtime error for the I2C cold path (M2 T1; the transfer task T7 raises the variants).
///
/// Implements [`embedded_hal::i2c::Error`] so the polled `i2c::I2c` impl can surface it through the
/// `embedded-hal` 1.0 seam (DECISIONS.md #5: a per-bus runtime error, separate from
/// [`DescriptorError`]). The variants mirror the classic event-based GD32 I2C status flags
/// (STAT0/STAT1: BERR, LOSTARB, AERR) plus a polled timeout; the exact flag-to-kind mapping is
/// finalized in T7 (open item I2C-2). Stubbed here so the bus tasks can `?`-propagate it.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// Bus error (misplaced START/STOP): GD `I2C_STAT0_BERR`.
    Bus,
    /// Arbitration lost (multi-master): GD `I2C_STAT0_LOSTARB`.
    ArbitrationLoss,
    /// Acknowledge failure on the address phase: GD `I2C_STAT0_AERR` after the address byte.
    NoAcknowledgeAddress,
    /// Acknowledge failure on a data byte: GD `I2C_STAT0_AERR` after a data byte.
    NoAcknowledgeData,
    /// Receive overrun / underrun: GD `I2C_STAT0_OUERR`.
    Overrun,
    /// A polled handshake did not complete within its bound (the F130 hang-if-done-wrong class).
    Timeout,
    /// An error the kind mapping does not name specifically.
    Other,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            I2cError::Bus => ErrorKind::Bus,
            I2cError::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            I2cError::NoAcknowledgeAddress => {
                ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
            }
            I2cError::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            I2cError::Overrun => ErrorKind::Overrun,
            // embedded-hal 1.0 i2c::ErrorKind has no Timeout; a polled timeout folds into Other.
            I2cError::Timeout | I2cError::Other => ErrorKind::Other,
        }
    }
}

/// Per-bus runtime error for the SPI cold path (M2 T1; the transfer task T9 raises the variants).
///
/// Implements [`embedded_hal::spi::Error`] so the polled `spi::SpiBus` impl can surface it through
/// the `embedded-hal` 1.0 seam (DECISIONS.md #5). The variants mirror the GD32 SPI status flags
/// (mode fault CONFERR, overrun RXORERR, CRC error); stubbed here so the bus tasks can
/// `?`-propagate it.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    /// Mode fault (NSS de-asserted in master mode): GD `SPI_STAT_CONFERR`.
    ModeFault,
    /// Receive overrun: GD `SPI_STAT_RXORERR`.
    Overrun,
    /// CRC mismatch (when hardware CRC is enabled): GD `SPI_STAT_CRCERR`.
    Crc,
    /// An error the kind mapping does not name specifically.
    Other,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        use embedded_hal::spi::ErrorKind;
        match self {
            SpiError::ModeFault => ErrorKind::ModeFault,
            SpiError::Overrun => ErrorKind::Overrun,
            // embedded-hal 1.0 spi::ErrorKind models a generic CRC-less Other for CRC issues.
            SpiError::Crc | SpiError::Other => ErrorKind::Other,
        }
    }
}

/// Per-bus runtime error for the ADC cold path (M2 T11; the read task raises [`AdcError::Timeout`]).
///
/// `embedded-hal` 1.0 has **NO ADC error trait** (there is no ADC trait at all; open item ADC-1),
/// so unlike [`I2cError`] / [`SpiError`] this is a **plain runtime-hal error** that implements no
/// `embedded-hal` `Error` via `ErrorKind` (DECISIONS.md #5 keeps it separate from
/// [`DescriptorError`] all the same). The only failure the single software-triggered conversion can
/// raise is a polled timeout on the end-of-conversion (EOC) flag (the F130 hang-if-done-wrong
/// class); a calibration that never completes is the same shape but happens at bring-up.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    /// A polled flag (EOC, or a calibration-done bit at bring-up) did not clear/set within its
    /// bounded budget. On real silicon this is a stuck conversion or a mis-sequenced calibration.
    Timeout,
    /// An error the read path does not name specifically (reserved; nothing raises it in M2).
    Other,
}

/// Runtime error for the clock-tree bring-up (the bounded-timeout variant).
///
/// The SPL-faithful [`crate::clock::configure_tree`] polls UNBOUNDED on each bring-up gate
/// (source-stable, PLL-lock, SCS-confirm), matching the GD SPL's `SystemInit` (and the M2 goldens
/// diff against that unbounded poll). [`crate::clock::configure_tree_timeout`] is the firmware-
/// robustness variant: it gives up after a bounded spin budget on each wait so a board whose
/// oscillator never stabilises / PLL never locks fails cleanly instead of hanging forever. This is
/// the same F130 hang-if-done-wrong class [`AdcError::Timeout`] / [`I2cError::Timeout`] name, at the
/// clock-tree boundary. `embedded-hal` 1.0 has no clock trait, so this is a plain runtime-hal error
/// kept separate from [`DescriptorError`] (DECISIONS.md #5).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockError {
    /// The selected oscillator never reported stable (IRC8MSTB / HXTALSTB) within the spin budget.
    SourceNotStable,
    /// The PLL never reported lock (PLLSTB) within the spin budget.
    PllNotLocked,
    /// The system-clock switch never read back as the requested source (SCSS) within the budget.
    SwitchNotConfirmed,
    /// The `ClockConfig`'s PLL multiplier is outside the chip-bound legal range (DR-T3
    /// `validate_for`): the part's PLLMF field is `2..=32`.
    InvalidPll,
    /// A `ClockConfig` prescaler divisor is not a legal value for its bus (AHB / APB) on the part.
    InvalidPrescaler,
    /// The `ClockConfig` wait-states are out of range for the target sysclk (too few for the flash
    /// timing at that clock, or above the 3-bit WSCNT field), or the sysclk exceeds the part ceiling.
    InvalidWaitStates,
    /// The descriptor did not resolve the RCU base the clock-tree bring-up needs (DR-T3).
    MissingRcuBase,
}

impl From<DescriptorError> for ClockError {
    fn from(_e: DescriptorError) -> Self {
        // The only DescriptorError configure_tree surfaces is a missing RCU base.
        ClockError::MissingRcuBase
    }
}

/// Per-peripheral runtime error for the advanced-timer complementary-PWM hot path (M3 T1).
///
/// Like [`AdcError`], `embedded-hal` 1.0 has **NO PWM/timer error trait** for the complementary,
/// dead-timed, cross-peripheral-triggered hot path runtime-hal expresses (the `embedded-hal` PWM
/// traits are single-channel duty setters that cannot express MOE/dead-time/break/trigger), so this
/// is a **plain runtime-hal error** implementing no `embedded-hal` `Error` via `ErrorKind`. It is
/// kept separate from [`DescriptorError`] (DECISIONS.md #5: parse vs runtime). The bring-up
/// (config) failures are surfaced as [`DescriptorError`] at parse; this names the few runtime
/// conditions the hot-path config/arming can hit.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    /// A requested duty / compare value exceeds the configured PWM period (CAR/ARR). Writing a
    /// compare above the auto-reload would never match in a center-aligned count, leaving that
    /// phase stuck; the handle's `set_duties` clamps or rejects it (filled in T5).
    DutyOutOfRange,
    /// The advanced-timer base did not resolve / sits outside the advanced-timer window (a wiring
    /// mistake the config path catches). Mirrors the [`DescriptorError::SelectorAddrMismatch`]
    /// class but at the hot-path config boundary.
    BadTimerBase,
    /// An error the hot-path PWM does not name specifically (reserved).
    Other,
}

/// Shared hot-path error (M3 T1, DECISIONS.md #5). The hot-path config/arming surface can fail in
/// either the PWM/timer half or the injected-ADC half; this is the unified error the
/// [`crate::hotpath`] config methods return so a caller `?`-propagates one type across the
/// cross-peripheral bring-up. Each arm wraps the corresponding per-peripheral runtime error.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HotPathError {
    /// A failure in the advanced-timer / complementary-PWM half.
    Pwm(PwmError),
    /// A failure in the timer-triggered injected-ADC half.
    Adc(AdcError),
    /// A descriptor-level failure surfaced at hot-path config (e.g. a base that does not resolve).
    Descriptor(DescriptorError),
}

impl From<PwmError> for HotPathError {
    fn from(e: PwmError) -> Self {
        HotPathError::Pwm(e)
    }
}

impl From<AdcError> for HotPathError {
    fn from(e: AdcError) -> Self {
        HotPathError::Adc(e)
    }
}

impl From<DescriptorError> for HotPathError {
    fn from(e: DescriptorError) -> Self {
        HotPathError::Descriptor(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{Error as _, ErrorKind as I2cKind, NoAcknowledgeSource};
    use embedded_hal::spi::{Error as _, ErrorKind as SpiKind};

    #[test]
    fn i2c_error_kinds_map_per_open_item_i2c2() {
        assert_eq!(I2cError::Bus.kind(), I2cKind::Bus);
        assert_eq!(I2cError::ArbitrationLoss.kind(), I2cKind::ArbitrationLoss);
        assert_eq!(
            I2cError::NoAcknowledgeAddress.kind(),
            I2cKind::NoAcknowledge(NoAcknowledgeSource::Address)
        );
        assert_eq!(
            I2cError::NoAcknowledgeData.kind(),
            I2cKind::NoAcknowledge(NoAcknowledgeSource::Data)
        );
        assert_eq!(I2cError::Overrun.kind(), I2cKind::Overrun);
        // A polled timeout has no dedicated embedded-hal 1.0 i2c kind; it folds into Other.
        assert_eq!(I2cError::Timeout.kind(), I2cKind::Other);
        assert_eq!(I2cError::Other.kind(), I2cKind::Other);
    }

    #[test]
    fn spi_error_kinds_map() {
        assert_eq!(SpiError::ModeFault.kind(), SpiKind::ModeFault);
        assert_eq!(SpiError::Overrun.kind(), SpiKind::Overrun);
        assert_eq!(SpiError::Crc.kind(), SpiKind::Other);
        assert_eq!(SpiError::Other.kind(), SpiKind::Other);
    }
}
