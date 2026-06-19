//! Code-level peripheral configuration types (descriptor-rework DR-T1).
//!
//! DECISIONS.md #10: the CBOR descriptor defines the CHIP only (register-model selectors, the
//! base-address table, the chip capabilities). The APPLICATION configures peripherals in CODE
//! through the typed config values in this module: pins, baud, bus speed, SPI mode, ADC channels,
//! and all PWM/timer timing live here, not in the descriptor.
//!
//! These types are the former `*Wiring` structs, relocated out of [`crate::descriptor`]'s
//! "decoded from flash" role into the HAL's public config API and renamed `*Config` (the name now
//! reflects "the application constructs this"), WITH the missing behavior knobs added as explicit
//! fields (the superseded configurability-fix audit, now code fields not CBOR keys).
//!
//! # No hidden HAL defaults
//!
//! Every behavior-determining field is set explicitly by the application on the config value; the
//! HAL bakes no policy. A type MAY offer a named convenience constructor (opted into by name),
//! but the default lives visibly in the type's API surface, never hidden inside a bring-up.
//! Correctness-derivations (ADC scan from channel count, input-clock from the clock + bus, the
//! injected ETSIC from the trigger link) stay derived and asserted in the bring-up, not defaulted.

use heapless::Vec;

use crate::addr::PeriphLabel;
use crate::descriptor::{MAX_ADC_CHANNELS, MAX_INJECTED_CHANNELS, MAX_PWM_CHANNELS};

/// Decode a logical pin byte `(port << 4) | pin` into `(port_index, pin_number)`.
///
/// `port_index` is 0=A..5=F; `pin_number` is 0..15. The SPEC.md pin model the config types use for
/// `tx`/`rx`/`scl`/`sda`/`sck`/`miso`/`mosi`/`nss`/`high`/`low`.
#[inline]
pub const fn decode_pin(pin: u8) -> (u8, u8) {
    (pin >> 4, pin & 0x0F)
}

// --- USART ------------------------------------------------------------------------------------

/// USART word length (`USART_WL_*`). 8N1 is no longer baked; the application names the frame.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WordLen {
    /// 8 data bits (`USART_WL_8BIT`, the CTL0 WL bit clear).
    #[default]
    Eight = 0,
    /// 9 data bits (`USART_WL_9BIT`, the CTL0 WL bit set).
    Nine = 1,
}

/// USART parity (`USART_PM_*`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Parity {
    /// No parity (`USART_PM_NONE`, PCEN clear).
    #[default]
    None = 0,
    /// Even parity (`USART_PM_EVEN`, PCEN set + PM clear).
    Even = 1,
    /// Odd parity (`USART_PM_ODD`, PCEN set + PM set).
    Odd = 2,
}

/// USART stop bits (`USART_STB_*`, CTL1 STB field BITS(12,13)).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StopBits {
    /// 1 stop bit (`USART_STB_1BIT`, STB = 0).
    #[default]
    One = 0,
    /// 0.5 stop bit (`USART_STB_0_5BIT`, STB = 1).
    Half = 1,
    /// 2 stop bits (`USART_STB_2BIT`, STB = 2).
    Two = 2,
    /// 1.5 stop bits (`USART_STB_1_5BIT`, STB = 3).
    OneAndHalf = 3,
}

/// USART oversampling (`USART_OVSMOD_*`, CTL0 OVSMOD bit on the F1x0 model). The /16 default is no
/// longer baked.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Oversampling {
    /// Oversample by 16 (`USART_OVSMOD_16`, the standard mode).
    #[default]
    By16 = 0,
    /// Oversample by 8 (`USART_OVSMOD_8`, higher baud).
    By8 = 1,
}

/// USART line frame: word length, parity, stop bits (superseded audit 3.1). No baked 8N1.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UsartFrame {
    /// Word length (8 or 9 data bits).
    pub word_len: WordLen,
    /// Parity mode.
    pub parity: Parity,
    /// Stop bits.
    pub stop: StopBits,
}

impl UsartFrame {
    /// The common 8N1 frame (8 data bits, no parity, 1 stop), opted into by name. This is a named
    /// convenience, not a hidden HAL default: the application chooses it explicitly.
    pub const EIGHT_N_ONE: UsartFrame = UsartFrame {
        word_len: WordLen::Eight,
        parity: Parity::None,
        stop: StopBits::One,
    };
}

/// One USART's application configuration (was `UsartWiring`).
///
/// The chip context ([`crate::Chip`]) resolves [`Self::usart`] to a base and supplies the register
/// model; the application supplies the pins, baud, frame, and oversampling. `tx`/`rx` are logical
/// pins in the SPEC.md model: one byte `(port << 4) | pin` (port 0=A..5=F).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsartConfig {
    /// Which USART instance label.
    pub usart: PeriphLabel,
    /// TX pin, `(port << 4) | pin`.
    pub tx: u8,
    /// RX pin, `(port << 4) | pin`.
    pub rx: u8,
    /// Target baud rate.
    pub baud: u32,
    /// Line frame (word length / parity / stop). No baked 8N1.
    pub frame: UsartFrame,
    /// Oversampling. No baked /16.
    pub oversampling: Oversampling,
}

// --- I2C --------------------------------------------------------------------------------------
//
// I2C no longer has a packed-pin `*Config` struct: [`crate::i2c::I2c::new`] takes the SCL/SDA pins
// as type-state [`crate::gpio::Pin`] handles from `split()` (the stm32f1xx-hal ownership pattern),
// so the application never writes a `(port << 4) | pin` byte. The bus speed + fast-mode duty live
// in [`crate::i2c::I2cMode`], and the instance is named directly as a [`PeriphLabel`]. USART/SPI
// still carry packed-u8 pins below; they could follow the same pin-handle pattern later (out of
// scope here).

// --- SPI --------------------------------------------------------------------------------------

/// SPI NSS management (superseded audit 3.3). No baked software-NSS.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NssMode {
    /// Software-managed NSS (`SPI_NSS_SOFT`, SWNSSEN): the application owns chip-select. The
    /// `embedded-hal` `SpiBus` trait does not own CS, so this is the ergonomic choice.
    #[default]
    Software,
    /// Hardware NSS (`SPI_NSS_HARD`, NSSDRV): the peripheral drives the NSS line.
    Hardware,
}

/// One SPI bus's application configuration (was `SpiWiring`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiConfig {
    /// Which SPI instance label.
    pub spi: PeriphLabel,
    /// SCK pin, `(port << 4) | pin`.
    pub sck: u8,
    /// MISO pin, `(port << 4) | pin`.
    pub miso: u8,
    /// MOSI pin, `(port << 4) | pin`.
    pub mosi: u8,
    /// NSS pin, `(port << 4) | pin` (carried for gpio config).
    pub nss: u8,
    /// SPI mode 0..3: `(CPOL << 1) | CPHA`.
    pub mode: u8,
    /// 16-bit frames (`true`) vs 8-bit (`false`).
    pub data16: bool,
    /// Target SCK frequency in Hz.
    pub target_hz: u32,
    /// Bit order (superseded audit 3.3): `true` = LSB-first. No baked MSB.
    pub lsb_first: bool,
    /// NSS management (superseded audit 3.3). No baked software-NSS.
    pub nss_mode: NssMode,
}

impl SpiConfig {
    /// The `embedded-hal` [`embedded_hal::spi::Mode`] for the `mode` code (0..3 = MODE_0..MODE_3).
    #[inline]
    pub fn mode(&self) -> embedded_hal::spi::Mode {
        use embedded_hal::spi::{Phase, Polarity};
        let cpol = (self.mode >> 1) & 1;
        let cpha = self.mode & 1;
        embedded_hal::spi::Mode {
            polarity: if cpol == 1 {
                Polarity::IdleHigh
            } else {
                Polarity::IdleLow
            },
            phase: if cpha == 1 {
                Phase::CaptureOnSecondTransition
            } else {
                Phase::CaptureOnFirstTransition
            },
        }
    }

    /// The [`crate::spi::DataSize`] for the `data16` flag.
    #[inline]
    pub fn data_size(&self) -> crate::spi::DataSize {
        if self.data16 {
            crate::spi::DataSize::Sixteen
        } else {
            crate::spi::DataSize::Eight
        }
    }
}

// --- ADC --------------------------------------------------------------------------------------

/// ADC clock prescaler (`RCU_(CK)ADC_CKAPB2_DIV*`, superseded audit 3.6). No baked /6.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AdcClockDiv {
    /// CK_APB2 / 2 (ADCPSC field code 0).
    Div2 = 0,
    /// CK_APB2 / 4 (ADCPSC field code 1).
    Div4 = 1,
    /// CK_APB2 / 6 (ADCPSC field code 2). 12 MHz at the 72 MHz APB2 tree.
    #[default]
    Div6 = 2,
    /// CK_APB2 / 8 (ADCPSC field code 3).
    Div8 = 3,
}

impl AdcClockDiv {
    /// The `RCU_CFG0` ADCPSC field value (0..=3).
    #[inline]
    pub const fn psc_code(self) -> u32 {
        self as u32
    }
}

/// One entry in the ADC regular-conversion sequence (was `AdcChannel`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcChannel {
    /// ADC input channel number (0..=15 external, 16 = temperature, 17 = VREFINT).
    pub channel: u8,
    /// Sample-time field code (`ADC_SAMPLETIME_*`, 0..=7; 7 = 239.5 cycles, the slowest/safest).
    pub sample_time: u8,
}

/// One ADC's application configuration (was `AdcWiring`).
///
/// Not `Copy`: carries a bounded channel Vec. Scan mode is a correctness-derivation from
/// `channels.len()`, asserted in the bring-up, not a field. Resolution (12-bit), the calibration
/// sequence, and ETERC-for-software-trigger stay HAL invariants.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdcConfig {
    /// Which ADC instance label.
    pub adc: PeriphLabel,
    /// The regular-conversion channel sequence (rank order), each with its sample time.
    pub channels: Vec<AdcChannel, MAX_ADC_CHANNELS>,
    /// Data alignment (superseded audit 2.8 / 3.4): `true` = left-aligned. No baked right-aligned.
    pub left_aligned: bool,
    /// ADC clock prescaler (superseded audit 3.6). No baked /6.
    pub clock_div: AdcClockDiv,
}

impl AdcConfig {
    /// True if the sequence contains an internal channel (16 = temperature, 17 = VREFINT), so the
    /// bring-up must set the `TSVREN` enable bit.
    #[inline]
    pub fn needs_internal_enable(&self) -> bool {
        self.channels
            .iter()
            .any(|c| c.channel == 16 || c.channel == 17)
    }
}

// --- timer PWM ---------------------------------------------------------------------------------

/// Center-aligned sub-mode + counting direction (superseded audit 2.1). No baked CAM=2.
///
/// Maps to the TIMER `CTL0` DIR + CAM fields. `EdgeUp` / `EdgeDown` are edge-aligned (CAM = 0) with
/// DIR up / down; `Center1`/`Center2`/`Center3` are the three center-aligned sub-modes (CAM = 1/2/3).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PwmAlign {
    /// Edge-aligned, counting up (CAM = 0, DIR = 0).
    EdgeUp = 0,
    /// Edge-aligned, counting down (CAM = 0, DIR = 1).
    EdgeDown = 1,
    /// Center-aligned mode 1 (CAM = 1): compare flags set when counting down.
    Center1 = 2,
    /// Center-aligned mode 2 (CAM = 2): compare flags set when counting up. The reference's mode.
    #[default]
    Center2 = 3,
    /// Center-aligned mode 3 (CAM = 3): compare flags set both directions.
    Center3 = 4,
}

impl PwmAlign {
    /// The `CTL0` DIR + CAM field bits (DIR = bit 4, CAM = `bits[6:5]`).
    #[inline]
    pub const fn ctl0_bits(self) -> u32 {
        match self {
            PwmAlign::EdgeUp => 0,
            PwmAlign::EdgeDown => 1 << 4,
            PwmAlign::Center1 => 1 << 5,
            PwmAlign::Center2 => 2 << 5,
            PwmAlign::Center3 => 3 << 5,
        }
    }
}

/// Output-compare mode for a trigger channel (superseded audit 2.3). No baked PWM0.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OcMode {
    /// PWM mode 0 (`TIMER_OC_MODE_PWM0`, COMCTL = 0b110).
    #[default]
    Pwm0 = 0,
    /// PWM mode 1 (`TIMER_OC_MODE_PWM1`, COMCTL = 0b111).
    Pwm1 = 1,
}

impl OcMode {
    /// The 3-bit COMCTL field value within a channel half.
    #[inline]
    pub const fn comctl(self) -> u32 {
        match self {
            OcMode::Pwm0 => 0b110,
            OcMode::Pwm1 => 0b111,
        }
    }
}

/// Dead-time / sampling clock divider (`TIMER_CKDIV_DIV*`, superseded audit 2.12). No baked /2.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClockDiv {
    /// fDTS = fTIMER_CK / 1 (`TIMER_CKDIV_DIV1`, CKDIV field 0).
    Div1 = 0,
    /// fDTS = fTIMER_CK / 2 (`TIMER_CKDIV_DIV2`, CKDIV field 1). The reference's value.
    #[default]
    Div2 = 1,
    /// fDTS = fTIMER_CK / 4 (`TIMER_CKDIV_DIV4`, CKDIV field 2).
    Div4 = 2,
}

impl ClockDiv {
    /// The `CTL0` CKDIV field value (`bits[9:8]`).
    #[inline]
    pub const fn ckdiv_code(self) -> u32 {
        self as u32
    }
}

/// Master-mode TRGO source (`TIMER_TRI_OUT_SRC_*`, superseded audit 2.13). No baked UPDATE.
///
/// Maps to the TIMER `CTL1` MMC field (`bits[6:4]`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrgoSource {
    /// Software reset (`TIMER_TRI_OUT_SRC_RESET`, MMC = 0).
    Reset = 0,
    /// Counter enable (`TIMER_TRI_OUT_SRC_ENABLE`, MMC = 1).
    Enable = 1,
    /// Update event (`TIMER_TRI_OUT_SRC_UPDATE`, MMC = 2). The reference's TRGO source.
    #[default]
    Update = 2,
    /// CH0 compare-pulse (`TIMER_TRI_OUT_SRC_O0CPRE`, MMC = 3).
    Ch0Pulse = 3,
    /// CH0 compare (`TIMER_TRI_OUT_SRC_O0CPRE`, MMC = 4).
    Ch0Compare = 4,
    /// CH1 compare (MMC = 5).
    Ch1Compare = 5,
    /// CH2 compare (MMC = 6).
    Ch2Compare = 6,
    /// CH3 compare (MMC = 7).
    Ch3Compare = 7,
}

impl TrgoSource {
    /// The `CTL1` MMC field value (`bits[6:4]`).
    #[inline]
    pub const fn mmc(self) -> u32 {
        self as u32
    }
}

/// Break-input configuration for the advanced-timer PWM (was `BreakConfig`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BreakConfig {
    /// Break input enabled (`true`) as a hardware kill, or disabled (`false`, the reference).
    pub enabled: bool,
    /// Break input active level when enabled: `false` = active-low, `true` = active-high.
    pub level: bool,
}

/// One complementary PWM channel pair's configuration (was `PwmChannel`).
///
/// A pair is one half-bridge: a high-side compare output (CHx) and its complementary low-side
/// output (CHxN). `high`/`low` are logical pins in the SPEC.md model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmChannelConfig {
    /// High-side output pin (CHx), `(port << 4) | pin`.
    pub high: u8,
    /// Low-side complementary output pin (CHxN), `(port << 4) | pin`.
    pub low: u8,
    /// Output polarity: `false` = active-high, `true` = active-low (the reference inverts the
    /// complementary low-side polarity so the bridge idles safe).
    pub polarity: bool,
    /// Main-output idle level when disarmed (MOE clear) (superseded audit 2.6: was a single
    /// `idle`). `true` = idle HIGH.
    pub idle_high: bool,
    /// Complementary-output idle level when disarmed (superseded audit 2.6). `true` = idle HIGH.
    pub idle_high_n: bool,
}

/// The advanced-timer / complementary-PWM application configuration (was `PwmWiring`).
///
/// Carries TIMER0's full hot-path config: the three complementary channel pairs, dead-time, period
/// / prescaler, break config, the ADC-trigger channel (CH3) compare, plus the timing-topology knobs
/// that were baked constants before (the superseded audit's seven dump-diff divergences). The duty
/// (CHxCV compare value) stays OFF the config (it is per-cycle data written through the handle).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmConfig {
    /// Which advanced-timer instance label.
    pub timer: PeriphLabel,
    /// The three complementary phase channel pairs (CH0/0N, CH1/1N, CH2/2N) in channel order.
    pub channels: [PwmChannelConfig; MAX_PWM_CHANNELS],
    /// PWM period (the TIMER CAR/ARR auto-reload value).
    pub period: u16,
    /// Prescaler (the TIMER PSC value; counter clock = `timer_clk / (prescaler + 1)`).
    pub prescaler: u16,
    /// Dead-time field code (the TIMER CCHP DTCFG encoding).
    pub dead_time: u8,
    /// Break-input configuration.
    pub brk: BreakConfig,
    /// The ADC-trigger compare channel (CH3) compare value.
    pub trigger_compare: u16,
    /// Center-aligned sub-mode + counting direction (audit 2.1). No baked CAM=2.
    pub align: PwmAlign,
    /// Auto-reload preload enable (audit 2.2). No baked ARSE=on.
    pub arse: bool,
    /// CH3 trigger-channel output-compare mode (audit 2.3). No baked PWM0.
    pub trigger_oc_mode: OcMode,
    /// CH3 trigger-channel output enable (audit 2.4). No baked CH3EN=off.
    pub trigger_ch_enable: bool,
    /// Repetition counter (audit 2.5). No baked CREP=0.
    pub crep: u8,
    /// Dead-time / sampling clock divider (audit 2.12). No baked CKDIV /2.
    pub ckdiv: ClockDiv,
    /// Master-mode TRGO source (audit 2.13). No baked UPDATE.
    pub trgo_src: TrgoSource,
}

// --- injected ADC ------------------------------------------------------------------------------

/// One injected-conversion ADC channel (was `InjectedChannel`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjectedChannel {
    /// ADC input channel number (0..=15 external phase-current channels; 16/17 internal).
    pub channel: u8,
    /// Sample-time field code (`ADC_SAMPLETIME_*`, 0..=7).
    pub sample_time: u8,
}

/// Which timer trigger event drives the injected group (was `TimerTriggerLink`).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerTriggerLink {
    /// The timer's CH3 compare event triggers the injected group (ADC injected ETSIC = TIMER0 CH3).
    Ch3 = 0,
    /// The timer's TRGO (master-mode) triggers the injected group (ETSIC = TIMER0 TRGO).
    Trgo = 1,
}

/// The timer-triggered injected-ADC application configuration (was `InjectedAdcWiring`).
///
/// Not `Copy`: carries a bounded channel Vec. The trigger linkage stays a correctness-derivation
/// (the raw ETSIC code is derived from `trigger_timer` + `trigger_link` + the family); EOICIE /
/// ETEIC stay invariants.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InjectedAdcConfig {
    /// Which ADC instance label.
    pub adc: PeriphLabel,
    /// The injected channel sequence (in injected-rank order), each with its sample time.
    pub channels: Vec<InjectedChannel, MAX_INJECTED_CHANNELS>,
    /// Data alignment: `true` = left-aligned (the reference), `false` = right-aligned.
    pub left_aligned: bool,
    /// The advanced timer whose trigger drives this injected group.
    pub trigger_timer: PeriphLabel,
    /// Which trigger event of `trigger_timer` drives the group (CH3 compare or TRGO).
    pub trigger_link: TimerTriggerLink,
    /// ADC clock prescaler (superseded audit 3.6, per-ADC; DR-5).
    pub clock_div: AdcClockDiv,
}
