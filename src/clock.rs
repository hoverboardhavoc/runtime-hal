//! Clock path (T3): RCU/RCC peripheral-clock **enable** for a USART instance and its GPIO port.
//!
//! This is one of the two divergent paths a USART needs (the other is [`crate::gpio`]). The
//! [`ClockPath`] selector chooses the register model at runtime (DECISIONS.md #8: one binary
//! carries both, the descriptor picks):
//!
//! - [`ClockPath::F10xRcc`] (`f10x_rcc`): GPIO ports and USART0 live on **APB2EN**; USART1/USART2
//!   live on **APB1EN**.
//! - [`ClockPath::F1x0Rcu`] (`f1x0_rcu`): GPIO ports live on **AHBEN**; USART0 on **APB2EN**,
//!   USART1 on **APB1EN**. (F1x0 has no USART2 and no port E.)
//!
//! Scope is **enable only** (set the enable bit). Each path owns its enable-register offsets and
//! bit positions; only the RCU base is data (from [`crate::addr::AddrTable`]). The enable registers are 32-bit,
//! so access is [`Reg32`] and the write is a read-modify-write that sets one bit, leaving the rest
//! of the register (other peripherals' enables) untouched, exactly as `rcu_periph_clock_enable`
//! does in the GD SPL.
//!
//! # Register facts (sourced from the GD SPL headers the vendor library uses)
//!
//! F10x (`framework-spl-gd32/.../gd32f10x/inc/gd32f10x_rcu.h`):
//! - `RCU_APB2EN` at offset `0x18`, `RCU_APB1EN` at offset `0x1C` (lines 54-55).
//! - APB2EN GPIO port enables: `PAEN=BIT(2)` .. `PFEN=BIT(7)`, `PGEN=BIT(8)` (lines 255-261).
//! - APB2EN `USART0EN=BIT(14)` (line 267).
//! - APB1EN `USART1EN=BIT(17)`, `USART2EN=BIT(18)` (lines 292-293).
//!
//! F1x0 (`framework-spl-gd32/.../gd32f1x0/inc/gd32f1x0_rcu.h`):
//! - `RCU_AHBEN` at offset `0x14`, `RCU_APB2EN` at `0x18`, `RCU_APB1EN` at `0x1C` (lines 54-56).
//! - AHBEN GPIO port enables: `PAEN=BIT(17)`, `PBEN=BIT(18)`, `PCEN=BIT(19)`, `PDEN=BIT(20)`,
//!   `PFEN=BIT(22)` (lines 188-192). (No `PEEN`: F1x0 has no port E; BIT(21) is the gap.)
//! - APB2EN `USART0EN=BIT(14)` (line 200); APB1EN `USART1EN=BIT(17)` (line 216).
//!
//! # M2 (T4) cold-path peripheral clock enables (sourced from the same SPL `rcu_periph_enum`)
//!
//! The bit positions match on both families (the `rcu_periph_enum` `RCU_REGIDX_BIT` table):
//! - **I2C0** `APB1EN BIT(21)`, **I2C1** `APB1EN BIT(22)` (both families; the F1x0 single I2C
//!   block is `I2c0`). (`gd32f1x0_rcu.h:380-381` / `gd32f10x_rcu.h:413-414`.)
//! - **SPI0** `APB2EN BIT(12)`, **SPI1** `APB1EN BIT(14)` (both families; F1x0's single SPI is
//!   `Spi0`). (`gd32f1x0_rcu.h:362,377` / `gd32f10x_rcu.h:439,407`.)
//! - **ADC0** `APB2EN BIT(9)`, **ADC1** `APB2EN BIT(10)` (F10x); the F1x0 single ADC is the
//!   unnumbered `RCU_ADC = APB2EN BIT(9)`, mapped to `Adc0`. (`gd32f1x0_rcu.h:360` /
//!   `gd32f10x_rcu.h:436-437`.)
//!
//! The F1x0 ADC additionally needs its dedicated clock prescaler set in the RCU CFG registers
//! (`rcu_adc_clock_config`, see [`enable_adc`]); the F10x ADC prescaler lives only in `RCU_CFG0`.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::ClockPath;
use crate::error::{ClockError, DescriptorError};
use crate::reg::Reg32;

/// The PLL / system clock source (relocated from the descriptor's `ClockProfile`).
///
/// On both families the IRC8M path feeds the PLL through a fixed divide-by-two
/// (`RCU_PLLSRC_IRC8M_DIV2`), so the PLL input is 4 MHz when the source is IRC8M.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// Internal 8 MHz RC oscillator. PLL input is IRC8M/2 = 4 MHz.
    Irc8m = 0,
    /// External high-speed crystal. PLL input is HXTAL.
    Hxtal = 1,
}

/// The application-supplied clock tree (descriptor-rework DR-T3; was the descriptor's
/// `ClockProfile`).
///
/// DECISIONS.md #10: the clock TARGET is an application decision, not a silicon identity, so the
/// clock tree is a code-level config the application constructs and passes to [`configure_tree`].
/// The CBOR descriptor keeps only the `clock` SELECTOR (which RCU register model) and the `Rcu`
/// base. The chip-bound limits (wait-states, valid PLL / prescaler ranges) are validated against
/// the family at bring-up via [`ClockConfig::validate_for`] / [`configure_tree`] (a `Result`), so a
/// combo the silicon cannot do is rejected loudly. No decode-time defaulting exists any more: the
/// application constructs every field.
///
/// The prescalers are stored as their integer DIVISORS (1, 2, 4, 8, 16, ...) rather than
/// register-bit codes; [`configure_tree`] maps a divisor to the family's `RCU_CFG0` prescaler bits.
/// `pll_mul` is the integer multiply factor (e.g. 18 for the IRC8M/2 -> 72 MHz recipe).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// Target system clock in Hz.
    pub sysclk_hz: u32,
    /// Flash access wait states for that sysclk.
    pub wait_states: u8,
    /// PLL / system clock source.
    pub source: ClockSource,
    /// PLL multiply factor (integer, e.g. 18). The path maps it to the family's PLLMF bits.
    pub pll_mul: u8,
    /// AHB prescaler divisor (1, 2, 4, ... 512). AHB = sysclk / `ahb_psc`.
    pub ahb_psc: u16,
    /// APB1 prescaler divisor (1, 2, 4, 8, 16). APB1 = AHB / `apb1_psc`.
    pub apb1_psc: u16,
    /// APB2 prescaler divisor (1, 2, 4, 8, 16). APB2 = AHB / `apb2_psc`.
    pub apb2_psc: u16,
}

impl ClockConfig {
    /// The 72 MHz reference tree (source IRC8M, `pll_mul = 18`, AHB/APB2 = /1, APB1 = /2 = 36 MHz,
    /// 2 wait states). A named convenience the firmware opts into by name, NOT a hidden HAL default:
    /// it reproduces the proven reference arrangement (AHB = sysclk, APB1 = sysclk/2).
    pub const REFERENCE_72M_IRC8M: ClockConfig = ClockConfig {
        sysclk_hz: 72_000_000,
        wait_states: 2,
        source: ClockSource::Irc8m,
        pll_mul: 18,
        ahb_psc: 1,
        apb1_psc: 2,
        apb2_psc: 1,
    };

    /// Validate this config against the chip-bound ranges the clock `path` declares (DECISIONS.md
    /// #10 / DR-1 mitigation). The selector owns the register model AND the legal-range table; this
    /// keeps the chip-bound FACTS in the HAL and the application's CHOICE in code.
    ///
    /// Range-checks. The RCU register MODEL is shared between the families, but the MAX SYSCLK and
    /// the FLASH wait-state timing are per-part FACTS, so those bounds are selected off `path`:
    /// - the PLL multiplier must be in `2..=32` (the PLLMF field range);
    /// - each prescaler divisor must be a legal value (AHB: 1,2,4,..512; APB: 1,2,4,8,16);
    /// - the wait-states must fit the 3-bit WSCNT field AND be at least the per-family flash minimum;
    /// - the resulting sysclk must not exceed the per-family ceiling.
    ///
    /// Per-family limits (GD datasheets, `reference/manuals/`):
    /// - **F10x** (GD32F103): CK_SYS <= 108 MHz, and the flash runs **zero wait state at the full
    ///   108 MHz** (GD32F103xx Datasheet Rev2.14: "at 108 MHz frequency with Flash accesses zero wait
    ///   states"), so the minimum wait-state is 0 across the whole range.
    /// - **F1x0**: CK_SYS <= 72 MHz (the F1x0 family max; GD32F130xx is a 48 MHz part, a per-part
    ///   sub-limit the family probe cannot distinguish, so the board owns staying within it). The
    ///   flash is zero-wait confirmed at 48 MHz (GD32F130xx Datasheet Rev3.7); above 48 MHz the
    ///   minimum wait-state is 1 (a conservative floor: the >48 MHz region is the overclock / larger
    ///   F1x0 parts, not confirmed zero-wait here, and under-setting wait states faults the core).
    pub fn validate_for(&self, path: ClockPath) -> Result<(), ClockError> {
        if self.pll_mul < 2 || self.pll_mul > 32 {
            return Err(ClockError::InvalidPll);
        }
        if !is_legal_ahb_psc(self.ahb_psc) {
            return Err(ClockError::InvalidPrescaler);
        }
        if !is_legal_apb_psc(self.apb1_psc) {
            return Err(ClockError::InvalidPrescaler);
        }
        if !is_legal_apb_psc(self.apb2_psc) {
            return Err(ClockError::InvalidPrescaler);
        }
        // Per-family ceiling + flash-timing minimum wait-state (the only chip-bound facts here).
        let (max_sysclk, min_ws): (u32, u8) = match path {
            ClockPath::F10xRcc => (108_000_000, 0),
            ClockPath::F1x0Rcu => (72_000_000, if self.sysclk_hz <= 48_000_000 { 0 } else { 1 }),
        };
        if self.sysclk_hz > max_sysclk {
            return Err(ClockError::InvalidWaitStates);
        }
        if self.wait_states > FMC_WS_WSCNT as u8 {
            return Err(ClockError::InvalidWaitStates);
        }
        if self.wait_states < min_ws {
            return Err(ClockError::InvalidWaitStates);
        }
        Ok(())
    }
}

/// True for a legal AHB prescaler divisor (`RCU_AHB_CKSYS_DIV*`): 1, 2, 4, 8, 16, 64, 128, 256, 512.
///
/// Written as explicit equality comparisons (not a `matches!` power-of-two range) so the lowering
/// stays plain `cmp`/`beq` instructions; the `matches!` form lowered to a `rbit`/`clz` + 32-bit
/// `ands.w` bit-trick that the harness's emulator mis-decoded inside its IT block.
#[inline]
fn is_legal_ahb_psc(div: u16) -> bool {
    div == 1
        || div == 2
        || div == 4
        || div == 8
        || div == 16
        || div == 64
        || div == 128
        || div == 256
        || div == 512
}

/// True for a legal APB prescaler divisor (`RCU_APBx_CKAHB_DIV*`): 1, 2, 4, 8, 16. Explicit
/// comparisons for the same emulator-codegen reason as [`is_legal_ahb_psc`].
#[inline]
fn is_legal_apb_psc(div: u16) -> bool {
    div == 1 || div == 2 || div == 4 || div == 8 || div == 16
}

// --- enable-register offsets (32-bit registers) -----------------------------------------------

/// `RCU_AHBEN` offset (F1x0 only; GPIO port enables live here).
const AHBEN: u32 = 0x14;
/// `RCU_APB2EN` offset (both families).
const APB2EN: u32 = 0x18;
/// `RCU_APB1EN` offset (both families).
const APB1EN: u32 = 0x1C;

/// Which RCU enable register a clock lives in, plus the bit to set.
struct EnableBit {
    /// Offset of the enable register from the RCU base.
    reg: u32,
    /// Bit position within that register.
    bit: u8,
}

impl EnableBit {
    /// Set this enable bit at `rcu_base`, leaving the rest of the register unchanged (RMW), the
    /// same single-bit set `rcu_periph_clock_enable` performs.
    #[inline]
    fn apply(&self, rcu_base: u32) {
        let mask = 1u32 << self.bit;
        Reg32::new(rcu_base, self.reg).modify(mask, mask);
    }
}

/// Enable the peripheral clock for a USART instance under the selected clock path.
///
/// `usart_label` must be a `Usart0/1/2` label. Returns [`DescriptorError::UnknownSelector`] if it
/// is not a USART, and [`DescriptorError::SelectorAddrMismatch`] if the path does not have that
/// USART (F1x0 has no USART2).
pub fn enable_usart(
    rcu_base: u32,
    path: ClockPath,
    usart_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    usart_enable_bit(path, usart_label)?.apply(rcu_base);
    Ok(())
}

/// Enable the peripheral clock for a GPIO port under the selected clock path.
///
/// `port` must be a `GPIOx` label. Returns [`DescriptorError::UnknownSelector`] if it is not a GPIO
/// port, and [`DescriptorError::SelectorAddrMismatch`] if the path does not have that port (F1x0
/// has no port E).
pub fn enable_gpio_port(
    rcu_base: u32,
    path: ClockPath,
    port: PeriphLabel,
) -> Result<(), DescriptorError> {
    gpio_enable_bit(path, port)?.apply(rcu_base);
    Ok(())
}

/// Enable the peripheral clock for an I2C instance under the selected clock path.
///
/// `i2c_label` must be an `I2c0/1` label. Both instances live on **APB1EN** on both families
/// (I2C0 = bit 21, I2C1 = bit 22). Returns [`DescriptorError::UnknownSelector`] for a non-I2C label.
pub fn enable_i2c(
    rcu_base: u32,
    path: ClockPath,
    i2c_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    let _ = path; // same enable register + bit on both families.
    i2c_enable_bit(i2c_label)?.apply(rcu_base);
    Ok(())
}

/// Enable the peripheral clock for a SPI instance under the selected clock path.
///
/// `spi_label` must be a `Spi0/1` label. SPI0 lives on **APB2EN** (bit 12) and SPI1 on **APB1EN**
/// (bit 14) on both families (the F1x0 single SPI block is mapped as `Spi0` and sits on APB1, but
/// the SPL keeps SPI0's enable on APB2EN bit 12, so this matches the SPL's `RCU_SPI0`). Returns
/// [`DescriptorError::UnknownSelector`] for a non-SPI label.
pub fn enable_spi(
    rcu_base: u32,
    path: ClockPath,
    spi_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    let _ = path; // same enable register + bit on both families.
    spi_enable_bit(spi_label)?.apply(rcu_base);
    Ok(())
}

/// Enable the peripheral clock for an ADC instance under the selected clock path, and set the ADC
/// clock prescaler the family needs.
///
/// `adc_label` must be an `Adc0/1` label, both on **APB2EN** (ADC0 = bit 9, ADC1 = bit 10). The
/// ADC has its own clock derived from APB2 through a prescaler; APB2 is 72 MHz on the default tree
/// and the ADC clock maximum is 14 MHz, so the default prescaler is **CK_APB2 / 6 = 12 MHz**
/// (`RCU_(CK)ADC_CKAPB2_DIV6`, the value the GD SPL examples use), set in `RCU_CFG0` ADCPSC
/// (`bits[15:14]`) on both families. On F1x0 the SPL `rcu_adc_clock_config` additionally selects the
/// APB2-derived clock source via `RCU_CFG2` ADCSEL (bit 8); the F10x ADC clock comes from APB2
/// unconditionally and touches only `RCU_CFG0`. This mirrors the per-family `rcu_adc_clock_config`.
///
/// Returns [`DescriptorError::UnknownSelector`] for a non-ADC label.
pub fn enable_adc(
    rcu_base: u32,
    path: ClockPath,
    adc_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    let bit = adc_enable_bit(adc_label)?;
    bit.apply(rcu_base);
    adc_clock_config(rcu_base, path);
    Ok(())
}

/// Enable the peripheral clock for an advanced-timer instance under the selected clock path.
///
/// `timer_label` must be a `Timer0/7` label. **TIMER0** is on **APB2EN bit 11** on both families
/// (`RCU_APB2EN_TIMER0EN = BIT(11)`, confirmed against `gd32f1x0_rcu.h:198` and
/// `gd32f10x_rcu.h:264`; the SPL `rcu_periph_clock_enable(RCU_TIMER0)` sets exactly this bit).
/// **TIMER7** exists only on F10x (**APB2EN bit 13**, `gd32f10x_rcu.h:266`); the F1x0 has no TIMER7,
/// so requesting it on the F1x0 path is a [`DescriptorError::SelectorAddrMismatch`]. Returns
/// [`DescriptorError::UnknownSelector`] for a non-timer label.
pub fn enable_timer(
    rcu_base: u32,
    path: ClockPath,
    timer_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    timer_enable_bit(path, timer_label)?.apply(rcu_base);
    Ok(())
}

/// Enable the peripheral clock for a GENERAL-purpose timer instance (G3).
///
/// `timer_label` must be a general-timer label (`Timer1`). **TIMER1** is on **APB1EN bit 0** on both
/// families (`RCU_APB1EN_TIMER1EN = BIT(0)`, confirmed against the GD32F10x User Manual line 5425
/// `0 TIMER1EN TIMER1 clock enable` and the GD32F1x0 User Manual RCU APB1EN; the SPL
/// `rcu_periph_clock_enable(RCU_TIMER1)` sets exactly this bit). The enable register + bit are the
/// SAME on both families, so there is no [`ClockPath`] branch (the `path` is accepted for symmetry
/// with the other enables and to document the family). Returns
/// [`DescriptorError::UnknownSelector`] for a non-general-timer label (including the ADVANCED timers
/// `Timer0`/`Timer7`, which the cold-path general PWM must never enable through this path).
pub fn enable_general_timer(
    rcu_base: u32,
    path: ClockPath,
    timer_label: PeriphLabel,
) -> Result<(), DescriptorError> {
    let _ = path; // same enable register + bit on both families.
    general_timer_enable_bit(timer_label)?.apply(rcu_base);
    Ok(())
}

// --- LSI / IRC40K (the free-watchdog clock source) + reset-cause flags (G-WDG) ----------------
//
// The free watchdog (FWDGT) is clocked from the always-available ~40 kHz internal RC oscillator
// (IRC40K / LSI). Its enable + stabilisation flag and the watchdog reset-cause flag all live in the
// RCU `RSTSCK` register (offset 0x24) with the SAME bit positions on both families
// (gd32f1x0_rcu.h / gd32f10x_rcu.h): IRC40KEN bit 0, IRC40KSTB bit 1, RSTFC (reset-flag-clear) bit
// 24, FWDGTRSTF bit 29. They sit in `clock.rs` because this module owns the RCU base + register
// model, NOT because they branch by family (they do not); the FWDGT register block itself is one
// model parameterised by base (see `watchdog.rs`).

/// `RCU_RSTSCK` (reset source / clock register) offset (both families, 0x24).
const RSTSCK: u32 = 0x24;
/// IRC40K (LSI) enable (`RSTSCK_IRC40KEN`, bit 0).
const RSTSCK_IRC40KEN: u32 = 1 << 0;
/// IRC40K (LSI) stabilisation flag (`RSTSCK_IRC40KSTB`, bit 1).
const RSTSCK_IRC40KSTB: u32 = 1 << 1;
/// Reset-flag clear (`RSTSCK_RSTFC`, bit 24): write 1 to clear all the reset-cause flags.
const RSTSCK_RSTFC: u32 = 1 << 24;
/// Free-watchdog reset flag (`RSTSCK_FWDGTRSTF`, bit 29): set when the last reset was the FWDGT.
const RSTSCK_FWDGTRSTF: u32 = 1 << 29;

/// Spin budget for the IRC40K stabilisation poll. Iterations, not cycles (clock-independent),
/// generous enough never to false-time a healthy oscillator but always escaping a dead one. Mirrors
/// the other bounded bring-up polls ([`crate::adc::ADC_TIMEOUT`]).
const LSI_STAB_TIMEOUT: u32 = 100_000;

/// Enable and stabilise the LSI / IRC40K oscillator (the free-watchdog clock source).
///
/// Sets `RSTSCK.IRC40KEN` (the SPL `rcu_osci_on(RCU_IRC40K)`) and spins, bounded, until
/// `RSTSCK.IRC40KSTB` reports it stable (the SPL `rcu_osci_stab_wait`). The enable bit and the
/// stabilisation flag are at the same positions on both families, so there is no [`ClockPath`]
/// branch here. Returns [`ClockError::SourceNotStable`] if the oscillator never stabilises within
/// the bounded budget (the F130 hang-if-done-wrong class), instead of spinning forever.
///
/// This is the one clock-side prerequisite of [`crate::watchdog::FreeWatchdog::start`]: the watchdog
/// cannot count until its clock is running.
pub fn enable_lsi(rcu_base: u32) -> Result<(), ClockError> {
    let rstsck = Reg32::new(rcu_base, RSTSCK);
    rstsck.modify(RSTSCK_IRC40KEN, RSTSCK_IRC40KEN);
    let mut budget = LSI_STAB_TIMEOUT;
    while rstsck.read() & RSTSCK_IRC40KSTB == 0 {
        budget -= 1;
        if budget == 0 {
            return Err(ClockError::SourceNotStable);
        }
    }
    Ok(())
}

/// True if the last reset was caused by the free watchdog (`RSTSCK.FWDGTRSTF`).
///
/// The reset-cause flag persists across the reset until [`clear_reset_flags`] clears it, so a
/// firmware reads it at boot to detect / log a watchdog recovery. Same bit position on both
/// families. The public entry point is [`crate::watchdog::was_watchdog_reset`].
#[inline]
pub fn was_fwdgt_reset(rcu_base: u32) -> bool {
    Reg32::new(rcu_base, RSTSCK).read() & RSTSCK_FWDGTRSTF != 0
}

/// Clear the RCU reset-cause flags by writing `RSTSCK.RSTFC` (reset-flag clear), so a later
/// [`was_fwdgt_reset`] reflects only a fresh reset cause. Read the cause BEFORE clearing it. The
/// public entry point is [`crate::watchdog::clear_reset_cause`].
#[inline]
pub fn clear_reset_flags(rcu_base: u32) {
    // RSTFC is a write-1-to-clear control bit; set it in an RMW so the IRC40K enable is undisturbed.
    Reg32::new(rcu_base, RSTSCK).modify(RSTSCK_RSTFC, RSTSCK_RSTFC);
}

// --- bit selection ----------------------------------------------------------------------------

/// The enable register + bit for a USART under the selected path.
fn usart_enable_bit(path: ClockPath, usart: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match (path, usart) {
        // Both families: USART0 is on APB2EN bit 14; USART1 is on APB1EN bit 17.
        (_, PeriphLabel::Usart0) => Ok(EnableBit {
            reg: APB2EN,
            bit: 14,
        }),
        (_, PeriphLabel::Usart1) => Ok(EnableBit {
            reg: APB1EN,
            bit: 17,
        }),
        // USART2 exists only on F10x (APB1EN bit 18); F1x0 has no USART2.
        (ClockPath::F10xRcc, PeriphLabel::Usart2) => Ok(EnableBit {
            reg: APB1EN,
            bit: 18,
        }),
        (ClockPath::F1x0Rcu, PeriphLabel::Usart2) => Err(DescriptorError::SelectorAddrMismatch),
        // Not a USART label.
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The enable register + bit for a GPIO port under the selected path.
fn gpio_enable_bit(path: ClockPath, port: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    if !port.is_gpio() {
        return Err(DescriptorError::UnknownSelector);
    }
    match path {
        // F10x: GPIO ports on APB2EN. PAEN=2, PBEN=3, PCEN=4, PDEN=5, PEEN=6, PFEN=7 (PGEN=8).
        ClockPath::F10xRcc => {
            let bit = match port {
                PeriphLabel::Gpioa => 2,
                PeriphLabel::Gpiob => 3,
                PeriphLabel::Gpioc => 4,
                PeriphLabel::Gpiod => 5,
                PeriphLabel::Gpioe => 6,
                PeriphLabel::Gpiof => 7,
                _ => return Err(DescriptorError::UnknownSelector),
            };
            Ok(EnableBit { reg: APB2EN, bit })
        }
        // F1x0: GPIO ports on AHBEN. PAEN=17, PBEN=18, PCEN=19, PDEN=20, (no PEEN), PFEN=22.
        ClockPath::F1x0Rcu => {
            let bit = match port {
                PeriphLabel::Gpioa => 17,
                PeriphLabel::Gpiob => 18,
                PeriphLabel::Gpioc => 19,
                PeriphLabel::Gpiod => 20,
                // F1x0 has no port E.
                PeriphLabel::Gpioe => return Err(DescriptorError::SelectorAddrMismatch),
                PeriphLabel::Gpiof => 22,
                _ => return Err(DescriptorError::UnknownSelector),
            };
            Ok(EnableBit { reg: AHBEN, bit })
        }
    }
}

/// The enable register + bit for an I2C instance (both families: I2C0 = APB1EN bit 21,
/// I2C1 = APB1EN bit 22).
fn i2c_enable_bit(i2c: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match i2c {
        PeriphLabel::I2c0 => Ok(EnableBit {
            reg: APB1EN,
            bit: 21,
        }),
        PeriphLabel::I2c1 => Ok(EnableBit {
            reg: APB1EN,
            bit: 22,
        }),
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The enable register + bit for a SPI instance (both families: SPI0 = APB2EN bit 12,
/// SPI1 = APB1EN bit 14).
fn spi_enable_bit(spi: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match spi {
        PeriphLabel::Spi0 => Ok(EnableBit {
            reg: APB2EN,
            bit: 12,
        }),
        PeriphLabel::Spi1 => Ok(EnableBit {
            reg: APB1EN,
            bit: 14,
        }),
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The enable register + bit for an ADC instance (both families: ADC0 = APB2EN bit 9,
/// ADC1 = APB2EN bit 10; the F1x0 single ADC is the unnumbered bit 9, mapped to `Adc0`).
fn adc_enable_bit(adc: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match adc {
        PeriphLabel::Adc0 => Ok(EnableBit {
            reg: APB2EN,
            bit: 9,
        }),
        PeriphLabel::Adc1 => Ok(EnableBit {
            reg: APB2EN,
            bit: 10,
        }),
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The enable register + bit for an advanced timer under the selected path (both families:
/// TIMER0 = APB2EN bit 11; F10x additionally has TIMER7 = APB2EN bit 13).
fn timer_enable_bit(path: ClockPath, timer: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match (path, timer) {
        // TIMER0 is APB2EN bit 11 on both families.
        (_, PeriphLabel::Timer0) => Ok(EnableBit {
            reg: APB2EN,
            bit: 11,
        }),
        // TIMER7 exists only on F10x (APB2EN bit 13); F1x0 has no TIMER7.
        (ClockPath::F10xRcc, PeriphLabel::Timer7) => Ok(EnableBit {
            reg: APB2EN,
            bit: 13,
        }),
        (ClockPath::F1x0Rcu, PeriphLabel::Timer7) => Err(DescriptorError::SelectorAddrMismatch),
        // Not a timer label.
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The enable register + bit for a GENERAL-purpose timer (G3). TIMER1 is on **APB1EN bit 0** on both
/// families. A non-general-timer label (including the advanced `Timer0`/`Timer7`) is
/// [`DescriptorError::UnknownSelector`].
fn general_timer_enable_bit(timer: PeriphLabel) -> Result<EnableBit, DescriptorError> {
    match timer {
        // TIMER1 (the general level-0 timer) is APB1EN bit 0 on both families.
        PeriphLabel::Timer1 => Ok(EnableBit {
            reg: APB1EN,
            bit: 0,
        }),
        _ => Err(DescriptorError::UnknownSelector),
    }
}

// --- ADC clock prescaler (the per-family piece of the ADC clock enable) -----------------------

/// `RCU_CFG2` offset (F1x0 only; holds ADCSEL among other kernel-clock selects).
const CFG2: u32 = 0x30;
/// `RCU_CFG0` ADCPSC field, bits[15:14] (both families).
const CFG0_ADCPSC: u32 = 0b11 << 14;
/// ADC prescaler default: CK_APB2 / 6 (code 2 in ADCPSC). 72 MHz APB2 / 6 = 12 MHz, within the
/// 14 MHz ADC clock ceiling. Matches `RCU_(CK)ADC_CKAPB2_DIV6` in both families' SPL.
const ADCPSC_APB2_DIV6: u32 = 2 << 14;
/// `RCU_CFG2` ADCSEL (bit 8): selects the APB2-derived ADC clock on F1x0 (set = APB2/div source).
const CFG2_ADCSEL: u32 = 1 << 8;

/// Set the ADC clock prescaler (and on F1x0 the ADC clock source select), mirroring the SPL
/// `rcu_adc_clock_config(RCU_(CK)ADC_CKAPB2_DIV6)`.
///
/// Both families RMW `RCU_CFG0` ADCPSC to the /6 code. F1x0 additionally RMWs `RCU_CFG2` ADCSEL to
/// select the APB2-derived clock (the F1x0 SPL clears then sets ADCSEL); F10x has no such select.
fn adc_clock_config(rcu_base: u32, path: ClockPath) {
    Reg32::new(rcu_base, CFG0).modify(CFG0_ADCPSC, ADCPSC_APB2_DIV6);
    if let ClockPath::F1x0Rcu = path {
        Reg32::new(rcu_base, CFG2).modify(CFG2_ADCSEL, CFG2_ADCSEL);
    }
}

// --- T2: the full descriptor-driven clock tree ------------------------------------------------
//
// `configure_tree` programs the WHOLE clock tree from a `ClockProfile`: flash wait states, the
// PLL source, the AHB/APB prescalers, the PLL multiplier, then enables the PLL and switches the
// system clock to it, polling at each gate. It reaches 72 MHz (APB1 = 36 MHz) with the default
// profile. This is the first runtime-hal path that POLLS during bring-up (source-stable, PLL-lock,
// SCS-confirm), the F130 hang-if-done-wrong class TESTING.md calls out.
//
// # Register facts (GD SPL CMSIS / peripheral headers)
//
// RCU CTL (the control register holding the oscillator enable + stable + PLL enable/lock bits) is
// `RCU_CTL0` on F1x0 and `RCU_CTL` on F10x, BOTH at offset `0x00` with the SAME bit positions:
//   IRC8MEN BIT(0), IRC8MSTB BIT(1), HXTALEN BIT(16), HXTALSTB BIT(17), PLLEN BIT(24), PLLSTB BIT(25).
// RCU CFG0 is at `0x04` on both, same field positions:
//   SCS BITS(0,1), SCSS BITS(2,3), AHBPSC BITS(4,7), APB1PSC BITS(8,10), APB2PSC BITS(11,13),
//   PLLSEL BIT(16), PLLMF = (BIT(27) | BITS(18,21)).
// So the f10x_rcc vs f1x0_rcu divergence here is only in the SPL recipe (F1x0 SystemInit sets
// APB1 = AHB/1; F10x sets APB1 = AHB/2) and the family-specific reset bring-up, NOT the register
// layout. The DIVERGENCE is therefore expressed entirely through the profile's prescalers and the
// per-family golden, while the register model is shared. The flash FMC_WS register is at FMC base
// `0x4002_2000` offset `0x00` on both families (AHB1 + 0x2000 on F1x0, AHB1 + 0xA000 on F10x both
// resolve to 0x4002_2000), WSCNT in BITS(0,2).

/// RCU CTL register offset (RCU_CTL0 on F1x0 / RCU_CTL on F10x; both 0x00).
const CTL: u32 = 0x00;
/// RCU CFG0 register offset (both families, 0x04).
const CFG0: u32 = 0x04;

// RCU_CTL bits (identical positions on both families).
const CTL_IRC8MEN: u32 = 1 << 0;
const CTL_IRC8MSTB: u32 = 1 << 1;
const CTL_HXTALEN: u32 = 1 << 16;
const CTL_HXTALSTB: u32 = 1 << 17;
const CTL_PLLEN: u32 = 1 << 24;
const CTL_PLLSTB: u32 = 1 << 25;

// RCU_CFG0 fields (identical positions on both families). The `<< 0` on the bit-0 fields is kept
// for visual alignment with their siblings (`<< 2`, `<< 4`, ...); it documents the field position.
#[allow(clippy::identity_op)]
const CFG0_SCS: u32 = 0b11 << 0; // system clock switch
const CFG0_SCSS: u32 = 0b11 << 2; // system clock switch status (read-back)
const CFG0_AHBPSC: u32 = 0b1111 << 4;
const CFG0_APB1PSC: u32 = 0b111 << 8;
const CFG0_APB2PSC: u32 = 0b111 << 11;
const CFG0_PLLSEL: u32 = 1 << 16;
/// PLLMF spans the high bit BIT(27) plus BITS(18,21): the multiply factor field.
const CFG0_PLLMF: u32 = (1 << 27) | (0b1111 << 18);

// SCS / SCSS encodings: 2 = PLL.
#[allow(clippy::identity_op)]
const SCS_PLL: u32 = 0b10 << 0;
const SCSS_PLL: u32 = 0b10 << 2;
/// PLL source select: IRC8M/2 = PLLSEL clear (0); HXTAL = PLLSEL set.
const PLLSRC_IRC8M_DIV2: u32 = 0;

/// The FMC wait-state register: FMC base `0x4002_2000`, `FMC_WS` at offset `0x00`, WSCNT BITS(0,2).
/// The FMC base is identical on both families, so it is a compile-time constant here (the RCU base
/// is the only address the descriptor carries for this path; the FMC is fixed by the part family
/// and both families place it at the same absolute address).
const FMC_WS_ADDR: u32 = 0x4002_2000;
const FMC_WS_WSCNT: u32 = 0b111;

/// Encode an AHB prescaler divisor into its `RCU_CFG0` AHBPSC field bits.
///
/// AHB uses code 0 for /1; /2../16 are codes 8..11; AHB then SKIPS /32, so /64../512 are codes
/// 12..15 (`RCU_AHB_CKSYS_DIV*`). The full divisor set is `{1,2,4,8,16,64,128,256,512}` (no /32);
/// `ClockConfig::validate_for` rejects anything outside it before this runs. An unexpected divisor
/// falls back to /1 (the safe, no-divide value).
#[inline]
fn ahb_psc_bits(div: u16) -> u32 {
    let code = match div {
        1 => 0,
        2 => 8,
        4 => 9,
        8 => 10,
        16 => 11,
        64 => 12,
        128 => 13,
        256 => 14,
        512 => 15,
        _ => 0,
    };
    (code as u32) << 4
}

/// Encode an APB prescaler divisor (1, 2, 4, 8, 16) into a `RCU_CFG0` APBxPSC field, at `shift`.
///
/// APB uses code 0 for /1, then `4 + log2(div)` for /2../16 (`RCU_APB1_CKAHB_DIV*`).
#[inline]
fn apb_psc_code(div: u16) -> u32 {
    match div {
        1 => 0,
        2 => 4,
        4 => 5,
        8 => 6,
        16 => 7,
        _ => 0,
    }
}

/// Encode a PLL multiply factor (integer 2..=32) into the `RCU_CFG0` PLLMF bits.
///
/// Both families use the same split: mul 2..=16 -> `(mul-2) << 18` (PLLMF4 clear); mul 17..=32 ->
/// `((mul-17) << 18) | BIT(27)` (PLLMF4 set). The default recipe is mul 18 -> `BIT(27) | (1 << 18)`
/// = `0x0804_0000` (IRC8M/2 = 4 MHz, *18 = 72 MHz), exactly `RCU_PLL_MUL18`.
#[inline]
fn pll_mul_bits(mul: u8) -> u32 {
    let m = mul as u32;
    if (2..=16).contains(&m) {
        (m - 2) << 18
    } else {
        // 17..=32 (and any out-of-range clamps to the high block via wrapping; callers pass valid).
        (1 << 27) | ((m.wrapping_sub(17)) << 18)
    }
}

/// Program the full clock tree from a [`ClockConfig`], reaching the config's `sysclk_hz` (72 MHz
/// with the default profile, APB1 = 36 MHz). This is the descriptor-driven replacement for the
/// SPL's `SystemInit` 72 MHz recipe; the [`ClockPath`] selector is accepted for symmetry with the
/// other paths and to document the family, but the RCU register layout is shared (see the module
/// note), so the family divergence lives in the profile's prescalers and the per-family golden.
///
/// Ordered steps (the gates that POLL are marked):
/// 1. **Flash wait states FIRST** (before raising the clock): set FMC_WS.WSCNT = `wait_states`.
///    Done first so the flash can keep up once the core speeds up (a wrong-order setup that raises
///    the clock before the wait states reads corrupt flash).
/// 2. Enable the selected source and **POLL for it to stabilise** (IRC8M: IRC8MEN -> IRC8MSTB;
///    HXTAL: HXTALEN -> HXTALSTB).
/// 3. Program AHB / APB2 / APB1 prescalers (RMW the three CFG0 fields).
/// 4. Program the PLL source (IRC8M/2 vs HXTAL via PLLSEL) and the PLL multiplier (PLLMF).
/// 5. Enable the PLL and **POLL for PLL lock** (PLLEN -> PLLSTB).
/// 6. Switch the system clock source to PLL (SCS = PLL) and **POLL the read-back** (SCSS = PLL) to
///    confirm the switch took.
///
/// The polls use the same `Reg32::read` the rest of runtime-hal uses; under the harness they are
/// stubbed busy -> busy -> done via `read_responses` (CLOCK-1), and a dropped poll is a golden
/// failure.
///
/// This is the **SPL-faithful reference** the M2 goldens diff against: each gate spins UNBOUNDED,
/// exactly as the GD SPL `SystemInit` does. A board that never stabilises / locks hangs here, the
/// same as the SPL. For firmware robustness on a flaky board use [`configure_tree_timeout`], which
/// shares this exact register sequence but gives up after a bounded spin budget and returns a
/// [`ClockError`]; the bounded variant is host-tested only (it must NOT get an emulated golden, or
/// it would diverge from the SPL's unbounded poll).
pub fn configure_tree(chip: &Chip, clock: &ClockConfig) -> Result<(), ClockError> {
    // Validate the application's free-form ClockConfig against the chip-bound ranges (DR-1
    // mitigation), THEN run the SPL-faithful unbounded register sequence. The unbounded path shares
    // the same register sequence as the bounded one (so they cannot drift): a `None` spin budget
    // means "spin forever", which keeps the SPL-faithful blocking behaviour the goldens diff against
    // (none of the three wait gates can return Err with `None`).
    clock.validate_for(chip.clock())?;
    let rcu_base = chip.rcu_base()?;
    configure_tree_inner(rcu_base, chip.clock(), clock, None)
}

/// Default spin cap for [`configure_tree_timeout`]: iterations per wait gate before giving up.
///
/// Each gate (source-stable, PLL-lock, SCS-confirm) settles in well under a millisecond on real
/// silicon (the IRC8M is ready in a few microseconds, the PLL locks in tens of microseconds). At a
/// 72 MHz core a bare `read`-and-test loop iterates far faster than the settle time, so 1,000,000
/// spins is a generous several-millisecond ceiling: long enough that a healthy board always passes,
/// short enough that a dead oscillator / unlockable PLL fails in bounded time instead of bricking
/// the boot. Firmware that wants a tighter or looser bound passes its own `max_spins`.
pub const DEFAULT_CLOCK_SPIN_CAP: u32 = 1_000_000;

/// Bounded-timeout clock-tree bring-up: the firmware-robustness variant of [`configure_tree`].
///
/// Runs the **same ordered register sequence** as [`configure_tree`] (shared via the private
/// `configure_tree_inner`, so the two cannot drift), but each of the three wait gates (source
/// stable, PLL lock, SCS confirm) spins at most `max_spins` times before giving up with the
/// matching [`ClockError`]. This lets firmware on a board whose oscillator never stabilises or PLL
/// never locks fail cleanly (and e.g. fall back to the internal RC, latch a fault, or reset) instead
/// of hanging forever in the unbounded poll.
///
/// `max_spins` is per-gate. Pass [`DEFAULT_CLOCK_SPIN_CAP`] for a sensible default. On success the
/// register writes are byte-for-byte the same as the unbounded path's (the only difference is the
/// loop exit condition), which the host tests assert.
///
/// NOTE: this variant is **host-tested only** and deliberately has NO emulated golden. The M2
/// goldens diff runtime-hal against the SPL's UNBOUNDED `SystemInit` poll; an emulated trace of the
/// bounded variant would diverge from that reference. Keep [`configure_tree`] as the golden-fidelity
/// path and use this one only for on-target robustness.
pub fn configure_tree_timeout(
    chip: &Chip,
    clock: &ClockConfig,
    max_spins: u32,
) -> Result<(), ClockError> {
    clock.validate_for(chip.clock())?;
    let rcu_base = chip.rcu_base()?;
    configure_tree_inner(rcu_base, chip.clock(), clock, Some(max_spins))
}

/// Spin until `cond()` is true. `budget == None` spins forever (the SPL-faithful unbounded gate);
/// `Some(n)` gives up after `n` iterations and returns `Err(())` so the caller maps it to the gate's
/// [`ClockError`]. Shared by both entry points so the bounded and unbounded waits use identical
/// exit conditions.
#[inline]
fn spin_until(budget: Option<u32>, mut cond: impl FnMut() -> bool) -> Result<(), ()> {
    match budget {
        None => {
            while !cond() {}
            Ok(())
        }
        Some(mut left) => {
            while !cond() {
                if left == 0 {
                    return Err(());
                }
                left -= 1;
            }
            Ok(())
        }
    }
}

/// The shared clock-tree register sequence. `budget` selects unbounded (`None`, the SPL-faithful
/// [`configure_tree`]) vs bounded (`Some(max_spins)`, [`configure_tree_timeout`]) waits. The MMIO
/// writes are identical in both modes; only the wait-gate exit differs, so there is a single source
/// of truth for the sequence the goldens pin.
fn configure_tree_inner(
    rcu_base: u32,
    path: ClockPath,
    profile: &ClockConfig,
    budget: Option<u32>,
) -> Result<(), ClockError> {
    let _ = path; // shared register model; the family divergence is in the profile + golden.

    let ctl = Reg32::new(rcu_base, CTL);
    let cfg0 = Reg32::new(rcu_base, CFG0);

    // 1. Flash wait states first.
    Reg32::new(FMC_WS_ADDR, 0).modify(FMC_WS_WSCNT, profile.wait_states as u32 & FMC_WS_WSCNT);

    // 2. Enable + stabilise the source.
    match profile.source {
        ClockSource::Irc8m => {
            ctl.modify(CTL_IRC8MEN, CTL_IRC8MEN);
            spin_until(budget, || ctl.read() & CTL_IRC8MSTB != 0)
                .map_err(|()| ClockError::SourceNotStable)?;
        }
        ClockSource::Hxtal => {
            ctl.modify(CTL_HXTALEN, CTL_HXTALEN);
            spin_until(budget, || ctl.read() & CTL_HXTALSTB != 0)
                .map_err(|()| ClockError::SourceNotStable)?;
        }
    }

    // 3. Prescalers: AHB, APB2, APB1.
    cfg0.modify(CFG0_AHBPSC, ahb_psc_bits(profile.ahb_psc));
    cfg0.modify(CFG0_APB2PSC, apb_psc_code(profile.apb2_psc) << 11);
    cfg0.modify(CFG0_APB1PSC, apb_psc_code(profile.apb1_psc) << 8);

    // 4. PLL source + multiplier.
    let pllsel = match profile.source {
        ClockSource::Irc8m => PLLSRC_IRC8M_DIV2,
        ClockSource::Hxtal => CFG0_PLLSEL,
    };
    cfg0.modify(CFG0_PLLSEL, pllsel);
    cfg0.modify(CFG0_PLLMF, pll_mul_bits(profile.pll_mul));

    // 5. Enable PLL, wait for lock.
    ctl.modify(CTL_PLLEN, CTL_PLLEN);
    spin_until(budget, || ctl.read() & CTL_PLLSTB != 0).map_err(|()| ClockError::PllNotLocked)?;

    // 6. Switch system clock to PLL, confirm via SCSS read-back.
    cfg0.modify(CFG0_SCS, SCS_PLL);
    spin_until(budget, || cfg0.read() & CFG0_SCSS == SCSS_PLL)
        .map_err(|()| ClockError::SwitchNotConfirmed)?;

    Ok(())
}

#[cfg(test)]
mod tests;
