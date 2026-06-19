//! Peripheral labels and the base-address table.
//!
//! SPEC.md "Naming convention": runtime-hal uses GigaDevice **0-indexed** labels internally
//! (`USART0` = ST `USART1`). The descriptor carries one base per label as data; the selected
//! path supplies the offsets/bitfields within the peripheral. This is the "data axis": a new
//! part with the same register models but different addresses is pure config.
//!
//! T1 scope (M1 open item 5): the labels M1 resolves are `USART0/1/2`, `GPIOA..GPIOF`, and the
//! `RCU`/`RCC` base.

use crate::descriptor::{ClockPath, GpioPath};
use crate::error::DescriptorError;

/// Peripheral labels the M1 milestone resolves.
///
/// `#[repr(u8)]` and contiguous from 0 so the discriminant doubles as the index into
/// [`AddrTable`]'s backing array. Extend toward the end as later milestones add peripherals;
/// keep the order stable so the index stays meaningful.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeriphLabel {
    /// USART0 (ST `USART1`).
    Usart0 = 0,
    /// USART1 (ST `USART2`).
    Usart1 = 1,
    /// USART2 (ST `USART3`).
    Usart2 = 2,
    /// GPIO port A base.
    Gpioa = 3,
    /// GPIO port B base.
    Gpiob = 4,
    /// GPIO port C base.
    Gpioc = 5,
    /// GPIO port D base.
    Gpiod = 6,
    /// GPIO port E base.
    Gpioe = 7,
    /// GPIO port F base.
    Gpiof = 8,
    /// Reset & clock unit base (GD `RCU` / ST `RCC`); one base, the clock path owns the offsets.
    Rcu = 9,
    // --- M2 (T1) cold-path additions. APPENDED after `Rcu` so the M1 discriminants (0..=9) and
    // their `AddrTable` indices stay stable (T1 index-stability invariant). The CBOR `addrs` key
    // range grows from 0..=9 to 0..=15 additively (DECISIONS.md #3).
    /// I2C0 (the on-board IMU link on the bench F130; classic event-based block on both families).
    I2c0 = 10,
    /// I2C1 (F10x second instance; F1x0 has only the single I2C block, mapped as `I2c0`).
    I2c1 = 11,
    /// SPI0 (on APB2 on F10x; F1x0's single SPI block is mapped as `Spi0`).
    Spi0 = 12,
    /// SPI1 (on APB1 on F10x).
    Spi1 = 13,
    /// ADC0 (both families; F1x0's single ADC is `Adc0`).
    Adc0 = 14,
    /// ADC1 (F10x second ADC; M2 brings up `Adc0` single on both, `Adc1` carried for completeness).
    Adc1 = 15,
    // --- M3 (T1) hot-path additions. APPENDED after the M2 labels so the M1/M2 discriminants
    // (0..=15) and their `AddrTable` indices stay stable (the index-stability invariant). The CBOR
    // `addrs` key range grows from 0..=15 to 0..=17 additively (DECISIONS.md #3).
    /// TIMER0 (the advanced timer driving the complementary PWM bridge; ST `TIM1`). On APB2 at
    /// `0x4001_2C00` on both families. The motor hot path's PWM timer.
    Timer0 = 16,
    /// TIMER7 (the second advanced timer on parts that declare `adv_timers == 2`; ST `TIM8`).
    /// Carried for completeness; the reference F1x0 board has only `Timer0`.
    Timer7 = 17,
    // --- G-WDG addition. APPENDED after `Timer7` so the M1/M2/M3 discriminants (0..=17) and their
    // `AddrTable` indices stay stable (the index-stability invariant). Additive, DECISIONS.md #3.
    /// FWDGT (the free / independent watchdog; ST `IWDG`). On APB1 at `0x4000_3000` on BOTH families.
    /// The register block is identical on both, so one model parameterised by this base drives it.
    Fwdgt = 18,
}

impl PeriphLabel {
    /// Number of labels; the [`AddrTable`] capacity.
    pub const COUNT: usize = 19;

    /// Index of this label into the address-table backing array.
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// True for a `GPIOx` label (used by the per-path range check).
    #[inline]
    pub const fn is_gpio(self) -> bool {
        matches!(
            self,
            PeriphLabel::Gpioa
                | PeriphLabel::Gpiob
                | PeriphLabel::Gpioc
                | PeriphLabel::Gpiod
                | PeriphLabel::Gpioe
                | PeriphLabel::Gpiof
        )
    }

    /// True for an `I2Cx` label (M2 T1).
    #[inline]
    pub const fn is_i2c(self) -> bool {
        matches!(self, PeriphLabel::I2c0 | PeriphLabel::I2c1)
    }

    /// True for a `SPIx` label (M2 T1).
    #[inline]
    pub const fn is_spi(self) -> bool {
        matches!(self, PeriphLabel::Spi0 | PeriphLabel::Spi1)
    }

    /// True for an `ADCx` label (M2 T1).
    #[inline]
    pub const fn is_adc(self) -> bool {
        matches!(self, PeriphLabel::Adc0 | PeriphLabel::Adc1)
    }

    /// True for an advanced-timer label (M3 T1): `Timer0` / `Timer7`.
    #[inline]
    pub const fn is_timer(self) -> bool {
        matches!(self, PeriphLabel::Timer0 | PeriphLabel::Timer7)
    }

    /// True for the free-watchdog label (G-WDG): `Fwdgt`.
    #[inline]
    pub const fn is_fwdgt(self) -> bool {
        matches!(self, PeriphLabel::Fwdgt)
    }
}

/// Per-path expected base-address ranges, used by [`AddrTable::check_ranges`].
///
/// Tightened in T3/T4 from the User Manual / GD SPL memory maps:
/// - F10x GPIO lives on **APB2** at `0x4001_0800` (GPIOA) with a `0x400` per-port stride, up to
///   GPIOG at `0x4001_1C00` (`gd32f10x.h`: `GPIO_BASE = APB2 0x4001_0800`).
/// - F1x0 GPIO lives on **AHB2** at `0x4800_0000` (GPIOA), same `0x400` stride
///   (`gd32f1x0.h`: `GPIO_BASE = AHB2 0x4800_0000`).
/// - The RCU/RCC base is `0x4002_1000` on both families (`gd32f10x.h` / `gd32f1x0.h`:
///   `RCU_BASE`); the clock tree below it diverges but the base does not.
///
/// So `gpio = ahb_ctl_afsel` paired with an APB (`0x4001_xxxx`) GPIO base, or `apb_crl_crh` paired
/// with an AHB (`0x4800_xxxx`) GPIO base, is a [`DescriptorError::SelectorAddrMismatch`]: each gpio
/// path declares the bus its ports sit on.
pub(crate) mod ranges {
    /// (inclusive_lo, exclusive_hi) for a GPIO base under the F10x `apb_crl_crh` path: APB2,
    /// six ports (A..F for M1; the window covers up to GPIOG) of `0x400` each from `0x4001_0800`.
    pub const GPIO_F10X_APB: (u32, u32) = (0x4001_0800, 0x4001_2000);
    /// (inclusive_lo, exclusive_hi) for a GPIO base under the F1x0 `ahb_ctl_afsel` path: AHB2,
    /// ports of `0x400` each from `0x4800_0000`.
    pub const GPIO_F1X0_AHB: (u32, u32) = (0x4800_0000, 0x4800_2000);
    /// (inclusive_lo, exclusive_hi) for the RCU/RCC base (both clock paths share the base).
    pub const RCU: (u32, u32) = (0x4002_1000, 0x4002_1400);

    /// (inclusive_lo, exclusive_hi) for a USART base on **APB1**. Both families place APB1 at
    /// `0x4000_0000`; the USART block starts at `+0x4400` (USART1) with `0x400` per instance
    /// (USART2 at `+0x4800`). (`gd32f10x.h` / `gd32f1x0.h`: `APB1_BUS_BASE = 0x4000_0000`,
    /// `USART_BASE = APB1 + 0x4400`.) The window covers the USART/UART block on APB1.
    pub const USART_APB1: (u32, u32) = (0x4000_4400, 0x4000_5000);
    /// (inclusive_lo, exclusive_hi) for a USART base on **APB2** (USART0). Both families place APB2
    /// at `0x4001_0000`; GD `USART0` is `USART_BASE + 0xF400 = 0x4001_3800` (`gd32f10x_usart.h:48`
    /// / `gd32f1x0_usart.h:46`).
    pub const USART_APB2: (u32, u32) = (0x4001_3800, 0x4001_3C00);

    // --- M2 (T1) cold-path peripheral windows. Bases confirmed against the GD SPL CMSIS headers
    // (gd32f10x.h / gd32f1x0.h) and the per-instance offsets in the SPL peripheral headers:
    //   I2C block  : APB1 + 0x5400 = 0x4000_5400 (I2C0), I2C1 at +0x400 = 0x4000_5800 (F10x).
    //   SPI0       : APB2 (F10x SPI0 = SPI_BASE + 0xF800 = 0x4001_3000).
    //   SPI1       : APB1 (F10x SPI1 = SPI_BASE = 0x4000_3800; the F1x0 single SPI is here too).
    //   ADC block  : APB2 + 0x2400 = 0x4001_2400 (ADC0), ADC1 at +0x400 = 0x4001_2800 (F10x).
    // The APB bus bases are identical across both families, so these windows are family-independent.

    /// (inclusive_lo, exclusive_hi) for an I2C base on **APB1**. I2C0 = `0x4000_5400`, I2C1 =
    /// `0x4000_5800` (F10x); the window spans both instances.
    pub const I2C_APB1: (u32, u32) = (0x4000_5400, 0x4000_5C00);
    /// (inclusive_lo, exclusive_hi) for SPI0 on **APB2** (F10x SPI0 = `0x4001_3000`). The F1x0
    /// single SPI block sits on APB1 (mapped as `Spi0` there), so a `Spi0` base in the APB1 SPI
    /// window is also accepted; see [`AddrTable::check_spi_base`].
    pub const SPI0_APB2: (u32, u32) = (0x4001_3000, 0x4001_3400);
    /// (inclusive_lo, exclusive_hi) for a SPI base on **APB1** (F10x SPI1 = `0x4000_3800`; the
    /// F1x0 single SPI block = `0x4000_3800`). SPI2 (`+0x400`) is in the window but not labelled.
    pub const SPI_APB1: (u32, u32) = (0x4000_3800, 0x4000_4000);
    /// (inclusive_lo, exclusive_hi) for an ADC base on **APB2**. ADC0 = `0x4001_2400`, ADC1 =
    /// `0x4001_2800` (F10x); the window spans both.
    pub const ADC_APB2: (u32, u32) = (0x4001_2400, 0x4001_2C00);

    // --- M3 (T1) advanced-timer window. Confirmed against the GD SPL CMSIS headers
    // (gd32f10x.h / gd32f1x0.h: `TIMER_BASE = APB1_BUS_BASE + 0`, `TIMER0 = TIMER_BASE + 0x12C00`)
    // and the per-timer offsets in the SPL peripheral headers (gd32f10x_timer.h / gd32f1x0_timer.h):
    //   TIMER0 : APB2 = TIMER_BASE + 0x12C00 = 0x4001_2C00 (the advanced timer, ST TIM1).
    //   TIMER7 : APB2 = TIMER_BASE + 0x13400 = 0x4001_3400 (F10x second advanced timer, ST TIM8).
    // Both advanced timers sit on APB2 just above the ADC window; the bases are family-independent.

    /// (inclusive_lo, exclusive_hi) for an advanced-timer base on **APB2**. TIMER0 = `0x4001_2C00`,
    /// TIMER7 = `0x4001_3400` (F10x); the window spans both advanced-timer instances. (The window
    /// stops below USART0 at `0x4001_3800`, which is its own range.)
    pub const ADV_TIMER_APB2: (u32, u32) = (0x4001_2C00, 0x4001_3800);

    // --- G-WDG free-watchdog window. Confirmed against the GD SPL CMSIS headers (gd32f10x.h /
    // gd32f1x0.h: `FWDGT_BASE = APB1_BUS_BASE + 0x3000 = 0x4000_3000`). The base is identical on
    // both families, so this window is family-independent.

    /// (inclusive_lo, exclusive_hi) for the FWDGT base on **APB1**: `0x4000_3000`, a single 0x400
    /// peripheral slot (the next slot at `0x4000_3400` is SPI/the F1x0 SPI block).
    pub const FWDGT_APB1: (u32, u32) = (0x4000_3000, 0x4000_3400);

    /// The GPIO range the selected gpio path expects.
    pub const fn gpio_for(gpio: crate::descriptor::GpioPath) -> (u32, u32) {
        match gpio {
            crate::descriptor::GpioPath::ApbCrlCrh => GPIO_F10X_APB,
            crate::descriptor::GpioPath::AhbCtlAfsel => GPIO_F1X0_AHB,
        }
    }
}

/// Base address per peripheral label, indexed by [`PeriphLabel::index`].
///
/// `[Option<u32>; N]` (DECISIONS.md #1): bounded, owned, no alloc. `None` means "this part has
/// no such instance / the descriptor did not carry it".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddrTable {
    bases: [Option<u32>; PeriphLabel::COUNT],
}

impl Default for AddrTable {
    fn default() -> Self {
        Self::new()
    }
}

impl AddrTable {
    /// An empty table (every label unset).
    pub const fn new() -> Self {
        Self {
            bases: [None; PeriphLabel::COUNT],
        }
    }

    /// Set the base for a label (builder-style; used by the decoder and by tests). `const` so the
    /// per-family descriptor constants ([`crate::detect::descriptor_f103`] /
    /// [`crate::detect::descriptor_f130`]) can be built in a `const fn`.
    pub const fn set(&mut self, label: PeriphLabel, base: u32) {
        self.bases[label.index()] = Some(base);
    }

    /// The raw base if present (no validation).
    #[inline]
    pub fn get(&self, label: PeriphLabel) -> Option<u32> {
        self.bases[label.index()]
    }

    /// Resolve a label to its base, erroring if the wiring needs a base the table lacks.
    #[inline]
    pub fn resolve(&self, label: PeriphLabel) -> Result<u32, DescriptorError> {
        self.get(label).ok_or(DescriptorError::MissingBase(label))
    }

    /// Validate every present base against the range its selected path expects.
    ///
    /// SPEC.md: "runtime-hal validates selector against address at parse: `gpio = ahb_ctl_afsel`
    /// paired with an APB GPIO base is rejected." The gpio range is chosen by the gpio path
    /// (APB2 for F10x, AHB2 for F1x0); the RCU base is shared by both clock paths.
    pub fn check_ranges(&self, gpio: GpioPath, clock: ClockPath) -> Result<(), DescriptorError> {
        let _ = clock; // the RCU base is the same for both clock paths; the clock tree below it diverges, not the base.
        let (lo, hi) = ranges::gpio_for(gpio);
        for label in [
            PeriphLabel::Gpioa,
            PeriphLabel::Gpiob,
            PeriphLabel::Gpioc,
            PeriphLabel::Gpiod,
            PeriphLabel::Gpioe,
            PeriphLabel::Gpiof,
        ] {
            if let Some(base) = self.get(label) {
                if base < lo || base >= hi {
                    return Err(DescriptorError::SelectorAddrMismatch);
                }
            }
        }
        if let Some(base) = self.get(PeriphLabel::Rcu) {
            let (lo, hi) = ranges::RCU;
            if base < lo || base >= hi {
                return Err(DescriptorError::SelectorAddrMismatch);
            }
        }
        Ok(())
    }

    /// Validate that a USART label's base sits on the APB bus that instance belongs to.
    ///
    /// SPEC.md's selector-vs-address consistency, applied to the USART block: GD `USART0` is on
    /// **APB2** and GD `USART1`/`USART2` are on **APB1** (the same split the clock path encodes:
    /// USART0 on `APB2EN`, USART1/2 on `APB1EN`). A descriptor that names `Usart1` but points its
    /// base into the APB2 window (or vice-versa) is a wiring mistake that would otherwise compute
    /// the wrong input clock, so it is rejected at parse as [`DescriptorError::SelectorAddrMismatch`].
    /// The APB bus bases are identical across both families, so this check does not depend on the
    /// clock path. A non-USART label is [`DescriptorError::UnknownSelector`].
    pub fn check_usart_base(&self, usart: PeriphLabel) -> Result<(), DescriptorError> {
        let (lo, hi) = match usart {
            PeriphLabel::Usart0 => ranges::USART_APB2,
            PeriphLabel::Usart1 | PeriphLabel::Usart2 => ranges::USART_APB1,
            _ => return Err(DescriptorError::UnknownSelector),
        };
        match self.get(usart) {
            Some(base) if base >= lo && base < hi => Ok(()),
            Some(_) => Err(DescriptorError::SelectorAddrMismatch),
            None => Err(DescriptorError::MissingBase(usart)),
        }
    }

    /// Validate that an I2C label's base sits in the APB1 I2C window (M2 T1).
    ///
    /// Both `I2c0` and `I2c1` are on APB1 on both families (the F1x0 single I2C block is `I2c0`).
    /// A base outside the window is [`DescriptorError::SelectorAddrMismatch`]; a non-I2C label is
    /// [`DescriptorError::UnknownSelector`]. The bus tasks (T6/T7) consume this at parse time.
    pub fn check_i2c_base(&self, i2c: PeriphLabel) -> Result<(), DescriptorError> {
        if !i2c.is_i2c() {
            return Err(DescriptorError::UnknownSelector);
        }
        let (lo, hi) = ranges::I2C_APB1;
        match self.get(i2c) {
            Some(base) if base >= lo && base < hi => Ok(()),
            Some(_) => Err(DescriptorError::SelectorAddrMismatch),
            None => Err(DescriptorError::MissingBase(i2c)),
        }
    }

    /// Validate that a SPI label's base sits on the bus that instance belongs to (M2 T1).
    ///
    /// On F10x, `Spi0` is on APB2 and `Spi1` is on APB1. The F1x0 single SPI block sits on APB1,
    /// and is mapped to the `Spi0` label, so a `Spi0` base in either the APB2 SPI0 window or the
    /// APB1 SPI window is accepted (the family is not known here, only the label and base). A base
    /// outside both windows is [`DescriptorError::SelectorAddrMismatch`]; a non-SPI label is
    /// [`DescriptorError::UnknownSelector`].
    pub fn check_spi_base(&self, spi: PeriphLabel) -> Result<(), DescriptorError> {
        let base = match spi {
            PeriphLabel::Spi0 => {
                let b = self.get(spi).ok_or(DescriptorError::MissingBase(spi))?;
                let (lo2, hi2) = ranges::SPI0_APB2;
                let (lo1, hi1) = ranges::SPI_APB1;
                if (b >= lo2 && b < hi2) || (b >= lo1 && b < hi1) {
                    return Ok(());
                }
                return Err(DescriptorError::SelectorAddrMismatch);
            }
            PeriphLabel::Spi1 => self.get(spi).ok_or(DescriptorError::MissingBase(spi))?,
            _ => return Err(DescriptorError::UnknownSelector),
        };
        let (lo, hi) = ranges::SPI_APB1;
        if base >= lo && base < hi {
            Ok(())
        } else {
            Err(DescriptorError::SelectorAddrMismatch)
        }
    }

    /// Validate that an ADC label's base sits in the APB2 ADC window (M2 T1).
    ///
    /// Both `Adc0` and `Adc1` are on APB2 on both families (the F1x0 single ADC is `Adc0`). A base
    /// outside the window is [`DescriptorError::SelectorAddrMismatch`]; a non-ADC label is
    /// [`DescriptorError::UnknownSelector`].
    pub fn check_adc_base(&self, adc: PeriphLabel) -> Result<(), DescriptorError> {
        if !adc.is_adc() {
            return Err(DescriptorError::UnknownSelector);
        }
        let (lo, hi) = ranges::ADC_APB2;
        match self.get(adc) {
            Some(base) if base >= lo && base < hi => Ok(()),
            Some(_) => Err(DescriptorError::SelectorAddrMismatch),
            None => Err(DescriptorError::MissingBase(adc)),
        }
    }

    /// Validate that an advanced-timer label's base sits in the APB2 advanced-timer window (M3 T1).
    ///
    /// Both `Timer0` and `Timer7` are on APB2 on both families (TIMER0 = `0x4001_2C00`, the F1x0
    /// reference board's only advanced timer). A base outside the window is
    /// [`DescriptorError::SelectorAddrMismatch`]; a non-timer label is
    /// [`DescriptorError::UnknownSelector`]. The hot-path timer tasks (T3+) consume this at parse
    /// time, the same shape as [`Self::check_adc_base`].
    pub fn check_timer_base(&self, timer: PeriphLabel) -> Result<(), DescriptorError> {
        if !timer.is_timer() {
            return Err(DescriptorError::UnknownSelector);
        }
        let (lo, hi) = ranges::ADV_TIMER_APB2;
        match self.get(timer) {
            Some(base) if base >= lo && base < hi => Ok(()),
            Some(_) => Err(DescriptorError::SelectorAddrMismatch),
            None => Err(DescriptorError::MissingBase(timer)),
        }
    }

    /// Validate that the FWDGT label's base sits in the APB1 free-watchdog window (G-WDG).
    ///
    /// `Fwdgt` is at `0x4000_3000` on both families. A base outside the window is
    /// [`DescriptorError::SelectorAddrMismatch`]; a non-FWDGT label is
    /// [`DescriptorError::UnknownSelector`]. Same shape as [`Self::check_timer_base`].
    pub fn check_fwdgt_base(&self, fwdgt: PeriphLabel) -> Result<(), DescriptorError> {
        if !fwdgt.is_fwdgt() {
            return Err(DescriptorError::UnknownSelector);
        }
        let (lo, hi) = ranges::FWDGT_APB1;
        match self.get(fwdgt) {
            Some(base) if base >= lo && base < hi => Ok(()),
            Some(_) => Err(DescriptorError::SelectorAddrMismatch),
            None => Err(DescriptorError::MissingBase(fwdgt)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The M1 index-stability invariant (T1): the M1 discriminants and their [`AddrTable`] indices
    /// must NOT have shifted when M2 appended the cold-path labels. A shift would silently
    /// re-key every existing descriptor blob's `addrs` map.
    #[test]
    fn m1_discriminants_are_stable() {
        assert_eq!(PeriphLabel::Usart0 as u8, 0);
        assert_eq!(PeriphLabel::Usart1 as u8, 1);
        assert_eq!(PeriphLabel::Usart2 as u8, 2);
        assert_eq!(PeriphLabel::Gpioa as u8, 3);
        assert_eq!(PeriphLabel::Gpiob as u8, 4);
        assert_eq!(PeriphLabel::Gpioc as u8, 5);
        assert_eq!(PeriphLabel::Gpiod as u8, 6);
        assert_eq!(PeriphLabel::Gpioe as u8, 7);
        assert_eq!(PeriphLabel::Gpiof as u8, 8);
        assert_eq!(PeriphLabel::Rcu as u8, 9);
        // M2 labels APPENDED after Rcu.
        assert_eq!(PeriphLabel::I2c0 as u8, 10);
        assert_eq!(PeriphLabel::I2c1 as u8, 11);
        assert_eq!(PeriphLabel::Spi0 as u8, 12);
        assert_eq!(PeriphLabel::Spi1 as u8, 13);
        assert_eq!(PeriphLabel::Adc0 as u8, 14);
        assert_eq!(PeriphLabel::Adc1 as u8, 15);
        // M3 labels APPENDED after Adc1.
        assert_eq!(PeriphLabel::Timer0 as u8, 16);
        assert_eq!(PeriphLabel::Timer7 as u8, 17);
        // G-WDG label APPENDED after Timer7.
        assert_eq!(PeriphLabel::Fwdgt as u8, 18);
        // index() doubles as the discriminant.
        assert_eq!(PeriphLabel::Rcu.index(), 9);
        assert_eq!(PeriphLabel::Adc1.index(), 15);
        assert_eq!(PeriphLabel::Timer0.index(), 16);
        assert_eq!(PeriphLabel::Timer7.index(), 17);
        assert_eq!(PeriphLabel::Fwdgt.index(), 18);
        // COUNT grew to cover the new labels.
        assert_eq!(PeriphLabel::COUNT, 19);
    }

    #[test]
    fn class_helpers() {
        assert!(PeriphLabel::I2c0.is_i2c() && PeriphLabel::I2c1.is_i2c());
        assert!(PeriphLabel::Spi0.is_spi() && PeriphLabel::Spi1.is_spi());
        assert!(PeriphLabel::Adc0.is_adc() && PeriphLabel::Adc1.is_adc());
        assert!(PeriphLabel::Timer0.is_timer() && PeriphLabel::Timer7.is_timer());
        assert!(PeriphLabel::Fwdgt.is_fwdgt());
        assert!(!PeriphLabel::Rcu.is_i2c());
        assert!(!PeriphLabel::Usart1.is_spi());
        assert!(!PeriphLabel::Adc0.is_timer());
        assert!(!PeriphLabel::Timer0.is_fwdgt());
    }

    #[test]
    fn check_fwdgt_base_accepts_apb1_window_and_rejects_others() {
        let mut t = AddrTable::new();
        // FWDGT = 0x4000_3000 on both families.
        t.set(PeriphLabel::Fwdgt, 0x4000_3000);
        assert_eq!(t.check_fwdgt_base(PeriphLabel::Fwdgt), Ok(()));
        // A SPI base (0x4000_3800) is above the FWDGT slot: a mismatch.
        t.set(PeriphLabel::Fwdgt, 0x4000_3800);
        assert_eq!(
            t.check_fwdgt_base(PeriphLabel::Fwdgt),
            Err(DescriptorError::SelectorAddrMismatch)
        );
        // A non-FWDGT label is UnknownSelector.
        assert_eq!(
            t.check_fwdgt_base(PeriphLabel::Timer0),
            Err(DescriptorError::UnknownSelector)
        );
    }

    #[test]
    fn check_timer_base_accepts_apb2_window() {
        let mut t = AddrTable::new();
        // TIMER0 = 0x4001_2C00, TIMER7 = 0x4001_3400 (both on APB2).
        t.set(PeriphLabel::Timer0, 0x4001_2C00);
        t.set(PeriphLabel::Timer7, 0x4001_3400);
        assert_eq!(t.check_timer_base(PeriphLabel::Timer0), Ok(()));
        assert_eq!(t.check_timer_base(PeriphLabel::Timer7), Ok(()));
    }

    #[test]
    fn check_timer_base_rejects_out_of_window_and_non_timer() {
        let mut t = AddrTable::new();
        // An ADC base (0x4001_2400) is below the advanced-timer window: a mismatch.
        t.set(PeriphLabel::Timer0, 0x4001_2400);
        assert_eq!(
            t.check_timer_base(PeriphLabel::Timer0),
            Err(DescriptorError::SelectorAddrMismatch)
        );
        // USART0 base (0x4001_3800) is just above the window: also a mismatch.
        t.set(PeriphLabel::Timer0, 0x4001_3800);
        assert_eq!(
            t.check_timer_base(PeriphLabel::Timer0),
            Err(DescriptorError::SelectorAddrMismatch)
        );
        assert_eq!(
            t.check_timer_base(PeriphLabel::Adc0),
            Err(DescriptorError::UnknownSelector)
        );
        assert_eq!(
            t.check_timer_base(PeriphLabel::Timer7),
            Err(DescriptorError::MissingBase(PeriphLabel::Timer7))
        );
    }

    #[test]
    fn check_i2c_base_accepts_apb1_window() {
        let mut t = AddrTable::new();
        t.set(PeriphLabel::I2c0, 0x4000_5400); // I2C0
        t.set(PeriphLabel::I2c1, 0x4000_5800); // I2C1
        assert_eq!(t.check_i2c_base(PeriphLabel::I2c0), Ok(()));
        assert_eq!(t.check_i2c_base(PeriphLabel::I2c1), Ok(()));
    }

    #[test]
    fn check_i2c_base_rejects_out_of_window_and_non_i2c() {
        let mut t = AddrTable::new();
        t.set(PeriphLabel::I2c0, 0x4001_3000); // a SPI0 base, wrong window
        assert_eq!(
            t.check_i2c_base(PeriphLabel::I2c0),
            Err(DescriptorError::SelectorAddrMismatch)
        );
        assert_eq!(
            t.check_i2c_base(PeriphLabel::Rcu),
            Err(DescriptorError::UnknownSelector)
        );
        assert_eq!(
            t.check_i2c_base(PeriphLabel::I2c1),
            Err(DescriptorError::MissingBase(PeriphLabel::I2c1))
        );
    }

    #[test]
    fn check_spi_base_handles_both_buses() {
        let mut t = AddrTable::new();
        // F10x: SPI0 on APB2.
        t.set(PeriphLabel::Spi0, 0x4001_3000);
        assert_eq!(t.check_spi_base(PeriphLabel::Spi0), Ok(()));
        // F1x0: the single SPI block on APB1, mapped as Spi0; also accepted.
        t.set(PeriphLabel::Spi0, 0x4000_3800);
        assert_eq!(t.check_spi_base(PeriphLabel::Spi0), Ok(()));
        // SPI1 on APB1.
        t.set(PeriphLabel::Spi1, 0x4000_3800);
        assert_eq!(t.check_spi_base(PeriphLabel::Spi1), Ok(()));
        // SPI1 in the APB2 SPI0 window is a mismatch.
        t.set(PeriphLabel::Spi1, 0x4001_3000);
        assert_eq!(
            t.check_spi_base(PeriphLabel::Spi1),
            Err(DescriptorError::SelectorAddrMismatch)
        );
        assert_eq!(
            t.check_spi_base(PeriphLabel::Adc0),
            Err(DescriptorError::UnknownSelector)
        );
    }

    #[test]
    fn check_adc_base_accepts_apb2_window() {
        let mut t = AddrTable::new();
        t.set(PeriphLabel::Adc0, 0x4001_2400);
        t.set(PeriphLabel::Adc1, 0x4001_2800);
        assert_eq!(t.check_adc_base(PeriphLabel::Adc0), Ok(()));
        assert_eq!(t.check_adc_base(PeriphLabel::Adc1), Ok(()));
        t.set(PeriphLabel::Adc0, 0x4000_5400);
        assert_eq!(
            t.check_adc_base(PeriphLabel::Adc0),
            Err(DescriptorError::SelectorAddrMismatch)
        );
    }
}
