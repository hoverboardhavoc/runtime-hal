//! The chip context (descriptor-rework DR-T3).
//!
//! The data/code split (DECISIONS.md #10) shows up in the bring-up CALL signatures: each call takes
//! (a) the chip-specific base address + selector, FROM the descriptor, and (b) the behavior, from a
//! code-level [`crate::config`] value. [`Chip`] is the chip context built once from the parsed
//! descriptor: it resolves a [`PeriphLabel`] to a base and carries the register-model selectors and
//! the RCU base, so a bring-up reads `Usart::bring_up(&chip, &clock, &UsartConfig { .. })`.
//!
//! `Chip` is just the descriptor wrapped with resolution helpers; it carries no behavior. It is the
//! single place "what silicon + where" is read, so every bring-up call pulls its chip-specific
//! inputs through it. This preserves the resolve-once intent (DECISIONS.md #4): the application
//! resolves a base once via `Chip`, constructs the handle, and the per-cycle path holds raw bases.

use crate::addr::PeriphLabel;
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::detect::Family;
use crate::error::DescriptorError;
use crate::gpio::{
    self, GpioOutput, GpioPort, PortAPins, PortBPins, PortCPins, PortDPins, PortFPins, PortPins,
};
use crate::reg::Reg32;

/// The chip context: built once from the parsed descriptor, it resolves a [`PeriphLabel`] to a base
/// and carries the register-model selectors and the RCU base. This is the descriptor's chip-only
/// data in a form the bring-up calls consume; the application keeps it for the life of the program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Chip {
    desc: McuDescriptor,
}

impl Chip {
    /// Build from a parsed descriptor (the descriptor IS the chip data now).
    #[inline]
    pub const fn from_descriptor(desc: McuDescriptor) -> Self {
        Self { desc }
    }

    /// The underlying chip descriptor.
    #[inline]
    pub const fn descriptor(&self) -> &McuDescriptor {
        &self.desc
    }

    /// The runtime-detected MCU [`Family`] ([`Family::F10x`] vs [`Family::F1x0`]).
    ///
    /// This is a DELIBERATE escape hatch from the HAL's usual rule of absorbing the family difference
    /// so the application never sees it. It exists ONLY for the peripherals the HAL deliberately does
    /// NOT abstract: architecture-specific setup such as general-purpose timer / PWM routing, where the
    /// two families diverge too far for one model (different timer catalog, different alternate-function
    /// mechanism, different modes). For everything the HAL already unifies (GPIO, USART, I2C, clock),
    /// do NOT branch on this: use the unified bring-up calls, which own the per-family branch internally
    /// (e.g. [`Chip::output_pin`] / [`Chip::gpioa`], the `Usart` / `I2c` / `Serial` bring-ups, and the
    /// [`ClockPath`] enable model). Reaching for `family()` to special-case those would re-leak the
    /// split the rest of the crate works to hide.
    ///
    /// The key differences a caller legitimately branches on (see the README's "GD32F103 (F10x) vs
    /// GD32F130 (F1x0) peripheral differences" section for the full list):
    ///
    /// - **GPIO alternate function.** F10x ([`Family::F10x`]) selects AF through the `CRL`/`CRH`
    ///   mode/cnf nibbles plus the AFIO remap groups; F1x0 ([`Family::F1x0`]) selects it per pin
    ///   through `AFSEL` and a per-pin AF mux. (For the unified GPIO paths this is already handled by
    ///   [`crate::gpio::configure_af`]; the escape hatch is for setup the HAL does not cover.)
    /// - **Timer / peripheral catalog.** The two families carry different advanced/general-purpose
    ///   timer instances and different ADC/SPI/USART instance counts, so timer/PWM routing (which timer
    ///   drives which pin, with which AF) is genuinely family-specific.
    ///
    /// Derived from the descriptor's existing register-model selector ([`McuDescriptor::gpio`]), the
    /// single source of truth: [`GpioPath::ApbCrlCrh`] is the F10x register model, so it maps to
    /// [`Family::F10x`]; [`GpioPath::AhbCtlAfsel`] is the F1x0 register model, so it maps to
    /// [`Family::F1x0`]. No redundant family field is stored.
    ///
    /// ```rust,ignore
    /// use runtime_hal::{Chip, Family};
    ///
    /// // Architecture-specific general-purpose timer / PWM routing, the kind of family-divergent
    /// // setup the HAL does NOT abstract:
    /// fn route_pwm_timer(chip: &Chip) {
    ///     match chip.family() {
    ///         Family::F10x => {
    ///             // F10x: configure the CRL/CRH AF nibble and any AFIO remap for the timer pin.
    ///         }
    ///         Family::F1x0 => {
    ///             // F1x0: set the per-pin AFSEL to the timer's AF number.
    ///         }
    ///     }
    /// }
    /// ```
    #[inline]
    pub const fn family(&self) -> Family {
        match self.desc.gpio {
            GpioPath::ApbCrlCrh => Family::F10x,
            GpioPath::AhbCtlAfsel => Family::F1x0,
        }
    }

    /// Resolve a label to its base address ([`DescriptorError::MissingBase`] if absent).
    #[inline]
    pub fn base(&self, label: PeriphLabel) -> Result<u32, DescriptorError> {
        self.desc.addrs.resolve(label)
    }

    /// The GPIO register-model path selector.
    #[inline]
    pub const fn gpio(&self) -> GpioPath {
        self.desc.gpio
    }

    /// Configure a pin as a general-purpose push-pull output and return a standard
    /// [`embedded_hal::digital::OutputPin`] handle for it.
    ///
    /// Resolves `port` to its base from the chip's address table, configures `pin` as a 50 MHz
    /// push-pull output through [`crate::gpio::configure_output`] (which owns the F10x/F1x0
    /// register-model branch internally), and returns the [`GpioOutput`] handle. Application code
    /// then drives the pin through the `embedded-hal` trait, never seeing the [`GpioPath`] split or
    /// a raw base. `pin` is the pin number (0..15) within the port. Returns
    /// [`DescriptorError::MissingBase`] if the port is not in the address table.
    #[inline]
    pub fn output_pin(&self, port: PeriphLabel, pin: u8) -> Result<GpioOutput, DescriptorError> {
        let base = self.base(port)?;
        gpio::configure_output(base, self.desc.gpio, pin);
        Ok(GpioOutput::new(base, self.desc.gpio, pin))
    }

    /// Resolve a GPIO port label to a typed [`GpioPort`], enabling its port clock.
    ///
    /// Shared body of the named getters ([`Chip::gpioa`] .. [`Chip::gpiof`]): it resolves the port's
    /// base from the chip's address table and enables the port's peripheral clock through the chip's
    /// clock path (the stm32f1xx-hal `split(&mut rcc)` clock-enable, done here so the application does
    /// not pass a clock handle). The type parameter `P` ties the port to the pin bag its
    /// [`GpioPort::split`] yields. Returns [`DescriptorError::MissingBase`] if this part's descriptor
    /// does not carry that port (port presence is a RUNTIME `Result`, not a compile-time guarantee,
    /// because the chip is detected at runtime).
    fn gpio_port<P: PortPins>(&self, port: PeriphLabel) -> Result<GpioPort<P>, DescriptorError> {
        let base = self.base(port)?;
        let rcu = self.rcu_base()?;
        crate::clock::enable_gpio_port(rcu, self.desc.clock, port)?;
        Ok(GpioPort::new(base, self.desc.gpio))
    }

    /// The GPIOA port, with its port clock enabled; [`split`](GpioPort::split) it for the
    /// `pa0..pa15` pins.
    ///
    /// Resolves the port base from the chip's address table and enables its port clock through the
    /// chip's clock path (the stm32f1xx-hal `split(&mut rcc)` clock-enable, done here so the
    /// application passes no clock handle). Returns [`DescriptorError::MissingBase`] if this part's
    /// descriptor does not carry GPIOA (port presence is a RUNTIME `Result`, not a compile-time
    /// guarantee, because the chip is detected at runtime).
    #[inline]
    pub fn gpioa(&self) -> Result<GpioPort<PortAPins>, DescriptorError> {
        self.gpio_port(PeriphLabel::Gpioa)
    }

    /// The GPIOB port, with its port clock enabled; [`split`](GpioPort::split) it for the
    /// `pb0..pb15` pins. Like [`Chip::gpioa`], the port clock is enabled and presence is a runtime
    /// `Result`.
    #[inline]
    pub fn gpiob(&self) -> Result<GpioPort<PortBPins>, DescriptorError> {
        self.gpio_port(PeriphLabel::Gpiob)
    }

    /// The GPIOC port, with its port clock enabled; [`split`](GpioPort::split) it for the
    /// `pc0..pc15` pins. Like [`Chip::gpioa`], the port clock is enabled and presence is a runtime
    /// `Result`.
    #[inline]
    pub fn gpioc(&self) -> Result<GpioPort<PortCPins>, DescriptorError> {
        self.gpio_port(PeriphLabel::Gpioc)
    }

    /// The GPIOD port, with its port clock enabled; [`split`](GpioPort::split) it for the
    /// `pd0..pd15` pins. Like [`Chip::gpioa`], the port clock is enabled and presence is a runtime
    /// `Result`.
    #[inline]
    pub fn gpiod(&self) -> Result<GpioPort<PortDPins>, DescriptorError> {
        self.gpio_port(PeriphLabel::Gpiod)
    }

    /// The GPIOF port, with its port clock enabled; [`split`](GpioPort::split) it for the
    /// `pf0..pf15` pins. Like [`Chip::gpioa`], the port clock is enabled and presence is a runtime
    /// `Result`.
    #[inline]
    pub fn gpiof(&self) -> Result<GpioPort<PortFPins>, DescriptorError> {
        self.gpio_port(PeriphLabel::Gpiof)
    }

    /// Make the JTAG-overlay pins (PA15 / PB3 / PB4) usable as GPIO while keeping SWD live.
    ///
    /// This is a CAPABILITY, not a policy: the HAL never calls it on its own. After reset the F10x
    /// parts route PA15 (JTDI), PB3 (JTDO), and PB4 (NJTRST) to the JTAG debug port, so those pins
    /// cannot drive GPIO until JTAG is disabled. An application that wires those pins (e.g. PA15 =
    /// LED on the RoboDurden split-board) calls this once after [`crate::detect_chip`] to free them.
    ///
    /// The register work is family-internal, so the application never sees the [`GpioPath`] split:
    ///
    /// - **F10x** ([`GpioPath::ApbCrlCrh`]): enable the AFIO peripheral clock (`RCU_APB2EN` bit 0,
    ///   `AFIOEN`, at offset `0x18` from the chip's RCU base), then set `AFIO_PCF0` (`0x4001_0004`)
    ///   `SWJ_CFG` (bits `[26:24]`) to `0b010` = "JTAG-DP disabled, SW-DP enabled". Both writes are
    ///   read-modify-write so no other enable / config bits are disturbed. This disables the JTAG
    ///   debug port (freeing PA15 / PB3 / PB4 for GPIO) but KEEPS the serial-wire debug port (PA13 /
    ///   PA14) live, so an attached SWD debugger stays connected.
    /// - **F1x0** ([`GpioPath::AhbCtlAfsel`]): a no-op that returns `Ok`. The F1x0 has no AFIO
    ///   peripheral (that address region is reserved and writing it could fault), and PA15 / PB3 are
    ///   already plain GPIO after reset, so nothing needs doing.
    ///
    /// Returns [`DescriptorError::MissingBase`] only if the chip descriptor did not carry the RCU
    /// base (needed for the F10x AFIO clock enable).
    pub fn free_jtag_pins(&self) -> Result<(), DescriptorError> {
        match self.desc.gpio {
            GpioPath::ApbCrlCrh => {
                // F10x: enable the AFIO clock, then remap SWJ to SW-only.
                const RCU_APB2EN_OFFSET: u32 = 0x18;
                const AFIOEN: u32 = 1 << 0;
                let rcu = self.rcu_base()?;
                Reg32::new(rcu, RCU_APB2EN_OFFSET).modify(AFIOEN, AFIOEN);

                // AFIO_PCF0 at 0x4001_0004: SWJ_CFG = bits [26:24] = 0b010 (JTAG-DP disabled,
                // SW-DP enabled), freeing PA15/PB3/PB4 while keeping SWD live.
                const AFIO_BASE: u32 = 0x4001_0000;
                const AFIO_PCF0_OFFSET: u32 = 0x04;
                const SWJ_CFG_MASK: u32 = 0b111 << 24;
                const SWJ_CFG_SW_ONLY: u32 = 0b010 << 24;
                Reg32::new(AFIO_BASE, AFIO_PCF0_OFFSET).modify(SWJ_CFG_MASK, SWJ_CFG_SW_ONLY);
                Ok(())
            }
            // F1x0: no AFIO, PA15/PB3 already GPIO. The reserved region must not be written.
            GpioPath::AhbCtlAfsel => Ok(()),
        }
    }

    /// The clock-tree / RCU register-model path selector.
    #[inline]
    pub const fn clock(&self) -> ClockPath {
        self.desc.clock
    }

    /// The ADC acquisition path selector.
    #[inline]
    pub const fn adc_path(&self) -> AdcPath {
        self.desc.adc
    }

    /// The interrupt / vector-table layout selector.
    #[inline]
    pub const fn irq(&self) -> IrqLayout {
        self.desc.irq
    }

    /// The RCU base ([`DescriptorError::MissingBase`] if the descriptor did not carry it).
    #[inline]
    pub fn rcu_base(&self) -> Result<u32, DescriptorError> {
        self.base(PeriphLabel::Rcu)
    }

    /// The flash page size.
    #[inline]
    pub const fn flash_page(&self) -> PageSize {
        self.desc.flash_page
    }

    /// Advanced-timer count capability (1 or 2).
    #[inline]
    pub const fn adv_timers(&self) -> u8 {
        self.desc.adv_timers
    }

    /// ADC instance count capability (1 = F1x0, 2 = F10x).
    #[inline]
    pub const fn adc_count(&self) -> u8 {
        self.desc.adc_count
    }
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;
    use crate::detect::{descriptor_f103, descriptor_f130};
    use crate::reg::{mock, Reg32};
    use embedded_hal::digital::OutputPin;

    // RCU base both families carry, and the F10x AFIO addresses free_jtag_pins touches.
    const RCU_BASE: u32 = 0x4002_1000;
    const RCU_APB2EN: u32 = 0x18;
    const AFIO_PCF0: u32 = 0x4001_0004;

    #[test]
    fn family_is_f10x_for_f103_descriptor() {
        let chip = Chip::from_descriptor(descriptor_f103());
        assert_eq!(chip.family(), Family::F10x);
    }

    #[test]
    fn family_is_f1x0_for_f130_descriptor() {
        let chip = Chip::from_descriptor(descriptor_f130());
        assert_eq!(chip.family(), Family::F1x0);
    }

    #[test]
    fn family_matches_the_descriptor_gpio_selector() {
        // family() is derived from the GpioPath (single source of truth), so it must agree with it.
        assert_eq!(Chip::from_descriptor(descriptor_f103()).gpio(), GpioPath::ApbCrlCrh);
        assert_eq!(Chip::from_descriptor(descriptor_f130()).gpio(), GpioPath::AhbCtlAfsel);
    }

    #[test]
    fn free_jtag_pins_f10x_enables_afio_and_sets_swj_sw_only() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f103());
        assert_eq!(chip.free_jtag_pins(), Ok(()));
        // AFIO clock enable: RCU_APB2EN bit 0 set.
        assert_eq!(Reg32::new(RCU_BASE, RCU_APB2EN).read() & 1, 1);
        // SWJ_CFG (bits [26:24]) = 0b010 (JTAG-DP disabled, SW-DP enabled).
        assert_eq!(Reg32::new(AFIO_PCF0, 0).read() & (0b111 << 24), 0b010 << 24);
    }

    #[test]
    fn free_jtag_pins_f1x0_is_a_noop_and_writes_nothing() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f130());
        assert_eq!(chip.free_jtag_pins(), Ok(()));
        // F1x0 has no AFIO: neither the RCU AFIO bit nor the reserved AFIO_PCF0 region is touched.
        assert_eq!(Reg32::new(RCU_BASE, RCU_APB2EN).read(), 0);
        assert_eq!(Reg32::new(AFIO_PCF0, 0).read(), 0);
    }

    #[test]
    fn port_getters_resolve_bases_and_enable_clocks_f10x() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f103());
        // GPIOC..F now resolve (added to the descriptor) at the APB2 +0x400*n bases.
        assert_eq!(chip.gpioa().map(|p| p.base()), Ok(0x4001_0800));
        assert_eq!(chip.gpioc().map(|p| p.base()), Ok(0x4001_1000));
        assert_eq!(chip.gpiod().map(|p| p.base()), Ok(0x4001_1400));
        assert_eq!(chip.gpiof().map(|p| p.base()), Ok(0x4001_1C00));
        // The port getter enabled the port clock (F10x GPIO ports on APB2EN: PAEN=bit 2, PCEN=bit 4).
        let apb2 = Reg32::new(RCU_BASE, RCU_APB2EN).read();
        assert_eq!(apb2 & (1 << 2), 1 << 2); // PAEN
        assert_eq!(apb2 & (1 << 4), 1 << 4); // PCEN
    }

    #[test]
    fn port_getters_resolve_bases_and_enable_clocks_f1x0() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f130());
        assert_eq!(chip.gpioa().map(|p| p.base()), Ok(0x4800_0000));
        assert_eq!(chip.gpioc().map(|p| p.base()), Ok(0x4800_0800));
        assert_eq!(chip.gpiof().map(|p| p.base()), Ok(0x4800_1400));
        // F1x0 GPIO ports on AHBEN (offset 0x14): PAEN=bit 17, PCEN=bit 19.
        let ahben = Reg32::new(RCU_BASE, 0x14).read();
        assert_eq!(ahben & (1 << 17), 1 << 17); // PAEN
        assert_eq!(ahben & (1 << 19), 1 << 19); // PCEN
    }

    #[test]
    fn split_then_into_push_pull_output_drives_bop_f10x() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f103());
        let gpioa = chip.gpioa().unwrap().split();
        let mut led = gpioa.pa15.into_push_pull_output();
        // PA15 -> CTL1 nibble at (15-8)*4 = [31:28], GP output push-pull 50 MHz = 0x3.
        assert_eq!(Reg32::new(0x4001_0800, 0x04).read(), 0x3u32 << 28);
        // Drive high: F10x BOP at 0x10, bit 15.
        led.set_high().unwrap();
        assert_eq!(Reg32::new(0x4001_0800, 0x10).read(), 1 << 15);
        // Drive low: reset half (bit 15+16).
        led.set_low().unwrap();
        assert_eq!(Reg32::new(0x4001_0800, 0x10).read(), 1 << (15 + 16));
    }

    #[test]
    fn split_then_into_push_pull_output_drives_bop_f1x0() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f130());
        let gpioa = chip.gpioa().unwrap().split();
        let mut led = gpioa.pa15.into_push_pull_output();
        // PA15 -> CTL [31:30] output mode = 1; OSPD [31:30] = 3 (50 MHz).
        assert_eq!(Reg32::new(0x4800_0000, 0x00).read(), 1u32 << 30);
        assert_eq!(Reg32::new(0x4800_0000, 0x08).read(), 3u32 << 30);
        // Drive high: F1x0 BOP at 0x18, bit 15.
        led.set_high().unwrap();
        assert_eq!(Reg32::new(0x4800_0000, 0x18).read(), 1 << 15);
    }
}
