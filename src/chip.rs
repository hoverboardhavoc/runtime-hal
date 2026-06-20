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
use crate::adc::{Adc, AdcCapability, DualAdc};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::DescriptorError;
use crate::gpio::gpio_in::INPUT_GROUP_LINES;
use crate::gpio::{
    self, GpioOutput, GpioPort, InputGroup, PortAPins, PortBPins, PortCPins, PortDPins, PortFPins,
    PortPins,
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

    /// Resolve a label to its base address ([`DescriptorError::MissingBase`] if absent).
    ///
    /// HAL-internal (`pub(crate)`): the general raw-base escape, used heavily in-crate to source a
    /// peripheral base from the descriptor. It is NOT public, so a caller never holds a raw base; the
    /// chip-based builders (e.g. [`Chip::output_pin`], [`Chip::input_group`], [`Chip::adc`],
    /// [`crate::timer::PwmTimer::configure`], [`crate::watchdog::FreeWatchdog::start`]) resolve the
    /// base internally and hand back a handle. If an external consumer needs a base, that is a signal
    /// to add the missing chip-based builder, not to re-expose this.
    #[inline]
    pub(crate) fn base(&self, label: PeriphLabel) -> Result<u32, DescriptorError> {
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
    /// push-pull output through `configure_output` (which owns the F10x/F1x0
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

    /// Whether this part HAS the given advanced timer (presence resolution): true iff `label` is an
    /// advanced-timer label whose base resolved into the descriptor in the APB2 advanced-timer window.
    /// `Timer0` is present on every part; `Timer7` only on parts detection measured a second advanced
    /// timer on. This exposes the presence difference WITHOUT exposing a raw base: a caller asks
    /// "is the second motor timer here?" and gets a yes/no, not an address.
    #[inline]
    pub fn has_advanced_timer(&self, label: PeriphLabel) -> bool {
        self.desc.addrs.check_timer_base(label).is_ok()
    }

    /// Whether this part HAS the given ADC instance (presence resolution): true iff `label` is an ADC
    /// label whose base resolved into the descriptor in the ADC window. Like [`Self::has_advanced_timer`],
    /// this answers presence without handing out a base.
    #[inline]
    pub fn has_adc(&self, label: PeriphLabel) -> bool {
        self.desc.addrs.check_adc_base(label).is_ok()
    }

    /// The chip's ADC sampling capability as a "fruit" (silicon shape, never a family flag): one ADC,
    /// or two (the F10x dual-ADC parts). The ADC base(s) are resolved INTERNALLY and handed back as
    /// [`crate::adc::Adc`] handle(s); the caller never sees a base, and matches the returned
    /// [`AdcCapability`] exhaustively (so its firmware handles both shapes and runs on either family).
    /// Returns [`DescriptorError::MissingBase`] if ADC0 (or, on a dual part, ADC1) is absent.
    pub fn adc(&self) -> Result<AdcCapability, DescriptorError> {
        let primary = Adc::at(self.base(PeriphLabel::Adc0)?);
        if self.adc_count() >= 2 {
            let secondary = Adc::at(self.base(PeriphLabel::Adc1)?);
            Ok(AdcCapability::Dual(DualAdc::new(primary, secondary)))
        } else {
            Ok(AdcCapability::Single(primary))
        }
    }

    /// Build a resolve-once multi-pin input reader ([`crate::gpio::InputGroup`]) over `pins`, the
    /// logical pin bytes (`(port << 4) | pin`, the same encoding [`Chip::route_advanced_pwm_pin`]
    /// takes). Each pin's GPIO port base is resolved INTERNALLY from the descriptor (via the same
    /// port-label mapping the routing uses), and the family's `GPIO_ISTAT` offset is picked from the
    /// descriptor's [`GpioPath`], so the caller never holds a raw base or names a family. A motor
    /// layer uses this for its rotor-position lines (read each cycle, decoded outside the HAL); the
    /// reader itself is a neutral GPIO sampler.
    ///
    /// Returns [`DescriptorError::MissingBase`] if any pin's port is not in the descriptor (or the
    /// port index is one this crate does not model). This is the base-hidden builder: it resolves the
    /// per-pin port bases and the family ISTAT offset internally, so no caller-facing raw base or
    /// `GpioPath` is involved.
    pub fn input_group(
        &self,
        pins: [u8; INPUT_GROUP_LINES],
    ) -> Result<InputGroup, DescriptorError> {
        let mut lines = [(0u32, 0u8); INPUT_GROUP_LINES];
        for (slot, &pin) in lines.iter_mut().zip(pins.iter()) {
            let base = self.base(gpio_port_label(pin)?)?;
            *slot = (base, pin & 0x0F);
        }
        Ok(InputGroup::resolve(self.desc.gpio, lines))
    }

    /// Route `pin` to the general-purpose-timer PWM output (TIMER1_CH1), doing ALL the family-specific
    /// register work INTERNALLY so the caller never names a family. Used by [`crate::pwm::PwmOut`] so
    /// pin routing folds into the PWM bring-up.
    ///
    /// - **F1x0** (per-pin AF mux): set `pin`'s `AFSEL` to AF2 (TIMER1_CH1).
    /// - **F10x** (AFIO remap groups): free the JTAG overlay (releasing PA15 / PB3 / PB4 from the
    ///   JTAG-DP while keeping SWD live), set the AFIO `TIMER1_REMAP` partial-remap-1 field (which maps
    ///   TIMER1_CH1 to PB3), then set `pin`'s CRL/CRH alternate-function nibble.
    ///
    /// Enables `pin`'s GPIO port clock as part of the routing. The family branch is on the descriptor's
    /// register-model selector ([`GpioPath`]), an internal detail, NOT a caller-visible family flag.
    /// Returns [`DescriptorError::MissingBase`] if the pin's port (or the RCU, needed for the F10x
    /// remap) is not in the descriptor.
    pub fn route_general_pwm_pin(&self, pin: u8) -> Result<(), DescriptorError> {
        let port = gpio_port_label(pin)?;
        let port_base = self.base(port)?;
        let rcu = self.rcu_base()?;
        // The AF write only sticks with the port clock on.
        crate::clock::enable_gpio_port(rcu, self.desc.clock, port)?;
        match self.desc.gpio {
            GpioPath::AhbCtlAfsel => {
                // F1x0: one per-pin AFSEL field (AF2 = TIMER1_CH1).
                gpio::configure_af(
                    port_base,
                    GpioPath::AhbCtlAfsel,
                    pin,
                    gpio::PinRole::GenTimerAfPushPull,
                );
            }
            GpioPath::ApbCrlCrh => {
                // F10x: free the JTAG overlay, AFIO TIMER1 partial-remap-1, then the CRL AF nibble.
                self.free_jtag_pins()?;
                gpio::remap_timer1_partial1(rcu);
                gpio::configure_af(
                    port_base,
                    GpioPath::ApbCrlCrh,
                    pin,
                    gpio::PinRole::GenTimerAfPushPull,
                );
            }
        }
        Ok(())
    }

    /// Route `pin` to an ADVANCED-timer (TIMER0 / TIMER7) complementary-output alternate function,
    /// family-internal. The advanced-timer gate pins are the DEFAULT alternate function on the F10x
    /// (no AFIO remap needed, unlike the general-timer LED pin), so this is the per-family AF write
    /// (F1x0: per-pin AFSEL = AF2; F10x: CRL/CRH nibble) plus the pin's port-clock enable. The caller
    /// (a motor application) passes its board gate pins as DATA; the family difference is absorbed.
    /// Returns [`DescriptorError::MissingBase`] if the pin's port (or the RCU) is not in the descriptor.
    pub fn route_advanced_pwm_pin(&self, pin: u8) -> Result<(), DescriptorError> {
        let port = gpio_port_label(pin)?;
        let port_base = self.base(port)?;
        let rcu = self.rcu_base()?;
        crate::clock::enable_gpio_port(rcu, self.desc.clock, port)?;
        gpio::configure_af(port_base, self.desc.gpio, pin, gpio::PinRole::TimerAfPushPull);
        Ok(())
    }
}

/// Map a logical pin byte (`port << 4 | pin`) to its GPIO port label. The high nibble is the port
/// index (0 = A, 1 = B, 2 = C, 3 = D, 5 = F), matching [`PeriphLabel`]'s GPIO variants. A port index
/// this crate does not model (E, and any > F) resolves to the GPIOA label, which then fails the base
/// lookup as [`DescriptorError::MissingBase`] (no part carries an out-of-range GPIO port).
fn gpio_port_label(pin: u8) -> Result<PeriphLabel, DescriptorError> {
    match pin >> 4 {
        0 => Ok(PeriphLabel::Gpioa),
        1 => Ok(PeriphLabel::Gpiob),
        2 => Ok(PeriphLabel::Gpioc),
        3 => Ok(PeriphLabel::Gpiod),
        5 => Ok(PeriphLabel::Gpiof),
        // No Gpioe label in this crate's catalog; report a GPIO base miss rather than guess a port.
        _ => Err(DescriptorError::MissingBase(PeriphLabel::Gpioa)),
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
    fn gpio_selector_matches_the_descriptor() {
        // The register-model selector is the single source of truth; it must match the descriptor.
        // (The HAL no longer exposes a family() tag: the caller never branches on family.)
        assert_eq!(Chip::from_descriptor(descriptor_f103()).gpio(), GpioPath::ApbCrlCrh);
        assert_eq!(Chip::from_descriptor(descriptor_f130()).gpio(), GpioPath::AhbCtlAfsel);
    }

    #[test]
    fn adc_fruit_is_single_or_dual_by_count() {
        use crate::adc::AdcCapability;
        // One ADC (the F1x0 baseline): Single.
        let single = Chip::from_descriptor(descriptor_f130());
        assert!(matches!(single.adc(), Ok(AdcCapability::Single(_))));

        // Two ADCs (as detect_chip populates for adc_count == 2): Dual, with ADC1 carried.
        let mut d = descriptor_f103();
        d.adc_count = 2;
        d.addrs.set(PeriphLabel::Adc1, 0x4001_2800);
        let dual = Chip::from_descriptor(d);
        assert!(matches!(dual.adc(), Ok(AdcCapability::Dual(_))));

        // A part that claims 2 ADCs but is missing the ADC1 base fails loud (no fake handle).
        let mut bad = descriptor_f103();
        bad.adc_count = 2;
        assert!(Chip::from_descriptor(bad).adc().is_err());
    }

    #[test]
    fn has_advanced_timer_reflects_descriptor_presence() {
        // Timer0 is present on every part; Timer7 only when its base is carried (detect_chip sets it
        // for adv_timers == 2). The base-address const carries only Timer0, so Timer7 is absent here.
        let f103 = Chip::from_descriptor(descriptor_f103());
        assert!(f103.has_advanced_timer(PeriphLabel::Timer0));
        assert!(!f103.has_advanced_timer(PeriphLabel::Timer7));
        // A non-timer label is never an advanced timer.
        assert!(!f103.has_advanced_timer(PeriphLabel::Gpioa));

        // A part with a second advanced timer (as detect_chip populates for adv_timers == 2) reports
        // it present, WITHOUT the caller ever seeing the base.
        let mut d = descriptor_f103();
        d.addrs.set(PeriphLabel::Timer7, 0x4001_3400);
        let dual = Chip::from_descriptor(d);
        assert!(dual.has_advanced_timer(PeriphLabel::Timer7));
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

    // PB3 (port B = 1, pin 3): the green LED / TIMER1_CH1, the general-PWM routing target.
    const PB3: u8 = (1 << 4) | 3;
    // GPIOB bases for each descriptor (one APB/AHB 0x400 stride above GPIOA).
    const F10X_GPIOB_BASE: u32 = 0x4001_0C00;
    const F1X0_GPIOB_BASE: u32 = 0x4800_0400;

    // route_general_pwm_pin does ALL the family-specific routing internally; the caller (PwmOut) just
    // names the pin. These pin the per-family register writes the hidden dispatch produces.

    #[test]
    fn route_general_pwm_pin_f1x0_sets_afsel_af2() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f130());
        assert_eq!(chip.route_general_pwm_pin(PB3), Ok(()));
        // F1x0: PB3 CTL [7:6] = AF mode (2); AFSEL0 nibble [15:12] = AF2 (TIMER1_CH1). No AFIO.
        assert_eq!(Reg32::new(F1X0_GPIOB_BASE, 0x00).read() & (0x3 << 6), 2 << 6);
        assert_eq!(Reg32::new(F1X0_GPIOB_BASE, 0x20).read() & (0xF << 12), 2 << 12);
    }

    #[test]
    fn route_general_pwm_pin_f10x_remaps_timer1_and_sets_crl_af() {
        let _g = mock::lock();
        mock::reset();
        let chip = Chip::from_descriptor(descriptor_f103());
        assert_eq!(chip.route_general_pwm_pin(PB3), Ok(()));
        // F10x: AFIO_PCF0 TIMER1_REMAP[9:8] = 0b01 (partial remap 1 -> TIMER1_CH1 / PB3).
        assert_eq!(Reg32::new(AFIO_PCF0, 0).read() & (0b11 << 8), 0b01 << 8);
        // PB3 CRL nibble [15:12] = AF push-pull 50 MHz = 0xB.
        assert_eq!(Reg32::new(F10X_GPIOB_BASE, 0x00).read() & (0xF << 12), 0xB << 12);
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
