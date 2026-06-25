//! Generated per vector by build_rusthal.py. This checked-in default routes the
//! F1x0 advanced-timer gate pin PA8 to its alternate function through the public
//! chip-based router, so the template crate builds standalone against the current
//! runtime-hal public surface; the harness overwrites it for each vector then
//! restores it.

use runtime_hal::{AddrTable, AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize, PeriphLabel};
use runtime_hal::Chip;

pub fn body() {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    addrs.set(PeriphLabel::Gpioa, 0x4800_0000);
    let chip = Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel,
        clock: ClockPath::F1x0Rcu,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1, flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    });
    let _ = chip.route_advanced_pwm_pin(0x08);
}
