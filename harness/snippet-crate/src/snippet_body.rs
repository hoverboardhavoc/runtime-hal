//! Generated per vector by build_rusthal.py. This checked-in default is the
//! F1x0 GPIO-AF body (PA2 = USART1 TX at AF1) so the template crate builds
//! standalone; the harness overwrites it for each vector then restores it.

use runtime_hal::descriptor::GpioPath;
use runtime_hal::gpio::{configure_af, PinRole};

const GPIOA_BASE: u32 = 0x4800_0000;

pub fn body() {
    configure_af(GPIOA_BASE, GpioPath::AhbCtlAfsel, 2, PinRole::Tx);
}
