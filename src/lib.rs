#![cfg_attr(not(feature = "mock"), no_std)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
//!
//! ---
//!
//! The `mock` Cargo feature is a host-test concern only: it enables `std` and swaps the register
//! accessor to a backing-array backend so host tests can read/write the simulated register space.
//! The normal build is real volatile MMIO. Features never gate a family path: the single binary
//! always carries both the F10x and F1x0 register models, and runtime detection picks between them.

pub mod adc;
pub mod addr;
pub mod chip;
pub mod clock;
pub mod config;
/// SysTick-based blocking delay implementing the `embedded-hal` 1.0 `DelayNs` trait
/// ([`delay::Delay`]).
pub mod delay;
pub mod descriptor;
/// Runtime heuristic detection: the ONLY way the HAL learns its chip identity. The boot-flow entry
/// ([`detect::detect_chip`]) runs the bus-fault-safe GPIO+RCU family probe (the family discriminator),
/// MEASURES the per-instance advanced-timer / ADC counts by a benign scratch write-back, reads the
/// flash density, and synthesizes the [`McuDescriptor`] the rest of the HAL is built on.
pub mod detect;
pub mod error;
pub mod gpio;
pub mod hotpath;
pub mod i2c;
pub mod irq;
pub mod reg;
pub mod serial;
pub mod spi;
/// Configurable-rate periodic SysTick tick (G-TICK): the cold-path outer-loop / cadence timebase
/// ([`timebase::Timebase`]). Runs SysTick in interrupt mode, mutually exclusive with [`delay::Delay`].
pub mod timebase;
pub mod timer;
pub mod usart;

pub use adc::{is_internal_channel, Adc};
pub use addr::{AddrTable, PeriphLabel};
pub use chip::Chip;
pub use clock::{
    configure_tree, configure_tree_timeout, enable_adc, enable_gpio_port, enable_i2c, enable_spi,
    enable_usart, ClockConfig, ClockSource, DEFAULT_CLOCK_SPIN_CAP,
};
pub use config::{
    decode_pin, AdcChannel, AdcClockDiv, AdcConfig, BreakConfig, ClockDiv, InjectedAdcConfig,
    InjectedChannel, NssMode, OcMode, Oversampling, Parity, PwmAlign, PwmChannelConfig, PwmConfig,
    SpiConfig, StopBits, TimerTriggerLink, UsartConfig, UsartFrame, WordLen,
};
pub use delay::Delay;
pub use descriptor::{
    AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize, MAX_ADCS, MAX_ADC_CHANNELS,
    MAX_GPIO_PORTS, MAX_I2CS, MAX_INJECTED_CHANNELS, MAX_PWM_CHANNELS, MAX_SPIS, MAX_TIMERS,
    MAX_USARTS,
};
pub use detect::{
    descriptor_f103, descriptor_f130, detect_chip, synthesize, Family, F10X_K2_THRESHOLD_KIB,
    FLASH_DENSITY_ADDR,
};
pub use error::{
    AdcError, ClockError, DescriptorError, DetectError, HotPathError, I2cError, PwmError, SpiError,
    UsartError,
};
pub use gpio::{
    configure_af, configure_output, read_pin, set_pin, Floating, GpioOutput, GpioPort, Input,
    Output, Pin, PinRole, PortAPins, PortBPins, PortCPins, PortDPins, PortFPins, PortPins,
    PullDown, PullUp, PushPull,
};
pub use hotpath::hall::HallReader;
pub use hotpath::{
    ComplementaryPwm, InjectedAdcController, InjectedHandle, PwmController, PwmHandle, TriggeredAdc,
};
pub use i2c::{i2c_input_clock, timing_for, FastDuty, I2c, I2cMode, I2cTiming};
pub use irq::{
    build_table, clear_tick_count, clear_tick_handler, on_systick, register_control_handler,
    register_tick_handler, tick_count, Handler, RamVectorTable, TickHandler,
};
pub use reg::{Reg16, Reg32};
pub use serial::{Serial, UsartSerial};
pub use spi::{mode_bits, prescaler_for, spi_input_clock, DataSize, Spi};
pub use timebase::{reload_for, Timebase, TimebaseError};
pub use timer::PwmTimer;
pub use usart::{compute_brr, usart_input_clock, Status, Usart, UsartBus, UsartModel};
