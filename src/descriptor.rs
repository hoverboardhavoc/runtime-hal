//! The MCU descriptor: bounded-owned, no alloc, CHIP-ONLY (descriptor-rework DR-T2).
//!
//! DECISIONS.md #1 / #10: [`McuDescriptor`] is a small fixed-size owned value carrying ONLY the
//! chip-level facts that let one binary run on an F103 vs an F130: the per-family register-model
//! selectors (`gpio`, `clock`, `adc`, `irq`), the base-address table (`addrs`), and the chip
//! capabilities (`flash_page`, `adv_timers`, `adc_count`). It is "what silicon is this, and how are
//! its registers laid out". Nothing about how a given application wires or times a peripheral
//! belongs here.
//!
//! The peripheral wiring (pins, baud, bus speed, channels, PWM/timer timing) and the clock tree are
//! NOT in the descriptor: the application constructs typed config values ([`crate::config`] and
//! [`crate::clock::ClockConfig`]) and passes them to the HAL bring-up calls, which supply the chip
//! base + selector from the descriptor (DECISIONS.md #10). The [`crate::Chip`] context wraps the
//! descriptor with resolution helpers for those calls.

use crate::addr::AddrTable;

/// Max USART config records an application typically constructs (capacity constant, over-provisioned).
pub const MAX_USARTS: usize = 4;

/// Max GPIO ports (A..F) (capacity constant).
pub const MAX_GPIO_PORTS: usize = 6;

/// Max I2C config records (capacity constant; over-provisioned). F10x has two I2C instances.
pub const MAX_I2CS: usize = 2;

/// Max SPI config records (capacity constant; over-provisioned). F10x has SPI0/SPI1.
pub const MAX_SPIS: usize = 2;

/// Max regular ADC channels in a sequence (capacity constant): bounds [`crate::config::AdcConfig`].
pub const MAX_ADC_CHANNELS: usize = 8;

/// Max ADC config records (capacity constant; over-provisioned). F10x has two ADC instances.
pub const MAX_ADCS: usize = 2;

/// Max advanced-timer config records (capacity constant). One per `adv_timers` (1 or 2).
pub const MAX_TIMERS: usize = 2;

/// Number of complementary channel pairs the advanced-timer PWM drives (capacity constant):
/// the three half-bridges (CH0/CH0N, CH1/CH1N, CH2/CH2N -> the 6 gate signals). The 4th compare
/// channel (CH3) is the ADC trigger and is carried separately on the PWM config, not in this count.
pub const MAX_PWM_CHANNELS: usize = 3;

/// Max injected-conversion ADC channels (capacity constant): the injected group the timer triggers.
pub const MAX_INJECTED_CHANNELS: usize = 4;

/// GPIO register-model path selector.
///
/// `apb_crl_crh` (F10x: CRL/CRH mode+cnf nibbles, AF implied) vs `ahb_ctl_afsel` (F1x0:
/// CTL/AFSEL/AF-mux). Fieldless `#[repr(u8)]` so it is a stable wire value.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioPath {
    /// F10x: CRL/CRH config registers, alternate function implied by mode/cnf.
    ApbCrlCrh = 0,
    /// F1x0: CTL + AFSEL + per-pin AF mux.
    AhbCtlAfsel = 1,
}

/// Clock-tree / reset-clock-unit path selector.
///
/// `f10x_rcc` vs `f1x0_rcu`: different enable registers/bits and clock tree.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockPath {
    /// F10x RCC register model.
    F10xRcc = 0,
    /// F1x0 RCU register model.
    F1x0Rcu = 1,
}

/// ADC acquisition path selector.
///
/// The register core is shared; F1x0 has one ADC and F10x two, so the paths differ in single vs
/// dual/simultaneous acquisition.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcPath {
    /// Single-ADC injected acquisition (baseline).
    Single = 0,
    /// Dual / simultaneous acquisition (F10x enhancement).
    Dual = 1,
}

/// Interrupt / RAM-vector-table layout selector.
///
/// `f1x0_grouped` (advanced-timer break/update/trigger/commutation bundled, EXTI lines grouped)
/// vs `f10x_separate` (separate IRQs at different positions).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqLayout {
    /// F1x0: grouped IRQ layout.
    F1x0Grouped = 0,
    /// F10x: separate IRQ layout.
    F10xSeparate = 1,
}

/// Flash page size (the FMC page-erase granularity).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PageSize {
    /// 1 KB pages.
    K1 = 0,
    /// 2 KB pages.
    K2 = 1,
}

impl PageSize {
    /// Page size in bytes.
    #[inline]
    pub const fn bytes(self) -> u32 {
        match self {
            PageSize::K1 => 1024,
            PageSize::K2 => 2048,
        }
    }
}

/// The MCU descriptor (DECISIONS.md #1 / #10 / SPEC.md): the CHIP only.
///
/// Bounded-owned, fixed-size, `Copy`: the four register-model selectors, the base-address table,
/// and the chip capabilities. The six `*Wiring` Vec fields and `clock_cfg` that earlier carried the
/// application's peripheral choices are removed (the wiring becomes code-level [`crate::config`]
/// types and the clock tree a code-level [`crate::clock::ClockConfig`]); the descriptor now carries
/// only "what silicon is this and where do its peripherals live". Produced by runtime detection
/// ([`crate::detect`]) or written as a literal in tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuDescriptor {
    /// GPIO register-model path.
    pub gpio: GpioPath,
    /// Clock-tree path.
    pub clock: ClockPath,
    /// ADC acquisition path.
    pub adc: AdcPath,
    /// Interrupt / vector-table layout.
    pub irq: IrqLayout,
    /// Base address per peripheral label (the data axis).
    pub addrs: AddrTable,
    /// Flash page size for FMC.
    pub flash_page: PageSize,
    /// Advanced-timer count (1 or 2).
    pub adv_timers: u8,
    /// ADC count (1 = F1x0, 2 = F10x): capability, not the single/dual choice.
    pub adc_count: u8,
}
