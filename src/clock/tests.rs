//! T3 host tests for the clock path (run under the `mock` feature against the backing-array
//! register space). Each test seeds the RCU enable registers with their reset value (0: all
//! peripheral clocks off on both families), runs the enable, then asserts the exact resulting
//! register bits equal what the GD SPL header says for that operation. Width-strict: the enable
//! registers are 32-bit and the assertions read them as `Reg32`.
#![cfg(feature = "mock")]

use crate::addr::PeriphLabel;
use crate::clock::{
    enable_adc, enable_general_timer, enable_gpio_port, enable_i2c, enable_spi, enable_timer,
    enable_usart,
};
use crate::descriptor::ClockPath;
use crate::error::DescriptorError;
use crate::reg::{mock, Reg32};
use std::sync::MutexGuard;

// Register offsets under the RCU base (identical numeric offsets, divergent contents).
const AHBEN: u32 = 0x14;
const APB2EN: u32 = 0x18;
const APB1EN: u32 = 0x1C;

/// A test RCU base inside the validated range; the offsets are what matter for the mock.
const RCU_BASE: u32 = 0x4002_1000;

/// Acquire the whole-case serialization lock and seed the RCU enable registers to their reset
/// value (0: all peripheral clocks off, both families). The returned guard must be held for the
/// rest of the case so a concurrent test's `reset` cannot race the seed/assert sequence.
fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    Reg32::new(RCU_BASE, AHBEN).write(0);
    Reg32::new(RCU_BASE, APB2EN).write(0);
    Reg32::new(RCU_BASE, APB1EN).write(0);
    g
}

fn read(off: u32) -> u32 {
    Reg32::new(RCU_BASE, off).read()
}

// --- F10x (f10x_rcc) --------------------------------------------------------------------------

#[test]
fn f10x_enable_usart1_sets_apb1en_bit17() {
    let _g = seed_reset();
    enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1).unwrap();
    // USART1EN = BIT(17) on APB1EN (gd32f10x_rcu.h:292). Nothing else touched.
    assert_eq!(read(APB1EN), 1 << 17);
    assert_eq!(read(APB2EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f10x_enable_usart0_sets_apb2en_bit14() {
    let _g = seed_reset();
    enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart0).unwrap();
    // USART0EN = BIT(14) on APB2EN (gd32f10x_rcu.h:267).
    assert_eq!(read(APB2EN), 1 << 14);
    assert_eq!(read(APB1EN), 0);
}

#[test]
fn f10x_enable_usart2_sets_apb1en_bit18() {
    let _g = seed_reset();
    enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart2).unwrap();
    // USART2EN = BIT(18) on APB1EN (gd32f10x_rcu.h:293).
    assert_eq!(read(APB1EN), 1 << 18);
}

#[test]
fn f10x_enable_gpioa_sets_apb2en_bit2() {
    let _g = seed_reset();
    enable_gpio_port(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Gpioa).unwrap();
    // PAEN = BIT(2) on APB2EN (gd32f10x_rcu.h:255). GPIO lives on APB2 for F10x.
    assert_eq!(read(APB2EN), 1 << 2);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f10x_enable_usart1_plus_gpioa_is_two_independent_bits() {
    let _g = seed_reset();
    // The M1 "enable USART + its GPIO port" sequence: APB1EN bit 17 (USART1) + APB2EN bit 2 (PA).
    enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1).unwrap();
    enable_gpio_port(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Gpioa).unwrap();
    assert_eq!(read(APB1EN), 1 << 17);
    assert_eq!(read(APB2EN), 1 << 2);
}

#[test]
fn f10x_enable_preserves_other_bits() {
    let _g = seed_reset();
    // A pre-existing enable (e.g. another peripheral) must survive the RMW.
    Reg32::new(RCU_BASE, APB1EN).write(1 << 14); // SPI1EN-ish neighbour bit
    enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1).unwrap();
    assert_eq!(read(APB1EN), (1 << 14) | (1 << 17));
}

// --- F1x0 (f1x0_rcu) --------------------------------------------------------------------------

#[test]
fn f1x0_enable_usart1_sets_apb1en_bit17() {
    let _g = seed_reset();
    enable_usart(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Usart1).unwrap();
    // USART1EN = BIT(17) on APB1EN (gd32f1x0_rcu.h:216).
    assert_eq!(read(APB1EN), 1 << 17);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f1x0_enable_usart0_sets_apb2en_bit14() {
    let _g = seed_reset();
    enable_usart(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Usart0).unwrap();
    // USART0EN = BIT(14) on APB2EN (gd32f1x0_rcu.h:200).
    assert_eq!(read(APB2EN), 1 << 14);
}

#[test]
fn f1x0_has_no_usart2() {
    let _g = seed_reset();
    // F1x0 has no USART2; the path rejects it rather than writing a wrong bit.
    assert_eq!(
        enable_usart(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Usart2),
        Err(DescriptorError::SelectorAddrMismatch)
    );
    assert_eq!(read(APB1EN), 0);
}

#[test]
fn f1x0_enable_gpioa_sets_ahben_bit17() {
    let _g = seed_reset();
    enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioa).unwrap();
    // PAEN = BIT(17) on AHBEN (gd32f1x0_rcu.h:188). GPIO lives on AHB for F1x0 (the divergence).
    assert_eq!(read(AHBEN), 1 << 17);
    assert_eq!(read(APB2EN), 0);
}

#[test]
fn f1x0_enable_gpiof_sets_ahben_bit22() {
    let _g = seed_reset();
    enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpiof).unwrap();
    // PFEN = BIT(22) on AHBEN (gd32f1x0_rcu.h:192). BIT(21) is the no-port-E gap.
    assert_eq!(read(AHBEN), 1 << 22);
}

#[test]
fn f1x0_has_no_port_e() {
    let _g = seed_reset();
    assert_eq!(
        enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioe),
        Err(DescriptorError::SelectorAddrMismatch)
    );
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f1x0_enable_usart1_plus_gpioa_uses_divergent_registers() {
    let _g = seed_reset();
    // The M1 sequence on F1x0: USART1 on APB1EN bit 17, but GPIOA on AHBEN bit 17 (not APB2).
    enable_usart(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Usart1).unwrap();
    enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioa).unwrap();
    assert_eq!(read(APB1EN), 1 << 17);
    assert_eq!(read(AHBEN), 1 << 17);
    assert_eq!(read(APB2EN), 0);
}

// --- M2 T4: bus + ADC clock enables -----------------------------------------------------------
//
// Same RMW shape and offsets as the USART/GPIO enables. The enable bits match on both families;
// the only family divergence is the F1x0 ADC clock prescaler also setting RCU_CFG2 ADCSEL.

const CFG0: u32 = 0x04;
const CFG2: u32 = 0x30;

#[test]
fn enable_i2c0_sets_apb1en_bit21_both_families() {
    let _g = seed_reset();
    enable_i2c(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::I2c0).unwrap();
    // I2C0EN = BIT(21) on APB1EN (gd32f1x0_rcu.h:380 / gd32f10x_rcu.h:413).
    assert_eq!(read(APB1EN), 1 << 21);
    assert_eq!(read(APB2EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn enable_i2c1_sets_apb1en_bit22() {
    let _g = seed_reset();
    enable_i2c(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::I2c1).unwrap();
    // I2C1EN = BIT(22) on APB1EN.
    assert_eq!(read(APB1EN), 1 << 22);
}

#[test]
fn enable_i2c0_plus_gpiob_clock_f1x0() {
    let _g = seed_reset();
    // The IMU bring-up sequence: GPIOB (PB6/PB7) clock then I2C0 clock, F1x0.
    enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpiob).unwrap();
    enable_i2c(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::I2c0).unwrap();
    // GPIOB = AHBEN bit 18 (F1x0); I2C0 = APB1EN bit 21.
    assert_eq!(read(AHBEN), 1 << 18);
    assert_eq!(read(APB1EN), 1 << 21);
}

#[test]
fn enable_i2c0_plus_gpiob_clock_f10x() {
    let _g = seed_reset();
    enable_gpio_port(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Gpiob).unwrap();
    enable_i2c(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::I2c0).unwrap();
    // GPIOB = APB2EN bit 3 (F10x); I2C0 = APB1EN bit 21.
    assert_eq!(read(APB2EN), 1 << 3);
    assert_eq!(read(APB1EN), 1 << 21);
}

#[test]
fn enable_spi0_sets_apb2en_bit12() {
    let _g = seed_reset();
    enable_spi(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Spi0).unwrap();
    // SPI0EN = BIT(12) on APB2EN (both families).
    assert_eq!(read(APB2EN), 1 << 12);
    assert_eq!(read(APB1EN), 0);
}

#[test]
fn enable_spi1_sets_apb1en_bit14() {
    let _g = seed_reset();
    enable_spi(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Spi1).unwrap();
    // SPI1EN = BIT(14) on APB1EN.
    assert_eq!(read(APB1EN), 1 << 14);
}

#[test]
fn enable_spi0_plus_gpioa_clock_f1x0() {
    let _g = seed_reset();
    // SPI0 on PA5/PA6/PA7: GPIOA clock then SPI0 clock, F1x0.
    enable_gpio_port(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioa).unwrap();
    enable_spi(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Spi0).unwrap();
    assert_eq!(read(AHBEN), 1 << 17); // GPIOA F1x0
    assert_eq!(read(APB2EN), 1 << 12); // SPI0
}

#[test]
fn enable_adc0_sets_apb2en_bit9_and_prescaler_f10x() {
    let _g = seed_reset();
    Reg32::new(RCU_BASE, CFG0).write(0);
    Reg32::new(RCU_BASE, CFG2).write(0);
    enable_adc(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Adc0).unwrap();
    // ADC0EN = BIT(9) on APB2EN.
    assert_eq!(read(APB2EN), 1 << 9);
    // ADC prescaler /6 = code 2 in CFG0 ADCPSC bits[15:14].
    assert_eq!(read(CFG0) & (0b11 << 14), 2 << 14);
    // F10x does NOT touch CFG2.
    assert_eq!(read(CFG2), 0);
}

#[test]
fn enable_adc0_sets_apb2en_bit9_prescaler_and_adcsel_f1x0() {
    let _g = seed_reset();
    Reg32::new(RCU_BASE, CFG0).write(0);
    Reg32::new(RCU_BASE, CFG2).write(0);
    enable_adc(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Adc0).unwrap();
    assert_eq!(read(APB2EN), 1 << 9);
    assert_eq!(read(CFG0) & (0b11 << 14), 2 << 14);
    // F1x0 also selects the APB2-derived ADC clock source: CFG2 ADCSEL bit 8.
    assert_eq!(read(CFG2) & (1 << 8), 1 << 8);
}

#[test]
fn enable_adc1_sets_apb2en_bit10_f10x() {
    let _g = seed_reset();
    enable_adc(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Adc1).unwrap();
    assert_eq!(read(APB2EN), 1 << 10);
}

#[test]
fn bus_enable_preserves_other_bits() {
    let _g = seed_reset();
    // A pre-existing APB1EN enable must survive the I2C RMW.
    Reg32::new(RCU_BASE, APB1EN).write(1 << 17); // USART1EN neighbour
    enable_i2c(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::I2c0).unwrap();
    assert_eq!(read(APB1EN), (1 << 17) | (1 << 21));
}

#[test]
fn non_bus_labels_rejected() {
    let _g = seed_reset();
    assert_eq!(
        enable_i2c(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Usart1),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(
        enable_spi(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioa),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(
        enable_adc(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::I2c0),
        Err(DescriptorError::UnknownSelector)
    );
}

// --- non-USART / non-GPIO labels rejected -----------------------------------------------------

#[test]
fn non_usart_label_rejected() {
    let _g = seed_reset();
    assert_eq!(
        enable_usart(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Gpioa),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(
        enable_gpio_port(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1),
        Err(DescriptorError::UnknownSelector)
    );
}

// --- DR-T4: advanced-timer clock enable (enable_timer) ----------------------------------------
//
// TIMER0 is APB2EN bit 11 on BOTH families (RCU_APB2EN_TIMER0EN = BIT(11), gd32f1x0_rcu.h:198 /
// gd32f10x_rcu.h:264). The M3 bench firmware had to set this bit with a raw Reg32 write because the
// HAL had no enable_timer; DR-T4 closes that gap. TIMER7 is F10x-only (APB2EN bit 13).

#[test]
fn f1x0_enable_timer0_sets_apb2en_bit11() {
    let _g = seed_reset();
    enable_timer(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Timer0).unwrap();
    // TIMER0EN = BIT(11) on APB2EN; nothing else touched.
    assert_eq!(read(APB2EN), 1 << 11);
    assert_eq!(read(APB1EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f10x_enable_timer0_sets_apb2en_bit11() {
    let _g = seed_reset();
    enable_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Timer0).unwrap();
    // TIMER0EN = BIT(11) on APB2EN: the SAME bit on both families.
    assert_eq!(read(APB2EN), 1 << 11);
    assert_eq!(read(APB1EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f10x_enable_timer7_sets_apb2en_bit13() {
    let _g = seed_reset();
    enable_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Timer7).unwrap();
    // TIMER7EN = BIT(13) on APB2EN (gd32f10x_rcu.h:266); F10x-only.
    assert_eq!(read(APB2EN), 1 << 13);
}

#[test]
fn f1x0_has_no_timer7() {
    let _g = seed_reset();
    // F1x0 has no TIMER7; the path rejects it rather than writing a wrong bit.
    assert_eq!(
        enable_timer(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Timer7),
        Err(DescriptorError::SelectorAddrMismatch)
    );
    assert_eq!(read(APB2EN), 0);
}

#[test]
fn enable_timer_rejects_non_timer_label() {
    let _g = seed_reset();
    assert_eq!(
        enable_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(
        enable_timer(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Gpioa),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(read(APB2EN), 0);
}

#[test]
fn enable_timer0_preserves_other_apb2en_bits() {
    let _g = seed_reset();
    // A pre-existing APB2 enable (e.g. GPIOA on F10x, bit 2) must survive the RMW.
    Reg32::new(RCU_BASE, APB2EN).write(1 << 2);
    enable_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Timer0).unwrap();
    assert_eq!(read(APB2EN), (1 << 2) | (1 << 11));
}

// --- G3: the GENERAL-purpose timer clock enable (TIMER1 on APB1EN bit 0, BOTH families) --------

#[test]
fn f1x0_enable_general_timer1_sets_apb1en_bit0() {
    let _g = seed_reset();
    enable_general_timer(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Timer1).unwrap();
    // TIMER1EN = BIT(0) on APB1EN (GD32F1x0 User Manual RCU APB1EN); nothing else touched.
    assert_eq!(read(APB1EN), 1 << 0);
    assert_eq!(read(APB2EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn f10x_enable_general_timer1_sets_apb1en_bit0() {
    let _g = seed_reset();
    enable_general_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Timer1).unwrap();
    // TIMER1EN = BIT(0) on APB1EN: the SAME bit on both families (GD32F10x User Manual line 5425).
    assert_eq!(read(APB1EN), 1 << 0);
    assert_eq!(read(APB2EN), 0);
    assert_eq!(read(AHBEN), 0);
}

#[test]
fn enable_general_timer_rejects_advanced_and_non_timer_labels() {
    let _g = seed_reset();
    // The ADVANCED timers are NOT general timers: this path must never enable TIMER0 (the bridge).
    assert_eq!(
        enable_general_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Timer0),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(
        enable_general_timer(RCU_BASE, ClockPath::F10xRcc, PeriphLabel::Usart1),
        Err(DescriptorError::UnknownSelector)
    );
    assert_eq!(read(APB1EN), 0);
    assert_eq!(read(APB2EN), 0);
}

#[test]
fn enable_general_timer1_preserves_other_apb1en_bits() {
    let _g = seed_reset();
    // A pre-existing APB1 enable (e.g. USART1 on bit 17) must survive the RMW.
    Reg32::new(RCU_BASE, APB1EN).write(1 << 17);
    enable_general_timer(RCU_BASE, ClockPath::F1x0Rcu, PeriphLabel::Timer1).unwrap();
    assert_eq!(read(APB1EN), (1 << 17) | (1 << 0));
}

// --- T2: full clock tree (configure_tree) -----------------------------------------------------

mod tree_tests {
    use crate::addr::{AddrTable, PeriphLabel};
    use crate::chip::Chip;
    use crate::clock::{configure_tree, ClockConfig, ClockSource};
    use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
    use crate::reg::{mock, Reg32};
    use std::sync::MutexGuard;

    const RCU_BASE: u32 = 0x4002_1000;
    const FMC_WS: u32 = 0x4002_2000;
    const CTL: u32 = 0x00;
    const CFG0: u32 = 0x04;

    // Bits, mirroring the source (kept local so the test is an independent oracle).
    const CTL_IRC8MSTB: u32 = 1 << 1;
    const CTL_HXTALSTB: u32 = 1 << 17;
    const CTL_PLLSTB: u32 = 1 << 25;
    const SCSS_PLL: u32 = 0b10 << 2;

    /// Seed the mock so the bring-up polls all exit immediately: the source-stable, PLL-lock, and
    /// SCS-confirm flags are pre-set (the mock has no clock HW that would set them). A real
    /// silicon / Unicorn run scripts these busy->busy->done (the with_polling golden); here we
    /// only assert the END register state, so pre-setting the flags is correct.
    fn seed(source: ClockSource) -> MutexGuard<'static, ()> {
        let g = mock::lock();
        mock::reset();
        let stable = match source {
            ClockSource::Irc8m => CTL_IRC8MSTB,
            ClockSource::Hxtal => CTL_HXTALSTB,
        };
        // CTL: source-stable + PLL-lock pre-set.
        Reg32::new(RCU_BASE, CTL).write(stable | CTL_PLLSTB);
        // CFG0: SCSS = PLL pre-set so the read-back confirm exits. configure_tree's RMWs preserve
        // these read-only status bits (modify only clears+sets the field it targets).
        Reg32::new(RCU_BASE, CFG0).write(SCSS_PLL);
        Reg32::new(FMC_WS, 0).write(0);
        g
    }

    fn default_72m() -> ClockConfig {
        ClockConfig::REFERENCE_72M_IRC8M
    }

    /// Build a `Chip` whose RCU base is `RCU_BASE` and whose clock-tree path is `path`. `bring_up`
    /// derives the RCU base and the path from the chip, so the configure_tree calls read both here.
    fn chip(path: ClockPath) -> Chip {
        let mut addrs = AddrTable::new();
        addrs.set(PeriphLabel::Rcu, RCU_BASE);
        Chip::from_descriptor(McuDescriptor {
            gpio: if path == ClockPath::F1x0Rcu {
                GpioPath::AhbCtlAfsel
            } else {
                GpioPath::ApbCrlCrh
            },
            clock: path,
            adc: AdcPath::Single,
            irq: IrqLayout::F1x0Grouped,
            addrs,
            flash_page: PageSize::K1,
            adv_timers: 1,
            adc_count: 1,
        })
    }

    #[test]
    fn default_72m_tree_programs_expected_registers_f1x0() {
        let _g = seed(ClockSource::Irc8m);
        configure_tree(&chip(ClockPath::F1x0Rcu), &default_72m()).expect("valid 72M config");

        // Flash wait states = 2.
        assert_eq!(Reg32::new(FMC_WS, 0).read() & 0b111, 2);

        // CFG0 end state: APB1PSC = /2 (code 4 << 8 = 0x400), AHBPSC = /1 (0), APB2PSC = /1 (0),
        // PLLSEL = IRC8M (0), PLLMF = mul18 (BIT(27)|BIT(18) = 0x0804_0000), SCS = PLL (0x2). The
        // pre-seeded SCSS read-back bits (0x8) are preserved by the field-scoped RMWs.
        let cfg0 = Reg32::new(RCU_BASE, CFG0).read();
        // Mask out the read-only SCSS status bits before comparing the programmed fields.
        let programmed = cfg0 & !SCSS_PLL;
        assert_eq!(programmed, 0x0804_0000 | 0x400 | 0x2, "cfg0 = {cfg0:#010x}");
    }

    #[test]
    fn default_72m_tree_programs_expected_registers_f10x() {
        let _g = seed(ClockSource::Irc8m);
        // F10x uses the same register layout; the per-family divergence is the profile prescalers
        // (here both families share the default profile, so the programmed bits match).
        configure_tree(&chip(ClockPath::F10xRcc), &default_72m()).expect("valid 72M config");
        assert_eq!(Reg32::new(FMC_WS, 0).read() & 0b111, 2);
        let cfg0 = Reg32::new(RCU_BASE, CFG0).read() & !SCSS_PLL;
        assert_eq!(cfg0, 0x0804_0000 | 0x400 | 0x2);
    }

    #[test]
    fn hxtal_source_sets_pllsel() {
        let _g = seed(ClockSource::Hxtal);
        let p = ClockConfig {
            source: ClockSource::Hxtal,
            ..default_72m()
        };
        configure_tree(&chip(ClockPath::F10xRcc), &p).expect("valid hxtal 72M config");
        // PLLSEL = BIT(16) set for HXTAL.
        let cfg0 = Reg32::new(RCU_BASE, CFG0).read();
        assert_ne!(cfg0 & (1 << 16), 0, "PLLSEL must be set for HXTAL");
    }

    #[test]
    fn wait_states_programmed_first_and_match_profile() {
        let _g = seed(ClockSource::Irc8m);
        // 48 MHz with 1 WS is a valid F1x0 config (>=0 WS minimum at 48 MHz; 1 is allowed). The
        // intent: configure_tree programs FMC WSCNT to exactly the config's wait-states.
        let p = ClockConfig {
            sysclk_hz: 48_000_000,
            wait_states: 1,
            ..default_72m()
        };
        configure_tree(&chip(ClockPath::F1x0Rcu), &p).expect("valid 48M / 1WS config");
        assert_eq!(Reg32::new(FMC_WS, 0).read() & 0b111, 1);
    }

    // --- Q2: the bounded-timeout variant (configure_tree_timeout) ----------------------------
    //
    // The bounded variant shares configure_tree's register sequence but gives up after a bounded
    // spin budget on each wait gate, so a board that never stabilises / locks fails cleanly with a
    // ClockError instead of hanging. Host-tested only (NO emulated golden: the M2 goldens diff
    // against the SPL's unbounded poll, so the bounded variant must not get its own trace).

    use crate::clock::configure_tree_timeout;
    use crate::error::ClockError;

    /// Seed like `seed`, but DO NOT set the PLL-lock bit, so the PLL-lock gate never exits. The
    /// source-stable bit IS set so the bring-up reaches the PLL-lock wait (the one under test).
    fn seed_pll_never_locks() -> MutexGuard<'static, ()> {
        let g = mock::lock();
        mock::reset();
        // IRC8M stable so gate 2 passes; PLLSTB deliberately CLEAR so gate 5 never exits; SCSS PLL
        // set so we would only fail at the PLL gate, not the switch gate.
        Reg32::new(RCU_BASE, CTL).write(CTL_IRC8MSTB);
        Reg32::new(RCU_BASE, CFG0).write(SCSS_PLL);
        Reg32::new(FMC_WS, 0).write(0);
        g
    }

    #[test]
    fn bounded_variant_returns_err_when_pll_never_locks_and_does_not_hang() {
        let _g = seed_pll_never_locks();
        // A small cap so the test is fast; the point is it RETURNS (bounded) rather than spinning
        // forever like configure_tree would on this never-locking mock.
        let r = configure_tree_timeout(&chip(ClockPath::F1x0Rcu), &default_72m(), 32);
        assert_eq!(r, Err(ClockError::PllNotLocked));
    }

    /// Seed so the source-stable gate never exits (no STB bit at all), exercising the first gate's
    /// bound.
    fn seed_source_never_stable() -> MutexGuard<'static, ()> {
        let g = mock::lock();
        mock::reset();
        Reg32::new(RCU_BASE, CTL).write(0); // no IRC8MSTB
        Reg32::new(RCU_BASE, CFG0).write(SCSS_PLL);
        Reg32::new(FMC_WS, 0).write(0);
        g
    }

    #[test]
    fn bounded_variant_times_out_on_unstable_source() {
        let _g = seed_source_never_stable();
        let r = configure_tree_timeout(&chip(ClockPath::F10xRcc), &default_72m(), 16);
        assert_eq!(r, Err(ClockError::SourceNotStable));
    }

    #[test]
    fn bounded_variant_ok_and_writes_match_the_unbounded_path() {
        // When every gate's flag IS set (the same `seed` the unbounded tests use), the bounded
        // variant returns Ok AND leaves the exact same register state as configure_tree: same
        // FMC_WS, same CFG0 programmed fields. (Same MMIO writes; only the loop exit differs.)
        let expected_cfg0 = 0x0804_0000 | 0x400 | 0x2;

        // Reference: the unbounded path's end state.
        {
            let _g = seed(ClockSource::Irc8m);
            configure_tree(&chip(ClockPath::F1x0Rcu), &default_72m()).expect("valid 72M config");
            assert_eq!(Reg32::new(FMC_WS, 0).read() & 0b111, 2);
            assert_eq!(Reg32::new(RCU_BASE, CFG0).read() & !SCSS_PLL, expected_cfg0);
        }

        // The bounded path with a generous cap reaches Ok and the identical end state.
        {
            let _g = seed(ClockSource::Irc8m);
            let r = configure_tree_timeout(&chip(ClockPath::F1x0Rcu), &default_72m(), 1_000);
            assert_eq!(r, Ok(()));
            assert_eq!(Reg32::new(FMC_WS, 0).read() & 0b111, 2);
            assert_eq!(Reg32::new(RCU_BASE, CFG0).read() & !SCSS_PLL, expected_cfg0);
        }
    }

    #[test]
    fn pll_mul_and_prescalers_encode_correctly() {
        let _g = seed(ClockSource::Irc8m);
        // A non-default profile: pll_mul 6 (low range, PLLMF4 clear), AHB /2, APB1 /4, APB2 /8.
        let p = ClockConfig {
            sysclk_hz: 48_000_000,
            wait_states: 1,
            source: ClockSource::Irc8m,
            pll_mul: 6,
            ahb_psc: 2,
            apb1_psc: 4,
            apb2_psc: 8,
        };
        configure_tree(&chip(ClockPath::F10xRcc), &p).expect("valid 48M config");
        let cfg0 = Reg32::new(RCU_BASE, CFG0).read() & !SCSS_PLL;
        // PLLMF mul6 = (6-2)<<18 = 4<<18 = 0x0010_0000 (PLLMF4 clear).
        // AHBPSC /2 = code 8 << 4 = 0x80. APB2PSC /8 = code 6 << 11 = 0x3000.
        // APB1PSC /4 = code 5 << 8 = 0x500. SCS = PLL = 0x2.
        let expected = 0x0010_0000 | 0x80 | 0x3000 | 0x500 | 0x2;
        assert_eq!(
            cfg0, expected,
            "cfg0 = {cfg0:#010x}, expected {expected:#010x}"
        );
    }

    // --- DR-1: ClockConfig::validate_for (the chip-bound range check) -------------------------

    #[test]
    fn validate_for_accepts_the_reference_72m_config() {
        // The proven 72 MHz / 2 WS reference tree is in range for the F1x0 path: pll18, all
        // prescalers legal, 2 WS at 72 MHz is the required flash timing.
        assert_eq!(
            ClockConfig::REFERENCE_72M_IRC8M.validate_for(ClockPath::F1x0Rcu),
            Ok(())
        );
    }

    #[test]
    fn validate_for_rejects_out_of_range_pll_mul() {
        // pll_mul 40 is outside the PLLMF field's 2..=32 range, so validation rejects it as
        // InvalidPll before any register is touched.
        let p = ClockConfig {
            pll_mul: 40,
            ..ClockConfig::REFERENCE_72M_IRC8M
        };
        assert_eq!(
            p.validate_for(ClockPath::F1x0Rcu),
            Err(ClockError::InvalidPll)
        );
    }

    // ============================================================================================
    // DR-T9: clock-configuration test completeness
    //
    // This block is the four-part DR-T9 deliverable for the clock tree:
    //   1. PERMUTATION MATRIX (proptest): every valid (source, pll_mul, ahb_psc, apb1_psc,
    //      apb2_psc) combination, on BOTH clock paths, diffed against an independent re-encoding of
    //      the GD SPL CFG0/CTL bit layout (the cheap host oracle, same idea as the M1 BRR sweep).
    //   2. CURATED BOUNDARIES: min/max pll_mul, every legal prescaler value on each bus, the
    //      wait-state thresholds, the IRC8M-vs-HXTAL source switch, as explicit named tests.
    //   3. REJECT TESTS: every out-of-range / chip-illegal combination asserts validate_for AND
    //      configure_tree return the correct ClockError variant.
    //
    // The SPL oracle below is encoded INDEPENDENTLY from src/clock.rs: it uses lookup tables keyed
    // directly off the SPL header's RCU_*_DIV* / RCU_PLL_MUL* CFG0_*(regval) macros
    // (gd32f1x0_rcu.h:588-661, identical on gd32f10x_rcu.h), so a mistake in the source encoder
    // (e.g. an unsupported divisor silently mapping to /1) is caught as a diff rather than papered
    // over by reusing the source's own table.
    // ============================================================================================
    mod dr_t9 {
        use super::{chip, seed, CFG0, CTL, FMC_WS, RCU_BASE, SCSS_PLL};
        use crate::clock::{configure_tree, ClockConfig, ClockSource};
        use crate::descriptor::ClockPath;
        use crate::error::ClockError;
        use crate::reg::Reg32;

        // --- the SPL CFG0/CTL oracle (independent re-encoding of the SPL header bit fields) -----

        /// SPL CTL oscillator-enable bit for the source (gd32f1x0_rcu.h CTL: IRC8MEN BIT(0),
        /// HXTALEN BIT(16)). The configure_tree sequence sets exactly this bit on CTL.
        fn spl_ctl_source_enable(source: ClockSource) -> u32 {
            match source {
                ClockSource::Irc8m => 1 << 0,  // RCU_CTL0_IRC8MEN
                ClockSource::Hxtal => 1 << 16, // RCU_CTL0_HXTALEN
            }
        }

        /// SPL `RCU_AHB_CKSYS_DIV*` field value: `CFG0_AHBPSC(code) = code << 4`, codes from
        /// gd32f1x0_rcu.h:588-596. Returns `None` for a divisor the SPL has no encoding for (so the
        /// oracle never invents a code, and an out-of-table divisor is a test error not a silent /1).
        fn spl_ahbpsc(div: u16) -> Option<u32> {
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
                _ => return None,
            };
            Some(code << 4)
        }

        /// SPL `RCU_APBx_CKAHB_DIV*` field code (pre-shift): codes from gd32f1x0_rcu.h:600-612. APB1
        /// shifts <<8, APB2 shifts <<11.
        fn spl_apb_code(div: u16) -> Option<u32> {
            Some(match div {
                1 => 0,
                2 => 4,
                4 => 5,
                8 => 6,
                16 => 7,
                _ => return None,
            })
        }

        /// SPL `RCU_PLL_MUL*` CFG0 bits: mul 2..=16 -> `CFG0_PLLMF(mul-2)` (BITS(18,21), PLLMF4
        /// clear); mul 17..=32 -> `RCU_CFG0_PLLMF4 | CFG0_PLLMF(mul-17)` (BIT(27) set). Encoded from
        /// gd32f1x0_rcu.h:630-661 (RCU_CFG0_PLLMF4 = BIT(27), gd32f1x0_rcu.h:103).
        fn spl_pllmf(mul: u8) -> Option<u32> {
            let m = mul as u32;
            if (2..=16).contains(&m) {
                Some((m - 2) << 18)
            } else if (17..=32).contains(&m) {
                Some((1 << 27) | ((m - 17) << 18))
            } else {
                None
            }
        }

        /// SPL PLLSEL bit (CFG0 BIT(16)): clear for IRC8M/2, set for HXTAL (gd32f1x0_rcu.h:622-623).
        fn spl_pllsel(source: ClockSource) -> u32 {
            match source {
                ClockSource::Irc8m => 0,
                ClockSource::Hxtal => 1 << 16,
            }
        }

        /// SCS = PLL (CFG0 BITS(0,1) = 2, gd32f1x0_rcu.h:578). configure_tree switches to PLL last.
        const SPL_SCS_PLL: u32 = 0b10 << 0;

        /// The full programmed CFG0 the SPL would leave for this config (the fields configure_tree
        /// RMWs: AHBPSC, APB2PSC, APB1PSC, PLLSEL, PLLMF, SCS). Excludes the read-only SCSS bits.
        /// Returns `None` if any field has no SPL encoding (an out-of-table divisor/mul).
        fn spl_cfg0(cfg: &ClockConfig) -> Option<u32> {
            let ahb = spl_ahbpsc(cfg.ahb_psc)?;
            let apb1 = spl_apb_code(cfg.apb1_psc)? << 8;
            let apb2 = spl_apb_code(cfg.apb2_psc)? << 11;
            let pllmf = spl_pllmf(cfg.pll_mul)?;
            let pllsel = spl_pllsel(cfg.source);
            Some(ahb | apb1 | apb2 | pllmf | pllsel | SPL_SCS_PLL)
        }

        /// Run configure_tree for `cfg` on `path` against a freshly-seeded mock (polls pre-satisfied
        /// for `cfg.source`), returning the programmed CFG0 (SCSS masked off), the CTL value, and the
        /// FMC wait-state field. Holds the mock serialization guard for the whole read-back.
        fn run_and_read(cfg: &ClockConfig, path: ClockPath) -> (u32, u32, u32) {
            let _g = seed(cfg.source);
            configure_tree(&chip(path), cfg).expect("valid config must configure");
            let cfg0 = Reg32::new(RCU_BASE, CFG0).read() & !SCSS_PLL;
            let ctl = Reg32::new(RCU_BASE, CTL).read();
            let ws = Reg32::new(FMC_WS, 0).read() & 0b111;
            (cfg0, ctl, ws)
        }

        // --- (1) PERMUTATION MATRIX ------------------------------------------------------------
        //
        // SWEEP DESIGN (logged per TESTING.md; no silent capping):
        //   - source:   EXHAUSTIVE, both variants {Irc8m, Hxtal}.
        //   - pll_mul:  EXHAUSTIVE, all 31 legal values 2..=32.
        //   - ahb_psc:  EXHAUSTIVE over the configure_tree-supported set {1,2,4,8,16}. The four
        //               larger SPL-legal divisors {64,128,256,512} are swept SEPARATELY in
        //               `large_ahb_psc_*` below because they exposed a source/SPL disagreement
        //               (see that test's comment) and must not silently pass here.
        //   - apb1_psc: EXHAUSTIVE, all 5 legal values {1,2,4,8,16}.
        //   - apb2_psc: EXHAUSTIVE, all 5 legal values {1,2,4,8,16}.
        //   - path:     EXHAUSTIVE, both {F10xRcc, F1x0Rcu}.
        //   - sysclk/wait_states: held at a fixed in-range pair (72 MHz / 2 WS) so every swept combo
        //               PASSES validate_for; the register end-state under test does NOT depend on
        //               sysclk/wait_states beyond the FMC WSCNT field, which the boundary tests
        //               sweep exhaustively. The full Cartesian product (2*31*5*5*5*2 = 15,500
        //               combos) is enumerated DETERMINISTICALLY below (not proptest-sampled), so the
        //               matrix is truly exhaustive over the chosen axes, not a random subset.

        #[test]
        fn permutation_matrix_cfg0_matches_spl_oracle_both_paths() {
            let sources = [ClockSource::Irc8m, ClockSource::Hxtal];
            let small_ahb = [1u16, 2, 4, 8, 16];
            let apb = [1u16, 2, 4, 8, 16];
            let paths = [ClockPath::F10xRcc, ClockPath::F1x0Rcu];

            let mut checked = 0u32;
            for &source in &sources {
                for pll_mul in 2u8..=32 {
                    for &ahb_psc in &small_ahb {
                        for &apb1_psc in &apb {
                            for &apb2_psc in &apb {
                                let cfg = ClockConfig {
                                    sysclk_hz: 72_000_000,
                                    wait_states: 2,
                                    source,
                                    pll_mul,
                                    ahb_psc,
                                    apb1_psc,
                                    apb2_psc,
                                };
                                // Every swept combo must be accepted by validate_for (so the matrix
                                // is over VALID configs, as DR-T9 requires).
                                for &path in &paths {
                                    assert_eq!(
                                        cfg.validate_for(path),
                                        Ok(()),
                                        "swept combo should be valid: {cfg:?} on {path:?}"
                                    );
                                    let expected_cfg0 =
                                        spl_cfg0(&cfg).expect("oracle has all swept fields");
                                    let (got_cfg0, got_ctl, got_ws) = run_and_read(&cfg, path);
                                    assert_eq!(
                                        got_cfg0, expected_cfg0,
                                        "CFG0 disagrees with SPL oracle: {cfg:?} on {path:?}: \
                                         got {got_cfg0:#010x}, want {expected_cfg0:#010x}"
                                    );
                                    // CTL: the source's oscillator-enable bit is set.
                                    let want_en = spl_ctl_source_enable(source);
                                    assert_eq!(
                                        got_ctl & want_en,
                                        want_en,
                                        "CTL source-enable bit missing: {cfg:?} on {path:?}"
                                    );
                                    // FMC WSCNT = the config's wait_states.
                                    assert_eq!(got_ws, 2, "FMC WSCNT must equal wait_states");
                                    checked += 1;
                                }
                            }
                        }
                    }
                }
            }
            // Sweep-size assertion: 2 sources * 31 muls * 5 ahb * 5 apb1 * 5 apb2 * 2 paths.
            assert_eq!(checked, 2 * 31 * 5 * 5 * 5 * 2, "exhaustive matrix size");
        }

        // --- (2) CURATED BOUNDARIES ------------------------------------------------------------

        /// A base config at a benign in-range point; boundary tests vary one axis off it.
        fn base() -> ClockConfig {
            ClockConfig::REFERENCE_72M_IRC8M
        }

        #[test]
        fn boundary_min_pll_mul_2() {
            // pll_mul = 2 is the field minimum: CFG0_PLLMF(0) = 0, PLLMF4 clear. At IRC8M/2 = 4 MHz
            // * 2 = 8 MHz this is a legal low-clock tree (0 WS region, but we keep 0 WS so it stays
            // valid; min_ws at 8 MHz is 0).
            let cfg = ClockConfig {
                sysclk_hz: 8_000_000,
                wait_states: 0,
                pll_mul: 2,
                ..base()
            };
            assert_eq!(cfg.validate_for(ClockPath::F1x0Rcu), Ok(()));
            let (got, _, _) = run_and_read(&cfg, ClockPath::F1x0Rcu);
            assert_eq!(got & ((1 << 27) | (0b1111 << 18)), 0, "PLLMF(mul2) = 0");
            assert_eq!(got, spl_cfg0(&cfg).unwrap());
        }

        #[test]
        fn boundary_max_pll_mul_32() {
            // pll_mul = 32 is the field maximum: PLLMF4 (BIT 27) set, CFG0_PLLMF(15) = 15<<18. We
            // pick a sysclk/WS that stays valid (high mul -> high clock); 96 MHz / 2 WS is in range
            // on the F10x path (<=108 MHz ceiling; F10x is zero-wait, so any WS 0..=2 is accepted).
            let cfg = ClockConfig {
                sysclk_hz: 96_000_000,
                wait_states: 2,
                pll_mul: 32,
                ..base()
            };
            assert_eq!(cfg.validate_for(ClockPath::F10xRcc), Ok(()));
            let (got, _, _) = run_and_read(&cfg, ClockPath::F10xRcc);
            assert_eq!(
                got & ((1 << 27) | (0b1111 << 18)),
                (1 << 27) | (15 << 18),
                "PLLMF(mul32) = PLLMF4 | 15<<18"
            );
            assert_eq!(got, spl_cfg0(&cfg).unwrap());
        }

        #[test]
        fn boundary_pll_mul_16_17_straddle_the_pllmf4_split() {
            // mul 16 is the last PLLMF4-clear value (CFG0_PLLMF(14)); mul 17 is the first PLLMF4-set
            // value (PLLMF4 | CFG0_PLLMF(0)). This pins the BIT(27) split the source's pll_mul_bits
            // performs.
            for (mul, want) in [(16u8, 14u32 << 18), (17u8, 1 << 27)] {
                let cfg = ClockConfig {
                    sysclk_hz: 64_000_000,
                    wait_states: 2,
                    pll_mul: mul,
                    ..base()
                };
                assert_eq!(cfg.validate_for(ClockPath::F1x0Rcu), Ok(()));
                let (got, _, _) = run_and_read(&cfg, ClockPath::F1x0Rcu);
                assert_eq!(got & ((1 << 27) | (0b1111 << 18)), want, "mul {mul} PLLMF");
            }
        }

        #[test]
        fn boundary_every_legal_apb1_psc_value() {
            // Each legal APB1 divisor {1,2,4,8,16} programs its SPL APB1PSC code at <<8.
            for &div in &[1u16, 2, 4, 8, 16] {
                let cfg = ClockConfig {
                    apb1_psc: div,
                    ..base()
                };
                assert_eq!(cfg.validate_for(ClockPath::F1x0Rcu), Ok(()));
                let (got, _, _) = run_and_read(&cfg, ClockPath::F1x0Rcu);
                let want = spl_apb_code(div).unwrap() << 8;
                assert_eq!(got & (0b111 << 8), want, "APB1PSC for /{div}");
                assert_eq!(got, spl_cfg0(&cfg).unwrap());
            }
        }

        #[test]
        fn boundary_every_legal_apb2_psc_value() {
            // Each legal APB2 divisor {1,2,4,8,16} programs its SPL APB2PSC code at <<11.
            for &div in &[1u16, 2, 4, 8, 16] {
                let cfg = ClockConfig {
                    apb2_psc: div,
                    ..base()
                };
                assert_eq!(cfg.validate_for(ClockPath::F10xRcc), Ok(()));
                let (got, _, _) = run_and_read(&cfg, ClockPath::F10xRcc);
                let want = spl_apb_code(div).unwrap() << 11;
                assert_eq!(got & (0b111 << 11), want, "APB2PSC for /{div}");
                assert_eq!(got, spl_cfg0(&cfg).unwrap());
            }
        }

        #[test]
        fn boundary_every_small_ahb_psc_value() {
            // The configure_tree-supported AHB divisors {1,2,4,8,16}: each programs its SPL AHBPSC
            // code at <<4. (The larger {64,128,256,512} are covered by `large_ahb_psc_*`.)
            for &div in &[1u16, 2, 4, 8, 16] {
                let cfg = ClockConfig {
                    ahb_psc: div,
                    ..base()
                };
                assert_eq!(cfg.validate_for(ClockPath::F1x0Rcu), Ok(()));
                let (got, _, _) = run_and_read(&cfg, ClockPath::F1x0Rcu);
                let want = spl_ahbpsc(div).unwrap();
                assert_eq!(got & (0b1111 << 4), want, "AHBPSC for /{div}");
            }
        }

        #[test]
        fn boundary_wait_state_thresholds_match_validate_for_bands() {
            // The F1x0 flash-timing bands: 0 WS up to 48 MHz (GD32F130 zero-wait), 1 WS above 48 MHz
            // (conservative floor) up to the 72 MHz ceiling. At each band edge the minimum legal WS is
            // accepted and one fewer is rejected, and the accepted WS reaches the FMC WSCNT field.
            // This pins where WSCNT changes with sysclk on the F1x0 path.
            // (sysclk_hz, min_ws_accepted)
            let bands = [
                (48_000_000u32, 0u8),
                (48_000_001u32, 1u8),
                (72_000_000u32, 1u8),
            ];
            for (sysclk, min_ws) in bands {
                let ok = ClockConfig {
                    sysclk_hz: sysclk,
                    wait_states: min_ws,
                    ..base()
                };
                assert_eq!(
                    ok.validate_for(ClockPath::F1x0Rcu),
                    Ok(()),
                    "min WS {min_ws} at {sysclk} Hz should be accepted"
                );
                let (_, _, ws) = run_and_read(&ok, ClockPath::F1x0Rcu);
                assert_eq!(
                    ws, min_ws as u32,
                    "FMC WSCNT must equal accepted wait_states"
                );

                // One fewer wait-state at the same clock is rejected (too few for the flash timing).
                if min_ws > 0 {
                    let too_few = ClockConfig {
                        wait_states: min_ws - 1,
                        ..ok
                    };
                    assert_eq!(
                        too_few.validate_for(ClockPath::F1x0Rcu),
                        Err(ClockError::InvalidWaitStates),
                        "{} WS at {sysclk} Hz is too few",
                        min_ws - 1
                    );
                }
            }
        }

        #[test]
        fn boundary_source_switch_irc8m_vs_hxtal() {
            // The source switch: IRC8M clears PLLSEL and enables IRC8MEN; HXTAL sets PLLSEL and
            // enables HXTALEN. Same tree otherwise, so only the source-dependent bits differ.
            let irc = ClockConfig {
                source: ClockSource::Irc8m,
                ..base()
            };
            let hxt = ClockConfig {
                source: ClockSource::Hxtal,
                ..base()
            };
            assert_eq!(irc.validate_for(ClockPath::F1x0Rcu), Ok(()));
            assert_eq!(hxt.validate_for(ClockPath::F1x0Rcu), Ok(()));

            let (irc_cfg0, irc_ctl, _) = run_and_read(&irc, ClockPath::F1x0Rcu);
            assert_eq!(irc_cfg0 & (1 << 16), 0, "IRC8M clears PLLSEL");
            assert_eq!(irc_ctl & (1 << 0), 1 << 0, "IRC8M sets IRC8MEN");
            assert_eq!(irc_cfg0, spl_cfg0(&irc).unwrap());

            let (hxt_cfg0, hxt_ctl, _) = run_and_read(&hxt, ClockPath::F1x0Rcu);
            assert_eq!(hxt_cfg0 & (1 << 16), 1 << 16, "HXTAL sets PLLSEL");
            assert_eq!(hxt_ctl & (1 << 16), 1 << 16, "HXTAL sets HXTALEN");
            assert_eq!(hxt_cfg0, spl_cfg0(&hxt).unwrap());
        }

        // The large AHB divisors {64,128,256,512} are SPL-legal (RCU_AHB_CKSYS_DIV64..512,
        // gd32f1x0_rcu.h:593-596) and validate_for accepts them, so a config using them is a VALID
        // combination the permutation matrix is required to cover. This test diffs configure_tree's
        // AHBPSC field against the SPL oracle for each.
        //
        // GENUINE DISCREPANCY FOUND BY THIS SWEEP (DR-T9, reported, NOT papered over):
        //   `ClockConfig::validate_for` ACCEPTS ahb_psc in {64,128,256,512} (is_legal_ahb_psc lists
        //   them, matching the SPL's RCU_AHB_CKSYS_DIV64..512), but `clock::ahb_psc_bits` has no
        //   match arm for them and falls through `_ => 0`, so `configure_tree` programs AHBPSC = /1
        //   for every one of them. Concretely: ahb_psc = 64 -> configure_tree writes AHBPSC code 0
        //   (/1) where the SPL programs code 12 (0xC0). A firmware that legally asks for AHB /64
        //   passes validation and silently runs the AHB bus 64x too fast.
        //
        //   FIXED: `ahb_psc_bits` gained the 64/128/256/512 -> 12/13/14/15 arms (the SPL codes this
        //   oracle encodes), so `configure_tree` now programs the large AHB divisors correctly. This
        //   test (the DR-T9 bug record) is active and passes; it guards against the regression.
        #[test]
        fn large_ahb_psc_matches_spl_oracle() {
            for &div in &[64u16, 128, 256, 512] {
                let cfg = ClockConfig {
                    // A low sysclk so the divided AHB stays sane; validity does not depend on it.
                    sysclk_hz: 8_000_000,
                    wait_states: 0,
                    ahb_psc: div,
                    ..base()
                };
                assert_eq!(
                    cfg.validate_for(ClockPath::F1x0Rcu),
                    Ok(()),
                    "AHB /{div} is an SPL-legal divisor and validate_for accepts it"
                );
                let want = spl_ahbpsc(div).expect("SPL has a code for /{div}");
                let (got, _, _) = run_and_read(&cfg, ClockPath::F1x0Rcu);
                assert_eq!(
                    got & (0b1111 << 4),
                    want,
                    "AHBPSC for /{div}: configure_tree must program the SPL code, \
                     got {:#x}, want {want:#x}",
                    got & (0b1111 << 4)
                );
            }
        }

        // --- (3) REJECT TESTS ------------------------------------------------------------------
        //
        // Each out-of-range / chip-illegal combination must be rejected by BOTH validate_for AND
        // configure_tree (configure_tree calls validate_for first), with the CORRECT ClockError
        // variant, on BOTH paths (the legal ranges are family-independent here, so both reject).

        /// Assert a config is rejected with `want` by validate_for AND configure_tree, both paths.
        fn assert_rejected(cfg: &ClockConfig, want: ClockError) {
            for &path in &[ClockPath::F10xRcc, ClockPath::F1x0Rcu] {
                assert_eq!(
                    cfg.validate_for(path),
                    Err(want),
                    "validate_for({path:?}) should reject {cfg:?} as {want:?}"
                );
                // configure_tree must reject before touching any wait gate (validate_for first).
                let _g = seed(cfg.source);
                assert_eq!(
                    configure_tree(&chip(path), cfg),
                    Err(want),
                    "configure_tree({path:?}) should reject {cfg:?} as {want:?}"
                );
            }
        }

        #[test]
        fn reject_pll_mul_below_range() {
            // pll_mul 1 is below the 2..=32 PLLMF range.
            assert_rejected(
                &ClockConfig {
                    pll_mul: 1,
                    ..base()
                },
                ClockError::InvalidPll,
            );
        }

        #[test]
        fn reject_pll_mul_above_range() {
            // pll_mul 33 is above the 2..=32 PLLMF range.
            assert_rejected(
                &ClockConfig {
                    sysclk_hz: 96_000_000,
                    wait_states: 2,
                    pll_mul: 33,
                    ..base()
                },
                ClockError::InvalidPll,
            );
        }

        #[test]
        fn reject_illegal_ahb_psc() {
            // 32 is not a legal AHB divisor (the AHB skips /32: ..16, then 64..). Rejected.
            assert_rejected(
                &ClockConfig {
                    ahb_psc: 32,
                    ..base()
                },
                ClockError::InvalidPrescaler,
            );
            // 3 (non-power-of-two) is illegal too.
            assert_rejected(
                &ClockConfig {
                    ahb_psc: 3,
                    ..base()
                },
                ClockError::InvalidPrescaler,
            );
        }

        #[test]
        fn reject_illegal_apb1_psc() {
            // 32 exceeds the APB max divisor of 16.
            assert_rejected(
                &ClockConfig {
                    apb1_psc: 32,
                    ..base()
                },
                ClockError::InvalidPrescaler,
            );
        }

        #[test]
        fn reject_illegal_apb2_psc() {
            assert_rejected(
                &ClockConfig {
                    apb2_psc: 64,
                    ..base()
                },
                ClockError::InvalidPrescaler,
            );
        }

        #[test]
        fn reject_wait_states_too_few_for_sysclk() {
            // Too-few-WS is an F1x0-path concern: F10x is zero-wait at the full 108 MHz, so no clock
            // ever requires a wait state there. On the F1x0 path, above the 48 MHz zero-wait point a
            // wait state is required, so 0 WS at 60 MHz and at 72 MHz are below the 1-WS minimum.
            for sysclk in [60_000_000u32, 72_000_000] {
                assert_eq!(
                    ClockConfig {
                        sysclk_hz: sysclk,
                        wait_states: 0,
                        ..base()
                    }
                    .validate_for(ClockPath::F1x0Rcu),
                    Err(ClockError::InvalidWaitStates),
                    "0 WS at {sysclk} Hz is too few on F1x0 (>48 MHz needs >=1 WS)"
                );
            }
            // The same 0 WS at 48 MHz is now VALID on F1x0 (GD32F130 is zero-wait at 48 MHz), and is
            // always valid on F10x (zero-wait at any clock).
            let at_48 = ClockConfig {
                sysclk_hz: 48_000_000,
                wait_states: 0,
                ..base()
            };
            assert_eq!(at_48.validate_for(ClockPath::F1x0Rcu), Ok(()));
            assert_eq!(at_48.validate_for(ClockPath::F10xRcc), Ok(()));
        }

        #[test]
        fn reject_wait_states_above_wscnt_field() {
            // WSCNT is a 3-bit field (0..=7); 8 is out of range.
            assert_rejected(
                &ClockConfig {
                    sysclk_hz: 72_000_000,
                    wait_states: 8,
                    ..base()
                },
                ClockError::InvalidWaitStates,
            );
        }

        #[test]
        fn reject_sysclk_above_part_ceiling() {
            // The ceiling is PER FAMILY: F10x = 108 MHz, F1x0 = 72 MHz. Use the max-legal WS so only
            // the ceiling check fires. A clock just above each family's ceiling is rejected on that
            // family, and the family's own ceiling is accepted.
            let just_over = |hz: u32| ClockConfig {
                sysclk_hz: hz,
                wait_states: 2,
                ..base()
            };
            // F10x: 109 MHz rejected, 108 MHz accepted.
            assert_eq!(
                just_over(109_000_000).validate_for(ClockPath::F10xRcc),
                Err(ClockError::InvalidWaitStates)
            );
            assert_eq!(
                just_over(108_000_000).validate_for(ClockPath::F10xRcc),
                Ok(())
            );
            // F1x0: 73 MHz rejected, 72 MHz accepted.
            assert_eq!(
                just_over(73_000_000).validate_for(ClockPath::F1x0Rcu),
                Err(ClockError::InvalidWaitStates)
            );
            assert_eq!(
                just_over(72_000_000).validate_for(ClockPath::F1x0Rcu),
                Ok(())
            );
            // The per-family difference itself: 96 MHz is fine on F10x but over-ceiling on F1x0.
            assert_eq!(
                just_over(96_000_000).validate_for(ClockPath::F10xRcc),
                Ok(())
            );
            assert_eq!(
                just_over(96_000_000).validate_for(ClockPath::F1x0Rcu),
                Err(ClockError::InvalidWaitStates)
            );
        }

        /// F10x is zero-wait at the full 108 MHz (GD32F103xx datasheet): 0 WS is accepted at the top
        /// of its range, where the old shared ST-style ladder would have demanded 2.
        #[test]
        fn f10x_is_zero_wait_at_full_clock() {
            let top = ClockConfig {
                sysclk_hz: 108_000_000,
                wait_states: 0,
                ..base()
            };
            assert_eq!(top.validate_for(ClockPath::F10xRcc), Ok(()));
        }

        #[test]
        fn reject_missing_rcu_base_in_configure_tree() {
            // A Chip whose addr table has NO Rcu entry: configure_tree resolves the RCU base and
            // surfaces it as ClockError::MissingRcuBase. validate_for cannot see this (it has no
            // Chip), so this reject is configure_tree-only (the path that resolves the base).
            use crate::addr::{AddrTable, PeriphLabel};
            use crate::chip::Chip;
            use crate::descriptor::{AdcPath, GpioPath, IrqLayout, McuDescriptor, PageSize};

            let _g = seed(ClockSource::Irc8m);
            let addrs = AddrTable::new(); // empty: no Rcu base
            let _ = PeriphLabel::Rcu;
            let chip_no_rcu = Chip::from_descriptor(McuDescriptor {
                gpio: GpioPath::AhbCtlAfsel,
                clock: ClockPath::F1x0Rcu,
                adc: AdcPath::Single,
                irq: IrqLayout::F1x0Grouped,
                addrs,
                flash_page: PageSize::K1,
                adv_timers: 1,
                adc_count: 1,
            });
            assert_eq!(
                configure_tree(&chip_no_rcu, &base()),
                Err(ClockError::MissingRcuBase)
            );
        }
    }
}
