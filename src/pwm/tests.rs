//! Host tests for the general-purpose-timer single-channel PWM (G3). Run with `cargo test
//! --features mock`. These assert the END STATE of each general-timer register after
//! [`PwmOut::new`] against the basic-PWM subset the GD SPL `timer_init` +
//! `timer_channel_output_*` recipe reaches, that NO bridge field (CCHP / complementary / MOE) is
//! ever written, and that the TIMER0-refusal guard rejects the advanced timer.

#![cfg(feature = "mock")]

use super::*;
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::reg::{mock, Reg32};
use embedded_hal::pwm::SetDutyCycle;

/// TIMER1 base (the general level-0 timer, the bottom of APB1; same on both families).
const TIMER1_BASE: u32 = 0x4000_0000;

/// A chip whose addrs resolves `Timer1` to [`TIMER1_BASE`] and `Timer0` to the advanced window.
fn chip() -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Timer1, TIMER1_BASE);
    addrs.set(PeriphLabel::Timer0, 0x4001_2C00);
    Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel,
        clock: ClockPath::F1x0Rcu,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        adv_timers: 1,
        adc_count: 1,
    })
}

fn reg(off: u32) -> u32 {
    Reg32::new(TIMER1_BASE, off).read()
}

#[test]
fn general_timer_offsets_match_spl() {
    // The basic-PWM subset offsets equal the advanced timer's (gd32*_timer.h / GD32F1x0 UM 15.2).
    assert_eq!(CTL0, 0x00);
    assert_eq!(CHCTL0, 0x18);
    assert_eq!(CHCTL2, 0x20);
    assert_eq!(PSC, 0x28);
    assert_eq!(CAR, 0x2C);
    assert_eq!(CH0CV, 0x34);
    // CH1CV = CH0CV + 4*1.
    assert_eq!(CH0CV + 4 * CHANNEL, 0x38);
}

#[test]
fn new_brings_up_ch1_pwm_and_starts_counter() {
    let _g = mock::lock();
    mock::reset();

    // 8 MHz reset clock, ~1 kHz target: CAR = round(8_000_000 / 1000) - 1 = 7999.
    let pwm = PwmOut::new(&chip(), PeriphLabel::Timer1, 1_000, 8_000_000).unwrap();
    assert_eq!(pwm.period(), 7999);
    assert_eq!(pwm.base(), TIMER1_BASE);

    // Time base: PSC = 0, CAR = 7999.
    assert_eq!(reg(PSC), 0);
    assert_eq!(reg(CAR), 7999);
    // CTL0: edge-aligned up-count (DIR + CAM clear), ARSE (bit 7), CEN (bit 0) set (counter running).
    assert_eq!(reg(CTL0) & (1 << 4), 0, "DIR clear (up-count)");
    assert_eq!(reg(CTL0) & (0b11 << 5), 0, "CAM clear (edge-aligned)");
    assert_eq!(reg(CTL0) & (1 << 7), 1 << 7, "ARSE set");
    assert_eq!(reg(CTL0) & (1 << 0), 1 << 0, "CEN set (counter started)");

    // CHCTL0 CH1 half: PWM0 (COMCTL 0b110 << 12 = 0x6000), COMSEN (bit 11 = 0x800), MS = 0.
    assert_eq!(reg(CHCTL0), 0x6000 | 0x800, "CH1 PWM0 + shadow, output mode");

    // CHCTL2: only CH1EN (bit 4) set; CH1P clear; no complementary bits (CH1NEN/CH1NP = 0).
    assert_eq!(reg(CHCTL2), 1 << 4, "CH1 enabled, active-high, no complementary");

    // Zero initial duty (CH1CV at 0x38).
    assert_eq!(reg(0x38), 0, "CH1CV initial duty 0");
}

#[test]
fn no_bridge_fields_are_ever_written() {
    let _g = mock::lock();
    mock::reset();
    let _ = PwmOut::new(&chip(), PeriphLabel::Timer1, 1_000, 8_000_000).unwrap();

    // CCHP (0x44: dead-time / break / MOE-POEN) must be untouched: this is a general timer, the
    // word does not exist and the cold path must never write it.
    assert_eq!(reg(0x44), 0, "CCHP (dead-time/break/MOE) never written");
    // CTL1 (0x04: the advanced timer's idle-state / MMC) untouched.
    assert_eq!(reg(0x04), 0, "CTL1 never written");
    // CREP (0x30: repetition counter, advanced-only) untouched.
    assert_eq!(reg(0x30), 0, "CREP never written");
    // The complementary / other-channel CHCTL2 bits stay clear (only CH1EN at bit 4).
    assert_eq!(reg(CHCTL2) & !(1u32 << 4), 0, "no other CHCTL2 bits set");
}

#[test]
fn refuses_advanced_timer_label() {
    let _g = mock::lock();
    mock::reset();
    // TIMER0 (the advanced bridge) must be REFUSED, and nothing must be written to its base.
    let r = PwmOut::new(&chip(), PeriphLabel::Timer0, 1_000, 8_000_000);
    assert_eq!(r.err(), Some(PwmError::BadTimerBase));
    // Neither the advanced base nor the general base was touched.
    assert_eq!(Reg32::new(0x4001_2C00, CTL0).read(), 0, "TIMER0 untouched");
    assert_eq!(reg(CTL0), 0, "TIMER1 untouched on a rejected label");
}

#[test]
fn refuses_non_timer_label() {
    let _g = mock::lock();
    mock::reset();
    let r = PwmOut::new(&chip(), PeriphLabel::Usart1, 1_000, 8_000_000);
    assert_eq!(r.err(), Some(PwmError::BadTimerBase));
}

#[test]
fn degenerate_frequency_is_rejected() {
    let _g = mock::lock();
    mock::reset();
    // Zero frequency and a frequency higher than half the timer clock (< 2 counts) are rejected.
    assert_eq!(
        PwmOut::new(&chip(), PeriphLabel::Timer1, 0, 8_000_000).err(),
        Some(PwmError::DutyOutOfRange)
    );
    assert_eq!(
        PwmOut::new(&chip(), PeriphLabel::Timer1, 8_000_000, 8_000_000).err(),
        Some(PwmError::DutyOutOfRange)
    );
}

#[test]
fn period_clamps_to_16_bit_counter() {
    let _g = mock::lock();
    mock::reset();
    // A very low frequency would need a CAR beyond 16 bits; it clamps to 0xFFFF.
    let pwm = PwmOut::new(&chip(), PeriphLabel::Timer1, 1, 8_000_000).unwrap();
    assert_eq!(pwm.period(), u16::MAX);
    assert_eq!(reg(CAR), u32::from(u16::MAX));
}

#[test]
fn set_duty_cycle_writes_ch1cv_as_32bit() {
    let _g = mock::lock();
    mock::reset();
    let mut pwm = PwmOut::new(&chip(), PeriphLabel::Timer1, 1_000, 8_000_000).unwrap();

    // Half duty.
    pwm.set_duty_cycle(4000).unwrap();
    assert_eq!(reg(0x38), 4000, "CH1CV = duty");
    // Width-strict: a 32-bit store (catches a 16-vs-32 slip).
    pwm.set_duty_cycle(0x08CA).unwrap();
    assert_eq!(mock::peek_byte(TIMER1_BASE + 0x38), 0xCA);
    assert_eq!(mock::peek_byte(TIMER1_BASE + 0x39), 0x08);
    assert_eq!(mock::peek_byte(TIMER1_BASE + 0x3A), 0x00);
    assert_eq!(mock::peek_byte(TIMER1_BASE + 0x3B), 0x00);
}

#[test]
fn set_duty_cycle_clamps_to_period() {
    let _g = mock::lock();
    mock::reset();
    let mut pwm = PwmOut::new(&chip(), PeriphLabel::Timer1, 1_000, 8_000_000).unwrap();
    let period = pwm.period();
    // A duty above the period clamps to the period (full-on, not never-matching).
    pwm.set_duty_cycle(u16::MAX).unwrap();
    assert_eq!(reg(0x38), u32::from(period));
}

#[test]
fn max_duty_cycle_is_the_period() {
    let _g = mock::lock();
    mock::reset();
    let pwm = PwmOut::new(&chip(), PeriphLabel::Timer1, 1_000, 8_000_000).unwrap();
    assert_eq!(pwm.max_duty_cycle(), 7999);
    assert_eq!(pwm.max_duty_cycle(), pwm.period());
}
