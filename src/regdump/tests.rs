//! Host tests for the read-only register dump (G8). Run with `cargo test --features mock`.
//!
//! These configure the real bring-up (the advanced-timer complementary-PWM + the timer-triggered
//! injected ADC) against the mock register space, then [`RegDumpConfig::dump`] it and assert the
//! snapshot reflects the configured state, that the dump itself writes nothing, and that the captured
//! MOE bit is CLEAR after a config-only bring-up (the SAFETY invariant the verification gate checks).

#![cfg(feature = "mock")]

use super::RegDumpConfig;
use crate::adc::{InjectedAdcController, TriggeredAdc};
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::config::{
    AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
    PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::reg::{mock, Reg32};
use crate::timer::arming::ArmGate;
use crate::timer::{ComplementaryPwm, PwmController};
use heapless::Vec;

/// A TIMER0 base inside the advanced-timer APB2 window.
const TIMER0_BASE: u32 = 0x4001_2C00;
/// An ADC0 base inside the ADC APB2 window.
const ADC0_BASE: u32 = 0x4001_2400;

/// CCHP offset + the MOE bit, re-stated here so the test pins them independently of the dump module.
const TIMER_CCHP: u32 = 0x44;
const CCHP_MOE: u32 = 1 << 15;

/// A single-advanced-timer / single-ADC F1x0-style chip resolving TIMER0 + ADC0.
fn chip() -> Chip {
    let mut a = AddrTable::new();
    a.set(PeriphLabel::Timer0, TIMER0_BASE);
    a.set(PeriphLabel::Adc0, ADC0_BASE);
    Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel,
        clock: ClockPath::F1x0Rcu,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs: a,
        flash_page: PageSize::K1,
        adv_timers: 1,
        adc_count: 1,
    })
}

/// The reference complementary-PWM config (mirrors the per-cycle-path test wiring).
fn pwm_config() -> PwmConfig {
    let ch = |high: u8, low: u8| PwmChannelConfig {
        high,
        low,
        polarity: true,
        idle_high: true,
        idle_high_n: true,
    };
    PwmConfig {
        timer: PeriphLabel::Timer0,
        channels: [ch(0x08, 0x1D), ch(0x09, 0x1E), ch(0x0A, 0x1F)],
        period: 2250,
        prescaler: 0,
        dead_time: 0x1C,
        brk: BreakConfig {
            enabled: false,
            level: false,
        },
        trigger_compare: 2249,
        align: PwmAlign::Center2,
        arse: true,
        trigger_oc_mode: OcMode::Pwm0,
        trigger_ch_enable: false,
        crep: 0,
        ckdiv: ClockDiv::Div2,
        trgo_src: TrgoSource::Update,
    }
}

/// The reference injected-ADC config (two channels, CH3-triggered, left-aligned).
fn injected_config() -> InjectedAdcConfig {
    let mut channels: Vec<InjectedChannel, { crate::descriptor::MAX_INJECTED_CHANNELS }> =
        Vec::new();
    channels
        .push(InjectedChannel {
            channel: 0,
            sample_time: 1,
        })
        .unwrap();
    channels
        .push(InjectedChannel {
            channel: 1,
            sample_time: 1,
        })
        .unwrap();
    InjectedAdcConfig {
        adc: PeriphLabel::Adc0,
        channels,
        left_aligned: true,
        trigger_timer: PeriphLabel::Timer0,
        trigger_link: TimerTriggerLink::Ch3,
        clock_div: AdcClockDiv::Div6,
    }
}

#[test]
fn dump_reflects_the_configured_timer_period_and_prescaler() {
    let _serial = mock::lock();
    mock::reset();
    let chip = chip();

    PwmController::new()
        .configure(&chip, &pwm_config())
        .unwrap();

    let snap = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    // CAR is the configured period; PSC the configured prescaler; CREP the rep counter.
    assert_eq!(snap.timer.car, 2250);
    assert_eq!(snap.timer.psc, 0);
    assert_eq!(snap.timer.crep, 0);
    // CCHP carries the dead-time field the bring-up wrote (DTCFG = 0x1C).
    assert_eq!(snap.timer.cchp & 0xFF, 0x1C);
}

#[test]
fn dump_shows_moe_clear_after_config_only_bring_up() {
    let _serial = mock::lock();
    mock::reset();
    let chip = chip();

    // Config-only bring-up: PwmController::configure leaves MOE OFF (arming is ArmGate's).
    PwmController::new()
        .configure(&chip, &pwm_config())
        .unwrap();

    let snap = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    // The SAFETY invariant the verification gate asserts: a configured-but-disarmed bridge reads MOE
    // clear. `moe()` is the typed accessor; the raw CCHP bit agrees.
    assert!(
        !snap.timer.moe(),
        "config-only must leave the bridge disarmed"
    );
    assert_eq!(snap.timer.cchp & CCHP_MOE, 0);
}

#[test]
fn dump_sees_moe_when_armed() {
    let _serial = mock::lock();
    mock::reset();
    let chip = chip();

    PwmController::new()
        .configure(&chip, &pwm_config())
        .unwrap();
    // Arm through the sole MOE writer, then dump: the capture must now show MOE set. This proves the
    // dump reads the live CCHP, so the gate can DETECT an unexpectedly-armed bridge.
    let base = chip.base(PeriphLabel::Timer0).unwrap();
    ArmGate::new(base).arm();

    let snap = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    assert!(snap.timer.moe(), "the dump must observe the armed MOE bit");
}

#[test]
fn dump_reflects_injected_adc_alignment_and_trigger() {
    let _serial = mock::lock();
    mock::reset();
    let chip = chip();

    // The injected bring-up writes the config registers BEFORE the calibration poll; in the flat
    // mock the calibration self-clearing bit never clears, so `configure` exits `Adc(Timeout)`
    // (the same flat-mock limitation the adc tests document). The config writes are present, so
    // the dump still reflects them; ignore the calibration timeout here.
    let _ = InjectedAdcController::new().configure(&chip, &injected_config());

    let snap = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    // Left-aligned data: CTL1 DAL (bit 11) set.
    assert_eq!(snap.adc_injected.ctl1 & (1 << 11), 1 << 11);
    // The injected end-of-conversion interrupt enable (EOICIE, CTL0 bit 7) is on (it is what makes
    // the control-loop ISR fire at the PWM rate).
    assert_eq!(snap.adc_injected.ctl0 & (1 << 7), 1 << 7);
    // ISQ injected length field (IL, bits [21:20]) = channels - 1 = 1 for two channels.
    assert_eq!((snap.adc_injected.isq >> 20) & 0x3, 1);
}

#[test]
fn dump_writes_nothing() {
    let _serial = mock::lock();
    mock::reset();
    let chip = chip();

    PwmController::new()
        .configure(&chip, &pwm_config())
        .unwrap();
    // Injected `configure` exits `Adc(Timeout)` in the flat mock (calibration bit never clears); the
    // config writes still land. Ignore the result; the dump reads whatever is in the register space.
    let _ = InjectedAdcController::new().configure(&chip, &injected_config());

    // Capture the live CCHP, then dump twice; a read-only dump must not perturb any register.
    let cchp_before = Reg32::new(TIMER0_BASE, TIMER_CCHP).read();
    let first = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    let second = RegDumpConfig::dump(TIMER0_BASE, ADC0_BASE);
    let cchp_after = Reg32::new(TIMER0_BASE, TIMER_CCHP).read();

    assert_eq!(first, second, "two dumps of the same state are identical");
    assert_eq!(cchp_before, cchp_after, "the dump must not write CCHP");
}
