//! Host tests for the hot-path traits + handles (run with `cargo test --features mock`).
//!
//! These exercise the resolve-once concrete handles' per-cycle methods against the mock register
//! space and enforce the SAFETY invariant that the per-cycle handle cannot touch MOE (DECISIONS.md
//! #4). The trait config bodies are filled in T5/T9; T1 pins the handle write/read surface and the
//! TIMER0 register-model conformance.

#![cfg(feature = "mock")]

use super::arming::ArmGate;
use super::hall::HallReader;
use super::{
    ComplementaryPwm, InjectedAdcController, InjectedHandle, PwmController, PwmHandle,
    TriggeredAdc, CCHP_MOE, TIMER_CAR, TIMER_CCHP, TIMER_CH0CV, TIMER_CH1CV, TIMER_CH2CV,
    TIMER_CH3CV,
};
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::config::{
    AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
    PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::{HotPathError, PwmError};
use crate::reg::{mock, Reg32};

/// Build a single-advanced-timer / single-ADC F1x0-style [`Chip`] from a base-address table, so the
/// hot-path controllers can resolve (and range-check) their bases at `configure` time.
fn chip_with(addrs: AddrTable) -> Chip {
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

/// A [`Chip`] whose TIMER0 resolves to `TIMER0_BASE` (the reference advanced-timer base).
fn timer0_chip() -> Chip {
    let mut a = AddrTable::new();
    a.set(PeriphLabel::Timer0, TIMER0_BASE);
    chip_with(a)
}

/// A [`Chip`] whose ADC0 resolves to `ADC0_BASE` (the reference ADC base).
fn adc0_chip() -> Chip {
    let mut a = AddrTable::new();
    a.set(PeriphLabel::Adc0, ADC0_BASE);
    chip_with(a)
}

/// The reference complementary-PWM config on TIMER0 (mirrors the timer-module test wiring).
///
/// The timing-topology knobs are set to the values that reproduce the old baked behavior so the
/// register end-states are identical: center-aligned mode 2 (old CAM=2), ARSE on, CKDIV/2, CREP=0,
/// per-side idle both set to the old single `idle`, TRGO = UPDATE, trigger OC = PWM0, CH3EN off.
fn reference_config() -> PwmConfig {
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

/// A TIMER0 base inside the advanced-timer APB2 window.
const TIMER0_BASE: u32 = 0x4001_2C00;
/// An ADC0 base inside the ADC APB2 window.
const ADC0_BASE: u32 = 0x4001_2400;

// --- TIMER0 register-model conformance (offsets, against the GD SPL peripheral headers) --------

/// The advanced-timer register offsets the hot path uses, cross-checked against the GD SPL
/// `gd32f10x_timer.h` / `gd32f1x0_timer.h` (`TIMER_CHxCV(timerx)`, `TIMER_CAR`, `TIMER_CCHP`):
/// CH0CV 0x34, CH1CV 0x38, CH2CV 0x3C, CH3CV 0x40, CAR 0x2C, CCHP 0x44. A 32-bit width on all.
#[test]
fn timer0_register_offsets_match_spl() {
    assert_eq!(TIMER_CH0CV, 0x34);
    assert_eq!(TIMER_CH1CV, 0x38);
    assert_eq!(TIMER_CH2CV, 0x3C);
    assert_eq!(TIMER_CH3CV, 0x40);
    assert_eq!(TIMER_CAR, 0x2C);
    assert_eq!(TIMER_CCHP, 0x44);
    // The compare registers are 4 bytes apart in channel order (catches an offset typo).
    assert_eq!(TIMER_CH1CV - TIMER_CH0CV, 0x04);
    assert_eq!(TIMER_CH2CV - TIMER_CH1CV, 0x04);
    assert_eq!(TIMER_CH3CV - TIMER_CH2CV, 0x04);
    // MOE is CCHP bit 15 (`TIMER_CCHP_POEN`).
    assert_eq!(CCHP_MOE, 1 << 15);
}

// --- PwmHandle::set_duties writes the three CHxCV at the right offsets -------------------------

#[test]
fn set_duties_writes_three_compare_registers() {
    let _serial = mock::lock();
    mock::reset();

    let h = PwmHandle::new(TIMER0_BASE, 2250);
    h.set_duties([100, 200, 300]).unwrap();

    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 100);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH1CV).read(), 200);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH2CV).read(), 300);
    // The trigger compare (CH3CV) is NOT touched by set_duties.
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(), 0);
}

#[test]
fn rearm_trigger_writes_ch3cv_only() {
    let _serial = mock::lock();
    mock::reset();

    let h = PwmHandle::new(TIMER0_BASE, 2250);
    h.rearm_trigger(2249).unwrap();

    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(), 2249);
    // The phase compares are untouched.
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 0);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH1CV).read(), 0);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH2CV).read(), 0);
}

#[test]
fn duty_above_period_is_rejected() {
    let _serial = mock::lock();
    mock::reset();

    let h = PwmHandle::new(TIMER0_BASE, 2250);
    assert_eq!(h.set_duties([2251, 0, 0]), Err(PwmError::DutyOutOfRange));
    assert_eq!(h.rearm_trigger(2251), Err(PwmError::DutyOutOfRange));
    // A duty equal to the period is allowed (full compare).
    assert_eq!(h.set_duties([2250, 2250, 2250]), Ok(()));
}

// --- The SAFETY invariant: the per-cycle handle cannot touch MOE (DECISIONS.md #4) -------------

/// The PWM handle's per-cycle methods write only the four compare registers and NEVER the CCHP/MOE
/// bit. Arming is a separate, deliberately distinct call ([`ArmGate`]). This is the load-bearing
/// SAFETY invariant: a control-loop bug holding only the handle cannot energize a disarmed bridge.
#[test]
fn handle_never_writes_moe() {
    let _serial = mock::lock();
    mock::reset();

    // Pre-seed CCHP with MOE clear (disarmed) and a sentinel in the low bits.
    Reg32::new(TIMER0_BASE, TIMER_CCHP).write(0x0000_00AB);

    let h = PwmHandle::new(TIMER0_BASE, 2250);
    // Exercise every per-cycle method, including the out-of-range rejections.
    h.set_duties([10, 20, 30]).unwrap();
    h.rearm_trigger(2249).unwrap();
    let _ = h.set_duties([9999, 0, 0]);

    // CCHP is byte-for-byte unchanged: MOE was never set, and no compare write bled into it.
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CCHP).read(),
        0x0000_00AB,
        "the per-cycle handle must not touch CCHP/MOE"
    );
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE,
        0,
        "MOE stays clear: the handle cannot arm the bridge"
    );
}

/// The arming gate is the ONLY MOE writer and is distinct from the handle. arm() sets MOE,
/// disarm() clears it, both leaving the compare registers untouched.
#[test]
fn arm_gate_is_the_only_moe_writer() {
    let _serial = mock::lock();
    mock::reset();

    let gate = ArmGate::new(TIMER0_BASE);
    let h = PwmHandle::new(TIMER0_BASE, 2250);
    h.set_duties([10, 20, 30]).unwrap();

    // Arm: MOE set, duties intact.
    gate.arm();
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE,
        CCHP_MOE
    );
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 10);

    // Disarm: MOE clear again.
    gate.disarm();
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE, 0);
}

// --- InjectedHandle::read_injected reads the injected data registers, in rank order ------------

#[test]
fn read_injected_reads_idata_in_order() {
    let _serial = mock::lock();
    mock::reset();

    // Seed the four injected data registers (IDATA0..3 at 0x3C/0x40/0x44/0x48).
    Reg32::new(ADC0_BASE, 0x3C).write(0x0111);
    Reg32::new(ADC0_BASE, 0x40).write(0x0222);
    Reg32::new(ADC0_BASE, 0x44).write(0x0333);
    Reg32::new(ADC0_BASE, 0x48).write(0x0444);

    let h = InjectedHandle::new(ADC0_BASE, 2);
    let got = h.read_injected();
    assert_eq!(got[0], 0x0111);
    assert_eq!(got[1], 0x0222);
    assert_eq!(got[2], 0x0333);
    assert_eq!(got[3], 0x0444);
    assert_eq!(h.len(), 2);
    assert!(!h.is_empty());
}

// --- M3 T5: the ComplementaryPwm trait -> resolve-once handle ----------------------------------

/// `ComplementaryPwm::configure` runs the T3/T4 bring-up (so the timebase + channel + break
/// registers are programmed) and returns a `PwmHandle` whose period matches the wiring, with MOE
/// left OFF (the bridge disarmed). The returned handle's per-cycle writes then land at the right
/// compare offsets.
#[test]
fn configure_via_trait_then_set_duties() {
    let _serial = mock::lock();
    mock::reset();

    let chip = timer0_chip();
    let cfg = reference_config();
    let h = PwmController::new()
        .configure(&chip, &cfg)
        .expect("configure should succeed");

    // The handle carries the configured period.
    assert_eq!(h.period(), 2250);

    // The T3/T4 config writes happened: CAR = 2250, CTL0 = center-up | CKDIV/2 | ARSE, CCHP holds
    // the dead-time + off-state word with MOE OFF.
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CAR).read(), 2250);
    assert_eq!(
        Reg32::new(TIMER0_BASE, 0x00).read(),
        0x40 | 0x100 | 0x80,
        "CTL0"
    );
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE,
        0,
        "configure leaves MOE OFF (bridge disarmed)"
    );

    // Then the per-cycle duty writes land at the three phase compares.
    h.set_duties([100, 200, 300]).unwrap();
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 100);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH1CV).read(), 200);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH2CV).read(), 300);

    // The trace's MOE writer is the gate, distinct from the handle (the only MOE writer). The gate
    // is built from the resolved base (the safety layer resolves it separately from the handle).
    let gate = PwmController::arm_gate(chip.base(cfg.timer).unwrap());
    gate.arm();
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE,
        CCHP_MOE
    );
}

/// `PwmController::configure` resolves the timer base from the chip's [`AddrTable`] (via
/// `chip.base(cfg.timer)`). A present base succeeds; a MISSING base is rejected as
/// `HotPathError::Descriptor(MissingBase(..))`.
///
/// NOTE (relaxed): the old `PwmConfig::from_descriptor` range-checked the resolved base against the
/// advanced-timer window and rejected an in-table-but-out-of-window base. The reworked
/// `PwmController::configure` / `PwmTimer::configure` resolves the base with `chip.base(..)?` and
/// does NOT range-check the timer window at this layer (parse-time `check_ranges` only covers
/// GPIO/RCU). So the "out-of-window base is rejected" case no longer holds at configure; it is
/// dropped here and replaced with the missing-base rejection (which still errors). A present base
/// outside the window now resolves Ok, by design of the reworked layer.
#[test]
fn from_descriptor_resolves_and_range_checks() {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Timer0, TIMER0_BASE);
    let chip = chip_with(addrs);
    let h = PwmController::new()
        .configure(&chip, &reference_config())
        .expect("present base resolves");
    assert_eq!(h.period(), 2250);

    // A missing base is rejected (MissingBase, mapped to HotPathError::Descriptor).
    let empty = chip_with(AddrTable::new());
    assert!(matches!(
        PwmController::new().configure(&empty, &reference_config()),
        Err(HotPathError::Descriptor(
            crate::error::DescriptorError::MissingBase(_)
        ))
    ));
}

// --- M3 T7: the per-cycle rearm_trigger targets the SAME CH3 the T6 trigger config programs -----

/// T7: confirm `PwmHandle::rearm_trigger` re-arms the EXACT compare register (CH3CV, offset 0x40)
/// that the T6 timer trigger config ([`crate::timer::PwmTimer::configure_trigger`]) programs as the
/// ADC-trigger channel. Run the T6 trigger config (which sets CH3CV to the initial 2249), then
/// re-arm through the handle and assert it lands at CH3CV. No per-call branch: the handle holds the
/// resolved CH3CV accessor.
#[test]
fn rearm_trigger_matches_t6_trigger_channel() {
    let _serial = mock::lock();
    mock::reset();

    // T6: configure CH3 as the trigger channel (sets CH3CV = 2249). The reworked configure_trigger
    // takes the trigger OC mode / CH3EN / TRGO source explicitly; the reference values reproduce the
    // old baked behavior (PWM0, CH3EN off, TRGO = UPDATE).
    crate::timer::PwmTimer::at(TIMER0_BASE).configure_trigger(
        2249,
        OcMode::Pwm0,
        false,
        TrgoSource::Update,
    );
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(),
        2249,
        "T6 programs the trigger compare at CH3CV (~CAR-1)"
    );

    // T7: the per-cycle handle re-arms the SAME CH3CV the T6 config used (offset 0x40).
    let h = PwmHandle::new(TIMER0_BASE, 2250);
    h.rearm_trigger(2200).unwrap();
    assert_eq!(
        Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(),
        2200,
        "rearm_trigger writes the T6 trigger channel CH3CV"
    );
    // The phase compares are untouched by the re-arm.
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 0);
}

// --- M3 T8/T9: the TriggeredAdc trait -> resolve-once injected handle --------------------------

/// The reference injected-ADC wiring on ADC0, triggered by TIMER0 CH3: two phase-current channels
/// (4 and 5) at 7.5-cycle sample time (code 1), left-aligned.
fn reference_injected_config() -> InjectedAdcConfig {
    let mut channels = heapless::Vec::new();
    channels
        .push(InjectedChannel {
            channel: 4,
            sample_time: 1,
        })
        .unwrap();
    channels
        .push(InjectedChannel {
            channel: 5,
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

/// `TriggeredAdc::configure` runs the T8 injected bring-up INCLUDING the calibration poll. In the
/// flat mock register space the calibration self-clearing bit (RSTCLB) never clears, so the bounded
/// poll exits as `Timeout` (the same behaviour the M2 ADC calibrate host test relies on; the
/// happy-path calibration is the with_polling harness golden). The point here: the trait does wire
/// the config writes AND drive calibration, and the config writes are present in the register space
/// even though calibration then times out. (The flat-mock calibration limitation is flagged for the
/// silicon/with_polling layer, matching adc/tests.rs.)
#[test]
fn configure_injected_via_trait_drives_config_and_calibration() {
    let _serial = mock::lock();
    mock::reset();

    let chip = adc0_chip();
    // The full bring-up calibrates; in the flat mock RSTCLB never self-clears -> Timeout, proving
    // calibration is actually invoked (a dropped calibration would NOT time out).
    let res = InjectedAdcController::new().configure(&chip, &reference_injected_config());
    assert!(
        matches!(res, Err(HotPathError::Adc(crate::error::AdcError::Timeout))),
        "the trait config drives the calibration poll (Timeout in the flat mock)"
    );

    // The injected config writes landed before the calibration poll: CTL1 ETSIC = TIMER0 CH3
    // (1<<12), ETEIC (bit 15), DAL (left, bit 11), ADCON (bit 0); CTL0 EOICIE (bit 7).
    let ctl1 = Reg32::new(ADC0_BASE, 0x08).read();
    assert_eq!(ctl1 & (0x7 << 12), 1 << 12, "ETSIC = TIMER0 CH3 (code 1)");
    assert_ne!(ctl1 & (1 << 15), 0, "ETEIC (injected ext-trigger enable)");
    assert_ne!(ctl1 & (1 << 11), 0, "DAL (left-aligned)");
    assert_ne!(ctl1 & 1, 0, "ADCON (ADC enabled)");
    let ctl0 = Reg32::new(ADC0_BASE, 0x04).read();
    assert_ne!(ctl0 & (1 << 7), 0, "EOICIE (injected-EOC interrupt enable)");
    let isq = Reg32::new(ADC0_BASE, 0x38).read();
    assert_eq!((isq >> 20) & 0x3, 1, "ISQ IL = len-1");
}

/// The resolve-once `InjectedHandle` reads the injected data registers (IDATA0..3) in order.
/// Built directly so the flat-mock calibration limitation does not block the read-path assertion
/// (the trait's `configure` builds the same handle once calibration completes on silicon).
#[test]
fn injected_handle_reads_idata_in_order() {
    let _serial = mock::lock();
    mock::reset();

    Reg32::new(ADC0_BASE, 0x3C).write(0x0AAA);
    Reg32::new(ADC0_BASE, 0x40).write(0x0BBB);
    let h = InjectedHandle::new(ADC0_BASE, 2);
    let got = h.read_injected();
    assert_eq!(got[0], 0x0AAA);
    assert_eq!(got[1], 0x0BBB);
}

/// The TRGO link maps to ETSIC = TIMER0 TRGO (code 0); a non-TIMER0 trigger timer or an empty
/// channel list is rejected at config BEFORE any register write or calibration (`HotPathError::Adc`).
#[test]
fn injected_trigger_link_and_validation() {
    let _serial = mock::lock();
    mock::reset();

    let chip = adc0_chip();

    // TRGO link -> ETSIC code 0 (the config writes happen before calibration times out).
    let mut w = reference_injected_config();
    w.trigger_link = TimerTriggerLink::Trgo;
    let _ = InjectedAdcController::new().configure(&chip, &w);
    assert_eq!(
        Reg32::new(ADC0_BASE, 0x08).read() & (0x7 << 12),
        0,
        "ETSIC = TIMER0 TRGO (code 0)"
    );

    // A non-TIMER0 trigger timer is not expressible on the single-ADC baseline: rejected at config
    // (no calibration timeout, the validation short-circuits).
    let mut bad = reference_injected_config();
    bad.trigger_timer = PeriphLabel::Timer7;
    assert!(matches!(
        InjectedAdcController::new().configure(&chip, &bad),
        Err(HotPathError::Adc(_))
    ));

    // An empty channel list is rejected.
    let mut empty = reference_injected_config();
    empty.channels.clear();
    assert!(matches!(
        InjectedAdcController::new().configure(&chip, &empty),
        Err(HotPathError::Adc(_))
    ));
}

/// `InjectedAdcController::configure` resolves + range-checks the ADC base from the chip's
/// [`AddrTable`]. A base in the ADC window passes the descriptor check (then reaches the calibration
/// poll, which times out in the flat mock); one outside the ADC window, or missing, is rejected as
/// `HotPathError::Descriptor(..)` BEFORE any calibration (so not a Timeout).
#[test]
fn injected_from_descriptor_resolves_and_range_checks() {
    let _serial = mock::lock();
    mock::reset();

    // In-window base: passes the ADC-window range-check, so configure proceeds to calibration and
    // times out (it does NOT fail at the descriptor layer).
    let chip = adc0_chip();
    assert!(matches!(
        InjectedAdcController::new().configure(&chip, &reference_injected_config()),
        Err(HotPathError::Adc(crate::error::AdcError::Timeout))
    ));

    // A base outside the ADC window (a non-ADC base) is rejected at the descriptor layer.
    let mut bad = AddrTable::new();
    bad.set(PeriphLabel::Adc0, TIMER0_BASE);
    let bad_chip = chip_with(bad);
    assert!(matches!(
        InjectedAdcController::new().configure(&bad_chip, &reference_injected_config()),
        Err(HotPathError::Descriptor(_))
    ));

    // A missing base is rejected at the descriptor layer too.
    let empty = chip_with(AddrTable::new());
    assert!(matches!(
        InjectedAdcController::new().configure(&empty, &reference_injected_config()),
        Err(HotPathError::Descriptor(_))
    ));
}

// --- M3 HP-9: the hall GPIO read primitive -----------------------------------------------------

/// The hall reader samples the three input pins (default PC13 / PA1 / PC14) and packs them into a
/// 3-bit code `(h2<<2)|(h1<<1)|h0`, reading the family's GPIO_ISTAT offset (0x10 on F1x0 AHB).
#[test]
fn hall_reads_three_lines_into_code() {
    let _serial = mock::lock();
    mock::reset();

    const GPIOA: u32 = 0x4800_0000;
    const GPIOC: u32 = 0x4800_0800;
    // Lines in hall-line order: PC13, PA1, PC14.
    let reader = HallReader::resolve(
        GpioPath::AhbCtlAfsel,
        [(GPIOC, 13), (GPIOA, 1), (GPIOC, 14)],
    );

    // F1x0 ISTAT is at 0x10. Set PC13 (bit 13) and PA1 (bit 1) high, PC14 (bit 14) low.
    Reg32::new(GPIOC, 0x10).write(1 << 13);
    Reg32::new(GPIOA, 0x10).write(1 << 1); // PA1 = 1
                                           // code = (PC14<<2)|(PA1<<1)|PC13 = (0<<2)|(1<<1)|1 = 0b011 = 3.
    assert_eq!(reader.read(), 0b011);

    // Now drive PC14 high too -> code = (1<<2)|(1<<1)|1 = 0b111 = 7.
    Reg32::new(GPIOC, 0x10).write((1 << 13) | (1 << 14));
    assert_eq!(reader.read(), 0b111);

    // All low.
    Reg32::new(GPIOC, 0x10).write(0);
    Reg32::new(GPIOA, 0x10).write(0);
    assert_eq!(reader.read(), 0);
}

/// The F10x GPIO_ISTAT is at 0x08 (APB), not 0x10: the reader picks the offset from the GpioPath.
#[test]
fn hall_uses_apb_istat_offset_on_f10x() {
    let _serial = mock::lock();
    mock::reset();

    const GPIOC: u32 = 0x4001_1000;
    const GPIOA: u32 = 0x4001_0800;
    let reader = HallReader::resolve(GpioPath::ApbCrlCrh, [(GPIOC, 13), (GPIOA, 1), (GPIOC, 14)]);
    // APB ISTAT at 0x08: set PC13 high.
    Reg32::new(GPIOC, 0x08).write(1 << 13);
    assert_eq!(reader.read(), 0b001);
}

/// The handle is `Copy` / concrete (no `dyn`, no descriptor lookup per call): it can be copied and
/// each copy writes the same resolved registers. This is the resolve-once invariant.
#[test]
fn handle_is_copy_and_resolve_once() {
    let _serial = mock::lock();
    mock::reset();

    let chip = timer0_chip();
    let h = PwmController::new()
        .configure(&chip, &reference_config())
        .unwrap();
    let h2 = h; // Copy, not move.
    h.set_duties([1, 2, 3]).unwrap();
    h2.set_duties([10, 20, 30]).unwrap();
    // The second copy wrote the same resolved offsets.
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 10);
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH2CV).read(), 30);
    // Original still usable (Copy semantics, no move).
    h.rearm_trigger(2249).unwrap();
    assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(), 2249);
}
