//! Host tests for the advanced-timer complementary-PWM bring-up (run with `cargo test --features
//! mock`). These assert the END STATE of each TIMER0 register after [`PwmTimer::configure`] against
//! the values the GD SPL `timer_init` / `timer_channel_output_config` /
//! `timer_channel_output_mode_config` / `timer_break_config` recipe reaches, and that MOE (CCHP
//! POEN) is never set by the config path. The byte-for-byte golden-vs-SPL agreement is the harness'
//! job; these pin the register math and the MOE-OFF invariant at the unit level.

#![cfg(feature = "mock")]

use super::*;
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::config::{
    BreakConfig, ClockDiv, OcMode, PwmAlign, PwmChannelConfig, PwmConfig, TrgoSource,
};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::reg::{mock, Reg32};

/// A TIMER0 base inside the advanced-timer APB2 window.
const TIMER0_BASE: u32 = 0x4001_2C00;

/// MOE (CCHP POEN) bit, mirrored from the hot-path arming layer (the value under test must stay 0).
const CCHP_POEN: u32 = 1 << 15;

/// A chip whose addrs resolves `Timer0` to [`TIMER0_BASE`] (the advanced-timer window). The
/// register model is family-independent for the advanced timer, so the F1x0 path is used.
fn chip() -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Timer0, TIMER0_BASE);
    Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel,
        clock: ClockPath::F1x0Rcu,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

/// The reference complementary-PWM config: PSC 0, CAR 2250, three pairs PA8/9/10 high + PB13/14/15
/// low (AF2), inverted low-side polarity, safe (HIGH) per-side idle, dead-time 0x1C, break DISABLED,
/// trigger compare 2249 (~CAR-1), center-aligned mode 2, ARSE on, CKDIV /2, CREP 0. Pins are
/// `(port<<4)|pin`: PA8 = 0x08, PB13 = 0x1D, etc.
fn reference_wiring() -> PwmConfig {
    let ch = |high: u8, low: u8| PwmChannelConfig {
        high,
        low,
        polarity: true,    // invert the low side
        idle_high: true,   // safe idle HIGH (main)
        idle_high_n: true, // safe idle HIGH (complementary)
    };
    PwmConfig {
        timer: PeriphLabel::Timer0,
        channels: [
            ch(0x08, 0x1D), // PA8 / PB13
            ch(0x09, 0x1E), // PA9 / PB14
            ch(0x0A, 0x1F), // PA10 / PB15
        ],
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

fn reg(off: u32) -> u32 {
    Reg32::new(TIMER0_BASE, off).read()
}

#[test]
fn timer_register_offsets_match_spl() {
    // Cross-check against gd32*_timer.h REG32 offsets.
    assert_eq!(CTL0, 0x00);
    assert_eq!(CTL1, 0x04);
    assert_eq!(SWEVG, 0x14);
    assert_eq!(CHCTL0, 0x18);
    assert_eq!(CHCTL1, 0x1C);
    assert_eq!(CHCTL2, 0x20);
    assert_eq!(PSC, 0x28);
    assert_eq!(CAR, 0x2C);
    assert_eq!(CREP, 0x30);
    assert_eq!(CH0CV, 0x34);
    assert_eq!(CCHP, 0x44);
}

#[test]
fn timebase_end_state_matches_spl() {
    let _serial = mock::lock();
    mock::reset();

    let _ = PwmTimer::configure(&chip(), &reference_wiring()).unwrap();

    // PSC = 0; CAR = 2250 (0x8CA); CREP = 0.
    assert_eq!(reg(PSC), 0);
    assert_eq!(reg(CAR), 2250);
    assert_eq!(reg(CREP), 0);
    // SWEVG UPG generated (update event to latch shadows).
    assert_eq!(reg(SWEVG) & 0x1, 0x1);
    // CTL0: center-aligned-up (CAM = 2 -> bits[6:5] = 0x40), CKDIV /2 (bits[9:8] = 0x100),
    // ARSE (bit 7 = 0x80). DIR clear (center-aligned). CEN clear (timer not started here).
    assert_eq!(
        reg(CTL0),
        0x40 | 0x100 | 0x80,
        "CTL0 = CAM_up | CKDIV/2 | ARSE"
    );
    assert_eq!(
        reg(CTL0) & 0x1,
        0,
        "CEN stays clear: bring-up does not start the counter"
    );
}

#[test]
fn channel_output_end_state_matches_spl() {
    let _serial = mock::lock();
    mock::reset();

    let _ = PwmTimer::configure(&chip(), &reference_wiring()).unwrap();

    // CHCTL0 holds CH0 (low byte) + CH1 (high byte): each 0x68 = COMSEN(3) | PWM0 COMCTL(0x60),
    // MS = 0. CHCTL1 holds CH2 in its low byte (CH3 untouched here, that is the T6 trigger).
    assert_eq!(reg(CHCTL0), 0x6868, "CH0 + CH1 PWM0 + shadow");
    assert_eq!(reg(CHCTL1), 0x0068, "CH2 PWM0 + shadow; CH3 untouched");

    // CHCTL2: each channel field (4 bits, shift 4*n) = EN(0) | CCXN_EN(2) | CCXN_P_LOW(3) = 0xD,
    // for the inverted-low-side reference. CH0=0xD, CH1=0xD<<4, CH2=0xD<<8 -> 0xDDD.
    assert_eq!(
        reg(CHCTL2),
        0xDDD,
        "three pairs: main+comp enabled, comp polarity low"
    );

    // CTL1: idle state HIGH on both outputs of each pair: ISO0/0N (8,9), ISO1/1N (10,11),
    // ISO2/2N (12,13) -> 0x3F00. ISO3 (bit 14, the CH3 trigger idle) untouched.
    assert_eq!(reg(CTL1), 0x3F00, "safe HIGH idle on all three pairs");

    // The three channel compares start at zero (the control loop writes real duties via the handle);
    // CH3CV (the trigger compare, T6) is untouched.
    assert_eq!(reg(0x34), 0, "CH0CV initial duty 0");
    assert_eq!(reg(0x38), 0, "CH1CV initial duty 0");
    assert_eq!(reg(0x3C), 0, "CH2CV initial duty 0");
    assert_eq!(reg(0x40), 0, "CH3CV (trigger) untouched in T3/T4");
}

#[test]
fn break_word_end_state_and_moe_stays_off() {
    let _serial = mock::lock();
    mock::reset();

    let _ = PwmTimer::configure(&chip(), &reference_wiring()).unwrap();

    // CCHP = dead-time 0x1C | ROS (bit 11 = 0x800) | IOS (bit 10 = 0x400). Break DISABLED, PROT 0,
    // OAEN 0, POEN (MOE) 0.
    assert_eq!(
        reg(CCHP),
        0x1C | 0x800 | 0x400,
        "DTCFG | ROS | IOS, break off"
    );
    assert_eq!(
        reg(CCHP) & CCHP_POEN,
        0,
        "MOE must be OFF after bring-up (disarmed)"
    );
}

#[test]
fn break_enabled_sets_brken_and_polarity() {
    let _serial = mock::lock();
    mock::reset();

    let mut w = reference_wiring();
    w.brk = BreakConfig {
        enabled: true,
        level: true, // active-high break
    };
    let _ = PwmTimer::configure(&chip(), &w).unwrap();

    // BRKEN (bit 12 = 0x1000) | BRKP_HIGH (bit 13 = 0x2000) added to the off-state + dead-time word.
    assert_eq!(reg(CCHP), 0x1C | 0x800 | 0x400 | 0x1000 | 0x2000);
    // Still no MOE: enabling break is a hardware kill, not arming.
    assert_eq!(reg(CCHP) & CCHP_POEN, 0);
}

#[test]
fn non_inverted_and_no_idle_clear_the_bits() {
    let _serial = mock::lock();
    mock::reset();

    let mut w = reference_wiring();
    for ch in w.channels.iter_mut() {
        ch.polarity = false;
        ch.idle_high = false;
        ch.idle_high_n = false;
    }
    let _ = PwmTimer::configure(&chip(), &w).unwrap();

    // No CCXN_P_LOW: each channel field = EN(0) | CCXN_EN(2) = 0b0101 = 0x5 -> 0x555.
    assert_eq!(
        reg(CHCTL2),
        0x555,
        "main+comp enabled, no polarity inversion"
    );
    // No idle bits set.
    assert_eq!(reg(CTL1), 0x0000, "no idle-state bits");
}

#[test]
fn period_and_prescaler_sweep_land_in_car_and_psc() {
    let _serial = mock::lock();
    for (psc, car) in [(0u16, 1u16), (7, 1024), (71, 999), (0xFFFF, 0xFFFF), (0, 0)] {
        mock::reset();
        let mut w = reference_wiring();
        w.prescaler = psc;
        w.period = car;
        let _ = PwmTimer::configure(&chip(), &w).unwrap();
        assert_eq!(reg(PSC), u32::from(psc), "PSC = prescaler");
        assert_eq!(reg(CAR), u32::from(car), "CAR = period");
        // CTL0 mode bits are independent of period/prescaler.
        assert_eq!(reg(CTL0), 0x40 | 0x100 | 0x80);
    }
    drop(_serial);
}

/// The SPL end-state oracle for the CCHP word given a dead-time code and the reference off-states
/// (ROS + IOS, break off, PROT off, MOE off). This is exactly what `timer_break_config` assembles.
fn spl_cchp_oracle(dead_time: u8) -> u32 {
    (u32::from(dead_time) & 0xFF) | (1 << 11) | (1 << 10)
}

/// The SPL end-state oracle for one channel's CHCTL2 4-bit field given the inverted-low-side
/// polarity flag: EN | NEN | (NP_LOW if inverted).
fn spl_chctl2_field_oracle(invert_low: bool) -> u32 {
    let mut v = (1 << 0) | (1 << 2);
    if invert_low {
        v |= 1 << 3;
    }
    v
}

proptest::proptest! {
    /// Period / prescaler / dead-time / polarity / idle sweep diffed against the SPL formula
    /// oracle. PSC = prescaler and CAR = period are identity assignments in the SPL; the dead-time
    /// lands in CCHP DTCFG[7:0]; the inverted-low-side polarity flips CHCTL2 NP_LOW; the idle flag
    /// sets the CTL1 ISO pair. A wrong dead-time encoding (the field TESTING flags as
    /// combination-sensitive) or a misplaced bit would diverge here.
    #[test]
    fn pwm_config_matches_spl_oracle(
        period in 1u16..=u16::MAX,
        prescaler in 0u16..=u16::MAX,
        dead_time in 0u8..=u8::MAX,
        invert_low in proptest::bool::ANY,
        idle in proptest::bool::ANY,
    ) {
        let _serial = mock::lock();
        mock::reset();

        let ch = PwmChannelConfig {
            high: 0x08,
            low: 0x1D,
            polarity: invert_low,
            idle_high: idle,
            idle_high_n: idle,
        };
        let w = PwmConfig {
            timer: PeriphLabel::Timer0,
            channels: [ch, ch, ch],
            period,
            prescaler,
            dead_time,
            brk: BreakConfig { enabled: false, level: false },
            trigger_compare: 0,
            align: PwmAlign::Center2,
            arse: true,
            trigger_oc_mode: OcMode::Pwm0,
            trigger_ch_enable: false,
            crep: 0,
            ckdiv: ClockDiv::Div2,
            trgo_src: TrgoSource::Update,
        };
        let _ = PwmTimer::configure(&chip(), &w).unwrap();

        // PSC / CAR are identity.
        proptest::prop_assert_eq!(reg(PSC), u32::from(prescaler));
        proptest::prop_assert_eq!(reg(CAR), u32::from(period));
        // CCHP dead-time + off-states, MOE OFF.
        proptest::prop_assert_eq!(reg(CCHP), spl_cchp_oracle(dead_time));
        proptest::prop_assert_eq!(reg(CCHP) & CCHP_POEN, 0);
        // CHCTL2: three identical channel fields.
        let f = spl_chctl2_field_oracle(invert_low);
        proptest::prop_assert_eq!(reg(CHCTL2), f | (f << 4) | (f << 8));
        // CTL1 idle bits: each pair (8+2*n .. ) set when idle is HIGH.
        let iso = if idle { 0x3F00u32 } else { 0 };
        proptest::prop_assert_eq!(reg(CTL1), iso);
        // CTL0 mode bits independent of the swept values.
        proptest::prop_assert_eq!(reg(CTL0), 0x40 | 0x100 | 0x80);
    }
}

#[test]
fn curated_min_max_dead_time() {
    // Boundary dead-time codes: 0 (no dead-time) and 0xFF (max). The DTCFG field is CCHP[7:0], so
    // a wider slip would corrupt the off-state/break bits above it.
    let _serial = mock::lock();
    for dt in [0x00u8, 0x01, 0x7F, 0x80, 0xFF] {
        mock::reset();
        let mut w = reference_wiring();
        w.dead_time = dt;
        let _ = PwmTimer::configure(&chip(), &w).unwrap();
        assert_eq!(reg(CCHP), spl_cchp_oracle(dt), "dead-time {dt:#x}");
        assert_eq!(reg(CCHP) & CCHP_POEN, 0, "MOE off for dead-time {dt:#x}");
    }
    drop(_serial);
}

#[test]
fn chcv_is_a_32bit_write_not_two_16bit() {
    // Width-strict: a CHnCV compare write is a single 32-bit store (catches a 16-vs-32 slip). The
    // bring-up writes 0 initially; a non-zero duty through the handle would land all four bytes.
    let _serial = mock::lock();
    mock::reset();

    // Write a sentinel duty directly via the same offset the bring-up uses, then read it back as a
    // 32-bit value to confirm width.
    Reg32::new(TIMER0_BASE, CH0CV).write(0x0000_08CA);
    assert_eq!(reg(0x34), 0x08CA);
    assert_eq!(mock::peek_byte(TIMER0_BASE + 0x34), 0xCA);
    assert_eq!(mock::peek_byte(TIMER0_BASE + 0x35), 0x08);
    assert_eq!(mock::peek_byte(TIMER0_BASE + 0x36), 0x00);
    assert_eq!(mock::peek_byte(TIMER0_BASE + 0x37), 0x00);
}

/// Host tests for the per-cycle PWM path: the resolve-once
/// [`PwmHandle`] + the [`arming::ArmGate`], and the [`ComplementaryPwm`] trait config. These pin the
/// handle write/read surface, the TIMER0 register-model conformance, and the load-bearing SAFETY
/// invariant that the per-cycle handle cannot touch MOE (DECISIONS.md #4).
mod per_cycle_path {
    use crate::addr::{AddrTable, PeriphLabel};
    use crate::chip::Chip;
    use crate::config::{
        BreakConfig, ClockDiv, OcMode, PwmAlign, PwmChannelConfig, PwmConfig, TrgoSource,
    };
    use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
    use crate::error::{BringUpError, PwmError};
    use crate::reg::{mock, Reg32};
    use crate::timer::arming::ArmGate;
    use crate::timer::{
        ComplementaryPwm, PwmController, PwmHandle, CCHP_MOE, TIMER_CAR, TIMER_CCHP, TIMER_CH0CV,
        TIMER_CH1CV, TIMER_CH2CV, TIMER_CH3CV,
    };

    /// A TIMER0 base inside the advanced-timer APB2 window.
    const TIMER0_BASE: u32 = 0x4001_2C00;

    /// Build a single-advanced-timer / single-ADC F1x0-style [`Chip`] from a base-address table, so
    /// the controller can resolve (and range-check) its base at `configure` time.
    fn chip_with(addrs: AddrTable) -> Chip {
        Chip::from_descriptor(McuDescriptor {
            gpio: GpioPath::AhbCtlAfsel,
            clock: ClockPath::F1x0Rcu,
            adc: AdcPath::Single,
            irq: IrqLayout::F1x0Grouped,
            addrs,
            flash_page: PageSize::K1,
            flash_kib: 64,
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

    /// The reference complementary-PWM config on TIMER0 (mirrors the timer-module test wiring).
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

    // --- TIMER0 register-model conformance (offsets, against the GD SPL peripheral headers) ------

    /// The advanced-timer register offsets the per-cycle path uses, cross-checked against the GD SPL
    /// `gd32f10x_timer.h` / `gd32f1x0_timer.h`: CH0CV 0x34, CH1CV 0x38, CH2CV 0x3C, CH3CV 0x40,
    /// CAR 0x2C, CCHP 0x44. A 32-bit width on all.
    #[test]
    fn timer0_register_offsets_match_spl() {
        assert_eq!(TIMER_CH0CV, 0x34);
        assert_eq!(TIMER_CH1CV, 0x38);
        assert_eq!(TIMER_CH2CV, 0x3C);
        assert_eq!(TIMER_CH3CV, 0x40);
        assert_eq!(TIMER_CAR, 0x2C);
        assert_eq!(TIMER_CCHP, 0x44);
        assert_eq!(TIMER_CH1CV - TIMER_CH0CV, 0x04);
        assert_eq!(TIMER_CH2CV - TIMER_CH1CV, 0x04);
        assert_eq!(TIMER_CH3CV - TIMER_CH2CV, 0x04);
        assert_eq!(CCHP_MOE, 1 << 15);
    }

    // --- PwmHandle::set_duties writes the three CHxCV at the right offsets ----------------------

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
        // The channel compares are untouched.
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

    // --- The SAFETY invariant: the per-cycle handle cannot touch MOE (DECISIONS.md #4) ----------

    /// The PWM handle's per-cycle methods write only the four compare registers and NEVER the
    /// CCHP/MOE bit. Arming is a separate, deliberately distinct call ([`ArmGate`]). This is the
    /// load-bearing SAFETY invariant: a control-loop bug holding only the handle cannot energize a
    /// disarmed bridge.
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

    // --- M3 T5: the ComplementaryPwm trait -> resolve-once handle -------------------------------

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

        // The T3/T4 config writes happened: CAR = 2250, CTL0 = center-up | CKDIV/2 | ARSE, CCHP
        // holds the dead-time + off-state word with MOE OFF.
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

        // Then the per-cycle duty writes land at the three channel compares.
        h.set_duties([100, 200, 300]).unwrap();
        assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 100);
        assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH1CV).read(), 200);
        assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH2CV).read(), 300);

        // The trace's MOE writer is the gate, distinct from the handle (the only MOE writer). The
        // gate is built from the resolved base (the safety layer resolves it separately).
        let gate = ArmGate::new(chip.base(cfg.timer).unwrap());
        gate.arm();
        assert_eq!(
            Reg32::new(TIMER0_BASE, TIMER_CCHP).read() & CCHP_MOE,
            CCHP_MOE
        );
    }

    /// `PwmController::configure` resolves the timer base from the chip's [`AddrTable`] (via
    /// `chip.base(cfg.timer)`). A present base succeeds; a MISSING base is rejected as
    /// `BringUpError::Descriptor(MissingBase(..))`.
    #[test]
    fn from_descriptor_resolves_and_range_checks() {
        let mut addrs = AddrTable::new();
        addrs.set(PeriphLabel::Timer0, TIMER0_BASE);
        let chip = chip_with(addrs);
        let h = PwmController::new()
            .configure(&chip, &reference_config())
            .expect("present base resolves");
        assert_eq!(h.period(), 2250);

        // A missing base is rejected (MissingBase, mapped to BringUpError::Descriptor).
        let empty = chip_with(AddrTable::new());
        assert!(matches!(
            PwmController::new().configure(&empty, &reference_config()),
            Err(BringUpError::Descriptor(
                crate::error::DescriptorError::MissingBase(_)
            ))
        ));
    }

    // --- M3 T7: the per-cycle rearm_trigger targets the SAME CH3 the T6 trigger config programs --

    /// T7: confirm `PwmHandle::rearm_trigger` re-arms the EXACT compare register (CH3CV, offset
    /// 0x40) that the T6 timer trigger config ([`crate::timer::PwmTimer::configure_trigger`])
    /// programs as the ADC-trigger channel.
    #[test]
    fn rearm_trigger_matches_t6_trigger_channel() {
        let _serial = mock::lock();
        mock::reset();

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

        let h = PwmHandle::new(TIMER0_BASE, 2250);
        h.rearm_trigger(2200).unwrap();
        assert_eq!(
            Reg32::new(TIMER0_BASE, TIMER_CH3CV).read(),
            2200,
            "rearm_trigger writes the T6 trigger channel CH3CV"
        );
        // The channel compares are untouched by the re-arm.
        assert_eq!(Reg32::new(TIMER0_BASE, TIMER_CH0CV).read(), 0);
    }

    /// The handle is `Copy` / concrete (no `dyn`, no descriptor lookup per call): it can be copied
    /// and each copy writes the same resolved registers. This is the resolve-once invariant.
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
}
