//! T10/T11 host tests for the shared regular-ADC driver (run under the `mock` feature against the
//! backing-array register space).
//!
//! Four groups:
//! - **config end state** ([`Adc::configure`] via the internal path): CTL0 SM clear (single), CTL1
//!   CTN/DAL clear (single, right-aligned) + ETSRC software code + ETERC + ADCON, TSVREN set only
//!   for an internal channel, the RSQ0 RL = 0 (length 1), the rank-0 channel field in RSQ2, and
//!   the channel's sample-time field in SAMPT0/1, vs the SPL `adc_*` recipe.
//! - **field placement** ([`Adc::set_regular_rank`] / [`Adc::set_sample_time`]): a rank lands in
//!   the right RSQ register/shift and a channel's sample time in the right SAMPT register/shift.
//! - **calibration** ([`Adc::calibrate`]): the bounded RSTCLB/CLB poll returns [`AdcError::Timeout`]
//!   rather than spinning forever when a calibration bit never clears (the hang-if-done-wrong
//!   class); the happy path is the with_polling harness golden (a sequencer, not this flat mock).
//! - **read API** ([`Adc::read_channel`] / [`Adc::read_data`], open item ADC-1): seed STAT EOC +
//!   RDATA, assert the read returns the seeded value; an unset EOC times out.
//!
//! The mock backend is a flat array (a static register snapshot, not a sequencer), so a
//! self-clearing bit (RSTCLB/CLB) never clears and EOC never sets on its own; the calibration
//! happy path and the EOC flag-by-flag progression are the with_polling golden's job (T11 harness
//! layer) and the bench VREFINT/temperature read (T13). This host layer proves the config register
//! end state, the field placement, the bounded-poll escape, and the read value path.
#![cfg(feature = "mock")]

use super::*;
use crate::reg::{mock, Reg32};
use std::sync::MutexGuard;

/// The ADC0 base (the mock window wraps modulo its size; only the offsets matter).
const ADC0_BASE: u32 = 0x4001_2400;
/// VREFINT internal channel (channel 17) and the slowest sample time (code 7, 239.5 cycles).
const VREFINT: u8 = 17;
const SAMPLE_239: u8 = 7;

fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

fn r(off: u32) -> u32 {
    Reg32::new(ADC0_BASE, off).read()
}
fn w(off: u32, v: u32) {
    Reg32::new(ADC0_BASE, off).write(v);
}

fn dev() -> Adc {
    Adc { base: ADC0_BASE }
}

// --- config end state -------------------------------------------------------------------------

#[test]
fn configure_internal_channel_sets_single_right_software_trigger_and_tsvren() {
    let _g = seed_reset();
    dev().configure(VREFINT, SAMPLE_239);

    // CTL0 SM clear (single, not scan).
    assert_eq!(r(CTL0) & CTL0_SM, 0, "SM clear (single conversion)");

    let ctl1 = r(CTL1);
    assert_eq!(ctl1 & CTL1_CTN, 0, "CTN clear (single, not continuous)");
    assert_eq!(ctl1 & CTL1_DAL, 0, "DAL clear (right-aligned 12-bit)");
    assert_eq!(
        ctl1 & CTL1_ETSRC,
        ETSRC_SOFTWARE,
        "ETSRC = software-trigger code (7)"
    );
    assert_eq!(ctl1 & CTL1_ETERC, CTL1_ETERC, "ETERC set");
    assert_eq!(ctl1 & CTL1_ADCON, CTL1_ADCON, "ADCON set (ADC on)");
    assert_eq!(
        ctl1 & CTL1_TSVREN,
        CTL1_TSVREN,
        "TSVREN set for an internal channel"
    );

    // Regular sequence: length 1 -> RSQ0 RL = 0; rank 0 = channel 17 in RSQ2.
    assert_eq!(r(RSQ0) & RSQ0_RL, 0, "RL = 0 (length-1 sequence)");
    assert_eq!(
        r(RSQ2) & RSQ_FIELD,
        VREFINT as u32,
        "rank 0 holds channel 17"
    );

    // Channel 17 sample time lives in SAMPT0 (channels 10..17), field index (17-10)=7, shift 3*7=21.
    let shift = 3 * (VREFINT as u32 - 10);
    assert_eq!(
        (r(SAMPT0) >> shift) & SAMPT_FIELD,
        SAMPLE_239 as u32,
        "channel 17 sample time = 239.5"
    );
}

#[test]
fn configure_external_channel_leaves_tsvren_clear() {
    let _g = seed_reset();
    // Channel 0 (an external pin channel): no TSVREN, sample time in SAMPT1 (channels 0..9).
    dev().configure(0, SAMPLE_239);
    assert_eq!(
        r(CTL1) & CTL1_TSVREN,
        0,
        "TSVREN stays clear for an external channel"
    );
    assert_eq!(r(RSQ2) & RSQ_FIELD, 0, "rank 0 holds channel 0");
    assert_eq!(
        r(SAMPT1) & SAMPT_FIELD,
        SAMPLE_239 as u32,
        "channel 0 sample time in SAMPT1"
    );
}

// --- field placement --------------------------------------------------------------------------

#[test]
fn rank_lands_in_the_right_rsq_register_and_shift() {
    let _g = seed_reset();
    let d = dev();
    // Ranks 0..5 -> RSQ2; 6..11 -> RSQ1; 12..15 -> RSQ0 (low bits, above the RL field).
    d.set_regular_rank(0, 3);
    d.set_regular_rank(5, 9);
    d.set_regular_rank(6, 1);
    d.set_regular_rank(12, 4);
    assert_eq!(r(RSQ2) & RSQ_FIELD, 3, "rank 0 -> RSQ2 bits[4:0]");
    assert_eq!(
        (r(RSQ2) >> (5 * 5)) & RSQ_FIELD,
        9,
        "rank 5 -> RSQ2 bits[29:25]"
    );
    assert_eq!(r(RSQ1) & RSQ_FIELD, 1, "rank 6 -> RSQ1 bits[4:0]");
    assert_eq!(
        r(RSQ0) & RSQ_FIELD,
        4,
        "rank 12 -> RSQ0 bits[4:0] (below RL)"
    );
}

#[test]
fn sample_time_lands_in_the_right_sampt_register_and_shift() {
    let _g = seed_reset();
    let d = dev();
    // Channel 9 -> SAMPT1 (0..9), channel 10 -> SAMPT0 (10..17).
    d.set_sample_time(9, 5);
    d.set_sample_time(10, 2);
    assert_eq!(
        (r(SAMPT1) >> (3 * 9)) & SAMPT_FIELD,
        5,
        "channel 9 -> SAMPT1 field 9"
    );
    assert_eq!(r(SAMPT0) & SAMPT_FIELD, 2, "channel 10 -> SAMPT0 field 0");
}

// --- calibration: bounded poll escapes a stuck bit --------------------------------------------

#[test]
fn calibrate_times_out_when_calibration_bit_never_clears() {
    let _g = seed_reset();
    // The flat mock holds the last write, so RSTCLB stays set after calibrate sets it: the bounded
    // poll must escape with Timeout, not spin forever (the F130 hang-if-done-wrong class).
    let e = dev().calibrate().unwrap_err();
    assert_eq!(
        e,
        AdcError::Timeout,
        "calibration must time out, not hang, on a stuck bit"
    );
}

// --- read API (ADC-1) -------------------------------------------------------------------------

#[test]
fn read_data_returns_rdata_when_eoc_set() {
    let _g = seed_reset();
    // Seed EOC set and a known conversion result. read_data polls EOC (passes immediately) then
    // reads RDATA.
    w(STAT, STAT_EOC);
    w(RDATA, 0x0654); // a 12-bit-ish value (1620 counts ~ VREFINT region)
    assert_eq!(
        dev().read_data().unwrap(),
        0x0654,
        "read_data returns the RDATA value"
    );
}

#[test]
fn read_channel_triggers_repoints_rank0_and_returns_value() {
    let _g = seed_reset();
    w(STAT, STAT_EOC);
    w(RDATA, 0x0321);
    let v = dev().read_channel(VREFINT).unwrap();
    assert_eq!(v, 0x0321, "read_channel returns the conversion value");
    // read_channel re-points rank 0 to the requested channel and sets the SWRCST software trigger.
    assert_eq!(
        r(RSQ2) & RSQ_FIELD,
        VREFINT as u32,
        "rank 0 re-pointed to the requested channel"
    );
    assert_eq!(
        r(CTL1) & CTL1_SWRCST,
        CTL1_SWRCST,
        "software trigger (SWRCST) set"
    );
}

#[test]
fn read_data_times_out_when_eoc_never_sets() {
    let _g = seed_reset();
    // STAT left at 0: EOC never sets, the bounded poll exhausts -> Timeout (not a hang).
    let e = dev().read_data().unwrap_err();
    assert_eq!(
        e,
        AdcError::Timeout,
        "EOC poll times out when the conversion never completes"
    );
}

#[test]
fn is_internal_channel_covers_16_and_17() {
    assert!(is_internal_channel(16) && is_internal_channel(17));
    assert!(!is_internal_channel(0) && !is_internal_channel(15) && !is_internal_channel(18));
}

// --- M3 T8: injected (inserted) conversion group config -----------------------------------------

/// `configure_injected` programs the injected group end state in SPL order: DAL (left), ISQ IL =
/// len-1, the per-rank ISQ channel fields (SPL reversed packing), per-channel SAMPT, ETSIC = the
/// trigger code, ETEIC, EOICIE (CTL0), ADCON. Two injected channels (4, 5) at 7.5 cycles.
#[test]
fn configure_injected_end_state() {
    let _g = seed_reset();
    let chans = [(4u8, 1u8), (5u8, 1u8)];
    let _ = Adc::configure_injected(ADC0_BASE, &chans, true, ETSIC_T0_CH3);

    // CTL1: DAL (left, bit 11), ETSIC = code 1 (TIMER0 CH3) in [14:12], ETEIC (bit 15), ADCON (0).
    let ctl1 = r(CTL1);
    assert_ne!(ctl1 & CTL1_DAL, 0, "left-aligned (DAL set)");
    assert_eq!(ctl1 & CTL1_ETSIC, 1 << 12, "ETSIC = TIMER0 CH3 (code 1)");
    assert_ne!(ctl1 & CTL1_ETEIC, 0, "ETEIC (injected ext-trigger enable)");
    assert_ne!(ctl1 & CTL1_ADCON, 0, "ADCON (enabled)");
    // CTL0: EOICIE (bit 7).
    assert_ne!(
        r(CTL0) & CTL0_EOICIE,
        0,
        "EOICIE (injected-EOC interrupt enable)"
    );

    // ISQ: IL = len-1 = 1. SPL reversed packing for length 2 (IL=1): rank 0 at bits[14:10] = ch 4,
    // rank 1 at bits[19:15] = ch 5.
    let isq = r(ISQ);
    assert_eq!((isq >> 20) & 0x3, 1, "ISQ IL = len-1");
    assert_eq!(
        (isq >> 10) & 0x1F,
        4,
        "rank 0 channel field (bits[14:10]) = ch 4"
    );
    assert_eq!(
        (isq >> 15) & 0x1F,
        5,
        "rank 1 channel field (bits[19:15]) = ch 5"
    );

    // SAMPT1 holds channels 0..9: ch 4 + ch 5 sample-time fields = code 1 each.
    let sampt1 = r(SAMPT1);
    assert_eq!((sampt1 >> (3 * 4)) & 0x7, 1, "ch4 sample time (7.5 cycles)");
    assert_eq!((sampt1 >> (3 * 5)) & 0x7, 1, "ch5 sample time (7.5 cycles)");
}

/// The TRGO trigger code maps to ETSIC = 0 (TIMER0 TRGO).
#[test]
fn configure_injected_trgo_etsic_is_zero() {
    let _g = seed_reset();
    let chans = [(4u8, 1u8)];
    let _ = Adc::configure_injected(ADC0_BASE, &chans, true, ETSIC_T0_TRGO);
    assert_eq!(r(CTL1) & CTL1_ETSIC, 0, "ETSIC = TIMER0 TRGO (code 0)");
}

/// `read_injected_data` reads IDATA0..3 (0x3C/0x40/0x44/0x48) by injected index.
#[test]
fn read_injected_data_reads_idata() {
    let _g = seed_reset();
    Reg32::new(ADC0_BASE, 0x3C).write(0x0123);
    Reg32::new(ADC0_BASE, 0x40).write(0x0456);
    let dev = Adc::at(ADC0_BASE);
    assert_eq!(dev.read_injected_data(0), 0x0123);
    assert_eq!(dev.read_injected_data(1), 0x0456);
}

/// The injected EOIC poll is bounded: an EOIC that never sets times out (not a hang).
#[test]
fn wait_eoic_times_out() {
    let _g = seed_reset();
    let e = Adc::at(ADC0_BASE).wait_eoic().unwrap_err();
    assert_eq!(e, AdcError::Timeout);
}

/// Host tests for the timer-triggered injected-ADC per-cycle path: the [`TriggeredAdc`] trait, the
/// resolve-once [`InjectedHandle`], and the trigger-link validation.
mod injected {
    use crate::adc::{InjectedAdcController, InjectedHandle, TriggeredAdc};
    use crate::addr::{AddrTable, PeriphLabel};
    use crate::chip::Chip;
    use crate::config::{
        AdcClockDiv, InjectedAdcConfig, InjectedChannel, TimerTriggerLink,
    };
    use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
    use crate::error::BringUpError;
    use crate::reg::{mock, Reg32};

    /// An ADC0 base inside the ADC APB2 window.
    const ADC0_BASE: u32 = 0x4001_2400;
    /// A TIMER0 base inside the advanced-timer APB2 window (used as a non-ADC base for the
    /// out-of-window rejection).
    const TIMER0_BASE: u32 = 0x4001_2C00;

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

    /// A [`Chip`] whose ADC0 resolves to `ADC0_BASE` (the reference ADC base).
    fn adc0_chip() -> Chip {
        let mut a = AddrTable::new();
        a.set(PeriphLabel::Adc0, ADC0_BASE);
        chip_with(a)
    }

    /// The reference injected-ADC wiring on ADC0, triggered by TIMER0 CH3: two channels (4 and 5)
    /// at 7.5-cycle sample time (code 1), left-aligned.
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

    /// `TriggeredAdc::configure` runs the T8 injected bring-up INCLUDING the calibration poll. In the
    /// flat mock register space the calibration self-clearing bit (RSTCLB) never clears, so the
    /// bounded poll exits as `Timeout`. The point here: the trait does wire the config writes AND
    /// drive calibration, and the config writes are present in the register space even though
    /// calibration then times out.
    #[test]
    fn configure_injected_via_trait_drives_config_and_calibration() {
        let _serial = mock::lock();
        mock::reset();

        let chip = adc0_chip();
        let res = InjectedAdcController::new().configure(&chip, &reference_injected_config());
        assert!(
            matches!(res, Err(BringUpError::Adc(crate::error::AdcError::Timeout))),
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
    /// channel list is rejected at config BEFORE any register write or calibration.
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

        // A non-TIMER0 trigger timer is not expressible on the single-ADC baseline: rejected.
        let mut bad = reference_injected_config();
        bad.trigger_timer = PeriphLabel::Timer7;
        assert!(matches!(
            InjectedAdcController::new().configure(&chip, &bad),
            Err(BringUpError::Adc(_))
        ));

        // An empty channel list is rejected.
        let mut empty = reference_injected_config();
        empty.channels.clear();
        assert!(matches!(
            InjectedAdcController::new().configure(&chip, &empty),
            Err(BringUpError::Adc(_))
        ));
    }

    /// `InjectedAdcController::configure` resolves + range-checks the ADC base from the chip's
    /// [`AddrTable`]. A base in the ADC window passes; one outside the ADC window, or missing, is
    /// rejected as `BringUpError::Descriptor(..)` BEFORE any calibration (so not a Timeout).
    #[test]
    fn injected_from_descriptor_resolves_and_range_checks() {
        let _serial = mock::lock();
        mock::reset();

        let chip = adc0_chip();
        assert!(matches!(
            InjectedAdcController::new().configure(&chip, &reference_injected_config()),
            Err(BringUpError::Adc(crate::error::AdcError::Timeout))
        ));

        // A base outside the ADC window (a non-ADC base) is rejected at the descriptor layer.
        let mut bad = AddrTable::new();
        bad.set(PeriphLabel::Adc0, TIMER0_BASE);
        let bad_chip = chip_with(bad);
        assert!(matches!(
            InjectedAdcController::new().configure(&bad_chip, &reference_injected_config()),
            Err(BringUpError::Descriptor(_))
        ));

        // A missing base is rejected at the descriptor layer too.
        let empty = chip_with(AddrTable::new());
        assert!(matches!(
            InjectedAdcController::new().configure(&empty, &reference_injected_config()),
            Err(BringUpError::Descriptor(_))
        ));
    }
}

/// Host tests for the F10x dual-ADC regular-simultaneous ("routine parallel") datapath: the SYNCM
/// mode write on the master + the paired RDATA read. Register writes are diffed against the GD SPL
/// recipe (`gd32f10x_adc.c` `adc_mode_config(ADC_DAUL_REGULAL_PARALLEL)` /
/// `adc_sync_mode_convert_value_read`) + the GD32F10x User Manual section 11.5.2 / Table 11-5 / the
/// ADC_CTL0 SYNCM and ADC_RDATA field tables. This is UNVALIDATABLE on the bench (the only dual-ADC
/// part we own is the dump-only 12-FET); the host register diff + the SPL recipe are the gate.
mod dual {
    use super::*;
    use crate::adc::DualAdc;

    /// ADC0 (master) + ADC1 (slave) bases (the mock window wraps modulo its size; the two differ in
    /// their low 16 bits by 0x400, well above the 0x4C-max register offset, so they do not alias).
    const ADC0: u32 = 0x4001_2400;
    const ADC1: u32 = 0x4001_2800;

    fn dual() -> DualAdc {
        DualAdc::new(Adc::at(ADC0), Adc::at(ADC1))
    }

    fn r0(off: u32) -> u32 {
        Reg32::new(ADC0, off).read()
    }
    fn r1(off: u32) -> u32 {
        Reg32::new(ADC1, off).read()
    }

    /// `configure_simultaneous` configures each ADC's regular single channel AND sets SYNCM = routine
    /// parallel (code 6) on ADC0 (the master). The User Manual ADC_CTL0 table is explicit that SYNCM
    /// lives only in ADC0, so ADC1's CTL0 must NOT carry it.
    #[test]
    fn configure_simultaneous_sets_syncm_on_master_only_and_configs_each_adc() {
        let _g = seed_reset();
        // ADC0 reads channel 4, ADC1 reads channel 5, both at 7.5 cycles (code 1).
        dual().configure_simultaneous(4, 5, 1);

        // SYNCM[19:16] = 6 (routine parallel) on ADC0's CTL0.
        assert_eq!(
            (r0(CTL0) >> 16) & 0xF,
            6,
            "ADC0 SYNCM = routine parallel (code 6)"
        );
        // SYNCM is ADC0-only: ADC1's CTL0 SYNCM field stays 0.
        assert_eq!((r1(CTL0) >> 16) & 0xF, 0, "ADC1 carries no SYNCM");

        // Each ADC got its single-channel config: ADCON set, rank 0 = its channel.
        assert_ne!(r0(CTL1) & CTL1_ADCON, 0, "ADC0 enabled");
        assert_ne!(r1(CTL1) & CTL1_ADCON, 0, "ADC1 enabled");
        assert_eq!(r0(RSQ2) & RSQ_FIELD, 4, "ADC0 rank 0 = channel 4");
        assert_eq!(r1(RSQ2) & RSQ_FIELD, 5, "ADC1 rank 0 = channel 5");
    }

    /// `read_simultaneous` software-triggers the master and reads the paired result from ADC0's
    /// 32-bit RDATA: `[15:0]` = ADC0, `[31:16]` = ADC1 (the ADC1RDTR half), per the User Manual
    /// ADC_RDATA field table + SPL `adc_sync_mode_convert_value_read`.
    #[test]
    fn read_simultaneous_unpacks_paired_rdata_halves() {
        let _g = seed_reset();
        // Seed EOC set on the master and a packed paired result: ADC1 (0x0ABC) in [31:16], ADC0
        // (0x0123) in [15:0].
        Reg32::new(ADC0, STAT).write(STAT_EOC);
        Reg32::new(ADC0, RDATA).write((0x0ABC << 16) | 0x0123);

        let (p, s) = dual().read_simultaneous().unwrap();
        assert_eq!(p, 0x0123, "primary (ADC0) from RDATA[15:0]");
        assert_eq!(s, 0x0ABC, "secondary (ADC1) from RDATA[31:16]");
        // The master was software-triggered (SWRCST set).
        assert_ne!(r0(CTL1) & CTL1_SWRCST, 0, "master software-triggered");
    }

    /// The paired-read EOC poll is bounded: if the master's EOC never sets, it times out (not a hang).
    #[test]
    fn read_simultaneous_times_out_when_eoc_never_sets() {
        let _g = seed_reset();
        let e = dual().read_simultaneous().unwrap_err();
        assert_eq!(e, AdcError::Timeout);
    }

    /// The two independent handles are reachable from the token (each usable for single conversions).
    #[test]
    fn dual_exposes_both_handles() {
        let d = dual();
        assert_eq!(d.primary().base(), ADC0);
        assert_eq!(d.secondary().base(), ADC1);
    }
}
