//! Shared advanced-timer complementary-PWM bring-up (M3 T3 + T4).
//!
//! SPEC.md: the advanced timer is **one shared path parameterised by base**. The advanced-timer
//! register block is identical on F10x and F1x0 (verified against `gd32f10x_timer.h` /
//! `gd32f1x0_timer.h`: same offsets, same bit positions), so there is a single register model here,
//! parameterised only by the base address (data, from [`crate::addr::AddrTable`]). There is no
//! [`crate::descriptor::ClockPath`]-style family selector on this path.
//!
//! # What this module does (config-only; MOE stays OFF)
//!
//! [`PwmTimer::configure`] reproduces the GD SPL advanced-timer complementary-PWM bring-up
//! sequence for TIMER0, in the SPL order:
//!
//! - `timer_init` (T3): PSC, the center-aligned mode + direction in CTL0, CAR (the period), the
//!   clock-division field in CTL0, the repetition counter (CREP = 0), then the update-generate
//!   software event (SWEVG UPG) that latches the shadowed PSC/CAR.
//! - `timer_channel_output_config` + `timer_channel_output_mode_config` +
//!   `timer_channel_output_shadow_config` + `timer_channel_output_pulse_value_config` per channel
//!   (T3 sets PWM mode0 + the compare shadow + a zero initial duty; T4 adds the
//!   complementary-output enable, the per-channel polarity and idle state).
//! - `timer_auto_reload_shadow_enable` (CTL0 ARSE).
//! - `timer_break_config` (T4): the CCHP dead-time / break / off-state / protect word, written as
//!   one assignment exactly as the SPL does. The reference DISABLES break; the path expresses
//!   enabling it.
//! - **MOE is left OFF.** `timer_primary_output_config(ENABLE)` (CCHP POEN) is NOT called here: the
//!   bridge stays disarmed at bring-up. MOE is owned by [`arming::ArmGate`], the
//!   only MOE writer, a SAFETY invariant (DECISIONS.md #4 + the M3 SAFETY section). Nothing in this
//!   module touches CCHP POEN.
//!
//! This is config-only: pure writes plus the RMW reads of reset values, so it traces in
//! `final_state` mode against the SPL goldens (the SPL builds CTL0/CHCTL/CHCTL2/CTL1 with many
//! `|=` / `&=` pairs in a different write count and order than runtime-hal's field-scoped
//! [`Reg32::modify`]; the correctness criterion is the END STATE of each register).
//!
//! # Register model (identical on both families; `gd32*_timer.h`)
//!
//! | reg      | offset | what                                                                  |
//! |----------|--------|-----------------------------------------------------------------------|
//! | `CTL0`   | `0x00` | CEN(0), DIR(4), `CAM[6:5]`, ARSE(7), `CKDIV[9:8]`                      |
//! | `CTL1`   | `0x04` | ISO0(8)/ISO0N(9)/ISO1(10)/ISO1N(11)/ISO2(12)/ISO2N(13)/ISO3(14) idle states |
//! | `SWEVG`  | `0x14` | UPG(0) update-event generate                                          |
//! | `CHCTL0` | `0x18` | CH0/CH1 mode-select + output-compare mode + shadow enable             |
//! | `CHCTL1` | `0x1C` | CH2/CH3 mode-select + output-compare mode + shadow enable             |
//! | `CHCTL2` | `0x20` | CHxEN / CHxP / CHxNEN / CHxNP (enable + polarity, main + complementary)|
//! | `PSC`    | `0x28` | prescaler                                                             |
//! | `CAR`    | `0x2C` | counter auto-reload (the PWM period)                                  |
//! | `CREP`   | `0x30` | repetition counter                                                    |
//! | `CH0CV`  | `0x34` | channel-0 compare (duty); `+4` per channel up to `CH3CV` 0x40         |
//! | `CCHP`   | `0x44` | `DTCFG[7:0]` / `PROT[9:8]` / IOS(10) / ROS(11) / BRKEN(12) / BRKP(13) / OAEN(14) / POEN(15) |
//!
//! The per-cycle compare writes (CH0CV/CH1CV/CH2CV + the CH3CV trigger compare) and the MOE gate
//! (CCHP POEN) live in this module too (the resolve-once [`PwmHandle`] + the [`arming`] layer); the
//! bring-up below is the one-time config that they build on.

use crate::chip::Chip;
use crate::config::{OcMode, PwmConfig, TrgoSource};
use crate::descriptor::MAX_PWM_CHANNELS;
use crate::error::{BringUpError, DescriptorError, PwmError};
use crate::reg::Reg32;

use arming::ArmGate;

// --- register offsets (identical on both families) --------------------------------------------

const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
const SWEVG: u32 = 0x14;
const CHCTL0: u32 = 0x18;
const CHCTL1: u32 = 0x1C;
const CHCTL2: u32 = 0x20;
const PSC: u32 = 0x28;
const CAR: u32 = 0x2C;
const CREP: u32 = 0x30;
/// CH0CV; channel `n` compare is `CH0CV + 4*n` (CH1CV 0x38, CH2CV 0x3C, CH3CV 0x40).
const CH0CV: u32 = 0x34;
const CCHP: u32 = 0x44;

// --- CTL0 fields (timer_init: center-aligned + direction + clock division + ARSE) -------------

/// Counter enable (CEN), CTL0[0]. Set to START the timer counter; the bring-up leaves it clear.
const CTL0_CEN: u32 = 1 << 0;
const CTL0_DIR: u32 = 1 << 4;
/// Center-aligned mode select, CTL0[6:5].
const CTL0_CAM: u32 = 0b11 << 5;
/// Auto-reload shadow enable (timer_auto_reload_shadow_enable).
const CTL0_ARSE: u32 = 1 << 7;
/// Clock division field, CTL0[9:8] (the dead-time / sampling clock divider, fDTS).
const CTL0_CKDIV: u32 = 0b11 << 8;

// --- SWEVG ------------------------------------------------------------------------------------

/// Update-event generate (SWEVG UPG): latches the shadowed PSC/CAR, as `timer_init`'s last step.
const SWEVG_UPG: u32 = 1 << 0;

// --- CHCTL0 / CHCTL1 output-compare fields (per channel; CH0/2 low half, CH1/3 high half) ------
//
// CH0 lives in CHCTL0[7:0], CH1 in CHCTL0[15:8]; CH2 in CHCTL1[7:0], CH3 in CHCTL1[15:8]. The
// per-channel fields within a half: MS[1:0] (mode-select; 0 = output), COMCTL[6:4] (output-compare
// mode), COMSEN(3) (compare shadow enable). The SPL builds these with masked RMWs; runtime-hal
// programs the whole channel half (clear the half's relevant fields, set MS=0 output, the PWM mode,
// and the shadow-enable bit) so the end state matches.

/// PWM mode 0 (`TIMER_OC_MODE_PWM0`): COMCTL = 0b110 -> bits[6:4] = 0x60 within a channel half.
const OC_MODE_PWM0: u32 = 0x60;
/// Output-compare shadow enable (`TIMER_OC_SHADOW_ENABLE`): bit 3 within a channel half.
const OC_SHADOW_ENABLE: u32 = 0x08;

// --- CHCTL2 enable / polarity fields (per channel, 4 bits each: EN/P/NEN/NP) -------------------
//
// CH0 occupies CHCTL2[3:0], CH1 [7:4], CH2 [11:8] (shift = 4*n): EN(0), P(1), NEN(2), NP(3).

/// Channel output enable (`TIMER_CCX_ENABLE`), bit 0 within a channel's 4-bit field.
const CCX_EN: u32 = 1 << 0;
/// Complementary output enable (`TIMER_CCXN_ENABLE`), bit 2 within the field.
const CCXN_EN: u32 = 1 << 2;
/// Complementary output polarity LOW (`TIMER_OCN_POLARITY_LOW`), bit 3 within the field.
const CCXN_P_LOW: u32 = 1 << 3;
/// All four enable/polarity bits of one channel's CHCTL2 field (the RMW mask per channel).
const CHCTL2_CHAN_FIELDS: u32 = 0b1111;

// --- CTL1 master-mode TRGO select (T6: timer_master_output_trigger_source_select) -------------
//
// CTL1[6:4] = MMC (master mode control): the TRGO source. The reference drives the ADC from the
// UPDATE event on TRGO (TIMER_TRI_OUT_SRC_UPDATE = MMC value 2). Identical on both families
// (gd32f1x0_timer.h / gd32f10x_timer.h: TIMER_CTL1_MMC = BITS(4,6)). MMC sits below the ISOx idle
// fields (bit 8+) in the same CTL1 word, so the trigger-source RMW and the idle-state RMW touch
// disjoint bits.

/// Master-mode control field, CTL1[6:4] (`TIMER_CTL1_MMC`). The TRGO source; set from
/// [`TrgoSource`] (no baked UPDATE).
const CTL1_MMC: u32 = 0b111 << 4;

// --- CTL1 idle-state fields (per channel: ISOx + ISOxN, two bits at 8 + 2*n) -------------------
//
// CH0: ISO0(8)/ISO0N(9); CH1: ISO1(10)/ISO1N(11); CH2: ISO2(12)/ISO2N(13). The idle state a
// disarmed/fault output (MOE clear) drives. shift = 8 + 2*n.

/// Main-output idle-state HIGH within a channel's CTL1 idle field (bit 0 of the pair).
const ISO_HIGH: u32 = 1 << 0;
/// Complementary-output idle-state HIGH within the field (bit 1 of the pair).
const ISON_HIGH: u32 = 1 << 1;
/// Both idle bits of one channel's CTL1 field (the RMW mask per channel).
const CTL1_ISO_CHAN_FIELDS: u32 = 0b11;

// --- CCHP fields (timer_break_config: one assignment of the whole word) ------------------------

/// Dead-time configure field, CCHP[7:0] (`TIMER_CCHP_DTCFG`).
const CCHP_DTCFG: u32 = 0xFF;
/// Run-mode off-state enable (`TIMER_ROS_STATE_ENABLE`), bit 11. With POEN set, the configured
/// idle drives the disabled channels; the reference enables it so a running but disabled channel
/// sits safe.
const CCHP_ROS: u32 = 1 << 11;
/// Idle-mode off-state enable (`TIMER_IOS_STATE_ENABLE`), bit 10. With POEN clear (disarmed), the
/// configured idle drives the outputs; the reference enables it so a disarmed bridge sits safe.
const CCHP_IOS: u32 = 1 << 10;
/// Break enable (`TIMER_BREAK_ENABLE`), bit 12.
const CCHP_BRKEN: u32 = 1 << 12;
/// Break polarity HIGH (`TIMER_BREAK_POLARITY_HIGH`), bit 13.
const CCHP_BRKP_HIGH: u32 = 1 << 13;

/// The shared advanced-timer complementary-PWM bring-up at `base`. Holds only the resolved base;
/// the per-cycle compare writes and the MOE arming gate are elsewhere (see the module docs).
#[derive(Debug, Clone, Copy)]
pub struct PwmTimer {
    base: u32,
    period: u16,
}

impl PwmTimer {
    /// Wrap an already-resolved advanced-timer base (no register access). HAL-internal constructor for
    /// the trigger-config path and tests; `period` is 0 (the per-cycle handle is built by
    /// [`Self::configure`], which records the real period). NOT public: the application cannot supply a
    /// base.
    #[inline]
    #[allow(dead_code)] // internal/test constructor; the prod path builds PwmTimer via configure().
    pub(crate) const fn at(base: u32) -> PwmTimer {
        PwmTimer { base, period: 0 }
    }

    /// Configure the advanced timer for the complementary bridge from `wiring`, reproducing the GD
    /// SPL bring-up sequence (see the module docs), and **leave MOE OFF** (the bridge disarmed).
    ///
    /// T3 programs the time base (PSC/CAR/CKDIV center-aligned) and three PWM-mode compare channels
    /// with the compare shadow + a zero initial duty; T4 adds the complementary-output enable, the
    /// per-channel polarity + idle state, and the CCHP dead-time / break / off-state word. The
    /// returned [`PwmTimer`] is the resolved base; the caller builds the resolve-once
    /// [`PwmHandle`] from it (T5).
    pub fn configure(chip: &Chip, cfg: &PwmConfig) -> Result<PwmTimer, DescriptorError> {
        // Guard: the config's timer label must be an ADVANCED timer in the APB2 advanced-timer
        // window, else a non-advanced label (e.g. the general-purpose Timer1) would run the full
        // complementary-bridge + CCHP dead-time bring-up against the wrong peripheral. Mirrors the
        // injected-ADC path's check_adc_base guard. Also gives Timer7 (when present) a clean resolve.
        chip.descriptor().addrs.check_timer_base(cfg.timer)?;
        let base = chip.base(cfg.timer)?;
        let dev = PwmTimer {
            base,
            period: cfg.period,
        };
        dev.timer_init(cfg);
        // Per-channel output config (mode + shadow + zero duty, enable/per-side idle/polarity).
        for (n, ch) in cfg.channels.iter().enumerate() {
            dev.channel_output_config(n as u32, ch.polarity, ch.idle_high, ch.idle_high_n);
            dev.channel_output_mode_pwm0(n as u32);
            dev.channel_output_shadow_enable(n as u32);
            // Zero initial duty (the control loop writes real duties per cycle through the handle).
            dev.channel_compare(n as u32, 0);
        }
        // timer_auto_reload_shadow_enable: CTL0 ARSE (from cfg.arse; no baked ARSE=on).
        dev.ctl0()
            .modify(CTL0_ARSE, if cfg.arse { CTL0_ARSE } else { 0 });
        // timer_break_config: the CCHP dead-time / break / off-state / protect word, one write.
        dev.break_config(cfg.dead_time, cfg.brk.enabled, cfg.brk.level);
        // MOE stays OFF: timer_primary_output_config(ENABLE) is NOT called. Arming is ArmGate's.
        //
        // NOTE: the CH3 ADC-trigger compare + TRGO master-mode are NOT programmed here; they are the
        // separate step ([`PwmTimer::configure_trigger`]) so the bring-up golden stays CH3-untouched.
        Ok(dev)
    }

    /// T6: configure CH3 as the ADC-trigger compare channel and set the TRGO master-mode source.
    ///
    /// CH3 (channel index 3) is NOT a bridge output channel: its compare match is used as the injected
    /// ADC's external trigger (the reference samples the injected channels at the PWM centre). It is
    /// programmed as an output-compare channel in PWM mode 0 with the compare shadow enabled (so a
    /// re-arm is buffered like the channel duties), its compare value (CH3CV) set near the up-count top
    /// (`trigger_compare`, ~CAR-1), exactly as the SPL `timer_channel_output_*` + `..._pulse_value_*`
    /// recipe for CH3. The channel OUTPUT-enable (CH3EN) is left OFF: the compare event drives the
    /// internal trigger; no CH3 pin is wired (so this does not disturb CHCTL2's channel fields).
    ///
    /// Then `timer_master_output_trigger_source_select(TIMER_TRI_OUT_SRC_UPDATE)`: CTL1 MMC = UPDATE,
    /// so the timer also presents UPDATE on TRGO (the second coexisting trigger expression, HP-5).
    /// The injected group selects either CH3 or TRGO via its ETSIC field (the ADC side, T8); this is
    /// the timer side of the coupling.
    pub fn configure_trigger(
        &self,
        trigger_compare: u16,
        oc_mode: OcMode,
        ch_enable: bool,
        trgo_src: TrgoSource,
    ) {
        // CH3 output-compare mode + compare-shadow enable, in CHCTL1[15:8] (the high half).
        let (chctl, half_shift) = self.chctl_half(3);
        // MS = 0 (output mode).
        chctl.modify(0b11 << half_shift, 0);
        // COMCTL[6:4] = the configured trigger output-compare mode (no baked PWM0).
        chctl.modify(0x70 << half_shift, (oc_mode.comctl() << 4) << half_shift);
        // COMSEN (compare shadow enable).
        chctl.modify(
            OC_SHADOW_ENABLE << half_shift,
            OC_SHADOW_ENABLE << half_shift,
        );
        // CH3CV = the trigger compare value (near CAR-1; the per-cycle re-arm rewrites it via the
        // handle). CH3CV = CH0CV + 4*3.
        Reg32::new(self.base, CH0CV + 4 * 3).write(u32::from(trigger_compare));
        // CH3 output enable (CH3EN, CHCTL2 bit at 4*3): the application's explicit choice (no baked
        // CH3EN=off). The compare event drives the internal trigger regardless of CH3EN, so the
        // reference leaves CH3EN OFF and does not touch CHCTL2. We only write CHCTL2 when ENABLING
        // the CH3 output (so the disabled case leaves CHCTL2 untouched, matching the SPL trigger
        // recipe which programs no CH3 pin).
        if ch_enable {
            let ch3_shift = 4 * 3;
            self.chctl2()
                .modify(CCX_EN << ch3_shift, CCX_EN << ch3_shift);
        }
        // TRGO master-mode select = the configured source (no baked UPDATE).
        self.ctl1()
            .modify(CTL1_MMC, (trgo_src.mmc() << 4) & CTL1_MMC);
    }

    /// `timer_init` for the advanced timer: PSC, the alignment (DIR + CAM) in CTL0, CAR, the CKDIV
    /// field in CTL0, CREP, then the SWEVG UPG update-generate that latches the shadows. The
    /// alignment, clock-division, and repetition counter come from `cfg` (no baked CAM=2 / CKDIV /2
    /// / CREP=0).
    fn timer_init(&self, cfg: &PwmConfig) {
        // PSC = prescaler (the SPL assigns it directly).
        self.psc().write(u32::from(cfg.prescaler));
        // CTL0: clear DIR|CAM, set the configured alignment (DIR + CAM sub-mode).
        self.ctl0()
            .modify(CTL0_DIR | CTL0_CAM, cfg.align.ctl0_bits());
        // CAR = period (the auto-reload; the SPL assigns it directly).
        self.car().write(u32::from(cfg.period));
        // CTL0: clear CKDIV, set the configured dead-time / sampling clock divider.
        self.ctl0()
            .modify(CTL0_CKDIV, (cfg.ckdiv.ckdiv_code() << 8) & CTL0_CKDIV);
        // CREP = repetition counter (the SPL assigns it directly).
        self.crep().write(u32::from(cfg.crep));
        // SWEVG UPG: generate an update event to latch the shadowed PSC/CAR.
        self.swevg().write(SWEVG_UPG);
    }

    /// `timer_channel_output_config` for channel `n` (0..2): enable the main + complementary
    /// outputs, set the per-channel polarity, and set the idle state, all positioned by `4*n`
    /// (CHCTL2) / `2*n` (CTL1). `polarity == true` inverts the **complementary (low-side, CHxN)**
    /// output polarity to active-low while the main (high-side, CHx) stays active-high (the
    /// reference inverts the low-side so the bridge idles safe); `idle` sets the disarmed/fault
    /// idle level HIGH on both outputs of the pair.
    ///
    /// The main + complementary outputs are ENABLED here (CHxEN / CHxNEN). They only reach the pins
    /// once MOE is set ([`arming::ArmGate::arm`]); with MOE clear the timer counts
    /// and the compare event toggles the internal channel, but no current flows. This is the
    /// disarmed-but-configured state the SAFETY section calls electrically safe to scope.
    fn channel_output_config(&self, n: u32, polarity: bool, idle_high: bool, idle_high_n: bool) {
        let shift = 4 * n;
        let mut val = CCX_EN | CCXN_EN;
        if polarity {
            // Inverted polarity (active-low) on the complementary (low-side) output only; the
            // high-side main output stays active-high. Matches the reference's inverted low side.
            val |= CCXN_P_LOW;
        }
        self.chctl2()
            .modify(CHCTL2_CHAN_FIELDS << shift, val << shift);

        // Mode-select MS = 0 (output mode): clear the channel's MS field in CHCTL0/CHCTL1.
        let (chctl, half_shift) = self.chctl_half(n);
        chctl.modify(0b11 << half_shift, 0);

        // Per-side idle state in CTL1 (ISOx / ISOxN): the level each disarmed output drives. The
        // main (high-side) and complementary (low-side) idle levels are independent (superseded
        // audit 2.6; the reference idles N-side per its safe-bridge convention).
        let iso_shift = 8 + 2 * n;
        let mut iso = 0;
        if idle_high {
            iso |= ISO_HIGH;
        }
        if idle_high_n {
            iso |= ISON_HIGH;
        }
        self.ctl1()
            .modify(CTL1_ISO_CHAN_FIELDS << iso_shift, iso << iso_shift);
    }

    /// `timer_channel_output_mode_config(PWM_MODE0)` for channel `n`: set the COMCTL field of the
    /// channel's half to PWM mode 0.
    fn channel_output_mode_pwm0(&self, n: u32) {
        let (chctl, half_shift) = self.chctl_half(n);
        // Clear then set the COMCTL[6:4] (and the surrounding MS/COMSEN are set elsewhere); here
        // just the 3-bit compare-mode field.
        chctl.modify(0x70 << half_shift, OC_MODE_PWM0 << half_shift);
    }

    /// `timer_channel_output_shadow_config(ENABLE)` for channel `n`: set the compare-shadow-enable
    /// bit (COMSEN) of the channel's half so a duty write is buffered to the next update.
    fn channel_output_shadow_enable(&self, n: u32) {
        let (chctl, half_shift) = self.chctl_half(n);
        chctl.modify(
            OC_SHADOW_ENABLE << half_shift,
            OC_SHADOW_ENABLE << half_shift,
        );
    }

    /// `timer_channel_output_pulse_value_config`: write channel `n`'s compare value (CHnCV). Used
    /// for the zero initial duty at bring-up; the per-cycle duties go through the handle.
    fn channel_compare(&self, n: u32, pulse: u16) {
        Reg32::new(self.base, CH0CV + 4 * n).write(u32::from(pulse));
    }

    /// `timer_break_config`: write the whole CCHP dead-time / break / off-state / protect word in
    /// one assignment, exactly as the SPL does (it builds the value by ORing the fields and assigns
    /// CCHP directly, NOT a RMW). The reference enables the run-mode and idle-mode off-states (so a
    /// disabled / disarmed channel drives its configured idle) and DISABLES break; `brk_enabled`
    /// expresses enabling it as a hardware kill, with `brk_high` selecting the active level.
    ///
    /// POEN (MOE) is NOT part of this word: timer_break_config never touches POEN, and neither do
    /// we. The bridge stays disarmed. PROT is left at 0 (protect off), matching the reference.
    fn break_config(&self, dead_time: u8, brk_enabled: bool, brk_high: bool) {
        let mut word = (u32::from(dead_time) & CCHP_DTCFG) | CCHP_ROS | CCHP_IOS;
        if brk_enabled {
            word |= CCHP_BRKEN;
            if brk_high {
                word |= CCHP_BRKP_HIGH;
            }
        }
        // OAEN (output automatic enable) is left disabled: arming is the explicit ArmGate step, not
        // an automatic re-enable after a break event. PROT off. POEN untouched (disarmed).
        self.cchp().write(word);
    }

    /// The underlying base address. HAL-internal only: the application reaches the timer through the
    /// handle / arm gate / counter methods, never a raw base.
    #[inline]
    pub(crate) const fn base(&self) -> u32 {
        self.base
    }

    /// The per-cycle PWM handle for this configured timer (the resolve-once compare + trigger writer).
    /// The base stays PRIVATE to the HAL: the application drives PWM only through this handle.
    #[inline]
    pub fn handle(&self) -> PwmHandle {
        PwmHandle::new(self.base, self.period)
    }

    /// The MOE arming gate for this configured timer (the SOLE MOE writer; see the SAFETY notes). A
    /// deliberately separate object from the per-cycle handle, also built from the HAL-private base.
    #[inline]
    pub fn arm_gate(&self) -> ArmGate {
        ArmGate::new(self.base)
    }

    /// Start the timer counter (CTL0 CEN). The bring-up ([`Self::configure`]) deliberately leaves the
    /// counter STOPPED; the application starts it here once ready. SAFE while disarmed: with MOE clear
    /// the counter runs and compare events toggle the internal channels, but no output reaches the
    /// gate pins, so no current flows until [`ArmGate::arm`].
    #[inline]
    pub fn enable_counter(&self) {
        self.ctl0().modify(CTL0_CEN, CTL0_CEN);
    }

    /// Stop the timer counter (clear CTL0 CEN). With disarming (MOE clear) this fully stops the bridge.
    #[inline]
    pub fn disable_counter(&self) {
        self.ctl0().modify(CTL0_CEN, 0);
    }

    // --- register accessors -------------------------------------------------------------------

    /// CHCTL register + the field shift for channel `n`: CH0/CH1 in CHCTL0 (shift 0 / 8), CH2/CH3
    /// in CHCTL1 (shift 0 / 8). The bridge channels are 0..2; the trigger CH3 (T6) is in CHCTL1[15:8].
    #[inline]
    fn chctl_half(&self, n: u32) -> (Reg32, u32) {
        if n < 2 {
            (Reg32::new(self.base, CHCTL0), 8 * n)
        } else {
            (Reg32::new(self.base, CHCTL1), 8 * (n - 2))
        }
    }

    #[inline]
    fn ctl0(&self) -> Reg32 {
        Reg32::new(self.base, CTL0)
    }
    #[inline]
    fn ctl1(&self) -> Reg32 {
        Reg32::new(self.base, CTL1)
    }
    #[inline]
    fn swevg(&self) -> Reg32 {
        Reg32::new(self.base, SWEVG)
    }
    #[inline]
    fn chctl2(&self) -> Reg32 {
        Reg32::new(self.base, CHCTL2)
    }
    #[inline]
    fn psc(&self) -> Reg32 {
        Reg32::new(self.base, PSC)
    }
    #[inline]
    fn car(&self) -> Reg32 {
        Reg32::new(self.base, CAR)
    }
    #[inline]
    fn crep(&self) -> Reg32 {
        Reg32::new(self.base, CREP)
    }
    #[inline]
    fn cchp(&self) -> Reg32 {
        Reg32::new(self.base, CCHP)
    }
}

// --- TIMER0 register offsets for the per-cycle handle + arming gate (identical on both families) -
//
// Confirmed against the GD SPL peripheral headers (gd32f10x_timer.h / gd32f1x0_timer.h): the
// advanced-timer register block is the same offsets on F10x and F1x0, so one model parameterised by
// base (data, from the AddrTable). The per-cycle path touches the compare-value registers each cycle.

/// TIMER0 channel-0 capture/compare value register (CH0CV), the channel-0 high-side duty.
pub(crate) const TIMER_CH0CV: u32 = 0x34;
/// TIMER0 channel-1 capture/compare value register (CH1CV).
pub(crate) const TIMER_CH1CV: u32 = 0x38;
/// TIMER0 channel-2 capture/compare value register (CH2CV).
pub(crate) const TIMER_CH2CV: u32 = 0x3C;
/// TIMER0 channel-3 capture/compare value register (CH3CV), the ADC-trigger compare.
pub(crate) const TIMER_CH3CV: u32 = 0x40;
/// TIMER0 counter auto-reload register (CAR/ARR), the PWM period; the duty clamp references it.
#[allow(dead_code)]
pub(crate) const TIMER_CAR: u32 = 0x2C;
/// TIMER0 complementary-channel protection register (CCHP), which holds MOE (bit 15). Owned by the
/// [`arming`] layer ONLY; the per-cycle handle never names this offset.
pub(crate) const TIMER_CCHP: u32 = 0x44;
/// MOE (main output enable) bit in CCHP (`TIMER_CCHP_POEN`, bit 15).
pub(crate) const CCHP_MOE: u32 = 1 << 15;

/// The advanced-timer complementary-PWM capability (M3 T5 fills the bodies). SPEC.md: configure
/// center-aligned PWM at a given period/prescaler, three complementary channel pairs with
/// dead-time, polarity/idle, optional break, and the MOE gate, plus the ADC-trigger compare
/// channel; per cycle set the three duties and re-arm the trigger compare. MOE arming is NOT here
/// (it is in [`arming`]).
pub trait ComplementaryPwm {
    /// The concrete per-cycle handle this config resolves into (DECISIONS.md #4).
    type Handle: Copy;

    /// Configure the advanced timer for the complementary bridge from the [`Chip`] (base + selector)
    /// and the code-level [`PwmConfig`] and return the resolve-once handle. Runs ONCE at bring-up;
    /// the selectors are resolved into the handle. Leaves MOE OFF (outputs disarmed): arming is a
    /// separate, deliberate [`arming`] call.
    fn configure(&self, chip: &Chip, cfg: &PwmConfig) -> Result<Self::Handle, BringUpError>;
}

/// The advanced-timer complementary-PWM controller (resolve-once config object): a timer that
/// implements [`ComplementaryPwm`]. Its [`ComplementaryPwm::configure`] runs the timer bring-up
/// ([`PwmTimer`]) and resolves the four compare offsets + the period once into a
/// [`PwmHandle`]. MOE arming is NOT here (the [`arming::ArmGate`] is built separately from the same
/// base).
///
/// (Was `PwmConfig`; renamed to `PwmController` in the descriptor-rework so the code-level
/// [`crate::config::PwmConfig`] application config can take that name. This object carries the
/// resolved base + selectors via the [`Chip`]; the config it consumes is the behavior.)
///
/// The base is resolved at `configure` time from the [`Chip`] for the config's timer label, with the
/// advanced-timer-window check; a base outside the window (or a non-timer label, or a missing base)
/// surfaces as a [`crate::error::DescriptorError`] / [`PwmError`]. The arming gate is obtained from
/// the configured [`PwmTimer`] ([`PwmTimer::arm_gate`]); the base stays internal to the HAL.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PwmController;

impl PwmController {
    /// A controller that resolves its timer base from the [`Chip`] at `configure` time.
    #[inline]
    pub const fn new() -> Self {
        PwmController
    }
}

impl ComplementaryPwm for PwmController {
    type Handle = PwmHandle;

    /// Run the advanced-timer bring-up (alignment/ARSE/CREP/CKDIV/per-side idle from `cfg`, three
    /// complementary pairs, dead-time, break) leaving MOE OFF, then resolve the four compare
    /// offsets + the period once into a [`PwmHandle`]. No per-call branch in the resulting handle.
    ///
    /// The CH3 ADC-trigger compare + the TRGO master-mode are a separate
    /// [`PwmTimer::configure_trigger`] step (so the channel-config golden stays CH3-untouched); the
    /// integrated per-cycle-path bring-up runs both.
    fn configure(&self, chip: &Chip, cfg: &PwmConfig) -> Result<Self::Handle, BringUpError> {
        let timer = PwmTimer::configure(chip, cfg).map_err(BringUpError::Descriptor)?;
        Ok(PwmHandle::new(timer.base(), cfg.period))
    }
}

/// The resolve-once complementary-PWM per-cycle handle (DECISIONS.md #4).
///
/// `Copy`, concrete, no `dyn`: it holds the resolved [`Reg32`] accessors for the four compare
/// registers (the three channel duties + the trigger compare) and the period for the duty clamp. The
/// per-cycle methods ([`Self::set_duties`], [`Self::rearm_trigger`]) write straight to those
/// resolved registers with no descriptor lookup and no branch. It holds NO accessor for CCHP/MOE
/// (the arming gate is [`arming`]'s, not the handle's, a SAFETY invariant).
#[derive(Debug, Clone, Copy)]
pub struct PwmHandle {
    /// CH0CV / CH1CV / CH2CV accessors (the three channel high-side duties), resolved once.
    ch_cv: [Reg32; MAX_PWM_CHANNELS],
    /// CH3CV accessor (the ADC-trigger compare), resolved once.
    trig_cv: Reg32,
    /// CHCTL2 accessor (per-channel output enable), resolved once. NOT CCHP/MOE: this gates which
    /// channel outputs are driven, it cannot arm the bridge.
    chctl2: Reg32,
    /// The PWM period (CAR/ARR), used to clamp/validate duties so a compare never exceeds it.
    period: u16,
}

impl PwmHandle {
    /// Construct the handle from a resolved timer base + period. HAL-internal (used by
    /// [`PwmTimer::handle`] and host tests); the application gets a handle from a
    /// configured `PwmTimer`, never by supplying a base. Resolves the compare + CHCTL2 accessors once;
    /// holds no MOE accessor.
    #[inline]
    pub(crate) fn new(timer_base: u32, period: u16) -> Self {
        Self {
            ch_cv: [
                Reg32::new(timer_base, TIMER_CH0CV),
                Reg32::new(timer_base, TIMER_CH1CV),
                Reg32::new(timer_base, TIMER_CH2CV),
            ],
            trig_cv: Reg32::new(timer_base, TIMER_CH3CV),
            chctl2: Reg32::new(timer_base, CHCTL2),
            period,
        }
    }

    /// Per-cycle: write the three channel duties to CH0CV/CH1CV/CH2CV. The ONLY per-cycle PWM write
    /// surface besides [`Self::rearm_trigger`]. Resolve-once: no descriptor lookup, no branch. A
    /// duty above the period is [`PwmError::DutyOutOfRange`] (it would never match in a
    /// center-aligned count). MOE is untouched (this cannot arm the bridge).
    #[inline]
    pub fn set_duties(&self, duties: [u16; MAX_PWM_CHANNELS]) -> Result<(), PwmError> {
        for &d in &duties {
            if d > self.period {
                return Err(PwmError::DutyOutOfRange);
            }
        }
        for (i, &d) in duties.iter().enumerate() {
            self.ch_cv[i].write(u32::from(d));
        }
        Ok(())
    }

    /// Per-cycle: re-arm the ADC-trigger compare (CH3CV). The reference re-writes this every PWM
    /// period so the injected sample stays at the PWM centre. Wired to the actual trigger channel
    /// in T7; the write surface is fixed here. MOE is untouched.
    #[inline]
    pub fn rearm_trigger(&self, compare: u16) -> Result<(), PwmError> {
        if compare > self.period {
            return Err(PwmError::DutyOutOfRange);
        }
        self.trig_cv.write(u32::from(compare));
        Ok(())
    }

    /// The configured PWM period (CAR/ARR). Exposed so the control crate / arming layer can size
    /// duties; read-only.
    #[inline]
    pub const fn period(&self) -> u16 {
        self.period
    }

    /// Per-channel OUTPUT ENABLE: enable or disable each channel pair's outputs (CHxEN + CHxNEN).
    ///
    /// This is the raw silicon capability, NOT a motor concept: the advanced timer can gate each
    /// channel's output on or off independently. Disabling a channel takes both its transistors off
    /// (the channel floats); enabling drives it per its compare value and the dead-time. A higher
    /// layer (the `control` crate) uses this to float a channel for block commutation, but the HAL
    /// neither names nor knows "commutation".
    ///
    /// Written as ONE read-modify-write touching only the enable bits (CHxEN bit 0 / CHxNEN bit 2 of
    /// each channel's CHCTL2 field), so the per-channel polarity set at bring-up is preserved. MOE is
    /// never touched: this gates outputs, it cannot ARM the bridge (arming is [`arming::ArmGate`]'s
    /// alone, the SAFETY invariant). The bring-up enables all three; call this only to change which
    /// channels are driven.
    #[inline]
    pub fn set_channel_outputs(&self, enabled: [bool; MAX_PWM_CHANNELS]) {
        let chan_en = CCX_EN | CCXN_EN;
        let mut value = 0u32;
        let mut mask = 0u32;
        for (n, &on) in enabled.iter().enumerate() {
            let shift = 4 * n as u32;
            mask |= chan_en << shift;
            if on {
                value |= chan_en << shift;
            }
        }
        self.chctl2.modify(mask, value);
    }
}

/// The safety / arming layer (DECISIONS.md #4 + SPEC.md SAFETY). MOE (the main-output-enable arming
/// gate) is owned HERE, not on the per-cycle [`PwmHandle`], so a control-loop bug cannot energize a
/// disarmed bridge. The arming primitive is a separate, deliberately distinct call.
///
/// This is the boundary scaffold (T1); the body is a thin stub. The reference firmware confirms the
/// shape: MOE (`timer_primary_output_config`) is owned by the rider-power state machine and is
/// cleared on every latched fault, while the 16 kHz ISR only writes the compare values. The
/// disarm-on-fault path and the software safe-disarm-before-halt are finalized in T4/T10 (HP-6).
pub mod arming {
    use super::{CCHP_MOE, TIMER_CCHP};
    use crate::reg::Reg32;

    /// The MOE arming gate for an advanced timer (the only MOE writer). Distinct from the per-cycle
    /// [`super::PwmHandle`]; holds the CCHP accessor the handle deliberately does not.
    #[derive(Debug, Clone, Copy)]
    pub struct ArmGate {
        cchp: Reg32,
    }

    impl ArmGate {
        /// Construct the arming gate from the resolved timer base. HAL-internal: the application gets
        /// its arm gate from [`crate::timer::PwmTimer::arm_gate`] (base private), not by base.
        #[inline]
        pub(crate) fn new(timer_base: u32) -> Self {
            Self {
                cchp: Reg32::new(timer_base, TIMER_CCHP),
            }
        }

        /// Arm the bridge: set MOE so the complementary outputs reach the pins. A deliberate,
        /// distinct call (NOT a per-cycle handle method). SAFETY: only call under the rider-power
        /// state machine with current limiting / a controlled bench setup; see the SAFETY section.
        #[inline]
        pub fn arm(&self) {
            self.cchp.modify(CCHP_MOE, CCHP_MOE);
        }

        /// Disarm the bridge: clear MOE so the outputs drop to their configured idle state. The
        /// software safe-disarm used on a latched fault and before any CPU halt with the bus
        /// energized.
        #[inline]
        pub fn disarm(&self) {
            self.cchp.modify(CCHP_MOE, 0);
        }
    }
}

/// Compile-time guard: the channel count this module loops over matches the descriptor's.
const _: () = assert!(MAX_PWM_CHANNELS == 3);

#[cfg(test)]
mod tests;
