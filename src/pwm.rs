//! General single-channel PWM on a GENERAL-purpose timer (G3, the cold-path gap).
//!
//! [`PwmOut`] is a minimal, single-channel edge-aligned PWM on a GENERAL-purpose timer (the GD32
//! "General level0 timer" TIMERx, x=1,2), distinct from the advanced-timer complementary-PWM bridge
//! in [`crate::timer`]. It exists to drive a cold-path output (an LED, a buzzer tone) without ever
//! touching the motor bridge: it REFUSES the advanced timers (`Timer0`/`Timer7`) and never writes
//! the MOE/POEN gate, the break/dead-time word, or any complementary-output field. The point of G3
//! is the architecture-aware ROUTING the application does around it (see "Family routing" below),
//! not a hidden abstraction: a general timer's channel drives its pin as soon as the channel and the
//! counter are enabled, with NO arming step (that asymmetry from [`crate::timer`] is exactly the
//! advanced-vs-general timer difference).
//!
//! # One model, parameterised by base (no per-family register branch)
//!
//! The general timer's PWM register block (`CTL0`/`CHCTL0`/`CHCTL2`/`PSC`/`CAR`/`CHxCV`) has the
//! SAME offsets and the SAME basic-PWM field positions as the advanced TIMER0 in [`crate::timer`]
//! (verified against the GD32F1x0 User Manual 15.2 "General level0 timer (TIMERx, x=1,2)" register
//! definitions: CTL0 0x00, CHCTL0 0x18, CHCTL2 0x20, PSC 0x28, CAR 0x2C, CH0CV 0x34, CH1CV 0x38,
//! all matching `gd32f10x_timer.h` / `gd32f1x0_timer.h`), MINUS every bridge-only field (no CHxN, no
//! dead-time/CCHP, no break, no MOE/POEN). So this is ONE register model parameterised only by the
//! general-timer base (data, from [`crate::addr::AddrTable`]); there is no [`crate::descriptor`]
//! family selector here. The family difference is entirely in the PIN ROUTING, which the HAL leaves
//! to the application (it is genuinely architecture-specific; see [`crate::Chip::family`]).
//!
//! # Family routing (the visible F10x-vs-F1x0 difference the application owns)
//!
//! The G3 target is `TIMER1_CH1 -> PB3` (the green LED, which lights without the SELF_HOLD rail).
//! Getting that one channel onto that one pin is where the two families diverge:
//!
//! - **F1x0** ([`crate::Family::F1x0`]): one per-pin AF mux field. PB3's `AFSEL` nibble is set to
//!   **AF2** (`GD32F130xx Datasheet` Port B alternate-function summary: PB3 AF2 = `TIMER1_CH1`).
//!   That is the whole routing: [`crate::gpio::configure_af`] with
//!   [`crate::gpio::PinRole::GenTimerAfPushPull`].
//! - **F10x** ([`crate::Family::F10x`]): PB3 is JTDO after reset and TIMER1_CH1 is not on PB3 by
//!   default, so it takes THREE steps: (1) [`crate::Chip::free_jtag_pins`] to release PB3 from the
//!   JTAG-DP (keeping SWD), (2) [`crate::gpio::remap_timer1_partial1`] to set `AFIO_PCF0`'s
//!   `TIMER1_REMAP[9:8]` field to `01` (partial remap 1, which maps `TIMER1_CH1 / PB3`; GD32F10x
//!   User Manual 7.5.9), and (3) PB3's CRL nibble set to alternate-function push-pull
//!   ([`crate::gpio::configure_af`], where the F10x AF is implied by the nibble).
//!
//! The constraint that fixes the target on TIMER1 (not TIMER2): TIMER2's remap to PB4/PB5 needs a
//! 64/100/144-pin package (GD32F10x User Manual, TIMER alternate-function remapping notes), so on a
//! 48-pin GD32F103C8 TIMER2's channels are NOT reachable; TIMER1's partial-remap-1 to PB3 IS. See
//! [`crate::gpio::remap_timer1_partial1`]. The shared datapath ([`PwmOut::new`] + the duty setter)
//! is identical once the pin is routed.
//!
//! # embedded-hal trait
//!
//! [`PwmOut`] implements the embedded-hal 1.0 [`embedded_hal::pwm::SetDutyCycle`] trait: a cold-path
//! PWM IS a single-channel duty setter, which the trait expresses exactly (unlike the advanced
//! bridge, whose MOE/dead-time/trigger coupling the trait cannot express, which is why
//! [`crate::timer`] deliberately is NOT a `SetDutyCycle`). `max_duty_cycle()` is the period CAR.
//!
//! # Register model (general timer; identical on both families)
//!
//! | reg      | offset | what (basic-PWM subset only)                                          |
//! |----------|--------|-----------------------------------------------------------------------|
//! | `CTL0`   | `0x00` | CEN(0), DIR(4)=0 up-count, `CAM[6:5]`=0 edge-aligned, ARSE(7)          |
//! | `CHCTL0` | `0x18` | CH1: `CH1MS[9:8]`=0 output, `CH1COMCTL[14:12]`=PWM0, CH1COMSEN(11) shadow |
//! | `CHCTL2` | `0x20` | CH1EN(4) enable, CH1P(5)=0 active-high                                 |
//! | `PSC`    | `0x28` | prescaler                                                             |
//! | `CAR`    | `0x2C` | counter auto-reload (the PWM period = max duty)                        |
//! | `CH1CV`  | `0x38` | channel-1 compare (the duty); `CH0CV` 0x34 + 4*1                       |

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::error::PwmError;
use crate::reg::Reg32;

// --- register offsets (the basic-PWM subset; identical to the advanced timer's in `timer.rs`) ---

/// Control register 0 (`TIMERx_CTL0`), offset 0x00.
const CTL0: u32 = 0x00;
/// Channel control register 0 (`TIMERx_CHCTL0`), offset 0x18 (holds CH0 low half, CH1 high half).
const CHCTL0: u32 = 0x18;
/// Channel control register 2 (`TIMERx_CHCTL2`), offset 0x20 (channel enable + polarity).
const CHCTL2: u32 = 0x20;
/// Prescaler register (`TIMERx_PSC`), offset 0x28.
const PSC: u32 = 0x28;
/// Counter auto-reload register (`TIMERx_CAR`), offset 0x2C (the PWM period).
const CAR: u32 = 0x2C;
/// Channel-0 compare value (`TIMERx_CH0CV`), offset 0x34; channel `n` is `CH0CV + 4*n`.
const CH0CV: u32 = 0x34;

// --- CTL0 fields --------------------------------------------------------------------------------

/// Counter enable (`TIMERx_CTL0_CEN`), bit 0: starts the counter (and, with a channel enabled,
/// drives the pin; a general timer has no MOE gate).
const CTL0_CEN: u32 = 1 << 0;
/// Direction (`DIR`, bit 4): 0 = up-count. Edge-aligned PWM up-counts, so we CLEAR it.
const CTL0_DIR: u32 = 1 << 4;
/// Center-aligned mode select (`CAM[6:5]`): 0 = edge-aligned. We CLEAR it (edge-aligned, the
/// general-PWM case, distinct from the advanced bridge's center-aligned mode).
const CTL0_CAM: u32 = 0b11 << 5;
/// Auto-reload shadow enable (`ARSE`, bit 7): buffer CAR so a period change applies cleanly at the
/// next update. Set so the time base is shadowed like the SPL `timer_auto_reload_shadow_enable`.
const CTL0_ARSE: u32 = 1 << 7;

// --- CHCTL0 channel-1 output-compare fields (CH1 is the high half, [15:8]) ----------------------
//
// CH1 occupies CHCTL0[15:8]: CH1MS[9:8] (mode select; 0 = output), CH1COMCTL[14:12] (output-compare
// mode), CH1COMSEN(11) (compare shadow enable). The general timer's CHCTL0 layout is identical to
// the advanced timer's (GD32F1x0 User Manual 15.2 / `gd32*_timer.h`).

/// CH1 mode-select field (`CH1MS`), CHCTL0[9:8]. 0 = output mode.
const CHCTL0_CH1MS: u32 = 0b11 << 8;
/// CH1 output-compare control field (`CH1COMCTL`), CHCTL0[14:12].
const CHCTL0_CH1COMCTL: u32 = 0b111 << 12;
/// PWM mode 0 (`TIMER_OC_MODE_PWM0`, COMCTL = 0b110) positioned in CH1COMCTL[14:12].
const CHCTL0_CH1_PWM0: u32 = 0b110 << 12;
/// CH1 output-compare shadow enable (`CH1COMSEN`), CHCTL0 bit 11.
const CHCTL0_CH1COMSEN: u32 = 1 << 11;

// --- CHCTL2 channel-1 enable / polarity (CH1 field at shift 4*1 = 4) ----------------------------
//
// CH1 occupies CHCTL2[7:4]: CH1EN(4), CH1P(5), CH1NEN(6), CH1NP(7). The general PWM uses ONLY
// CH1EN (active-high, CH1P clear); it never touches the complementary CH1NEN/CH1NP bits (those are
// the bridge's, left at their reset 0).

/// CH1 capture/compare enable (`CH1EN`), CHCTL2 bit 4.
const CHCTL2_CH1EN: u32 = 1 << 4;
/// CH1 polarity (`CH1P`), CHCTL2 bit 5. 0 = active-high; cleared for normal polarity.
const CHCTL2_CH1P: u32 = 1 << 5;

/// The channel index this single-channel general PWM drives: TIMER1_CH1 (the green LED on PB3). A
/// const so the field math (`CH0CV + 4*CHANNEL`) reads against the documented channel.
const CHANNEL: u32 = 1;

/// A single-channel general-purpose-timer PWM output, resolved once to its general-timer base.
///
/// Holds the base and the configured period (CAR, also the max duty). Built by [`PwmOut::new`]
/// (which REFUSES an advanced-timer label); the per-use duty write goes through
/// [`embedded_hal::pwm::SetDutyCycle::set_duty_cycle`] (DECISIONS.md #4: resolve once into a
/// concrete `Copy` handle, the per-use path holds the raw base).
#[derive(Debug, Clone, Copy)]
pub struct PwmOut {
    base: u32,
    /// The configured period (CAR). Also the maximum duty (full on at `duty == period`).
    period: u16,
}

impl PwmOut {
    /// Bring up a single-channel edge-aligned PWM on the GENERAL timer `instance`'s CH1, at
    /// `freq_hz`, returning the configured [`PwmOut`] with the counter RUNNING at zero duty.
    ///
    /// `chip` supplies the timer base + clock path. `timer_clk_hz` is the timer's input clock (the
    /// caller passes it because the clock tree is an application decision, e.g. the 8 MHz reset
    /// IRC8M, not a HAL default). The period is `CAR = round(timer_clk_hz / freq_hz) - 1`, clamped
    /// to a 16-bit counter; `freq_hz == 0` or a period that would be 0 is rejected as
    /// [`PwmError::DutyOutOfRange`] (a degenerate period).
    ///
    /// This REFUSES the advanced timers: `instance` MUST be a general-timer label (`Timer1`), or it
    /// returns [`PwmError::BadTimerBase`]. This is the host-enforced guard that keeps the cold-path
    /// PWM off the motor bridge (it never touches TIMER0 or the MOE/POEN gate). The pin routing is
    /// the APPLICATION's job and is NOT done here (it is family-specific; see the module docs); the
    /// caller routes the pin to TIMER1_CH1 before or after this bring-up.
    ///
    /// Steps (the SPL `timer_init` + single-channel `timer_channel_output_*` subset, MINUS every
    /// bridge field):
    /// 1. PSC = 0, CAR = period, edge-aligned up-count (CTL0 DIR/CAM cleared), ARSE on.
    /// 2. CH1 output PWM mode 0 + compare shadow enable (CHCTL0), zero initial duty (CH1CV).
    /// 3. CH1 enable, active-high polarity (CHCTL2). NO complementary, NO break, NO MOE.
    /// 4. CEN: start the counter. With CH1 enabled the pin is driven immediately (no arming step).
    pub fn new(
        chip: &Chip,
        instance: PeriphLabel,
        freq_hz: u32,
        timer_clk_hz: u32,
    ) -> Result<PwmOut, PwmError> {
        // GUARD: refuse the advanced timers (and any non-general-timer label). The advanced bridge
        // is owned by `crate::timer` / the arming layer; this cold-path PWM must never reach it.
        if !instance.is_general_timer() {
            return Err(PwmError::BadTimerBase);
        }
        // Resolve + parse-check the base sits in the general-timer window.
        chip.descriptor()
            .addrs
            .check_general_timer_base(instance)
            .map_err(|_| PwmError::BadTimerBase)?;
        let base = chip.base(instance).map_err(|_| PwmError::BadTimerBase)?;

        let period = Self::period_for(freq_hz, timer_clk_hz)?;
        let dev = PwmOut { base, period };
        dev.configure(period);
        Ok(dev)
    }

    /// Compute the auto-reload `CAR` for `freq_hz` against `timer_clk_hz`: `CAR = round(clk/freq) - 1`,
    /// clamped to a 16-bit counter. A zero frequency, a zero timer clock, or a result that would not
    /// leave at least one count is [`PwmError::DutyOutOfRange`] (a degenerate period).
    fn period_for(freq_hz: u32, timer_clk_hz: u32) -> Result<u16, PwmError> {
        if freq_hz == 0 || timer_clk_hz == 0 {
            return Err(PwmError::DutyOutOfRange);
        }
        // round(clk/freq): integer rounding to the nearest count.
        let ticks = (timer_clk_hz as u64 + (freq_hz as u64) / 2) / freq_hz as u64;
        if ticks < 2 {
            // Fewer than two counts cannot express a duty: the frequency is too high for this clock.
            return Err(PwmError::DutyOutOfRange);
        }
        let car = ticks - 1; // CAR = period - 1.
        // Clamp to a 16-bit counter (the longest period the hardware can express).
        let car = if car > u16::MAX as u64 {
            u16::MAX as u64
        } else {
            car
        };
        Ok(car as u16)
    }

    /// Program the time base + CH1 PWM, zero the duty, and START the counter. Separated from
    /// [`PwmOut::new`] so the host tests can drive the register sequence directly.
    fn configure(&self, period: u16) {
        // 1. Time base: PSC = 0 (the caller picks the frequency via CAR against the timer clock),
        //    CAR = period, edge-aligned up-count (clear DIR + CAM), ARSE on.
        self.reg(PSC).write(0);
        self.reg(CAR).write(u32::from(period));
        self.reg(CTL0)
            .modify(CTL0_DIR | CTL0_CAM | CTL0_ARSE, CTL0_ARSE);

        // 2. CH1 output: PWM mode 0, mode-select = output (CH1MS clear), compare shadow enable.
        self.reg(CHCTL0).modify(
            CHCTL0_CH1MS | CHCTL0_CH1COMCTL | CHCTL0_CH1COMSEN,
            CHCTL0_CH1_PWM0 | CHCTL0_CH1COMSEN,
        );
        // Zero initial duty (CH1CV).
        self.compare(0);

        // 3. CH1 enable, active-high polarity (clear CH1P). No complementary, break, dead-time, MOE.
        self.reg(CHCTL2)
            .modify(CHCTL2_CH1EN | CHCTL2_CH1P, CHCTL2_CH1EN);

        // 4. Start the counter. A general timer drives the pin as soon as the channel + counter are
        //    enabled (no arming / MOE step).
        self.reg(CTL0).modify(CTL0_CEN, CTL0_CEN);
    }

    /// Write the CH1 compare value (the duty), `CH1CV = CH0CV + 4*1`. A single 32-bit store.
    #[inline]
    fn compare(&self, duty: u16) {
        Reg32::new(self.base, CH0CV + 4 * CHANNEL).write(u32::from(duty));
    }

    /// The configured period (CAR), which is also the maximum duty value.
    #[inline]
    pub const fn period(&self) -> u16 {
        self.period
    }

    /// The underlying general-timer base address.
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    #[inline]
    fn reg(&self, off: u32) -> Reg32 {
        Reg32::new(self.base, off)
    }
}

impl embedded_hal::pwm::ErrorType for PwmOut {
    type Error = PwmError;
}

impl embedded_hal::pwm::SetDutyCycle for PwmOut {
    /// The maximum duty value: the period (CAR). `duty == period` is full-on.
    #[inline]
    fn max_duty_cycle(&self) -> u16 {
        self.period
    }

    /// Set the CH1 duty (CH1CV). `duty` is clamped to the period by the trait's default helpers; a
    /// direct call here writes it as-is up to the period (a value above the period is clamped to the
    /// period so the channel stays full-on rather than never matching).
    #[inline]
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let d = if duty > self.period { self.period } else { duty };
        self.compare(d);
        Ok(())
    }
}

impl embedded_hal::pwm::Error for PwmError {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        // embedded-hal 1.0 pwm::ErrorKind has only the non-exhaustive `Other`; every PWM error
        // folds into it.
        embedded_hal::pwm::ErrorKind::Other
    }
}

#[cfg(test)]
mod tests;
