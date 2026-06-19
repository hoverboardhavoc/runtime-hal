//! Configurable-rate periodic SysTick tick (G-TICK), the cold-path outer-loop / cadence timebase.
//!
//! [`Timebase`] owns the Cortex-M SysTick peripheral and runs it in INTERRUPT mode: it programs the
//! reload register so SysTick fires its exception at a chosen `tick_hz`, clocked from the processor
//! clock ([`SystClkSource::Core`]). Each SysTick exception runs the HAL's [`crate::on_systick`],
//! which bumps a free-running tick count ([`crate::tick_count`]) and calls a registered tick handler
//! ([`crate::register_tick_handler`]). This is the cold-path analogue of the stock firmware's 250 Hz
//! `SysTick_Handler -> sched_tick` outer loop and the basis for any fixed cadence (scheduler,
//! telemetry, a buzzer tone toggle).
//!
//! # Reload math (the only part with host tests)
//!
//! SysTick counts `reload + 1` core cycles per wrap, so for a tick at `tick_hz` from a core running
//! at `sysclk_hz` the reload is `sysclk_hz / tick_hz - 1`. SysTick's reload register is 24-bit, so a
//! reload of `0x00FF_FFFF` or larger is rejected with [`TimebaseError::ReloadTooLarge`] (the exact
//! 24-bit limit the stock `scheduler.c` `SCHED_LOAD_MAX` check enforces). A `tick_hz` of 0, or one
//! above `sysclk_hz` (reload would be 0 or the divide underflows), is rejected with
//! [`TimebaseError::InvalidRate`]. [`reload_for`] is the pure arithmetic, host-tested like
//! `delay::cycles_for_ns`.
//!
//! # Conflict with [`crate::Delay`]
//!
//! SysTick is a single 24-bit down-counter. [`crate::Delay`] (blocking `DelayNs`) POLLS it; this
//! [`Timebase`] runs it in INTERRUPT mode. They cannot coexist: a program uses EITHER a `Delay` OR a
//! SysTick `Timebase`, never both from the same `SYST`. The type system already enforces this (both
//! consume `SYST` by value, so only one can hold it at a time). A firmware that needs both a blocking
//! delay AND a periodic tick must drive one of them from a basic/general timer instead (the
//! basic-timer `Timebase` variant the cold-path plan sketches), or busy-loop the delay.
//!
//! # Family independence
//!
//! SysTick is a Cortex-M core peripheral, identical on the F10x and F1x0, so there is NO GD register
//! path and no runtime family branch here. The SysTick exception slot is system vector 15, common to
//! both [`crate::descriptor::IrqLayout`] layouts (see `irq.rs`).
//!
//! [`SystClkSource::Core`]: cortex_m::peripheral::syst::SystClkSource::Core

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

/// SysTick's reload register is 24-bit: a reload value of this or larger does not fit.
const SYSTICK_RELOAD_MAX: u32 = 0x00FF_FFFF;

/// Failures configuring a SysTick [`Timebase`].
///
/// `#[non_exhaustive]` and additive (DECISIONS.md #5).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimebaseError {
    /// `tick_hz` was 0, or greater than `sysclk_hz` (the reload would be 0 or the divide would
    /// underflow): no valid SysTick reload exists for that rate at that clock.
    InvalidRate,
    /// The computed reload (`sysclk_hz / tick_hz - 1`) does not fit SysTick's 24-bit reload register
    /// (it is `>= 0x0100_0000`): the requested `tick_hz` is too slow for this `sysclk_hz`. Use a
    /// faster tick, a lower core clock, or a basic-timer timebase with a prescaler.
    ReloadTooLarge,
}

/// Compute the SysTick reload value for a tick at `tick_hz` from a core clock of `sysclk_hz`.
///
/// Pure arithmetic (no SysTick access) so it is host-testable, like [`crate::delay::cycles_for_ns`].
/// SysTick counts `reload + 1` cycles per wrap, so `reload = sysclk_hz / tick_hz - 1` (floor divide:
/// the real tick is at or slightly faster than requested, never slower). Returns
/// [`TimebaseError::InvalidRate`] when `tick_hz` is 0 or exceeds `sysclk_hz`, and
/// [`TimebaseError::ReloadTooLarge`] when the result does not fit the 24-bit reload register.
///
/// Example: a 250 Hz tick from a 72 MHz core is `72_000_000 / 250 - 1 = 287_999`; a 4 kHz tick from
/// the 8 MHz reset clock is `8_000_000 / 4_000 - 1 = 1_999`.
pub fn reload_for(sysclk_hz: u32, tick_hz: u32) -> Result<u32, TimebaseError> {
    if tick_hz == 0 || tick_hz > sysclk_hz {
        return Err(TimebaseError::InvalidRate);
    }
    // sysclk_hz / tick_hz >= 1 here (tick_hz <= sysclk_hz), so the subtraction does not underflow.
    let reload = sysclk_hz / tick_hz - 1;
    if reload > SYSTICK_RELOAD_MAX {
        return Err(TimebaseError::ReloadTooLarge);
    }
    Ok(reload)
}

/// A configurable-rate periodic tick driven by the Cortex-M SysTick exception (G-TICK).
///
/// Build it with [`Timebase::new`], passing the actual core clock and the desired tick rate; it
/// programs SysTick (reload, core clock source, TICKINT) and starts counting. Each tick runs the
/// HAL's [`crate::on_systick`] (count + registered handler). Call [`Timebase::stop`] /
/// [`Timebase::start`] to gate ticks and [`Timebase::free`] to release SysTick.
///
/// See the module docs for the [`crate::Delay`] conflict: SysTick hosts EITHER a `Delay` OR a
/// `Timebase`, not both.
pub struct Timebase {
    syst: SYST,
    reload: u32,
}

impl Timebase {
    /// Configure SysTick to fire its exception at `tick_hz` from a `sysclk_hz` core clock, and start
    /// it.
    ///
    /// Sets the reload to `sysclk_hz / tick_hz - 1` (see [`reload_for`]), selects the processor clock
    /// as the source ([`SystClkSource::Core`]), clears the current value, then enables the counter
    /// AND the tick interrupt (CTRL = ENABLE | TICKINT | CLKSOURCE, the stock SysTick recipe).
    ///
    /// `sysclk_hz` MUST be the actual core clock the chip is running at (the reset IRC8M is 8 MHz; a
    /// PLL-raised clock is its target), exactly as for [`crate::Delay::new`]: a wrong value scales
    /// the tick rate by the frequency ratio. Returns [`TimebaseError`] if no valid 24-bit reload
    /// exists for the requested rate.
    ///
    /// The firmware MUST have a SysTick exception that reaches [`crate::on_systick`] (a one-line
    /// `#[exception] fn SysTick() { runtime_hal::on_systick() }`, or the RAM vector table installed),
    /// and SHOULD register a tick handler (or poll [`crate::tick_count`]) BEFORE calling this, so the
    /// first tick is already routed.
    ///
    /// [`SystClkSource::Core`]: cortex_m::peripheral::syst::SystClkSource::Core
    pub fn new(mut syst: SYST, sysclk_hz: u32, tick_hz: u32) -> Result<Self, TimebaseError> {
        let reload = reload_for(sysclk_hz, tick_hz)?;
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(reload);
        syst.clear_current();
        let mut tb = Timebase { syst, reload };
        tb.start();
        Ok(tb)
    }

    /// Start (or restart) the tick: clear the current value, enable the tick interrupt, enable the
    /// counter. Idempotent.
    pub fn start(&mut self) {
        self.syst.clear_current();
        self.syst.enable_interrupt();
        self.syst.enable_counter();
    }

    /// Stop the tick: disable the counter and the tick interrupt. The reload is retained, so a later
    /// [`Timebase::start`] resumes at the same rate. Idempotent.
    pub fn stop(&mut self) {
        self.syst.disable_counter();
        self.syst.disable_interrupt();
    }

    /// The configured reload value (`sysclk_hz / tick_hz - 1`). Mostly for diagnostics / tests.
    pub fn reload(&self) -> u32 {
        self.reload
    }

    /// Stop the tick and release the SysTick peripheral, consuming the `Timebase`. After this, SYST
    /// is free for a [`crate::Delay`] again.
    pub fn free(mut self) -> SYST {
        self.stop();
        self.syst
    }
}

#[cfg(test)]
mod tests {
    use super::{reload_for, TimebaseError};

    #[test]
    fn two_fifty_hz_at_72mhz() {
        // The stock outer-loop cadence: 72_000_000 / 250 - 1 = 287_999.
        assert_eq!(reload_for(72_000_000, 250), Ok(287_999));
    }

    #[test]
    fn four_khz_at_8mhz() {
        // The buzzer example's tone toggle rate on the reset clock: 8_000_000 / 4_000 - 1 = 1_999.
        assert_eq!(reload_for(8_000_000, 4_000), Ok(1_999));
    }

    #[test]
    fn one_khz_at_8mhz() {
        assert_eq!(reload_for(8_000_000, 1_000), Ok(7_999));
    }

    #[test]
    fn floor_divide_when_not_exact() {
        // 8_000_000 / 3_000 = 2666 (floor), minus 1 = 2665; the real tick is slightly fast, never
        // slow.
        assert_eq!(reload_for(8_000_000, 3_000), Ok(2_665));
    }

    #[test]
    fn zero_rate_is_invalid() {
        assert_eq!(reload_for(8_000_000, 0), Err(TimebaseError::InvalidRate));
    }

    #[test]
    fn rate_above_clock_is_invalid() {
        assert_eq!(
            reload_for(8_000_000, 9_000_000),
            Err(TimebaseError::InvalidRate)
        );
    }

    #[test]
    fn rate_equal_to_clock_is_one_cycle() {
        // reload 0 counts exactly one cycle per wrap; the fastest valid tick.
        assert_eq!(reload_for(8_000_000, 8_000_000), Ok(0));
    }

    #[test]
    fn too_slow_a_tick_overflows_24_bits() {
        // 72_000_000 / 1 - 1 = 71_999_999 > 0x00FF_FFFF (16_777_215): does not fit.
        assert_eq!(
            reload_for(72_000_000, 1),
            Err(TimebaseError::ReloadTooLarge)
        );
    }

    #[test]
    fn largest_reload_that_still_fits() {
        // reload exactly 0x00FF_FFFF (16_777_215) fits: sysclk = 16_777_216, tick = 1.
        assert_eq!(reload_for(16_777_216, 1), Ok(0x00FF_FFFF));
        // One more cycle of clock pushes the reload to 0x0100_0000, which does not fit.
        assert_eq!(
            reload_for(16_777_217, 1),
            Err(TimebaseError::ReloadTooLarge)
        );
    }
}
