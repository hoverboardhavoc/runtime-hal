//! SysTick-based blocking delay implementing the `embedded-hal` 1.0 [`DelayNs`] trait.
//!
//! [`Delay`] owns the Cortex-M SysTick peripheral and busy-waits for a requested duration by
//! converting the time to core clock cycles and spinning until the down-counter wraps. It uses the
//! processor clock as the SysTick source ([`SystClkSource::Core`]), so the conversion is just
//! `cycles = time * core_clock`. This is the same technique `cortex_m::delay::Delay` uses; it is
//! reproduced here so the HAL exposes a `DelayNs` implementer without pulling in that type's API.
//!
//! # Clock frequency
//!
//! [`Delay::new`] takes the actual core clock the chip is running at in Hz. This MUST match the
//! real clock: the reset internal RC oscillator (IRC8M) is 8 MHz, so an application that has NOT
//! brought up the PLL passes `8_000_000`. An application that raised the clock with
//! [`crate::configure_tree`] passes that target frequency (e.g. the 72 MHz reference tree's
//! `sysclk_hz`). Passing the wrong value scales every delay by the ratio of the wrong to the right
//! frequency, so a `Delay` built at 8 MHz on a chip actually running at 72 MHz delays nine times
//! too long.
//!
//! [`DelayNs`]: embedded_hal::delay::DelayNs
//! [`SystClkSource::Core`]: cortex_m::peripheral::syst::SystClkSource::Core

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use embedded_hal::delay::DelayNs;

/// SysTick's reload register is 24-bit, so a single timed interval can be at most this many core
/// cycles. Longer delays are spun in repeated chunks no larger than this.
const SYSTICK_RELOAD_MAX: u32 = 0x00FF_FFFF;

/// A blocking delay driven by the Cortex-M SysTick down-counter, clocked from the processor clock.
///
/// Construct it with [`Delay::new`], passing the actual core clock frequency (see the module note),
/// then use the [`DelayNs`] methods `delay_ns` / `delay_us` / `delay_ms`. Call [`Delay::free`] to
/// release SysTick when done.
///
/// [`DelayNs`]: embedded_hal::delay::DelayNs
pub struct Delay {
    syst: SYST,
    sysclk_hz: u32,
}

impl Delay {
    /// Build a `Delay` that owns SysTick and busy-waits against `sysclk_hz`.
    ///
    /// Configures SysTick to count from the processor clock ([`SystClkSource::Core`]) and stores the
    /// frequency for the time-to-cycles conversion. `sysclk_hz` MUST be the actual core clock the
    /// chip is running at: the reset IRC8M is 8 MHz, so pass `8_000_000` before any PLL bring-up; if
    /// the application raised the clock via [`crate::configure_tree`], pass that frequency. Getting
    /// it wrong scales all delays by the frequency ratio.
    ///
    /// [`SystClkSource::Core`]: cortex_m::peripheral::syst::SystClkSource::Core
    pub fn new(mut syst: SYST, sysclk_hz: u32) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        Delay { syst, sysclk_hz }
    }

    /// Release the SysTick peripheral, consuming the `Delay`.
    pub fn free(self) -> SYST {
        self.syst
    }

    /// Busy-wait for `cycles` core clock cycles, spinning in 24-bit-bounded chunks.
    ///
    /// Each chunk sets the reload to `chunk - 1` (the counter counts `reload + 1` cycles per wrap),
    /// clears the current value, enables the counter, polls `has_wrapped`, then disables it.
    fn wait_cycles(&mut self, mut cycles: u64) {
        while cycles > 0 {
            let chunk = if cycles > SYSTICK_RELOAD_MAX as u64 {
                SYSTICK_RELOAD_MAX
            } else {
                cycles as u32
            };
            // A reload of N counts N+1 cycles (it wraps through zero), so program chunk - 1. A chunk
            // of 1 reloads 0, which still produces one wrap.
            self.syst.set_reload(chunk - 1);
            self.syst.clear_current();
            self.syst.enable_counter();
            while !self.syst.has_wrapped() {}
            self.syst.disable_counter();
            cycles -= chunk as u64;
        }
    }
}

/// Convert a duration in nanoseconds to core clock cycles at `hz`, rounding up so a delay is never
/// shorter than requested.
///
/// Pure arithmetic (no SysTick access) so it is host-testable. Done in `u64` to avoid overflow:
/// `ns * hz` can exceed `u32`. Rounds up because a busy-wait that is one cycle short of the request
/// is a worse failure than one that is one cycle long.
///
/// Example: 1_000_000 ns at 8 MHz is `1_000_000 * 8_000_000 / 1_000_000_000 = 8000` cycles; 1 ms
/// (1_000_000 ns) at 72 MHz is 72_000 cycles.
pub fn cycles_for_ns(ns: u64, hz: u32) -> u64 {
    // (ns * hz + (1e9 - 1)) / 1e9, ceiling division so we never under-delay.
    let numerator = ns.saturating_mul(hz as u64);
    numerator.div_ceil(1_000_000_000)
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = cycles_for_ns(ns as u64, self.sysclk_hz);
        self.wait_cycles(cycles);
    }

    fn delay_us(&mut self, us: u32) {
        // us * hz / 1e6, in u64 to avoid overflow, ceiling so we never under-delay.
        let cycles = (us as u64)
            .saturating_mul(self.sysclk_hz as u64)
            .div_ceil(1_000_000);
        self.wait_cycles(cycles);
    }

    fn delay_ms(&mut self, ms: u32) {
        // ms * hz / 1e3, in u64 to avoid overflow, ceiling so we never under-delay.
        let cycles = (ms as u64)
            .saturating_mul(self.sysclk_hz as u64)
            .div_ceil(1_000);
        self.wait_cycles(cycles);
    }
}

#[cfg(test)]
mod tests {
    use super::cycles_for_ns;

    #[test]
    fn one_millisecond_at_8mhz_is_8000_cycles() {
        // 1_000_000 ns at 8 MHz.
        assert_eq!(cycles_for_ns(1_000_000, 8_000_000), 8_000);
    }

    #[test]
    fn one_millisecond_at_72mhz_is_72000_cycles() {
        // 1_000_000 ns at 72 MHz.
        assert_eq!(cycles_for_ns(1_000_000, 72_000_000), 72_000);
    }

    #[test]
    fn one_microsecond_at_8mhz_is_8_cycles() {
        assert_eq!(cycles_for_ns(1_000, 8_000_000), 8);
    }

    #[test]
    fn rounds_up_to_never_underdelay() {
        // 1 ns at 8 MHz is 0.008 cycles exactly; ceiling gives 1 (never round down to 0).
        assert_eq!(cycles_for_ns(1, 8_000_000), 1);
        // 125 ns at 8 MHz is exactly 1 cycle (8e6 / 8e6 == 1).
        assert_eq!(cycles_for_ns(125, 8_000_000), 1);
        // 126 ns at 8 MHz is 1.008 cycles; ceiling gives 2.
        assert_eq!(cycles_for_ns(126, 8_000_000), 2);
    }

    #[test]
    fn zero_ns_is_zero_cycles() {
        assert_eq!(cycles_for_ns(0, 72_000_000), 0);
    }

    #[test]
    fn large_value_does_not_overflow() {
        // 4_000_000_000 ns (~4 s) at 72 MHz is 288_000_000 cycles; the u64 math holds it.
        assert_eq!(cycles_for_ns(4_000_000_000, 72_000_000), 288_000_000);
    }
}
