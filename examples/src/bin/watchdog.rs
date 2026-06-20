//! Free (independent) watchdog on either board: run the watchdog while blinking a "being fed" liveness
//! LED, with a button that suppresses feeding to trigger (and then detect, on the next boot) a reset.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It demonstrates the HAL's `FreeWatchdog`:
//! the FWDGT / IWDG independent watchdog clocked from the ~40 kHz LSI/IRC40K. The FWDGT
//! register block is identical on both families, so the same calls drive both; only the LSI enable
//! and the reset-cause flag (in the RCU) are touched per-family, and those have the same bit
//! positions on both, so this example is fully board-agnostic.
//!
//! # What it does
//!
//! 1. Read `was_watchdog_reset()` at boot. If the LAST reset was the watchdog (i.e. a previous run
//!    stopped feeding it and it reset the chip), play a DISTINCTIVE fast triple-blink on the UPPER LED
//!    (the reboot indicator) so the behaviour is observable on the bench, then clear the reset-cause.
//! 2. Start the watchdog with a GENEROUS timeout (8 s). Generous so a long init or an SWD halt
//!    cannot reset-loop the board (bench-safety, below).
//! 3. Run forever: feed the watchdog every pass and blink the LOWER LED (normal-operation liveness),
//!    UNLESS foot pad A (the suppress button) has been tapped, which latches "stop feeding": the LOWER
//!    LED then holds solid and the unfed watchdog resets the board, so the UPPER LED triple-blinks on
//!    the next boot.
//!
//! # Bench-safety (read before changing the loop)
//!
//! The default loop is SAFE: it feeds the watchdog every pass with a generous 8 s timeout, so a
//! normally-running board NEVER resets and stays easy to re-flash over SWD. This is deliberately NOT
//! a tight reset loop, which could make the board hard to re-flash.
//!
//! It also calls `FreeWatchdog::freeze_on_debug_halt()` once at boot: this sets the DBGMCU
//! `FWDGT_HOLD` debug-freeze bit so that while an attached debugger holds the core halted, the
//! watchdog is frozen and does not reset the board out from under the SWD session. The bit has no
//! effect when no debugger is attached, so it is harmless on a production board.
//!
//! It NEVER touches TIMER0 or the MOE / POEN arming gate.
//!
//! ## Triggering the reset (the suppress button)
//!
//! To SEE the watchdog reset the board, tap foot pad A (PA11, the same pad the `switches` example
//! reads). That latches suppression: feeding stops, the LOWER LED holds solid, and within ~8 s the
//! watchdog resets the chip; the next boot triple-blinks the UPPER LED. A fresh boot clears the latch,
//! so feeding resumes until the pad is tapped again. The board never sits in a tight reset loop on its
//! own, so it stays easy to re-flash.
//!
//! # Pins
//!
//! Upper LED = PB2 (watchdog-reboot indicator), lower LED = PB5 (normal-operation liveness). Both sit
//! behind the SELF_HOLD power latch (PB12), so the example drives PB12 high at boot to power their rail
//! (the same rail-enable the buzzer / usart_link examples use). Suppress button = foot pad A (PA11), a
//! pull-down input. None of PB2/PB5/PB12/PA11 is a JTAG-overlay pin, so the `free_jtag_pins()` call is
//! not strictly needed here (it is harmless and kept to match the other examples' boot shape). Runs on
//! the 8 MHz reset IRC8M clock (no PLL bring-up): the watchdog's LSI is a separate oscillator.
//!
//! The application defines NO fault handler: `detect_chip()` owns its discrimination BusFault.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use panic_halt as _;

use runtime_hal::{detect_chip, was_watchdog_reset, FreeWatchdog, WdgTimeout};

/// A generous watchdog timeout. Generous so a long init / an SWD halt cannot reset-loop the board.
const WDG_TIMEOUT_MS: u32 = 8_000;

/// The reset IRC8M core clock this example runs on (no PLL bring-up); the `sysclk_hz` for `Delay`.
const SYSCLK_HZ: u32 = 8_000_000;

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the Delay. detect_chip() uses
    // Peripherals::steal() for its probe, so this take() still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();

    // Detect the chip; a part matching neither family panics (panic_halt halts): fail-loud.
    let chip = detect_chip().unwrap();

    // Free the JTAG-overlay pins so PB3 can drive the LED (F10x: keeps SWD live; F1x0: no-op).
    chip.free_jtag_pins().ok();

    // PB3 (green LED) is on GPIOB. split() hands back named pins; reconfigure PB3 as a push-pull
    // output. PB3 lights without the SELF_HOLD rail, so no PB12 latch is needed here.
    let gpiob = chip.gpiob().unwrap().split();
    // Two-LED status: LOWER (PB5) = normal-operation liveness, UPPER (PB2) = watchdog-reboot indicator.
    // Both sit behind the SELF_HOLD power latch (PB12) on the bench board, so drive PB12 high to power
    // their rail (the same rail-enable the buzzer / usart_link examples use).
    let mut self_hold = gpiob.pb12.into_push_pull_output();
    let _ = self_hold.set_high();
    let mut led_normal = gpiob.pb5.into_push_pull_output();
    let mut led_reboot = gpiob.pb2.into_push_pull_output();
    let _ = led_normal.set_low();
    let _ = led_reboot.set_low();

    // Suppress button: foot pad A (PA11), a pull-down input (pin-high = pressed), the same pin the
    // `switches` example reads. Tapping it latches "stop feeding the watchdog" so the reset can be
    // triggered on demand, instead of the board running a tight no-feed reset loop.
    let gpioa = chip.gpioa().unwrap().split();
    let mut suppress_btn = gpioa.pa11.into_pull_down_input();

    // SysTick-backed blocking delay on the 8 MHz reset clock.
    let mut delay = runtime_hal::Delay::new(cp.SYST, SYSCLK_HZ);

    // 1. Was the LAST reset the watchdog? Read BEFORE clearing the cause. A detected chip always
    //    carries the RCU base, so this resolve is infallible in practice.
    let rcu = chip.rcu_base().unwrap();
    if was_watchdog_reset(rcu) {
        // Distinctive fast triple-blink: the previous run stopped feeding the watchdog and it reset
        // the board. Clear the cause so the next non-watchdog reset does not re-trigger this.
        for _ in 0..3 {
            let _ = led_reboot.set_high();
            delay.delay_ms(60);
            let _ = led_reboot.set_low();
            delay.delay_ms(60);
        }
        runtime_hal::clear_reset_cause(rcu);
        delay.delay_ms(400);
    }

    // Keep the watchdog frozen while a debugger holds the core halted, so an SWD session is not reset
    // out from under us. Harmless with no debugger attached. (Never touches TIMER0 / the arming gate.)
    FreeWatchdog::freeze_on_debug_halt();

    // 2. Start the free watchdog with the generous timeout. This enables + stabilises the LSI/IRC40K,
    //    then runs the five-write key recipe. The FWDGT base is resolved internally from the descriptor.
    let mut wdg = FreeWatchdog::start(&chip, WdgTimeout::from_millis(WDG_TIMEOUT_MS))
        .expect("watchdog start (LSI stable + FWDGT update)");

    // 3. Run forever. While pad A is untouched, feed the watchdog every pass and blink the normal LED
    //    (healthy). Tap pad A to LATCH suppression: feeding stops, the normal LED goes solid (the
    //    watchdog is now starving), and the unfed watchdog resets the board within ~8 s. On the next
    //    boot the reboot LED triple-blinks (the prior-watchdog-reset indicator), and since a fresh boot
    //    clears the latch, normal feeding resumes until pad A is tapped again. Default (pad untouched)
    //    NEVER resets, so the board stays easy to re-flash.
    let mut suppressed = false;
    loop {
        // Latch suppression on a press (pin-high = pad pressed).
        if suppress_btn.is_high().unwrap_or(false) {
            suppressed = true;
        }

        if suppressed {
            // Starving the watchdog: hold the normal LED solid as a visible "no longer feeding" state
            // and do NOT feed, so the watchdog resets the board within the timeout.
            let _ = led_normal.set_high();
        } else {
            // Healthy: feed every pass (well within the 8 s timeout) and blink the normal LED.
            wdg.feed();
            let _ = led_normal.toggle();
        }
        delay.delay_ms(150);
    }
}
