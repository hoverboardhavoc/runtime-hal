//! Drive a passive buzzer on PB9 with the G-TICK periodic SysTick interrupt: a tone + beep pattern.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It demonstrates the HAL's `Timebase` (G-TICK):
//! SysTick runs in INTERRUPT mode at a fixed `tick_hz`, and the main loop polls the HAL tick count
//! (`runtime_hal::tick_count()`) to (a) toggle PB9 every tick, making a square-wave tone, and (b)
//! count ticks for a beep envelope. There is NO `Delay` here: SysTick IS the timebase, and a
//! `Delay` would need the same SysTick down-counter (see the tradeoff below).
//!
//! # Tone and envelope
//!
//! The tick runs at 4000 Hz. Toggling PB9 on every tick produces a ~2 kHz square wave (two ticks =
//! one full period), audible on a PASSIVE (non-self-oscillating) buzzer or a small speaker. The
//! envelope is a 3-beep startup pattern, then silence: each beep is 200 ms of tone (800 ticks at
//! 4 kHz) followed by 200 ms of gap, three times, then PB9 is parked low forever.
//!
//! With an ACTIVE (self-oscillating) buzzer the per-tick toggle is irrelevant (the buzzer makes its
//! own tone whenever it is powered); driving PB9 high for the "on" window and low for the "off"
//! window would beep on/off the same way. This example keeps the passive-buzzer toggle so it works
//! on either buzzer type: a passive buzzer tones, an active buzzer just beeps.
//!
//! # SysTick vs Delay tradeoff
//!
//! `Timebase` (this example) and `runtime_hal::Delay` both want the single Cortex-M SysTick
//! down-counter, so they are mutually exclusive: `Delay` POLLS SysTick to busy-wait, while
//! `Timebase` runs it in INTERRUPT mode. A program uses one or the other from the same `SYST`. The
//! examples that blink an LED (`blinky`) use `Delay`; this one uses `Timebase` and
//! does its timing by counting ticks, with no blocking delay at all. A firmware that needs both a
//! blocking delay AND a periodic tick would put one of them on a basic/general timer instead.
//!
//! # Pin
//!
//! Buzzer = PB9 (the bench board buzzer pin). PB9 is NOT a JTAG-overlay pin, so freeing the JTAG
//! pins is not required for it; the example calls `free_jtag_pins()` anyway (harmless, a no-op on the
//! F1x0 and only touching PB3/PB4/PA15 on the F10x) to match the other examples' boot shape. Runs on
//! the 8 MHz reset IRC8M clock (no PLL bring-up), which is the `sysclk_hz` passed to `Timebase`.
//!
//! # Interrupt wiring
//!
//! The SysTick exception symbol is owned by cortex-m-rt (`#[exception] SysTick`), so this example
//! defines a one-line `SysTick` that delegates to `runtime_hal::on_systick()`, the same
//! HAL-delegation shape detection uses for its BusFault. `on_systick()` bumps the HAL tick count
//! (and would call a registered tick handler, which this example does not use: it polls the count
//! from `main` instead). The application defines NO BusFault handler: `detect_chip()` owns its
//! discrimination BusFault entirely.

#![no_std]
#![no_main]

use cortex_m_rt::{entry, exception};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_halt as _;

use runtime_hal::{detect_chip, tick_count, Timebase};

/// Tick rate: SysTick fires this many times per second. Toggling PB9 each tick halves it into the
/// audible tone frequency (4000 Hz tick => ~2 kHz square wave on a passive buzzer).
const TICK_HZ: u32 = 4_000;

/// The reset IRC8M core clock this example runs on (no PLL bring-up); the `sysclk_hz` for `Timebase`.
const SYSCLK_HZ: u32 = 8_000_000;

/// Beep "on" window in ticks: 200 ms of tone = 0.200 s * 4000 ticks/s = 800 ticks.
const BEEP_ON_TICKS: u32 = TICK_HZ / 5; // 800
/// Beep "off" (silent gap) window in ticks: another 200 ms.
const BEEP_OFF_TICKS: u32 = TICK_HZ / 5; // 800
/// Number of startup beeps before going silent.
const BEEPS: u32 = 3;

/// cortex-m-rt's SysTick exception: one line, delegating to the HAL (mirrors detection's BusFault
/// delegate). `runtime_hal::on_systick()` bumps the HAL tick count that `main` polls below.
#[exception]
fn SysTick() {
    runtime_hal::on_systick();
}

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the Timebase. detect_chip() uses
    // Peripherals::steal() for its probe, so this take() still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();

    // Detect the chip; a part matching neither family panics (panic_halt halts): fail-loud.
    let chip = detect_chip().unwrap();

    // Match the other examples' boot shape. PB9 is not a JTAG-overlay pin, so this is not strictly
    // needed for the buzzer; it is a harmless no-op on the F1x0 and only touches PB3/PB4/PA15 on the
    // F10x.
    chip.free_jtag_pins().ok();

    // PB9 (buzzer) is on GPIOB. split() hands back named pins; reconfigure PB9 as a push-pull output.
    let gpiob = chip.gpiob().unwrap().split();
    let mut buzzer = gpiob.pb9.into_push_pull_output();
    let _ = buzzer.set_low();

    // Build the SysTick interrupt-mode timebase at 4 kHz. From here SysTick fires the exception every
    // 250 us; the exception (above) bumps the HAL tick count. Constructed on the 8 MHz reset clock.
    let _timebase = Timebase::new(cp.SYST, SYSCLK_HZ, TICK_HZ).unwrap();

    // The full startup pattern length in ticks: BEEPS * (on + off).
    let pattern_ticks = BEEPS * (BEEP_ON_TICKS + BEEP_OFF_TICKS);

    // Poll the HAL tick count and drive PB9 from it. NO Delay: the timing is the tick count.
    // - During an "on" window, toggle PB9 on every new tick => the ~2 kHz tone.
    // - During an "off" window (and after the pattern), hold PB9 low => silence.
    let mut last_tick = tick_count();
    loop {
        let now = tick_count();
        if now == last_tick {
            // No new tick yet; spin (cheap, single atomic read).
            continue;
        }
        last_tick = now;

        // Where are we in the 3-beep startup pattern? Once past it, stay silent.
        if now >= pattern_ticks {
            let _ = buzzer.set_low();
            continue;
        }
        let phase = now % (BEEP_ON_TICKS + BEEP_OFF_TICKS);
        if phase < BEEP_ON_TICKS {
            // Tone window: a new tick arrived, so toggle the pin (square wave).
            let _ = buzzer.toggle();
        } else {
            // Silent gap: hold low.
            let _ = buzzer.set_low();
        }
    }
}
