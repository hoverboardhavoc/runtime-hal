//! Blink one LED, on either supported board, from a single image.
//!
//! This is the headline runtime-hal example: ONE binary, flashed unchanged to a GD32F103C8T6 (F10x
//! family) or a GD32F130C8T6 (F1x0 family), blinks an LED on both. There is no compile-time chip
//! selection: `detect_chip()` works out the family at boot and picks the matching register model, so
//! the same `GpioPort` / `Pin` calls below drive the F10x CRL/CRH model on one board and the F1x0
//! CTL/OMODE/OSPD model on the other.
//!
//! The application defines NO fault handler. `detect_chip()` performs a deliberate reserved-region
//! read to discriminate the family; it owns that BusFault entirely by installing its own
//! probe-scoped vector table (via VTOR) and restoring it before returning, so there is no
//! `#[exception] BusFault` here.
//!
//! LED pin: PB3 = green LED on the bench boards (board LED map: green PB3, orange PA15, red PB4,
//! upper PB2, lower PB5). On the F10x, PB3 is JTDO after reset and must be freed from the JTAG debug
//! port before it can drive GPIO; `chip.free_jtag_pins()` does that (and keeps SWD live). On the
//! F1x0 that call is a no-op (no AFIO, PB3 is already GPIO).
//!
//! It also demonstrates the HAL's `embedded_hal::delay::DelayNs` implementer, `runtime_hal::Delay`:
//! the blink interval is timed by SysTick rather than a hand-rolled `nop` loop.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::detect_chip;

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the delay. `detect_chip()` uses
    // `cortex_m::Peripherals::steal()` internally for the SCB during its probe, so this `take()` in
    // main still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();
    // 1. Detect the chip at runtime (family probe + peripheral-presence measurement). On a part that
    //    matches neither known family, `detect_chip` returns Err and this `.unwrap()` panics; the
    //    `panic_halt` handler then halts. That is fail-loud, rather than guessing a register layout.
    let chip = detect_chip().unwrap();

    // 2. Free the JTAG-overlay pins so PB3 can drive the LED. F10x: disables JTAG-DP, keeps SW-DP
    //    (SWD stays attached). F1x0: no-op. `.ok()` because the only failure is a missing RCU base,
    //    which a detected chip always carries.
    chip.free_jtag_pins().ok();

    // 3. Take GPIOB (this enables its port clock through the detected chip's clock path) and split it
    //    into named pins. `split()` hands back the port's pins directly, so `pb3` resolves with no
    //    register-handle argument (unlike a compile-time HAL's `into_push_pull_output(&mut crl)`).
    let gpiob = chip.gpiob().unwrap().split();

    // 4. Reconfigure PB3 (reset state: floating input) as a push-pull output. The F10x-vs-F1x0
    //    register-model branch lives inside the HAL; this line is identical on both boards.
    let mut led = gpiob.pb3.into_push_pull_output();

    // 5. Build the SysTick-backed delay. 8 MHz is the reset IRC8M clock this example runs on: it
    //    never brings up the PLL, so the core is still on the internal RC oscillator here.
    let mut delay = runtime_hal::Delay::new(cp.SYST, 8_000_000);

    // 6. Blink forever. set_high / set_low are infallible here (Infallible error type); the 200 ms
    //    interval is timed by SysTick through the `DelayNs` trait.
    loop {
        let _ = led.set_high();
        delay.delay_ms(200);
        let _ = led.set_low();
        delay.delay_ms(200);
    }
}
