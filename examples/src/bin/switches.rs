//! Mirror the two foot pads onto two LEDs, on either supported board, from a single image.
//!
//! This is the GPIO-input companion to `blinky`: it reads the board's two foot-pad inputs and
//! lights an LED for each. Step on a pad and its LED comes on. ONE binary, flashed unchanged to a
//! GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family). There is no compile-time chip
//! selection: `detect_chip()` works out the family at boot, so the same `Pin` calls below drive the
//! F10x CRL/CRH model on one board and the F1x0 CTL/PUD model on the other.
//!
//! It demonstrates the HAL's embedded-hal `digital::InputPin` (the foot pads) alongside
//! `digital::OutputPin` (the LEDs), both through the type-state split() API: each pin starts as a
//! reset `Pin<Input<Floating>>` and is reconfigured with `into_pull_down_input` /
//! `into_push_pull_output`.
//!
//! Foot pads (digital inputs, pin-high = foot present):
//! - pad A = PA11 (on GPIOA)
//! - pad B = PC15 (on GPIOC)
//!
//! Source: `Declassyfied/firmware/inputs.c` section 2.3 (the RoboDurden 2-x-20 defines omit the foot
//! pads; this is the RE-recovered mapping). PA11 and PC15 are plain GPIO inputs (NOT JTAG-overlay
//! pins), so they need no remap; PC15 is on GPIOC, which the descriptor now carries. Both pads are
//! configured as PULL-DOWN inputs: pin-high = present, so a pull-down is the safe default (the pin
//! idles low when no foot is on the pad). The exact pull is to be confirmed on the bench.
//!
//! LEDs (push-pull outputs, on the bench board LED map: green PB3, red PB4, orange PA15):
//! - green = PB3 (on GPIOB), mirrors pad A
//! - red = PB4 (on GPIOB), mirrors pad B
//!
//! PB3 (JTDO) and PB4 (NJTRST) are JTAG-overlay pins on the F103, so `free_jtag_pins()` is needed
//! before they can drive GPIO (it disables JTAG-DP but keeps SWD live, freeing PB3/PB4/PA15). On the
//! F1x0 that call is a no-op (no AFIO, the pins are already GPIO). Note: `free_jtag_pins` also frees
//! PA15 (the orange LED); since NJTRST/JTDI carry internal pull-ups, an LED on a FREED-but-undriven
//! JTAG pin floats high on the F103 (appears stuck on), so this example drives all three (green, red,
//! and orange-parked-off) to known states.
//!
//! The application defines NO fault handler: `detect_chip()` owns its discrimination BusFault
//! entirely (own probe-scoped vector table via VTOR, restored before it returns), so there is no
//! `#[exception] BusFault` here.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use panic_halt as _;

use runtime_hal::detect_chip;

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the poll delay. `detect_chip()` uses
    // `cortex_m::Peripherals::steal()` internally for its probe, so this `take()` still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();

    // 1. Detect the chip at runtime (family probe + peripheral-presence measurement). On a part that
    //    matches neither known family this `.unwrap()` panics and `panic_halt` halts: fail-loud,
    //    rather than guessing a register layout.
    let chip = detect_chip().unwrap();

    // 2. Free the JTAG-overlay pins so PB3 (green) and PB4 (red) can drive their LEDs. F10x:
    //    disables JTAG-DP, keeps SW-DP (SWD stays attached). F1x0: no-op. `.ok()` because the only
    //    failure is a missing RCU base, which a detected chip always carries.
    chip.free_jtag_pins().ok();

    // 3. Take the three ports (each call enables that port's clock through the detected chip's clock
    //    path) and split each into its named pins. The F10x-vs-F1x0 register-model branch lives
    //    inside the HAL; these lines are identical on both boards.
    let gpioa = chip.gpioa().unwrap().split();
    let gpiob = chip.gpiob().unwrap().split();
    let gpioc = chip.gpioc().unwrap().split();

    // 4. Foot pads as pull-down digital inputs (pin-high = foot present, per inputs.c section 2.3).
    //    Pull-down is the safe default: the pin idles low when nothing is on the pad.
    let mut pad_a = gpioa.pa11.into_pull_down_input(); // foot pad A
    let mut pad_b = gpioc.pc15.into_pull_down_input(); // foot pad B

    // 5. LEDs as push-pull outputs. green + red mirror the two pads; orange (PA15) is parked low
    //    because free_jtag_pins also freed it and an undriven PA15 floats high on the F103.
    let mut led_green = gpiob.pb3.into_push_pull_output(); // mirrors pad A
    let mut led_red = gpiob.pb4.into_push_pull_output(); // mirrors pad B
    let mut led_orange = gpioa.pa15.into_push_pull_output();
    led_orange.set_low().unwrap(); // unused as an indicator here; held off, not floating

    // 6. Build the SysTick-backed delay. 8 MHz is the reset IRC8M clock this example runs on (it
    //    never brings up the PLL, so the core is still on the internal RC oscillator).
    let mut delay = runtime_hal::Delay::new(cp.SYST, 8_000_000);

    // 7. Poll forever: read each pad's level and drive its LED to match. Reads and writes are
    //    infallible here (Infallible error type), so the `Result`s are unwrapped. A 10 ms poll
    //    interval is plenty for a foot pad.
    loop {
        // Pad A -> green LED.
        if pad_a.is_high().unwrap() {
            led_green.set_high().unwrap();
        } else {
            led_green.set_low().unwrap();
        }

        // Pad B -> red LED.
        if pad_b.is_high().unwrap() {
            led_red.set_high().unwrap();
        } else {
            led_red.set_low().unwrap();
        }

        delay.delay_ms(10);
    }
}
