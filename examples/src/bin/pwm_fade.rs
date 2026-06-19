//! Fade the green LED with a hardware general-purpose-timer PWM, on either board, one image (G3).
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It drives `TIMER1_CH1` (a GENERAL-purpose
//! timer, NOT the motor bridge) onto PB3 (the green LED) and sweeps the duty up and down so the LED
//! FADES. The shared PWM datapath (`runtime_hal::PwmOut`) is identical on both boards; what differs,
//! and what this example makes VISIBLE, is the per-family PIN ROUTING.
//!
//! # The family difference this example makes visible
//!
//! Getting the SAME timer channel (`TIMER1_CH1`) onto the SAME pin (PB3) takes a DIFFERENT register
//! mechanism on each family, so the routing branches on `chip.family()` (the deliberate escape hatch
//! for architecture-specific setup the HAL does not abstract). This is the visible F10x-vs-F1x0
//! difference, not a hidden one:
//!
//! - **F10x (GD32F103)**: PB3 is JTDO after reset and `TIMER1_CH1` is on PA1 by default, so it takes
//!   THREE steps. (1) `chip.free_jtag_pins()` releases PB3 from the JTAG debug port (keeping SWD
//!   live). (2) `remap_timer1_partial1()` sets the AFIO `TIMER1_REMAP[9:8]` field to `01` (partial
//!   remap 1), which maps `TIMER1_CH1 / PB3`. (3) PB3's CRL nibble is set to alternate-function
//!   push-pull (on the F10x the AF is implied by the mode/cnf nibble, there is no per-pin AF mux).
//! - **F1x0 (GD32F130)**: ONE field. PB3's per-pin `AFSEL` mux is set to AF2 (`TIMER1_CH1` on AF2,
//!   per the GD32F130xx datasheet's Port B alternate-function summary). No AFIO, no remap.
//!
//! Why `TIMER1` and not `TIMER2`: on the 48-pin GD32F103C8, `TIMER2`'s remap to PB4/PB5 needs a
//! 64/100/144-pin package, so those channels are not reachable. `TIMER1`'s partial-remap-1 to PB3
//! IS reachable on the 48-pin part, which is why the G3 target is `TIMER1`. Neither path touches
//! `TIMER0` (the motor bridge) or the MOE/POEN arming gate; `PwmOut` refuses an advanced-timer label.
//!
//! # Pin and clock
//!
//! LED = PB3 (green on the bench boards; it lights WITHOUT the SELF_HOLD power rail, so this example
//! does NOT drive PB12). Runs on the 8 MHz reset IRC8M clock (no PLL bring-up): the PWM is ~1 kHz
//! (`CAR = round(8 MHz / 1 kHz) - 1 = 7999`), far above the eye's flicker-fusion, so the duty sweep
//! reads as a smooth brightness fade. The application defines NO fault handler (`detect_chip()` owns
//! its probe BusFault internally; see `blinky`).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;

use runtime_hal::{detect_chip, Arch, PeriphLabel, PinRole, PwmOut};

/// The 8 MHz reset IRC8M clock this example runs on (no PLL bring-up). It is the TIMER1 input clock.
const TIMER_CLK_HZ: u32 = 8_000_000;
/// ~1 kHz PWM (flicker-free for an LED fade).
const PWM_FREQ_HZ: u32 = 1_000;
/// PB3 logical pin byte (port B = 1, pin 3): the green LED = TIMER1_CH1.
const PB3: u8 = (1 << 4) | 3;

#[entry]
fn main() -> ! {
    // 1. Detect the chip at runtime (family probe + peripheral-presence measurement). Fail-loud: a
    //    part matching neither family panics here and `panic_halt` halts (no guessed register model).
    let chip = detect_chip().unwrap();

    // 2. Architecture-aware pin routing: drive TIMER1_CH1 onto PB3. This is the VISIBLE family
    //    branch, the kind of architecture-specific setup the HAL deliberately does NOT abstract. The
    //    chip.arch() witness hands back a family TOKEN: each token bakes in the correct register model
    //    and exposes only the operations that family supports, so the wrong-architecture register
    //    write is a COMPILE error, not a runtime fault (e.g. remap_timer1_partial1 exists ONLY on the
    //    F10x token; there is no AFIO on the F1x0).
    match chip.arch() {
        Arch::F10x(f10x) => {
            // F10x: three steps to get TIMER1_CH1 onto PB3 (JTDO at reset, CH1 elsewhere by default).
            // (a) Free PB3 from the JTAG debug port (keeps SWD live). F10x-only (absent on the F1x0).
            f10x.free_jtag_pins().ok();
            // (b) AFIO TIMER1_REMAP = partial-remap-1: maps TIMER1_CH1 -> PB3. F10x-only (no AFIO on
            //     the F1x0); the method does not exist on the F1x0 token. Enables the AFIO clock too.
            f10x.remap_timer1_partial1().ok();
            // (c) PB3 = alternate-function push-pull (on the F10x the AF is implied by the nibble).
            //     The F10x CRL/CRH path is baked into the token; the F1x0 AFSEL write is unreachable.
            f10x.configure_pin_af(PB3, PinRole::GenTimerAfPushPull).ok();
        }
        Arch::F1x0(f1x0) => {
            // F1x0: ONE field. PB3's per-pin AFSEL mux -> AF2 (TIMER1_CH1 on AF2). No AFIO, no remap.
            // The F1x0 AFSEL path is baked into the token; the F10x CRL/CRH write is unreachable here.
            f1x0.configure_pin_af(PB3, PinRole::GenTimerAfPushPull).ok();
        }
    }
    // Enable the GPIOB port clock (the routing above wrote the port's config registers; the clock
    // must be on for those writes to stick on hardware). The port getter enables it through the
    // detected chip's clock path; we only need the side effect, not the split pins.
    let _ = chip.gpiob();

    // 3. Shared datapath: bring up the single-channel general PWM on TIMER1_CH1. PwmOut REFUSES an
    //    advanced-timer label, so this can never reach TIMER0 / the MOE gate. The counter starts
    //    running at zero duty; the general timer drives the pin as soon as the channel + counter are
    //    enabled (no arming step). This call is IDENTICAL on both families.
    let mut pwm = PwmOut::new(&chip, PeriphLabel::Timer1, PWM_FREQ_HZ, TIMER_CLK_HZ).unwrap();
    let max = pwm.max_duty_cycle();

    // 4. Sweep the duty up then down forever so the green LED fades. The sweep is done in fixed steps
    //    with a short busy-wait between writes; on the 8 MHz reset clock this gives a visible breathe.
    loop {
        // Fade up.
        let mut duty: u32 = 0;
        while duty <= max as u32 {
            let _ = pwm.set_duty_cycle(duty as u16);
            busy_wait(2_000);
            duty += (max as u32 / 64).max(1);
        }
        // Fade down.
        let mut duty: i32 = max as i32;
        while duty >= 0 {
            let _ = pwm.set_duty_cycle(duty as u16);
            busy_wait(2_000);
            duty -= (max as i32 / 64).max(1);
        }
    }
}

/// A crude busy-wait between duty steps (no `Delay`/SysTick claimed, so the example stays focused on
/// the PWM). `cortex_m::asm::nop` is not optimised away, so the loop count sets the step dwell.
#[inline(never)]
fn busy_wait(count: u32) {
    for _ in 0..count {
        cortex_m::asm::nop();
    }
}
