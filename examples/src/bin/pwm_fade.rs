//! Fade the green LED with a hardware general-purpose-timer PWM, on either board, one image (G3).
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It drives `TIMER1_CH1` (a GENERAL-purpose
//! timer, NOT the motor bridge) onto PB3 (the green LED) and sweeps the duty up and down so the LED
//! FADES. The same call works on both boards; the per-family pin routing is HIDDEN inside the HAL.
//!
//! # The family difference is absorbed, not exposed
//!
//! Getting `TIMER1_CH1` onto PB3 takes a DIFFERENT register mechanism on each family (F10x: free the
//! JTAG overlay + AFIO `TIMER1_REMAP` partial-remap-1 + the CRL AF nibble; F1x0: one per-pin `AFSEL`
//! field = AF2). The application does NOT branch on the family: it passes the output pin to
//! `PwmOut::new`, which routes it via `Chip::route_general_pwm_pin` internally. There is no
//! `chip.family()` / `chip.arch()` in this example, the HAL absorbs the difference.
//!
//! Why `TIMER1` and not `TIMER2`: on the 48-pin GD32F103C8, `TIMER2`'s remap to PB4/PB5 needs a
//! 64/100/144-pin package, so those channels are not reachable. `TIMER1`'s partial-remap-1 to PB3
//! IS reachable on the 48-pin part. `PwmOut` refuses an advanced-timer label, so this can never reach
//! `TIMER0` (the motor bridge) or the MOE/POEN arming gate.
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

use runtime_hal::{detect_chip, PeriphLabel, PwmOut};

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

    // 2. Bring up the single-channel general PWM on TIMER1_CH1, routed to PB3. The per-family pin
    //    routing (F10x AFIO remap + JTAG-free + CRL nibble; F1x0 AFSEL) is done INSIDE PwmOut::new,
    //    so this one call is identical on both boards, no chip.family() / chip.arch() branch. PwmOut
    //    REFUSES an advanced-timer label, so it can never reach TIMER0 / the MOE gate. The counter
    //    starts running at zero duty (a general timer drives the pin with no arming step).
    let mut pwm = PwmOut::new(&chip, PeriphLabel::Timer1, PB3, PWM_FREQ_HZ, TIMER_CLK_HZ).unwrap();
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
