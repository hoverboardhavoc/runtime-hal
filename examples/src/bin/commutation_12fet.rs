//! Hall-driven 6-step (block) commutation on the 12-FET DUAL-MOTOR board (one GD32F103RCT6 driving
//! TWO motors on TIMER0 / TIM1 and TIMER7 / TIM8). Spins both motors slowly under hall feedback.
//!
//! # This example is DUMP-ONLY / UNVALIDATED, and for the 12-FET dual-motor board ONLY
//!
//! The 12-FET board was reverse-engineered from a firmware image only (it is not on the bench), so
//! this example has NOT been run on hardware. It builds and encodes the intended dual-motor structure
//! from the RE'd contracts (EFeru layout: right halls PC10/11/12 + gates PA8/9/10 / PB13/14/15 on
//! TIMER0; left halls PB5/6/7 + gates PC6/7/8 / PA7+PB0/PB1 on TIMER7; dead-time DTG = 32 on both;
//! BalanceAgain findings/twelvefet_dualmotor_commutation_contract.md).
//!
//! Do NOT flash this to the bench SPLIT board: it drives a second advanced timer (TIMER7 / TIM8) that
//! a 48-pin F103C8 does not have, and uses the dual-motor hall pins. Use `commutation_splitboard` for
//! the bench. On a single-advanced-timer part `PwmTimer::configure(.., Timer7, ..)` fails loud
//! (`Timer7` is absent from the descriptor), so the wrong board does not silently mis-drive.
//!
//! # No magic addresses, no family branch
//!
//! `detect_chip()` populates `Timer7`'s base in the descriptor when it measures a second advanced
//! timer (adv_timers == 2), so BOTH motors resolve their timer the SAME way, `PwmTimer::configure`
//! with `Timer0` / `Timer7`. There is no hardcoded peripheral address and no `chip.family()`: the gate
//! routing is absorbed by `Chip::route_advanced_pwm_pin`.
//!
//! # SAFETY (read before any attempt to run). This example ENERGIZES two motor bridges.
//!
//! It arms both timers (sets MOE), so all twelve FET gates go live. Current-limit the supply
//! (<= 0.5 A), keep both wheels free, start at the low [`DUTY_PERCENT`]. The dead-time (DTG = 32) is
//! programmed from the RE'd contract; at this example's 8 MHz clock that is a LONGER (safer) absolute
//! dead-time than the stock value, never shorter. Per motor, [`ALIGN_OFFSET_RIGHT`] /
//! [`ALIGN_OFFSET_LEFT`] must be swept (0..5) to find the smooth-spin alignment. A sensor-fault hall
//! code coasts that motor.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use panic_halt as _;

use control::{bring_up_motor, Direction, MotorContract, SixStep};
use runtime_hal::{detect_chip, PeriphLabel};

/// The reset IRC8M core clock this example runs on (no PLL bring-up).
const SYSCLK_HZ: u32 = 8_000_000;
/// PWM period (CAR). Center-aligned at 8 MHz: `8 MHz / (2 * 250) = 16 kHz`.
const PWM_PERIOD: u16 = 250;
/// Starting duty (percent of period): a gentle, slow spin.
const DUTY_PERCENT: u32 = 30;
/// Per-motor hall-to-state alignment (0..5), swept on the bench. Right = TIMER0, left = TIMER7.
const ALIGN_OFFSET_RIGHT: u8 = 0;
const ALIGN_OFFSET_LEFT: u8 = 0;
/// Rotation direction for each motor.
const DIRECTION: Direction = Direction::Forward;
/// Hall sampling period.
const LOOP_US: u32 = 200;

// GPIO port indices for the `(port << 4) | pin` contract encoding.
const A: u8 = 0;
const B: u8 = 1;
const C: u8 = 2;
const fn p(port: u8, pin: u8) -> u8 {
    MotorContract::pin(port, pin)
}

/// The 12-FET dual-motor board's RIGHT motor (EFeru-layout, TIMER0 / TIM1): halls PC10/11/12, gates
/// high PA8/9/10 + low PB13/14/15, dead-time DTG = 32. Same gate pins as the split board, DIFFERENT
/// halls and dead-time.
const DUAL_MOTOR_RIGHT: MotorContract = MotorContract {
    hall_pins: [p(C, 10), p(C, 11), p(C, 12)],
    gate_high: [p(A, 8), p(A, 9), p(A, 10)],
    gate_low: [p(B, 13), p(B, 14), p(B, 15)],
    dead_time: 32,
};

/// The 12-FET dual-motor board's LEFT motor (EFeru-layout, TIMER7 / TIM8): halls PB5/6/7, gates high
/// PC6/7/8 + low PA7 / PB0 / PB1, dead-time DTG = 32.
const DUAL_MOTOR_LEFT: MotorContract = MotorContract {
    hall_pins: [p(B, 5), p(B, 6), p(B, 7)],
    gate_high: [p(C, 6), p(C, 7), p(C, 8)],
    gate_low: [p(A, 7), p(B, 0), p(B, 1)],
    dead_time: 32,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let chip = detect_chip().unwrap();

    // Enable the port clocks both motors touch (A, B, C). The getters absorb the family clock model.
    let _ = chip.gpioa();
    let _ = chip.gpiob();
    let _ = chip.gpioc();

    // Right motor on TIMER0, left motor on TIMER7. BOTH resolve their timer base from the descriptor
    // (Timer7 is populated when a second advanced timer was detected): same call, no magic address.
    // bring_up_motor does NOT arm; both bridges are energized explicitly below.
    let (comm_r, gate_r, halls_r) = bring_up_motor(
        &chip,
        &DUAL_MOTOR_RIGHT,
        PeriphLabel::Timer0,
        PWM_PERIOD,
        SixStep::new(DIRECTION, ALIGN_OFFSET_RIGHT),
    )
    .unwrap();
    let (comm_l, gate_l, halls_l) = bring_up_motor(
        &chip,
        &DUAL_MOTOR_LEFT,
        PeriphLabel::Timer7,
        PWM_PERIOD,
        SixStep::new(DIRECTION, ALIGN_OFFSET_LEFT),
    )
    .unwrap();

    let mut delay = runtime_hal::Delay::new(cp.SYST, SYSCLK_HZ);
    let duty = (u32::from(PWM_PERIOD) * DUTY_PERCENT / 100) as u16;

    // SAFETY: arm both bridges (MOE on). All twelve gates are now live; current-limit the supply.
    gate_r.arm();
    gate_l.arm();

    // Hall-driven 6-step loop for both motors.
    loop {
        let _ = comm_r.apply(halls_r.read(), duty);
        let _ = comm_l.apply(halls_l.read(), duty);
        delay.delay_us(LOOP_US);
    }
}
