//! Hall-driven 6-step (block) commutation on the bench SPLIT board (the F103 master / F130 slave
//! pair), one image, runtime-detected. Spins ONE motor slowly under hall feedback.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x) or a GD32F130C8T6 (F1x0); `detect_chip()`
//! picks the register model at boot. The HAL gives the silicon mechanism (the complementary
//! advanced-timer bring-up, the per-channel output enable, the arming gate, the multi-pin input read,
//! the hidden per-family pin routing); the `control` crate gives the 6-step DECODE (hall code -> which
//! channel sources / sinks / floats). NO `chip.family()` / `chip.arch()` anywhere: the per-family
//! differences are absorbed by the HAL. The pin map and the shoot-through-safe dead-time come from the
//! reverse-engineered `SPLIT_BOARD` contract (halls PC13 / PA1 / PC14, gates PA8/9/10 high +
//! PB13/14/15 low, dead-time DTG = 25; BalanceAgain findings/sixfet_commutation_contract.md).
//!
//! # SAFETY (read before flashing). This example ENERGIZES the motor bridge.
//!
//! It calls `ArmGate::arm()` (sets the timer's MOE), so the FET gates are live. Before running it:
//!
//! - Current-limit the bench supply (start at <= 0.5 A) and be ready to cut power.
//! - The wheel must be free to spin (nothing in the spokes).
//! - The dead-time is programmed from the RE'd contract (DTG = 25). At this example's 8 MHz reset
//!   clock that same count is a LONGER absolute dead-time than on the stock clock, which errs SAFE
//!   (more shoot-through margin), never shorter. Do not set the dead-time to zero.
//! - It starts at a low duty ([`DUTY_PERCENT`]) so torque / current stay gentle.
//!
//! # Tuning on the bench (the motor-specific knobs)
//!
//! - [`ALIGN_OFFSET`] (0..5): which hall code maps to which commutation state depends on the motor's
//!   hall placement. Only one offset per direction spins smoothly; the others cog or run backward.
//!   Sweep 0..5 until it spins, then fix it here.
//! - [`DIRECTION`]: forward / reverse (swaps source and sink).
//! - [`DUTY_PERCENT`]: raise gradually for more speed/torque once the offset is right.
//!
//! On a sensor-fault hall code (all-low / all-high) the commutator floats every channel (coast), so a
//! disconnected hall does not drive the bridge into a bad state.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use control::{bring_up_motor, Direction, MotorContract, SixStep};
use runtime_hal::{detect_chip, PeriphLabel};

/// The reset IRC8M core clock this example runs on (no PLL bring-up): the `sysclk_hz` for `Delay`
/// and the TIMER0 input clock.
const SYSCLK_HZ: u32 = 8_000_000;
/// PWM period (CAR). Center-aligned at 8 MHz this is `8 MHz / (2 * 250) = 16 kHz`, matching the
/// stock PWM rate and above most of the audible range.
const PWM_PERIOD: u16 = 250;
/// Starting duty as a percent of the period. Low: a gentle, slow spin.
const DUTY_PERCENT: u32 = 30;
/// The motor-specific hall-to-state alignment (0..5). Sweep on the bench until it spins smoothly.
const ALIGN_OFFSET: u8 = 0;
/// Rotation direction.
const DIRECTION: Direction = Direction::Forward;
/// Hall sampling period. Short enough to track the rotor at a slow spin.
const LOOP_US: u32 = 200;

// GPIO port indices for the `(port << 4) | pin` contract encoding.
const A: u8 = 0;
const B: u8 = 1;
const C: u8 = 2;
/// Shorthand for the contract pin encoding ([`MotorContract::pin`]).
const fn p(port: u8, pin: u8) -> u8 {
    MotorContract::pin(port, pin)
}

/// The 6-FET split bench board commutation contract (RoboDurden-layout, one motor on TIMER0 / TIM1),
/// reverse-engineered from the stock F103 image: halls PC13 / PA1 / PC14, gates high PA8 / PA9 / PA10
/// + low PB13 / PB14 / PB15, dead-time DTG = 25. (BalanceAgain `findings/sixfet_commutation_contract.md`.)
const SPLIT_BOARD: MotorContract = MotorContract {
    hall_pins: [p(C, 13), p(A, 1), p(C, 14)],
    gate_high: [p(A, 8), p(A, 9), p(A, 10)],
    gate_low: [p(B, 13), p(B, 14), p(B, 15)],
    dead_time: 25,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    // Detect the chip; a part matching neither family panics (panic_halt halts): fail-loud.
    let chip = detect_chip().unwrap();

    // SELF_HOLD (PB12) high: latch the board's main power rail on (the gate-driver rail sits behind
    // it on the bench board). output_pin absorbs the family GPIO model; no family branch.
    let mut self_hold = chip.output_pin(PeriphLabel::Gpiob, 12).unwrap();
    let _ = self_hold.set_high();

    // Halls are plain GPIO inputs (floating at reset), so they need only the port clock. Enable the
    // hall ports' clocks (the gate / SELF_HOLD ports were enabled by their routing / output_pin); the
    // bring-up helper builds the hall reader over these lines.
    let _ = chip.gpioa();
    let _ = chip.gpioc();

    // Bring up TIMER0's complementary bridge from the contract: configure the timer (dead-time +
    // period), route the six gate pins (family-internal), start the counter, and hand back the
    // control-layer commutator (6-step decode + the HAL handle), the arming gate, and the hall
    // reader. MOE stays OFF (the helper does NOT arm); the bridge is energized explicitly below.
    let (commutator, gate, reader) =
        bring_up_motor(&chip, &SPLIT_BOARD, PeriphLabel::Timer0, PWM_PERIOD, SixStep::new(DIRECTION, ALIGN_OFFSET))
            .unwrap();

    let mut delay = runtime_hal::Delay::new(cp.SYST, SYSCLK_HZ);
    let duty = (u32::from(PWM_PERIOD) * DUTY_PERCENT / 100) as u16;

    // SAFETY: arm the bridge (MOE on). From here the gates are live; current-limit the supply.
    gate.arm();

    // Hall-driven 6-step loop: read the rotor position, drive the matching commutation step at a low
    // duty, and the motor self-commutates. A sensor-fault code coasts (all channels floated).
    loop {
        let code = reader.read();
        let _ = commutator.apply(code, duty);
        delay.delay_us(LOOP_US);
    }
}
