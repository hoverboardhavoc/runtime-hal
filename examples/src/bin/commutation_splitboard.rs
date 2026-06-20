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
use runtime_hal::clock::{configure_tree, ClockConfig};
use runtime_hal::{detect_chip, PeriphLabel};

/// Core clock after the PLL bring-up below: the 72 MHz IRC8M->PLL tree, matching the stock board.
/// This is the `sysclk_hz` for `Delay` and the TIMER0 input clock.
const SYSCLK_HZ: u32 = 72_000_000;
/// PWM period (CAR). Center-aligned at 72 MHz this is `72 MHz / (2 * 2250) = 16 kHz`, reproducing the
/// stock ARR = 2250 / 16 kHz exactly (so the dead-time count DTG = 25 also yields the stock ~694 ns).
const PWM_PERIOD: u16 = 2250;
/// Starting duty as a percent of the period. Low: a gentle, slow spin.
const DUTY_PERCENT: u32 = 30;
/// The motor-specific hall-to-state alignment (0..5). Sweep on the bench until it spins smoothly.
const ALIGN_OFFSET: u8 = 1;
/// Rotation direction.
const DIRECTION: Direction = Direction::Forward;
/// Hall sampling period. Short enough to track the rotor at a slow spin.
const LOOP_US: u32 = 200;

/// SWD-readable observation block (find by the `COMMUT_OBS` symbol or read its RAM address).
/// `magic` marks it; `seq` increments each loop (liveness); `hall_code` is the raw 3-bit hall
/// reading (1..=6 valid, 0/7 = sensor fault -> coast); `applied` is 1 if the commutator drove a
/// step, 0 if it coasted. A spinning motor shows `hall_code` cycling through 1..=6.
#[repr(C)]
struct CommutObs {
    magic: u32,
    seq: u32,
    hall_code: u32,
    applied: u32,
}
const OBS_MAGIC: u32 = 0xC0_77_07_05;
#[no_mangle]
static mut COMMUT_OBS: CommutObs = CommutObs {
    magic: 0,
    seq: 0,
    hall_code: 0,
    applied: 0,
};

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

    // Bring up the 72 MHz IRC8M->PLL clock tree (the stock board's rate). With CAR = 2250 this gives
    // the stock 16 kHz PWM, and at this clock the contract's DTG = 25 / CKD = /2 is the stock ~694 ns
    // dead-time. configure_tree validates against the family ceiling and polls for PLL lock + switch.
    configure_tree(&chip, &ClockConfig::REFERENCE_72M_IRC8M).unwrap();

    // Enable the port clocks FIRST. A GPIO write (pin-mode config or set_high) only sticks once the
    // port's peripheral clock is on, and `output_pin` does NOT enable it. GPIOB carries SELF_HOLD
    // (PB12) and the low-side gates (PB13/14/15); GPIOA/GPIOC carry the high-side gates / halls.
    let _ = chip.gpioa();
    let _ = chip.gpiob();
    let _ = chip.gpioc();

    // SELF_HOLD (PB12) high: latch the board's main power rail on (the gate-driver rail sits behind it
    // on the slave board; the master's rail is powered regardless). GPIOB's clock is enabled above, so
    // this write takes; without it PB12 stays low and the slave bridge has no gate-driver power.
    let mut self_hold = chip.output_pin(PeriphLabel::Gpiob, 12).unwrap();
    let _ = self_hold.set_high();

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
    let mut seq: u32 = 0;
    loop {
        let code = reader.read();
        let applied = commutator.apply(code, duty).unwrap_or(false);
        seq = seq.wrapping_add(1);
        // SWD readback: publish the live hall code + whether a step was driven (liveness via seq).
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(COMMUT_OBS),
                CommutObs {
                    magic: OBS_MAGIC,
                    seq,
                    hall_code: u32::from(code),
                    applied: applied as u32,
                },
            );
        }
        delay.delay_us(LOOP_US);
    }
}
