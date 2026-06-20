//! Hall-driven 6-step (block) commutation on the 12-FET DUAL-MOTOR board (one GD32F103RCT6 that can
//! drive TWO motors on TIMER0 / TIM1 and TIMER7 / TIM8).
//!
//! # SINGLE-WHEEL-FIRST mode
//!
//! For the first on-hardware bring-up this example drives ONLY the RIGHT motor (TIMER0), so only one
//! bridge is energized while the alignment is found. The LEFT motor (TIMER7) is defined but NOT
//! brought up or armed; re-enable it via the commented block in `main` once the right wheel spins.
//!
//! # 12-FET dual-motor board ONLY (RE'd, previously dump-only)
//!
//! The contracts come from the EFeru layout (right halls PC10/11/12 + gates PA8/9/10 / PB13/14/15 on
//! TIMER0; left halls PB5/6/7 + gates PC6/7/8 / PA7+PB0/PB1 on TIMER7; dead-time DTG = 32 on both;
//! BalanceAgain findings/twelvefet_dualmotor_commutation_contract.md).
//!
//! Do NOT flash this to the bench SPLIT board: it targets TIMER7 / TIM8, which a 48-pin F103C8 does
//! not have, and uses the dual-motor hall pins. Use `commutation_splitboard` for the bench split board.
//! On a single-advanced-timer part `PwmTimer::configure(.., Timer7, ..)` fails loud, so the left-motor
//! block (when re-enabled) does not silently mis-drive the wrong board.
//!
//! # No magic addresses, no family branch
//!
//! `detect_chip()` populates `Timer7`'s base in the descriptor when it measures a second advanced
//! timer, so both motors resolve their timer the SAME way (`PwmTimer::configure` with `Timer0` /
//! `Timer7`). No hardcoded peripheral address, no `chip.family()`.
//!
//! # SWD readback
//!
//! `COMMUT_OBS` (a `#[no_mangle]` static) carries `{ magic, seq, hall_code, applied }` for the right
//! motor, updated every pass. `magic` (0xC0770712); `seq` increments (liveness); `hall_code` is the
//! raw 3-bit hall reading (1..=6 valid, 0/7 = fault -> coast); `applied` = 1 if a step was driven.
//!
//! # SAFETY (read before any attempt to run). This ENERGIZES the right motor bridge.
//!
//! It arms TIMER0 (sets MOE), so the right six FET gates go live. Current-limit the supply
//! (<= 0.5 A), keep the wheel free, start at the low [`DUTY_PERCENT`]. The dead-time (DTG = 32) is
//! programmed from the RE'd contract; at this 8 MHz clock that is a LONGER (safer) absolute dead-time
//! than the stock value, never shorter. Sweep [`ALIGN_OFFSET_RIGHT`] (0..5) for the smooth-spin
//! alignment. A sensor-fault hall code coasts the motor.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use control::{bring_up_motor, Direction, MotorContract, SixStep};
use runtime_hal::{detect_chip, PeriphLabel};

/// The reset IRC8M core clock this example runs on (no PLL bring-up).
const SYSCLK_HZ: u32 = 8_000_000;
/// PWM period (CAR). Center-aligned at 8 MHz: `8 MHz / (2 * 250) = 16 kHz`.
const PWM_PERIOD: u16 = 250;
/// Starting duty (percent of period): the average phase voltage is `DUTY_PERCENT% * Vbus`, so this is
/// the speed knob. Kept LOW for a first bring-up at 35 V (10% of 35 V ~= 3.5 V average). Raise it if
/// the wheel won't start (too little torque to break stiction); the 0.5 A supply limit is the hard cap.
const DUTY_PERCENT: u32 = 10;
/// Right-motor (TIMER0) hall-to-state alignment (0..5), swept on the bench.
const ALIGN_OFFSET_RIGHT: u8 = 1;
/// Left-motor (TIMER7) alignment; used only when the left-motor block in `main` is re-enabled.
#[allow(dead_code)]
const ALIGN_OFFSET_LEFT: u8 = 0;
/// Rotation direction.
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
/// PC6/7/8 + low PA7 / PB0 / PB1, dead-time DTG = 32. NOT driven in single-wheel-first mode.
#[allow(dead_code)]
const DUAL_MOTOR_LEFT: MotorContract = MotorContract {
    hall_pins: [p(B, 5), p(B, 6), p(B, 7)],
    gate_high: [p(C, 6), p(C, 7), p(C, 8)],
    gate_low: [p(A, 7), p(B, 0), p(B, 1)],
    dead_time: 32,
};

/// SWD-readable observation block for the right motor. `magic` marks it; `seq` proves liveness;
/// `offset` echoes the active alignment (so a sweep can confirm the override took).
#[repr(C)]
struct CommutObs {
    magic: u32,
    seq: u32,
    hall_code: u32,
    applied: u32,
    offset: u32,
}
const OBS_MAGIC: u32 = 0xC0_77_07_12;
#[no_mangle]
static mut COMMUT_OBS: CommutObs = CommutObs {
    magic: 0,
    seq: 0,
    hall_code: 0,
    applied: 0,
    offset: 0,
};

/// SWD-readable observation block for the LEFT motor (same layout as the right).
#[no_mangle]
static mut COMMUT_OBS_LEFT: CommutObs = CommutObs {
    magic: 0,
    seq: 0,
    hall_code: 0,
    applied: 0,
    offset: 0,
};

/// SWD-WRITABLE alignment override for the RIGHT motor (bench offset sweep): write 0..5 here over SWD
/// and the loop re-points the right commutator without a reflash. Initialised to the default.
#[no_mangle]
static mut ALIGN_OVERRIDE: u8 = ALIGN_OFFSET_RIGHT;

/// SWD-WRITABLE alignment override for the LEFT motor (its own sweep, independent of the right).
#[no_mangle]
static mut ALIGN_OVERRIDE_LEFT: u8 = ALIGN_OFFSET_LEFT;

/// SWD-WRITABLE duty override, in TIMER counts (0..=PWM_PERIOD), shared by both motors: raise it over
/// SWD for more torque without a reflash. Clamped to PWM_PERIOD in the loop. Default = DUTY_PERCENT.
#[no_mangle]
static mut DUTY_OVERRIDE: u16 = (PWM_PERIOD as u32 * DUTY_PERCENT / 100) as u16;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let chip = detect_chip().unwrap();

    // Enable the port clocks the right motor touches (A, B, C). The getters absorb the family clock
    // model. (output_pin/analog_pin self-enable, but bring_up_motor's hall InputGroup does not, so the
    // hall ports' clocks must be on here.)
    let _ = chip.gpioa();
    let _ = chip.gpiob();
    let _ = chip.gpioc();

    // EFeru power latch: this board is variant 0, so OFF_PIN = PA5. The stock firmware holds it high to
    // keep the board powered and the gate-driver rail on (confirmed by reading the running stock:
    // GPIOA_OCTL bit5 set). It is the SELF_HOLD analogue here; without it the bridge has no gate power
    // (armed but no current). output_pin self-enables the port clock, so this write takes.
    let mut off_latch = chip.output_pin(PeriphLabel::Gpioa, 5).unwrap();
    let _ = off_latch.set_high();

    // RIGHT motor on TIMER0. bring_up_motor enables the timer clock, configures it, routes the gates,
    // and starts the counter (disarmed: outputs do not reach the pins until MOE is set). It does NOT
    // arm; the bridge is energized explicitly below.
    let (mut comm_r, gate_r, halls_r) = bring_up_motor(
        &chip,
        &DUAL_MOTOR_RIGHT,
        PeriphLabel::Timer0,
        PWM_PERIOD,
        SixStep::new(DIRECTION, ALIGN_OFFSET_RIGHT),
    )
    .unwrap();

    // LEFT motor on TIMER7 (detect_chip populates Timer7's base when a 2nd advanced timer is measured;
    // on a single-advanced-timer part this fails loud). Same bring-up, its own contract / halls.
    let (mut comm_l, gate_l, halls_l) = bring_up_motor(
        &chip,
        &DUAL_MOTOR_LEFT,
        PeriphLabel::Timer7,
        PWM_PERIOD,
        SixStep::new(DIRECTION, ALIGN_OFFSET_LEFT),
    )
    .unwrap();

    let mut delay = runtime_hal::Delay::new(cp.SYST, SYSCLK_HZ);

    // SAFETY: arm BOTH bridges (MOE on). All twelve gates are now live; current-limit the supply.
    gate_r.arm();
    gate_l.arm();

    let mut seq: u32 = 0;
    loop {
        // Apply the SWD-writable duty override (shared) + per-motor alignment overrides.
        let mut duty = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(DUTY_OVERRIDE)) };
        if duty > PWM_PERIOD {
            duty = PWM_PERIOD;
        }
        let off = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ALIGN_OVERRIDE)) };
        comm_r.set_offset(off);
        let off_l = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ALIGN_OVERRIDE_LEFT)) };
        comm_l.set_offset(off_l);

        let code = halls_r.read();
        let applied = comm_r.apply(code, duty).unwrap_or(false);
        let code_l = halls_l.read();
        let applied_l = comm_l.apply(code_l, duty).unwrap_or(false);
        seq = seq.wrapping_add(1);
        // SWD readback: right motor + left motor.
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(COMMUT_OBS_LEFT),
                CommutObs {
                    magic: OBS_MAGIC,
                    seq,
                    hall_code: u32::from(code_l),
                    applied: applied_l as u32,
                    offset: u32::from(off_l),
                },
            );
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(COMMUT_OBS),
                CommutObs {
                    magic: OBS_MAGIC,
                    seq,
                    hall_code: u32::from(code),
                    applied: applied as u32,
                    offset: u32::from(off),
                },
            );
        }
        delay.delay_us(LOOP_US);
    }
}
