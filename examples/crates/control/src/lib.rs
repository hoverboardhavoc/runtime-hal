//! 6-step (block / trapezoidal) hall commutation decode + board commutation contracts.
//!
//! This is the small `control` layer above `runtime-hal`: it owns the DECODE (a hall sensor code ->
//! which phase sources, which sinks, which floats), which the HAL deliberately does NOT. The HAL gives
//! only the silicon mechanism: [`runtime_hal::PwmHandle`] (set duties + per-channel output enable) and
//! [`runtime_hal::InputGroup`] (a neutral multi-pin GPIO input read). This crate adds the motor
//! meaning: [`PhaseDrive`], the [`SixStep`] hall decode, and the [`Commutator`] that drives a
//! `PwmHandle` from a hall code.
//!
//! It also defines the [`MotorContract`] TYPE (a board's hall pins, gate pins, and dead-time); the
//! per-board contract CONSTANTS live in the applications that use them (the commutation examples), not
//! here, since they are board data, not reusable logic.
//!
//! `no_std` and host-testable: nothing here touches a register (the decode is pure logic over
//! [`PhaseDrive`]).
//!
//! # 6-step commutation in one paragraph
//!
//! A 3-phase BLDC with hall sensors reports rotor position as a 3-bit code (one bit per hall line).
//! Of the eight codes, six are valid (the two all-low / all-high codes are a sensor fault). Each
//! valid code selects one of six commutation STATES: one phase driven by the chopping PWM (high
//! side), one phase sinking the return current (low side on), and one phase floating. Advancing
//! through the six states as the rotor turns produces continuous torque; the hall code tells us which
//! state the rotor is in, so the motor self-commutates with no open-loop ramp.
//!
//! # The alignment offset (read before spinning a real motor)
//!
//! Which hall code maps to which commutation state depends on the motor's hall placement and phase
//! wiring. There are six rotational alignments and two directions; only one alignment per direction
//! produces smooth forward rotation, the others cog or run backward. [`SixStep`] takes an `offset`
//! (0..5) so the working alignment is found empirically on the bench (sweep the offset until the
//! motor spins smoothly). The decode itself is correct; the offset is the motor-specific calibration.

#![no_std]

use runtime_hal::{PwmError, PwmHandle};

/// Per-phase drive action for one 6-step (block) commutation step. This is MOTOR-CONTROL vocabulary,
/// it lives here, not in the HAL: the silicon only knows "channel output enabled/disabled" and "set a
/// compare value". [`Commutator`] translates these into the HAL's [`PwmHandle::set_channel_outputs`]
/// + [`PwmHandle::set_duties`] silicon calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseDrive {
    /// Source: high-side chops at the step duty (channel enabled, compare = duty).
    Pwm,
    /// Sink: the current return path (channel enabled, compare = 0, so the complementary low-side is
    /// on).
    Sink,
    /// Floating: the phase is electrically off (channel output disabled).
    Float,
}

/// Rotation direction. Reversing swaps each state's source and sink phases (a half-turn, +3, through
/// the six-state sequence), keeping the same floating phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// The forward electrical sequence.
    Forward,
    /// The reverse sequence (source and sink swapped).
    Reverse,
}

/// The six commutation states in electrical order. State `i` and state `i + 3` are mirror images
/// (source and sink swapped, same float), which is why [`Direction::Reverse`] is `+3`.
///
/// Phase order is `[A, B, C]`. The sequence rotates the PWM (source) phase and the sink phase around
/// the three phases, floating the third each step.
const STATES: [[PhaseDrive; 3]; 6] = {
    use PhaseDrive::{Float, Pwm, Sink};
    [
        [Pwm, Sink, Float],
        [Pwm, Float, Sink],
        [Float, Pwm, Sink],
        [Sink, Pwm, Float],
        [Sink, Float, Pwm],
        [Float, Sink, Pwm],
    ]
};

/// Hall 3-bit code (0..7) -> commutation sector (0..5), or [`INVALID`] for the two fault codes.
///
/// This is the canonical 120-degree hall ordering (code = `h_a | h_b << 1 | h_c << 2`): codes 0 and
/// 7 (all hall lines low / high) are impossible for a healthy sensor set and decode to a fault. The
/// per-motor rotation of this table is applied by [`SixStep`]'s `offset`, so this base table is the
/// same for every motor.
const HALL_TO_SECTOR: [u8; 8] = [INVALID, 0, 2, 1, 4, 5, 3, INVALID];

/// Sentinel for an invalid hall code in [`HALL_TO_SECTOR`].
const INVALID: u8 = 0xFF;

/// The 6-step hall commutation decoder: direction + the motor-specific alignment offset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SixStep {
    direction: Direction,
    offset: u8,
}

impl SixStep {
    /// A decoder for `direction` with the motor alignment `offset` (taken mod 6). Find the working
    /// offset empirically on the bench (sweep 0..5 until the motor spins smoothly; see the module
    /// docs).
    #[inline]
    pub const fn new(direction: Direction, offset: u8) -> Self {
        Self {
            direction,
            offset: offset % 6,
        }
    }

    /// The configured direction.
    #[inline]
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// The configured alignment offset (0..5).
    #[inline]
    pub const fn offset(&self) -> u8 {
        self.offset
    }

    /// Decode a 3-bit hall `code` into the per-phase commutation pattern, or `None` if the code is a
    /// sensor fault (0 or 7, all-low / all-high). Only the low three bits of `code` are used.
    ///
    /// The result feeds [`Commutator::apply`] / the HAL silicon calls: `[A, B, C]` phase drives.
    #[inline]
    pub fn pattern(&self, code: u8) -> Option<[PhaseDrive; 3]> {
        let sector = HALL_TO_SECTOR[(code & 0x7) as usize];
        if sector == INVALID {
            return None;
        }
        let half = match self.direction {
            Direction::Forward => 0,
            Direction::Reverse => 3,
        };
        let index = ((sector + self.offset + half) % 6) as usize;
        Some(STATES[index])
    }

    /// True if `code` is one of the six valid hall codes (not a sensor-fault 0 / 7).
    #[inline]
    pub fn is_valid_code(code: u8) -> bool {
        HALL_TO_SECTOR[(code & 0x7) as usize] != INVALID
    }
}

/// Drives a [`PwmHandle`] from hall codes using a [`SixStep`] decode: the control-layer object that
/// turns "the rotor is at hall code X" into the HAL's silicon calls
/// ([`PwmHandle::set_channel_outputs`] + [`PwmHandle::set_duties`]). `Copy` (it holds the resolved
/// handle + the decoder). It does NOT arm the bridge (that is the HAL `ArmGate`'s job): a commutation
/// bug here can only float phases or write duties, never energize a disarmed bridge.
#[derive(Debug, Clone, Copy)]
pub struct Commutator {
    handle: PwmHandle,
    step: SixStep,
}

impl Commutator {
    /// Build from a resolved HAL PWM handle and a decoder.
    #[inline]
    pub const fn new(handle: PwmHandle, step: SixStep) -> Self {
        Self { handle, step }
    }

    /// Apply the commutation step for hall `code` at `duty` (the high-side chop compare).
    ///
    /// On a valid code: floats the floating phase (disables its outputs), enables the PWM + sink
    /// phases, and writes the duties (PWM phase = `duty`, others = 0); returns `Ok(true)`. On a
    /// sensor-fault code (0 or 7): disables ALL outputs so the motor coasts, and returns `Ok(false)`.
    /// A `duty` above the configured period is [`PwmError::DutyOutOfRange`] (checked before any
    /// enable change). The bridge is never armed/disarmed here.
    pub fn apply(&self, code: u8, duty: u16) -> Result<bool, PwmError> {
        match self.step.pattern(code) {
            Some(pattern) => {
                let mut duties = [0u16; 3];
                let mut enabled = [false; 3];
                for (i, drive) in pattern.iter().enumerate() {
                    enabled[i] = !matches!(drive, PhaseDrive::Float);
                    duties[i] = if matches!(drive, PhaseDrive::Pwm) {
                        duty
                    } else {
                        0
                    };
                }
                // Range-check + write the compares first, then gate the outputs. set_channel_outputs
                // does not run if the duty is rejected, so a bad duty cannot change which phases drive.
                self.handle.set_duties(duties)?;
                self.handle.set_channel_outputs(enabled);
                Ok(true)
            }
            None => {
                // Sensor fault: coast. All phase outputs off (the bridge stays armed but floats).
                self.handle.set_channel_outputs([false, false, false]);
                Ok(false)
            }
        }
    }

    /// The decoder this commutator uses.
    #[inline]
    pub const fn step(&self) -> SixStep {
        self.step
    }
}

/// One motor's reverse-engineered commutation contract: the hall input pins, the gate output pins,
/// and the shoot-through-safe dead-time, as recovered from the stock firmware (see the BalanceAgain
/// `findings/*_commutation_contract.md` and the project memory).
///
/// Pins are logical `(port << 4) | pin` bytes (port A = 0, B = 1, C = 2), the same encoding
/// [`runtime_hal`] uses. `hall_pins` is in hall-line order `[A, B, C]`; `gate_high` / `gate_low` are
/// in phase order `[A, B, C]` (high-side CHx / complementary low-side CHxN).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorContract {
    /// Hall input pins, hall-line order `[A, B, C]`.
    pub hall_pins: [u8; 3],
    /// High-side gate pins (TIMER CHx), phase order `[A, B, C]`.
    pub gate_high: [u8; 3],
    /// Complementary low-side gate pins (TIMER CHxN), phase order `[A, B, C]`.
    pub gate_low: [u8; 3],
    /// Dead-time field code (the TIMER CCHP DTCFG count) recovered from the stock firmware. The
    /// absolute dead-time scales with the timer clock; at a slower example clock the same count is a
    /// LONGER (safer) dead-time, never a shorter one.
    pub dead_time: u8,
}

impl MotorContract {
    /// Encode a `(port << 4) | pin` byte for the given `port` (A = 0, B = 1, C = 2, D = 3) and `pin`
    /// (0..15), the same encoding [`runtime_hal`] uses. A `const fn` so a board's contract can be a
    /// `const` (the per-board contract constants live in the commutation examples).
    pub const fn pin(port: u8, pin: u8) -> u8 {
        (port << 4) | pin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every valid hall code (1..=6) decodes to a pattern; the two fault codes (0, 7) decode to None.
    #[test]
    fn valid_codes_decode_fault_codes_are_none() {
        let s = SixStep::new(Direction::Forward, 0);
        assert!(s.pattern(0).is_none());
        assert!(s.pattern(7).is_none());
        for code in 1..=6u8 {
            assert!(s.pattern(code).is_some(), "code {code} should decode");
        }
    }

    /// Each commutation state has exactly one PWM (source), one Sink, and one Float phase: a
    /// well-formed block-commutation step never doubles a role or shorts the bus.
    #[test]
    fn every_state_has_one_pwm_one_sink_one_float() {
        for state in STATES {
            let pwm = state.iter().filter(|d| matches!(d, PhaseDrive::Pwm)).count();
            let sink = state.iter().filter(|d| matches!(d, PhaseDrive::Sink)).count();
            let float = state
                .iter()
                .filter(|d| matches!(d, PhaseDrive::Float))
                .count();
            assert_eq!((pwm, sink, float), (1, 1, 1), "malformed state {state:?}");
        }
    }

    /// The six valid hall codes map to six DISTINCT commutation states (a full electrical revolution
    /// with no repeated or skipped sector).
    #[test]
    fn six_codes_cover_six_distinct_states() {
        let s = SixStep::new(Direction::Forward, 0);
        let mut seen = [false; 6];
        for code in 1..=6u8 {
            let pat = s.pattern(code).unwrap();
            let idx = STATES.iter().position(|st| *st == pat).unwrap();
            assert!(!seen[idx], "state {idx} repeated");
            seen[idx] = true;
        }
        assert!(seen.iter().all(|&b| b), "not all six states were produced");
    }

    /// Reverse is the per-state mirror of forward: the same float phase, with PWM and Sink swapped.
    #[test]
    fn reverse_swaps_source_and_sink_keeps_float() {
        let fwd = SixStep::new(Direction::Forward, 0);
        let rev = SixStep::new(Direction::Reverse, 0);
        for code in 1..=6u8 {
            let f = fwd.pattern(code).unwrap();
            let r = rev.pattern(code).unwrap();
            for phase in 0..3 {
                match f[phase] {
                    PhaseDrive::Float => assert_eq!(r[phase], PhaseDrive::Float),
                    PhaseDrive::Pwm => assert_eq!(r[phase], PhaseDrive::Sink),
                    PhaseDrive::Sink => assert_eq!(r[phase], PhaseDrive::Pwm),
                }
            }
        }
    }

    /// The alignment offset rotates the hall->state mapping (offset N shifts every code's state by N).
    #[test]
    fn offset_rotates_the_state_mapping() {
        let base = SixStep::new(Direction::Forward, 0);
        let shifted = SixStep::new(Direction::Forward, 2);
        for code in 1..=6u8 {
            let b = base.pattern(code).unwrap();
            let s = shifted.pattern(code).unwrap();
            let bi = STATES.iter().position(|st| *st == b).unwrap();
            let si = STATES.iter().position(|st| *st == s).unwrap();
            assert_eq!(si, (bi + 2) % 6, "offset did not rotate code {code}");
        }
    }

    /// The pin encoder packs `(port << 4) | pin` (spot-check PC13 / PA1 and a gate pin). The per-board
    /// contract CONSTANTS live in the examples, not here.
    #[test]
    fn pin_encoder_packs_port_and_pin() {
        assert_eq!(MotorContract::pin(2, 13), 0x2D); // PC13
        assert_eq!(MotorContract::pin(0, 1), 0x01); // PA1
        assert_eq!(MotorContract::pin(0, 8), 0x08); // PA8 (a gate)
    }
}
