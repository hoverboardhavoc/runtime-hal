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

use runtime_hal::{
    enable_timer, ArmGate, BreakConfig, Chip, ClockDiv, DescriptorError, InputGroup, OcMode,
    PeriphLabel, PwmAlign, PwmChannelConfig, PwmConfig, PwmError, PwmHandle, PwmTimer, TrgoSource,
};

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

    /// Build the stock-convention complementary-bridge [`PwmConfig`] for this contract on `timer`.
    ///
    /// This is the reusable BLDC bring-up config: it fills the fixed stock-convention fields (the
    /// values the commutation examples used to hardcode) from the contract's gate pins + dead-time,
    /// and takes `timer` and `period` as the clock-dependent knobs (`period` is the center-aligned
    /// CAR the application picks for its core clock; the prescaler is fixed at 0). It encodes:
    ///
    /// - one complementary channel per phase (high-side `gate_high[i]` / low-side `gate_low[i]`),
    ///   active-low (`polarity = true`) so the bridge idles safe, with both idle levels high;
    /// - the shoot-through-safe `dead_time` from the contract;
    /// - center-aligned mode 1 ([`PwmAlign::Center1`], the stock CMS=01), auto-reload-shadow on
    ///   (`arse = true`), timer clock
    ///   divided by two ([`ClockDiv::Div2`]), TRGO on Update ([`TrgoSource::Update`]);
    /// - no break input, and the CH3 trigger channel DISABLED (6-step uses no injected-ADC trigger).
    pub fn pwm_config(&self, timer: PeriphLabel, period: u16) -> PwmConfig {
        let ch = |high: u8, low: u8| PwmChannelConfig {
            high,
            low,
            // Inverted (active-low) complementary low side so the bridge idles safe (stock convention).
            polarity: true,
            idle_high: true,
            idle_high_n: true,
        };
        PwmConfig {
            timer,
            channels: [
                ch(self.gate_high[0], self.gate_low[0]),
                ch(self.gate_high[1], self.gate_low[1]),
                ch(self.gate_high[2], self.gate_low[2]),
            ],
            period,
            prescaler: 0,
            // The shoot-through-safe dead-time recovered from the stock firmware.
            dead_time: self.dead_time,
            brk: BreakConfig {
                enabled: false,
                level: false,
            },
            // 6-step uses NO injected ADC, so the CH3 trigger channel is unused / disabled.
            trigger_compare: 0,
            align: PwmAlign::Center1,
            arse: true,
            trigger_oc_mode: OcMode::Pwm0,
            trigger_ch_enable: false,
            crep: 0,
            ckdiv: ClockDiv::Div2,
            trgo_src: TrgoSource::Update,
        }
    }
}

/// Bring up one motor's advanced-timer complementary bridge from its contract and hand back the
/// per-motor trio the commutation examples drive: the [`Commutator`] (6-step decode + PWM handle),
/// the [`ArmGate`], and the [`InputGroup`] hall reader.
///
/// This is the motor-control bring-up orchestration (it DOES touch registers, unlike the pure decode
/// above), so it lives here in `control` as the BLDC bring-up layer. It:
///
/// 1. builds the config via [`MotorContract::pwm_config`] and [`PwmTimer::configure`] (the timer base
///    resolves from the runtime descriptor, no magic address);
/// 2. routes each gate pin with [`Chip::route_advanced_pwm_pin`] (the family-specific AF write is
///    absorbed by the HAL);
/// 3. starts the counter ([`PwmTimer::enable_counter`]), which is safe while disarmed (outputs do not
///    reach the pins until MOE is set);
/// 4. builds the [`Commutator`] over the PWM handle + a [`SixStep`] `step`, the [`ArmGate`], and the
///    hall [`InputGroup`] over the contract's hall pins.
///
/// It does NOT arm the bridge: arming (setting MOE on the returned `ArmGate`) is the application's
/// explicit, deliberate step, which keeps the energize point a visible safety boundary in the app.
///
/// The caller must have enabled the GPIO port clocks for the hall pins (the gate pins' clocks are
/// enabled by the routing). The advanced timer's OWN peripheral clock is enabled here (an unclocked
/// GD32 timer silently ignores all config writes and reads back zero), so the caller does not. Returns
/// [`DescriptorError`] if a pin's port or the timer base is absent from the descriptor (e.g. a second
/// advanced timer on a single-advanced-timer part: fail-loud).
pub fn bring_up_motor(
    chip: &Chip,
    c: &MotorContract,
    timer: PeriphLabel,
    period: u16,
    step: SixStep,
) -> Result<(Commutator, ArmGate, InputGroup), DescriptorError> {
    // Enable the timer's peripheral clock BEFORE configuring it: an unclocked GD32 advanced timer
    // ignores register writes and reads back zero, so `PwmTimer::configure` would be a no-op without
    // this (the cause of a bridge that never drives).
    enable_timer(chip.rcu_base()?, chip.clock(), timer)?;
    let cfg = c.pwm_config(timer, period);
    let pwm = PwmTimer::configure(chip, &cfg)?;
    for &gate in c.gate_high.iter().chain(c.gate_low.iter()) {
        chip.route_advanced_pwm_pin(gate)?;
    }
    pwm.enable_counter();
    let commutator = Commutator::new(pwm.handle(), step);
    let gate = pwm.arm_gate();
    // The HAL resolves each hall pin's port base internally; the caller never holds a base.
    let reader = chip.input_group(c.hall_pins)?;
    Ok((commutator, gate, reader))
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

    /// `pwm_config` fills the stock-convention complementary-bridge fields from the contract: the
    /// three channels carry the contract's gate pins (active-low, idle-high), the dead-time matches,
    /// the alignment is Center1, the clock divide is Div2, the timer/period knobs are passed through,
    /// and the (injected-ADC) trigger channel is disabled. This is the byte-for-byte config both
    /// commutation examples used to build inline.
    #[test]
    fn pwm_config_fills_stock_convention_fields() {
        let c = MotorContract {
            hall_pins: [MotorContract::pin(2, 13), MotorContract::pin(0, 1), MotorContract::pin(2, 14)],
            gate_high: [MotorContract::pin(0, 8), MotorContract::pin(0, 9), MotorContract::pin(0, 10)],
            gate_low: [MotorContract::pin(1, 13), MotorContract::pin(1, 14), MotorContract::pin(1, 15)],
            dead_time: 25,
        };
        let cfg = c.pwm_config(PeriphLabel::Timer0, 250);

        // The clock-dependent knobs are passed through; the prescaler is fixed at 0.
        assert_eq!(cfg.timer, PeriphLabel::Timer0);
        assert_eq!(cfg.period, 250);
        assert_eq!(cfg.prescaler, 0);

        // Each channel carries the contract's high/low gate pins, in phase order, active-low + idle-high.
        for i in 0..3 {
            assert_eq!(cfg.channels[i].high, c.gate_high[i], "channel {i} high gate");
            assert_eq!(cfg.channels[i].low, c.gate_low[i], "channel {i} low gate");
            assert!(cfg.channels[i].polarity, "channel {i} active-low");
            assert!(cfg.channels[i].idle_high, "channel {i} idle high");
            assert!(cfg.channels[i].idle_high_n, "channel {i} complementary idle high");
        }

        // The dead-time comes from the contract.
        assert_eq!(cfg.dead_time, c.dead_time);

        // The fixed stock-convention timer fields.
        assert_eq!(cfg.align, PwmAlign::Center1);
        assert_eq!(cfg.ckdiv, ClockDiv::Div2);
        assert!(cfg.arse);
        assert_eq!(cfg.trgo_src, TrgoSource::Update);

        // No break, and the injected-ADC trigger channel is disabled (6-step uses no trigger).
        assert!(!cfg.brk.enabled);
        assert!(!cfg.trigger_ch_enable);
    }
}
