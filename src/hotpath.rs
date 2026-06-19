//! Hot-path peripheral traits and resolve-once concrete handles (M3 T1).
//!
//! SPEC.md "Hot path: runtime-hal's own peripheral traits": `embedded-hal` cannot express the
//! motor hot path because the FOC loop needs the timer's compare event to **trigger the ADC's
//! injected conversion** so phase currents are sampled at the PWM centre, a coupling **between two
//! peripherals** plus an MCU-specific trigger matrix. So runtime-hal defines its own traits:
//! [`ComplementaryPwm`] (the advanced-timer complementary bridge + the ADC-trigger compare channel)
//! and [`TriggeredAdc`] (the timer-triggered injected conversion group).
//!
//! # Resolve-once concrete handle (DECISIONS.md #4)
//!
//! Each trait's config method (run ONCE at bring-up, consuming the T3/T4/T6/T8 config) returns a
//! concrete per-cycle **handle** ([`PwmHandle`], [`InjectedHandle`]). The handle is `Copy`, holds
//! resolved register accessors (no `dyn`, no descriptor lookup per call), and exposes the tiny
//! per-cycle methods: `set_duties([u16; 3])` + `rearm_trigger(..)` on the PWM handle,
//! `read_injected() -> ..` on the injected handle. The runtime path selectors are resolved once
//! into the handle, so there is **no per-call branch in the PWM-rate ISR**.
//!
//! # MOE is NOT on the handle (DECISIONS.md #4 + the SAFETY section)
//!
//! The main-output-enable (MOE) arming gate is deliberately absent from the per-cycle handle. It
//! lives in the safety/arming layer ([`arming`]) as a separate, distinct call owned by the
//! rider-power state machine, so a control-loop bug that only has the handle **cannot energize a
//! disarmed bridge**. The handle can write duties and re-arm the trigger; it cannot arm. This is a
//! SAFETY invariant, not an API nicety, and is enforced by a host test
//! (the `hotpath` test module): the handle's methods touch no MOE bit.
//!
//! # T1 scope
//!
//! This module commits the trait SIGNATURES and the concrete handle TYPES so the later tasks
//! (T3-T9) fill the bodies against a fixed shape. The config-method and per-cycle bodies are
//! stubbed (`unimplemented!()` / minimal register math) here; T3-T5 flesh out the PWM config +
//! `set_duties`/`rearm_trigger`, T6-T9 the trigger output + injected config + `read_injected`. No
//! timer/ADC peripheral is enabled by this module (the substrate is T2; T8 first carries the
//! control loop).

use crate::adc::{Adc, ETSIC_T0_CH3, ETSIC_T0_TRGO};
use crate::chip::Chip;
use crate::config::{InjectedAdcConfig, PwmConfig, TimerTriggerLink};
use crate::descriptor::{MAX_INJECTED_CHANNELS, MAX_PWM_CHANNELS};
use crate::error::{AdcError, HotPathError, PwmError};
use crate::reg::Reg32;
use crate::timer::PwmTimer;

// --- TIMER0 register offsets (identical on both families; see addr.rs ADV_TIMER_APB2 note) ------
//
// Confirmed against the GD SPL peripheral headers (gd32f10x_timer.h / gd32f1x0_timer.h): the
// advanced-timer register block is the same offsets on F10x and F1x0, so one model parameterised by
// base (data, from the AddrTable). The hot path touches the compare-value registers per cycle.

/// TIMER0 channel-0 capture/compare value register (CH0CV), the high-side phase-0 duty.
pub(crate) const TIMER_CH0CV: u32 = 0x34;
/// TIMER0 channel-1 capture/compare value register (CH1CV).
pub(crate) const TIMER_CH1CV: u32 = 0x38;
/// TIMER0 channel-2 capture/compare value register (CH2CV).
pub(crate) const TIMER_CH2CV: u32 = 0x3C;
/// TIMER0 channel-3 capture/compare value register (CH3CV), the ADC-trigger compare.
pub(crate) const TIMER_CH3CV: u32 = 0x40;
/// TIMER0 counter auto-reload register (CAR/ARR), the PWM period; the duty clamp references it.
/// (Consumed by the T3 timer-base config; pinned + conformance-checked here at T1.)
#[allow(dead_code)]
pub(crate) const TIMER_CAR: u32 = 0x2C;
/// TIMER0 complementary-channel protection register (CCHP), which holds MOE (bit 15). Owned by the
/// [`arming`] layer ONLY; the per-cycle handle never names this offset.
pub(crate) const TIMER_CCHP: u32 = 0x44;
/// MOE (main output enable) bit in CCHP (`TIMER_CCHP_POEN`, bit 15).
pub(crate) const CCHP_MOE: u32 = 1 << 15;

/// The advanced-timer complementary-PWM capability (M3 T5 fills the bodies). SPEC.md: configure
/// center-aligned PWM at a given period/prescaler, three complementary channel pairs with
/// dead-time, polarity/idle, optional break, and the MOE gate, plus the ADC-trigger compare
/// channel; per cycle set the three duties and re-arm the trigger compare. MOE arming is NOT here
/// (it is in [`arming`]).
pub trait ComplementaryPwm {
    /// The concrete per-cycle handle this config resolves into (DECISIONS.md #4).
    type Handle: Copy;

    /// Configure the advanced timer for the complementary bridge from the [`Chip`] (base + selector)
    /// and the code-level [`PwmConfig`] and return the resolve-once handle. Runs ONCE at bring-up;
    /// the selectors are resolved into the handle. Leaves MOE OFF (outputs disarmed): arming is a
    /// separate, deliberate [`arming`] call.
    fn configure(&self, chip: &Chip, cfg: &PwmConfig) -> Result<Self::Handle, HotPathError>;
}

/// The timer-triggered injected-ADC capability (M3 T9 fills the bodies). SPEC.md: an injected
/// conversion group triggered by the timer's trigger channel (sampled near the PWM centre), a small
/// channel list with per-channel sample time, left-aligned data, and an end-of-injected-conversion
/// interrupt that runs the control loop; read the injected results.
pub trait TriggeredAdc {
    /// The concrete per-cycle handle this config resolves into (DECISIONS.md #4).
    type Handle: Copy;

    /// Configure the injected conversion group from the [`Chip`] (base + selector) and the
    /// code-level [`InjectedAdcConfig`] (trigger source = the timer, channel sequence + sample
    /// times, left-aligned, EOIC enable) and return the resolve-once handle. Runs ONCE at bring-up.
    fn configure(&self, chip: &Chip, cfg: &InjectedAdcConfig)
        -> Result<Self::Handle, HotPathError>;
}

/// The advanced-timer complementary-PWM controller (resolve-once config object): a timer that
/// implements [`ComplementaryPwm`]. Its [`ComplementaryPwm::configure`] runs the timer bring-up
/// ([`crate::timer::PwmTimer`]) and resolves the four compare offsets + the period once into a
/// [`PwmHandle`]. MOE arming is NOT here (the [`arming::ArmGate`] is built separately from the same
/// base).
///
/// (Was `PwmConfig`; renamed to `PwmController` in the descriptor-rework so the code-level
/// [`crate::config::PwmConfig`] application config can take that name. This object carries the
/// resolved base + selectors via the [`Chip`]; the config it consumes is the behavior.)
///
/// The base is resolved at `configure` time from the [`Chip`] for the config's timer label, with the
/// advanced-timer-window check; a base outside the window (or a non-timer label, or a missing base)
/// surfaces as a [`crate::error::DescriptorError`] / [`PwmError`]. An already-resolved base can be
/// used directly ([`Self::arm_gate`]) for the arming-gate path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PwmController;

impl PwmController {
    /// A controller that resolves its timer base from the [`Chip`] at `configure` time.
    #[inline]
    pub const fn new() -> Self {
        PwmController
    }

    /// The MOE arming gate for a resolved timer base (the only MOE writer). A deliberately separate
    /// object from the per-cycle [`PwmHandle`]: the control loop gets the handle, the safety layer
    /// gets the gate. The firmware resolves the base via `chip.base(cfg.timer)?` and passes it here.
    #[inline]
    pub fn arm_gate(base: u32) -> arming::ArmGate {
        arming::ArmGate::new(base)
    }
}

impl ComplementaryPwm for PwmController {
    type Handle = PwmHandle;

    /// Run the advanced-timer bring-up (alignment/ARSE/CREP/CKDIV/per-side idle from `cfg`, three
    /// complementary pairs, dead-time, break) leaving MOE OFF, then resolve the four compare
    /// offsets + the period once into a [`PwmHandle`]. No per-call branch in the resulting handle.
    ///
    /// The CH3 ADC-trigger compare + the TRGO master-mode are a separate
    /// [`PwmTimer::configure_trigger`] step (so the phase-config golden stays CH3-untouched); the
    /// integrated hot-path bring-up runs both.
    fn configure(&self, chip: &Chip, cfg: &PwmConfig) -> Result<Self::Handle, HotPathError> {
        let timer = PwmTimer::configure(chip, cfg).map_err(HotPathError::Descriptor)?;
        Ok(PwmHandle::new(timer.base(), cfg.period))
    }
}

/// The resolve-once complementary-PWM per-cycle handle (DECISIONS.md #4).
///
/// `Copy`, concrete, no `dyn`: it holds the resolved [`Reg32`] accessors for the four compare
/// registers (the three phase duties + the trigger compare) and the period for the duty clamp. The
/// per-cycle methods ([`Self::set_duties`], [`Self::rearm_trigger`]) write straight to those
/// resolved registers with no descriptor lookup and no branch. It holds NO accessor for CCHP/MOE
/// (the arming gate is [`arming`]'s, not the handle's, a SAFETY invariant).
#[derive(Debug, Clone, Copy)]
pub struct PwmHandle {
    /// CH0CV / CH1CV / CH2CV accessors (the three phase high-side duties), resolved once.
    ch_cv: [Reg32; MAX_PWM_CHANNELS],
    /// CH3CV accessor (the ADC-trigger compare), resolved once.
    trig_cv: Reg32,
    /// The PWM period (CAR/ARR), used to clamp/validate duties so a compare never exceeds it.
    period: u16,
}

impl PwmHandle {
    /// Construct the handle from a resolved timer base + period (used by the T5 `configure` body and
    /// by host tests). Resolves the four compare-register accessors once; holds no MOE accessor.
    #[inline]
    pub fn new(timer_base: u32, period: u16) -> Self {
        Self {
            ch_cv: [
                Reg32::new(timer_base, TIMER_CH0CV),
                Reg32::new(timer_base, TIMER_CH1CV),
                Reg32::new(timer_base, TIMER_CH2CV),
            ],
            trig_cv: Reg32::new(timer_base, TIMER_CH3CV),
            period,
        }
    }

    /// Per-cycle: write the three phase duties to CH0CV/CH1CV/CH2CV. The ONLY per-cycle PWM write
    /// surface besides [`Self::rearm_trigger`]. Resolve-once: no descriptor lookup, no branch. A
    /// duty above the period is [`PwmError::DutyOutOfRange`] (it would never match in a
    /// center-aligned count). MOE is untouched (this cannot arm the bridge).
    #[inline]
    pub fn set_duties(&self, duties: [u16; MAX_PWM_CHANNELS]) -> Result<(), PwmError> {
        for &d in &duties {
            if d > self.period {
                return Err(PwmError::DutyOutOfRange);
            }
        }
        for (i, &d) in duties.iter().enumerate() {
            self.ch_cv[i].write(u32::from(d));
        }
        Ok(())
    }

    /// Per-cycle: re-arm the ADC-trigger compare (CH3CV). The reference re-writes this every PWM
    /// period so the injected sample stays at the PWM centre. Wired to the actual trigger channel
    /// in T7; the write surface is fixed here. MOE is untouched.
    #[inline]
    pub fn rearm_trigger(&self, compare: u16) -> Result<(), PwmError> {
        if compare > self.period {
            return Err(PwmError::DutyOutOfRange);
        }
        self.trig_cv.write(u32::from(compare));
        Ok(())
    }

    /// The configured PWM period (CAR/ARR). Exposed so the control crate / arming layer can size
    /// duties; read-only.
    #[inline]
    pub const fn period(&self) -> u16 {
        self.period
    }
}

/// The timer-triggered injected-ADC config object (M3 T8/T9): a resolved single-ADC base that
/// implements [`TriggeredAdc`]. Its [`TriggeredAdc::configure`] runs the T8 injected bring-up
/// ([`crate::adc::Adc::configure_injected`] + calibration) and resolves the injected-data offsets +
/// the channel count once into an [`InjectedHandle`].
///
/// The base is resolved at `configure` time from the [`Chip`] for the config's ADC label, with the
/// ADC-window check. The wiring it consumes carries only the logical config (channel list, sample
/// times, alignment, the timer-trigger link).
///
/// DECISIONS.md #9: single ADC first (the F1x0 baseline). The F10x dual / simultaneous arm is T11.
/// The timer-triggered injected-ADC controller (resolve-once config object): a single-ADC that
/// implements [`TriggeredAdc`]. Its [`TriggeredAdc::configure`] runs the injected bring-up
/// ([`crate::adc::Adc::configure_injected`] + calibration) and resolves the injected-data offsets +
/// the channel count once into an [`InjectedHandle`].
///
/// (Was `InjectedAdcConfig`; renamed to `InjectedAdcController` so the code-level
/// [`crate::config::InjectedAdcConfig`] application config can take that name.) The ADC base is
/// resolved at `configure` time from the [`Chip`] for the config's ADC label, with the ADC-window
/// check.
///
/// DECISIONS.md #9: single ADC first (the F1x0 baseline). The F10x dual / simultaneous arm is later.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InjectedAdcController;

impl InjectedAdcController {
    /// A controller that resolves its ADC base from the [`Chip`] at `configure` time.
    #[inline]
    pub const fn new() -> Self {
        InjectedAdcController
    }
}

/// Map the logical timer-trigger link (HP-5) to the raw ADC injected external-trigger source
/// (ETSIC) field value. The reference triggers off TIMER0 (the only advanced timer the board uses);
/// on both families TIMER0 CH3 = 1 and TIMER0 TRGO = 0. A non-TIMER0 trigger timer is not
/// expressible on the single-ADC F1x0 baseline (the ETSIC matrix has no slot for it), so it is
/// rejected at config (`None`).
#[inline]
fn etsic_for_link(cfg: &InjectedAdcConfig) -> Option<u32> {
    use crate::addr::PeriphLabel;
    if cfg.trigger_timer != PeriphLabel::Timer0 {
        return None;
    }
    Some(match cfg.trigger_link {
        TimerTriggerLink::Ch3 => ETSIC_T0_CH3,
        TimerTriggerLink::Trgo => ETSIC_T0_TRGO,
    })
}

impl TriggeredAdc for InjectedAdcController {
    type Handle = InjectedHandle;

    /// Run the T8 injected bring-up (data alignment, injected sequence length + channel list with
    /// per-channel sample time, the timer external-trigger source derived from the HP-5 link, the
    /// EOIC interrupt enable, ADC enable, then the calibration poll) and resolve the injected-data
    /// offsets + the channel count once into an [`InjectedHandle`]. No per-call branch in the handle.
    ///
    /// The injected-EOC ISR routing is already in place on the T2 substrate (the ADC vector calls
    /// the registered control handler); enabling EOICIE here is what makes that ISR fire at the PWM
    /// rate. The firmware registers its control handler via
    /// [`crate::irq::register_control_handler`] at boot.
    fn configure(
        &self,
        chip: &Chip,
        cfg: &InjectedAdcConfig,
    ) -> Result<Self::Handle, HotPathError> {
        // Resolve + range-check the ADC base from the chip (the config carries only behavior).
        chip.descriptor()
            .addrs
            .check_adc_base(cfg.adc)
            .map_err(HotPathError::Descriptor)?;
        let base = chip.base(cfg.adc).map_err(HotPathError::Descriptor)?;
        let len = cfg.channels.len();
        if len == 0 || len > MAX_INJECTED_CHANNELS {
            return Err(HotPathError::Adc(AdcError::Other));
        }
        let etsic = etsic_for_link(cfg).ok_or(HotPathError::Adc(AdcError::Other))?;
        // The injected channel list as (channel, sample_time) pairs in injected-rank order.
        let mut chans = [(0u8, 0u8); MAX_INJECTED_CHANNELS];
        for (i, c) in cfg.channels.iter().enumerate() {
            chans[i] = (c.channel, c.sample_time);
        }
        let dev = Adc::bring_up_injected(base, &chans[..len], cfg.left_aligned, etsic)?;
        Ok(InjectedHandle::new(dev.base(), len as u8))
    }
}

/// The resolve-once injected-ADC per-cycle handle (DECISIONS.md #4).
///
/// `Copy`, concrete, no `dyn`: it holds the resolved [`Reg32`] accessors for the injected data
/// registers and the channel count, so [`Self::read_injected`] reads the conversion results with no
/// descriptor lookup and no branch. The raw injected values are returned as-is (offset correction /
/// calibration is the `control` crate, out of M3).
#[derive(Debug, Clone, Copy)]
pub struct InjectedHandle {
    /// IDATA0..IDATA3 accessors (the injected data registers), resolved once.
    idata: [Reg32; MAX_INJECTED_CHANNELS],
    /// Number of valid injected channels (<= [`MAX_INJECTED_CHANNELS`]).
    len: u8,
}

/// ADC injected data registers (IDATA0..IDATA3) offsets, identical on both families (gd32*_adc.h:
/// `ADC_IDATA0..3` at 0x3C/0x40/0x44/0x48). T8/T9 fill the body; the offsets are pinned here.
const ADC_IDATA: [u32; MAX_INJECTED_CHANNELS] = [0x3C, 0x40, 0x44, 0x48];

impl InjectedHandle {
    /// Construct the handle from a resolved ADC base + injected channel count (used by the T9
    /// `configure` body and host tests). Resolves the injected-data accessors once.
    #[inline]
    pub fn new(adc_base: u32, len: u8) -> Self {
        Self {
            idata: [
                Reg32::new(adc_base, ADC_IDATA[0]),
                Reg32::new(adc_base, ADC_IDATA[1]),
                Reg32::new(adc_base, ADC_IDATA[2]),
                Reg32::new(adc_base, ADC_IDATA[3]),
            ],
            len,
        }
    }

    /// Per-cycle: read the injected conversion results in injected-rank order (left-aligned, raw).
    /// Resolve-once: no descriptor lookup, no branch. Returns a fixed-size array; entries beyond
    /// [`Self::len`] are read but the caller uses only `len` of them (the control crate knows the
    /// channel list). The body is filled in T9; the shape is fixed here.
    #[inline]
    pub fn read_injected(&self) -> [u16; MAX_INJECTED_CHANNELS] {
        let mut out = [0u16; MAX_INJECTED_CHANNELS];
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = self.idata[i].read() as u16;
        }
        out
    }

    /// The number of valid injected channels.
    #[inline]
    pub const fn len(&self) -> u8 {
        self.len
    }

    /// True if no injected channels are configured.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// The safety / arming layer (DECISIONS.md #4 + SPEC.md SAFETY). MOE (the main-output-enable arming
/// gate) is owned HERE, not on the per-cycle [`PwmHandle`], so a control-loop bug cannot energize a
/// disarmed bridge. The arming primitive is a separate, deliberately distinct call.
///
/// This is the boundary scaffold (T1); the body is a thin stub. The reference firmware confirms the
/// shape: MOE (`timer_primary_output_config`) is owned by the rider-power state machine and is
/// cleared on every latched fault, while the 16 kHz ISR only writes the compare values. The
/// disarm-on-fault path and the software safe-disarm-before-halt are finalized in T4/T10 (HP-6).
pub mod arming {
    use super::{CCHP_MOE, TIMER_CCHP};
    use crate::reg::Reg32;

    /// The MOE arming gate for an advanced timer (the only MOE writer). Distinct from the per-cycle
    /// [`super::PwmHandle`]; holds the CCHP accessor the handle deliberately does not.
    #[derive(Debug, Clone, Copy)]
    pub struct ArmGate {
        cchp: Reg32,
    }

    impl ArmGate {
        /// Construct the arming gate from the resolved timer base. Owned by the safety layer.
        #[inline]
        pub fn new(timer_base: u32) -> Self {
            Self {
                cchp: Reg32::new(timer_base, TIMER_CCHP),
            }
        }

        /// Arm the bridge: set MOE so the complementary outputs reach the pins. A deliberate,
        /// distinct call (NOT a per-cycle handle method). SAFETY: only call under the rider-power
        /// state machine with current limiting / a controlled bench setup; see the SAFETY section.
        #[inline]
        pub fn arm(&self) {
            self.cchp.modify(CCHP_MOE, CCHP_MOE);
        }

        /// Disarm the bridge: clear MOE so the outputs drop to their configured idle state. The
        /// software safe-disarm used on a latched fault and before any CPU halt with the bus
        /// energized.
        #[inline]
        pub fn disarm(&self) {
            self.cchp.modify(CCHP_MOE, 0);
        }
    }
}

/// Hall-sensor GPIO read primitive (M3 HP-9).
///
/// The reference reads the three hall lines as **plain GPIO inputs** (no timer hall mode, no EXTI):
/// the default lines are PC13 / PA1 / PC14, read into a 3-bit code each cycle. This module is the
/// raw read only: it samples the three input pins and packs them `(h2 << 2) | (h1 << 1) | h0`.
/// Debounce and the 6-step commutation decode are the `control` crate's job, not here.
///
/// The GPIO input-status register (`GPIO_ISTAT`) is at a **family-dependent offset**: 0x10 on the
/// F1x0 (AHB) GPIO and 0x08 on the F10x (APB) GPIO (verified against `gd32f1x0_gpio.h` /
/// `gd32f10x_gpio.h`). The offset is the only family divergence, so the resolve-once
/// [`hall::HallReader::resolve`] takes the [`crate::GpioPath`] and picks it; the per-cycle
/// [`hall::HallReader::read`] is then a branch-free three-pin read.
pub mod hall {
    use crate::descriptor::GpioPath;
    use crate::reg::Reg32;

    /// Number of hall lines (a 3-phase BLDC has three).
    pub const HALL_LINES: usize = 3;

    /// `GPIO_ISTAT` offset on the F1x0 (AHB) GPIO block.
    const ISTAT_AHB: u32 = 0x10;
    /// `GPIO_ISTAT` offset on the F10x (APB) GPIO block.
    const ISTAT_APB: u32 = 0x08;

    /// One hall line: the resolved GPIO-port base ISTAT accessor and the pin number within it.
    #[derive(Debug, Clone, Copy)]
    struct HallLine {
        istat: Reg32,
        pin: u8,
    }

    /// The resolve-once hall reader (DECISIONS.md #4 shape): holds the three lines' resolved ISTAT
    /// accessors + pin numbers, so [`Self::read`] is a branch-free three-pin sample. `Copy`,
    /// concrete, no `dyn`.
    #[derive(Debug, Clone, Copy)]
    pub struct HallReader {
        lines: [HallLine; HALL_LINES],
    }

    impl HallReader {
        /// Resolve the three hall lines from their `(port_base, pin)` pairs and the [`GpioPath`]
        /// (which picks the family's `GPIO_ISTAT` offset). The pairs are in hall-line order
        /// (line 0 -> code bit 0, etc.); the default board wiring is PC13 / PA1 / PC14.
        #[inline]
        pub fn resolve(path: GpioPath, lines: [(u32, u8); HALL_LINES]) -> Self {
            let istat_off = match path {
                GpioPath::AhbCtlAfsel => ISTAT_AHB,
                GpioPath::ApbCrlCrh => ISTAT_APB,
            };
            let mut out = [HallLine {
                istat: Reg32::new(0, istat_off),
                pin: 0,
            }; HALL_LINES];
            for (i, &(base, pin)) in lines.iter().enumerate() {
                out[i] = HallLine {
                    istat: Reg32::new(base, istat_off),
                    pin,
                };
            }
            Self { lines: out }
        }

        /// Per-cycle: sample the three hall input pins and pack them into a 3-bit code
        /// `(h2 << 2) | (h1 << 1) | h0`. Raw lines only (debounce/decode is the control crate).
        #[inline]
        pub fn read(&self) -> u8 {
            let mut code = 0u8;
            for (i, line) in self.lines.iter().enumerate() {
                let bit = (line.istat.read() >> u32::from(line.pin)) & 1;
                code |= (bit as u8) << i;
            }
            code
        }
    }
}

#[cfg(test)]
mod tests;
