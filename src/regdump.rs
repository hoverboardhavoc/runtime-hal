//! Read-only register-dump surface (G8): the verification gate's capture.
//!
//! The per-cycle [`crate::timer::PwmHandle`] deliberately exposes ONLY the compare writes (no
//! CCHP/MOE), which is correct for the control path but leaves the verification gate with no
//! first-class way to read back the full configured register set for the section-3 register-
//! equivalence diff. This module is that read-only capture: [`RegDumpConfig::dump`] reads the TIMER0
//! advanced-timer block and the timer-triggered injected-ADC block into a plain `Copy` value type so
//! a host golden diff or an on-target SWD read compares against the SAME fields.
//!
//! # Read-only, never an MOE writer (SAFETY)
//!
//! Every field here is a [`Reg32::read`] result. CCHP is READ (so the diff can confirm MOE is CLEAR,
//! [`TimerRegs::moe`]), but this type has NO method that writes CCHP or any other register.
//! [`crate::timer::arming::ArmGate`] remains the sole MOE writer (DECISIONS.md #4 + the M3 SAFETY
//! section). Reading registers with MOE off and no drain supply is electrically harmless, so the
//! dump is trivially safe under the M3 SAFETY rules.
//!
//! # Family independence
//!
//! The advanced-timer and ADC register blocks are identical on the F10x and F1x0 (one model
//! parameterised by base, exactly as [`crate::timer`] and [`crate::adc`] already rely on), so there
//! is no family branch on the timer/ADC capture. The GPIO input/gate pin fields DO differ by family
//! (the CRL/CRH nibble vs CTL/AFSEL/OMODE/OSPD/PUD split); normalising those into a family-neutral
//! struct is a separate follow-up tied to the verification-gate example, not part of this capture.
//!
//! [`Reg32::read`]: crate::reg::Reg32::read

use crate::reg::Reg32;

// --- TIMER0 register offsets (the bring-up + per-cycle set, identical on both families) ---------
//
// These mirror the offsets `crate::timer` already documents and writes; the dump reads the same
// locations so the captured value diffs field-for-field against the bring-up golden.

const TIMER_CTL0: u32 = 0x00;
const TIMER_CTL1: u32 = 0x04;
const TIMER_SMCFG: u32 = 0x08;
const TIMER_DMAINTEN: u32 = 0x0C;
const TIMER_CHCTL0: u32 = 0x18;
const TIMER_CHCTL1: u32 = 0x1C;
const TIMER_CHCTL2: u32 = 0x20;
const TIMER_PSC: u32 = 0x28;
const TIMER_CAR: u32 = 0x2C;
const TIMER_CREP: u32 = 0x30;
const TIMER_CH0CV: u32 = 0x34;
const TIMER_CCHP: u32 = 0x44;

/// MOE (main output enable) bit in CCHP (`TIMER_CCHP_POEN`, bit 15). The dump reads it so the gate
/// can confirm the bridge is DISARMED; the dump never writes it.
const CCHP_MOE: u32 = 1 << 15;

// --- injected-ADC register offsets (identical on both families; see `crate::adc`) ---------------

const ADC_CTL0: u32 = 0x04;
const ADC_CTL1: u32 = 0x08;
const ADC_SAMPT0: u32 = 0x0C;
const ADC_SAMPT1: u32 = 0x10;
const ADC_ISQ: u32 = 0x38;

/// A read-only snapshot of the advanced-timer (TIMER0) configuration registers.
///
/// Every field is the raw value read from the timer block at dump time. The four `chxcv` compare
/// values are the per-cycle duties (CH0/1/2) plus the ADC-trigger compare (CH3). `cchp` carries the
/// dead-time / break / off-state word INCLUDING MOE; use [`Self::moe`] to test the arm bit. No field
/// is writable: this is a capture, not a control surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerRegs {
    /// CTL0: CEN / DIR / center-align mode / ARSE / CKDIV.
    pub ctl0: u32,
    /// CTL1: master-mode TRGO (MMC) + the per-channel idle states (ISOx/ISOxN).
    pub ctl1: u32,
    /// SMCFG: slave-mode / trigger-input config (the reference leaves this at reset).
    pub smcfg: u32,
    /// DMAINTEN: the update / channel / break interrupt + DMA enables.
    pub dmainten: u32,
    /// CHCTL0: CH0/CH1 mode-select + output-compare mode + compare-shadow enable.
    pub chctl0: u32,
    /// CHCTL1: CH2/CH3 mode-select + output-compare mode + compare-shadow enable.
    pub chctl1: u32,
    /// CHCTL2: per-channel output enable + polarity (main CHxEN/CHxP, complementary CHxNEN/CHxNP).
    pub chctl2: u32,
    /// PSC: prescaler.
    pub psc: u32,
    /// CAR: counter auto-reload (the PWM period).
    pub car: u32,
    /// CREP: repetition counter.
    pub crep: u32,
    /// CH0CV..CH3CV: the three channel duties (0..2) and the ADC-trigger compare (3).
    pub chxcv: [u32; 4],
    /// CCHP: dead-time / break / off-state / protect word, INCLUDING the MOE arm bit (see
    /// [`Self::moe`]).
    pub cchp: u32,
}

impl TimerRegs {
    /// True if MOE (the main-output-enable arm bit) is SET in the captured CCHP. The verification
    /// gate asserts this is `false` on a configured-but-disarmed bridge (MOE is the
    /// [`crate::timer::arming`] layer's sole responsibility; a dump showing it set after config-only
    /// is a SAFETY violation the gate must catch).
    #[inline]
    pub const fn moe(&self) -> bool {
        self.cchp & CCHP_MOE != 0
    }
}

/// A read-only snapshot of the timer-triggered injected-ADC configuration registers.
///
/// Every field is the raw value read at dump time. `ctl1` carries the data alignment (DAL), the
/// injected external-trigger source (ETSIC) and its enable (ETEIC); `ctl0` carries the injected
/// end-of-conversion interrupt enable (EOICIE); `isq` the injected sequence length + channel ranks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdcInjectedRegs {
    /// CTL0: the injected end-of-conversion interrupt enable (EOICIE, bit 7) + scan mode.
    pub ctl0: u32,
    /// CTL1: ADCON / data alignment (DAL) / injected external-trigger source (ETSIC) + enable
    /// (ETEIC).
    pub ctl1: u32,
    /// SAMPT0: sample-time fields for channels 10..17.
    pub sampt0: u32,
    /// SAMPT1: sample-time fields for channels 0..9.
    pub sampt1: u32,
    /// ISQ: injected sequence length (IL) + the injected channel ranks (ISQN).
    pub isq: u32,
}

/// A read-only capture of the per-cycle-path configuration: the TIMER0 advanced-timer block and the
/// timer-triggered injected-ADC block, read into a plain `Copy` value type.
///
/// Built with [`RegDumpConfig::dump`] from the resolved timer + ADC bases (the same bases the
/// [`crate::timer::PwmHandle`] / [`crate::adc::InjectedHandle`] hold). It holds ONLY register
/// reads; it can never arm the bridge or write any register. The verification gate diffs this against
/// the expected configured state (and asserts [`TimerRegs::moe`] is clear); a bench SWD read produces
/// the same fields for an on-target diff.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegDumpConfig {
    /// The advanced-timer (TIMER0) register snapshot.
    pub timer: TimerRegs,
    /// The injected-ADC register snapshot.
    pub adc_injected: AdcInjectedRegs,
}

impl RegDumpConfig {
    /// Read the per-cycle-path configuration registers from the resolved `timer_base` (the advanced
    /// timer) and `adc_base` (the injected ADC) into a snapshot. Pure reads, no writes, no MOE: safe
    /// to call at any time (the M3 SAFETY rules: reading with MOE off and no drain supply is harmless).
    ///
    /// The bases are HAL-internal; application code calls [`crate::chip::Chip::dump_config`], which
    /// resolves the timer + ADC labels from the descriptor and calls this, matching the resolve-once
    /// handle pattern (DECISIONS.md #4) so a caller never holds a raw base.
    #[must_use]
    pub fn dump(timer_base: u32, adc_base: u32) -> RegDumpConfig {
        RegDumpConfig {
            timer: TimerRegs::dump(timer_base),
            adc_injected: AdcInjectedRegs::dump(adc_base),
        }
    }
}

impl TimerRegs {
    /// Read the advanced-timer register block at `timer_base` into a snapshot (pure reads).
    #[must_use]
    pub fn dump(timer_base: u32) -> TimerRegs {
        let r = |off: u32| Reg32::new(timer_base, off).read();
        TimerRegs {
            ctl0: r(TIMER_CTL0),
            ctl1: r(TIMER_CTL1),
            smcfg: r(TIMER_SMCFG),
            dmainten: r(TIMER_DMAINTEN),
            chctl0: r(TIMER_CHCTL0),
            chctl1: r(TIMER_CHCTL1),
            chctl2: r(TIMER_CHCTL2),
            psc: r(TIMER_PSC),
            car: r(TIMER_CAR),
            crep: r(TIMER_CREP),
            chxcv: [
                r(TIMER_CH0CV),
                r(TIMER_CH0CV + 4),
                r(TIMER_CH0CV + 8),
                r(TIMER_CH0CV + 12),
            ],
            cchp: r(TIMER_CCHP),
        }
    }
}

impl AdcInjectedRegs {
    /// Read the injected-ADC register block at `adc_base` into a snapshot (pure reads).
    #[must_use]
    pub fn dump(adc_base: u32) -> AdcInjectedRegs {
        let r = |off: u32| Reg32::new(adc_base, off).read();
        AdcInjectedRegs {
            ctl0: r(ADC_CTL0),
            ctl1: r(ADC_CTL1),
            sampt0: r(ADC_SAMPT0),
            sampt1: r(ADC_SAMPT1),
            isq: r(ADC_ISQ),
        }
    }
}

#[cfg(test)]
mod tests;
