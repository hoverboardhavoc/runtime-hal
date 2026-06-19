//! Free (independent) watchdog bring-up: the FWDGT / IWDG on the LSI/IRC40K.
//!
//! [`FreeWatchdog`] brings up the GD32 free watchdog timer (`FWDGT`, ST `IWDG`), the independent
//! reset-on-hang watchdog clocked from the always-available ~40 kHz internal RC oscillator
//! (IRC40K / LSI). A shipping firmware [`FreeWatchdog::start`]s it once with a generous timeout and
//! [`FreeWatchdog::feed`]s it every main-loop pass; if the loop hangs and the counter is not fed in
//! time, the watchdog resets the chip. This is the robustness half of firmware block B12
//! (`docs/firmware-readiness.md`): the stock firmware runs FWDGT at /256, reload 0x400 (~6.5 s) and
//! reloads it each pass; the hack firmware runs DIV16 / 0x0FFF.
//!
//! # One model, parameterised by base (no per-family register branch)
//!
//! The FWDGT register block is **identical on the F10x and F1x0** (verified against
//! `gd32f10x_fwdgt.h` / `gd32f1x0_fwdgt.h`: `CTL` 0x00, `PSC` 0x04, `RLD` 0x08, `STAT` 0x0C, the
//! same key values 0x5555/0xAAAA/0xCCCC, the same `PUD`/`RUD` busy bits). So this is ONE model
//! parameterised only by the FWDGT base (data, from [`crate::addr::AddrTable`]), like
//! [`crate::adc`] / [`crate::i2c`]; there is no [`crate::descriptor`] selector here. The only
//! family-touched pieces sit in the RCU layer ([`crate::clock`]): the LSI/IRC40K enable
//! ([`crate::clock::enable_lsi`]) and the reset-cause flag read/clear
//! ([`crate::clock::was_fwdgt_reset`] / [`crate::clock::clear_reset_flags`]). Both have the SAME bit
//! positions on both families (RSTSCK `IRC40KEN`/`IRC40KSTB` bits 0/1, `FWDGTRSTF` bit 29), so even
//! those are family-independent; they live in `clock.rs` because that is the module that owns the
//! RCU base + register model, not because they branch by family.
//!
//! # Register model
//!
//! | reg    | offset | what                                                                       |
//! |--------|--------|----------------------------------------------------------------------------|
//! | `CTL`  | `0x00` | command/key register: 0x5555 unlock, 0xAAAA reload, 0xCCCC start            |
//! | `PSC`  | `0x04` | prescaler divider code (`[2:0]`): 0=/4, 1=/8, .. 6=/256                     |
//! | `RLD`  | `0x08` | 12-bit reload value (`[11:0]`)                                              |
//! | `STAT` | `0x0C` | `PUD` (bit 0) PSC-update busy, `RUD` (bit 1) RLD-update busy                |
//!
//! These are 16-bit register fields, but the SPL accesses them as 32-bit words (`REG32`), so this
//! module uses [`Reg32`] to match.
//!
//! # The five-write start recipe (the SPL `fwdgt_config` + `fwdgt_enable`)
//!
//! [`FreeWatchdog::start`] reproduces the SPL recipe (`gd32f1x0_fwdgt.c` `fwdgt_config`), in order:
//!
//! 1. `CTL = 0x5555` (write-access enable / unlock PSC + RLD).
//! 2. `PSC = prescaler` (the divider code).
//! 3. `RLD = reload` (the 12-bit reload).
//! 4. poll `STAT` until `PUD` AND `RUD` are clear (the PSC/RLD update has propagated to the LSI
//!    clock domain), bounded by [`FWDGT_TIMEOUT`] like [`crate::adc::ADC_TIMEOUT`].
//! 5. `CTL = 0xAAAA` (reload the counter), then `CTL = 0xCCCC` (start the watchdog).
//!
//! [`FreeWatchdog::feed`] is the single `CTL = 0xAAAA` reload, called every loop pass.
//!
//! # No embedded-hal trait
//!
//! `embedded-hal` 1.0 has **no watchdog trait** (the `watchdog::Watchdog` / `WatchdogEnable` traits
//! are `embedded-hal` 0.2 only). For the same dependency reason [`crate::adc`] gives for declining
//! the deprecated `nb` 0.2 `adc::OneShot`, this is a plain runtime-hal handle, not a 0.2 trait impl;
//! fallible calls return `Result` with the crate's per-peripheral [`WatchdogError`] (DECISIONS.md
//! #5).

use crate::clock;
use crate::error::WatchdogError;
use crate::reg::Reg32;

// --- register offsets (identical on both families) --------------------------------------------

/// FWDGT control/key register (`FWDGT_CTL`), offset 0x00.
const CTL: u32 = 0x00;
/// FWDGT prescaler register (`FWDGT_PSC`), offset 0x04.
const PSC: u32 = 0x04;
/// FWDGT reload register (`FWDGT_RLD`), offset 0x08.
const RLD: u32 = 0x08;
/// FWDGT status register (`FWDGT_STAT`), offset 0x0C.
const STAT: u32 = 0x0C;

// --- key / command values (CTL) ---------------------------------------------------------------

/// `FWDGT_WRITEACCESS_ENABLE`: unlock write access to PSC + RLD.
const KEY_UNLOCK: u32 = 0x5555;
/// `FWDGT_KEY_RELOAD`: reload the counter from RLD (the per-pass feed).
const KEY_RELOAD: u32 = 0xAAAA;
/// `FWDGT_KEY_ENABLE`: start the watchdog counter.
const KEY_START: u32 = 0xCCCC;

// --- STAT busy flags --------------------------------------------------------------------------

/// `FWDGT_STAT_PUD` (bit 0): a write to PSC is propagating to the LSI clock domain.
const STAT_PUD: u32 = 1 << 0;
/// `FWDGT_STAT_RUD` (bit 1): a write to RLD is propagating to the LSI clock domain.
const STAT_RUD: u32 = 1 << 1;

// --- hardware limits --------------------------------------------------------------------------

/// The reload register is 12 bits: the maximum reload value (`RLD[11:0]`).
pub const RELOAD_MAX: u16 = 0x0FFF;

/// The smallest prescaler divider (PSC code 0 = /4).
pub const PRESCALER_MIN: u16 = 4;
/// The largest prescaler divider (PSC code 6 = /256).
pub const PRESCALER_MAX: u16 = 256;

/// The IRC40K / LSI nominal frequency in Hz (the GD32 free-watchdog clock source). The real LSI is
/// only loosely trimmed (tens of percent on some parts), so a watchdog period computed against this
/// nominal is approximate; round UP so the actual timeout is never SHORTER than requested (see
/// [`WdgTimeout::from_millis`]).
pub const LSI_HZ: u32 = 40_000;

/// Bounded poll budget for the PSC/RLD update-busy flags in [`FreeWatchdog::start`]. Counts loop
/// iterations, not cycles, so it is clock-independent; generous enough never to false-time a working
/// update at any representative LSI clock, but always escaping a stuck busy bit. Mirrors
/// [`crate::adc::ADC_TIMEOUT`] / [`crate::i2c::I2C_TIMEOUT`].
pub const FWDGT_TIMEOUT: u32 = 100_000;

/// Map a prescaler DIVISOR (4, 8, 16, .. 256) to its PSC register code (0..=6). A divisor that is
/// not a legal power-of-two in `4..=256` is clamped to the nearest legal code; callers pass a value
/// produced by [`WdgTimeout::resolve`], which only ever yields a legal divisor.
#[inline]
const fn psc_code(divisor: u16) -> u32 {
    match divisor {
        4 => 0,
        8 => 1,
        16 => 2,
        32 => 3,
        64 => 4,
        128 => 5,
        256 => 6,
        _ => 6, // clamp an unexpected divisor to the slowest (/256).
    }
}

/// A requested free-watchdog timeout, expressed as a target period and resolved to a legal
/// `(prescaler, reload)` pair against the LSI nominal ([`LSI_HZ`]).
///
/// Build it with [`WdgTimeout::from_millis`]. The resolved pair is chosen so the ACTUAL timeout is
/// **at or above** the requested period (round UP, never shorter than asked): a watchdog that fires
/// EARLY would reset a healthy board, so the rounding bias is toward a slightly longer period. The
/// pair is clamped to the hardware ranges (12-bit reload [`RELOAD_MAX`], prescaler
/// [`PRESCALER_MIN`]..=[`PRESCALER_MAX`]); a request longer than the maximum (~26 s at /256 * 4096
/// on a 40 kHz LSI) clamps to that maximum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WdgTimeout {
    millis: u32,
}

impl WdgTimeout {
    /// A target watchdog period of `ms` milliseconds.
    ///
    /// The period is resolved to a legal `(prescaler, reload)` pair lazily by [`WdgTimeout::resolve`]
    /// (and by [`FreeWatchdog::start`]); this constructor just records the request.
    #[inline]
    pub const fn from_millis(ms: u32) -> WdgTimeout {
        WdgTimeout { millis: ms }
    }

    /// The requested period in milliseconds.
    #[inline]
    pub const fn millis(self) -> u32 {
        self.millis
    }

    /// Resolve this requested period to a legal `(prescaler_divisor, reload)` pair against the LSI
    /// nominal ([`LSI_HZ`]), rounding so the ACTUAL timeout is >= the requested period and clamping
    /// to the hardware ranges.
    ///
    /// The watchdog period is `(prescaler * (reload + 1)) / LSI_HZ` seconds. For a requested period
    /// of `millis` ms the required total LSI-tick count is `ticks = ceil(LSI_HZ * millis / 1000)`
    /// (ceil so the period is never short). For each prescaler divisor from the smallest (/4, the
    /// finest resolution) up to /256, the needed reload is `ceil(ticks / prescaler) - 1`; the first
    /// divisor whose reload fits the 12-bit [`RELOAD_MAX`] is chosen (smallest prescaler that fits =
    /// finest resolution for the period). A request too long for even /256 clamps to
    /// `(256, RELOAD_MAX)` (the longest period the hardware can express); a request of 0 clamps to
    /// the shortest (`(4, 0)`).
    ///
    /// Returns `(prescaler_divisor, reload)` where `prescaler_divisor` is one of 4,8,..,256 and
    /// `reload` is in `0..=RELOAD_MAX`.
    pub const fn resolve(self) -> (u16, u16) {
        // ticks = ceil(LSI_HZ * millis / 1000), in u64 to avoid overflow for large millis.
        let lsi = LSI_HZ as u64;
        let ms = self.millis as u64;
        let numer = lsi * ms;
        // ceil division by 1000.
        let ticks = numer.div_ceil(1000);
        if ticks == 0 {
            // A zero (or sub-tick) request: the shortest expressible period.
            return (PRESCALER_MIN, 0);
        }

        // Try each prescaler from /4 (finest) up to /256; pick the first whose reload fits 12 bits.
        let divisors = [4u16, 8, 16, 32, 64, 128, 256];
        let mut i = 0;
        while i < divisors.len() {
            let div = divisors[i] as u64;
            // reload + 1 = ceil(ticks / div) -> reload = ceil(ticks / div) - 1.
            let reload_plus_one = ticks.div_ceil(div);
            // reload_plus_one >= 1 (ticks >= 1), so the subtraction does not underflow.
            let reload = reload_plus_one - 1;
            if reload <= RELOAD_MAX as u64 {
                return (divisors[i], reload as u16);
            }
            i += 1;
        }
        // Longer than the hardware maximum (/256 * 4096): clamp to the longest period.
        (PRESCALER_MAX, RELOAD_MAX)
    }
}

/// The free (independent) watchdog, resolved once to its FWDGT base (the register model is shared,
/// so there is no per-family field). [`FreeWatchdog::start`] runs the key recipe; the per-pass
/// [`FreeWatchdog::feed`] hangs off the returned handle (DECISIONS.md #4: resolve once into a
/// concrete `Copy` handle, the per-use path holds the raw base).
#[derive(Debug, Clone, Copy)]
pub struct FreeWatchdog {
    base: u32,
}

impl FreeWatchdog {
    /// A bare handle over the FWDGT at `base`, performing **no** register access. Use this to feed a
    /// watchdog that is already running (e.g. across a function boundary). [`FreeWatchdog::start`] is
    /// the configuring entry point.
    #[inline]
    pub const fn at(base: u32) -> FreeWatchdog {
        FreeWatchdog { base }
    }

    /// Bring up and START the free watchdog at `base` with the requested `timeout`, enabling its
    /// LSI/IRC40K clock source first.
    ///
    /// `chip` supplies the RCU base + clock path so the LSI can be enabled through
    /// [`crate::clock::enable_lsi`] before the watchdog is started (the watchdog cannot count until
    /// its clock is running). `base` is the FWDGT base, resolved from the descriptor's
    /// [`crate::addr::AddrTable`] (`chip.base(PeriphLabel::Fwdgt)`); pass it explicitly so the caller
    /// owns the resolve-once step, the same shape the other peripherals use.
    ///
    /// Steps:
    /// 1. Enable + stabilise the LSI/IRC40K ([`crate::clock::enable_lsi`], bounded poll).
    /// 2. The five-write key recipe (see the module docs): unlock, PSC, RLD, wait PSC/RLD update,
    ///    reload, start.
    ///
    /// Returns [`WatchdogError::LsiNotStable`] if the LSI never stabilises, or
    /// [`WatchdogError::Timeout`] if the PSC/RLD update never propagates, both within their bounded
    /// budgets (the F130 hang-if-done-wrong class). On success the watchdog is RUNNING and must be
    /// fed within the timeout from here on.
    ///
    /// SAFETY (bench): once started, the free watchdog cannot be stopped in software (only a reset
    /// clears it). Use a GENEROUS timeout and feed it every loop pass; see
    /// [`FreeWatchdog::freeze_on_debug_halt`] to keep an SWD session from being reset out from under
    /// the core.
    pub fn start(
        chip: &crate::Chip,
        base: u32,
        timeout: WdgTimeout,
    ) -> Result<FreeWatchdog, WatchdogError> {
        let rcu = chip.rcu_base().map_err(|_| WatchdogError::MissingRcuBase)?;
        clock::enable_lsi(rcu)?;

        let dev = FreeWatchdog { base };
        let (divisor, reload) = timeout.resolve();
        dev.program(psc_code(divisor), reload)?;
        Ok(dev)
    }

    /// The five-write key recipe, given the resolved PSC code and 12-bit reload. Separated from
    /// [`FreeWatchdog::start`] so the LSI-enable (a clock-path concern) and the FWDGT register
    /// sequence (this module) are distinct, and so the host tests can drive the recipe directly.
    fn program(&self, psc_code: u32, reload: u16) -> Result<(), WatchdogError> {
        // 1. Unlock write access to PSC + RLD.
        self.ctl().write(KEY_UNLOCK);
        // 2. Prescaler.
        self.reg(PSC).write(psc_code & 0x7);
        // 3. Reload (12-bit).
        self.reg(RLD).write((reload as u32) & RELOAD_MAX as u32);
        // 4. Wait until the PSC and RLD updates have propagated (PUD + RUD clear), bounded.
        self.wait_update()?;
        // 5. Reload the counter, then start the watchdog.
        self.ctl().write(KEY_RELOAD);
        self.ctl().write(KEY_START);
        Ok(())
    }

    /// Feed (reload) the watchdog: a single `CTL = 0xAAAA`. Call this every main-loop pass, well
    /// within the configured timeout, so a healthy loop keeps the counter from reaching zero. This
    /// is the only call on the per-pass hot path; it performs one register write and cannot fail.
    #[inline]
    pub fn feed(&mut self) {
        self.ctl().write(KEY_RELOAD);
    }

    /// Freeze the free watchdog while the debugger holds the core halted (the DBGMCU `FWDGT_HOLD`
    /// debug-freeze bit), so an attached SWD session that halts the core is not reset out from under
    /// the debugger.
    ///
    /// This sets `DBG_CTL.FWDGT_HOLD` (bit 8) at the DBG base `0xE0042000` offset `0x04`. The DBG
    /// base and the bit position are IDENTICAL on both families (`gd32f10x_dbg.h` `DBG_CTL` /
    /// `gd32f1x0_dbg.h` `DBG_CTL0`, both at DBG+0x04, FWDGT_HOLD = bit 8), and the DBG block sits at
    /// a fixed absolute address the descriptor does not carry (like the FMC base in `clock.rs`), so
    /// there is no family branch or descriptor lookup here. It does NOT touch TIMER0 or any arming
    /// gate. When no debugger is attached the bit has no effect, so calling it unconditionally at
    /// bring-up is harmless on a production board.
    pub fn freeze_on_debug_halt() {
        const DBG_BASE: u32 = 0xE004_2000;
        const DBG_CTL_OFFSET: u32 = 0x04;
        const FWDGT_HOLD: u32 = 1 << 8;
        Reg32::new(DBG_BASE, DBG_CTL_OFFSET).modify(FWDGT_HOLD, FWDGT_HOLD);
    }

    /// The underlying FWDGT base address.
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    // --- polling --------------------------------------------------------------------------------

    /// Spin until both the PSC-update (PUD) and RLD-update (RUD) busy flags in STAT are clear (the
    /// SPL waits for each separately; waiting for both together is equivalent and is what the plan's
    /// recipe step 4 describes), bounded by [`FWDGT_TIMEOUT`]. Exhaustion is [`WatchdogError::Timeout`].
    fn wait_update(&self) -> Result<(), WatchdogError> {
        let mut budget = FWDGT_TIMEOUT;
        while self.reg(STAT).read() & (STAT_PUD | STAT_RUD) != 0 {
            budget -= 1;
            if budget == 0 {
                return Err(WatchdogError::Timeout);
            }
        }
        Ok(())
    }

    // --- register accessors ---------------------------------------------------------------------

    #[inline]
    fn reg(&self, off: u32) -> Reg32 {
        Reg32::new(self.base, off)
    }
    #[inline]
    fn ctl(&self) -> Reg32 {
        self.reg(CTL)
    }
}

/// True if the last reset was caused by the free watchdog (the RCU `FWDGTRSTF` reset-cause flag).
///
/// A convenience re-export of [`crate::clock::was_fwdgt_reset`] so a firmware can log / signal a
/// watchdog recovery at boot. `rcu_base` is the chip's RCU base (`chip.rcu_base()`). The reset-cause
/// flag stays set across the reset until [`clear_reset_cause`] clears it, so read it BEFORE clearing.
#[inline]
pub fn was_watchdog_reset(rcu_base: u32) -> bool {
    clock::was_fwdgt_reset(rcu_base)
}

/// Clear the RCU reset-cause flags (`RSTSCK.RSTFC`), so the next boot's [`was_watchdog_reset`]
/// reflects only a fresh reset cause. A convenience re-export of [`crate::clock::clear_reset_flags`].
#[inline]
pub fn clear_reset_cause(rcu_base: u32) {
    clock::clear_reset_flags(rcu_base);
}

#[cfg(test)]
mod tests;
