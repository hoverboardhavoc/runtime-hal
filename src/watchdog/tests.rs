//! Host tests for the free-watchdog driver (run under the `mock` feature against the
//! backing-array register space).
//!
//! Four groups:
//! - **timeout -> (prescaler, reload) mapping** ([`WdgTimeout::resolve`]): several requested
//!   periods, the round-UP-not-down bias (actual >= requested), and clamping at both extremes.
//! - **start recipe** ([`FreeWatchdog::program`], the FWDGT half of [`FreeWatchdog::start`]): the
//!   key/PSC/RLD writes land with the right values and the final CTL is the start key, vs the SPL
//!   `fwdgt_config` + `fwdgt_enable` recipe; the bounded PSC/RLD-update poll escapes a stuck busy
//!   bit with [`WatchdogError::Timeout`].
//! - **feed** ([`FreeWatchdog::feed`]): writes ONLY the reload key (CTL = 0xAAAA), leaving PSC/RLD
//!   untouched.
//! - **reset cause** ([`was_watchdog_reset`] / [`clear_reset_cause`], over `clock::was_fwdgt_reset`
//!   / `clock::clear_reset_flags`): reads the `FWDGTRSTF` bit, and the clear writes `RSTFC`.
//!
//! The mock backend is a flat array (a static snapshot, not a sequencer): each CTL write overwrites
//! the last, so the inter-write ORDER of the three CTL keys is not separately observable here (it is
//! pinned by the SPL recipe + the bench). What the flat mock DOES prove is the end state (PSC/RLD
//! hold the values written BETWEEN the unlock and the reload/start keys, so the unlock necessarily
//! preceded them and the start key is last), the resolved values, the bounded-poll escape, and that
//! feed touches only CTL.
#![cfg(feature = "mock")]

use super::*;
use crate::reg::{mock, Reg32};
use std::sync::MutexGuard;

/// The FWDGT base both families carry (`0x4000_3000`); the mock window wraps modulo its size, only
/// the offsets matter.
const FWDGT_BASE: u32 = 0x4000_3000;
/// The RCU base both families carry; `RSTSCK` is at offset 0x24.
const RCU_BASE: u32 = 0x4002_1000;
const RSTSCK: u32 = 0x24;

fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

fn r(off: u32) -> u32 {
    Reg32::new(FWDGT_BASE, off).read()
}

fn dev() -> FreeWatchdog {
    FreeWatchdog { base: FWDGT_BASE }
}

// --- timeout -> (prescaler, reload) mapping ---------------------------------------------------

/// The actual watchdog period (in milliseconds) a resolved (divisor, reload) pair yields against the
/// LSI nominal: `(divisor * (reload + 1)) / LSI_HZ` seconds, in ms.
fn actual_ms(divisor: u16, reload: u16) -> u64 {
    let ticks = divisor as u64 * (reload as u64 + 1);
    ticks * 1000 / LSI_HZ as u64
}

#[test]
fn maps_short_period_to_finest_prescaler() {
    // 100 ms at 40 kHz = 4000 ticks. /4 gives reload+1 = 1000 -> reload 999, which fits 12 bits, so
    // the finest prescaler (/4) is chosen.
    let (div, reload) = WdgTimeout::from_millis(100).resolve();
    assert_eq!(div, 4, "shortest prescaler that fits");
    assert_eq!(reload, 999);
    assert!(actual_ms(div, reload) >= 100, "actual >= requested");
}

#[test]
fn maps_stock_6500ms_period() {
    // The stock firmware's ~6.5 s watchdog. 6500 ms at 40 kHz = 260_000 ticks.
    // /4..32 overflow 12 bits; /64 -> reload+1 = ceil(260000/64) = 4063 -> reload 4062, fits 12 bits.
    let (div, reload) = WdgTimeout::from_millis(6500).resolve();
    assert!(reload <= RELOAD_MAX);
    assert!(
        actual_ms(div, reload) >= 6500,
        "actual {} >= 6500",
        actual_ms(div, reload)
    );
    // The chosen divisor is the smallest one whose reload fits.
    assert_eq!(div, 64);
    assert_eq!(reload, 4062);
}

#[test]
fn rounds_up_never_down() {
    // A period that is not an exact multiple of the tick: 101 ms at 40 kHz = 4040 ticks. /4 ->
    // reload+1 = ceil(4040/4) = 1010 -> reload 1009. actual = 4*1010/40000 s = 101 ms exactly here,
    // but the key property across the board is actual >= requested. Sweep a range and assert it.
    for ms in [1u32, 7, 13, 100, 333, 1000, 3001, 6500, 12345] {
        let (div, reload) = WdgTimeout::from_millis(ms).resolve();
        assert!(div >= PRESCALER_MIN && div <= PRESCALER_MAX);
        assert!(reload <= RELOAD_MAX);
        assert!(
            actual_ms(div, reload) >= ms as u64,
            "ms={ms}: actual {} must be >= requested",
            actual_ms(div, reload)
        );
    }
}

#[test]
fn clamps_zero_to_shortest() {
    // A zero request: the shortest expressible period, /4 reload 0.
    assert_eq!(WdgTimeout::from_millis(0).resolve(), (PRESCALER_MIN, 0));
}

#[test]
fn clamps_overlong_to_maximum() {
    // The hardware maximum is /256 * 4096 ticks / 40 kHz = ~26.2 s. A request well beyond that
    // clamps to (256, RELOAD_MAX), the longest period the registers can express.
    let max_ms = actual_ms(PRESCALER_MAX, RELOAD_MAX);
    assert_eq!(
        WdgTimeout::from_millis(60_000).resolve(),
        (PRESCALER_MAX, RELOAD_MAX),
        "60 s (> ~{max_ms} ms max) clamps to the longest period"
    );
    // The exact maximum still maps to the max pair (or finer), never overflows.
    let (div, reload) = WdgTimeout::from_millis(max_ms as u32).resolve();
    assert!(reload <= RELOAD_MAX && div <= PRESCALER_MAX);
}

#[test]
fn psc_code_maps_divisors() {
    assert_eq!(psc_code(4), 0);
    assert_eq!(psc_code(8), 1);
    assert_eq!(psc_code(16), 2);
    assert_eq!(psc_code(32), 3);
    assert_eq!(psc_code(64), 4);
    assert_eq!(psc_code(128), 5);
    assert_eq!(psc_code(256), 6);
}

// --- start recipe -----------------------------------------------------------------------------

#[test]
fn program_writes_psc_rld_and_starts() {
    let _g = seed_reset();
    // /128, reload 2031 (the stock-ish pair).
    dev().program(psc_code(128), 2031).unwrap();

    // PSC holds the prescaler code (5 for /128); only the low 3 bits are the field.
    assert_eq!(r(PSC) & 0x7, 5, "PSC = /128 code");
    // RLD holds the 12-bit reload.
    assert_eq!(r(RLD) & RELOAD_MAX as u32, 2031, "RLD = reload");
    // The LAST CTL write is the start key (0xCCCC): the unlock (0x5555) preceded PSC/RLD (which hold
    // their values, proving write access was unlocked), then reload (0xAAAA), then start.
    assert_eq!(r(CTL), KEY_START, "final CTL = start key");
}

#[test]
fn program_poll_times_out_on_stuck_busy_bit() {
    let _g = seed_reset();
    // Seed STAT with the PUD busy bit stuck set: the bounded update-poll must give up with Timeout
    // rather than spin forever (the hang-if-done-wrong class).
    Reg32::new(FWDGT_BASE, STAT).write(STAT_PUD);
    assert_eq!(
        dev().program(psc_code(4), 100),
        Err(WatchdogError::Timeout),
        "stuck PUD -> bounded Timeout"
    );
}

// --- feed -------------------------------------------------------------------------------------

#[test]
fn feed_writes_only_the_reload_key() {
    let _g = seed_reset();
    // Seed PSC/RLD as if the watchdog were already configured.
    Reg32::new(FWDGT_BASE, PSC).write(5);
    Reg32::new(FWDGT_BASE, RLD).write(2031);

    let mut wdg = dev();
    wdg.feed();

    // feed() writes ONLY the reload key to CTL, leaving PSC/RLD untouched.
    assert_eq!(r(CTL), KEY_RELOAD, "feed writes the reload key");
    assert_eq!(r(PSC) & 0x7, 5, "feed leaves PSC untouched");
    assert_eq!(
        r(RLD) & RELOAD_MAX as u32,
        2031,
        "feed leaves RLD untouched"
    );
}

// --- reset cause ------------------------------------------------------------------------------

#[test]
fn was_watchdog_reset_reads_fwdgtrstf() {
    let _g = seed_reset();
    // FWDGTRSTF is bit 29 of RSTSCK.
    assert!(!was_watchdog_reset(RCU_BASE), "clear flag -> false");
    Reg32::new(RCU_BASE, RSTSCK).write(1 << 29);
    assert!(was_watchdog_reset(RCU_BASE), "set FWDGTRSTF -> true");
}

#[test]
fn clear_reset_cause_writes_rstfc() {
    let _g = seed_reset();
    // Seed the FWDGT reset flag, then clear: clear_reset_cause sets RSTFC (bit 24).
    Reg32::new(RCU_BASE, RSTSCK).write(1 << 29);
    clear_reset_cause(RCU_BASE);
    assert_ne!(
        Reg32::new(RCU_BASE, RSTSCK).read() & (1 << 24),
        0,
        "RSTFC (reset-flag-clear) bit written"
    );
}
