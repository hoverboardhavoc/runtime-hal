//! GD32 debug-control (DBG / DBGMCU) helpers: the SWD-lockout guard primitive.
//!
//! The GD32F130 (and friends) power the debug interface down in the low-power states unless the
//! DBG control register's hold bits are set: a bare `wfi` with `DBG_CTL0 = 0` **locks SWD
//! re-attach** (looks like a permanently AP-write-dead part; recoverable only via
//! connect-under-reset + mass-erase, which the bench's ST-Link clones cannot drive). Proven in
//! isolation on the bench (`bench-fw/wfi-lock-repro` vs `bench-fw/wfi-dbghold-repro`, the A/B
//! pair). Production firmware that must sleep calls [`debug_hold_on_sleep`] EARLY in boot, before
//! any `wfi`; the register resets to 0 on every cold boot, so the firmware must set it itself (a
//! debugger setting it in-session is lost on the next power cycle).

use crate::reg::Reg32;

/// The DBG register block base, `0xE004_2000` (GD32F1x0 User Manual 9.4: "DBG"; same block on
/// F10x: GD SPL `gd32f10x_dbg.h:41` `#define DBG DBG_BASE`, the `0xE004_2000` core-debug region).
const DBG_BASE: u32 = 0xE004_2000;
/// `DBG_CTL0` offset `0x04` (GD32F1x0 UM 9.4.2 "Control register 0 (DBG_CTL0)", address
/// `0xE004_2004`; F10x: `gd32f10x_dbg.h:49` `DBG_CTL = DBG + 0x04`).
const DBG_CTL0: u32 = 0x04;
/// The three low-power debug-hold bits (GD32F1x0 UM 9.4.2; F10x `gd32f10x_dbg.h:56-58`):
/// bit 0 `SLP_HOLD` (keep the debugger connection during sleep), bit 1 `DSLP_HOLD` (deep-sleep),
/// bit 2 `STB_HOLD` (standby).
const DBG_HOLD_LOWPOWER: u32 = 0b111;

/// `TIMER0_HOLD` (bit 10 of `DBG_CTL0`, the same register the low-power holds live in): when set, the
/// advanced-timer TIMER0 counter FREEZES while the core is halted. Verified at **bit 10** on both
/// families (GD32F1x0 User Manual Rev3.6 and GD32F10x User Manual Rev2.6, the `DBG_CTL0` bit table).
const DBG_HOLD_TIMER0: u32 = 1 << 10;

/// Keep the debug interface alive through sleep / deep-sleep / standby, so SWD stays attachable
/// across `wfi` (the GD32 SWD-lockout guard): RMW `DBG_CTL0 |= SLP_HOLD | DSLP_HOLD | STB_HOLD`.
///
/// Call EARLY in boot, before any `wfi`/sleep is possible; `DBG_CTL0` resets to 0 on every cold
/// boot. Idempotent; touches nothing but the three hold bits (a set-only RMW). Family-shared: the
/// register block and bit layout are identical on F10x and F1x0 (citations on the constants), and
/// the address is in the Cortex-M private peripheral region, present on every supported part, so
/// no chip resolution is needed.
pub fn debug_hold_on_sleep() {
    Reg32::new(DBG_BASE, DBG_CTL0).modify(DBG_HOLD_LOWPOWER, DBG_HOLD_LOWPOWER);
}

/// Freeze the TIMER0 counter while the core is halted (set `DBG_CTL0` `TIMER0_HOLD`, bit 10): a
/// set-only RMW that touches nothing else. This is the motor-integration **R6** debug-hold surface
/// (`specs/motor-integration.md`).
///
/// **Call it EARLY in boot, before the motor bring-up starts the counter.** `DBG_CTL0` is
/// power-reset-only, so the firmware must set it on every cold boot; a debugger setting it in-session
/// is lost at the next power cycle. It is the BACKSTOP half of the adopted halt posture: it covers the
/// halt nobody initiated (a probe attach that halts, a hardware breakpoint, a debugger reset), where
/// the alternative is the recorded failure of a free-running counter re-applying stale duties
/// indefinitely. It is not a silencing act on its own: a frozen counter holds the compare outputs at
/// their instantaneous levels, so a halt caught with a high-side on leaves that phase at 100% duty. The
/// deliberate half is disarm-then-halt tooling (MOE cleared over the DAP before the halt), and the
/// bench rule "never halt while energized" stands under both.
///
/// Family-shared: `DBG_CTL0` bit 10 is `TIMER0_HOLD` on both F10x and F1x0 (see the constant's
/// citations), and the register is in the always-present Cortex-M debug region, so no chip resolution
/// is needed.
pub fn debug_hold_timer0() {
    Reg32::new(DBG_BASE, DBG_CTL0).modify(DBG_HOLD_TIMER0, DBG_HOLD_TIMER0);
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use crate::reg::{mock, Reg32};

    #[test]
    fn debug_hold_sets_only_the_three_hold_bits() {
        let _g = mock::lock();
        mock::reset();
        // Pre-existing unrelated bits (e.g. TRACE_IOEN, bit 5) must survive the RMW.
        Reg32::new(0xE004_2000, 0x04).write(1 << 5);
        super::debug_hold_on_sleep();
        let v = Reg32::new(0xE004_2000, 0x04).read();
        assert_eq!(v & 0b111, 0b111, "SLP_HOLD|DSLP_HOLD|STB_HOLD set");
        assert_eq!(v & (1 << 5), 1 << 5, "unrelated bits preserved");
        // Idempotent.
        super::debug_hold_on_sleep();
        assert_eq!(Reg32::new(0xE004_2000, 0x04).read(), (1 << 5) | 0b111);
    }

    #[test]
    fn timer0_hold_sets_only_bit_10() {
        let _g = mock::lock();
        mock::reset();
        // Pre-existing unrelated bits (the low-power holds) must survive the set-only RMW.
        Reg32::new(0xE004_2000, 0x04).write(0b111);
        super::debug_hold_timer0();
        let v = Reg32::new(0xE004_2000, 0x04).read();
        assert_eq!(v & (1 << 10), 1 << 10, "TIMER0_HOLD (bit 10) set");
        assert_eq!(v & 0b111, 0b111, "unrelated low-power holds preserved");
        // Idempotent.
        super::debug_hold_timer0();
        assert_eq!(Reg32::new(0xE004_2000, 0x04).read(), (1 << 10) | 0b111);
    }
}
