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
}
