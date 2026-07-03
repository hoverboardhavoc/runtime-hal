//! The F10x AFIO owner (`specs/afio-ownership.md`): the ONE module that declares the AFIO base and
//! writes `AFIO_PCF0`, the shared whole-MCU remap register.
//!
//! On F10x the alternate-function routing is **global**: one AFIO peripheral (base `0x4001_0000` on
//! APB2, GD32F10x User Manual ch.7) whose `AFIO_PCF0` register (offset `0x04`, UM 7.5.9) holds every
//! remap field in one 32-bit word - including `SWJ_CFG[26:24]` (UM 7.4.3), the JTAG/SWD
//! configuration. A writer that blind-writes `PCF0` (or masks `SWJ_CFG` to 0) **disables SW-DP and
//! bricks debug re-attach**, so the SWD-preserve discipline must live in exactly one place. Before
//! this module the base and the AFIOEN clock-enable RMW were declared twice (`chip.rs` and
//! `gpio.rs`), each writer re-implementing the discipline independently (the 2026-07-02 audit's
//! finding #3); this consolidates them, byte-for-byte identical in effect.
//!
//! The contract every helper here obeys (`specs/afio-ownership.md`, "The contract"):
//! 1. RMW a single named field, never a blind `PCF0` write.
//! 2. Preserve `SWJ_CFG[26:24]` (only [`set_swj_sw_only`] touches it, and only to SW-only).
//! 3. Enable the AFIO clock (`RCU_APB2EN.AFIOEN`, idempotent RMW) before the first `PCF0` write.
//!
//! Per-field helpers exist ONLY for fields with a real writer today (`SWJ_CFG`, `TIMER1_REMAP`);
//! `USART0_REMAP` is added when `specs/usart-pin-remap.md` lands (out of scope: its default mapping
//! is the PA9/PA10 gate pins, so its bring-up is gate-pin-adjacent). **F10x-only**: the F1x0 has no
//! AFIO at all (per-pin `GPIOx_AFSEL` routing; the AFIO address region is reserved and must not be
//! written), so nothing here is reachable from an F1x0 code path.

use crate::reg::Reg32;

/// The F10x AFIO base. Fixed by the part family at this absolute address on every F10x; the
/// descriptor does not carry it. Declared HERE and nowhere else.
const AFIO_BASE: u32 = 0x4001_0000;
/// `AFIO_PCF0` (AF port-config register 0) offset, 0x04 (GD32F10x User Manual 7.5.9). Word-access
/// only.
const AFIO_PCF0: u32 = 0x04;

/// `RCU_APB2EN` offset and its `AFIOEN` bit (bit 0): the AFIO peripheral clock enable.
const RCU_APB2EN_OFFSET: u32 = 0x18;
const AFIOEN: u32 = 1 << 0;

/// `SWJ_CFG[26:24]` (UM 7.4.3): the JTAG/SWD configuration field. `0b010` = JTAG-DP disabled,
/// SW-DP enabled (frees PA15/PB3/PB4 while keeping SWD live). The brick hazard: any other value
/// reached accidentally can disable SW-DP.
const PCF0_SWJ_CFG: u32 = 0b111 << 24;
const PCF0_SWJ_CFG_SW_ONLY: u32 = 0b010 << 24;

/// `TIMER1_REMAP[1:0]` field in `AFIO_PCF0`, bits `[9:8]` (UM 7.5.9).
const PCF0_TIMER1_REMAP: u32 = 0b11 << 8;
/// `TIMER1_REMAP` partial-remap value `01`: `TIMER1_CH0-ETI / PA15, TIMER1_CH1 / PB3, TIMER1_CH2 /
/// PA2, TIMER1_CH3 / PA3` (UM 7.5.9). The value that puts TIMER1_CH1 onto PB3 (the green LED), the
/// G3 target.
///
/// Naming: this field value `01` is the GD32 SPL's `GPIO_TIMER1_PARTIAL_REMAP0` (`0x00180100`,
/// field `01`), NOT `GPIO_TIMER1_PARTIAL_REMAP1`. The SPL's `GPIO_TIMER1_PARTIAL_REMAP1`
/// (`0x00180200`) is a DIFFERENT value, field `10`, which routes `TIMER1_CH2/CH3` to `PB10/PB11`
/// and does NOT put any channel on PB3. We name this constant `..._PARTIAL0` to match the SPL and
/// avoid that trap; the written register value remains `01`.
const PCF0_TIMER1_REMAP_PARTIAL0: u32 = 0b01 << 8;

/// Enable the AFIO peripheral clock (`RCU_APB2EN.AFIOEN`). Idempotent RMW; every field helper calls
/// it so each is self-contained regardless of caller order.
#[inline]
fn enable_clock(rcu_base: u32) {
    Reg32::new(rcu_base, RCU_APB2EN_OFFSET).modify(AFIOEN, AFIOEN);
}

/// **F10x-only**: set `SWJ_CFG` to SW-only (`0b010`: JTAG-DP disabled, SW-DP enabled), freeing
/// PA15/PB3/PB4 while keeping SWD live. The ONE writer of the `SWJ_CFG` field (the
/// [`crate::Chip::free_jtag_pins`] F10x arm); an RMW of only that field, so every remap field is
/// left untouched.
#[inline]
pub(crate) fn set_swj_sw_only(rcu_base: u32) {
    enable_clock(rcu_base);
    Reg32::new(AFIO_BASE, AFIO_PCF0).modify(PCF0_SWJ_CFG, PCF0_SWJ_CFG_SW_ONLY);
}

/// **F10x-only**: remap TIMER1 to partial remap (value `01` = the SPL's
/// `GPIO_TIMER1_PARTIAL_REMAP0`), putting `TIMER1_CH1` onto **PB3** (and `TIMER1_CH0` onto PA15),
/// by RMW of the `TIMER1_REMAP[1:0]` field of `AFIO_PCF0`.
///
/// This is the F10x half of the G3 general-timer routing (the F1x0 routes purely through the
/// per-pin `AFSEL` mux in `gpio::configure_af`, so this primitive does NOT exist for the F1x0 and
/// is never called there). The RMW touches only `TIMER1_REMAP`, so the `SWJ_CFG` bits
/// [`set_swj_sw_only`] set (and every other remap field) are left untouched - the SWD-brick guard
/// the host tests pin. It does NOT touch any timer register, TIMER0, or any arming gate.
///
/// (TIMER1 remap is not available on a 36-pin package; the bench parts are 48-pin C8, so PB3 is
/// reachable. TIMER2 remap by contrast needs a 64/100/144-pin package, which is why the G3 target
/// is TIMER1, not TIMER2.)
#[inline]
pub(crate) fn remap_timer1_partial0(rcu_base: u32) {
    enable_clock(rcu_base);
    Reg32::new(AFIO_BASE, AFIO_PCF0).modify(PCF0_TIMER1_REMAP, PCF0_TIMER1_REMAP_PARTIAL0);
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use crate::reg::{mock, Reg32};

    const RCU_BASE: u32 = 0x4002_1000;
    const AFIO_PCF0_ABS: u32 = 0x4001_0004;

    /// Serialize + zero the mock register window per case.
    fn seed() -> std::sync::MutexGuard<'static, ()> {
        let g = mock::lock();
        mock::reset();
        g
    }

    #[test]
    fn remap_timer1_partial0_sets_pcf0_field_to_01_and_enables_afio() {
        let _g = seed();
        super::remap_timer1_partial0(RCU_BASE);
        // AFIO clock enabled: RCU_APB2EN (0x18) bit 0.
        assert_eq!(Reg32::new(RCU_BASE, 0x18).read() & 1, 1);
        // TIMER1_REMAP[9:8] = 0b01 (SPL GPIO_TIMER1_PARTIAL_REMAP0: TIMER1_CH1 -> PB3); no other
        // PCF0 bits set.
        assert_eq!(Reg32::new(AFIO_PCF0_ABS, 0).read(), 0b01 << 8);
    }

    #[test]
    fn remap_timer1_partial0_preserves_swj_cfg() {
        let _g = seed();
        // free_jtag_pins sets SWJ_CFG (AFIO_PCF0[26:24]) = 0b010; the TIMER1 remap RMW must not
        // disturb it (the SWD-brick guard, specs/afio-ownership.md contract #2).
        Reg32::new(AFIO_PCF0_ABS, 0).write(0b010 << 24); // pretend set_swj_sw_only ran
        super::remap_timer1_partial0(RCU_BASE);
        let pcf0 = Reg32::new(AFIO_PCF0_ABS, 0).read();
        assert_eq!(pcf0 & (0b111 << 24), 0b010 << 24, "SWJ_CFG preserved");
        assert_eq!(pcf0 & (0b11 << 8), 0b01 << 8, "TIMER1_REMAP = 01");
    }

    #[test]
    fn set_swj_sw_only_touches_only_the_swj_field() {
        let _g = seed();
        // A pre-existing remap field (TIMER1_REMAP = 01) must survive the SWJ write: the owner's
        // per-field RMW discipline cuts both ways.
        Reg32::new(AFIO_PCF0_ABS, 0).write(0b01 << 8);
        super::set_swj_sw_only(RCU_BASE);
        let pcf0 = Reg32::new(AFIO_PCF0_ABS, 0).read();
        assert_eq!(pcf0 & (0b111 << 24), 0b010 << 24, "SWJ_CFG = SW-only");
        assert_eq!(pcf0 & (0b11 << 8), 0b01 << 8, "TIMER1_REMAP untouched");
        assert_eq!(Reg32::new(RCU_BASE, 0x18).read() & 1, 1, "AFIOEN on");
    }
}
