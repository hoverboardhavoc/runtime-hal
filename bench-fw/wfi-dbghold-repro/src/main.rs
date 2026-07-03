//! A/B partner to `wfi-lock-repro`: tests the PROPER fix for the GD32F130 `wfi` SWD-lockout.
//!
//! Identical to the plain repro (write a RAM marker, then `loop { wfi() }`) EXCEPT it first sets the
//! GD32F1x0 debug control register's sleep-hold bits, which keep the debug clock alive in low-power so
//! the debugger can still re-attach/halt the core through `wfi`.
//!   - plain `wfi-lock-repro` after a power-cycle  -> SWD locked (AP-write dead).
//!   - this one after a power-cycle                -> should re-attach FINE (AP-write works).
//! If so, setting `DBG_CTL0` sleep-hold is the fix that lets production firmware sleep safely.
//!
//! GD32F1x0 User Manual section 9.4.2, DBG_CTL0 @ 0xE004_2004:
//!   bit0 SLP_HOLD (sleep), bit1 DSLP_HOLD (deep-sleep), bit2 STB_HOLD (standby).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

const MARKER_ADDR: u32 = 0x2000_1F00;
const MARKER: u32 = 0x5EEF_D8B6; // distinct from the plain repro's 0x5EEF_5EEF, so we know which ran

#[entry]
fn main() -> ! {
    // Keep debug alive through sleep/deep-sleep/standby so SWD stays attachable across `wfi` -
    // through the HAL primitive this repro validated (runtime_hal::debug_hold_on_sleep, the
    // DBG_CTL0 |= SLP_HOLD|DSLP_HOLD|STB_HOLD RMW; the raw poke this crate used to carry moved
    // into the HAL, debt-paydown slice 9).
    runtime_hal::debug_hold_on_sleep();
    // SAFETY: MARKER_ADDR is the reserved RAM tail (see memory.x); single writer.
    unsafe { core::ptr::write_volatile(MARKER_ADDR as *mut u32, MARKER) };

    loop {
        cortex_m::asm::wfi();
    }
}
