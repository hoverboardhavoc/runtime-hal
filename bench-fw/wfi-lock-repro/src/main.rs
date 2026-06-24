//! Minimal ISOLATION repro for the GD32F130 `wfi` SWD-lockout.
//!
//! Does NOTHING but write one RAM marker word and then idle in `wfi` forever. No runtime-hal, no
//! clock-tree setup, no GPIO, no peripherals, no bus-fault probe, no detect/probe logic. The whole
//! point is to isolate `wfi`: flash this, power-cycle, then re-probe over SWD.
//!   - If AP-write is then DEAD (examination fails, the CSW write aborts) -> `wfi` ALONE is sufficient
//!     to lock SWD re-attach on this part, independent of any other firmware.
//!   - If it re-attaches fine -> `wfi` alone is NOT enough; the lockout needs something the
//!     detect/probe firmware does in combination with it.
//!
//! Recover either way with a probe that drives NRST (the ESP32 elaphureLink): connect-under-reset ->
//! `reset halt` -> `stm32f1x mass_erase`.
//!
//! Marker: `0x5EEF_5EEF` at `0x2000_1F00` (start of the reserved RAM tail, see memory.x), written
//! BEFORE entering `wfi` so a reader can confirm `main` ran (visible on the immediate post-flash read,
//! before any power-cycle lock).

#![no_std]
#![no_main]

// SAFETY GUARD: this firmware DELIBERATELY locks SWD re-attach on a GD32F130 after a power-cycle.
// Recovery requires a connect-under-reset + `stm32f1x mass_erase` with a probe that actually drives
// NRST (an ESP32 elaphureLink or a genuine ST-Link, NOT an ST-Link clone). To stop it being built or
// flashed by accident, it does not compile without an explicit opt-in feature:
//     cargo build --release --features yes-lock-this-board
#[cfg(not(feature = "yes-lock-this-board"))]
compile_error!(
    "wfi-lock-repro DELIBERATELY locks SWD re-attach on GD32F130 (recoverable only via \
     connect-under-reset + mass_erase with a probe that drives NRST). Build with \
     `--features yes-lock-this-board` only if you mean to."
);

use cortex_m_rt::entry;
use panic_halt as _;

const MARKER_ADDR: u32 = 0x2000_1F00;
const MARKER: u32 = 0x5EEF_5EEF;

#[entry]
fn main() -> ! {
    // SAFETY: MARKER_ADDR is the reserved RAM tail (see memory.x); single writer, no concurrency.
    unsafe { core::ptr::write_volatile(MARKER_ADDR as *mut u32, MARKER) };

    // The entire experiment: sleep. Nothing else has run.
    loop {
        cortex_m::asm::wfi();
    }
}
