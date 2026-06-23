//! On-silicon validator for `runtime_hal::fmc::Fmc`.
//!
//! Runs the FMC spec's "Target (bench)" cases through the REAL driver on the detected chip and writes
//! a fixed-layout `#[repr(C)]` result struct ([`FmcResult`]) to `RESULT_ADDR` (`0x2000_1F00`), `magic`
//! written LAST, the same result-struct pattern the `coldpath` / `detect` / `probe` firmwares use: the
//! SWD reader reads the struct at that constant address (no nm) and `magic` last means the run
//! completed. One image for all parts: the scratch page and the out-of-flash address derive from
//! `flash_size_bytes()` / `page_size()`, so K1/64 KiB (C8) vs K2/256 KiB (12-FET) adapt at runtime (on
//! the 12-FET the scratch lands high, exercising the 2 KiB page + >64 KiB extent).
//!
//! This NEVER drives the motor bridge / a timer / MOE, it only erases + programs a scratch flash page
//! (the last page of the part), which it leaves erased on exit. Safe regardless of bus power.
//!
//! Error codes (the `*_code` fields): Ok=0, BadArg=1, NotErased=2, WriteProtect=3, ProgramError=4,
//! Timeout=5, other=0xFF. Every field is a `u32`, so a reader can `mdw RESULT_ADDR 16` and decode by
//! the struct's field order below.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use runtime_hal::{detect_chip, Fmc, FmcError};

// --- the SWD-readable result struct ------------------------------------------------------------

/// The fixed-layout FMC validator result. `#[repr(C)]` fixes the field order/offsets so the SWD reader
/// can index by byte offset; `magic` is written LAST (0 = the run never completed). All fields are
/// `u32`, so the struct is exactly 16 words and `mdw RESULT_ADDR 16` reads it whole.
#[repr(C)]
struct FmcResult {
    /// `0xF300_C0DE`, written LAST = the full run completed.
    magic: u32,
    /// `page_size()` (expect 1024 C8 / 2048 12-FET).
    page_size: u32,
    /// `flash_size_bytes() / 1024` (expect 64 C8 / 256 12-FET).
    flash_kib: u32,
    /// The scratch page address used (last page of the part).
    scratch: u32,
    /// `erase_page(scratch)` result code, expect 0 (Ok).
    erase_code: u32,
    /// Word at `scratch` after erase, expect 0xFFFFFFFF.
    erase_readback: u32,
    /// `program(scratch, DEADBEEF)` result code, expect 0 (Ok).
    prog_code: u32,
    /// Word at `scratch` after program, expect `PATTERN_LE` (0xEFBEADDE).
    prog_readback: u32,
    /// `program(scratch, ..)` AGAIN result code, expect 2 (NotErased, pre-check).
    reprog_code: u32,
    /// Word at `scratch` after the rejected reprogram, expect 0xEFBEADDE (unchanged, no partial write).
    reprog_readback: u32,
    /// erase-then-program again result code, expect 0 (Ok, proves the rejection left no stuck state).
    recover_code: u32,
    /// `erase_page(scratch + 2)` (misaligned), expect 1 (BadArg).
    badarg_misalign: u32,
    /// `erase_page(flash_end)` (out of flash), expect 1 (BadArg).
    badarg_oof_erase: u32,
    /// `program(flash_end, ..)` (out of flash), expect 1 (BadArg).
    badarg_oof_prog: u32,
    /// `program(scratch + 1, ..)` (odd address), expect 1 (BadArg).
    badarg_oddaddr: u32,
    /// `program(scratch, 3 bytes)` (odd length), expect 1 (BadArg).
    badarg_oddlen: u32,
}

/// The magic value written last once every case has run.
const MAGIC: u32 = 0xF300_C0DE;

/// Fixed RAM address of the result struct: the top of the (shrunk) RAM region, reserved by `memory.x`
/// (RAM length minus a 256-byte tail). The SWD reader reads this constant directly, no nm.
const RESULT_ADDR: u32 = 0x2000_1F00;

/// Initial result contents, written to [`RESULT_ADDR`] at startup so a stale RAM image cannot be
/// mistaken for a completed run (`magic` 0 until the end).
const INIT_RESULT: FmcResult = FmcResult {
    magic: 0,
    page_size: 0,
    flash_kib: 0,
    scratch: 0,
    erase_code: 0,
    erase_readback: 0,
    prog_code: 0,
    prog_readback: 0,
    reprog_code: 0,
    reprog_readback: 0,
    recover_code: 0,
    badarg_misalign: 0,
    badarg_oof_erase: 0,
    badarg_oof_prog: 0,
    badarg_oddaddr: 0,
    badarg_oddlen: 0,
};

const FLASH_BASE: u32 = 0x0800_0000;
/// Test pattern programmed into the scratch page; read back as a little-endian `u32` it is `PATTERN_LE`.
const PATTERN: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const PATTERN_LE: u32 = 0xEFBE_ADDE;
const _: () = assert!(PATTERN_LE == 0xEFBE_ADDE); // doc anchor: read-back expectation

/// Encode a driver result as a small code for the SWD reader.
fn code(r: Result<(), FmcError>) -> u32 {
    match r {
        Ok(()) => 0,
        Err(FmcError::BadArg) => 1,
        Err(FmcError::NotErased) => 2,
        Err(FmcError::WriteProtect) => 3,
        Err(FmcError::ProgramError) => 4,
        Err(FmcError::Timeout) => 5,
        Err(_) => 0xFF, // FmcError is #[non_exhaustive]
    }
}

/// Memory-mapped flash read (the validator reading flash content, between FMC ops; not the driver).
#[inline(never)]
fn rd32(addr: u32) -> u32 {
    // SAFETY: addr is in mapped flash; a plain load, no FMC op in flight.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

// --- result-struct writers (volatile, through the raw pointer to the pinned struct) ------------
//
// Mirrors the coldpath `store!` pattern: volatile stores so the optimiser cannot drop/reorder the
// writes the SWD reader depends on, and so the magic genuinely lands last.

#[inline]
fn result_ptr() -> *mut FmcResult {
    RESULT_ADDR as *mut FmcResult
}

macro_rules! store {
    ($field:ident, $val:expr) => {{
        // SAFETY: single-threaded firmware, no interrupts touch the result struct; the only writer is
        // this path, reads are external (SWD). Volatile so the stores are not elided/reordered.
        unsafe {
            let p = result_ptr();
            core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
        }
    }};
}

#[entry]
fn main() -> ! {
    // Clear the whole block (magic 0) so a stale RAM image is never read as a completed run.
    // SAFETY: RESULT_ADDR is the reserved RAM tail (see memory.x); single writer.
    unsafe { core::ptr::write_volatile(result_ptr(), INIT_RESULT) };

    let chip = match detect_chip() {
        Ok(c) => c,
        Err(_) => halt(),
    };
    let mut fmc = Fmc::new(&chip);
    let page = fmc.page_size();
    let extent = fmc.flash_size_bytes();
    let scratch = FLASH_BASE + extent - page; // last page of this part
    let flash_end = FLASH_BASE + extent; // one past the end (out of flash)

    store!(page_size, page);
    store!(flash_kib, extent / 1024);
    store!(scratch, scratch);

    // 2. erase scratch -> Ok, read back all 0xFF.
    store!(erase_code, code(fmc.erase_page(scratch)));
    store!(erase_readback, rd32(scratch));

    // 3. program the pattern into the erased page -> Ok, read back matches.
    store!(prog_code, code(fmc.program(scratch, &PATTERN)));
    store!(prog_readback, rd32(scratch));

    // 4. program again at the same address -> NotErased (pre-check), content unchanged (no partial).
    store!(reprog_code, code(fmc.program(scratch, &PATTERN)));
    store!(reprog_readback, rd32(scratch));

    // 5. recovery: erase then program again -> Ok (proves the NotErased rejection left no stuck state).
    let _ = fmc.erase_page(scratch);
    store!(recover_code, code(fmc.program(scratch, &PATTERN)));

    // 6-10. argument rejections, each BadArg, before the FMC is touched (independent of flash state).
    store!(badarg_misalign, code(fmc.erase_page(scratch + 2))); // misaligned erase addr
    store!(badarg_oof_erase, code(fmc.erase_page(flash_end))); // erase out of flash
    store!(badarg_oof_prog, code(fmc.program(flash_end, &PATTERN))); // program out of flash
    store!(badarg_oddaddr, code(fmc.program(scratch + 1, &PATTERN))); // odd program addr
    store!(badarg_oddlen, code(fmc.program(scratch, &[0u8; 3]))); // odd program length

    // Leave the scratch page erased.
    let _ = fmc.erase_page(scratch);

    store!(magic, MAGIC); // commit: the run completed
    halt()
}

fn halt() -> ! {
    // Busy-spin, NOT wfi: a GD32F130 that idles in wfi with no DBGMCU debug-low-power bits locks SWD
    // re-attach after a power-cycle (recoverable only via connect-under-reset + mass-erase). On the
    // bench ST-Link clones, which cannot drive NRST, that lockout is unrecoverable. nop keeps the
    // debug port live.
    loop {
        cortex_m::asm::nop();
    }
}
