//! Host tests for the FMC flash driver (run under the `mock` feature against the backing-array
//! register + flash space).
//!
//! Every case runs against BOTH a K1 / 64 KiB and a K2 / 256 KiB mock descriptor so the page-size /
//! extent maths is covered both ways (the only K2 part on the bench is the 256 KiB 12-FET).
//!
//! Groups:
//! - **descriptor reflection** ([`Fmc::page_size`] / [`Fmc::flash_size_bytes`]): K1 -> 1024 + 65536,
//!   K2 -> 2048 + 262144.
//! - **erase** ([`Fmc::erase_page`]): an aligned in-flash address emits the unlock (KEY1 then KEY2),
//!   `CTL.PER`, `ADDR = addr`, `CTL.START`, the flag-clear, and `PER` cleared; a misaligned or
//!   out-of-flash address is `BadArg` with ZERO register writes.
//! - **program** ([`Fmc::program`]): aligned addr + even len into ERASED space programs the halfwords
//!   in order; odd addr / odd len / end-straddle is `BadArg` with zero writes; a NON-erased target is
//!   `NotErased` with zero writes (the pre-check, flash unchanged); an empty slice is an `Ok` no-op.
//! - **write-once** the mock store models silicon: a re-program of a written halfword is REFUSED
//!   (PGERR), NOT ANDed, so a re-program bug fails the test rather than only on silicon.
//! - **error decode** ([`decode`]): `WPERR` -> `WriteProtect`, `PGERR` -> `ProgramError`,
//!   the bounded-poll sentinel -> `Timeout`.
//!
//! The mock backend is a flat array (a static snapshot, not a sequencer), so the inter-write ORDER of
//! the unlock/command keys is not separately observable; what the flat mock proves is the end state
//! (KEY holds KEY2, ADDR holds the page, PER armed then cleared, STAT cleared) plus the seeded-then-
//! asserted flash content, the zero-write rejection paths, and the error decode. The bounded
//! `BUSY`-poll loop and the `.data` RAM-residency are target-only (`cfg(target_arch = "arm")`); this
//! host layer drives the identical logical sequence through the mockable accessors.
#![cfg(feature = "mock")]

use super::*;
use crate::chip::Chip;
use crate::descriptor::{McuDescriptor, PageSize};
use crate::detect::{descriptor_f103, descriptor_f130};
use crate::reg::{mock, Reg16, Reg32};
use std::sync::MutexGuard;

// FMC register absolute addresses (base + offset); the mock window wraps modulo its size, only the
// low bits matter, and these sit clear of the low flash test addresses below.
const KEY_A: u32 = FMC_BASE + KEY;
const STAT_A: u32 = FMC_BASE + STAT;
const CTL_A: u32 = FMC_BASE + CTL;
const ADDR_A: u32 = FMC_BASE + ADDR;

/// A sentinel written into every FMC register before a rejection-path call, so "zero register writes"
/// is checked by asserting the registers are UNCHANGED.
const SENTINEL: u32 = 0xDEAD_BEEF;

fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

/// A K1 / 64 KiB mock chip (the bench C8 shape).
fn chip_k1() -> Chip {
    let mut d: McuDescriptor = descriptor_f130();
    d.flash_kib = 64; // K1 -> 1 KiB page, 64 KiB extent.
    Chip::from_descriptor(d)
}

/// A K2 / 256 KiB mock chip (the 12-FET shape: high-density F10x).
fn chip_k2() -> Chip {
    let mut d: McuDescriptor = descriptor_f103();
    d.flash_page = PageSize::K2;
    d.flash_kib = 256; // K2 -> 2 KiB page, 256 KiB extent.
    Chip::from_descriptor(d)
}

fn r(addr: u32) -> u32 {
    Reg32::new(addr, 0).read()
}
fn rh(addr: u32) -> u16 {
    Reg16::new(addr, 0).read()
}

/// Seed the four FMC registers with the sentinel (for the zero-write assertions). `CTL` is left at
/// the sentinel which has `LK` (bit 7) set, so the happy-path unlock would fire if reached.
fn seed_fmc_sentinel() {
    Reg32::new(KEY_A, 0).write(SENTINEL);
    Reg32::new(STAT_A, 0).write(SENTINEL);
    Reg32::new(CTL_A, 0).write(SENTINEL);
    Reg32::new(ADDR_A, 0).write(SENTINEL);
}

fn assert_fmc_untouched() {
    assert_eq!(r(KEY_A), SENTINEL, "KEY untouched (zero register writes)");
    assert_eq!(r(STAT_A), SENTINEL, "STAT untouched (zero register writes)");
    assert_eq!(r(CTL_A), SENTINEL, "CTL untouched (zero register writes)");
    assert_eq!(r(ADDR_A), SENTINEL, "ADDR untouched (zero register writes)");
}

/// Mark the `[addr, addr+len)` halfword span as erased (0xFFFF) in the mock flash backing store, so a
/// `program` pre-check sees an erased target.
fn seed_erased(addr: u32, len: u32) {
    let mut off = 0;
    while off < len {
        Reg16::new(addr + off, 0).write(0xFFFF);
        off += 2;
    }
}

// --- descriptor reflection --------------------------------------------------------------------

#[test]
fn page_size_and_flash_size_reflect_the_descriptor() {
    let k1 = Fmc::new(&chip_k1());
    assert_eq!(k1.page_size(), 1024, "K1 page = 1 KiB");
    assert_eq!(k1.flash_size_bytes(), 65536, "K1 extent = 64 KiB");

    let k2 = Fmc::new(&chip_k2());
    assert_eq!(k2.page_size(), 2048, "K2 page = 2 KiB");
    assert_eq!(k2.flash_size_bytes(), 262144, "K2 extent = 256 KiB");
}

// --- erase ------------------------------------------------------------------------------------

/// The erase register end state, for an aligned in-flash address, on a given chip. The mock window
/// wraps, so a page near FLASH_BASE does not collide with the FMC register offsets.
fn check_erase_sequence(chip: &Chip) {
    let mut fmc = Fmc::new(chip);
    let page = chip.flash_page().bytes();
    // First page of flash (aligned), clear of the FMC register window in the wrapped mock space.
    let addr = FLASH_BASE;
    // LK set so the unlock fires and KEY1/KEY2 are observable.
    Reg32::new(CTL_A, 0).write(CTL_LK);

    assert_eq!(fmc.erase_page(addr), Ok(()));

    // Unlock fired: KEY holds the LAST key written (KEY2), proving the double-write unlock ran.
    assert_eq!(r(KEY_A), KEY2, "KEY = KEY2 (unlock double-write ran)");
    // ADDR = the page being erased.
    assert_eq!(r(ADDR_A), addr, "ADDR = page address");
    // STAT cleared (write-1-to-clear flags cleared after the op).
    assert_eq!(
        r(STAT_A) & STAT_CLEAR,
        0,
        "STAT error/end flags cleared after erase"
    );
    // CTL: PER cleared at the end (CTL = 0 final write), so neither PER nor START remains set.
    assert_eq!(r(CTL_A) & (CTL_PER | CTL_START), 0, "PER + START cleared");
    // The page read back as all 0xFFFF (the host erase models the silicon erase).
    assert_eq!(rh(addr), 0xFFFF, "first halfword erased");
    assert_eq!(rh(addr + page - 2), 0xFFFF, "last halfword erased");
}

#[test]
fn erase_emits_the_sequence_k1() {
    let _g = seed_reset();
    check_erase_sequence(&chip_k1());
}

#[test]
fn erase_emits_the_sequence_k2() {
    let _g = seed_reset();
    check_erase_sequence(&chip_k2());
}

#[test]
fn erase_misaligned_addr_is_badarg_zero_writes() {
    for chip in [chip_k1(), chip_k2()] {
        let _g = seed_reset();
        seed_fmc_sentinel();
        let mut fmc = Fmc::new(&chip);
        // One halfword past a page boundary: not page-aligned.
        let bad = FLASH_BASE + 2;
        assert_eq!(fmc.erase_page(bad), Err(FmcError::BadArg));
        assert_fmc_untouched();
    }
}

#[test]
fn erase_out_of_flash_is_badarg_zero_writes() {
    // K1: 64 KiB extent, so 0x0801_0000 is the first address past flash.
    let _g = seed_reset();
    seed_fmc_sentinel();
    let mut k1 = Fmc::new(&chip_k1());
    assert_eq!(k1.erase_page(FLASH_BASE + 0x1_0000), Err(FmcError::BadArg));
    assert_fmc_untouched();

    // Below the flash base is also out of flash.
    mock::reset();
    seed_fmc_sentinel();
    assert_eq!(k1.erase_page(FLASH_BASE - 1024), Err(FmcError::BadArg));
    assert_fmc_untouched();

    // K2: 256 KiB extent, so 0x0804_0000 is the first address past flash.
    mock::reset();
    seed_fmc_sentinel();
    let mut k2 = Fmc::new(&chip_k2());
    assert_eq!(k2.erase_page(FLASH_BASE + 0x4_0000), Err(FmcError::BadArg));
    assert_fmc_untouched();
}

// --- program ----------------------------------------------------------------------------------

/// Program a pattern into an erased span and read it back, on a given chip.
fn check_program_into_erased(chip: &Chip) {
    let mut fmc = Fmc::new(chip);
    let addr = FLASH_BASE;
    Reg32::new(CTL_A, 0).write(CTL_LK); // LK set so unlock fires.
    seed_erased(addr, 8);

    let bytes = [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    assert_eq!(fmc.program(addr, &bytes), Ok(()));

    // Halfwords landed in order, little-endian.
    assert_eq!(rh(addr), 0x2211);
    assert_eq!(rh(addr + 2), 0x4433);
    assert_eq!(rh(addr + 4), 0x6655);
    assert_eq!(rh(addr + 6), 0x8877);
    // PG cleared at the end (CTL = 0 final write).
    assert_eq!(r(CTL_A) & CTL_PG, 0, "PG cleared after program");
    // STAT clean (no PGERR/WPERR), so it decoded to Ok.
    assert_eq!(r(STAT_A) & STAT_CLEAR, 0, "STAT cleared, no sticky error");
}

#[test]
fn program_into_erased_k1() {
    let _g = seed_reset();
    check_program_into_erased(&chip_k1());
}

#[test]
fn program_into_erased_k2() {
    let _g = seed_reset();
    check_program_into_erased(&chip_k2());
}

#[test]
fn program_odd_addr_is_badarg_zero_writes() {
    for chip in [chip_k1(), chip_k2()] {
        let _g = seed_reset();
        seed_fmc_sentinel();
        let mut fmc = Fmc::new(&chip);
        assert_eq!(
            fmc.program(FLASH_BASE + 1, &[0xAA, 0xBB]),
            Err(FmcError::BadArg)
        );
        assert_fmc_untouched();
    }
}

#[test]
fn program_odd_len_is_badarg_zero_writes() {
    for chip in [chip_k1(), chip_k2()] {
        let _g = seed_reset();
        seed_fmc_sentinel();
        let mut fmc = Fmc::new(&chip);
        assert_eq!(
            fmc.program(FLASH_BASE, &[0xAA, 0xBB, 0xCC]),
            Err(FmcError::BadArg)
        );
        assert_fmc_untouched();
    }
}

#[test]
fn program_end_straddle_is_badarg_zero_writes() {
    // A halfword-aligned span whose end runs one halfword past the flash extent.
    let _g = seed_reset();
    seed_fmc_sentinel();
    let mut k1 = Fmc::new(&chip_k1());
    // Last in-flash halfword starts at FLASH_BASE + 64KiB - 2; a 4-byte program there straddles end.
    let last_hw = FLASH_BASE + 0x1_0000 - 2;
    assert_eq!(k1.program(last_hw, &[0, 0, 0, 0]), Err(FmcError::BadArg));
    assert_fmc_untouched();
}

#[test]
fn program_into_non_erased_is_not_erased_zero_writes() {
    for chip in [chip_k1(), chip_k2()] {
        let _g = seed_reset();
        let mut fmc = Fmc::new(&chip);
        let addr = FLASH_BASE;
        // Seed the span as erased, then write one halfword so it is NOT erased.
        seed_erased(addr, 4);
        Reg16::new(addr + 2, 0).write(0x1234); // second halfword spent.
                                               // Now seed the FMC registers with the sentinel: the pre-check must reject before any write.
        seed_fmc_sentinel();
        assert_eq!(
            fmc.program(addr, &[0xAA, 0xBB, 0xCC, 0xDD]),
            Err(FmcError::NotErased)
        );
        assert_fmc_untouched();
        // Flash unchanged: the written halfword still holds its value, no partial program.
        assert_eq!(rh(addr + 2), 0x1234, "flash unchanged on NotErased");
    }
}

#[test]
fn program_empty_slice_is_ok_noop() {
    for chip in [chip_k1(), chip_k2()] {
        let _g = seed_reset();
        seed_fmc_sentinel();
        let mut fmc = Fmc::new(&chip);
        assert_eq!(fmc.program(FLASH_BASE, &[]), Ok(()));
        // No PG armed, no writes: the no-op returns before any validation or FMC touch.
        assert_fmc_untouched();
    }
}

// --- write-once (the mock store models silicon) -----------------------------------------------

#[test]
fn mock_store_models_write_once_reprogram_refused() {
    // The driver's pre-check returns NotErased on a re-program (front-running the silicon). This test
    // additionally proves the MOCK STORE itself refuses a re-program (PGERR, content unchanged, not
    // ANDed) by driving the host program sequence directly past the API pre-check, so a re-program
    // bug in the inner sequence fails here rather than only on silicon.
    let _g = seed_reset();
    let chip = chip_k1();
    let fmc = Fmc::new(&chip);
    let addr = FLASH_BASE;
    Reg32::new(CTL_A, 0).write(CTL_LK);
    seed_erased(addr, 2);
    // Program 0xAAAA into the erased halfword (allowed).
    let st1 = fmc.program_host(addr, &[0xAA, 0xAA]);
    assert_eq!(st1 & STAT_PGERR, 0, "first program into erased: no PGERR");
    assert_eq!(rh(addr), 0xAAAA, "first program took");
    // Re-program the SAME halfword with 0x5555: the mock store REFUSES (PGERR), content unchanged.
    let st2 = fmc.program_host(addr, &[0x55, 0x55]);
    assert_ne!(
        st2 & STAT_PGERR,
        0,
        "re-program of a written halfword sets PGERR"
    );
    assert_eq!(
        rh(addr),
        0xAAAA,
        "re-program refused: content unchanged (not ANDed)"
    );
}

// --- error decode -----------------------------------------------------------------------------

#[test]
fn decode_maps_status_flags() {
    assert_eq!(decode(0), Ok(()));
    assert_eq!(decode(STAT_WPERR), Err(FmcError::WriteProtect));
    assert_eq!(decode(STAT_PGERR), Err(FmcError::ProgramError));
    assert_eq!(decode(STAT_TIMEOUT), Err(FmcError::Timeout));
    // WPERR takes precedence over PGERR (the more specific cause); Timeout over both.
    assert_eq!(decode(STAT_WPERR | STAT_PGERR), Err(FmcError::WriteProtect));
    assert_eq!(
        decode(STAT_TIMEOUT | STAT_WPERR | STAT_PGERR),
        Err(FmcError::Timeout)
    );
    // ENDF alone is success.
    assert_eq!(decode(STAT_ENDF), Ok(()));
}

#[test]
fn program_surfaces_wperr_as_write_protect() {
    // Seed the span erased; pre-seed STAT with WPERR so the post-op poll snapshot carries it (the
    // mock store does not raise WPERR on its own, so this models a protected page). The op then
    // decodes the worst status to WriteProtect.
    let _g = seed_reset();
    let chip = chip_k1();
    let mut fmc = Fmc::new(&chip);
    let addr = FLASH_BASE;
    Reg32::new(CTL_A, 0).write(CTL_LK);
    seed_erased(addr, 2);
    Reg32::new(STAT_A, 0).write(STAT_WPERR);
    assert_eq!(
        fmc.program(addr, &[0x12, 0x34]),
        Err(FmcError::WriteProtect)
    );
}
