//! FMC flash erase/program driver (the on-target flash primitive).
//!
//! [`Fmc`] is the family-aware FMC (flash memory controller) mechanism the HAL owns, like every
//! other peripheral driver. It pokes the GD32 FMC controller (`0x4002_2000`) to erase pages and
//! program halfwords at ABSOLUTE flash addresses. It knows nothing about flash PLACEMENT (regions,
//! store layout, app slots) or pin safety, that is firmware policy (see the FMC spec, "Mechanism vs
//! policy"). Two consumers, one primitive: the config store and the bootloader both program flash
//! through this, neither hand-rolls the FMC register sequence.
//!
//! # The chip is the detected chip
//!
//! [`Fmc::new`] takes the runtime-detected [`Chip`]: the FMC base is the fixed `0x4002_2000` on both
//! families (a constant), and the page size + flash extent come from the descriptor (the extent from
//! the `0x1FFF_F7E0` density read [`crate::detect`] performs at boot). So `new` is INFALLIBLE, no
//! `Result`: there is no failure mode (unlike `Usart::bring_up`, which can `MissingBase` when a port
//! is absent on the part). Size + page are read-only descriptor facts, never probed by writing.
//!
//! # Granularity (three distinct units, bench-confirmed)
//!
//! - **read**: any byte / any width, flash is memory-mapped, a plain load (the caller does it, no
//!   FMC). There is no `read` method here.
//! - **program (write)**: a HALFWORD (16 bits) is the smallest unit AND the unit of allocation, it
//!   must be erased (`0xFFFF`) and is WRITE-ONCE. Re-programming a written halfword is REFUSED by the
//!   controller with `PGERR` (the content is left UNCHANGED, it does NOT clear-more-bits / does NOT
//!   AND, bench-confirmed). So [`Fmc::program`] pre-checks the whole span is erased and returns
//!   [`FmcError::NotErased`] BEFORE touching the FMC (all-or-nothing, no partial write); the silicon
//!   `PGERR` is kept only as the backstop the pre-check front-runs.
//! - **erase**: a whole PAGE (1 KiB C8 / 2 KiB 12-FET) back to `0xFFFF`, the only way to reclaim a
//!   spent halfword. [`Fmc::erase_page`] is page-at-a-time only (no mass erase: it keeps a bricking
//!   footgun out of the immutable bootloader's reach).
//!
//! # Straddle: reject, do not read-modify-write
//!
//! [`Fmc::program`] requires halfword-aligned `addr`/`len`. The store already aligns/pads and the
//! bootloader pads its image to even length, so a straddle is a caller bug, rejected with
//! [`FmcError::BadArg`]. The disproven read-modify-write straddle path (built on a NOR-AND model that
//! silicon refuses, see the FMC spec) is deliberately NOT lifted from the firmware's `store/fmc.rs`.
//!
//! # Register model (bank0; identical on F10x and F1x0, no per-family selector)
//!
//! Confirmed against GD32F10x Rev2.6 / GD32F1x0 Rev3.6: the bank0 registers, bit positions, and
//! unlock keys are identical across both families (and match STM32F10x), so there is no
//! `UsartModel`-style per-family register model here, just the page size + extent from the
//! descriptor. The driver is bank0-only by design (parts <= 512 KiB are single-bank; the whole fleet
//! including the 256 KiB 12-FET is single-bank). A future > 512 KiB part would be the trigger to add
//! bank1, not built pre-emptively.
//!
//! | reg    | offset | bits                                                                |
//! |--------|--------|---------------------------------------------------------------------|
//! | `KEY`  | `0x04` | unlock: write KEY1 (`0x4567_0123`) then KEY2 (`0xCDEF_89AB`)         |
//! | `STAT` | `0x0C` | `BUSY` b0, `PGERR` b2, `WPERR` b4, `ENDF` b5                         |
//! | `CTL`  | `0x10` | `PG` b0, `PER` b1, `START` b6, `LK` b7                               |
//! | `ADDR` | `0x14` | the absolute flash address for a page erase                         |
//!
//! # RAM-resident critical section (target-only)
//!
//! An FMC erase/program stalls flash instruction fetch for the whole bank (a page erase is tens of
//! ms), so the inner unlock -> command -> `BUSY`-poll runs FROM RAM (`#[link_section = ".data"]`,
//! startup copies `.data` to RAM so the bytes execute from RAM, not the stalled flash) with
//! interrupts off (PRIMASK). That, and the raw-MMIO that backs it, are the TARGET-ONLY bits, gated on
//! `cfg(target_arch = "arm")`. The host build / host tests run the SAME logical sequence through the
//! mockable [`crate::reg`] accessors (no critical section, no `.data` placement), so the validation,
//! pre-check, sequence, and error decode are all host-tested, exactly how [`crate::adc`] separates
//! target MMIO from the host mock. PRIMASK is off for the duration of ONE op only (a single page
//! erase / halfword program, each well under any watchdog timeout); a caller erasing many pages feeds
//! the watchdog BETWEEN ops, never wraps the whole loop in one interrupts-off span.
//!
//! # Wait states
//!
//! None needed on these parts: the GD32 has zero instruction-fetch wait states within the first
//! 256 KiB even at the 108 MHz max (all fleet flash is inside that), so the bootloader needs no
//! `FMC_WS` config. This is OPPOSITE STM32, which needs a wait-state ladder. (Moot for the
//! erase/program code itself, which is RAM-resident.)

use crate::chip::Chip;
use crate::error::FmcError;
use crate::reg::Reg16;
// `Reg32` is reached only on the host path (the FMC register accessor + the host sequence); the
// target path uses raw RAM-resident MMIO, so the import is host-only to avoid an unused-import warning
// on the `thumbv7m` build.
#[cfg(not(target_arch = "arm"))]
use crate::reg::Reg32;

// --- FMC base + register offsets (bank0; identical on both families) --------------------------

/// FMC peripheral base, the fixed `0x4002_2000` on both families (a constant, not a descriptor
/// lookup), which is why [`Fmc::new`] is infallible.
const FMC_BASE: u32 = 0x4002_2000;
/// Unlock key register (`FMC_KEY`), offset 0x04. Write KEY1 then KEY2 to clear the `LK` bit.
const KEY: u32 = 0x04;
/// Status register (`FMC_STAT`), offset 0x0C: `BUSY` b0, `PGERR` b2, `WPERR` b4, `ENDF` b5.
const STAT: u32 = 0x0C;
/// Control register (`FMC_CTL`), offset 0x10: `PG` b0, `PER` b1, `START` b6, `LK` b7.
const CTL: u32 = 0x10;
/// Address register (`FMC_ADDR`), offset 0x14. The absolute flash address for a page erase.
const ADDR: u32 = 0x14;

/// Unlock key 1 (`FMC_KEY` first write).
const KEY1: u32 = 0x4567_0123;
/// Unlock key 2 (`FMC_KEY` second write); the pair clears `LK`.
const KEY2: u32 = 0xCDEF_89AB;

// CTL bits.
const CTL_PG: u32 = 1 << 0;
const CTL_PER: u32 = 1 << 1;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;

// STAT flags.
const STAT_BUSY: u32 = 1 << 0;
const STAT_PGERR: u32 = 1 << 2;
const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;
/// The write-1-to-clear error/end flags cleared after EVERY op (and on the error path), matching the
/// proven C tool's `0x34` = `ENDF | WPERR | PGERR`. Flags are STICKY: a left-set `PGERR` corrupts the
/// NEXT operation (an uncleared error fails the following erase), so the clear is mandatory.
const STAT_CLEAR: u32 = STAT_ENDF | STAT_WPERR | STAT_PGERR;

/// Main flash base on both families (the absolute-address space `Fmc` operates in). The out-of-flash
/// bound is `[FLASH_BASE, FLASH_BASE + flash_size_bytes())`.
const FLASH_BASE: u32 = 0x0800_0000;

/// The erased-halfword value (NOR erased state).
const ERASED: u16 = 0xFFFF;

/// Bounded `BUSY`-poll budget, a fixed `u32` count sized at the MAX clock (F10x 108 MHz / F1x0
/// 72 MHz). An FMC erase/program is internally timed (the flash charge pump), so its duration is a
/// roughly-FIXED real time (datasheet worst cases: page erase max 400 ms, word program max 105 us),
/// NOT scaled by HCLK, but the poll loop's iterations-per-ms DO scale with HCLK. The same op is
/// called from the bootloader (maybe 8 MHz) AND the config store at 72 MHz, so the count is sized at
/// the max clock so it never false-`Timeout`s at the high end: at 108 MHz `~1e8 x ~2 cyc / 108e6
/// ~= 1.8 s`, ~4.5x over the 400 ms erase worst case, and fits `u32`. At lower clocks it is just a
/// longer (still-bounded) backstop, harmless since it only ever fires on a genuinely stuck op where
/// exact duration does not matter, only that it is finite. DWT `CYCCNT` is NOT used (the 32-bit
/// counter wraps at `2^32 / 108 MHz ~= 40 ms`, so it cannot span a 400 ms erase without
/// wrap-counting, not worth the complexity in a PRIMASK-off section).
pub const FMC_BUSY_TIMEOUT: u32 = 100_000_000;

/// The FMC flash erase/program driver, resolved once from the detected [`Chip`]: the fixed base plus
/// the page size + flash extent from the descriptor (DECISIONS.md #4: resolve once into a concrete
/// `Copy` handle).
#[derive(Debug, Clone, Copy)]
pub struct Fmc {
    /// Erase/program granularity in bytes, from the descriptor's [`crate::descriptor::PageSize`].
    page_size: u32,
    /// Total flash size in bytes, the out-of-flash bound, from the descriptor's density read.
    flash_extent: u32,
}

impl Fmc {
    /// Acquire the FMC for the detected `chip`. INFALLIBLE: the base is the fixed [`FMC_BASE`]
    /// constant and the page size + extent are always in the descriptor, so there is no failure mode
    /// (a `Result` that can never be `Err` would only force callers to handle an impossible error).
    #[inline]
    pub fn new(chip: &Chip) -> Fmc {
        Fmc {
            page_size: chip.flash_page().bytes(),
            flash_extent: chip.flash_size_bytes(),
        }
    }

    /// Erase/program granularity in bytes (1 KiB on the medium-density C8s, 2 KiB on the high-density
    /// 12-FET), FROM THE DESCRIPTOR. [`Fmc::erase_page`] aligns to this value.
    #[inline]
    pub const fn page_size(&self) -> u32 {
        self.page_size
    }

    /// Total flash size in bytes (`flash_kib * 1024`, from the descriptor's `0x1FFF_F7E0` read). The
    /// absolute-address bound the erase/program out-of-flash `BadArg` check uses. Read-only; the
    /// driver never probes the extent by writing.
    #[inline]
    pub const fn flash_size_bytes(&self) -> u32 {
        self.flash_extent
    }

    /// Erase the page at the absolute flash `addr` -> all `0xFF`. `addr` MUST be [`page_size`]-aligned
    /// and in flash, else [`FmcError::BadArg`] (rejected, NOT rounded down to the containing page),
    /// caught BEFORE the FMC is touched. The erase dance (PER, ADDR, START, bounded `BUSY` poll, clear
    /// flags, clear PER) runs RAM-resident with interrupts off on the target.
    ///
    /// [`page_size`]: Fmc::page_size
    pub fn erase_page(&mut self, addr: u32) -> Result<(), FmcError> {
        // Arg validation BEFORE the FMC: page-aligned and inside flash, else BadArg with zero writes.
        // The page-alignment check runs only when the address is in flash (so the subtraction below
        // never underflows).
        if !self.in_flash(addr, self.page_size)
            || !(addr - FLASH_BASE).is_multiple_of(self.page_size)
        {
            return Err(FmcError::BadArg);
        }
        let stat = self.run_erase(addr);
        decode(stat)
    }

    /// Program `bytes` at absolute flash `addr`, halfword (16-bit) quantum: `addr` and `bytes.len()`
    /// must both be halfword-aligned and the span must be in flash, else [`FmcError::BadArg`]; an
    /// empty slice is an `Ok` no-op.
    ///
    /// PREVENT, don't probe-by-failing: the whole target span MUST be erased (`0xFFFF`). `program`
    /// first READS the span and, if any halfword is not erased, returns [`FmcError::NotErased`] BEFORE
    /// touching the FMC (all-or-nothing, no partial write left in flash). Else it programs halfwords
    /// IN ORDER (PG, write u16, bounded poll, check + clear flags). `WPERR` -> [`FmcError::WriteProtect`],
    /// the unforeseen `PGERR` backstop -> [`FmcError::ProgramError`]. Sticky STAT flags are cleared
    /// after the op including the error path.
    ///
    /// To change a value, erase the whole page first; never re-program a written halfword (not even to
    /// clear more bits). An incremental/append caller advances `addr` across fresh halfwords in one
    /// erased page without re-erasing (see the FMC spec).
    pub fn program(&mut self, addr: u32, bytes: &[u8]) -> Result<(), FmcError> {
        // Empty slice: a no-op, before any validation or FMC touch.
        if bytes.is_empty() {
            return Ok(());
        }
        let len = bytes.len() as u32;
        // Arg validation BEFORE the FMC: halfword-aligned addr + len, span inside flash, else BadArg
        // with zero writes. (A straddle of the flash end fails in_flash; an odd addr/len fails the
        // alignment check.)
        if !addr.is_multiple_of(2) || !len.is_multiple_of(2) || !self.in_flash(addr, len) {
            return Err(FmcError::BadArg);
        }
        // Pre-check the whole span is erased (0xFFFF) BEFORE touching the FMC: a NON-erased target is
        // NotErased with zero register writes, flash unchanged (the silicon PGERR is the backstop this
        // front-runs). The reads go through the mockable Reg16 accessor (flash is memory-mapped and
        // readable here, this is NOT mid-operation), so host tests can seed flash content.
        let mut off = 0u32;
        while off < len {
            if Reg16::new(addr + off, 0).read() != ERASED {
                return Err(FmcError::NotErased);
            }
            off += 2;
        }
        // The span is erased: program it halfword by halfword in order.
        let stat = self.run_program(addr, bytes);
        decode(stat)
    }

    /// True iff `[addr, addr + span)` lies inside main flash (`[FLASH_BASE, FLASH_BASE + extent)`),
    /// with no wraparound. The out-of-flash `BadArg` bound for both ops.
    #[inline]
    fn in_flash(&self, addr: u32, span: u32) -> bool {
        if addr < FLASH_BASE {
            return false;
        }
        match addr.checked_add(span) {
            Some(end) => end <= FLASH_BASE + self.flash_extent,
            None => false,
        }
    }

    /// A 32-bit FMC register accessor at `off` from the fixed [`FMC_BASE`]. Host-only: the target
    /// inner sequence uses raw RAM-resident MMIO, so this mockable accessor is reached only on the
    /// host path (and the host tests).
    #[cfg(not(target_arch = "arm"))]
    #[inline]
    fn reg(&self, off: u32) -> Reg32 {
        Reg32::new(FMC_BASE, off)
    }
}

/// Decode a returned `FMC_STAT` snapshot into a [`FmcError`] result: `WPERR` -> [`FmcError::WriteProtect`],
/// `PGERR` -> [`FmcError::ProgramError`] (the backstop the [`Fmc::program`] pre-check front-runs), the
/// sentinel [`STAT_TIMEOUT`] (set by the bounded poll on exhaustion) -> [`FmcError::Timeout`], else
/// `Ok`. WPERR is checked before PGERR (a protected page is the more specific cause).
fn decode(stat: u32) -> Result<(), FmcError> {
    if stat & STAT_TIMEOUT != 0 {
        return Err(FmcError::Timeout);
    }
    if stat & STAT_WPERR != 0 {
        return Err(FmcError::WriteProtect);
    }
    if stat & STAT_PGERR != 0 {
        return Err(FmcError::ProgramError);
    }
    Ok(())
}

/// A sentinel bit OR-ed into the returned status by the bounded poll when it exhausts its budget,
/// so [`decode`] maps a stuck op to [`FmcError::Timeout`]. It is NOT a real FMC_STAT bit (bit 1 is
/// unused in the bank0 STAT on both families), so it never collides with a hardware flag.
const STAT_TIMEOUT: u32 = 1 << 1;

// ==============================================================================================
// The inner erase/program sequence, split target (RAM-resident raw MMIO) vs host (mockable reg).
//
// Both run the SAME logical steps; only the MMIO mechanism and the RAM-residency differ. The target
// path is the proven sequence lifted from the firmware's store/fmc.rs (unlock / PER / ADDR / START /
// BUSY-poll / flag-clear, halfword PG + write16 + poll), placed in `.data` and run with interrupts
// off. The host path drives the identical sequence through the mockable Reg32/Reg16 accessors so the
// logic is host-tested; it ALSO models silicon write-once (a halfword store to a non-0xFFFF cell is
// refused with PGERR, not ANDed), so a re-program bug fails a host test rather than only on silicon.
// ==============================================================================================

impl Fmc {
    /// Erase one page at `addr`, returning the (possibly timeout-sentinel-augmented) FMC_STAT.
    #[inline]
    fn run_erase(&self, addr: u32) -> u32 {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: addr is page-aligned and in flash (checked by erase_page); the routine runs from
            // RAM with interrupts off (mandatory, an erase stalls flash fetch).
            unsafe { critical(|| target::erase_ram(addr)) }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            self.erase_host(addr)
        }
    }

    /// Program `bytes` at `addr` (halfword-aligned, erased, in flash), returning the FMC_STAT.
    #[inline]
    fn run_program(&self, addr: u32, bytes: &[u8]) -> u32 {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: [addr, addr+len) is halfword-aligned and in flash (checked by program); runs
            // from RAM with interrupts off.
            unsafe { critical(|| target::program_ram(addr, bytes)) }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            self.program_host(addr, bytes)
        }
    }

    // --- host sequence (the mockable reg path; this is what the host tests exercise) --------------

    /// Host erase: the identical unlock / PER / ADDR / START / poll / flag-clear / clear-PER sequence
    /// as the target path, through the mockable [`Reg32`] accessor.
    #[cfg(not(target_arch = "arm"))]
    fn erase_host(&self, addr: u32) -> u32 {
        self.unlock_host();
        self.reg(CTL).write(CTL_PER);
        self.reg(ADDR).write(addr);
        self.reg(CTL).write(CTL_PER | CTL_START);
        // Model the erase: on real silicon START kicks the page erase; the mock backing store has no
        // such effect, so the host path zeroes-to-0xFFFF the page itself so a later read-back / a
        // program pre-check sees the erased state, mirroring the silicon. This keeps the store
        // self-consistent for append/read tests without a separate sequencer.
        let mut off = 0u32;
        while off < self.page_size {
            Reg16::new(addr + off, 0).write(ERASED);
            off += 2;
        }
        let stat = self.wait_host();
        self.reg(CTL).write(0);
        stat
    }

    /// Host program: the identical PG / per-halfword write16 + poll / clear-PG sequence as the target
    /// path, through the mockable [`Reg16`]/[`Reg32`] accessors, MODELING silicon write-once (a store
    /// to a non-erased halfword is refused with PGERR, content unchanged, NOT ANDed).
    #[cfg(not(target_arch = "arm"))]
    fn program_host(&self, addr: u32, bytes: &[u8]) -> u32 {
        self.unlock_host();
        self.reg(CTL).write(CTL_PG);
        let mut worst = 0u32;
        let mut i = 0usize;
        while i < bytes.len() {
            let a = addr + i as u32;
            let hw = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
            let cell = Reg16::new(a, 0);
            if cell.read() != ERASED {
                // Write-once: silicon refuses a re-program with PGERR and leaves the cell UNCHANGED.
                self.reg(STAT).modify(STAT_PGERR, STAT_PGERR);
            } else {
                cell.write(hw);
            }
            worst |= self.wait_host();
            i += 2;
        }
        self.reg(CTL).write(0);
        worst
    }

    /// Host unlock: write KEY1 then KEY2 only when `LK` is set (a redundant unlock re-locks the FPEC).
    #[cfg(not(target_arch = "arm"))]
    fn unlock_host(&self) {
        if self.reg(CTL).read() & CTL_LK != 0 {
            self.reg(KEY).write(KEY1);
            self.reg(KEY).write(KEY2);
        }
    }

    /// Host `BUSY` poll: bounded by [`FMC_BUSY_TIMEOUT`]; on exhaustion OR a clear poll, clear the
    /// sticky `ENDF | WPERR | PGERR` flags and return the snapshot (with the [`STAT_TIMEOUT`] sentinel
    /// OR-ed in on exhaustion). The mock backing store never sets `BUSY`, so the loop falls through on
    /// the first read, which is the post-completion state the target reaches after the charge pump.
    #[cfg(not(target_arch = "arm"))]
    fn wait_host(&self) -> u32 {
        let mut budget = FMC_BUSY_TIMEOUT;
        loop {
            let st = self.reg(STAT).read();
            if st & STAT_BUSY == 0 {
                // Clear the write-1-to-clear flags and return the snapshot for error classification.
                self.reg(STAT).modify(STAT_CLEAR, 0);
                return st;
            }
            budget -= 1;
            if budget == 0 {
                self.reg(STAT).modify(STAT_CLEAR, 0);
                return st | STAT_TIMEOUT;
            }
        }
    }
}

// --- target RAM-resident raw-MMIO sequence (the proven store/fmc.rs path) ---------------------
//
// `cfg(target_arch = "arm")`-gated: these run from RAM (`#[link_section = ".data"]`, copied to RAM by
// cortex-m-rt startup) so the core fetches+executes them from RAM, not the flash bank the FMC stalls
// mid-operation. `#[inline(never)]` so the body cannot be hoisted into a flash-resident caller; they
// touch only MMIO + their arguments (no flash literals / no calls into flash). `critical` runs them
// with interrupts off (PRIMASK) so no ISR fetches from the stalled bank or re-enters the FMC.

#[cfg(target_arch = "arm")]
mod target {
    use super::{
        ADDR, CTL, CTL_LK, CTL_PER, CTL_PG, CTL_START, FMC_BASE, FMC_BUSY_TIMEOUT, KEY, KEY1, KEY2,
        STAT, STAT_BUSY, STAT_CLEAR, STAT_TIMEOUT,
    };
    use core::ptr::{read_volatile, write_volatile};

    const KEY_PTR: *mut u32 = (FMC_BASE + KEY) as *mut u32;
    const STAT_PTR: *mut u32 = (FMC_BASE + STAT) as *mut u32;
    const CTL_PTR: *mut u32 = (FMC_BASE + CTL) as *mut u32;
    const ADDR_PTR: *mut u32 = (FMC_BASE + ADDR) as *mut u32;

    /// Unlock the FPEC if locked: write KEY1 then KEY2 only when `LK` is set (a double-unlock
    /// re-locks). RAM-resident.
    #[link_section = ".data"]
    #[inline(never)]
    pub(super) unsafe fn unlock_ram() {
        if read_volatile(CTL_PTR) & CTL_LK != 0 {
            write_volatile(KEY_PTR, KEY1);
            write_volatile(KEY_PTR, KEY2);
        }
    }

    /// Poll `FMC_STAT` until `BUSY` clears (bounded by [`FMC_BUSY_TIMEOUT`]), clear the write-1-to-clear
    /// flags, and return the snapshot (with the [`STAT_TIMEOUT`] sentinel OR-ed in on exhaustion).
    /// RAM-resident.
    #[link_section = ".data"]
    #[inline(never)]
    pub(super) unsafe fn wait_ram() -> u32 {
        let mut budget = FMC_BUSY_TIMEOUT;
        loop {
            let st = read_volatile(STAT_PTR);
            if st & STAT_BUSY == 0 {
                write_volatile(STAT_PTR, STAT_CLEAR);
                return st;
            }
            budget -= 1;
            if budget == 0 {
                write_volatile(STAT_PTR, STAT_CLEAR);
                return st | STAT_TIMEOUT;
            }
        }
    }

    /// Erase one page: unlock; PER; ADDR; PER|START; wait; CTL=0. Returns FMC_STAT. RAM-resident (an
    /// erase is tens of ms during which flash fetch is stalled).
    #[link_section = ".data"]
    #[inline(never)]
    pub(super) unsafe fn erase_ram(addr: u32) -> u32 {
        unlock_ram();
        write_volatile(CTL_PTR, CTL_PER);
        write_volatile(ADDR_PTR, addr);
        write_volatile(CTL_PTR, CTL_PER | CTL_START);
        let st = wait_ram();
        write_volatile(CTL_PTR, 0);
        st
    }

    /// Program `bytes` at absolute `addr` by halfwords in order: unlock; PG; per halfword write16 +
    /// wait; CTL=0. `addr`/`len` are halfword-aligned and the span erased (checked by the caller), so
    /// there is NO read-modify-write of a straddling halfword (the disproven NOR-AND path). A
    /// write-once cell that somehow is not erased sets PGERR, surfaced via the returned status (the
    /// backstop the caller's pre-check front-runs). Returns the worst FMC_STAT seen (any error
    /// sticks). RAM-resident.
    #[link_section = ".data"]
    #[inline(never)]
    pub(super) unsafe fn program_ram(addr: u32, bytes: &[u8]) -> u32 {
        unlock_ram();
        write_volatile(CTL_PTR, CTL_PG);
        let mut worst = 0u32;
        let mut i = 0usize;
        let n = bytes.len();
        while i < n {
            let a = addr + i as u32;
            let hw = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
            // Halfword store to flash triggers the program (the span is erased + halfword-aligned, so
            // no read-modify-write straddle).
            write_volatile(a as *mut u16, hw);
            worst |= wait_ram();
            i += 2;
        }
        write_volatile(CTL_PTR, 0);
        worst
    }
}

/// Run `f` with interrupts disabled (PRIMASK set), restoring the prior PRIMASK after. Target-only:
/// the FMC erase/program inner sequence must run with no ISR fetching from the stalled flash bank or
/// re-entering the FMC.
#[cfg(target_arch = "arm")]
#[inline(always)]
unsafe fn critical<R>(f: impl FnOnce() -> R) -> R {
    let primask: u32;
    core::arch::asm!("mrs {}, PRIMASK", out(reg) primask, options(nomem, nostack, preserves_flags));
    core::arch::asm!("cpsid i", options(nomem, nostack, preserves_flags));
    let r = f();
    // Restore: only re-enable if it was enabled before (PRIMASK bit0 == 0 meant enabled).
    if primask & 1 == 0 {
        core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
    }
    r
}

#[cfg(test)]
mod tests;
