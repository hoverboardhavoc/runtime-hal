//! Shared I2C driver (T6 bring-up + T7 polled transfer / `embedded-hal` `i2c::I2c`).
//!
//! This is the single I2C bring-up + transfer path. SPEC.md: I2C is the **classic event-based**
//! block on both families (CTL0/CTL1/STAT0/STAT1/CKCFG/RT, the STM32F1-style peripheral, no
//! `TIMINGR`), so unlike [`crate::usart`] there is **one register model shared by both families**:
//! the I2C peripheral offsets and bit positions are identical on F10x and F1x0 (verified against
//! `gd32f10x_i2c.h` and `gd32f1x0_i2c.h`). The path is parameterised only by the base address
//! (data, from [`crate::addr::AddrTable`]) and by the bus clock (from the [`ClockConfig`]); there
//! is no [`crate::ClockPath`]-style selector here.
//!
//! # Register model (identical on both families)
//!
//! | reg     | offset | what                                                              |
//! |---------|--------|-------------------------------------------------------------------|
//! | `CTL0`  | `0x00` | I2CEN(0), START(8), STOP(9), ACKEN(10) (`*_i2c.h`)                 |
//! | `CTL1`  | `0x04` | `I2CCLK[6:0]` peripheral-clock-MHz field (`I2C_CTL1_I2CCLK`)       |
//! | `SADDR0`| `0x08` | own address + address format (7-bit)                              |
//! | `DATA`  | `0x10` | transfer buffer (address byte / TX / RX)                          |
//! | `STAT0` | `0x14` | SBSEND(0) ADDSEND(1) BTC(2) RBNE(6) TBE(7) BERR(8) LOSTARB(9) AERR(10) OUERR(11) |
//! | `STAT1` | `0x18` | MASTER(0) I2CBSY(1) TR(2); read after STAT0 clears ADDSEND        |
//! | `CKCFG` | `0x1C` | `CLKC[11:0]`, DTCY(14), FAST(15)                                  |
//! | `RT`    | `0x20` | `RISETIME[6:0]`                                                   |
//!
//! # Timing (CKCFG / RT) and the CKCFG formula (open item I2C-1)
//!
//! Bus speed is carried as a free `u32` Hz in the [`I2cMode`] passed to [`I2c::new`] (not a fixed
//! standard/fast enum table), so any rate the SPL accepts is expressible; the IMU runs at 100 kHz, the
//! reference firmware at 400 kHz, and both fall out of the same formula. The CKCFG/RT/I2CCLK values
//! are computed by [`timing_for`] EXACTLY as the GD SPL `i2c_clock_config` does, from the APB1 bus
//! clock (which [`ClockConfig`] + the I2C-on-APB1 fact give) and the target speed:
//!
//! - `I2CCLK = clamp(pclk1 / 1_000_000, 2, 0x48)` (the peripheral-clock MHz field, `I2CCLK_MIN`..`MAX`).
//! - **Standard mode** (speed <= 100 kHz): `RT = clamp(pclk1/1e6 + 1, 2, 0x48)`,
//!   `CLKC = max(pclk1 / (speed*2), 4)`, `CKCFG = CLKC` (FAST = 0, DTCY = 0).
//! - **Fast mode** (100 kHz < speed <= 400 kHz): `RT = (I2CCLK*300)/1000 + 1`; with DTCY_2
//!   `CLKC = pclk1/(speed*3)` (DTCY = 0), with DTCY_16/9 `CLKC = pclk1/(speed*25)` (DTCY = 1);
//!   if CLKC's low 12 bits are 0 force CLKC |= 1; `CKCFG = FAST | (DTCY?) | CLKC`.
//!
//! Because CKCFG depends on the APB1 clock, a clock-tree mistake (wrong PLL / prescaler) surfaces
//! here as a wrong CKCFG value, exactly the way M1's BAUD caught a wrong USART clock. At the proven
//! 72 MHz tree APB1 = 36 MHz, so at 100 kHz: I2CCLK = 36 (0x24), RT = 37 (0x25), CKCFG = 180 (0xB4).
//!
//! The bench probe (`i2c_probe.c`) ran at the 8 MHz reset clock (pclk1 = 8 MHz), which gives
//! I2CCLK = 8, RT = 9, CKCFG = 40; runtime-hal reproduces whatever the supplied [`ClockConfig`]
//! implies, so the same code matches the probe at 8 MHz and the 72 MHz bring-up.
//!
//! # Transfer sequence (T7): master receive is the manual's **Solution B**, plus error recovery
//!
//! Master transmit is the classic event-based STAT0/STAT1 handshake: START -> SBSEND -> address ->
//! ADDSEND (cleared by reading STAT0 then STAT1) -> bytes via TBE with a final BTC. Transmit is
//! inherently delay-tolerant (the master stretches SCL when TBE/BTC go unserviced).
//!
//! Master **receive** uses the user manual's **Solution B** (GD32F1x0 UM Rev 3.6 / GD32F10x UM
//! Rev 2.6, "Programming model in master receiving mode"), NOT Solution A. The manual is explicit
//! that "Solution A requires the software's quick response to I2C events, while Solution B
//! doesn't": Solution A's ACKEN-clear + STOP must land inside the last byte's reception, so any
//! interrupt delaying the polled loop at that point lets the last byte be ACKed and the STOP fall
//! mid-byte, which can corrupt the transfer and wedge the bus. **Observed on silicon
//! (2026-07-18, the integrated firmware's 250 Hz IMU burst on the bench F130 under SysTick + DMA
//! ISR load): the peripheral deadlocked with CTL0 = I2CEN|START|STOP|ACKEN pending, STAT1 =
//! I2CBSY-with-MASTER-clear, STAT0 = 0, permanently, while the same 14-byte burst runs error-free
//! in an interrupt-free image.** Solution B parks every software decision under an SCL stretch
//! (RBNE+BTC both set holds SCL low), so ISR latency can never break the NACK/STOP placement:
//!
//! - N = 1: ACKEN clear before clearing ADDSEND; STOP after clearing ADDSEND; read on RBNE.
//! - N = 2: POAP set before START (ACKEN then governs the *next* byte); ACKEN clear before
//!   clearing ADDSEND; wait BTC (both bytes held, SCL stretched); STOP; read twice.
//! - N > 2: read via RBNE until N-3 bytes are in hand; wait BTC (byte N-2 in DR, N-1 in shift,
//!   SCL stretched); clear ACKEN; read N-2 (the last byte then arrives NACKed); wait BTC again
//!   (stretched); STOP; read N-1; read N.
//!
//! Every poll is bounded so a missing/stuck device cannot hang (the F130 hang-if-done-wrong
//! class), and a fresh transfer first waits (bounded) for I2CBSY to clear. The bound is scoped to
//! a whole bus acquisition and derived from the core clock, so it is a WALL-CLOCK guarantee
//! ([`worst_case_block_us`]) rather than an iteration count whose duration happens to fall out of
//! codegen; see "the polled-transfer time budget" below for why the caller needs it in those terms.
//!
//! **Error recovery (the persistence half of the same silicon finding):** the sticky STAT0 error
//! flags (BERR/LOSTARB/AERR/OUERR, all rc_w0) are cleared only by software, and a transfer
//! abandoned mid-flight leaves the slave mid-transaction and the peripheral with START/STOP
//! pending, so without recovery ONE failed transfer latches every later transfer dead (the same
//! failure class as the UART ORE latch). Every failed transfer now runs [`I2c::recover`]: clear
//! the sticky error flags, STOP if still master, and if the peripheral stays wedged (I2CBSY
//! without MASTER, or START/STOP request stuck) pulse SRESET (CTL0 bit 15) and reprogram
//! timing/mode/enable. The error still surfaces to the caller (one lost sample); the peripheral
//! is usable again on the next call.
//!
//! The GD/ST register naming stays on this side of the trait boundary (SPEC.md): what crosses into
//! `embedded-hal` `i2c::I2c` is bytes, a 7-bit address, and an [`I2cError`].

use core::cell::Cell;

use embedded_hal::i2c::{self, Operation, SevenBitAddress};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::error::{DescriptorError, I2cError};
use crate::gpio::{configure_af, Pin, PinRole};
use crate::reg::Reg32;

// --- register offsets (identical on both families) --------------------------------------------

const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
const SADDR0: u32 = 0x08;
const DATA: u32 = 0x10;
const STAT0: u32 = 0x14;
const STAT1: u32 = 0x18;
const CKCFG: u32 = 0x1C;
const RT: u32 = 0x20;

// CTL0 bits.
const CTL0_I2CEN: u32 = 1 << 0;
const CTL0_SMBEN: u32 = 1 << 1;
const CTL0_START: u32 = 1 << 8;
const CTL0_STOP: u32 = 1 << 9;
const CTL0_ACKEN: u32 = 1 << 10;
/// POAP: with it set, ACKEN governs the **next** byte to be received, not the current one (the
/// manual's Solution B N=2 case; set before START, cleared after the transfer).
const CTL0_POAP: u32 = 1 << 11;
/// SRESET: software reset of the I2C block (the recovery path's escape for a wedged peripheral).
const CTL0_SRESET: u32 = 1 << 15;

// CTL1 / CKCFG / RT fields.
const CTL1_I2CCLK: u32 = 0x7F; // BITS(0,6)
const CKCFG_CLKC: u32 = 0xFFF; // BITS(0,11)
const CKCFG_DTCY: u32 = 1 << 14;
const CKCFG_FAST: u32 = 1 << 15;

// SADDR0 / address format. 7-bit address format is the 0 value of the ADDFORMAT field; the SPL
// masks the address with I2C_ADDRESS_MASK (BITS(1,9)) and ORs the format in. The 7-bit own address
// is held left-shifted by one in bits [7:1], the same field the master sends.
const I2C_ADDRESS_MASK: u32 = 0x03FF; // BITS(0,9): masks the address bits the SPL writes

// STAT0 flags.
const STAT0_SBSEND: u32 = 1 << 0;
const STAT0_ADDSEND: u32 = 1 << 1;
const STAT0_BTC: u32 = 1 << 2;
const STAT0_RBNE: u32 = 1 << 6;
const STAT0_TBE: u32 = 1 << 7;
const STAT0_BERR: u32 = 1 << 8;
const STAT0_LOSTARB: u32 = 1 << 9;
const STAT0_AERR: u32 = 1 << 10;
const STAT0_OUERR: u32 = 1 << 11;
/// The sticky (rc_w0, software-cleared) STAT0 error flags the recovery path clears.
const STAT0_ERRORS: u32 = STAT0_BERR | STAT0_LOSTARB | STAT0_AERR | STAT0_OUERR;

// STAT1 bits.
/// MASTER: the block is in master mode.
const STAT1_MASTER: u32 = 1 << 0;
/// I2CBSY: the bus is busy (a transfer in progress, or a line held low).
const STAT1_I2CBSY: u32 = 1 << 1;

/// SPL `I2CCLK_MAX` / `I2CCLK_MIN` clamp bounds for the peripheral-clock MHz field.
const I2CCLK_MAX: u32 = 0x48;
const I2CCLK_MIN: u32 = 0x02;

// --- the polled-transfer time budget ----------------------------------------------------------
//
// The budget is scoped to a whole BUS ACQUISITION, not to one flag wait, and it is derived from
// the core clock so it means something in microseconds. Both of those are deliberate.
//
// Why microseconds at all: the consumer is a balance control task whose torque word is watched by
// a commutation ISR that floats all three motor phases if the task goes quiet too long. A polled
// transfer that spins past that window cuts motor torque on a machine trying to stay upright, so
// the driver's failure cost is a real-time property, not a stylistic one. A budget counted in raw
// loop iterations is clock-independent in CYCLES but not in TIME, and time is what the coast is
// measured in: the previous fixed 100_000-iteration budget worked out at 22 ms at 72 MHz, already
// past a 16 ms coast, while being only ~2 ms at 8 MHz. Deriving the iteration count from the core
// clock makes the wall-clock bound the constant and the iteration count the derived quantity,
// which is the way round the caller can actually reason about.
//
// Why per acquisition rather than per wait: a per-wait budget cannot bound a transfer. It detects
// "no progress", but a slave that clock-stretches each byte to just inside a per-wait limit makes
// PROGRESS on every byte while the transfer as a whole runs arbitrarily long, and a 14-byte burst
// has 16 waits in it. One countdown shared by every wait in the transaction bounds the total by
// construction, with no case analysis about which waits can stack.
//
// Why not a DWT cycle-counter deadline, which would be exact: `wait_flag` would then depend on
// DWT CYCCNT being enabled, and a caller that had not enabled it reads a frozen counter and never
// times out. A safety bound whose failure mode is "spins forever, silently" is worse than one
// whose failure mode is a documented conversion constant. The iteration countdown always runs.

/// Core cycles one [`I2c::wait_flag`] poll iteration costs, as an UPPER bound.
///
/// This is the divisor that turns a microsecond target into an iteration count, so the UPPER bound
/// is the correct end of the range to take: over-stating it under-states the count and the real
/// block finishes EARLY, while under-stating it would let the real block overrun the target.
///
/// Measured, not guessed. Disassembled from the consuming firmware's release image, the loop is an
/// APB load of `STAT0`, five shift/branch flag tests, a load and store of the budget through
/// memory, a zero test and a taken back-branch: 21 core cycles on Cortex-M3 before any bus latency.
/// On top of that sit the AHB-to-APB1 bridge stall on the `STAT0` load (I2C is on APB1, half the
/// core clock on the reference tree) and flash wait states on the loop fetch, neither of which is
/// exactly countable from the listing, hence a bound rather than a figure.
///
/// The safety argument does NOT rest on 32 being right. What has to hold is that the real block
/// stays inside the consumer's coast, and with the budget this sizes that survives a real loop of
/// up to roughly 39 cycles, 1.85x the disassembled core cost. So the bound would have to be wrong
/// by most of a factor of two before the guarantee it underwrites is affected.
pub const POLL_CYCLES: u32 = 32;

/// Wall-clock ceiling, in microseconds, on the polling one bus acquisition may do.
///
/// Sized between the two things that squeeze it. Below: a healthy 14-byte IMU burst is ~0.4 ms at
/// 400 kHz and ~1.7 ms at 100 kHz, so 6 ms leaves roughly 15x and 3.5x of headroom against
/// false-timing a working bus. Above: [`worst_case_block_us`] must stay well inside the consumer's
/// coast, and at 6 ms it comes out at 6.6 ms against a 16 ms window. Those two pincers are why it
/// is not simply set lower: a smaller number would fail healthy 100 kHz transfers.
///
/// This is a bound for SHORT sensor transactions, which is all this driver has callers for (a
/// 14-byte burst and a 1-byte `WHO_AM_I`). A caller wanting a long transfer on a slow bus (a 64-byte
/// read at 100 kHz is already 5.8 ms of legitimate wire time) has to split it across transactions
/// rather than raise this number, because raising it spends the motor coast.
pub const TXN_BUDGET_US: u32 = 6_000;

/// Poll iterations that fit in [`TXN_BUDGET_US`] at `core_hz`: the per-acquisition budget.
///
/// Saturates at 1 so a nonsensically slow `core_hz` still terminates.
pub const fn poll_budget(core_hz: u32) -> u32 {
    let n = (core_hz / 1_000_000) * TXN_BUDGET_US / POLL_CYCLES;
    if n == 0 {
        1
    } else {
        n
    }
}

/// The bus-idle sub-cap: a tenth of [`poll_budget`]. A healthy bus goes idle within a couple of
/// byte times of the previous STOP, so a fresh transfer that finds the line still held should give
/// up quickly and recover rather than spend the whole acquisition budget before it has even
/// started. Also the bound on [`I2c::recover`]'s own STOP wait, which runs AFTER the acquisition
/// budget may already be exhausted and so cannot draw from it.
pub const fn idle_budget(core_hz: u32) -> u32 {
    let n = poll_budget(core_hz) / 10;
    if n == 0 {
        1
    } else {
        n
    }
}

/// Worst-case microseconds a single failing bus acquisition can block the caller.
///
/// The acquisition budget, plus the recovery that runs on the way out (recovery has its own budget
/// because it must work after the acquisition budget is spent). Const so a caller with a real-time
/// deadline can `const_assert` its own coast against it and fail the BUILD if either moves.
pub const fn worst_case_block_us(core_hz: u32) -> u32 {
    let polls = poll_budget(core_hz) + idle_budget(core_hz);
    // A sub-MHz core divides to zero: report the bound as unbounded rather than dividing by it.
    match (polls * POLL_CYCLES).checked_div(core_hz / 1_000_000) {
        Some(us) => us,
        None => u32::MAX,
    }
}

/// Fast-mode duty cycle (open item I2C-1). `Two` (Tlow/Thigh = 2, DTCY = 0) is the default the IMU
/// reference and the bench probe use; `SixteenNine` (16/9, DTCY = 1) is expressible for callers
/// that need it. Ignored in standard mode (<= 100 kHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FastDuty {
    /// Tlow/Thigh = 2 (DTCY = 0), the SPL `I2C_DTCY_2` default.
    #[default]
    Two,
    /// Tlow/Thigh = 16/9 (DTCY = 1), the SPL `I2C_DTCY_16_9`.
    SixteenNine,
}

/// The I2C bus mode: the target SCL frequency plus, for fast mode, the duty cycle.
///
/// This repackages the old `I2cConfig`'s `speed_hz` + `duty` into the small mode value [`I2c::new`]
/// takes (the stm32f1xx-hal `Mode` analogue). [`I2cMode::standard`] is for <= 100 kHz (the duty is
/// ignored there, so it is not even named); [`I2cMode::fast`] names the fast-mode duty explicitly.
/// The two values flow straight into [`timing_for`] (the GD SPL `i2c_clock_config` math).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cMode {
    /// Target SCL frequency in Hz (100_000 standard, 400_000 fast; any SPL-valid rate).
    pub speed_hz: u32,
    /// Fast-mode duty cycle. Ignored by [`timing_for`] in standard mode (<= 100 kHz).
    pub duty: FastDuty,
}

impl I2cMode {
    /// Standard-mode bus at `speed_hz` (<= 100 kHz). The fast-mode duty is irrelevant here, so it
    /// defaults to `FastDuty::Two` and is never consulted by [`timing_for`].
    #[inline]
    pub const fn standard(speed_hz: u32) -> Self {
        Self {
            speed_hz,
            duty: FastDuty::Two,
        }
    }

    /// Fast-mode bus at `speed_hz` (100 kHz < speed <= 400 kHz) with the named duty cycle.
    #[inline]
    pub const fn fast(speed_hz: u32, duty: FastDuty) -> Self {
        Self { speed_hz, duty }
    }
}

/// The computed I2C timing: the three register values the SPL `i2c_clock_config` programs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cTiming {
    /// `CTL1` `I2CCLK[6:0]` field (the APB1 clock in MHz, clamped).
    pub i2cclk: u32,
    /// `CKCFG` value (CLKC plus FAST/DTCY for fast mode).
    pub ckcfg: u32,
    /// `RT` `RISETIME[6:0]` field.
    pub rt: u32,
}

/// Compute the `I2CCLK` / `CKCFG` / `RT` values for a given APB1 bus clock and target speed,
/// byte-for-byte as the GD SPL `i2c_clock_config(i2c, clkspeed, dutycyc)` does.
///
/// `pclk1_hz` is the APB1 clock (the I2C peripheral clock; I2C is on APB1 on both families).
/// `speed_hz` is the target SCL frequency. `duty` selects the fast-mode duty cycle (ignored in
/// standard mode). A speed above 400 kHz is out of the SPL's range and is clamped to fast-mode
/// behaviour at 400 kHz by the caller's contract; here it falls into the fast branch.
pub fn timing_for(pclk1_hz: u32, speed_hz: u32, duty: FastDuty) -> I2cTiming {
    // I2CCLK = pclk1 in MHz, clamped to I2CCLK_MAX (the SPL clamps the high side only here).
    let freq = (pclk1_hz / 1_000_000).min(I2CCLK_MAX);
    let i2cclk = freq;

    if speed_hz <= 100_000 {
        // Standard mode. RT = pclk1_MHz + 1, clamped to [MIN, MAX] (the SPL's if/elseif/else clamp).
        let risetime = (pclk1_hz / 1_000_000) + 1;
        let rt = risetime.clamp(I2CCLK_MIN, I2CCLK_MAX);
        // CLKC = pclk1 / (speed*2), min 4 in standard mode. CKCFG = CLKC (FAST=0, DTCY=0).
        let mut clkc = pclk1_hz / (speed_hz * 2);
        if clkc < 0x04 {
            clkc = 0x04;
        }
        I2cTiming {
            i2cclk,
            ckcfg: clkc & CKCFG_CLKC,
            rt,
        }
    } else {
        // Fast mode. RT = (I2CCLK*300)/1000 + 1.
        let rt = (freq * 300) / 1000 + 1;
        let (mut clkc, dtcy_bit) = match duty {
            FastDuty::Two => (pclk1_hz / (speed_hz * 3), 0),
            FastDuty::SixteenNine => (pclk1_hz / (speed_hz * 25), CKCFG_DTCY),
        };
        // The SPL forces the CLKC field to at least 1 in fast mode.
        if clkc & CKCFG_CLKC == 0 {
            clkc |= 0x0001;
        }
        I2cTiming {
            i2cclk,
            ckcfg: CKCFG_FAST | dtcy_bit | clkc,
            rt,
        }
    }
}

/// Derive the I2C peripheral (APB1) clock in Hz from a [`ClockConfig`]. I2C lives on APB1 on both
/// families, so this is `AHB / apb1_psc = (sysclk / ahb_psc) / apb1_psc`, the same chain the SPL
/// `rcu_clock_freq_get(CK_APB1)` walks from the prescaler bits and that [`crate::usart`] uses for
/// the USART APB1 input clock.
#[inline]
pub fn i2c_input_clock(clock: &ClockConfig) -> u32 {
    let ahb = clock.sysclk_hz / clock.ahb_psc.max(1) as u32;
    ahb / clock.apb1_psc.max(1) as u32
}

// --- the handle -------------------------------------------------------------------------------

/// A configured I2C master, resolved once at bring-up: the base (the register model is shared,
/// so there is no per-family field) plus the computed timing, kept so the recovery path's SRESET
/// can reprogram the block without re-deriving the clock tree. The polled transfer primitives and
/// the `embedded-hal` `i2c::I2c` impl hang off this (DECISIONS.md #4: resolve once into a
/// concrete handle).
///
/// NOT `Copy`/`Clone`: the handle now carries the live poll budget of the acquisition in flight, so
/// a duplicate would be a second handle to the same peripheral with its own idea of how much of the
/// control loop's time budget had been spent. It was only ever `Copy` because nothing had made it
/// stateful yet; every caller already moves it or borrows it mutably.
#[derive(Debug)]
pub struct I2c {
    base: u32,
    /// The bring-up timing (CTL1 I2CCLK / CKCFG / RT), reused by [`I2c::recover`]'s SRESET reinit.
    timing: I2cTiming,
    /// Poll iterations one bus acquisition may spend ([`poll_budget`] at the bring-up core clock),
    /// and a tenth of it for the bus-idle wait and for recovery ([`idle_budget`]). Resolved once at
    /// bring-up because the clock is known there and nowhere else in this module.
    poll_budget: u32,
    idle_budget: u32,
    /// Budget left in the CURRENT bus acquisition, shared by every wait in it. Reset by the
    /// transfer that acquires the bus (`fresh`); a repeated-START continuation keeps counting down
    /// the same one, so a whole `write_read` is bounded, not each of its two halves separately.
    txn_left: Cell<u32>,
}

/// The master own-address value programmed into `SADDR0` (the bench probe's `0x24`). As a
/// single-master bus controller the own address is rarely matched; the SPL programs it, so we keep
/// the bench-validated value rather than leave it unset (the bring-up byte-for-byte agreement with
/// the SPL only depends on writing it directly the SPL way).
const MASTER_OWN_ADDR: u8 = 0x24;

/// The maximum SCL pulses a bus-clear issues: 9 (a full stuck byte's 8 data clocks + the ACK clock),
/// the standard I2C recovery clock count (NXP UM10204 / AN10216 §3.1.7 "bus clear").
const BUS_CLEAR_MAX_PULSES: u32 = 9;

/// Pure I2C bus-clear sequencer, driven by pin closures so the decision logic (and the pulse bound)
/// is host-testable without silicon.
///
/// A slave that was mid-READ when the master reset (a warm MCU reset, NOT a POR) can be left holding
/// SDA low, wedging the bus so the next master bring-up's `probe` bus-errors (the round-10 warm-reset
/// hang; POR-cleared only). This clocks the slave out: if SDA is already high the bus is clean and NO
/// pulses are issued (only-if-needed); otherwise it pulses SCL (drive low, release high) up to
/// [`BUS_CLEAR_MAX_PULSES`] times, stopping as SOON as the slave releases SDA, then generates a STOP
/// (SDA low then released high while SCL is high) to leave the bus idle for the peripheral. Returns the
/// number of SCL pulses issued (0 on a clean bus, at most 9), so the pulse bound is host-assertable.
///
/// The closures ARE the HAL gpio surface bound by the hardware caller ([`set_pin`](crate::gpio) /
/// [`read_pin`](crate::gpio)); this function pokes no registers itself (SPEC: no ad-hoc register access
/// from the I2C module, route through the HAL gpio surface). `drive_*(true)` RELEASES the open-drain
/// line (high-Z, pulled up); `drive_*(false)` drives it low.
#[cfg_attr(not(target_arch = "arm"), allow(dead_code))]
fn bus_clear_seq(
    mut read_sda: impl FnMut() -> bool,
    mut drive_scl: impl FnMut(bool),
    mut drive_sda: impl FnMut(bool),
    mut settle: impl FnMut(),
) -> u32 {
    // Start with both lines released (high-Z, pulled up) and let them settle.
    drive_scl(true);
    drive_sda(true);
    settle();

    // Only-if-needed: a clean bus (SDA reads high) needs no recovery at all.
    if read_sda() {
        return 0;
    }

    // Clock the stuck slave out: SCL pulses (low then release-high), stopping when SDA releases.
    let mut pulses = 0;
    while pulses < BUS_CLEAR_MAX_PULSES && !read_sda() {
        drive_scl(false);
        settle();
        drive_scl(true);
        settle();
        pulses += 1;
    }

    // STOP: with SCL released high, pull SDA low then release it high. A low->high SDA edge while SCL
    // is high is a STOP condition, which returns any confused slave to its idle state.
    drive_scl(true);
    settle();
    drive_sda(false);
    settle();
    drive_sda(true);
    settle();

    pulses
}

impl I2c {
    /// Bring up the I2C master, CONSUMING the SCL/SDA [`Pin`] handles from `split()`.
    ///
    /// This is the headline I2C API, the stm32f1xx-hal `I2c::new(I2C1, (scl, sda), mode, &mut rcc)`
    /// analogue: the application passes the named pins it got from `chip.gpiob().split()` (e.g.
    /// `gpiob.pb6` / `gpiob.pb7`), never a packed `(port << 4) | pin` byte, and never sees the
    /// [`crate::descriptor::GpioPath`] register model. Generic over the pins' current mode markers
    /// `S` / `D` (they come from `split()` in their reset [`crate::gpio::Input`] state); `new`
    /// reconfigures them and so takes them by value.
    ///
    /// Steps, in order:
    /// 1. Resolve the I2C `instance` to its base from the chip's address table.
    /// 2. Enable the I2C peripheral clock (`enable_i2c`). The SCL/SDA **GPIO port** clock was already
    ///    enabled when the chip handed back the [`crate::gpio::GpioPort`] the pins were split from
    ///    (the `split(&mut rcc)` clock-enable lives in `chip.gpiob()`), so it is not re-enabled here.
    /// 3. Configure both pins as AF open-drain with pull-up ([`PinRole::I2cAfOpenDrain`]) via
    ///    `configure_af`, which owns the F10x/F1x0 register-model branch internally (PB6/PB7 at
    ///    AF1 on F1x0, AF-OD nibble on F10x: the bench-validated mux).
    /// 4. Program the timing from `clock` + `mode` and enable the peripheral + ACK (the SPL
    ///    `i2c_clock_config` -> `i2c_mode_addr_config` -> `i2c_enable` -> `i2c_ack_config` sequence).
    ///
    /// The returned [`I2c`] implements [`embedded_hal::i2c::I2c`], so an IMU driver generic over that
    /// trait drives it directly.
    pub fn new<S, D>(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        pins: (Pin<S>, Pin<D>),
        mode: I2cMode,
    ) -> Result<I2c, DescriptorError> {
        let (scl, sda) = pins;

        // 1. The per-configuration pin-ownership backstop (Layer 7), FIRST: this constructor
        //    configures both pins itself, and its bus-clear step drives them as open-drain
        //    outputs before the peripheral claims them, so the refusal has to precede everything
        //    or a refused pin would still have been driven.
        crate::pincap::check_pin_ownership(chip, scl.pin(), crate::pincap::PinClaim::NonGate)?;
        crate::pincap::check_pin_ownership(chip, sda.pin(), crate::pincap::PinClaim::NonGate)?;

        let base = chip.base(instance)?;

        // 2. Enable the I2C peripheral clock (the GPIO-port clock was enabled by `chip.gpiob()`).
        crate::clock::enable_i2c(chip.rcu_base()?, chip.clock(), instance)?;

        // 2a. Warm-reset bus-clear (round-10 warm-reset I2C hang): a warm MCU reset mid-transaction can
        //     leave a slave holding SDA low, wedging the bus so the NEXT bring-up's `probe` bus-errors
        //     (imu_configured=0; POR-cleared only). BEFORE the peripheral claims the pins, clock the
        //     stuck slave out and STOP, as a GPIO-level bit-bang through the HAL's own gpio surface
        //     (open-drain drive + level read), NOT a raw-I2C probe. A clean bus (SDA already high) is a
        //     no-op (only-if-needed), so this is harmless on every boot and on both families.
        Self::bus_clear_before_claim(&scl, &sda, clock);

        // 3. SCL/SDA as AF open-drain with pull-up. Take the resolved port base + register-model path
        //    + logical pin byte straight from each consumed Pin; the application never built them.
        configure_af(
            scl.port_base(),
            scl.path(),
            scl.pin(),
            PinRole::I2cAfOpenDrain,
        );
        configure_af(
            sda.port_base(),
            sda.path(),
            sda.pin(),
            PinRole::I2cAfOpenDrain,
        );

        // 4. Timing + mode + enable + ACK.
        let pclk1 = i2c_input_clock(clock);
        let timing = timing_for(pclk1, mode.speed_hz, mode.duty);
        let dev = I2c {
            base,
            timing,
            poll_budget: poll_budget(clock.sysclk_hz),
            idle_budget: idle_budget(clock.sysclk_hz),
            txn_left: Cell::new(0),
        };
        dev.configure_timing(timing);
        dev.configure_mode_addr(MASTER_OWN_ADDR);
        // i2c_enable: set I2CEN.
        dev.ctl0().modify(CTL0_I2CEN, CTL0_I2CEN);
        // i2c_ack_config(ENABLE): set ACKEN.
        dev.set_ack(true);
        Ok(dev)
    }

    /// Run the warm-reset bus-clear on the SCL/SDA pins BEFORE the peripheral claims them (hardware).
    ///
    /// Configures both pins as general-purpose OPEN-DRAIN outputs through the HAL gpio surface, then
    /// runs [`bus_clear_seq`] with closures bound to `set_pin` / `read_pin` and a ~100 kHz busy-wait
    /// settle (half-period = `sysclk / 200_000` cycles). No raw register access lives here: the drive
    /// and level-read go through the gpio module. A clean bus issues zero pulses. The pins are left as
    /// open-drain outputs; the caller's [`configure_af`] immediately re-claims them as AF open-drain.
    #[cfg(target_arch = "arm")]
    fn bus_clear_before_claim<S, D>(scl: &Pin<S>, sda: &Pin<D>, clock: &ClockConfig) {
        let (scl_base, path, scl_pin) = (scl.port_base(), scl.path(), scl.pin());
        let (sda_base, sda_pin) = (sda.port_base(), sda.pin());
        // Both bus pins as open-drain outputs with the internal pull-up (F1x0; F10x relies on the
        // board's external I2C pull-up). Idle level is high (released) once driven high below.
        crate::gpio::configure_output_od(scl_base, path, scl_pin, true);
        crate::gpio::configure_output_od(sda_base, path, sda_pin, true);
        // ~100 kHz half-period in CPU cycles for the busy-wait settle between edges.
        let half = (clock.sysclk_hz / 200_000).max(1);
        bus_clear_seq(
            || crate::gpio::read_pin(sda_base, path, sda_pin),
            |high| crate::gpio::set_pin(scl_base, path, scl_pin, high),
            |high| crate::gpio::set_pin(sda_base, path, sda_pin, high),
            || cortex_m::asm::delay(half),
        );
    }

    /// Host stub for [`bus_clear_before_claim`]: the bit-bang needs real silicon + timing, so on the
    /// host it is a no-op (the `new` host tests still exercise the AF-config path). The sequencer
    /// logic is host-tested directly via [`bus_clear_seq`].
    #[cfg(not(target_arch = "arm"))]
    fn bus_clear_before_claim<S, D>(_scl: &Pin<S>, _sda: &Pin<D>, _clock: &ClockConfig) {}

    /// Program CTL1 I2CCLK, RT, then CKCFG, exactly as `i2c_clock_config` does (CTL1 via a
    /// clear-then-set of the I2CCLK field; RT and CKCFG are written directly, CKCFG from its reset
    /// 0 so the SPL's `|=` ORs and a single set write reach the same end state).
    fn configure_timing(&self, t: I2cTiming) {
        // CTL1: clear the I2CCLK field, set the computed MHz value (SPL: temp &= ~I2CCLK; temp |= freq).
        self.ctl1().modify(CTL1_I2CCLK, t.i2cclk);
        // RT: the SPL assigns it directly (= risetime), not RMW.
        self.rt().write(t.rt);
        // CKCFG: the SPL ORs CLKC/FAST/DTCY into a reset-0 register; the net value is t.ckcfg.
        self.ckcfg().write(t.ckcfg);
    }

    /// Select I2C mode (clear SMBEN) and program the own-address register, exactly as
    /// `i2c_mode_addr_config(i2c, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, own_addr)` does.
    ///
    /// `I2C_I2CMODE_ENABLE` is 0 and `I2C_ADDFORMAT_7BITS` is 0, so this clears SMBEN in CTL0 and
    /// writes `own_addr & I2C_ADDRESS_MASK` to SADDR0. NOTE: the SPL's `addr` argument is the **raw
    /// SADDR0 value** (already positioned), NOT a 7-bit address to shift, so `own_addr` is written
    /// directly the SPL way; the bench probe passes `0x24` and it lands as `0x24`. As a
    /// single-master bus controller the own address is rarely matched, so the exact value only has
    /// to agree with the SPL byte-for-byte, which writing it directly guarantees.
    fn configure_mode_addr(&self, own_addr: u8) {
        // CTL0: clear SMBEN (select I2C, not SMBus); mode value is 0 so nothing is OR'd in.
        self.ctl0().modify(CTL0_SMBEN, 0);
        // SADDR0: 7-bit format (0) | the own-address register value, masked the SPL way.
        self.saddr0().write((own_addr as u32) & I2C_ADDRESS_MASK);
    }

    /// The underlying base address (for code that needs the register-level view).
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    // --- register accessors -------------------------------------------------------------------

    #[inline]
    fn ctl0(&self) -> Reg32 {
        Reg32::new(self.base, CTL0)
    }
    #[inline]
    fn ctl1(&self) -> Reg32 {
        Reg32::new(self.base, CTL1)
    }
    #[inline]
    fn saddr0(&self) -> Reg32 {
        Reg32::new(self.base, SADDR0)
    }
    #[inline]
    fn data(&self) -> Reg32 {
        Reg32::new(self.base, DATA)
    }
    #[inline]
    fn stat0(&self) -> Reg32 {
        Reg32::new(self.base, STAT0)
    }
    #[inline]
    fn stat1(&self) -> Reg32 {
        Reg32::new(self.base, STAT1)
    }
    #[inline]
    fn ckcfg(&self) -> Reg32 {
        Reg32::new(self.base, CKCFG)
    }
    #[inline]
    fn rt(&self) -> Reg32 {
        Reg32::new(self.base, RT)
    }

    // --- low-level event primitives (the SPL `i2c_*` calls, GD-named) -------------------------

    /// `i2c_start_on_bus`: set CTL0 START.
    #[inline]
    fn start_on_bus(&self) {
        self.ctl0().modify(CTL0_START, CTL0_START);
    }

    /// `i2c_stop_on_bus`: set CTL0 STOP.
    #[inline]
    fn stop_on_bus(&self) {
        self.ctl0().modify(CTL0_STOP, CTL0_STOP);
    }

    /// `i2c_ack_config`: set or clear CTL0 ACKEN.
    #[inline]
    fn set_ack(&self, enable: bool) {
        self.ctl0()
            .modify(CTL0_ACKEN, if enable { CTL0_ACKEN } else { 0 });
    }

    /// `i2c_ackpos_config`: set or clear CTL0 POAP (Solution B's N=2 ACK repositioning).
    #[inline]
    fn set_poap(&self, next: bool) {
        self.ctl0()
            .modify(CTL0_POAP, if next { CTL0_POAP } else { 0 });
    }

    /// `i2c_master_addressing`: write the address byte to DATA. `read` selects the R/W bit:
    /// transmitter clears bit 0, receiver sets it (matching the SPL's `& I2C_TRANSMITTER` /
    /// `| I2C_RECEIVER`). `addr7` is the 7-bit address; the byte sent is `(addr7 << 1) | rw`.
    #[inline]
    fn send_address(&self, addr7: u8, read: bool) {
        let byte = ((addr7 as u32) << 1) | (read as u32);
        self.data().write(byte);
    }

    /// `i2c_data_transmit`: write a data byte to DATA.
    #[inline]
    fn transmit(&self, byte: u8) {
        self.data().write(byte as u32);
    }

    /// `i2c_data_receive`: read a data byte from DATA.
    #[inline]
    fn receive(&self) -> u8 {
        (self.data().read() & 0xFF) as u8
    }

    /// Clear ADDSEND the block's way: read STAT0 then STAT1 (the read pair clears the flag).
    #[inline]
    fn clear_addsend(&self) {
        let _ = self.stat0().read();
        let _ = self.stat1().read();
    }

    /// Poll STAT0 until `flag` is set, mapping a concurrently-set error flag (BERR/LOSTARB/AERR) to
    /// the corresponding [`I2cError`], and a budget exhaustion to [`I2cError::Timeout`]. `nack_src`
    /// names whether an AERR seen here is an address NACK or a data NACK (open item I2C-2). Returns
    /// the STAT0 snapshot that satisfied `flag` on success.
    fn wait_flag(&self, flag: u32, nack_src: NackKind) -> Result<u32, I2cError> {
        loop {
            let s = self.stat0().read();
            if s & flag != 0 {
                return Ok(s);
            }
            // Surface a bus/arbitration/NACK error if it appears while waiting (the probe checks
            // these in the same loop). BERR/LOSTARB first (bus-level), then AERR (NACK).
            if s & STAT0_BERR != 0 {
                return Err(I2cError::Bus);
            }
            if s & STAT0_LOSTARB != 0 {
                return Err(I2cError::ArbitrationLoss);
            }
            if s & STAT0_AERR != 0 {
                return Err(match nack_src {
                    NackKind::Address => I2cError::NoAcknowledgeAddress,
                    NackKind::Data => I2cError::NoAcknowledgeData,
                });
            }
            if s & STAT0_OUERR != 0 {
                return Err(I2cError::Overrun);
            }
            if !self.spend() {
                return Err(I2cError::Timeout);
            }
        }
    }

    /// Charge one poll iteration to the current acquisition's budget. `false` once it is spent.
    #[inline]
    fn spend(&self) -> bool {
        let left = self.txn_left.get();
        if left == 0 {
            return false;
        }
        self.txn_left.set(left - 1);
        true
    }

    /// Start a fresh bus acquisition's budget. Called by the transfer that ACQUIRES the bus; a
    /// repeated-START continuation deliberately does not call it.
    #[inline]
    fn begin_txn(&self) {
        self.txn_left.set(self.poll_budget);
    }

    // --- error recovery (the persistence half of the 2026-07-18 silicon finding) --------------

    /// Bounded wait for the bus to go idle (I2CBSY clear) before a FRESH transfer's START. On
    /// exhaustion, run [`I2c::recover`] once (an SRESET un-wedges the observed
    /// START/STOP-pending deadlock) and re-check; a bus still busy after that is a genuinely
    /// held line, surfaced as [`I2cError::Bus`].
    fn wait_bus_idle(&self) -> Result<(), I2cError> {
        let mut budget = self.idle_budget;
        loop {
            if self.stat1().read() & STAT1_I2CBSY == 0 {
                return Ok(());
            }
            budget -= 1;
            // The sub-cap AND the acquisition budget: whichever runs out first ends the wait, so
            // this wait can never spend more than a tenth of the acquisition on a held line.
            if budget == 0 || !self.spend() {
                self.recover();
                if self.stat1().read() & STAT1_I2CBSY == 0 {
                    return Ok(());
                }
                return Err(I2cError::Bus);
            }
        }
    }

    /// Put the peripheral back into a usable state after a failed transfer. Without this, ONE
    /// failure latches the block dead (the silicon-observed wedge: sticky error flags, a
    /// never-sent STOP pending, I2CBSY stuck with MASTER dropped). Steps:
    ///
    /// 1. Clear the sticky rc_w0 error flags (the SPL `i2c_flag_clear` shape: RMW writing the
    ///    flag bits low, leaving the read-only bits untouched).
    /// 2. If still master, release the bus with a STOP (bounded wait for the request to clear).
    /// 3. If the peripheral stays wedged (I2CBSY without MASTER, or a START/STOP request stuck
    ///    pending), pulse SRESET and reprogram timing/mode/enable from the stored bring-up state.
    /// 4. Restore the next-transfer posture (POAP clear, ACKEN set).
    fn recover(&self) {
        // 1. Sticky error flags (rc_w0: cleared by writing 0 to the flag bit).
        let s = self.stat0().read();
        self.stat0().write(s & !STAT0_ERRORS);

        // 2. Release the bus if this side still drives it.
        if self.stat1().read() & STAT1_MASTER != 0 {
            self.stop_on_bus();
            // Recovery's OWN budget, not the acquisition's: recovery runs precisely when that one
            // is spent, so drawing from it would make recovery a no-op exactly when it is needed.
            let mut budget = self.idle_budget;
            while self.ctl0().read() & CTL0_STOP != 0 {
                budget -= 1;
                if budget == 0 {
                    break;
                }
            }
        }

        // 3. A wedged block: busy without mastering the bus (the observed MASTER-dropped
        //    deadlock), or a START/STOP request that never completed.
        let busy_not_master = self.stat1().read() & (STAT1_I2CBSY | STAT1_MASTER) == STAT1_I2CBSY;
        let request_stuck = self.ctl0().read() & (CTL0_START | CTL0_STOP) != 0;
        if busy_not_master || request_stuck {
            self.soft_reset_reinit();
        }

        // 4. Next-transfer posture.
        self.set_poap(false);
        self.set_ack(true);
    }

    /// Pulse CTL0 SRESET (hold, release) and reprogram the block from the stored bring-up state
    /// (SRESET clears the configuration registers). The recovery path's escape for a wedged
    /// peripheral; mirrors the bring-up order timing -> mode/address -> I2CEN -> ACKEN.
    fn soft_reset_reinit(&self) {
        self.ctl0().write(CTL0_SRESET);
        self.ctl0().write(0);
        self.configure_timing(self.timing);
        self.configure_mode_addr(MASTER_OWN_ADDR);
        self.ctl0().modify(CTL0_I2CEN, CTL0_I2CEN);
        self.set_ack(true);
    }

    // --- polled master transfers (transmit event-based; receive per the manual's Solution B) --

    /// Polled master write: START -> address+W -> ADDSEND (clear) -> each byte via TBE then a final
    /// BTC -> optional STOP. If `stop` is false the transfer is left without a STOP so a repeated
    /// START can follow (the register-pointer phase of a read). `fresh` marks a transfer that
    /// starts on a bus this side does not already hold: it first waits (bounded) for I2CBSY to
    /// clear, so a wedged or still-finishing bus is detected up front instead of corrupting the
    /// new transfer. A failed transfer runs [`I2c::recover`] before the error is returned.
    ///
    /// An empty `bytes` still does the START + address handshake (an `embedded-hal` zero-length
    /// write is a bus presence check); the trailing BTC wait is skipped when no byte was sent.
    pub fn write_bytes(
        &self,
        addr7: u8,
        bytes: &[u8],
        fresh: bool,
        stop: bool,
    ) -> Result<(), I2cError> {
        if fresh {
            self.begin_txn();
            self.wait_bus_idle()?; // recovers internally on exhaustion
        }
        let r = self.write_inner(addr7, bytes, stop);
        if r.is_err() {
            self.recover();
        }
        r
    }

    fn write_inner(&self, addr7: u8, bytes: &[u8], stop: bool) -> Result<(), I2cError> {
        self.set_ack(true);
        self.start_on_bus();
        self.wait_flag(STAT0_SBSEND, NackKind::Address)?;
        self.send_address(addr7, false);
        // Wait for ADDSEND (address ACKed); an AERR here is an ADDRESS NACK.
        self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
        self.clear_addsend();

        for (i, &b) in bytes.iter().enumerate() {
            // TBE: the data register can take the next byte. An AERR while sending data is a DATA NACK.
            self.wait_flag(STAT0_TBE, NackKind::Data)?;
            self.transmit(b);
            // After the LAST byte, wait for BTC (byte transfer complete) so the shift register has
            // drained before a STOP or repeated START.
            if i == bytes.len() - 1 {
                self.wait_flag(STAT0_BTC, NackKind::Data)?;
            }
        }

        if stop {
            self.stop_on_bus();
        }
        Ok(())
    }

    /// Polled master read per the manual's **Solution B** (see the module docs: delay-tolerant,
    /// every NACK/STOP decision under an SCL stretch, immune to ISR latency; Solution A's
    /// quick-response requirement is what wedged the integrated image on silicon). `fresh` marks
    /// a transfer that starts on a bus this side does not already hold (false for the repeated
    /// START after a register-pointer write); it first waits (bounded) for I2CBSY to clear. A
    /// failed transfer runs [`I2c::recover`] before the error is returned.
    pub fn read_bytes(&self, addr7: u8, buf: &mut [u8], fresh: bool) -> Result<(), I2cError> {
        if buf.is_empty() {
            return Ok(());
        }
        if fresh {
            self.begin_txn();
            self.wait_bus_idle()?; // recovers internally on exhaustion
        }
        let r = self.read_inner(addr7, buf);
        if r.is_err() {
            self.recover();
        }
        r
    }

    fn read_inner(&self, addr7: u8, buf: &mut [u8]) -> Result<(), I2cError> {
        // An empty buffer never reaches the bus. `read_bytes` already refuses one and is this
        // function's only caller, but without this the N==0 case would fall out of the match below
        // having issued a START and an address with ADDSEND uncleared and no STOP, leaving the
        // peripheral mid-transaction. The local code should be safe on its own terms rather than
        // on a guard in another function.
        if buf.is_empty() {
            return Ok(());
        }
        self.set_ack(true);
        // Solution B, N=2: POAP before START, so the ACKEN cleared below applies to the SECOND
        // byte (the one to NACK), not the first. A two-element pattern rather than a length
        // compare, for the same reason as the receive match below: the length never becomes a
        // number this function computes with.
        if let [_, _] = *buf {
            self.set_poap(true);
        }
        self.start_on_bus();
        self.wait_flag(STAT0_SBSEND, NackKind::Address)?;
        self.send_address(addr7, true);

        // The receive sequence differs by length, so the length cases are slice patterns and every
        // byte stored is a binding the pattern proves exists: nothing is indexed and nothing is
        // subtracted from the length, so the compiler has no bound left to check and the firmware's
        // 250 Hz IMU read carries no `slice_index_fail` (`specs/panic-free.md` in the firmware
        // repo, and its `tools/panic-reach.py` gate). The "at least three bytes" fact the general
        // arm needs comes from its own pattern test, not from the empty arm: that arm is here to
        // make the match exhaustive and nothing more (a `_` arm in its place leaves the bounds
        // check just as gone). The register writes per arm, and their order, are the silicon-proven
        // Solution B sequence and must not move.
        match buf {
            // Unreachable: refused above. Exhaustiveness only.
            [] => {}
            [only] => {
                // N=1: ACKEN clear BEFORE clearing ADDSEND; STOP after; read on RBNE.
                self.set_ack(false);
                self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
                self.clear_addsend();
                self.stop_on_bus();
                self.wait_flag(STAT0_RBNE, NackKind::Data)?;
                *only = self.receive();
            }
            [first, second] => {
                // N=2 (POAP set above): ACKEN clear before clearing ADDSEND (NACKs byte 2); then
                // both bytes arrive into DR + shift and SCL stretches (BTC); STOP; read twice.
                self.set_ack(false);
                self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
                self.clear_addsend();
                self.wait_flag(STAT0_BTC, NackKind::Data)?;
                self.stop_on_bus();
                *first = self.receive();
                self.wait_flag(STAT0_RBNE, NackKind::Data)?;
                *second = self.receive();
                self.set_poap(false);
            }
            [head @ .., third_last, second_last, last] => {
                // N>2: RBNE-read every byte ahead of the final three; the third-last stays in DR
                // until BTC signals the second-last in the shift register with SCL stretched;
                // ACKEN clear (the LAST byte will be NACKed); read the third-last (releases the
                // bus for the last byte); BTC again (second-last in DR, last in shift, stretched);
                // STOP; read the second-last; read the last.
                self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
                self.clear_addsend();
                for b in head.iter_mut() {
                    self.wait_flag(STAT0_RBNE, NackKind::Data)?;
                    *b = self.receive();
                }
                self.wait_flag(STAT0_BTC, NackKind::Data)?;
                self.set_ack(false);
                *third_last = self.receive();
                self.wait_flag(STAT0_BTC, NackKind::Data)?;
                self.stop_on_bus();
                *second_last = self.receive();
                self.wait_flag(STAT0_RBNE, NackKind::Data)?;
                *last = self.receive();
            }
        }
        // Re-enable ACK for the next transfer.
        self.set_ack(true);
        Ok(())
    }
}

/// Whether an acknowledge failure (AERR) seen during a poll is an address-phase or a data-phase
/// NACK (open item I2C-2: address NACK -> `NoAcknowledge(Address)`, data NACK ->
/// `NoAcknowledge(Data)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NackKind {
    Address,
    Data,
}

// --- embedded-hal 1.0 i2c::I2c ----------------------------------------------------------------

impl i2c::ErrorType for I2c {
    type Error = I2cError;
}

impl i2c::I2c<SevenBitAddress> for I2c {
    /// Read `read.len()` bytes from `address` (a fresh master read with its own START/STOP).
    fn read(&mut self, address: SevenBitAddress, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(address, read, true)
    }

    /// Write `write` to `address` with a terminating STOP.
    fn write(&mut self, address: SevenBitAddress, write: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(address, write, true, true)
    }

    /// Write `write` then, with a repeated START, read `read` (the register-pointer-then-read
    /// pattern; the IMU WHO_AM_I sequence). The write phase does NOT issue a STOP so the read's
    /// START is a repeated START, and the read is NOT `fresh` (this side already holds the bus,
    /// so I2CBSY is expectedly set).
    fn write_read(
        &mut self,
        address: SevenBitAddress,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_bytes(address, write, true, false)?;
        self.read_bytes(address, read, false)
    }

    /// Execute an `embedded-hal` operation list against `address` as one logical transaction.
    ///
    /// Each `Write`/`Read` runs as its own START; a write's STOP is issued only when it is the
    /// final operation, so a `[Write, Read]` list is the repeated-start register read (the IMU
    /// WHO_AM_I sequence). A `Read` always terminates with a STOP (the receive sequence programs
    /// it), which means a `Read` that is not the final operation is not supported as a
    /// non-terminating phase; M2's transactions (write-then-read) always place the read last.
    /// `fresh` tracks bus ownership across the list: the first operation and any operation after
    /// a STOP wait for bus idle; an operation after a non-stopped write is a repeated START.
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let last = operations.len();
        let mut fresh = true;
        for (i, op) in operations.iter_mut().enumerate() {
            let is_last = i + 1 == last;
            match op {
                Operation::Write(w) => {
                    self.write_bytes(address, w, fresh, is_last)?;
                    fresh = is_last; // a non-stopped write keeps the bus for the next op
                }
                Operation::Read(r) => {
                    self.read_bytes(address, r, fresh)?;
                    fresh = true; // a read always STOPs
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
