//! On-silicon validator for the multi-instance USART RX spec (`specs/uart-rx-multi-instance.md`)
//! and the TX/RX split + reconfigure spec (`specs/usart-split.md`), in TWO images from this one
//! crate:
//!
//! **Default image (single F103 bluepill, intra-board cross-wire):**
//!   - `PA2`  (HAL `PeriphLabel::Usart1` TX, GD USART1 block @0x4000_4400) -> `PB11` (`Usart2` RX,
//!     GD USART2 block @0x4000_4800, F10x-only).
//!   - `PB10` (`Usart2` TX) -> `PA3` (`Usart1` RX).
//!   Both instances are on APB1. Each USART is both a sender and a receiver.
//!   - **S0 (polled, unsplit handles):** lockstep polled loopback each direction.
//!   - **S1 (interrupt-buffered `BufferedRx`):** each USART is `split()`; the senders drive the
//!     `UsartTx` halves, the receivers consume the `UsartRx` halves. Module slot + vector (IRQ 39),
//!     USART1 regression, coexistence.
//!   - **S2 (DMA-ring `RingBufferedRx`):** module channel (DMA0 Ch2), USART1 regression (Ch5),
//!     coexistence, the 2.25 Mbit/s stress, and the 9600 pass. Baud changes between stages run the
//!     spec's ownership sequence: `release -> Usart::rejoin -> set_baud -> split -> re-arm` (the
//!     old `bring_up`-a-second-handle `reprogram()` is gone).
//!
//! **Pair image (`--features pair`, both boards of the F103+F130 pair, role-detected):** re-signs
//! the DMA-ring gate on BOTH families with split handles over the proven master<->slave USART1
//! cross-wire (PA2/PA3, 115200, the 72 MHz tree), the original Gate-B pair topology. Each board is
//! exercised as the DMA receiver (the families differ in channel + IRQ grouping: F10x Ch5
//! separate, F1x0 Ch4 grouped-demux):
//!   - Stage 1 (F1x0 as receiver): the F10x master streams 4096 bytes at 115200; the slave's
//!     `RingBufferedRx` (256 B ring, on the split RX half) drains it. Expected: recv 4096, loss 0,
//!     overrun 0, IDLE seen (the pre-split S3 numbers).
//!   - Stage 2 (F1x0 reconfigure-while-split rule on silicon): the slave runs `release -> rejoin ->
//!     set_baud(9600) -> split -> re-arm` and receives a 64-byte frame at 9600 (the master
//!     reprograms the same way to send it).
//!   - Stage 3 (F10x as receiver, the reverse): both boards reprogram back to 115200 through the
//!     same sequence; the slave streams 4096 bytes and the master's `RingBufferedRx` (Ch5) drains
//!     them. Expected: the same 4096 / 0 / 0 + IDLE.
//!
//! Result is a fixed RAM block at [`RESULT_ADDR`] (the reserved RAM tail, 8 KiB-safe so the pair
//! image fits the F130), `magic` written LAST. PB9 pulses on a full pass (default image). Busy-spin
//! forever, NEVER `wfi` (a bare `wfi` with `DBG_CTL0 = 0` locks SWD re-attach on the GD32).
//!
//! The prior pair-validator (F103+F130 Gate-B, scenarios S2-S5) was committed at `b6bcf04`; the
//! pre-split single-chip validator at `bc61938^` - both recoverable from git history.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use panic_halt as _;

use runtime_hal::{
    clock,
    clock::ClockConfig,
    descriptor::ClockPath,
    detect_chip,
    irq::{install, RamVectorTable, MAX_VECTORS},
    Chip, PeriphLabel, RingBufferedRx, Usart, UsartRx, UsartTx,
};

// --- shared tunables / result plumbing ---------------------------------------------------------

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const BAUD: u32 = 115_200;
/// Deployment-representative baud (the BLE module's rate), used by the reprogram stages.
const BAUD_9600: u32 = 9_600;
/// The DMA ring buffer size (per instance). 256 B is the pre-split S3/S5-validated default.
const DMA_CAP: usize = 256;
/// Empty-iteration budget for a bounded drain wait (bounds a stalled stage so a missed frame
/// cannot hang the run). Single-image only (the pair receiver waits unbounded for a stage start).
#[cfg(not(feature = "pair"))]
const RX_ITERS: u32 = 2_000_000;

const RESULT_ADDR: u32 = 0x2000_1F00;

/// The owned RAM vector table (interrupt + DMA RX need the vectors routed; `install` flips VTOR).
static mut VECTORS: RamVectorTable = RamVectorTable {
    slots: [0; MAX_VECTORS],
};

/// The application-owned `'static` DMA ring buffers. The module + USART1 buffers are distinct so
/// two live DMA channels write disjoint memory (single-image coexistence); the pair image uses
/// only the first.
static mut DMA_BUF_U1: [u8; DMA_CAP] = [0; DMA_CAP];
#[cfg(not(feature = "pair"))]
static mut DMA_BUF_MOD: [u8; DMA_CAP] = [0; DMA_CAP];

/// A fresh `'static` view of a DMA buffer (one active receiver per phase on each buffer; the
/// channel is disabled by `release`/the next `new` before re-use).
fn buf_u1() -> &'static mut [u8] {
    // SAFETY: 'static; a single USART1 RingBufferedRx is active at a time.
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF_U1) as *mut u8, DMA_CAP) }
}
#[cfg(not(feature = "pair"))]
fn buf_mod() -> &'static mut [u8] {
    // SAFETY: as `buf_u1`, on the distinct module buffer.
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF_MOD) as *mut u8, DMA_CAP) }
}

macro_rules! store {
    ($struct:ty, $field:ident, $val:expr) => {{
        // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single-threaded firmware, single writer;
        // reads are external (SWD).
        unsafe {
            let p = RESULT_ADDR as *mut $struct;
            core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
        }
    }};
}

/// Halt forever on an unrecoverable bring-up error. Busy-spin (NEVER wfi: GD32 SWD-lockout rule).
fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

/// Send a frame on the TX half (polled): `write_byte` blocks until TC, so each byte has fully
/// shifted out (and the receiver's DMA/ISR serviced it) by the time this returns.
#[cfg(not(feature = "pair"))]
fn send(tx: &UsartTx, frame: &[u8]) {
    for &b in frame {
        tx.write_byte(b);
    }
}

/// Stream `n` bytes back-to-back with value `(start + i) & 0xFF` (the contiguous pattern the DMA
/// receiver checks for loss).
fn stream(tx: &UsartTx, n: usize, start: usize) {
    for i in 0..n {
        tx.write_byte(((start + i) & 0xFF) as u8);
    }
}

/// Contiguity checker for a known stream (value = position & 0xFF): counts delivered bytes and
/// FORWARD gaps (real loss), ignoring a backward jump (a spurious resync, no data lost).
/// Self-calibrates from the first byte.
struct SeqCheck {
    expected: u8,
    started: bool,
    loss: usize,
    recv: usize,
    first: u8,
}

impl SeqCheck {
    fn new() -> Self {
        SeqCheck {
            expected: 0,
            started: false,
            loss: 0,
            recv: 0,
            first: 0,
        }
    }
    fn push(&mut self, b: u8) {
        self.recv += 1;
        if !self.started {
            self.started = true;
            self.first = b;
        } else if b != self.expected {
            let forward = b.wrapping_sub(self.expected);
            if forward != 0 && forward < 128 {
                self.loss += 1;
            }
        }
        self.expected = b.wrapping_add(1);
    }
}

/// Drain a `RingBufferedRx` until `expect` bytes have arrived or its IDLE boundary / a bounded
/// timeout. Returns `(recv, loss, overrun-count, IDLE-seen, first-byte)`. Single-image only (the
/// pair receiver's stage drain waits unbounded for the stream start).
#[cfg(not(feature = "pair"))]
fn drain_dma(rx: &mut RingBufferedRx, expect: usize) -> (u16, u16, u8, bool, u8) {
    let mut seq = SeqCheck::new();
    let mut ovr = 0u8;
    let mut idle = false;
    let mut empty = 0u32;
    let mut scratch = [0u8; 32];
    loop {
        match rx.read(&mut scratch) {
            Ok(0) => {}
            Ok(n) => {
                for &b in scratch.iter().take(n) {
                    seq.push(b);
                }
                empty = 0;
            }
            Err(_) => {
                ovr = ovr.saturating_add(1);
                empty = 0;
            }
        }
        if rx.take_idle() && seq.recv > 0 {
            idle = true;
            // The burst ended (line idle): drain the rest fully.
            loop {
                match rx.read(&mut scratch) {
                    Ok(0) => break,
                    Ok(n) => {
                        for &b in scratch.iter().take(n) {
                            seq.push(b);
                        }
                    }
                    Err(_) => break,
                }
            }
            break;
        }
        if seq.recv >= expect {
            break;
        }
        empty += 1;
        if empty > RX_ITERS {
            break;
        }
        cortex_m::asm::nop();
    }
    (
        seq.recv.min(u16::MAX as usize) as u16,
        seq.loss.min(u16::MAX as usize) as u16,
        ovr,
        idle,
        seq.first,
    )
}

/// The spec's reconfigure sequence (`specs/usart-split.md` D4): rejoin the halves, reprogram the
/// baud on the whole peripheral, split again. The ONLY way to change baud; a live receiver must
/// have been `release`d first to get the RX half back.
fn reprogram_split(tx: UsartTx, rx: UsartRx, baud: u32) -> (UsartTx, UsartRx) {
    let mut whole = Usart::rejoin(tx, rx);
    whole.set_baud(&CLOCK, baud);
    whole.split()
}

// ================================ default image: single-chip S0/S1/S2 ==========================

#[cfg(not(feature = "pair"))]
mod single {
    use super::*;
    use embedded_hal::digital::OutputPin;
    use runtime_hal::{BufferedRx, RxRing};

    /// S2 high-rate stress baud: APB1/16 = 36 MHz / 16 = 2.25 Mbit/s (`USART_BAUD` 0x10, 0%
    /// divisor error), the F1-series USART maximum on the proven 72 MHz tree.
    const BAUD_FAST: u32 = 2_250_000;
    /// The known frame for the module direction (USART1 TX -> module RX): incrementing.
    const FRAME: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
    /// The known frame for the USART1 direction (module TX -> USART1 RX): a DISJOINT high range,
    /// so cross-talk between the two live slots shows up as a content mismatch.
    const FRAME_HI: [u8; 16] = [
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E,
        0x8F,
    ];
    /// Per-byte empty-poll budget for the S0 lockstep path (a hang backstop).
    const RX_BUDGET: u32 = 1_000_000;
    /// S2 moderate stream length for stages 1-3 + the 9600 pass: fits the DMA ring, so no lap.
    const S2_N: usize = 64;
    /// S2 stress stream length: far larger than the ring (8 wraps).
    const S2_STRESS: usize = 2048;
    /// S2 stress burst size (bytes streamed before each drain): `< DMA_CAP`, so the ring never laps.
    const S2_BURST: usize = 64;
    /// S2 coexistence stream start values (module = LOW range, USART1 = HIGH range, so a
    /// channel/context collision shows up as an out-of-range first byte).
    const COX_MOD_START: u8 = 0x10;
    const COX_U1_START: u8 = 0xC0;

    #[repr(C)]
    pub struct Results {
        /// 0x5332_5242 ("S2RB"), written LAST = the run completed.
        pub magic: u32,
        /// 1 = F10x (the bench part), 2 = F1x0.
        pub role: u8,
        /// Non-zero if `detect_chip` failed.
        pub detect_err: u8,

        // --- S0 (polled loopback) ---
        pub s0_a_len: u8,
        pub s0_a_match: u8,
        pub s0_b_len: u8,
        pub s0_b_match: u8,
        pub s0_pass: u8,

        // --- S1 (interrupt-buffered BufferedRx) ---
        pub s1_mod_len: u8,
        pub s1_mod_match: u8,
        pub s1_mod_idle: u8,
        pub s1_u1_len: u8,
        pub s1_u1_match: u8,
        pub s1_u1_idle: u8,
        pub s1_cox_mod_match: u8,
        pub s1_cox_u1_match: u8,
        pub s1_pass: u8,

        // --- S2 (DMA-ring RingBufferedRx) counts (u16) ---
        pub s2_mod_recv: u16,
        pub s2_u1_recv: u16,
        pub s2_cox_mod_recv: u16,
        pub s2_cox_u1_recv: u16,
        pub s2_stress_recv: u16,
        pub s2_stress_loss: u16,
        pub s2_stress_baud_k: u16,
        pub s2_9600_recv: u16,

        // --- S2 flags (u8) ---
        pub s2_mod_ovr: u8,
        pub s2_mod_idle: u8,
        pub s2_u1_ovr: u8,
        pub s2_9600_match: u8,
        pub s2_cox_ok: u8,
        pub s2_pass: u8,
    }

    const MAGIC: u32 = 0x5332_5242;

    /// The two application-owned `'static` SPSC rings (S1 `BufferedRx`, one per instance). All ops
    /// are `&self` (the HAL-owned `RxRing`), so plain statics, no `static mut`.
    static RING_U1: RxRing<64> = RxRing::new();
    static RING_MOD: RxRing<64> = RxRing::new();

    macro_rules! st {
        ($field:ident, $val:expr) => {
            store!(Results, $field, $val)
        };
    }

    pub fn run() -> ! {
        // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single writer.
        unsafe {
            core::ptr::write_bytes(RESULT_ADDR as *mut u8, 0, core::mem::size_of::<Results>())
        };

        let chip: Chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => {
                st!(detect_err, 1);
                halt();
            }
        };
        st!(
            role,
            match chip.clock() {
                ClockPath::F10xRcc => 1,
                ClockPath::F1x0Rcu => 2,
            }
        );
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // GPIOB carries PB9 (LED) plus the module-USART pins PB10 (TX) / PB11 (RX); GPIOA carries
        // the USART1 pins PA2 (TX) / PA3 (RX).
        let gpiob = match chip.gpiob() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        let mut led = gpiob.pb9.into_push_pull_output();
        let _ = led.set_low();

        // USART1 on PA2/PA3 and the module USART (HAL `Usart2`) on PB10/PB11, 115200 8N1. `new`
        // configures the GPIO AF once; later baud changes go through `set_baud` on the rejoined
        // handle, never a second bring-up.
        let usart1 = match Usart::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart1,
            (gpioa.pa2, gpioa.pa3),
            BAUD,
        ) {
            Ok(u) => u,
            Err(_) => halt(),
        };
        let usart2 = match Usart::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart2,
            (gpiob.pb10, gpiob.pb11),
            BAUD,
        ) {
            Ok(u) => u,
            Err(_) => halt(),
        };

        let s0_pass = run_s0(&usart1, &usart2);
        let (s1_pass, usart1, usart2) = run_s1(&chip, usart1, usart2);
        let s2_pass = run_s2(&chip, usart1, usart2);

        if s0_pass && s1_pass && s2_pass {
            pulse(&mut led);
        }

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        st!(magic, MAGIC);
        halt();
    }

    // --- S0: polled loopback on the unsplit handles ---------------------------------------------

    fn run_s0(usart1: &Usart, usart2: &Usart) -> bool {
        let (a_len, a_match) = loopback(usart1, usart2);
        st!(s0_a_len, a_len);
        st!(s0_a_match, a_match);

        let (b_len, b_match) = loopback(usart2, usart1);
        st!(s0_b_len, b_len);
        st!(s0_b_match, b_match);

        let pass = a_match as usize == FRAME.len() && b_match as usize == FRAME.len();
        st!(s0_pass, pass as u8);
        pass
    }

    /// Send [`FRAME`] on `tx` and read it back polled on `rx`, lockstep.
    fn loopback(tx: &Usart, rx: &Usart) -> (u8, u8) {
        let mut len = 0u8;
        let mut matched = 0u8;
        for (i, &b) in FRAME.iter().enumerate() {
            tx.write_byte(b);
            let mut budget = RX_BUDGET;
            loop {
                match rx.try_read_byte() {
                    Ok(Some(got)) => {
                        len += 1;
                        if got == FRAME[i] {
                            matched += 1;
                        }
                        break;
                    }
                    _ => {
                        budget -= 1;
                        if budget == 0 {
                            return (len, matched);
                        }
                    }
                }
            }
        }
        (len, matched)
    }

    // --- S1: interrupt-buffered BufferedRx on both instances, split handles --------------------

    /// Split both ports, run the three S1 stages, release + rejoin, and hand the whole handles
    /// back for S2.
    fn run_s1(chip: &Chip, usart1: Usart, usart2: Usart) -> (bool, Usart, Usart) {
        // The interrupt + DMA RX need the RAM vector table routed and VTOR flipped BEFORE any `new`
        // unmasks its IRQ. Done once here; S2 reuses it.
        // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
        unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
        // SAFETY: enabling interrupts after the table is installed.
        unsafe { cortex_m::interrupt::enable() };

        let (u1_tx, u1_rx) = usart1.split();
        let (u2_tx, u2_rx) = usart2.split();

        let mut rx_u1 = match BufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, &RING_U1) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        // Module BufferedRx is F10x-only; on F1x0 this fails loud.
        let mut rx_mod = match BufferedRx::new(chip, u2_rx, PeriphLabel::Usart2, &RING_MOD) {
            Ok(r) => r,
            Err(_) => halt(),
        };

        // Stage 1 - module via BufferedRx: USART1's TX half sends FRAME (PA2) -> the module slot.
        send(&u1_tx, &FRAME);
        let (mod_len, mod_match, mod_idle) = drain_buffered(&mut rx_mod, &FRAME);
        st!(s1_mod_len, mod_len);
        st!(s1_mod_match, mod_match);
        st!(s1_mod_idle, mod_idle as u8);

        // Stage 2 - USART1 regression: the module's TX half sends FRAME_HI (PB10) -> USART1's slot.
        send(&u2_tx, &FRAME_HI);
        let (u1_len, u1_match, u1_idle) = drain_buffered(&mut rx_u1, &FRAME_HI);
        st!(s1_u1_len, u1_len);
        st!(s1_u1_match, u1_match);
        st!(s1_u1_idle, u1_idle as u8);

        // Stage 3 - coexistence: interleave both TX lines so both slots fill concurrently.
        for i in 0..FRAME.len() {
            u1_tx.write_byte(FRAME[i]);
            u2_tx.write_byte(FRAME_HI[i]);
        }
        let (_, cox_mod_match, _) = drain_buffered(&mut rx_mod, &FRAME);
        let (_, cox_u1_match, _) = drain_buffered(&mut rx_u1, &FRAME_HI);
        st!(s1_cox_mod_match, cox_mod_match);
        st!(s1_cox_u1_match, cox_u1_match);

        let full = FRAME.len() as u8;
        let pass = mod_match == full
            && mod_idle
            && u1_match == full
            && u1_idle
            && cox_mod_match == full
            && cox_u1_match == full;
        st!(s1_pass, pass as u8);

        // Release the RX halves and rejoin: S2 gets whole handles to split for the DMA stages.
        let u1_rx = rx_u1.release();
        let u2_rx = rx_mod.release();
        (
            pass,
            Usart::rejoin(u1_tx, u1_rx),
            Usart::rejoin(u2_tx, u2_rx),
        )
    }

    /// Drain a `BufferedRx` until its IDLE boundary (or a timeout), comparing against `expect`.
    fn drain_buffered(rx: &mut BufferedRx, expect: &[u8]) -> (u8, u8, bool) {
        let mut bytes = [0u8; 64];
        let mut len = 0usize;
        let mut idle = false;
        let mut empty = 0u32;
        loop {
            match rx.read(&mut bytes[len..]) {
                Ok(0) => {}
                Ok(n) => {
                    len += n;
                    empty = 0;
                }
                Err(_) => empty = 0,
            }
            if rx.take_idle() && len > 0 {
                idle = true;
                break;
            }
            if len >= bytes.len() {
                break;
            }
            empty += 1;
            if empty > RX_ITERS {
                break;
            }
            cortex_m::asm::nop();
        }
        let mut matched = 0u8;
        for i in 0..len.min(expect.len()) {
            if bytes[i] == expect[i] {
                matched += 1;
            }
        }
        (len.min(u8::MAX as usize) as u8, matched, idle)
    }

    // --- S2: DMA-ring RingBufferedRx on both instances, split handles + set_baud ---------------

    /// Run the five S2 stages. Stage transitions release the RX half; the baud changes (stages
    /// 4-5) run the full `release -> rejoin -> set_baud -> split` sequence on BOTH ports.
    fn run_s2(chip: &Chip, usart1: Usart, usart2: Usart) -> bool {
        let (u1_tx, u1_rx) = usart1.split();
        let (u2_tx, u2_rx) = usart2.split();

        // Stage 1 - module via RingBufferedRx @115200: USART1 streams -> the module channel (Ch2).
        let mut rxm = match RingBufferedRx::new(chip, u2_rx, PeriphLabel::Usart2, buf_mod()) {
            Ok(r) => r,
            Err(_) => halt(), // module RingBufferedRx is F10x-only; on F1x0 this fails loud
        };
        stream(&u1_tx, S2_N, 0);
        let (mod_recv, _mod_loss, mod_ovr, mod_idle, mod_first) = drain_dma(&mut rxm, S2_N);
        st!(s2_mod_recv, mod_recv);
        st!(s2_mod_ovr, mod_ovr);
        st!(s2_mod_idle, mod_idle as u8);
        let mod_ok = mod_recv as usize == S2_N && mod_ovr == 0 && mod_idle && mod_first < 0x80;
        let u2_rx = rxm.release();

        // Stage 2 - USART1 RingBufferedRx regression @115200 (Ch5): the module streams -> USART1.
        let mut rx1 = match RingBufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, buf_u1()) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        stream(&u2_tx, S2_N, 0);
        let (u1_recv, _u1_loss, u1_ovr, _u1_idle, _u1_first) = drain_dma(&mut rx1, S2_N);
        st!(s2_u1_recv, u1_recv);
        st!(s2_u1_ovr, u1_ovr);
        let u1_ok = u1_recv as usize == S2_N && u1_ovr == 0;
        let u1_rx = rx1.release();

        // Stage 3 - coexistence: both DMA receivers live, disjoint streams.
        let mut rxm = match RingBufferedRx::new(chip, u2_rx, PeriphLabel::Usart2, buf_mod()) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        let mut rx1 = match RingBufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, buf_u1()) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        for i in 0..S2_N {
            u1_tx.write_byte(COX_MOD_START.wrapping_add(i as u8)); // -> module RX (Ch2)
            u2_tx.write_byte(COX_U1_START.wrapping_add(i as u8)); // -> USART1 RX (Ch5)
        }
        let (cox_mod_recv, _, _, _, cox_mod_first) = drain_dma(&mut rxm, S2_N);
        let (cox_u1_recv, _, _, _, cox_u1_first) = drain_dma(&mut rx1, S2_N);
        st!(s2_cox_mod_recv, cox_mod_recv);
        st!(s2_cox_u1_recv, cox_u1_recv);
        let cox_ok = cox_mod_recv as usize == S2_N
            && cox_mod_first < 0x80
            && cox_u1_recv as usize == S2_N
            && cox_u1_first >= 0x80;
        st!(s2_cox_ok, cox_ok as u8);
        let u2_rx = rxm.release();
        let u1_rx = rx1.release();

        // Stage 4 - high-rate stress at 2.25 Mbit/s: the spec's reconfigure sequence on BOTH ports
        // (release happened above; rejoin -> set_baud -> split), then a fresh module receiver.
        let (u1_tx, u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD_FAST);
        let (u2_tx, u2_rx) = reprogram_split(u2_tx, u2_rx, BAUD_FAST);
        let mut rxm = match RingBufferedRx::new(chip, u2_rx, PeriphLabel::Usart2, buf_mod()) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        let (stress_recv, stress_loss) = stress(&u1_tx, &mut rxm, S2_STRESS, S2_BURST);
        st!(s2_stress_recv, stress_recv);
        st!(s2_stress_loss, stress_loss);
        st!(s2_stress_baud_k, (BAUD_FAST / 1000) as u16);
        // Allow a couple of in-flight bytes at the final drain; loss must be exactly 0.
        let stress_ok = (stress_recv as usize) + 4 >= S2_STRESS && stress_loss == 0;
        let u2_rx = rxm.release();

        // Stage 5 - the deployment-representative 9600 8N1 pass, same reconfigure sequence.
        let (u1_tx, _u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD_9600);
        let (_u2_tx, u2_rx) = reprogram_split(u2_tx, u2_rx, BAUD_9600);
        let recv96 = match RingBufferedRx::new(chip, u2_rx, PeriphLabel::Usart2, buf_mod()) {
            Ok(mut r) => {
                stream(&u1_tx, S2_N, 0);
                let (recv, _, _, _, _) = drain_dma(&mut r, S2_N);
                recv
            }
            Err(_) => halt(),
        };
        st!(s2_9600_recv, recv96);
        let ok96 = recv96 as usize == S2_N;
        st!(s2_9600_match, ok96 as u8);

        let pass = mod_ok && u1_ok && cox_ok && stress_ok && ok96;
        st!(s2_pass, pass as u8);
        pass
    }

    /// High-rate stress: stream `total` bytes (>> the ring) in `burst`-sized chunks, draining the
    /// module `RingBufferedRx` after each burst.
    fn stress(tx: &UsartTx, rx: &mut RingBufferedRx, total: usize, burst: usize) -> (u16, u16) {
        let mut seq = SeqCheck::new();
        let mut scratch = [0u8; 64];
        let mut sent = 0usize;
        while sent < total {
            let k = burst.min(total - sent);
            for i in 0..k {
                tx.write_byte(((sent + i) & 0xFF) as u8);
            }
            sent += k;
            loop {
                match rx.read(&mut scratch) {
                    Ok(0) => break,
                    Ok(n) => {
                        for &b in scratch.iter().take(n) {
                            seq.push(b);
                        }
                    }
                    Err(_) => break, // a lap overrun resyncs; the next forward jump records the loss
                }
            }
        }
        // Settle (~1 ms), then a final drain to catch the last in-flight bytes.
        cortex_m::asm::delay(72_000);
        loop {
            match rx.read(&mut scratch) {
                Ok(0) => break,
                Ok(n) => {
                    for &b in scratch.iter().take(n) {
                        seq.push(b);
                    }
                }
                Err(_) => break,
            }
        }
        (
            seq.recv.min(u16::MAX as usize) as u16,
            seq.loss.min(u16::MAX as usize) as u16,
        )
    }

    /// Pulse PB9 (~100 ms): the human-visible "S0 + S1 + S2 passed".
    fn pulse(led: &mut impl OutputPin) {
        let _ = led.set_high();
        cortex_m::asm::delay(7_200_000);
        let _ = led.set_low();
    }
}

// ================================ pair image: bidirectional F103 <-> F130 DMA gate =============

#[cfg(feature = "pair")]
mod pair {
    use super::*;

    /// Stage 1: the pre-split S3 gate stream (4 KB continuous at 115200 into the 256 B ring).
    const P1_N: usize = 4096;
    /// Stage 2: the deployment-representative frame at 9600 after the split reconfigure.
    const P2_N: usize = 64;
    /// Receiver empty-iteration budget WITHIN a stage once bytes are flowing / for the tail.
    const GAP_ITERS: u32 = 20_000_000;

    #[repr(C)]
    pub struct PairResults {
        /// 0x5052_5242 ("PRRB"), written LAST = this board's run completed.
        pub magic: u32,
        /// 1 = F10x (master: stage-1/2 sender, stage-3 receiver), 2 = F1x0 (slave: stage-1/2
        /// receiver, stage-3 sender).
        pub role: u8,
        /// Non-zero if `detect_chip` failed.
        pub detect_err: u8,
        /// Slave stage 1 (4096 @ 115200, Ch4 grouped): received / real-loss / error count /
        /// IDLE-seen.
        pub p1_recv: u16,
        pub p1_loss: u16,
        pub p1_ovr: u8,
        pub p1_idle: u8,
        /// Slave stage 2 (64 @ 9600 after release -> rejoin -> set_baud -> split -> re-arm).
        pub p2_recv: u16,
        pub p2_ovr: u8,
        /// Master stage 3 (4096 @ 115200, Ch5 separate, after reprogramming back up): received /
        /// real-loss / error count / IDLE-seen.
        pub r1_recv: u16,
        pub r1_loss: u16,
        pub r1_ovr: u8,
        pub r1_idle: u8,
        /// This board's receiver stages at their expected counts with zero loss/errors.
        pub pass: u8,
        /// Sender stages: bytes streamed (a record of what was driven).
        pub sent1: u16,
        pub sent2: u16,
        pub sent_rev: u16,
    }

    const MAGIC: u32 = 0x5052_5242;

    macro_rules! st {
        ($field:ident, $val:expr) => {
            store!(PairResults, $field, $val)
        };
    }

    pub fn run() -> ! {
        // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single writer.
        unsafe {
            core::ptr::write_bytes(
                RESULT_ADDR as *mut u8,
                0,
                core::mem::size_of::<PairResults>(),
            )
        };

        let chip: Chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => {
                st!(detect_err, 1);
                halt();
            }
        };
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            halt();
        }

        // USART1 on PA2 (TX) / PA3 (RX), 115200 8N1: the proven master<->slave cross-wire.
        let gpioa = match chip.gpioa() {
            Ok(p) => p.split(),
            Err(_) => halt(),
        };
        let usart1 = match Usart::new(
            &chip,
            &CLOCK,
            PeriphLabel::Usart1,
            (gpioa.pa2, gpioa.pa3),
            BAUD,
        ) {
            Ok(u) => u,
            Err(_) => halt(),
        };

        // Both roles arm a DMA receiver at some point, so both need the RAM vector table + IRQs.
        // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
        unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
        // SAFETY: enabling interrupts after the table is installed.
        unsafe { cortex_m::interrupt::enable() };

        match chip.clock() {
            ClockPath::F10xRcc => {
                st!(role, 1);
                run_master(&chip, usart1)
            }
            ClockPath::F1x0Rcu => {
                st!(role, 2);
                run_slave(&chip, usart1)
            }
        }
    }

    /// F10x master: stream stages 1-2 on the TX half (with the 9600 reconfigure between), then
    /// reprogram back to 115200 and become the STAGE-3 DMA RECEIVER (Ch5 separate IRQ): the F10x
    /// half of the family gate.
    fn run_master(chip: &Chip, usart1: Usart) -> ! {
        let (u1_tx, u1_rx) = usart1.split();

        // Boot settle: the slave is flashed + armed FIRST (it waits indefinitely for the first
        // byte), so a fixed delay only has to cover this board's own reset (~3 s for margin).
        cortex_m::asm::delay(72_000_000 * 3);

        stream(&u1_tx, P1_N, 0);
        st!(sent1, P1_N as u16);

        // Give the slave time to record stage 1 + run its reconfigure sequence (~instant; 1 s is
        // generous), then reconfigure this side the same way and send the 9600 frame.
        cortex_m::asm::delay(72_000_000);
        let (u1_tx, u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD_9600);
        stream(&u1_tx, P2_N, 0);
        st!(sent2, P2_N as u16);

        // Stage 3: back to 115200 through the same sequence, arm the Ch5 receiver, and drain the
        // slave's 4096-byte stream (the slave waits ~3 s after its stage 2 before streaming, which
        // covers this arm-up many times over).
        let (_u1_tx, u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD);
        let mut ring = match RingBufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, buf_u1()) {
            Ok(r) => r,
            Err(_) => halt(),
        };
        let (recv, loss, ovr, idle) = drain_stage(&mut ring, P1_N);
        st!(r1_recv, recv);
        st!(r1_loss, loss);
        st!(r1_ovr, ovr);
        st!(r1_idle, idle as u8);

        let pass = recv as usize == P1_N && loss == 0 && ovr == 0;
        st!(pass, pass as u8);

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        st!(magic, MAGIC);
        halt();
    }

    /// F1x0 slave: the F1x0 DMA gate. `RingBufferedRx` on the split RX half (grouped-demux Ch4)
    /// drains stage 1, the reconfigure sequence + stage 2 at 9600, then it reprograms back to
    /// 115200 and becomes the STAGE-3 SENDER for the master's receiver.
    fn run_slave(chip: &Chip, usart1: Usart) -> ! {
        let (u1_tx, u1_rx) = usart1.split();
        let mut ring = match RingBufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, buf_u1()) {
            Ok(r) => r,
            Err(_) => halt(),
        };

        // Stage 1: wait indefinitely for the master's stream, then drain it (gap-bounded).
        let (recv1, loss1, ovr1, idle1) = drain_stage(&mut ring, P1_N);
        st!(p1_recv, recv1);
        st!(p1_loss, loss1);
        st!(p1_ovr, ovr1);
        st!(p1_idle, idle1 as u8);

        // The reconfigure-while-split rule, on F1x0 silicon: release -> rejoin -> set_baud ->
        // split -> re-arm.
        let u1_rx = ring.release();
        let (u1_tx, u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD_9600);
        let mut ring = match RingBufferedRx::new(chip, u1_rx, PeriphLabel::Usart1, buf_u1()) {
            Ok(r) => r,
            Err(_) => halt(),
        };

        // Stage 2: the 9600 frame.
        let (recv2, _loss2, ovr2, _idle2) = drain_stage(&mut ring, P2_N);
        st!(p2_recv, recv2);
        st!(p2_ovr, ovr2);

        let pass = recv1 as usize == P1_N && loss1 == 0 && ovr1 == 0 && recv2 as usize == P2_N;
        st!(pass, pass as u8);

        // Stage 3: reprogram back to 115200 (release -> rejoin -> set_baud -> split), give the
        // master ~3 s to finish its own reprogram + arm its Ch5 receiver, then stream.
        let u1_rx = ring.release();
        let (u1_tx, _u1_rx) = reprogram_split(u1_tx, u1_rx, BAUD);
        cortex_m::asm::delay(72_000_000 * 3);
        stream(&u1_tx, P1_N, 0);
        st!(sent_rev, P1_N as u16);

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        st!(magic, MAGIC);
        halt();
    }

    /// Wait indefinitely for a stage's FIRST byte (the bench flashes + arms this board before the
    /// sender starts), then drain gap-bounded until `expect` bytes or the stream stops. Returns
    /// `(recv, loss, error-count, idle-seen)`.
    fn drain_stage(rx: &mut RingBufferedRx, expect: usize) -> (u16, u16, u8, bool) {
        let mut seq = SeqCheck::new();
        let mut ovr = 0u8;
        let mut idle = false;
        let mut empty = 0u32;
        let mut scratch = [0u8; 32];
        loop {
            match rx.read(&mut scratch) {
                Ok(0) => {}
                Ok(n) => {
                    for &b in scratch.iter().take(n) {
                        seq.push(b);
                    }
                    empty = 0;
                }
                Err(_) => {
                    ovr = ovr.saturating_add(1);
                    empty = 0;
                }
            }
            if rx.take_idle() {
                idle = true;
            }
            if seq.recv >= expect {
                break;
            }
            if seq.recv > 0 {
                // Bytes are flowing: a bounded gap ends the stage (the stream stopped short).
                empty += 1;
                if empty > GAP_ITERS {
                    break;
                }
            }
            // recv == 0: wait forever for the stage to start.
            cortex_m::asm::nop();
        }
        // Tail drain.
        loop {
            match rx.read(&mut scratch) {
                Ok(0) => break,
                Ok(n) => {
                    for &b in scratch.iter().take(n) {
                        seq.push(b);
                    }
                }
                Err(_) => break,
            }
        }
        (
            seq.recv.min(u16::MAX as usize) as u16,
            seq.loss.min(u16::MAX as usize) as u16,
            ovr,
            idle,
        )
    }
}

#[entry]
fn main() -> ! {
    #[cfg(not(feature = "pair"))]
    single::run();
    #[cfg(feature = "pair")]
    pair::run();
}
