//! The remap image (`--features remap`): USART0 on its remapped PB6/PB7 on an F103C8 bluepill
//! (`runtime-hal/specs/usart-pin-remap.md` section 5, "Bench").
//!
//! Wiring (USART2 is the witness for USART0's remapped pins; USART0's default pins PA9/PA10 stay
//! unjumpered):
//!
//! | Jumper | Proves |
//! |---|---|
//! | PB6 -> PB11 | USART0's remapped TX is on PB6 (USART2 receives it) |
//! | PB10 -> PB7 | USART0's remapped RX is on PB7 (USART0 receives USART2) |
//! | PA2 -> PA3 | USART1 self-loopback, running at the same time |
//!
//! Checks, in order, all recorded in [`Results`] at `RESULT_ADDR` (`magic` written last):
//!
//! 1. **Remap, polled.** `free_jtag_pins` first (so check 3 can see whether `SWJ_CFG` survives the
//!    remap's read-modify-write). Then, with `USART0_REMAP` still clear: PB6/PB7 are routed as
//!    USART pins by hand (`Chip::route_usart_pin`) and USART0 is brought up without pins
//!    (`Usart::bring_up`), so USART0's TX goes to PA9 and PB6 drives no USART0 signal; USART0
//!    sends its pattern and USART2 must receive none of it. Then `Usart::new(Usart0, PB6, PB7)`
//!    sets the remap; USART0 -> USART2 and USART2 -> USART0 must each deliver the 16-byte pattern.
//!    Each transmitter has its own byte range, so a byte from the wrong one cannot count.
//! 2. **DMA, all three at once.** USART0 and USART2 stream to each other and USART1 to itself, each
//!    through `DmaTx` into a `RingBufferedRx`, [`DMA_N`] bytes each at [`LINK_BAUD`]: every receiver
//!    must get its pattern intact (zero mismatches) with zero read errors and zero transmit
//!    timeouts. The 256-byte rings wrap eight times, so every instance's DMA receive vector runs
//!    (F10x: USART0 Ch4 = IRQ 15, USART1 Ch5 = IRQ 16, USART2 Ch2 = IRQ 13).
//! 3. **Registers.** `PCF0` before and after the remap, `GPIOB_CTL0` (PB6/PB7 nibbles) and
//!    `GPIOA_CTL1` (PA9/PA10 nibbles) after the run, read by the firmware into the result block;
//!    the bench reads the same addresses over SWD and checks the probe re-attaches.
//!
//! F10x only: on any other family the run records `role` and halts before touching a USART.

use super::*;
use runtime_hal::{
    gpio::{PinRole, UsartAf},
    DescriptorError, DmaTx, Oversampling, UsartConfig, UsartFrame,
};

/// The inter-board link's baud on the hoverboard firmware, so check 2 runs at the rate a link uses.
const LINK_BAUD: u32 = 460_800;
/// Bytes each transmitter streams in check 2: eight laps of a 256-byte ring.
const DMA_N: usize = 2048;
/// The DMA transmit buffer, per USART: the largest chunk one `try_send` takes.
const TX_CHUNK: usize = 32;
/// Polled-check per-byte empty-poll budget (a hang backstop, about 50 ms at 72 MHz).
const POLL_BUDGET: u32 = 1_000_000;
/// Check-2 no-progress budget: main-loop passes with nothing sent or received before giving up.
const STALL_BUDGET: u32 = 4_000_000;

/// Pattern bases per transmitter (pattern byte i = base + i). Check 1's 16-byte frames are disjoint
/// ranges that avoid 0x00 and 0xFF (what a held-low or floating line delivers), so a junk byte
/// cannot count as a match; in check 2 the streams wrap through every value and cross-talk shows as
/// a break in the receiver's sequence.
const BASE_U0: u8 = 0x30;
const BASE_U2: u8 = 0x90;
const BASE_U1: u8 = 0xC5;
/// Check 1's frame length.
const FRAME_LEN: u8 = 16;

/// `AFIO_PCF0` and the two GPIO config words check 3 reads (F10x absolute addresses).
const AFIO_PCF0: u32 = 0x4001_0004;
const GPIOA_CTL1: u32 = 0x4001_0804;
const GPIOB_CTL0: u32 = 0x4001_0C00;

/// The result block at `RESULT_ADDR` (0x2000_1F00). Offsets for an SWD read:
///
/// | off | field | pass value |
/// |---|---|---|
/// | 0x00 | `magic` | `0x5245_4D50` ("REMP"), written last |
/// | 0x04 | `pcf0_before` | `0x0200_0000` (`SWJ_CFG` = `0b010` after `free_jtag_pins`), bit 2 clear |
/// | 0x08 | `pcf0_after` | `0x0200_0004` (bit 2 set, `SWJ_CFG` kept) |
///
/// `SWJ_CFG` is documented write-only (GD32F10x UM Rev2.6 section 7.5.9). If `pcf0_before` reads
/// `[26:24]` = 0, the field does not read back, and every PCF0 read-modify-write in `afio.rs`
/// (this remap and `remap_timer1_partial0`) writes it back as `0b000`, which re-enables JTAG on
/// PA15/PB3/PB4 (SWD stays on): a finding for `afio.rs`, not a failed remap.
/// | 0x0C | `gpiob_ctl0` | `[31:24]` = `0x4B` (PB7 floating input, PB6 AF push-pull) |
/// | 0x10 | `gpioa_ctl1` | `[11:4]` = `0x44` (PA10, PA9 at reset: floating input) |
/// | 0x14 | `dma_u0_recv` / `dma_u0_bad` (u16 each) | 2048 / 0 |
/// | 0x18 | `dma_u2_recv` / `dma_u2_bad` | 2048 / 0 |
/// | 0x1C | `dma_u1_recv` / `dma_u1_bad` | 2048 / 0 |
/// | 0x20 | `role`, `stage`, `err_code`, `a_u2_recv` (u8 each) | 1, 4, 0, any |
/// | 0x24 | `a_u2_match`, `b_u2_match`, `b_u0_match`, `c1_pass` | 0, 16, 16, 1 |
/// | 0x28 | `dma_u0_err`, `dma_u2_err`, `dma_u1_err`, `dma_tx_timeouts` | 0, 0, 0, 0 |
/// | 0x2C | `c2_pass`, `pass` | 1, 1 |
#[repr(C)]
pub struct Results {
    pub magic: u32,
    pub pcf0_before: u32,
    pub pcf0_after: u32,
    pub gpiob_ctl0: u32,
    pub gpioa_ctl1: u32,
    /// Check 2: bytes USART0's ring received (from USART2), and how many were not the pattern.
    pub dma_u0_recv: u16,
    pub dma_u0_bad: u16,
    /// Check 2: USART2's ring (from USART0).
    pub dma_u2_recv: u16,
    pub dma_u2_bad: u16,
    /// Check 2: USART1's ring (its own loopback).
    pub dma_u1_recv: u16,
    pub dma_u1_bad: u16,
    /// 1 = F10x, 2 = F1x0 (the run halts there), 0 = detection failed.
    pub role: u8,
    /// The last stage entered: 1 = check 1 with the bit clear, 2 = with it set, 3 = check 2,
    /// 4 = done. A halt leaves the stage it stopped in.
    pub stage: u8,
    /// `DescriptorError::code()` of a refused bring-up (the run halts), 0 = none.
    pub err_code: u8,
    /// Check 1, bit clear: bytes USART2 received while USART0 sent (junk allowed).
    pub a_u2_recv: u8,
    /// Check 1, bit clear: of those, USART0 pattern bytes. Must be 0.
    pub a_u2_match: u8,
    /// Check 1, bit set: USART0 -> USART2 and USART2 -> USART0 pattern matches. Must be 16 each.
    pub b_u2_match: u8,
    pub b_u0_match: u8,
    pub c1_pass: u8,
    /// Check 2: read errors per receiver (line errors or ring laps) and abandoned transmits.
    pub dma_u0_err: u8,
    pub dma_u2_err: u8,
    pub dma_u1_err: u8,
    pub dma_tx_timeouts: u8,
    pub c2_pass: u8,
    pub pass: u8,
}

const MAGIC: u32 = 0x5245_4D50;

// The table above is the SWD reader's contract.
const _: () = assert!(core::mem::size_of::<Results>() == 0x30);

/// The three receive rings and three transmit buffers (application-owned, DECISIONS.md #10).
static mut RX_U0: [u8; DMA_CAP] = [0; DMA_CAP];
static mut RX_U1: [u8; DMA_CAP] = [0; DMA_CAP];
static mut RX_U2: [u8; DMA_CAP] = [0; DMA_CAP];
static mut TX_U0: [u8; TX_CHUNK] = [0; TX_CHUNK];
static mut TX_U1: [u8; TX_CHUNK] = [0; TX_CHUNK];
static mut TX_U2: [u8; TX_CHUNK] = [0; TX_CHUNK];

/// A `'static` view of one of the buffers above, of length `$len`. Each is handed to exactly one
/// receiver or transmitter, once.
macro_rules! buf {
    ($b:ident, $len:expr) => {
        // SAFETY: a 'static buffer of `$len` bytes handed to one owner, once, in a single-threaded
        // run.
        unsafe { core::slice::from_raw_parts_mut(addr_of_mut!($b) as *mut u8, $len) }
    };
}

macro_rules! st {
    ($field:ident, $val:expr) => {
        store!(Results, $field, $val)
    };
}

fn read32(addr: u32) -> u32 {
    // SAFETY: a word read of an F10x peripheral register (AFIO / GPIO), no side effect on read.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Record a refused bring-up's code and halt (busy-spin).
fn refused(e: DescriptorError) -> ! {
    st!(err_code, e.code());
    halt();
}

pub fn run() -> ! {
    // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single writer.
    unsafe { core::ptr::write_bytes(RESULT_ADDR as *mut u8, 0, core::mem::size_of::<Results>()) };

    let chip: Chip = match detect_chip() {
        Ok(c) => c,
        Err(_) => halt(),
    };
    match chip.clock() {
        ClockPath::F10xRcc => st!(role, 1),
        ClockPath::F1x0Rcu => {
            st!(role, 2);
            halt();
        }
    }
    if clock::configure_tree(&chip, &CLOCK).is_err() {
        halt();
    }
    // The interrupt and DMA receivers need the RAM vector table routed before any `new` unmasks.
    // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
    unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
    // SAFETY: enabling interrupts after the table is installed.
    unsafe { cortex_m::interrupt::enable() };

    // SW-only debug (frees PA15/PB3/PB4, keeps SWD): check 3 then shows whether the remap's
    // read-modify-write of PCF0 keeps SWJ_CFG.
    if let Err(e) = chip.free_jtag_pins() {
        refused(e);
    }
    st!(pcf0_before, read32(AFIO_PCF0));

    let pin = |p: u8| chip.pin(p).unwrap_or_else(|e| refused(e));
    let usart1 = Usart::new(
        &chip,
        &CLOCK,
        PeriphLabel::Usart1,
        (pin(0x02), pin(0x03)),
        LINK_BAUD,
    )
    .unwrap_or_else(|e| refused(e));
    let usart2 = Usart::new(
        &chip,
        &CLOCK,
        PeriphLabel::Usart2,
        (pin(0x1A), pin(0x1B)),
        LINK_BAUD,
    )
    .unwrap_or_else(|e| refused(e));

    // --- check 1a: the bit clear. PB6/PB7 routed as USART pins, USART0 up without the remap. ---
    st!(stage, 1);
    if let Err(e) = chip.route_usart_pin(0x16, PinRole::Tx(UsartAf::Af0)) {
        refused(e);
    }
    if let Err(e) = chip.route_usart_pin(0x17, PinRole::Rx(UsartAf::Af0)) {
        refused(e);
    }
    // The pin-less handle lives only in this block, ending before the real bring-up (one handle
    // per instance).
    let (a_recv, a_match) = {
        let unmapped = Usart::bring_up(
            &chip,
            &CLOCK,
            &UsartConfig {
                usart: PeriphLabel::Usart0,
                baud: LINK_BAUD,
                frame: UsartFrame::EIGHT_N_ONE,
                oversampling: Oversampling::By16,
            },
        )
        .unwrap_or_else(|e| refused(e));
        flush(&usart2);
        polled(&unmapped, &usart2, BASE_U0)
    };
    st!(a_u2_recv, a_recv);
    st!(a_u2_match, a_match);

    // --- check 1b: `Usart::new` sets USART0_REMAP; both directions on the remapped pins. -------
    st!(stage, 2);
    let usart0 = Usart::new(
        &chip,
        &CLOCK,
        PeriphLabel::Usart0,
        (pin(0x16), pin(0x17)),
        LINK_BAUD,
    )
    .unwrap_or_else(|e| refused(e));
    st!(pcf0_after, read32(AFIO_PCF0));
    flush(&usart2);
    flush(&usart0);
    let (_, b_u2) = polled(&usart0, &usart2, BASE_U0);
    let (_, b_u0) = polled(&usart2, &usart0, BASE_U2);
    st!(b_u2_match, b_u2);
    st!(b_u0_match, b_u0);
    let c1 = a_match == 0 && b_u2 == FRAME_LEN && b_u0 == FRAME_LEN;
    st!(c1_pass, c1 as u8);

    // --- check 2: DMA on all three at once. ------------------------------------------------------
    st!(stage, 3);
    flush(&usart0);
    flush(&usart1);
    flush(&usart2);
    let c2 = dma_all(&chip, usart0, usart1, usart2);
    st!(c2_pass, c2 as u8);

    // --- check 3: the registers after the run. ---------------------------------------------------
    st!(gpiob_ctl0, read32(GPIOB_CTL0));
    st!(gpioa_ctl1, read32(GPIOA_CTL1));
    st!(pass, (c1 && c2) as u8);
    st!(stage, 4);
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    st!(magic, MAGIC);
    halt();
}

/// Read and discard whatever a USART holds (stale bytes, a latched line error) so a check starts
/// from an empty receiver.
fn flush(u: &Usart) {
    let mut quiet = 0u32;
    while quiet < 10_000 {
        match u.try_read_byte() {
            Ok(None) => quiet += 1,
            _ => quiet = 0,
        }
    }
}

/// Send the 16-byte pattern `base + i` on `tx`, polling `rx` after each byte. Returns
/// `(bytes received, of those equal to the pattern byte sent)`.
fn polled(tx: &Usart, rx: &Usart, base: u8) -> (u8, u8) {
    let (mut recv, mut matched) = (0u8, 0u8);
    for i in 0..FRAME_LEN {
        let b = base.wrapping_add(i);
        tx.write_byte(b);
        let mut budget = POLL_BUDGET;
        while budget > 0 {
            match rx.try_read_byte() {
                Ok(Some(got)) => {
                    recv = recv.saturating_add(1);
                    if got == b {
                        matched += 1;
                    }
                    break;
                }
                _ => budget -= 1,
            }
        }
    }
    (recv, matched)
}

/// One receiver's tally: the next expected pattern byte, bytes received, mismatches, read errors.
struct Tally {
    expect: u8,
    recv: usize,
    bad: usize,
    err: usize,
}

impl Tally {
    fn new(base: u8) -> Self {
        Tally {
            expect: base,
            recv: 0,
            bad: 0,
            err: 0,
        }
    }

    /// Drain `rx`; true if anything happened.
    fn drain(&mut self, rx: &mut RingBufferedRx) -> bool {
        let mut scratch = [0u8; 32];
        match rx.read(&mut scratch) {
            Ok(0) => false,
            Ok(n) => {
                for &b in &scratch[..n] {
                    if b != self.expect {
                        self.bad += 1;
                    }
                    self.expect = b.wrapping_add(1);
                    self.recv += 1;
                }
                true
            }
            Err(_) => {
                self.err += 1;
                true
            }
        }
    }
}

/// One transmitter's progress: the pattern base and bytes handed to the channel so far.
struct Feed {
    base: u8,
    sent: usize,
}

impl Feed {
    /// Hand the next chunk to `tx` if it is ready; true if it took bytes.
    fn pump(&mut self, tx: &mut DmaTx) -> bool {
        if self.sent >= DMA_N || !tx.ready() {
            return false;
        }
        let mut chunk = [0u8; TX_CHUNK];
        let k = TX_CHUNK.min(DMA_N - self.sent);
        for (i, b) in chunk[..k].iter_mut().enumerate() {
            *b = self.base.wrapping_add((self.sent + i) as u8);
        }
        match tx.try_send(&chunk[..k]) {
            Ok(n) => {
                self.sent += n;
                n > 0
            }
            Err(_) => false,
        }
    }
}

/// Check 2: arm a `RingBufferedRx` and a `DmaTx` on every USART, stream [`DMA_N`] bytes from each
/// transmitter at once, and tally every receiver.
fn dma_all(chip: &Chip, usart0: Usart, usart1: Usart, usart2: Usart) -> bool {
    let (tx0, rx0) = usart0.split();
    let (tx1, rx1) = usart1.split();
    let (tx2, rx2) = usart2.split();
    let ring = |rx, label, b: &'static mut [u8]| {
        RingBufferedRx::new(chip, rx, label, b).unwrap_or_else(|e| refused(e))
    };
    let mut r0 = ring(rx0, PeriphLabel::Usart0, buf!(RX_U0, DMA_CAP));
    let mut r1 = ring(rx1, PeriphLabel::Usart1, buf!(RX_U1, DMA_CAP));
    let mut r2 = ring(rx2, PeriphLabel::Usart2, buf!(RX_U2, DMA_CAP));
    let core_hz = CLOCK.sysclk_hz;
    let dma = |tx, b: &'static mut [u8]| {
        DmaTx::new(chip, tx, b, core_hz, LINK_BAUD).unwrap_or_else(|e| refused(e))
    };
    let mut t0 = dma(tx0, buf!(TX_U0, TX_CHUNK));
    let mut t1 = dma(tx1, buf!(TX_U1, TX_CHUNK));
    let mut t2 = dma(tx2, buf!(TX_U2, TX_CHUNK));

    // USART0 -> USART2, USART2 -> USART0, USART1 -> itself.
    let (mut f0, mut f1, mut f2) = (
        Feed {
            base: BASE_U0,
            sent: 0,
        },
        Feed {
            base: BASE_U1,
            sent: 0,
        },
        Feed {
            base: BASE_U2,
            sent: 0,
        },
    );
    let (mut k0, mut k1, mut k2) = (
        Tally::new(BASE_U2),
        Tally::new(BASE_U1),
        Tally::new(BASE_U0),
    );
    let mut stall = 0u32;
    while (k0.recv < DMA_N || k1.recv < DMA_N || k2.recv < DMA_N) && stall < STALL_BUDGET {
        let moved = f0.pump(&mut t0)
            | f1.pump(&mut t1)
            | f2.pump(&mut t2)
            | k0.drain(&mut r0)
            | k1.drain(&mut r1)
            | k2.drain(&mut r2);
        stall = if moved { 0 } else { stall + 1 };
    }

    let n16 = |n: usize| n.min(u16::MAX as usize) as u16;
    let n8 = |n: usize| n.min(u8::MAX as usize) as u8;
    st!(dma_u0_recv, n16(k0.recv));
    st!(dma_u0_bad, n16(k0.bad));
    st!(dma_u0_err, n8(k0.err));
    st!(dma_u1_recv, n16(k1.recv));
    st!(dma_u1_bad, n16(k1.bad));
    st!(dma_u1_err, n8(k1.err));
    st!(dma_u2_recv, n16(k2.recv));
    st!(dma_u2_bad, n16(k2.bad));
    st!(dma_u2_err, n8(k2.err));
    let timeouts = t0.timeouts() as usize + t1.timeouts() as usize + t2.timeouts() as usize;
    st!(dma_tx_timeouts, n8(timeouts));

    [&k0, &k1, &k2]
        .iter()
        .all(|k| k.recv == DMA_N && k.bad == 0 && k.err == 0)
        && timeouts == 0
}
