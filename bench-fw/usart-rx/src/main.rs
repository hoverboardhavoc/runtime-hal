//! On-silicon Gate B validator for the DMA-ring USART RX (G-DMA-UART), scenarios S2-S5.
//!
//! ONE binary for the F103 + F130 bench pair: the chip is DETECTED at runtime and the **role** follows
//! the family, F10x = master (orchestrator/sender), F1x0 = slave (the primary DMA-ring receiver).
//! USART1 PA2 (TX) / PA3 (RX) at 115200 8N1, cross-wired; PB9 (buzzer/LED) is the human-visible blip.
//! Receivers use [`RingBufferedRx`] (circular DMA + IDLE). Busy-spin only, NEVER `wfi` (a bare `wfi`
//! with `DBG_CTL0 = 0` locks SWD re-attach on the GD32F130).
//!
//! The slave runs ONE polled measurement first, then is purely COMMAND-DRIVEN: it reads IDLE-delimited
//! frames and dispatches on the first byte, so the protocol is self-clocked by the data (robust to boot
//! skew) rather than timed lockstep. Scenarios:
//!   S3-polled : the slave's FIRST action - POLLED-receive the master's opening stream with a jitter
//!               consumer (keeps up on average, stalls on spikes -> drops during spikes), the baseline.
//!   S2        : `CMD_S2` frame -> slave DMA-ring-receives a 16-byte frame; `CMD_REQ` -> slave sends a
//!               frame so the MASTER DMA-ring-receives it (covers the F10x Ch5 / separate-IRQ path).
//!   S3-DMA    : `CMD_S3` marker, then the master streams `S3_TOTAL` (>> the buffer) -> the slave
//!               DMA-ring-receives it with the SAME jitter consumer; the small ring absorbs each spike,
//!               so it drops nothing where polling did - the evidence the throughput gap is closed.
//!   S4        : `CMD_S4` marker, then 4 variable-length frames (3,7,1,31) each + gap -> the slave reads
//!               exactly one frame per IDLE boundary.
//!   S5        : `CMD_S5_0..3` marker, then a stream (>> every swept size) -> the slave re-arms DMA-ring
//!               at `S5_SIZES[idx]` and counts lap-overruns; the curve + the smallest jitter-absorbing
//!               floor (expected << the stream size).
//!
//! Result is a fixed RAM block at [`RESULT_ADDR`] (the reserved RAM tail), `magic` written LAST.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::{
    clock,
    clock::ClockConfig,
    descriptor::ClockPath,
    detect_chip,
    irq::{install, RamVectorTable, MAX_VECTORS},
    Chip, PeriphLabel, RingBufferedRx, Usart,
};

// --- protocol facts / tunables ----------------------------------------------------------------

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const PB9: u8 = 9;
const S2_LEN: usize = 16;
/// S3/S5 continuous stream length: **much larger than the DMA buffer** (the spec's ~4 KB regime), so
/// the buffer cannot hold the whole message - it only absorbs jitter. This is what makes S3 evidence
/// that the gap is closed, not just that DMA holds one message.
const S3_TOTAL: usize = 4096;
/// The physical DMA ring buffer (and the largest S5 sweep size): far smaller than `S3_TOTAL`.
const DMA_CAP: usize = 256;
/// S5 DMA buffer sizes to sweep (bytes), all `<< S3_TOTAL` and bracketing the jitter peak (~19 bytes):
/// the small ones overrun, the larger ones do not, so the floor is the smallest that absorbs the
/// consumer's jitter (NOT the message size).
const S5_SIZES: [u16; 4] = [8, 16, 32, 64];
/// Loss count at/below which a sweep size is taken to absorb the jitter. With the library wrap-boundary
/// fix in place a sized buffer reports EXACTLY 0 real-loss bytes (the earlier ~1 event/run transient
/// was the spurious wrap-boundary overrun, now eliminated), so this is strict zero; undersized buffers
/// lose tens of bytes (genuine capacity laps), so the floor is unambiguous.
const S5_NOISE: usize = 0;

// The consumer model: keeps up with the line ON AVERAGE (average work/byte < the ~87 us byte period at
// 115200), but has a periodic latency SPIKE (a burst of processing every `JITTER_PERIOD` bytes). The
// DMA ring buffers the bytes that arrive during a spike; the FIFO-less polled path cannot, so it drops
// during spikes while DMA does not. This is the realistic jitter the DMA mode tolerates.
/// Steady per-byte work (~2.8 us at 72 MHz, far under the 87 us byte period): the consumer drains with
/// a large margin between spikes, so the buffer occupancy is just the spike accumulation (not creep).
const WORK_PER_BYTE: u32 = 200;
/// A processing spike every this many consumed bytes. Infrequent enough that its amortized cost
/// (`JITTER_SPIKE / JITTER_PERIOD` ~= 6.5 us/byte) leaves a large average margin, so the slave fully
/// drains between spikes (never piling up beyond one spike) and the buffer occupancy is the
/// deterministic per-spike peak.
const JITTER_PERIOD: u32 = 256;
/// Spike duration (~1.67 ms at 72 MHz, ~19 byte-times): the bytes that arrive during one spike must be
/// buffered, so the buffer floor lands near 19 - i.e. between the 16 and 32 sweep points, << the 4 KB
/// stream. The polled path (no buffer) drops these on every spike.
const JITTER_SPIKE: u32 = 120_000;

// Command bytes (first byte of a master->slave control frame).
const CMD_S2: u8 = 0xA2; // [CMD_S2, 0..16): slave records S2
const CMD_REQ: u8 = 0xE0; // [CMD_REQ]: slave sends a 16-byte frame (for the master's S2 receive)
const CMD_S3: u8 = 0xB1; // [CMD_S3] then a burst: slave DMA-receives it
const CMD_S4: u8 = 0xC4; // [CMD_S4] then 4 frames: slave records their lengths
const CMD_S5_0: u8 = 0xD0; // [CMD_S5_0+idx] then a burst: slave re-arms at S5_SIZES[idx]
const READY: u8 = 0x5A; // slave->master: "my DMA-ring is armed + settled for this S5 size, stream now"

// Busy-spin budgets / gaps (cycle-based delays for the sender; iteration-based for receive timeouts).
const STARTUP: u32 = 72_000_000; // ~1 s before the master's opening stream (covers reset skew)
const BIG_GAP: u32 = 40_000_000; // ~0.55 s after the polled stream (slave finishes polled + arms DMA)
const CMD_GAP: u32 = 900_000; // ~12 ms after each command/frame (the IDLE boundary)
const PHASE_GAP: u32 = 18_000_000; // ~0.25 s after a stream burst
const RX_ITERS: u32 = 8_000_000; // per-receive empty-poll timeout (~hundreds of ms)
const STREAM_GAP_ITERS: u32 = 200_000; // empty-poll backstop for end-of-stream (IDLE is primary for DMA)

// --- the SWD-readable result block ------------------------------------------------------------

#[repr(C)]
struct Results {
    /// 0x4732_5842 ("G2XB"), written LAST = the run completed.
    magic: u32,
    /// 1 = master (F10x), 2 = slave (F1x0).
    role: u8,
    /// S2: matched bytes of the received 16-byte frame.
    s2_match: u8,
    /// S2: frame length received (slave sees 17 = CMD + 16; master sees 16 = the plain response).
    s2_len: u8,
    /// S2: bit0 = IDLE seen, bit1 = no overrun.
    s2_idle: u8,
    /// S3 burst length (expected count).
    s3_total: u16,
    /// S3 POLLED receive count (slave): what the loaded polled consumer kept up with.
    s3_polled_recv: u16,
    /// S3 DMA-ring receive count (slave): should equal `s3_total`.
    s3_dma_recv: u16,
    /// S3 DMA-ring overruns (slave): should be 0.
    s3_dma_overrun: u16,
    /// S4 per-frame received lengths (expect 3, 7, 1, 31).
    s4_lens: [u8; 4],
    /// S4: 1 if all four frames returned exactly one frame each at the right lengths.
    s4_ok: u8,
    /// S5 swept buffer sizes.
    s5_size: [u16; 4],
    /// S5 lap-overrun count per buffer size.
    s5_overrun: [u16; 4],
    /// S5 smallest zero-overrun buffer size (floor); 0 if none.
    s5_floor: u16,
    /// Non-zero if detection failed.
    detect_err: u8,
}

const MAGIC: u32 = 0x4732_5842;
const RESULT_ADDR: u32 = 0x2000_1F00;

const INIT: Results = Results {
    magic: 0,
    role: 0,
    s2_match: 0,
    s2_len: 0,
    s2_idle: 0,
    s3_total: S3_TOTAL as u16,
    s3_polled_recv: 0,
    s3_dma_recv: 0,
    s3_dma_overrun: 0,
    s4_lens: [0; 4],
    s4_ok: 0,
    s5_size: S5_SIZES,
    s5_overrun: [0; 4],
    s5_floor: 0,
    detect_err: 0,
};

/// The `'static` DMA ring buffer (`DMA_CAP` bytes, far smaller than the S3 stream). S5 re-arms over
/// sub-slices of it; S3 streams `S3_TOTAL` >> `DMA_CAP` through it, so the buffer only absorbs jitter.
static mut DMA_BUF: [u8; DMA_CAP] = [0; DMA_CAP];
/// The owned RAM vector table (DMA-ring needs the DMA + USART IRQs routed; install flips VTOR first).
static mut VECTORS: RamVectorTable = RamVectorTable {
    slots: [0; MAX_VECTORS],
};

// --- result-block writer (defined before use) -------------------------------------------------

#[inline]
fn result_ptr() -> *mut Results {
    RESULT_ADDR as *mut Results
}

macro_rules! store {
    ($field:ident, $val:expr) => {{
        // SAFETY: single-threaded firmware; the only writer is this code path, reads are external (SWD).
        unsafe {
            let p = result_ptr();
            core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
        }
    }};
}

// --- entry ------------------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single writer.
    unsafe { core::ptr::write_volatile(result_ptr(), INIT) };

    let chip: Chip = match detect_chip() {
        Ok(c) => c,
        Err(_) => {
            store!(detect_err, 1);
            halt();
        }
    };
    if clock::configure_tree(&chip, &CLOCK).is_err() {
        halt();
    }
    // SAFETY: RAM init done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
    unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };

    let mut led = match chip.output_pin(PeriphLabel::Gpiob, PB9) {
        Ok(p) => p,
        Err(_) => halt(),
    };
    let _ = led.set_low();

    let gpioa = match chip.gpioa() {
        Ok(p) => p.split(),
        Err(_) => halt(),
    };
    let usart = match Usart::new(
        &chip,
        &CLOCK,
        PeriphLabel::Usart1,
        (gpioa.pa2, gpioa.pa3),
        115_200,
    ) {
        Ok(u) => u,
        Err(_) => halt(),
    };
    // SAFETY: enabling interrupts after the table is installed; handlers are registered by each `new`.
    unsafe { cortex_m::interrupt::enable() };

    match chip.clock() {
        ClockPath::F10xRcc => run_master(&chip, usart, &mut led),
        ClockPath::F1x0Rcu => run_slave(&chip, usart, &mut led),
    }

    write_magic();
    loop {
        cortex_m::asm::nop();
    }
}

// --- master (F10x): orchestrator / sender + the S2 master-receive -----------------------------

fn run_master(chip: &Chip, usart: Usart, led: &mut impl OutputPin) {
    store!(role, 1);
    delay(STARTUP);

    // S3-polled: the opening stream for the slave's polled receive.
    stream(&usart, S3_TOTAL);
    delay(BIG_GAP);

    // Arm the master as a DMA-ring receiver, persistent for the rest of the run: used both for the S2
    // master-receive (CMD_REQ response) and the S5 READY handshake. Covers the F10x Ch5 / separate-IRQ
    // DMA receive path.
    let mut mrx = match RingBufferedRx::new(chip, usart, dma_subslice(DMA_CAP)) {
        Ok(r) => r,
        Err(_) => return,
    };

    // S2 master-receive: request a frame from the slave with retries (the slave responds to CMD_REQ).
    for _ in 0..40 {
        send_cmd(&usart, CMD_REQ);
        let f = recv_frame(&mut mrx);
        if f.len >= S2_LEN {
            let m = count_incrementing(&f.bytes, 0, S2_LEN);
            store!(s2_match, m);
            store!(s2_len, f.len as u8);
            store!(s2_idle, f.idle as u8 | ((f.overrun == 0) as u8) << 1);
            if m as usize == S2_LEN && f.idle {
                pulse(led);
            }
            break;
        }
    }
    delay(PHASE_GAP);

    // S2 slave-receive: send CMD_S2 + the 16-byte frame, repeated so the slave surely catches one.
    for _ in 0..6 {
        send_s2_frame(&usart);
        delay(CMD_GAP);
    }
    delay(PHASE_GAP);

    // S3-DMA: marker, then stream the burst.
    send_cmd(&usart, CMD_S3);
    delay(CMD_GAP);
    stream(&usart, S3_TOTAL);
    delay(PHASE_GAP);

    // S4: marker, then four variable-length frames each + gap.
    send_cmd(&usart, CMD_S4);
    delay(CMD_GAP);
    for &n in &[3usize, 7, 1, 31] {
        send_incrementing(&usart, n);
        delay(CMD_GAP);
    }
    delay(PHASE_GAP);

    // S5: per size, a marker, then WAIT for the slave's READY (it re-armed + settled the channel) before
    // streaming - so no byte arrives before the re-armed channel is live, which would inject a
    // startup-transient overrun unrelated to buffer capacity.
    for idx in 0..S5_SIZES.len() {
        send_cmd(&usart, CMD_S5_0 + idx as u8);
        // Wait for the slave's READY before streaming. Plain retries (no marker re-send: re-sending
        // after the slave re-armed would land in its stream as a data byte). Framed commands are
        // reliable, so one marker reaches the slave; the READY confirms the channel is live + settled.
        for _ in 0..8 {
            let f = recv_frame(&mut mrx);
            if f.len >= 1 && f.bytes[0] == READY {
                break;
            }
        }
        delay(CMD_GAP);
        stream(&usart, S3_TOTAL);
        delay(PHASE_GAP);
    }
}

// --- slave (F1x0): polled baseline, then a command-driven DMA-ring receiver -------------------

fn run_slave(chip: &Chip, usart: Usart, led: &mut impl OutputPin) {
    store!(role, 2);

    // S3-polled (first action, before DMA is armed): POLLED-receive the opening stream with the jitter
    // consumer. Retry if a run caught only a stray byte (a transient at the master's USART setup, then
    // the inter-phase gap, can falsely "end" a run): the real stream is large, so a tiny run is junk.
    let mut polled = 0usize;
    for _ in 0..6 {
        polled = recv_stream_polled(&usart);
        if polled > 64 {
            break;
        }
    }
    store!(s3_polled_recv, polled.min(u16::MAX as usize) as u16);

    // Arm DMA-ring for command frames (`usart` is Copy: the slave keeps its handle for TX + re-arming).
    let mut rx = match RingBufferedRx::new(chip, usart, dma_subslice(DMA_CAP)) {
        Ok(r) => r,
        Err(_) => return,
    };

    // Command loop: read one frame, dispatch on the first byte. Bounded so a missed command cannot hang
    // it; breaks once the last S5 size is done. PB9 is pulsed ONCE after the loop (a blocking pulse
    // mid-loop would coalesce later frames). The S5 curve/floor are tracked in locals, stored at the end.
    let mut s2_pass = false;
    let mut s5_ovr = [0u16; 4];
    let mut s5_floor = 0u16;
    for _ in 0..80 {
        let f = recv_frame(&mut rx);
        if f.len == 0 {
            continue; // timeout, retry
        }
        match f.bytes[0] {
            CMD_S2 => {
                let m = count_incrementing(&f.bytes, 1, S2_LEN);
                store!(s2_match, m);
                store!(s2_len, f.len as u8);
                store!(s2_idle, f.idle as u8 | ((f.overrun == 0) as u8) << 1);
                if m as usize == S2_LEN && f.idle {
                    s2_pass = true;
                }
            }
            CMD_REQ => send_incrementing(&usart, S2_LEN),
            CMD_S3 => {
                // Re-arm a FRESH DMA-ring for the stream (a clean cursor + buffer): reusing the
                // command-loop receiver carries residual frame state that perturbs the stream math.
                if let Ok(mut r) = RingBufferedRx::new(chip, usart, dma_subslice(DMA_CAP)) {
                    let (recv, ovr) = recv_stream_dma(&mut r);
                    // `overrun == 0` is the reliable pass signal (the library's lap detection); cap the
                    // count at what was sent (a lap-recovery resync can transiently re-count a few bytes).
                    store!(s3_dma_recv, recv.min(S3_TOTAL) as u16);
                    store!(s3_dma_overrun, ovr.min(u16::MAX as usize) as u16);
                }
                rx = match RingBufferedRx::new(chip, usart, dma_subslice(DMA_CAP)) {
                    Ok(r) => r,
                    Err(_) => return,
                };
            }
            CMD_S4 => {
                let mut lens = [0u8; 4];
                let mut ok = true;
                let expect = [3u8, 7, 1, 31];
                for (slot, &want) in lens.iter_mut().zip(expect.iter()) {
                    let fr = recv_frame(&mut rx);
                    *slot = fr.len as u8;
                    if fr.len as u8 != want || !fr.idle {
                        ok = false;
                    }
                }
                store!(s4_lens, lens);
                store!(s4_ok, ok as u8);
            }
            c @ CMD_S5_0..=0xD3 => {
                let idx = (c - CMD_S5_0) as usize;
                let size = S5_SIZES[idx];
                // Re-arm the DMA-ring at this size (a sub-slice of the one buffer). Signal READY only
                // AFTER the channel is armed + a brief settle, so the master streams into a live channel
                // (no startup-transient overrun). Then receive + count - all overruns are now real
                // buffer-capacity laps.
                if let Ok(mut r) = RingBufferedRx::new(chip, usart, dma_subslice(size as usize)) {
                    delay(CMD_GAP); // let the freshly-armed channel + IDLE settle before inviting bytes
                    let _ = r.take_idle(); // clear any boundary latched during re-arm
                    send_cmd(&usart, READY);
                    let (_recv, ovr) = recv_stream_dma(&mut r);
                    s5_ovr[idx] = ovr.min(u16::MAX as usize) as u16;
                    store!(s5_overrun, s5_ovr);
                    // The floor is the smallest size whose loss is at the noise floor (<= S5_NOISE):
                    // capacity laps at an undersized buffer are many (tens), cleanly separated from the
                    // rare ~1-event/run transient that appears at the peak edge independent of capacity.
                    if ovr <= S5_NOISE && (s5_floor == 0 || size < s5_floor) {
                        s5_floor = size;
                        store!(s5_floor, s5_floor);
                    }
                }
                // Re-arm at the full buffer for any subsequent command frames.
                rx = match RingBufferedRx::new(chip, usart, dma_subslice(DMA_CAP)) {
                    Ok(r) => r,
                    Err(_) => return,
                };
                if idx + 1 == S5_SIZES.len() {
                    break; // last S5 size done: the run is complete
                }
            }
            _ => {}
        }
    }

    if s2_pass {
        pulse(led); // human-visible: the DMA-ring receive chain worked
    }
}

// --- frame / stream helpers -------------------------------------------------------------------

struct Frame {
    len: usize,
    idle: bool,
    overrun: u8,
    bytes: [u8; 64],
}

/// Count bytes `bytes[offset + i] == i` for `i in 0..n` (the incrementing-pattern match).
fn count_incrementing(bytes: &[u8; 64], offset: usize, n: usize) -> u8 {
    let mut m = 0u8;
    for i in 0..n {
        if offset + i < bytes.len() && bytes[offset + i] == i as u8 {
            m += 1;
        }
    }
    m
}

/// Receive one IDLE-delimited frame via DMA-ring: drain until the boundary. `read` never clears the
/// IDLE latch (only `take_idle` does), so the boundary is consumed exactly once with no race.
fn recv_frame(rx: &mut RingBufferedRx) -> Frame {
    let mut bytes = [0u8; 64];
    let mut len = 0usize;
    let mut idle = false;
    let mut overrun = 0u8;
    let mut empty = 0u32;
    loop {
        match rx.read(&mut bytes[len..]) {
            Ok(0) => {}
            Ok(n) => {
                len += n;
                empty = 0;
            }
            Err(_) => overrun = overrun.saturating_add(1),
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
    Frame {
        len,
        idle,
        overrun,
        bytes,
    }
}

/// The consumer model: a steady per-byte cost plus a periodic latency SPIKE. On average it beats the
/// line, but during a spike it stalls - the polled path then drops (FIFO-less), the DMA path buffers.
fn consume(processed: &mut u32) {
    cortex_m::asm::delay(WORK_PER_BYTE);
    *processed = processed.wrapping_add(1);
    if *processed % JITTER_PERIOD == 0 {
        cortex_m::asm::delay(JITTER_SPIKE);
    }
}

/// POLLED receive of a stream with the jitter consumer, until a gap once data has started. The polled
/// path keeps up between spikes but drops during each spike (no buffer), so `recv < stream`.
fn recv_stream_polled(usart: &Usart) -> usize {
    let mut recv = 0usize;
    let mut empty = 0u32;
    let mut started = false;
    let mut processed = 0u32;
    let mut budget = RX_ITERS.saturating_mul(8);
    loop {
        match usart.try_read_byte() {
            Ok(Some(_)) => {
                recv += 1;
                started = true;
                empty = 0;
                consume(&mut processed);
            }
            _ => {
                empty += 1;
                if started && empty > STREAM_GAP_ITERS {
                    break;
                }
            }
        }
        budget = budget.saturating_sub(1);
        if budget == 0 {
            break;
        }
    }
    recv
}

/// State for measuring REAL data loss against the master's known stream (bytes = position & 0xFF).
struct SeqCheck {
    /// The next byte value expected if the stream is contiguous.
    expected: u8,
    /// Whether the sequence baseline has been set (first byte seen).
    started: bool,
    /// Count of FORWARD gaps = real data loss (the DMA buffer lapped, dropping unread bytes). A
    /// *backward* jump (the cursor was resynced backward, re-reading old bytes) is the library's rare
    /// wrap-boundary spurious overrun, NOT data loss, so it is not counted.
    loss: usize,
    /// Total bytes delivered (may exceed the sent count if a spurious overrun caused re-reads).
    recv: usize,
}

impl SeqCheck {
    fn new() -> Self {
        SeqCheck {
            expected: 0,
            started: false,
            loss: 0,
            recv: 0,
        }
    }
    /// Classify each delivered byte against the contiguous stream: a forward jump (bytes skipped) is a
    /// real loss; a backward jump (re-read) is a spurious resync and is ignored.
    fn push(&mut self, b: u8) {
        self.recv += 1;
        if !self.started {
            self.started = true;
        } else if b != self.expected {
            let forward = b.wrapping_sub(self.expected); // distance b is ahead of expected (mod 256)
            if forward != 0 && forward < 128 {
                self.loss += 1; // bytes were skipped: a real buffer-capacity lap
            }
        }
        self.expected = b.wrapping_add(1);
    }
}

/// DMA-ring receive of a stream with the SAME jitter consumer, until the IDLE boundary once data has
/// started. The small ring buffers the bytes arriving during each spike; a buffer >= the jitter peak
/// loses nothing even though the stream is far larger than the buffer. Returns (bytes delivered, real
/// loss events). "Real loss" is measured by contiguity of the known stream, so the library's rare
/// wrap-boundary spurious overrun (a backward cursor resync, no data lost) does not inflate the count.
fn recv_stream_dma(rx: &mut RingBufferedRx) -> (usize, usize) {
    // Drain in small chunks so the consumer (and its spikes) runs between frequent drains: the buffer
    // then only ever holds about one spike's worth of arrivals, not a whole batch's processing time.
    let mut scratch = [0u8; 4];
    let mut seq = SeqCheck::new();
    let mut empty = 0u32;
    let mut started = false;
    let mut processed = 0u32;
    let mut budget = RX_ITERS.saturating_mul(8);
    loop {
        match rx.read(&mut scratch) {
            Ok(0) => {}
            Ok(n) => {
                started = true;
                empty = 0;
                for &b in scratch.iter().take(n) {
                    seq.push(b);
                    consume(&mut processed);
                }
            }
            // A library overrun is not counted directly: the resulting jump in the delivered byte
            // sequence (forward = real loss, backward = spurious) is what `SeqCheck` classifies.
            Err(_) => {
                started = true;
                empty = 0;
            }
        }
        if started && rx.take_idle() {
            // The burst ended (line idle): drain the rest FULLY (the DMA is no longer writing, so
            // `read` returns 0 once the cursor catches the frozen write index).
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
        empty += 1;
        if started && empty > STREAM_GAP_ITERS {
            break;
        }
        budget = budget.saturating_sub(1);
        if budget == 0 {
            break;
        }
        cortex_m::asm::nop();
    }
    (seq.recv, seq.loss)
}

/// Stream `n` bytes back-to-back (the throughput burst): value = index & 0xFF.
fn stream(usart: &Usart, n: usize) {
    for i in 0..n {
        usart.write_byte((i & 0xFF) as u8);
    }
}

/// Send a single-byte command frame, then a short idle gap (the IDLE boundary).
fn send_cmd(usart: &Usart, cmd: u8) {
    usart.write_byte(cmd);
    delay(CMD_GAP);
}

/// Send the S2 frame: `[CMD_S2, 0, 1, .., 15]`, then a gap.
fn send_s2_frame(usart: &Usart) {
    usart.write_byte(CMD_S2);
    for i in 0..S2_LEN {
        usart.write_byte(i as u8);
    }
    delay(CMD_GAP);
}

/// Send an `n`-byte incrementing frame (`0..n`), then a short idle gap.
fn send_incrementing(usart: &Usart, n: usize) {
    for i in 0..n {
        usart.write_byte(i as u8);
    }
    delay(CMD_GAP);
}

/// Pulse PB9 (~100 ms): the human-visible "received correctly".
fn pulse(led: &mut impl OutputPin) {
    let _ = led.set_high();
    cortex_m::asm::delay(7_200_000);
    let _ = led.set_low();
}

#[inline]
fn delay(cycles: u32) {
    cortex_m::asm::delay(cycles);
}

/// A `'static` sub-slice of the single DMA buffer of length `n` (`n <= S3_TOTAL`), built via
/// `from_raw_parts_mut` to avoid an indexing autoref on the raw pointer. One receiver is active at a
/// time; the channel is disabled by the next `RingBufferedRx::new` before re-use.
fn dma_subslice(n: usize) -> &'static mut [u8] {
    // SAFETY: `n <= S3_TOTAL` (DMA_BUF size); 'static; single active receiver per phase.
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF) as *mut u8, n) }
}

fn write_magic() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    store!(magic, MAGIC);
}

/// Halt forever on an unrecoverable bring-up error. Busy-spin (NEVER wfi: GD32F130 SWD-lockout rule).
fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
