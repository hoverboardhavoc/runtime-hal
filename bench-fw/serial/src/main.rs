//! On-silicon validator for the `embedded-io` serial adapters (`specs/serial-adapters.md`,
//! section 5): the F103+F130 pair over the proven USART1 PA2/PA3 cross-wire, 115200 8N1, the
//! slice-3/4 pair topology. Role-detected (F10x = master, F1x0 = slave); every data-path byte
//! crosses the `embedded-io` traits (`Write` on the sender, `Read`/`ReadReady` on the receiver);
//! no direct backend calls on the data path. Baseline = the slice-3 pair numbers (4096 recv /
//! 0 loss / 0 err into the 256 B DMA ring).
//!
//! Stages (master streams, slave receives, then reverse):
//!   1. slave via `SplitSerial<RingBufferedRx>`: 4096 @ 115200 -> expect 4096 / 0 / 0.
//!   2. slave via `SplitSerial<BufferedRx>` (the interrupt adapter's silicon proof on this same
//!      topology): 4096 @ 115200 -> expect 4096 / 0 / 0.
//!   3. slave via `PolledSerial` (rejoined whole handle): 256 @ 115200 -> expect 256 / 0 / 0.
//!   4. reverse: master via `SplitSerial<RingBufferedRx>` (Ch5); the slave streams 4096 through
//!      its `PolledSerial::write` -> expect 4096 / 0 / 0.
//!
//! Result is a fixed RAM block at [`RESULT_ADDR`] (the reserved 8 KiB-safe tail), `magic` written
//! LAST per board. Busy-spin forever, NEVER `wfi` (GD32 SWD-lockout rule).

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embedded_io::{Read, ReadReady, Write};
use heapless::spsc::Queue;
use panic_halt as _;

use runtime_hal::{
    clock,
    clock::ClockConfig,
    descriptor::ClockPath,
    detect_chip,
    irq::{install, RamVectorTable, MAX_VECTORS},
    BufferedRx, Chip, PeriphLabel, PolledSerial, RingBufferedRx, SplitSerial, Usart,
};

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const BAUD: u32 = 115_200;
/// The DMA ring size: the slice-3 baseline geometry.
const DMA_CAP: usize = 256;
/// The interrupt-path SPSC ring capacity word (255 usable bytes: the same order as the DMA ring).
const RING_N: usize = 256;
/// Stages 1/2/4 stream length (the baseline stream) and stage 3's polled frame.
const N_STREAM: usize = 4096;
const N_POLLED: usize = 256;
/// Receiver empty-iteration budget once a stage's bytes are flowing (the stage-end gap bound).
const GAP_ITERS: u32 = 20_000_000;

const RESULT_ADDR: u32 = 0x2000_1F00;
const MAGIC: u32 = 0x5345_5242; // "SERB"

#[repr(C)]
struct Results {
    /// Written LAST = this board's run completed.
    magic: u32,
    /// 1 = F10x (master), 2 = F1x0 (slave).
    role: u8,
    /// Non-zero if `detect_chip` failed.
    detect_err: u8,
    /// Slave stage 1: `SplitSerial<RingBufferedRx>` @115200: recv / SeqCheck loss / adapter
    /// `line_errors`.
    p1_recv: u16,
    p1_loss: u16,
    p1_err: u16,
    /// Slave stage 2: `SplitSerial<BufferedRx>`.
    p2_recv: u16,
    p2_loss: u16,
    p2_err: u16,
    /// Slave stage 3: `PolledSerial` (rejoined whole handle).
    p3_recv: u16,
    p3_loss: u16,
    p3_err: u16,
    /// Master stage 4: `SplitSerial<RingBufferedRx>` (Ch5), the reverse stream.
    r_recv: u16,
    r_loss: u16,
    r_err: u16,
    /// This board's receiver stages at their expected counts with zero loss/errors.
    pass: u8,
    /// Sender stages: bytes streamed (a record of what was driven).
    sent1: u16,
    sent2: u16,
    sent3: u16,
    sent_rev: u16,
}

static mut VECTORS: RamVectorTable = RamVectorTable {
    slots: [0; MAX_VECTORS],
};
static mut DMA_BUF: [u8; DMA_CAP] = [0; DMA_CAP];
static mut RING: Queue<u8, RING_N> = Queue::new();

fn dma_buf() -> &'static mut [u8] {
    // SAFETY: 'static; one RingBufferedRx is active at a time (stages are sequential).
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF) as *mut u8, DMA_CAP) }
}

macro_rules! st {
    ($field:ident, $val:expr) => {{
        // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single-threaded, single writer; reads
        // are external (SWD).
        unsafe {
            let p = RESULT_ADDR as *mut Results;
            core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
        }
    }};
}

fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

/// Stream `n` pattern bytes (`(start + i) & 0xFF`) through an adapter's `Write` in 64-byte chunks.
fn stream<S: Write>(s: &mut S, n: usize) {
    let mut chunk = [0u8; 64];
    let mut sent = 0usize;
    while sent < n {
        let k = chunk.len().min(n - sent);
        for (i, b) in chunk.iter_mut().enumerate().take(k) {
            *b = ((sent + i) & 0xFF) as u8;
        }
        let _ = s.write_all(&chunk[..k]);
        sent += k;
    }
    let _ = s.flush();
}

/// Contiguity checker for the known stream (value = position & 0xFF): counts delivered bytes and
/// FORWARD gaps (real loss), self-calibrating from the first byte.
struct SeqCheck {
    expected: u8,
    started: bool,
    loss: usize,
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
    fn push(&mut self, b: u8) {
        self.recv += 1;
        if !self.started {
            self.started = true;
        } else if b != self.expected {
            let forward = b.wrapping_sub(self.expected);
            if forward != 0 && forward < 128 {
                self.loss += 1;
            }
        }
        self.expected = b.wrapping_add(1);
    }
}

/// Drain one stage THROUGH the `embedded-io` traits: wait unbounded for the stage's first byte
/// (this board is armed before the sender starts), gate every read on `ReadReady`, then a bounded
/// gap ends the stage. Returns `(recv, loss)`.
fn drain_stage<S: Read + ReadReady>(s: &mut S, expect: usize) -> (u16, u16) {
    let mut seq = SeqCheck::new();
    let mut empty = 0u32;
    let mut scratch = [0u8; 32];
    loop {
        if s.read_ready().unwrap_or(false) {
            let n = s.read(&mut scratch).unwrap_or(0);
            for &b in scratch.iter().take(n) {
                seq.push(b);
            }
            if n > 0 {
                empty = 0;
            }
        }
        if seq.recv >= expect {
            break;
        }
        if seq.recv > 0 {
            empty += 1;
            if empty > GAP_ITERS {
                break; // the stream stopped short: record what arrived
            }
        }
        cortex_m::asm::nop();
    }
    // Tail drain (a few in-flight bytes after the count target).
    while s.read_ready().unwrap_or(false) {
        let n = s.read(&mut scratch).unwrap_or(0);
        if n == 0 {
            break;
        }
        for &b in scratch.iter().take(n) {
            seq.push(b);
        }
    }
    (
        seq.recv.min(u16::MAX as usize) as u16,
        seq.loss.min(u16::MAX as usize) as u16,
    )
}

#[entry]
fn main() -> ! {
    // SAFETY: RESULT_ADDR is reserved RAM (memory.x); single writer.
    unsafe { core::ptr::write_bytes(RESULT_ADDR as *mut u8, 0, core::mem::size_of::<Results>()) };

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

    // Both roles arm a DMA/interrupt receiver at some point: RAM vector table + IRQs up-front.
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

/// F10x master: send stages 1-3 through `PolledSerial::write`, then become the stage-4 receiver
/// through `SplitSerial<RingBufferedRx>` (Ch5).
fn run_master(chip: &Chip, usart1: Usart) -> ! {
    let mut serial = PolledSerial::from_usart(usart1);

    // Boot settle: the slave is flashed + armed FIRST (it waits unbounded for the first byte), so
    // a fixed delay only has to cover this board's own reset.
    cortex_m::asm::delay(72_000_000 * 3);

    stream(&mut serial, N_STREAM); // stage 1 -> slave's SplitSerial<RingBufferedRx>
    st!(sent1, N_STREAM as u16);
    cortex_m::asm::delay(72_000_000 + 72_000_000 / 2); // slave records + re-arms (~instant); 1.5 s margin

    stream(&mut serial, N_STREAM); // stage 2 -> slave's SplitSerial<BufferedRx>
    st!(sent2, N_STREAM as u16);
    cortex_m::asm::delay(72_000_000 + 72_000_000 / 2);

    stream(&mut serial, N_POLLED); // stage 3 -> slave's PolledSerial
    st!(sent3, N_POLLED as u16);

    // Stage 4: become the receiver. The whole handle comes back out of the adapter, splits, and
    // the DMA ring goes behind a fresh SplitSerial. The slave waits ~3 s after its stage 3 before
    // streaming, which covers this arm-up many times over.
    let (tx, rx) = serial.into_usart().split();
    let ring = match RingBufferedRx::new(chip, rx, PeriphLabel::Usart1, dma_buf()) {
        Ok(r) => r,
        Err(_) => halt(),
    };
    let mut serial = SplitSerial::new(tx, ring);
    let (recv, loss) = drain_stage(&mut serial, N_STREAM);
    st!(r_recv, recv);
    st!(r_loss, loss);
    st!(r_err, serial.line_errors());

    let pass = recv as usize == N_STREAM && loss == 0 && serial.line_errors() == 0;
    st!(pass, pass as u8);

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    st!(magic, MAGIC);
    halt();
}

/// F1x0 slave: receive stage 1 through `SplitSerial<RingBufferedRx>` (Ch4 grouped), stage 2
/// through `SplitSerial<BufferedRx>`, stage 3 through `PolledSerial` (rejoined), then stream the
/// stage-4 reverse through the polled adapter's `Write`.
fn run_slave(chip: &Chip, usart1: Usart) -> ! {
    let (tx, rx) = usart1.split();

    // Stage 1: the DMA adapter.
    let ring = match RingBufferedRx::new(chip, rx, PeriphLabel::Usart1, dma_buf()) {
        Ok(r) => r,
        Err(_) => halt(),
    };
    let mut s1 = SplitSerial::new(tx, ring);
    let (recv, loss) = drain_stage(&mut s1, N_STREAM);
    st!(p1_recv, recv);
    st!(p1_loss, loss);
    st!(p1_err, s1.line_errors());
    let p1_ok = recv as usize == N_STREAM && loss == 0 && s1.line_errors() == 0;

    // Stage 2: the interrupt adapter (release the DMA ring, arm BufferedRx on the same half).
    let (tx, ring) = s1.into_parts();
    let rx = ring.release();
    // SAFETY: RING is a 'static, used only as this receiver's SPSC buffer.
    let buffered = match BufferedRx::new(chip, rx, PeriphLabel::Usart1, unsafe {
        &mut *addr_of_mut!(RING)
    }) {
        Ok(b) => b,
        Err(_) => halt(),
    };
    let mut s2 = SplitSerial::new(tx, buffered);
    let (recv, loss) = drain_stage(&mut s2, N_STREAM);
    st!(p2_recv, recv);
    st!(p2_loss, loss);
    st!(p2_err, s2.line_errors());
    let p2_ok = recv as usize == N_STREAM && loss == 0 && s2.line_errors() == 0;

    // Stage 3: the polled adapter over the rejoined whole handle.
    let (tx, buffered) = s2.into_parts();
    let rx = buffered.release();
    let mut s3 = PolledSerial::from_usart(Usart::rejoin(tx, rx));
    let (recv, loss) = drain_stage(&mut s3, N_POLLED);
    st!(p3_recv, recv);
    st!(p3_loss, loss);
    st!(p3_err, s3.line_errors());
    let p3_ok = recv as usize == N_POLLED && loss == 0 && s3.line_errors() == 0;

    st!(pass, (p1_ok && p2_ok && p3_ok) as u8);

    // Stage 4: give the master ~3 s to arm its Ch5 receiver, then stream the reverse through this
    // adapter's Write.
    cortex_m::asm::delay(72_000_000 * 3);
    stream(&mut s3, N_STREAM);
    st!(sent_rev, N_STREAM as u16);

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    st!(magic, MAGIC);
    halt();
}
