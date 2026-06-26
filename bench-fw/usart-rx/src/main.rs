//! On-silicon S0 + S1 + S2 validator for the multi-instance USART RX spec
//! (`specs/uart-rx-multi-instance.md`, Acceptance S0, S1, S2).
//!
//! Topology is a SINGLE F103 bluepill with an intra-board cross-wire (NOT the prior F103+F130 pair):
//!   - `PA2`  (HAL `PeriphLabel::Usart1` TX, GD USART1 block @0x4000_4400) -> `PB11` (`Usart2` RX,
//!     GD USART2 block @0x4000_4800, F10x-only).
//!   - `PB10` (`Usart2` TX) -> `PA3` (`Usart1` RX).
//! Both instances are on APB1. Each USART is both a sender and a receiver.
//!
//! **S0 (polled, zero new library code):** lockstep polled loopback each direction.
//! **S1 (interrupt-buffered `BufferedRx`):** the module receives via its own slot + vector (IRQ 39),
//! USART1 regresses, and both run simultaneously without cross-talk.
//! **S2 (DMA-ring `RingBufferedRx`, the DMA-path generalization):** confirms the new DMA channel
//! mapping on silicon (a wrong channel passes the write-back self-check but receives ZERO bytes):
//!   1. module-via-RingBufferedRx: USART1 streams on PA2 -> the module `RingBufferedRx` receives on
//!      PB11 through GD `USART2_RX` = DMA0 Channel 2 + its own vector (IRQ 13). Record recv + overrun=0
//!      + IDLE + the first byte (in the module-stream range, so a channel collision is caught).
//!   2. USART1 regression: the module streams on PB10 -> USART1's `RingBufferedRx` on PA3 (Ch5/IRQ 16).
//!   3. coexistence: both `RingBufferedRx` live at once, disjoint streams, each receives only its own.
//!   4. high-rate stress: the module `RingBufferedRx` receives a stream far larger than the ring at
//!      **2.25 Mbit/s** (APB1/16, `USART_BAUD` 0x10, 0% divisor error), exercising fast wraps; single-
//!      chip self-loopback shares the TX/RX clock so even the max rate is reliable.
//!   5. one pass at the module's **9600 8N1** (deployment-representative).
//!
//! Baud is changed between S2 stages with [`Usart::bring_up`] (reprograms the USART registers without
//! re-touching the GPIO AF, which [`Usart::new`] configured once at boot).
//!
//! PB9 (LED/buzzer) pulses on a full S0+S1+S2 pass. Result is a fixed RAM block at [`RESULT_ADDR`] (the
//! reserved RAM tail), `magic` written LAST. Busy-spin forever, NEVER `wfi` (a bare `wfi` with
//! `DBG_CTL0 = 0` locks SWD re-attach on the GD32).
//!
//! The prior pair-validator (F103+F130 Gate-B `RingBufferedRx`, scenarios S2-S5) was committed at
//! `b6bcf04` ("G-DMA-UART: interrupt-buffered + DMA-ring USART RX"); recover it from git history.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use heapless::spsc::Queue;
use panic_halt as _;

use runtime_hal::{
    clock,
    clock::ClockConfig,
    descriptor::ClockPath,
    detect_chip,
    irq::{install, RamVectorTable, MAX_VECTORS},
    BufferedRx, Chip, Oversampling, PeriphLabel, RingBufferedRx, Usart, UsartConfig, UsartFrame,
};

// --- tunables / facts -------------------------------------------------------------------------

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const BAUD: u32 = 115_200;
/// S2 high-rate stress baud: APB1/16 = 36 MHz / 16 = 2.25 Mbit/s (`USART_BAUD` 0x10, 0% divisor error),
/// the F1-series USART maximum on the proven 72 MHz tree.
const BAUD_FAST: u32 = 2_250_000;
/// S2 deployment-representative baud (the BLE module's rate).
const BAUD_9600: u32 = 9_600;
/// The known frame for the module direction (USART1 TX -> module RX): incrementing `byte[i] == i`.
const FRAME: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
/// The known frame for the USART1 direction (module TX -> USART1 RX): a DISJOINT high range
/// (`0x80 + i`), so any cross-talk between the two live slots shows up as a content mismatch.
const FRAME_HI: [u8; 16] = [
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
];
/// Per-byte empty-poll budget for the S0 lockstep path (only a hang backstop: the byte is already in
/// the RX register when `write_byte` returns).
const RX_BUDGET: u32 = 1_000_000;
/// Empty-iteration budget for an S1/S2 buffered drain (the ring absorbs the burst; this bounds the wait
/// for the IDLE boundary / stragglers so a missed frame cannot hang the run).
const RX_ITERS: u32 = 2_000_000;
/// S2 moderate stream length for stages 1-3 + the 9600 pass: fits the DMA ring, so no lap.
const S2_N: usize = 64;
/// S2 high-rate stress stream length: far larger than the ring (8 wraps), so the ring only absorbs the
/// per-burst arrivals while the buffer wraps repeatedly at the max baud.
const S2_STRESS: usize = 2048;
/// S2 stress burst size (bytes streamed before each drain): `< DMA_CAP`, so the ring never laps.
const S2_BURST: usize = 64;
/// The DMA ring buffer size (per instance).
const DMA_CAP: usize = 256;
/// S2 coexistence stream start values: the module direction starts in the LOW range, the USART1
/// direction in the HIGH range, so a channel/context collision (one slot receiving the other's data)
/// shows up as an out-of-range first byte.
const COX_MOD_START: u8 = 0x10;
const COX_U1_START: u8 = 0xC0;

// --- the SWD-readable result block ------------------------------------------------------------

#[repr(C)]
struct Results {
    /// 0x5332_5242 ("S2RB"), written LAST = the run completed.
    magic: u32,
    /// 1 = F10x (master family, the bench part), 2 = F1x0. The bench is an F103, so 1 is expected.
    role: u8,
    /// Non-zero if `detect_chip` failed.
    detect_err: u8,

    // --- S0 (polled loopback) ---
    s0_a_len: u8,
    s0_a_match: u8,
    s0_b_len: u8,
    s0_b_match: u8,
    s0_pass: u8,

    // --- S1 (interrupt-buffered BufferedRx) ---
    s1_mod_len: u8,
    s1_mod_match: u8,
    s1_mod_idle: u8,
    s1_u1_len: u8,
    s1_u1_match: u8,
    s1_u1_idle: u8,
    s1_cox_mod_match: u8,
    s1_cox_u1_match: u8,
    s1_pass: u8,

    // --- S2 (DMA-ring RingBufferedRx) counts (u16) ---
    /// Stage 1: module-via-RingBufferedRx bytes received (Ch2). Expected `S2_N`.
    s2_mod_recv: u16,
    /// Stage 2: USART1 RingBufferedRx regression bytes received (Ch5). Expected `S2_N`.
    s2_u1_recv: u16,
    /// Stage 3: coexistence bytes received, module slot / USART1 slot. Expected `S2_N` each.
    s2_cox_mod_recv: u16,
    s2_cox_u1_recv: u16,
    /// Stage 4: high-rate stress bytes received contiguously / real loss events. Expected ~`S2_STRESS` / 0.
    s2_stress_recv: u16,
    s2_stress_loss: u16,
    /// Stage 4: the stress baud / 1000 (a record of what was actually run, e.g. 2250 = 2.25 Mbit/s).
    s2_stress_baud_k: u16,
    /// Stage 5: 9600 8N1 module bytes received. Expected `S2_N`.
    s2_9600_recv: u16,

    // --- S2 flags (u8) ---
    /// Stage 1: overrun count (0 expected) / IDLE-seen.
    s2_mod_ovr: u8,
    s2_mod_idle: u8,
    /// Stage 2: overrun count (0 expected).
    s2_u1_ovr: u8,
    /// Stage 5: 1 if 9600 received all `S2_N` bytes.
    s2_9600_match: u8,
    /// Stage 3: 1 if both slots received their full stream AND the right (in-range) content.
    s2_cox_ok: u8,
    /// 1 if every S2 stage passed.
    s2_pass: u8,
}

const MAGIC: u32 = 0x5332_5242;
const RESULT_ADDR: u32 = 0x2000_1F00;

const INIT: Results = Results {
    magic: 0,
    role: 0,
    detect_err: 0,
    s0_a_len: 0,
    s0_a_match: 0,
    s0_b_len: 0,
    s0_b_match: 0,
    s0_pass: 0,
    s1_mod_len: 0,
    s1_mod_match: 0,
    s1_mod_idle: 0,
    s1_u1_len: 0,
    s1_u1_match: 0,
    s1_u1_idle: 0,
    s1_cox_mod_match: 0,
    s1_cox_u1_match: 0,
    s1_pass: 0,
    s2_mod_recv: 0,
    s2_u1_recv: 0,
    s2_cox_mod_recv: 0,
    s2_cox_u1_recv: 0,
    s2_stress_recv: 0,
    s2_stress_loss: 0,
    s2_stress_baud_k: 0,
    s2_9600_recv: 0,
    s2_mod_ovr: 0,
    s2_mod_idle: 0,
    s2_u1_ovr: 0,
    s2_9600_match: 0,
    s2_cox_ok: 0,
    s2_pass: 0,
};

/// The two application-owned `'static` SPSC rings (S1 `BufferedRx`, one per instance).
static mut RING_U1: Queue<u8, 64> = Queue::new();
static mut RING_MOD: Queue<u8, 64> = Queue::new();
/// The two application-owned `'static` DMA ring buffers (S2 `RingBufferedRx`, one per instance). The
/// module + USART1 buffers are distinct so the two DMA channels write disjoint memory (coexistence).
static mut DMA_BUF_U1: [u8; DMA_CAP] = [0; DMA_CAP];
static mut DMA_BUF_MOD: [u8; DMA_CAP] = [0; DMA_CAP];
/// The owned RAM vector table (interrupt + DMA RX need the vectors routed; `install` flips VTOR).
static mut VECTORS: RamVectorTable = RamVectorTable {
    slots: [0; MAX_VECTORS],
};

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
    store!(
        role,
        match chip.clock() {
            ClockPath::F10xRcc => 1,
            ClockPath::F1x0Rcu => 2,
        }
    );
    if clock::configure_tree(&chip, &CLOCK).is_err() {
        halt();
    }

    // GPIOB carries PB9 (LED) plus the module-USART pins PB10 (TX) / PB11 (RX); GPIOA carries the
    // USART1 pins PA2 (TX) / PA3 (RX). Split each port once into its named pins.
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

    // USART1 on PA2 (TX) / PA3 (RX), 115200 8N1. This `new` configures the GPIO AF (once); later S2
    // stages reprogram only the BAUD via `bring_up`, leaving the AF untouched.
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
    // Module USART (HAL `Usart2`, the GD USART2 block) on PB10 (TX) / PB11 (RX), 115200 8N1.
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
    let s1_pass = run_s1(&chip, usart1, usart2);
    let s2_pass = run_s2(&chip, usart1, usart2);

    if s0_pass && s1_pass && s2_pass {
        pulse(&mut led); // human-visible: the polled, interrupt, and DMA second-instance paths all passed
    }

    write_magic();
    loop {
        cortex_m::asm::nop();
    }
}

// --- S0: polled loopback ----------------------------------------------------------------------

/// Lockstep polled loopback both directions; record + return whether both matched exactly.
fn run_s0(usart1: &Usart, usart2: &Usart) -> bool {
    let (a_len, a_match) = loopback(usart1, usart2);
    store!(s0_a_len, a_len);
    store!(s0_a_match, a_match);

    let (b_len, b_match) = loopback(usart2, usart1);
    store!(s0_b_len, b_len);
    store!(s0_b_match, b_match);

    let pass = a_match as usize == FRAME.len() && b_match as usize == FRAME.len();
    store!(s0_pass, pass as u8);
    pass
}

/// Send [`FRAME`] on `tx` and read it back polled on `rx`, lockstep (one byte sent then drained before
/// the next); return `(bytes received, bytes matching FRAME)`.
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

// --- S1: interrupt-buffered BufferedRx on both instances --------------------------------------

/// Install the RAM vector table, enable interrupts, bring up a `BufferedRx` on each instance, run the
/// three S1 stages, and record + return whether every stage matched exactly.
fn run_s1(chip: &Chip, usart1: Usart, usart2: Usart) -> bool {
    // The interrupt + DMA RX need the RAM vector table routed and VTOR flipped BEFORE any `new` unmasks
    // its IRQ. Done once here; S2 reuses it.
    // SAFETY: RAM init is done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
    unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
    // SAFETY: enabling interrupts after the table is installed; handlers are registered by each `new`.
    unsafe { cortex_m::interrupt::enable() };

    // SAFETY: each ring is a 'static, used only as this receiver's SPSC buffer.
    let mut rx_u1 = match BufferedRx::new(chip, usart1, PeriphLabel::Usart1, unsafe {
        &mut *addr_of_mut!(RING_U1)
    }) {
        Ok(r) => r,
        Err(_) => return false,
    };
    // SAFETY: as above.
    let mut rx_mod = match BufferedRx::new(chip, usart2, PeriphLabel::Usart2, unsafe {
        &mut *addr_of_mut!(RING_MOD)
    }) {
        Ok(r) => r,
        Err(_) => return false, // module BufferedRx is F10x-only; on F1x0 this fails loud
    };

    // Stage 1 - module via BufferedRx: USART1 polled-sends FRAME (PA2) -> the module slot receives it.
    send(&usart1, &FRAME);
    let (mod_len, mod_match, mod_idle) = drain_buffered(&mut rx_mod, &FRAME);
    store!(s1_mod_len, mod_len);
    store!(s1_mod_match, mod_match);
    store!(s1_mod_idle, mod_idle as u8);

    // Stage 2 - USART1 regression: the module polled-sends FRAME_HI (PB10) -> USART1's slot receives it.
    send(&usart2, &FRAME_HI);
    let (u1_len, u1_match, u1_idle) = drain_buffered(&mut rx_u1, &FRAME_HI);
    store!(s1_u1_len, u1_len);
    store!(s1_u1_match, u1_match);
    store!(s1_u1_idle, u1_idle as u8);

    // Stage 3 - coexistence: interleave both TX lines so both slots fill concurrently, then drain each
    // and confirm it holds ONLY its own (disjoint) frame.
    for i in 0..FRAME.len() {
        usart1.write_byte(FRAME[i]);
        usart2.write_byte(FRAME_HI[i]);
    }
    let (_, cox_mod_match, _) = drain_buffered(&mut rx_mod, &FRAME);
    let (_, cox_u1_match, _) = drain_buffered(&mut rx_u1, &FRAME_HI);
    store!(s1_cox_mod_match, cox_mod_match);
    store!(s1_cox_u1_match, cox_u1_match);

    let full = FRAME.len() as u8;
    let pass = mod_match == full
        && mod_idle
        && u1_match == full
        && u1_idle
        && cox_mod_match == full
        && cox_u1_match == full;
    store!(s1_pass, pass as u8);
    pass
}

// --- S2: DMA-ring RingBufferedRx on both instances --------------------------------------------

/// Run the five S2 stages and record + return whether each passed. The RAM table + interrupts are
/// already up (from `run_s1`). `usart1`/`usart2` are at 115200 (the boot config); stages 4-5 reprogram
/// the BAUD via `bring_up`.
fn run_s2(chip: &Chip, usart1: Usart, usart2: Usart) -> bool {
    // Stage 1 - module via RingBufferedRx @115200: USART1 streams -> the module DMA channel (Ch2)
    // receives. recv > 0 is what confirms Ch2 is the channel the module USART actually feeds (a wrong
    // channel passes the self-check but gets nothing). First byte in the module-stream range confirms
    // it is not pulling from USART1's channel.
    let mut rxm = match new_ring(chip, usart2, PeriphLabel::Usart2, buf_mod()) {
        Ok(r) => r,
        Err(_) => return false, // module RingBufferedRx is F10x-only; on F1x0 this fails loud
    };
    stream(&usart1, S2_N, 0);
    let (mod_recv, _mod_loss, mod_ovr, mod_idle, mod_first) = drain_dma(&mut rxm, S2_N);
    store!(s2_mod_recv, mod_recv);
    store!(s2_mod_ovr, mod_ovr);
    store!(s2_mod_idle, mod_idle as u8);
    let mod_ok = mod_recv as usize == S2_N && mod_ovr == 0 && mod_idle && mod_first < 0x80;

    // Stage 2 - USART1 RingBufferedRx regression @115200 (Ch5): the module streams -> USART1 receives.
    let mut rx1 = match new_ring(chip, usart1, PeriphLabel::Usart1, buf_u1()) {
        Ok(r) => r,
        Err(_) => return false,
    };
    stream(&usart2, S2_N, 0);
    let (u1_recv, _u1_loss, u1_ovr, _u1_idle, _u1_first) = drain_dma(&mut rx1, S2_N);
    store!(s2_u1_recv, u1_recv);
    store!(s2_u1_ovr, u1_ovr);
    let u1_ok = u1_recv as usize == S2_N && u1_ovr == 0;

    // Stage 3 - coexistence: both DMA receivers live. Interleave disjoint streams (module = LOW range,
    // USART1 = HIGH range), then drain each and confirm it received its full stream with in-range
    // content (a channel/context collision would put the other stream's range into a slot).
    let mut rxm = match new_ring(chip, usart2, PeriphLabel::Usart2, buf_mod()) {
        Ok(r) => r,
        Err(_) => return false,
    };
    let mut rx1 = match new_ring(chip, usart1, PeriphLabel::Usart1, buf_u1()) {
        Ok(r) => r,
        Err(_) => return false,
    };
    for i in 0..S2_N {
        usart1.write_byte(COX_MOD_START.wrapping_add(i as u8)); // -> module RX (Ch2)
        usart2.write_byte(COX_U1_START.wrapping_add(i as u8)); // -> USART1 RX (Ch5)
    }
    let (cox_mod_recv, _, _, _, cox_mod_first) = drain_dma(&mut rxm, S2_N);
    let (cox_u1_recv, _, _, _, cox_u1_first) = drain_dma(&mut rx1, S2_N);
    store!(s2_cox_mod_recv, cox_mod_recv);
    store!(s2_cox_u1_recv, cox_u1_recv);
    let cox_ok = cox_mod_recv as usize == S2_N
        && cox_mod_first < 0x80 // module received the LOW-range stream, not USART1's HIGH-range one
        && cox_u1_recv as usize == S2_N
        && cox_u1_first >= 0x80;
    store!(s2_cox_ok, cox_ok as u8);

    // Stage 4 - high-rate stress on the module RingBufferedRx at 2.25 Mbit/s. Reprogram both USARTs to
    // the fast baud (AF already configured), arm a fresh module receiver, and burst-stream far more than
    // the ring so the buffer wraps repeatedly while the contiguity check measures real loss.
    let uf1 = reprogram(chip, PeriphLabel::Usart1, BAUD_FAST);
    let uf2 = reprogram(chip, PeriphLabel::Usart2, BAUD_FAST);
    let (stress_recv, stress_loss) = match new_ring(chip, uf2, PeriphLabel::Usart2, buf_mod()) {
        Ok(mut r) => stress(&uf1, &mut r, S2_STRESS, S2_BURST),
        Err(_) => return false,
    };
    store!(s2_stress_recv, stress_recv);
    store!(s2_stress_loss, stress_loss);
    store!(s2_stress_baud_k, (BAUD_FAST / 1000) as u16);
    // Allow a couple of in-flight bytes not yet committed at the final drain; loss must be exactly 0.
    let stress_ok = (stress_recv as usize) + 4 >= S2_STRESS && stress_loss == 0;

    // Stage 5 - the deployment-representative 9600 8N1 pass on the module RingBufferedRx.
    let us1 = reprogram(chip, PeriphLabel::Usart1, BAUD_9600);
    let us2 = reprogram(chip, PeriphLabel::Usart2, BAUD_9600);
    let recv96 = match new_ring(chip, us2, PeriphLabel::Usart2, buf_mod()) {
        Ok(mut r) => {
            stream(&us1, S2_N, 0);
            let (recv, _, _, _, _) = drain_dma(&mut r, S2_N);
            recv
        }
        Err(_) => return false,
    };
    store!(s2_9600_recv, recv96);
    let ok96 = recv96 as usize == S2_N;
    store!(s2_9600_match, ok96 as u8);

    let pass = mod_ok && u1_ok && cox_ok && stress_ok && ok96;
    store!(s2_pass, pass as u8);
    pass
}

/// Reprogram a USART's BAUD (and re-assert the 8N1 frame + RX/TX enable) without re-touching its GPIO
/// AF, via `bring_up`. The pins were AF-configured once by `Usart::new`; this changes only the
/// peripheral registers, returning a fresh handle on the same base.
fn reprogram(chip: &Chip, label: PeriphLabel, baud: u32) -> Usart {
    let cfg = UsartConfig {
        usart: label,
        tx: 0,
        rx: 0,
        baud,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    };
    match Usart::bring_up(chip, &CLOCK, &cfg) {
        Ok(u) => u,
        Err(_) => halt(),
    }
}

/// Arm a `RingBufferedRx` on `usart`/`instance` over the whole DMA buffer `buf`.
fn new_ring(
    chip: &Chip,
    usart: Usart,
    instance: PeriphLabel,
    buf: &'static mut [u8],
) -> Result<RingBufferedRx, runtime_hal::DescriptorError> {
    RingBufferedRx::new(chip, usart, instance, buf)
}

/// A fresh `'static` view of the module DMA buffer (one active receiver per phase; the channel is
/// disabled by the next `RingBufferedRx::new` before re-use).
fn buf_mod() -> &'static mut [u8] {
    // SAFETY: 'static; a single module RingBufferedRx is active at a time (phases are sequential, and
    // the only concurrent receiver, USART1's, uses the distinct DMA_BUF_U1).
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF_MOD) as *mut u8, DMA_CAP) }
}

/// A fresh `'static` view of the USART1 DMA buffer.
fn buf_u1() -> &'static mut [u8] {
    // SAFETY: as `buf_mod`, on the distinct USART1 buffer.
    unsafe { core::slice::from_raw_parts_mut(addr_of_mut!(DMA_BUF_U1) as *mut u8, DMA_CAP) }
}

// --- stream / receive helpers -----------------------------------------------------------------

/// Send a frame on `tx` (polled): `write_byte` blocks until TC, so each byte has fully shifted out (and
/// the receiver's DMA/ISR serviced it) by the time this returns.
fn send(tx: &Usart, frame: &[u8]) {
    for &b in frame {
        tx.write_byte(b);
    }
}

/// Stream `n` bytes back-to-back with value `(start + i) & 0xFF` (a contiguous pattern the DMA receiver
/// checks for loss). `write_byte` blocks per byte, so the DMA fills the receiver's ring concurrently.
fn stream(tx: &Usart, n: usize, start: usize) {
    for i in 0..n {
        tx.write_byte(((start + i) & 0xFF) as u8);
    }
}

/// Drain a `BufferedRx` (S1) until its IDLE boundary (or a timeout), comparing against `expect`.
/// Returns `(bytes received, bytes matching expect, IDLE-seen)`.
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

/// Contiguity checker for a known stream (value = position & 0xFF): counts delivered bytes and FORWARD
/// gaps (real loss = the ring lapped, dropping unread bytes), ignoring a backward jump (a spurious
/// resync, no data lost). Self-calibrates from the first byte, so it does not need the absolute start.
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

/// Drain a `RingBufferedRx` (S2) until `expect` bytes have arrived or its IDLE boundary / a timeout.
/// Returns `(recv, loss, overrun-count, IDLE-seen, first-byte)`.
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

/// High-rate stress: stream `total` bytes (>> the ring) in `burst`-sized chunks, draining the module
/// `RingBufferedRx` after each burst. The burst is smaller than the ring, so the buffer never laps; the
/// total is far larger, so it wraps repeatedly. Returns `(recv, real-loss)` from the contiguity check.
fn stress(tx: &Usart, rx: &mut RingBufferedRx, total: usize, burst: usize) -> (u16, u16) {
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

// --- shared helpers ---------------------------------------------------------------------------

/// Pulse PB9 (~100 ms): the human-visible "S0 + S1 + S2 passed".
fn pulse(led: &mut impl OutputPin) {
    let _ = led.set_high();
    cortex_m::asm::delay(7_200_000);
    let _ = led.set_low();
}

fn write_magic() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    store!(magic, MAGIC);
}

/// Halt forever on an unrecoverable bring-up error. Busy-spin (NEVER wfi: GD32 SWD-lockout rule).
fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
