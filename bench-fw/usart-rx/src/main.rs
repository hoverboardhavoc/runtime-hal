//! On-silicon S0 + S1 validator for the multi-instance USART RX spec
//! (`specs/uart-rx-multi-instance.md`, Acceptance S0 and S1).
//!
//! Topology is a SINGLE F103 bluepill with an intra-board cross-wire (NOT the prior F103+F130 pair):
//!   - `PA2`  (HAL `PeriphLabel::Usart1` TX, GD USART1 block @0x4000_4400) -> `PB11` (`Usart2` RX,
//!     GD USART2 block @0x4000_4800, F10x-only).
//!   - `PB10` (`Usart2` TX) -> `PA3` (`Usart1` RX).
//! Both instances are on APB1. Each USART is both a sender and a receiver.
//!
//! **S0 (polled, zero new library code):** lockstep polled loopback each direction, proving both
//! jumpers + both USARTs' GPIO/AF/clock/baud. `write_byte` blocks until TC, so a byte is fully shifted
//! out (and received) before the next is sent; this avoids overrunning the FIFO-less 1-byte RX register
//! on a single chip that both sends and reads.
//!
//! **S1 (interrupt-buffered `BufferedRx`, the second-instance generalization):** the owned RAM vector
//! table is installed (VTOR flipped) and interrupts enabled, then a `BufferedRx` is brought up on BOTH
//! instances. Three stages:
//!   1. module-via-BufferedRx: USART1 polled-sends `FRAME` on PA2 -> the module `BufferedRx` receives it
//!      on PB11 through ITS OWN slot + vector (GD `USART2_IRQn` = 39). This is what proves the new
//!      vector wiring fires on silicon (a wrong IRQ number receives nothing).
//!   2. USART1 regression: the module polled-sends `FRAME_HI` on PB10 -> USART1's `BufferedRx` receives
//!      on PA3 (slot 0 / IRQ 38) exactly as before.
//!   3. coexistence: both `BufferedRx` live at once; bytes interleaved on both TX lines; each slot
//!      receives only its own frame (`FRAME` vs the disjoint `FRAME_HI`), no loss or cross-talk.
//!
//! PB9 (LED/buzzer) pulses on a full S0+S1 pass. Result is a fixed RAM block at [`RESULT_ADDR`] (the
//! reserved RAM tail), `magic` written LAST. Busy-spin forever at the end, NEVER `wfi` (a bare `wfi`
//! with `DBG_CTL0 = 0` locks SWD re-attach on the GD32).
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
    BufferedRx, Chip, PeriphLabel, Usart,
};

// --- tunables / facts -------------------------------------------------------------------------

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const BAUD: u32 = 115_200;
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
/// Empty-iteration budget for an S1 buffered drain (the ring absorbs the burst; this bounds the wait
/// for the IDLE boundary / stragglers so a missed frame cannot hang the run).
const RX_ITERS: u32 = 2_000_000;

// --- the SWD-readable result block ------------------------------------------------------------

#[repr(C)]
struct Results {
    /// 0x5331_5242 ("S1RB"), written LAST = the run completed.
    magic: u32,
    /// 1 = F10x (master family, the bench part), 2 = F1x0. The bench is an F103, so 1 is expected.
    role: u8,
    /// Non-zero if `detect_chip` failed.
    detect_err: u8,

    // --- S0 (polled loopback) ---
    /// S0 direction A (USART1 TX -> module RX): bytes received / matched.
    s0_a_len: u8,
    s0_a_match: u8,
    /// S0 direction B (module TX -> USART1 RX): bytes received / matched.
    s0_b_len: u8,
    s0_b_match: u8,
    /// 1 if both S0 directions matched all `FRAME.len()` bytes.
    s0_pass: u8,

    // --- S1 (interrupt-buffered BufferedRx) ---
    /// S1 stage 1, module-via-BufferedRx (USART1 TX -> module RX): bytes / matched / IDLE-seen.
    s1_mod_len: u8,
    s1_mod_match: u8,
    s1_mod_idle: u8,
    /// S1 stage 2, USART1 BufferedRx regression (module TX -> USART1 RX): bytes / matched / IDLE-seen.
    s1_u1_len: u8,
    s1_u1_match: u8,
    s1_u1_idle: u8,
    /// S1 stage 3, coexistence: the module slot matched its own frame / the USART1 slot matched its own.
    s1_cox_mod_match: u8,
    s1_cox_u1_match: u8,
    /// 1 if every S1 stage matched all 16 bytes (both directions, both idles, both coexistence slots).
    s1_pass: u8,
}

const MAGIC: u32 = 0x5331_5242;
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
};

/// The two application-owned `'static` SPSC rings (one per `BufferedRx` instance). Capacity word 64 =
/// holds 63 bytes, far more than a 16-byte frame.
static mut RING_U1: Queue<u8, 64> = Queue::new();
static mut RING_MOD: Queue<u8, 64> = Queue::new();
/// The owned RAM vector table (interrupt RX needs the USART vectors routed; `install` flips VTOR).
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

    // USART1 on PA2 (TX) / PA3 (RX), 115200 8N1.
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

    if s0_pass && s1_pass {
        pulse(&mut led); // human-visible: S0 + the second-instance interrupt path both passed
    }

    write_magic();
    loop {
        cortex_m::asm::nop();
    }
}

// --- S0: polled loopback ----------------------------------------------------------------------

/// Lockstep polled loopback both directions; record + return whether both matched exactly.
fn run_s0(usart1: &Usart, usart2: &Usart) -> bool {
    // Direction A: USART1 TX (PA2) -> module-USART RX (PB11).
    let (a_len, a_match) = loopback(usart1, usart2);
    store!(s0_a_len, a_len);
    store!(s0_a_match, a_match);

    // Direction B: module-USART TX (PB10) -> USART1 RX (PA3).
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
                // No byte yet (or a transient line error that self-cleared): keep polling.
                _ => {
                    budget -= 1;
                    if budget == 0 {
                        return (len, matched); // RX stalled: report what arrived
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
    // The interrupt RX needs the RAM vector table routed (USART1 -> IRQ 38, module -> IRQ 39) and VTOR
    // flipped BEFORE any `BufferedRx::new` unmasks its IRQ.
    // SAFETY: RAM init is done, no peripheral IRQ enabled yet, VECTORS is a 'static aligned table.
    unsafe { install(&mut *addr_of_mut!(VECTORS), chip.irq()) };
    // SAFETY: enabling interrupts after the table is installed; handlers are registered by each `new`.
    unsafe { cortex_m::interrupt::enable() };

    // Both receivers live at once: USART1 BufferedRx (slot 0 / IRQ 38), module BufferedRx (slot 1 /
    // IRQ 39, F10x-only). `usart*` are Copy, so each receiver keeps a handle for polled TX.
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
    let (mod_len, mod_match, mod_idle) = drain(&mut rx_mod, &FRAME);
    store!(s1_mod_len, mod_len);
    store!(s1_mod_match, mod_match);
    store!(s1_mod_idle, mod_idle as u8);

    // Stage 2 - USART1 regression: the module polled-sends FRAME_HI (PB10) -> USART1's slot receives it.
    send(&usart2, &FRAME_HI);
    let (u1_len, u1_match, u1_idle) = drain(&mut rx_u1, &FRAME_HI);
    store!(s1_u1_len, u1_len);
    store!(s1_u1_match, u1_match);
    store!(s1_u1_idle, u1_idle as u8);

    // Stage 3 - coexistence: interleave both TX lines so both slots fill concurrently, then drain each
    // and confirm it holds ONLY its own (disjoint) frame.
    for i in 0..FRAME.len() {
        usart1.write_byte(FRAME[i]);
        usart2.write_byte(FRAME_HI[i]);
    }
    let (_, cox_mod_match, _) = drain(&mut rx_mod, &FRAME);
    let (_, cox_u1_match, _) = drain(&mut rx_u1, &FRAME_HI);
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

/// Send a frame on `tx` (polled): `write_byte` blocks until TC, so the receiver's ISR has serviced each
/// byte (and, after the last, the line goes idle and raises the IDLE-line interrupt) by the time this
/// returns.
fn send(tx: &Usart, frame: &[u8]) {
    for &b in frame {
        tx.write_byte(b);
    }
}

/// Drain a `BufferedRx` until its IDLE boundary (or a timeout), comparing against `expect`. Returns
/// `(bytes received, bytes matching expect, IDLE-seen)`.
fn drain(rx: &mut BufferedRx, expect: &[u8]) -> (u8, u8, bool) {
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
            // A transient line error self-clears in the library; keep draining.
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

// --- shared helpers ---------------------------------------------------------------------------

/// Pulse PB9 (~100 ms): the human-visible "S0 + S1 passed".
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
