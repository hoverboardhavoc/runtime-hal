//! On-silicon S0 validator for the multi-instance USART RX spec
//! (`specs/uart-rx-multi-instance.md`, Acceptance S0): board bring-up using ONLY the existing,
//! already-proven instance-generic polled path, with ZERO new library code.
//!
//! Topology is a SINGLE F103 bluepill with an intra-board cross-wire (NOT the prior F103+F130 pair):
//!   - `PA2`  (HAL `PeriphLabel::Usart1` TX, ST USART2 block @0x4000_4400) -> `PB11` (`Usart2` RX,
//!     ST USART3 block @0x4000_4800, F10x-only).
//!   - `PB10` (`Usart2` TX) -> `PA3` (`Usart1` RX).
//! Both instances are on APB1. Each USART is both a polled sender and a polled receiver.
//!
//! Direction A: send a known frame on USART1 TX (PA2) -> read it polled on the module-USART RX (PB11).
//! Direction B: send the frame on the module-USART TX (PB10) -> read it polled on USART1 RX (PA3).
//! A single chip both sends and polls, so a naive "send whole frame then read" would overrun the
//! FIFO-less 1-byte RX register. The transfer is therefore LOCKSTEP: `write_byte` blocks until TC
//! (the frame fully shifted out and so received), then the byte is polled out before the next is sent.
//!
//! PB9 (LED/buzzer) pulses on a full pass (both directions match exactly). Result is a fixed RAM block
//! at [`RESULT_ADDR`] (the reserved RAM tail), `magic` written LAST. Busy-spin forever at the end,
//! NEVER `wfi` (a bare `wfi` with `DBG_CTL0 = 0` locks SWD re-attach on the GD32).
//!
//! The prior pair-validator (F103+F130 Gate-B `RingBufferedRx`, scenarios S2-S5) was committed at
//! `b6bcf04` ("G-DMA-UART: interrupt-buffered + DMA-ring USART RX"); recover it from git history.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::{
    clock, clock::ClockConfig, descriptor::ClockPath, detect_chip, Chip, PeriphLabel, Usart,
};

// --- tunables / facts -------------------------------------------------------------------------

const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;
const BAUD: u32 = 115_200;
/// The known frame both directions exchange: a 16-byte incrementing pattern (`byte[i] == i`).
const FRAME: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
/// Per-byte empty-poll budget. With lockstep TX (`write_byte` waits for TC) the byte is already in
/// the RX register, so the first poll returns it; this is only a hang backstop.
const RX_BUDGET: u32 = 1_000_000;

// --- the SWD-readable result block ------------------------------------------------------------

#[repr(C)]
struct Results {
    /// 0x5330_5242 ("S0RB"), written LAST = the run completed.
    magic: u32,
    /// 1 = F10x (master family, the bench part), 2 = F1x0. The bench is an F103, so 1 is expected.
    role: u8,
    /// Non-zero if `detect_chip` failed.
    detect_err: u8,
    /// Direction A (USART1 TX -> module-USART RX): bytes received.
    a_len: u8,
    /// Direction A: bytes that matched `FRAME` (== `FRAME.len()` on a clean pass).
    a_match: u8,
    /// Direction B (module-USART TX -> USART1 RX): bytes received.
    b_len: u8,
    /// Direction B: bytes that matched `FRAME`.
    b_match: u8,
    /// 1 if BOTH directions matched all `FRAME.len()` bytes.
    pass: u8,
}

const MAGIC: u32 = 0x5330_5242;
const RESULT_ADDR: u32 = 0x2000_1F00;

const INIT: Results = Results {
    magic: 0,
    role: 0,
    detect_err: 0,
    a_len: 0,
    a_match: 0,
    b_len: 0,
    b_match: 0,
    pass: 0,
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

    // USART1 on PA2 (TX) / PA3 (RX), polled, 115200 8N1.
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
    // Module USART (HAL `Usart2`, the ST USART3 block) on PB10 (TX) / PB11 (RX), polled, 115200 8N1.
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

    // Direction A: USART1 TX (PA2) -> module-USART RX (PB11).
    let (a_len, a_match) = loopback(&usart1, &usart2);
    store!(a_len, a_len);
    store!(a_match, a_match);

    // Direction B: module-USART TX (PB10) -> USART1 RX (PA3).
    let (b_len, b_match) = loopback(&usart2, &usart1);
    store!(b_len, b_len);
    store!(b_match, b_match);

    let pass = a_match as usize == FRAME.len() && b_match as usize == FRAME.len();
    store!(pass, pass as u8);
    if pass {
        pulse(&mut led); // human-visible: both jumpers + both USARTs' GPIO/clock/baud are correct
    }

    write_magic();
    loop {
        cortex_m::asm::nop();
    }
}

// --- loopback -----------------------------------------------------------------------------------

/// Send [`FRAME`] on `tx` and read it back polled on `rx`, lockstep (one byte sent then drained
/// before the next), and return `(bytes received, bytes matching FRAME)`.
///
/// `write_byte` blocks until TC (transmission complete), so by the time it returns the byte has fully
/// shifted out of `tx` and into `rx`'s 1-byte RX register; the immediately-following poll drains it.
/// Lockstep is what keeps the FIFO-less RX from overrunning on a single chip that both sends and reads.
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

/// Pulse PB9 (~100 ms): the human-visible "both directions matched".
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
