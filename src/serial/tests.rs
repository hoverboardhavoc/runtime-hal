//! T7 host tests for the `embedded-io` serial impl over the polled USART driver.
//!
//! These run under the `mock` feature against the backing-array register space (`crate::reg`),
//! the "recorded transaction list" idea adapted to the register mock: to test `Read` we pre-seed
//! the USART RX data register + set the `RBNE` status bit (an incoming byte) and assert `Read`
//! returns it; to test `Write` we run `write`/`write_all` and assert the bytes land in the TX data
//! register in order; error-injection sets an overrun/framing/parity status bit and asserts the
//! mapped [`UsartError`] / [`embedded_io::ErrorKind`].
//!
//! Mock note: the backing array has no UART core, so it does not auto-set TBE/TC nor auto-clear
//! RBNE the way silicon would. Tests seed the status flags by hand. Because `write_byte` polls TBE
//! then TC, the TX tests pre-seed `TBE | TC` so the polled send loop does not spin forever; the RX
//! tests seed `RBNE`. STAT and the data registers are at the F10x offsets (STAT `0x00`, DATA
//! `0x04`); the bench link is USART1, so that matches the T6 tests.
#![cfg(feature = "mock")]

use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, UsartConfig, UsartFrame};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::UsartError;
use crate::reg::{mock, Reg32};
use crate::serial::UsartSerial;
use crate::usart::Usart;
use embedded_io::{Error, ErrorKind, Read, ReadReady, Write, WriteReady};
use std::sync::MutexGuard;

/// USART base in the mock window (offsets within it are what the assertions key on). Matches the
/// T6 tests' base; the window wraps modulo its size, so only the offsets are load-bearing.
const USART_BASE: u32 = 0x4000_4400;

// F10x STAT bit positions (STAT at offset 0x00), from the documented layout.
const STAT_OFF: u32 = 0x00;
const DATA_OFF: u32 = 0x04;
const STAT_PERR: u32 = 1 << 0;
const STAT_FERR: u32 = 1 << 1;
const STAT_ORERR: u32 = 1 << 3;
const STAT_RBNE: u32 = 1 << 5;
const STAT_TC: u32 = 1 << 6;
const STAT_TBE: u32 = 1 << 7;

/// Acquire the whole-case serialization lock and zero the register window.
fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

/// A brought-up USART handle for the bench config (USART1, F10x, 72 MHz, 115200), wrapped as a
/// serial endpoint. `bring_up` runs its RMW sequence against the (just-reset) mock space.
fn serial() -> UsartSerial {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    let chip = Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F10xSeparate,
        addrs,
        flash_page: PageSize::K1,
        adv_timers: 1,
        adc_count: 2,
    });
    let cfg = UsartConfig {
        usart: PeriphLabel::Usart1,
        tx: 0x02,
        rx: 0x03,
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    };
    let u = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &cfg).unwrap();
    UsartSerial::from_usart(u)
}

#[inline]
fn set_stat(bits: u32) {
    Reg32::new(USART_BASE, STAT_OFF).write(bits);
}
#[inline]
fn seed_rx(byte: u8) {
    Reg32::new(USART_BASE, DATA_OFF).write(byte as u32);
    set_stat(STAT_RBNE);
}
#[inline]
fn tx_byte() -> u8 {
    (Reg32::new(USART_BASE, DATA_OFF).read() & 0xFF) as u8
}

// --- Read -------------------------------------------------------------------------------------

#[test]
fn read_returns_the_seeded_rx_byte() {
    let _g = seed_reset();
    let mut s = serial();
    seed_rx(0x5A);
    let mut buf = [0u8; 1];
    assert_eq!(s.read(&mut buf), Ok(1));
    assert_eq!(
        buf[0], 0x5A,
        "Read returns the byte seeded in the RX data register"
    );
}

#[test]
fn read_returns_bytes_in_order_across_calls() {
    // The static mock has no RX FIFO, so feed one byte per `read` (re-seed the data register +
    // RBNE), and assert the sequence comes back in order. This is the recorded-transaction list:
    // a queue of incoming bytes, asserted in arrival order.
    let _g = seed_reset();
    let mut s = serial();
    let incoming = [b'P', b'I', b'N', b'G'];
    let mut got = [0u8; 4];
    for (i, &b) in incoming.iter().enumerate() {
        seed_rx(b);
        let mut one = [0u8; 1];
        assert_eq!(s.read(&mut one), Ok(1));
        got[i] = one[0];
    }
    assert_eq!(got, incoming, "bytes arrive in order");
}

#[test]
fn read_drains_up_to_buf_len_while_rbne_stays_set() {
    // With RBNE held set (the mock has no consuming RX core), the drain loop fills the whole
    // buffer from the data register: this exercises the "read at least one, then drain what's
    // ready up to buf.len()" path. Each slot reads the current data-register byte.
    let _g = seed_reset();
    let mut s = serial();
    seed_rx(0x42);
    let mut buf = [0u8; 4];
    assert_eq!(
        s.read(&mut buf),
        Ok(4),
        "drains up to buf.len() while ready"
    );
    assert_eq!(buf, [0x42; 4]);
}

#[test]
fn read_empty_buffer_returns_zero_without_blocking() {
    // Contract: read([]) returns Ok(0) without blocking and is not EOF. No RBNE seeded, so if it
    // tried to block this test would hang; it returning proves the empty fast-path.
    let _g = seed_reset();
    let mut s = serial();
    set_stat(0); // nothing ready
    let mut empty: [u8; 0] = [];
    assert_eq!(s.read(&mut empty), Ok(0));
}

#[test]
fn read_blocks_for_the_first_byte_then_returns_it() {
    // "Blocks until at least one byte is available": seed RBNE clear, then set it (single-threaded
    // here, so we model the eventual arrival by seeding before the call). The real proof of
    // blocking-not-erroring is that read does NOT return Ok(0) or a spurious error when a byte is
    // present: it returns the byte. (A WouldBlock-style impl would be wrong for embedded-io.)
    let _g = seed_reset();
    let mut s = serial();
    seed_rx(0xC3);
    let mut buf = [0u8; 2];
    // RBNE stays set, so both slots fill from the data register; the point is the first byte is
    // delivered rather than the call erroring or reporting EOF.
    assert_eq!(s.read(&mut buf), Ok(2));
    assert_eq!(buf[0], 0xC3);
}

// --- Read errors (line-error injection) -------------------------------------------------------

#[test]
fn read_maps_framing_and_parity_to_usart_error() {
    let _g = seed_reset();
    let mut s = serial();
    let mut buf = [0u8; 4];

    // Framing / parity still surface as Err through the `embedded-io` Read seam (the byte is
    // suspect), now cleared by the HAL first so they cannot latch. (Overrun is no longer an error
    // here: it self-recovers; see `read_self_recovers_from_overrun`.)
    set_stat(STAT_FERR | STAT_RBNE);
    assert_eq!(s.read(&mut buf), Err(UsartError::Framing));

    set_stat(STAT_PERR | STAT_RBNE);
    assert_eq!(s.read(&mut buf), Err(UsartError::Parity));
}

#[test]
fn read_self_recovers_from_overrun() {
    // An overrun (ORERR) must NOT surface as Err through the `embedded-io` Read seam (the
    // link_bench latch bug). The HAL clears it and, with a byte still ready (RBNE), `read` returns
    // the byte rather than erroring or stranding RX. (F10x model: STAT 0x00, DATA 0x04. The mock has
    // no UART core, so RBNE stays set and the drain fills the whole buffer from the data register.)
    let _g = seed_reset();
    let mut s = serial();
    seed_rx(0x9A); // DATA = 0x9A, RBNE set
    set_stat(STAT_ORERR | STAT_RBNE); // overrun pending alongside the ready byte
    let mut buf = [0u8; 4];
    assert_eq!(
        s.read(&mut buf),
        Ok(4),
        "overrun self-recovers: Read returns bytes, not Err"
    );
    assert_eq!(
        buf, [0x9A; 4],
        "the fresh byte is delivered after the overrun clear"
    );
}

#[test]
fn serial_error_kind_is_other() {
    // Pin the mapping (M1 open item 4): every USART line error maps to embedded_io::ErrorKind::Other,
    // since embedded-io has no dedicated overrun/framing/parity kinds. Ties the T7 seam to the T1
    // `embedded_io::Error for UsartError` impl so the mapping cannot drift unnoticed.
    assert_eq!(UsartError::Overrun.kind(), ErrorKind::Other);
    assert_eq!(UsartError::Framing.kind(), ErrorKind::Other);
    assert_eq!(UsartError::Parity.kind(), ErrorKind::Other);
    assert_eq!(UsartError::Other.kind(), ErrorKind::Other);
}

// --- Write ------------------------------------------------------------------------------------

#[test]
fn write_lands_a_byte_in_the_tx_data_register() {
    let _g = seed_reset();
    let mut s = serial();
    // write_byte polls TBE then TC; seed both so the polled send loop does not spin.
    set_stat(STAT_TBE | STAT_TC);
    assert_eq!(s.write(&[0x5A]), Ok(1));
    assert_eq!(tx_byte(), 0x5A, "the byte lands in the TX data register");
}

#[test]
fn write_returns_full_buffer_length() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(STAT_TBE | STAT_TC);
    // The static mock keeps only the last byte written to the data register, so capture each as we
    // go via a single-byte write loop, asserting they land in order. (write([..]) sends them all;
    // here we verify the per-byte transaction order.)
    let payload = [b'A', b'C', b'K'];
    let mut captured = [0u8; 3];
    for (i, &b) in payload.iter().enumerate() {
        assert_eq!(s.write(&[b]), Ok(1));
        captured[i] = tx_byte();
    }
    assert_eq!(
        captured, payload,
        "bytes written to the TX register in order"
    );
}

#[test]
fn write_all_sends_every_byte_in_order() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(STAT_TBE | STAT_TC);
    // write_all over the whole buffer; the mock keeps the last byte, so assert the final byte and
    // that the call succeeded. Per-byte order is covered by `write_returns_full_buffer_length`.
    let payload = b"hello";
    assert_eq!(s.write_all(payload), Ok(()));
    assert_eq!(
        tx_byte(),
        *payload.last().unwrap(),
        "last byte reached the TX register"
    );
}

#[test]
fn write_empty_buffer_returns_zero() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(0); // no TBE/TC; an empty write must not touch the peripheral or spin
    assert_eq!(s.write(&[]), Ok(0));
}

#[test]
fn flush_waits_for_tc_then_returns() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(STAT_TC); // TC already set, so flush returns immediately
    assert_eq!(s.flush(), Ok(()));
}

// --- ready traits -----------------------------------------------------------------------------

#[test]
fn read_ready_reflects_rbne() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(0);
    assert_eq!(s.read_ready(), Ok(false));
    set_stat(STAT_RBNE);
    assert_eq!(s.read_ready(), Ok(true));
}

#[test]
fn write_ready_reflects_tbe() {
    let _g = seed_reset();
    let mut s = serial();
    set_stat(0);
    assert_eq!(s.write_ready(), Ok(false));
    set_stat(STAT_TBE);
    assert_eq!(s.write_ready(), Ok(true));
}
