//! Host tests for the non-blocking `embedded-io` serial adapters (`specs/serial-adapters.md`
//! section 4), under the `mock` feature against the backing-array register space.
//!
//! Polled section: seed the USART RX data register + STAT bits and drive [`PolledSerial`] through
//! the `embedded-io` traits. Split section: replicate the `usart_rx` staging machinery (mock DMA
//! CHxCNT / INTF, `mock_vtor` ISR dispatch) and drive [`SplitSerial`] over real
//! [`RingBufferedRx`] / [`BufferedRx`] backends, so the wrap-boundary and latch semantics are
//! asserted THROUGH the adapter.
//!
//! Mock note: the backing array has no UART core, so it does not auto-set TBE/TC nor auto-clear
//! RBNE/line flags the way silicon would; tests seed the status flags by hand (and a held RBNE
//! makes `read` drain the same data-register byte repeatedly, which several cases exploit).
#![cfg(feature = "mock")]

/// The reference 72 MHz tree, the clock every adapter in these tests derives its write budget
/// from (`ClockConfig::reference()`'s `sysclk_hz`).
const TEST_CORE_HZ: u32 = 72_000_000;

use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, UsartConfig, UsartFrame};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::dma::{DmaRxMap, DMA0_BASE};
use crate::error::UsartError;
use crate::irq::{install_mock, mock_vtor, F10X_DMA0_CH5_IRQ, F10X_USART1_IRQ};
use crate::reg::{mock, Reg32};
use crate::serial::{
    worst_case_write_block_us, write_budget, PolledSerial, SplitSerial, WRITE_BUDGET_US,
    WRITE_POLL_CYCLES,
};
use crate::usart::Usart;
use crate::usart_rx::{BufferedRx, RingBufferedRx, RxRing};
use embedded_io::{Error, ErrorKind, Read, ReadReady, Write, WriteReady};

use std::boxed::Box;
use std::sync::MutexGuard;
use std::vec;

/// USART base in the mock window (offsets within it are what the assertions key on).
const USART_BASE: u32 = 0x4000_4400;
/// A non-zero RAM-table address for `install_mock` (stands in for the section).
const RAM_ADDR: u32 = 0x2000_4000;

// F10x register offsets + STAT bit positions (STAT at 0x00), from the documented layout.
const STAT_OFF: u32 = 0x00;
const DATA_OFF: u32 = 0x04;
const STAT_PERR: u32 = 1 << 0;
const STAT_FERR: u32 = 1 << 1;
const STAT_NERR: u32 = 1 << 2;
const STAT_ORERR: u32 = 1 << 3;
const STAT_IDLEF: u32 = 1 << 4;
const STAT_RBNE: u32 = 1 << 5;
const STAT_TC: u32 = 1 << 6;
const STAT_TBE: u32 = 1 << 7;

// DMA channel register offsets (stride 0x14) + CHCTL enable bit, as pinned by the usart_rx suite.
const CHEN: u32 = 1 << 0;
fn ch_ctl(ch: u8) -> u32 {
    0x08 + 0x14 * ch as u32
}
fn ch_cnt(ch: u8) -> u32 {
    0x0C + 0x14 * ch as u32
}
fn ftf_flag(ch: u8) -> u32 {
    1 << (4 * ch as u32 + 1)
}

/// Acquire the whole-case serialization lock and reset the register window + RX/DMA statics +
/// the recorded VTOR (a fresh world per case).
fn setup() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    // The DMA silicon rule the split-adapter DMA cases depend on: writing INTC (0x04) clears the
    // written bits in INTF (0x00). Declared by the harness.
    mock::w1c_pair(DMA0_BASE + 0x04, DMA0_BASE);
    crate::usart_rx::reset_for_test();
    crate::dma::reset_for_test();
    mock_vtor::reset();
    g
}

/// The F10x bench chip (USART1 + RCU mapped; separate-IRQ layout so the DMA path is Ch5/IRQ 16).
fn chip() -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F10xSeparate,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 2,
    })
}

fn bring_up() -> Usart {
    let cfg = UsartConfig {
        usart: PeriphLabel::Usart1,
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    };
    Usart::bring_up(&chip(), &ClockConfig::REFERENCE_72M_IRC8M, &cfg).unwrap()
}

/// A polled adapter over the bench USART.
fn polled() -> PolledSerial {
    PolledSerial::from_usart(bring_up(), TEST_CORE_HZ)
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

/// A `SplitSerial<RingBufferedRx>` over the bench USART: split, arm the DMA ring (`len` bytes),
/// flip the RAM table. Returns the adapter, the resolved channel, and the raw ring pointer (so a
/// test models DMA stores).
fn split_ring(len: usize) -> (SplitSerial<RingBufferedRx>, u8, *mut u8) {
    let c = chip();
    let (tx, rx) = bring_up().split();
    let buf: &'static mut [u8] = vec![0u8; len].leak();
    let ptr = buf.as_mut_ptr();
    let ch = DmaRxMap::usart1_rx(&c).channel;
    let ring = RingBufferedRx::new(&c, rx, PeriphLabel::Usart1, buf).unwrap();
    install_mock(IrqLayout::F10xSeparate, RAM_ADDR);
    (SplitSerial::new(tx, ring, TEST_CORE_HZ), ch, ptr)
}

/// Model a DMA store into the ring buffer (host RAM, not the mock register space).
fn dma_write(ptr: *mut u8, pos: usize, val: u8) {
    // SAFETY: `pos` is within the leaked buffer the test sized.
    unsafe { *ptr.add(pos) = val };
}

/// Fire the DMA ISR (a counted buffer wrap) / the USART RX ISR.
fn fire_dma() {
    // SAFETY: the RAM table is installed and the DMA slot holds the handler.
    unsafe { mock_vtor::dispatch(F10X_DMA0_CH5_IRQ) };
}
fn fire_usart() {
    // SAFETY: as above, the USART1 slot.
    unsafe { mock_vtor::dispatch(F10X_USART1_IRQ) };
}

// ================================ PolledSerial ==================================================

#[test]
fn polled_read_returns_zero_when_empty() {
    // The headline non-blocking contract (D2): nothing available -> Ok(0), no blocking, no error.
    let _g = setup();
    let mut s = polled();
    set_stat(0);
    let mut buf = [0u8; 4];
    assert_eq!(s.read(&mut buf), Ok(0));
}

#[test]
fn polled_read_drains_available_bytes() {
    let _g = setup();
    let mut s = polled();
    seed_rx(0x5A);
    // The mock RBNE stays set, so the drain fills the buffer from the data register.
    let mut buf = [0u8; 4];
    assert_eq!(s.read(&mut buf), Ok(4));
    assert_eq!(buf, [0x5A; 4]);
}

#[test]
fn polled_read_returns_bytes_in_order_across_calls() {
    let _g = setup();
    let mut s = polled();
    let incoming = [b'P', b'I', b'N', b'G'];
    let mut got = [0u8; 4];
    for (i, &b) in incoming.iter().enumerate() {
        seed_rx(b);
        let mut one = [0u8; 1];
        assert_eq!(s.read(&mut one), Ok(1));
        got[i] = one[0];
    }
    assert_eq!(got, incoming);
}

#[test]
fn polled_read_absorbs_framing_and_parity_and_counts() {
    // D3: line errors never escape as Err; the counter ticks; the flag was cleared by
    // `try_read_byte` so the NEXT read continues. (The mock's STAT is not auto-cleared by the F10x
    // read-pair, so each case re-seeds by hand, modelling the post-clear state.)
    let _g = setup();
    let mut s = polled();
    let mut buf = [0u8; 4];

    set_stat(STAT_FERR | STAT_RBNE);
    assert_eq!(s.read(&mut buf), Ok(0), "framing absorbed, not surfaced");
    assert_eq!(s.line_errors(), 1);

    set_stat(STAT_PERR | STAT_RBNE);
    assert_eq!(s.read(&mut buf), Ok(0), "parity absorbed, not surfaced");
    assert_eq!(s.line_errors(), 2);

    seed_rx(0xB7); // clean byte afterwards: reading continues
    let mut one = [0u8; 1];
    assert_eq!(s.read(&mut one), Ok(1));
    assert_eq!(one[0], 0xB7);
}

#[test]
fn polled_read_self_recovers_from_overrun_without_surfacing_an_error() {
    // An overrun must never SURFACE (it is fully recovered inside `try_read_byte`, D3): with a
    // byte still ready the read returns bytes and no `Err`. It is counted, but as an overrun, not
    // as a line error.
    let _g = setup();
    let mut s = polled();
    seed_rx(0x9A);
    set_stat(STAT_ORERR | STAT_RBNE);
    let mut buf = [0u8; 4];
    assert_eq!(s.read(&mut buf), Ok(4));
    assert_eq!(buf, [0x9A; 4]);
    assert_eq!(s.line_errors(), 0, "overrun is not a counted line error");
    // The mock has no UART core, so ORERR stays set across the whole drain (silicon clears it on
    // the family-correct write/read-pair): each of the four passes therefore observes the
    // condition. On silicon the same drain scores 1.
    assert_eq!(
        s.rx_overruns(),
        4,
        "each observed ORE condition counts once"
    );
}

#[test]
fn polled_read_counts_a_recovered_overrun_apart_from_line_errors() {
    // The BLE port's lost-byte instrument (`specs/ble.md`, the RX-loss counter): a byte lost while
    // the consumer was away is countable, and it is countable AS AN OVERRUN, so a noisy wire
    // cannot masquerade as a consumer that stopped reading (the OQ1 split on the polled path).
    let _g = setup();
    let mut s = polled();
    seed_rx(0x9A);
    set_stat(STAT_ORERR | STAT_RBNE);
    let mut one = [0u8; 1];
    assert_eq!(s.read(&mut one), Ok(1), "the surviving byte still arrives");
    assert_eq!(one[0], 0x9A);
    assert_eq!(s.rx_overruns(), 1, "the lost byte is counted as an overrun");
    assert_eq!(
        s.line_errors(),
        0,
        "an overrun is NOT a line error (the OQ1 split)"
    );

    // And a framing error is the mirror image: it counts as a line error, never as an overrun.
    set_stat(STAT_FERR | STAT_RBNE);
    assert_eq!(s.read(&mut one), Ok(0));
    assert_eq!(s.line_errors(), 1);
    assert_eq!(s.rx_overruns(), 1, "a wire glitch did not touch the losses");
}

#[test]
fn polled_read_leaves_the_overrun_count_at_zero_on_a_clean_stream() {
    // The negative control, and the half that makes a bench zero mean something: a stream read
    // without ever falling behind must leave the instrument at zero, so a non-zero reading on
    // silicon is evidence rather than noise.
    let _g = setup();
    let mut s = polled();
    for &b in b"PING" {
        seed_rx(b);
        let mut one = [0u8; 1];
        assert_eq!(s.read(&mut one), Ok(1));
        assert_eq!(one[0], b);
    }
    // Including the idle poll that finds nothing.
    set_stat(0);
    let mut buf = [0u8; 4];
    assert_eq!(s.read(&mut buf), Ok(0));
    assert_eq!(s.rx_overruns(), 0, "a clean stream loses nothing");
    assert_eq!(s.line_errors(), 0);
}

/// How many 9600 8N1 characters land in a caller-away window of `us` microseconds.
///
/// The character time is 10 bit-times at 9600 baud = 1,041.7 us. The windows are measured, not
/// assumed: the 250 Hz control callback is DWT-bracketed at 1,011 us on the F103 and 1,399 us on an
/// IMU-equipped F130, and the bounded wedged-I2C path at 3,941 us worst case
/// (`hoverboard-firmware/specs/bench-evidence/2026-08-02/wide-division/RECORD.md`).
fn chars_in_window(us: u32) -> usize {
    // Tenths of a microsecond, so the 1,041.7 us character time stays exact in integer math.
    (us as usize * 10) / 10_417 + 1
}

/// A polled receiver LOSES characters across a caller-away window: its whole buffer is the one-byte
/// receive register, so the second character of the window is gone before the caller returns.
///
/// This is the defect side of the BLE port's carrier choice, stated as a property rather than as a
/// register anecdote. The window used is the measured F130 control callback (1,399 us), which is
/// LONGER than one character time, so the port is structurally deaf once per control tick.
#[test]
fn polled_loses_characters_across_a_caller_away_window() {
    let _g = setup();
    // The device behaviour the polled reader depends on: reading RX_DATA clears RBNE.
    mock::read_clears(USART_BASE + DATA_OFF, USART_BASE + STAT_OFF, STAT_RBNE);
    let mut s = polled();

    let n = chars_in_window(1_399);
    assert_eq!(n, 2, "a 1,399 us window holds 2 characters at 9600 8N1");

    // The caller is away for the whole window: characters arrive with no intervening read.
    let sent: Vec<u8> = (0..n).map(|i| b'A' + i as u8).collect();
    for &b in &sent {
        s.inject_rx_byte(b);
    }

    // Drain everything the port has.
    let mut got = vec![];
    let mut buf = [0u8; 16];
    for _ in 0..4 {
        match s.read(&mut buf) {
            Ok(0) => break,
            Ok(k) => got.extend_from_slice(&buf[..k]),
            Err(_) => break,
        }
    }
    assert!(
        got.len() < sent.len(),
        "the one-byte register cannot hold a whole away window (got {got:?})"
    );
    assert_eq!(s.rx_overruns(), 1, "and the loss is counted as an overrun");
    assert_eq!(s.line_errors(), 0, "a loss is not a wire glitch (OQ1)");

    // On THIS family the window costs both characters, not one. The bench chip here is F10x, whose
    // overrun clear is the STAT-then-data read PAIR (`UsartRegs::clear_line_errors`): completing the
    // pair reads the data register and discards the byte, and that same read clears `RBNE`, so the
    // character that was still sitting in the register goes with the one the wire lost. On F1x0 the
    // clear is an `INTC` write instead, so the resident character survives and the window costs one.
    // Either way the caller-away window loses data, which is the property; the family only decides
    // how much.
    assert_eq!(
        got, b"",
        "F10x: the overrun clear discards the resident byte"
    );
}

/// A DMA-ring receiver DELIVERS every character of a caller-away window: the buffer is `len` bytes
/// of RAM the DMA fills with no CPU involvement, so the caller's absence costs nothing.
///
/// The fix side of the same property, over the two measured windows: the 1,399 us control callback
/// and the 3,941 us worst-case wedged-I2C path (about four characters, which is the run of
/// consecutive frame losses that zeroes a 20 Hz drive demand).
#[test]
fn ring_delivers_every_character_of_a_caller_away_window() {
    for window_us in [1_399u32, 3_941] {
        let _g = setup();
        let (mut s, _ch, _ptr) = split_ring(64);

        let n = chars_in_window(window_us);
        let sent: Vec<u8> = (0..n).map(|i| b'a' + i as u8).collect();
        for &b in &sent {
            s.inject_rx_byte(b);
        }

        let mut got = vec![];
        let mut buf = [0u8; 16];
        for _ in 0..4 {
            match s.read(&mut buf) {
                Ok(0) => break,
                Ok(k) => got.extend_from_slice(&buf[..k]),
                Err(e) => panic!("{window_us} us window surfaced {e:?}"),
            }
        }
        assert_eq!(got, sent, "{window_us} us window: every byte, in order");
        assert_eq!(s.lap_overruns(), 0, "{window_us} us window: nothing lapped");
        assert_eq!(s.line_errors(), 0, "{window_us} us window: no wire glitch");
    }
}

/// The injected characters are the same ones the ordinary read path returns, across the circular
/// wrap: the injector is the DMA engine's model, not a shortcut past it.
#[test]
fn ring_injection_survives_the_circular_wrap() {
    let _g = setup();
    let (mut s, _ch, _ptr) = split_ring(8);

    // Two and a half laps of an 8-byte ring, drained every 4 bytes so nothing ever laps.
    let mut expected = vec![];
    for chunk in 0..5u8 {
        let sent: Vec<u8> = (0..4).map(|i| chunk * 4 + i).collect();
        for &b in &sent {
            s.inject_rx_byte(b);
        }
        let mut buf = [0u8; 8];
        let k = s.read(&mut buf).expect("no condition");
        assert_eq!(&buf[..k], &sent[..], "chunk {chunk} read back in order");
        expected.extend_from_slice(&sent);
    }
    assert_eq!(expected.len(), 20, "20 bytes through an 8-byte ring");
    assert_eq!(s.lap_overruns(), 0);
    assert_eq!(s.line_errors(), 0);
}

#[test]
fn polled_read_ready_reflects_rbne_and_pending_overrun() {
    // D4: ReadReady = "a read makes progress": RBNE, or a pending overrun the read would clear.
    let _g = setup();
    let mut s = polled();
    set_stat(0);
    assert_eq!(s.read_ready(), Ok(false));
    set_stat(STAT_RBNE);
    assert_eq!(s.read_ready(), Ok(true));
    set_stat(STAT_ORERR);
    assert_eq!(
        s.read_ready(),
        Ok(true),
        "pending overrun counts as progress"
    );
    // Non-consuming: still ready, and the flag is untouched.
    assert_eq!(s.read_ready(), Ok(true));
}

#[test]
fn polled_write_lands_bytes_in_order_and_flushes() {
    let _g = setup();
    let mut s = polled();
    set_stat(STAT_TBE | STAT_TC);
    let payload = [b'A', b'C', b'K'];
    let mut captured = [0u8; 3];
    for (i, &b) in payload.iter().enumerate() {
        assert_eq!(s.write(&[b]), Ok(1));
        captured[i] = tx_byte();
    }
    assert_eq!(captured, payload);
    assert_eq!(s.flush(), Ok(()));
    assert_eq!(s.write(&[]), Ok(0), "empty write touches nothing");
}

/// The BLE-path requirement: a write must cost a register access, not a character time. TC is left
/// CLEAR (the previous byte is still shifting out at 9600 baud, 1.04 ms of it) and TBE is set, so a
/// `write` that waited for transmission to complete would never return. It must hand the byte over
/// and come straight back.
#[test]
fn polled_write_does_not_wait_for_transmission_to_complete() {
    let _g = setup();
    let mut s = polled();
    set_stat(STAT_TBE); // TBE only: the register is free, the wire is not
    assert_eq!(s.write(&[0x5A]), Ok(1));
    assert_eq!(tx_byte(), 0x5A);
}

/// The other half: a run of bytes is taken only as far as the transmit register accepts it, and the
/// count says how far. Modelled by the device rule that a STAT read clears TBE, so the register goes
/// busy after the first byte exactly as it does on silicon; a `write` that blocked for the register
/// to free would never return the remaining two.
#[test]
fn polled_write_reports_a_partial_take_rather_than_waiting_for_the_register() {
    let _g = setup();
    let mut s = polled();
    mock::read_clears(USART_BASE + STAT_OFF, USART_BASE + STAT_OFF, STAT_TBE);
    set_stat(STAT_TBE);
    assert_eq!(
        s.write(&[b'A', b'C', b'K']),
        Ok(1),
        "one byte accepted, two reported unwritten"
    );
    assert_eq!(tx_byte(), b'A');
    // `write_all` is the caller that finishes the run; it loops on this, at the wire's pace.
    set_stat(STAT_TBE);
    assert_eq!(s.write(&[b'C', b'K']), Ok(1));
    assert_eq!(tx_byte(), b'C');
}

#[test]
fn polled_write_ready_reflects_tbe() {
    let _g = setup();
    let mut s = polled();
    set_stat(0);
    assert_eq!(s.write_ready(), Ok(false));
    set_stat(STAT_TBE);
    assert_eq!(s.write_ready(), Ok(true));
}

#[test]
fn usart_error_kind_is_other() {
    // The UsartError -> embedded_io::ErrorKind mapping stays pinned even though the adapters are
    // Infallible: the receivers' fail-loud reads still carry UsartError for non-adapter callers.
    assert_eq!(UsartError::Overrun.kind(), ErrorKind::Other);
    assert_eq!(UsartError::RingOverrun.kind(), ErrorKind::Other);
    assert_eq!(UsartError::LineError.kind(), ErrorKind::Other);
    assert_eq!(UsartError::Framing.kind(), ErrorKind::Other);
    assert_eq!(UsartError::Parity.kind(), ErrorKind::Other);
}

// ================================ SplitSerial<RingBufferedRx> ===================================

#[test]
fn split_dma_reads_staged_bytes_across_the_wrap() {
    // B8 THROUGH the adapter: bytes behind the DMA head come back in order, including across the
    // circular-buffer end.
    let _g = setup();
    let (mut s, ch, ptr) = split_ring(8);

    let first = [10u8, 11, 12, 13, 14, 15];
    for (i, &b) in first.iter().enumerate() {
        dma_write(ptr, i, b);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 6);

    let mut out = [0u8; 16];
    assert_eq!(s.read(&mut out), Ok(6));
    assert_eq!(&out[..6], &first);

    // 4 more bytes, wrapping: buf[6], buf[7], buf[0], buf[1]; the wrap is counted by the DMA ISR.
    dma_write(ptr, 6, 16);
    dma_write(ptr, 7, 17);
    dma_write(ptr, 0, 20);
    dma_write(ptr, 1, 21);
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    fire_dma();
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 2);

    assert_eq!(s.read(&mut out), Ok(4));
    assert_eq!(&out[..4], &[16, 17, 20, 21]);
    assert_eq!(s.line_errors(), 0);
    assert_eq!(s.lap_overruns(), 0);
}

#[test]
fn split_dma_ready_is_nonconsuming_and_sees_the_pending_wrap() {
    // D4 + B13 THROUGH the adapter: ReadReady reflects staged bytes without consuming them, and
    // the pending-FTF wrap snapshot (CHxCNT reloaded to len, ISR not yet run) reads ready, then
    // the read returns the new bytes with NO spurious error.
    let _g = setup();
    let (mut s, ch, ptr) = split_ring(8);

    assert_eq!(s.read_ready(), Ok(false), "fresh ring: nothing available");

    dma_write(ptr, 0, 42);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 1);
    assert_eq!(s.read_ready(), Ok(true));
    assert_eq!(s.read_ready(), Ok(true), "ready consumed nothing");
    let mut out = [0u8; 4];
    assert_eq!(s.read(&mut out), Ok(1));
    assert_eq!(out[0], 42);
    assert_eq!(s.read_ready(), Ok(false), "drained");

    // The B13 hazard: the DMA completed a lap exactly (CHxCNT reloaded to len, FTFIF pending, the
    // wrap-counter ISR not yet run). The remaining 7 bytes of the lap were written; the snapshot
    // must attribute the pending wrap, not undercount or report a spurious condition.
    for (i, v) in (1..8).zip(50u8..) {
        dma_write(ptr, i, v);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8); // reloaded
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch)); // FTF pending, not yet counted
    assert_eq!(s.read_ready(), Ok(true), "pending-wrap bytes are visible");
    let mut out = [0u8; 16];
    assert_eq!(
        s.read(&mut out),
        Ok(7),
        "the lap's bytes, no spurious error"
    );
    assert_eq!(&out[..7], &[50, 51, 52, 53, 54, 55, 56]);
    assert_eq!(s.line_errors(), 0, "B13 must not count as a condition");
    assert_eq!(s.lap_overruns(), 0, "B13 is not a lap either");
}

#[test]
fn split_dma_absorbs_a_lap_and_keeps_reading() {
    // B9 THROUGH the adapter: a genuine lap (data overwritten) is absorbed - the counter ticks,
    // Err never escapes, and the same read call returns post-resync state; later bytes flow. The OQ1
    // split: a lap counts `lap_overruns` (a slow-consumer loss), NOT `line_errors` (a wire glitch).
    let _g = setup();
    let (mut s, ch, ptr) = split_ring(8);

    // Two counted wraps + write index 1 with the cursor still at 0: lapped.
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    fire_dma();
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    fire_dma();
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 1);

    let mut out = [0u8; 16];
    assert_eq!(s.read(&mut out), Ok(0), "lap absorbed; cursor resynced");
    assert_eq!(s.lap_overruns(), 1, "the lap counted once as a lap-overrun");
    assert_eq!(
        s.line_errors(),
        0,
        "a lap is NOT a line error (the OQ1 split)"
    );

    // The channel stayed live: new bytes past the resync point still arrive.
    dma_write(ptr, 1, 0xEE);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 2);
    assert_eq!(s.read(&mut out), Ok(1));
    assert_eq!(out[0], 0xEE);
}

#[test]
fn split_dma_consumes_the_idle_latch_invisibly() {
    // D3: the IDLE latch is adapter-owned. Latch it via the shared USART ISR, read through the
    // adapter (which consumes it), then confirm via the backend that it is spent.
    let _g = setup();
    let (mut s, _ch, _ptr) = split_ring(8);

    set_stat(STAT_IDLEF);
    fire_usart(); // the shared RX ISR latches idle_seen for this instance
    let mut out = [0u8; 4];
    assert_eq!(s.read(&mut out), Ok(0), "no bytes; latch consumed silently");

    let (_tx, ring) = s.into_parts();
    assert!(!ring.take_idle(), "the adapter consumed the latch");
}

#[test]
fn split_dma_absorbs_a_line_error_and_keeps_reading() {
    // The always-on-link self-heal (silicon 2026-07-17) THROUGH the adapter: a hardware line error
    // under DMA (the ERRIE path) no longer disables the channel. The adapter absorbs the one surfaced
    // LineError (counter ticks - the recovered-line-error observable), the channel stays LIVE, and
    // later bytes still flow, with no re-arm. The OQ1 split: a line error counts `line_errors` (a wire
    // glitch), NOT `lap_overruns` (a slow-consumer loss).
    let _g = setup();
    let (mut s, ch, ptr) = split_ring(8);

    // 3 bytes were mid-stream when the glitch hit; the resync drops them.
    dma_write(ptr, 0, 0xAA);
    dma_write(ptr, 1, 0xBB);
    dma_write(ptr, 2, 0xCC);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 3);
    set_stat(STAT_ORERR); // the shared ISR records the line error for the DMA path
    fire_usart();

    let mut out = [0u8; 8];
    assert_eq!(
        s.read(&mut out),
        Ok(0),
        "line error absorbed; the 3 disturbed bytes dropped by the resync"
    );
    assert_eq!(s.line_errors(), 1, "the recovered line error counted once");
    assert_eq!(
        s.lap_overruns(),
        0,
        "a line error is NOT a lap-overrun (the OQ1 split)"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "channel stays LIVE (self-heal), not disabled"
    );

    // The channel stayed live: a fresh byte past the resync point (index 3) still arrives.
    dma_write(ptr, 3, 0xEE);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 4);
    assert_eq!(s.read(&mut out), Ok(1));
    assert_eq!(out[0], 0xEE);
    assert_eq!(
        s.line_errors(),
        1,
        "no further counting once the line is clean"
    );
    assert_eq!(s.lap_overruns(), 0, "still no lap");
}

#[test]
fn split_dma_counts_a_noise_only_frame_as_a_line_error() {
    // A frame whose ONLY defect is noise reaches the adapter's `line_errors` counter, which is what
    // the firmware publishes as the inter-board port's `CTRL_OBS` word 13. Before this it reached
    // nothing: `Status` could not represent NERR, the ISR recognised no flag, cleared none, and the
    // counter stayed at 0 while the level-sensitive USART line re-entered the vector until the
    // watchdog reset the board (bench evidence 2026-10-23, where `link_line_errors` read 0 beside a
    // lap-overrun of 1 on a line that had plainly been disturbed).
    let _g = setup();
    let (mut s, ch, ptr) = split_ring(8);

    dma_write(ptr, 0, 0xAA);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 1);
    set_stat(STAT_NERR); // noise alone: no overrun, no framing, no parity
    fire_usart();

    let mut out = [0u8; 8];
    assert_eq!(s.read(&mut out), Ok(0), "the disturbed byte is dropped");
    assert_eq!(
        s.line_errors(),
        1,
        "a noise-only frame is counted as a line error (CTRL_OBS word 13)"
    );
    assert_eq!(s.lap_overruns(), 0, "it is a wire glitch, not a lap");
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "channel stays LIVE: noise self-heals like any other line disturbance"
    );

    // The line is clean again and bytes still flow.
    set_stat(0);
    dma_write(ptr, 1, 0xEE);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 2);
    assert_eq!(s.read(&mut out), Ok(1));
    assert_eq!(out[0], 0xEE);
    assert_eq!(s.line_errors(), 1, "counted once, not per read");
}

#[test]
fn split_dma_into_parts_release_rearm_flows_again() {
    // D5: the reconfigure path through the adapter (into_parts -> release -> re-arm -> new).
    let _g = setup();
    let (s, _ch, _ptr) = split_ring(8);
    let (tx, ring) = s.into_parts();
    let half = ring.release();

    let c = chip();
    let buf2: &'static mut [u8] = vec![0u8; 8].leak();
    let ptr2 = buf2.as_mut_ptr();
    let ch = DmaRxMap::usart1_rx(&c).channel;
    let ring2 = RingBufferedRx::new(&c, half, PeriphLabel::Usart1, buf2).unwrap();
    let mut s = SplitSerial::new(tx, ring2, TEST_CORE_HZ);

    dma_write(ptr2, 0, 0x77);
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 1);
    let mut out = [0u8; 2];
    assert_eq!(s.read(&mut out), Ok(1));
    assert_eq!(out[0], 0x77);
}

#[test]
fn split_write_goes_out_the_tx_half() {
    let _g = setup();
    let (mut s, _ch, _ptr) = split_ring(8);
    set_stat(STAT_TBE | STAT_TC);
    assert_eq!(s.write(&[0xA5]), Ok(1));
    assert_eq!(tx_byte(), 0xA5);
    assert_eq!(s.flush(), Ok(()));
    assert_eq!(s.write_ready(), Ok(true));
}

// ================================ SplitSerial<BufferedRx> =======================================

/// A `SplitSerial<BufferedRx>` over the bench USART (interrupt path, ring capacity word `N`).
fn split_buffered<const N: usize>() -> SplitSerial<BufferedRx> {
    // The ISR's drain loop depends on the silicon rule that a data-register read clears RBNE;
    // declare it for this suite's F10x offsets (the harness owns the device model).
    mock::read_clears(USART_BASE + DATA_OFF, USART_BASE + STAT_OFF, STAT_RBNE);
    let c = chip();
    let (tx, rx) = bring_up().split();
    let storage: &'static RxRing<N> = Box::leak(Box::new(RxRing::new()));
    let b = BufferedRx::new(&c, rx, PeriphLabel::Usart1, storage).unwrap();
    install_mock(IrqLayout::F10xSeparate, RAM_ADDR);
    SplitSerial::new(tx, b, TEST_CORE_HZ)
}

#[test]
fn split_buffered_reads_isr_staged_bytes() {
    let _g = setup();
    let mut s = split_buffered::<8>();

    assert_eq!(s.read_ready(), Ok(false));
    for b in [0x11u8, 0x22, 0x33] {
        seed_rx(b);
        fire_usart();
    }
    assert_eq!(s.read_ready(), Ok(true));
    let mut out = [0u8; 8];
    assert_eq!(s.read(&mut out), Ok(3));
    assert_eq!(&out[..3], &[0x11, 0x22, 0x33]);
    assert_eq!(s.read(&mut out), Ok(0), "drained: non-blocking empty");
}

#[test]
fn split_buffered_absorbs_ring_overflow_and_continues() {
    // Ring capacity word 4 = 3 usable slots; a 4th ISR byte overflows -> sticky Overrun. The
    // adapter absorbs it (counter), the buffered bytes still arrive, and ReadReady saw the pending
    // condition as progress. The OQ1 split: a ring-full overflow is a buffer-overrun loss (the
    // consumer fell behind), so it counts `lap_overruns`, NOT `line_errors`.
    let _g = setup();
    let mut s = split_buffered::<4>();

    for b in [1u8, 2, 3, 4] {
        seed_rx(b);
        fire_usart();
    }
    assert_eq!(s.read_ready(), Ok(true));
    let mut out = [0u8; 8];
    // First read surfaces-and-absorbs the overflow, then (same call, retried drain) the 3 bytes.
    assert_eq!(s.read(&mut out), Ok(3));
    assert_eq!(&out[..3], &[1, 2, 3]);
    assert_eq!(
        s.lap_overruns(),
        1,
        "the ring-full overflow counted once as a lap-overrun"
    );
    assert_eq!(
        s.line_errors(),
        0,
        "a buffer overflow is NOT a line error (the OQ1 split)"
    );
}

#[test]
fn split_buffered_consumes_the_idle_latch_invisibly() {
    let _g = setup();
    let mut s = split_buffered::<8>();
    set_stat(STAT_IDLEF);
    fire_usart();
    let mut out = [0u8; 4];
    assert_eq!(s.read(&mut out), Ok(0));
    let (_tx, b) = s.into_parts();
    assert!(!b.take_idle(), "the adapter consumed the latch");
}

// --- the write path's time bound (`specs/serial-adapters.md`; the i2c acquisition-budget twin) ---
//
// The defect these pin: every wait on the write path used to be an unbounded `while !flag {}`, on
// a path a consuming main loop takes 125 times a second inside a 500 ms watchdog. The offroad
// master took eleven watchdog resets in one bench session on 2026-10-22 with no other unbounded
// wait in its loop.

/// A write whose `TBE` never comes RETURNS, counts the timeout, and reports the buffer consumed.
/// Reporting it consumed is the contract, not a convenience: `Ok(0)` on a non-empty buffer makes
/// `embedded-io`'s `write_all` panic, and the consumer is a panic-free image.
#[test]
fn polled_write_times_out_instead_of_spinning_forever() {
    let _g = setup();
    let mut s = polled();
    set_stat(0); // TBE never comes: the register stays full
    assert_eq!(
        s.write(&[1, 2, 3]),
        Ok(3),
        "the frame is dropped, not retried"
    );
    assert_eq!(s.tx_timeouts(), 1);
    // The port is still usable the moment the flag returns.
    set_stat(STAT_TBE);
    assert_eq!(s.write(&[0x42]), Ok(1));
    assert_eq!(s.tx_timeouts(), 1, "a healthy write counts nothing");
}

/// `flush`'s `TC` wait is bounded by the same budget and counted the same way.
#[test]
fn polled_flush_times_out_instead_of_spinning_forever() {
    let _g = setup();
    let mut s = polled();
    set_stat(STAT_TBE); // TBE set, TC never
    assert_eq!(s.flush(), Ok(()));
    assert_eq!(s.tx_timeouts(), 1);
}

/// The split adapter no longer waits for transmission to COMPLETE per byte. TC is left clear (the
/// previous byte is still on the wire) and the write must still land: on the 9600-baud BLE port
/// that wait was 1.04 ms of main loop per metered byte, and on the inter-board link it was ~10
/// percent of the consuming board's whole CPU.
#[test]
fn split_write_does_not_wait_for_transmission_to_complete() {
    let _g = setup();
    let (mut s, _ch, _p) = split_ring(32);
    set_stat(STAT_TBE); // TBE only: the register is free, the wire is not
    assert_eq!(s.write(&[0x5A]), Ok(1));
    assert_eq!(tx_byte(), 0x5A);
    assert_eq!(s.tx_timeouts(), 0);
}

/// ...and its waits are bounded, both of them.
#[test]
fn split_write_and_flush_are_bounded() {
    let _g = setup();
    let (mut s, _ch, _p) = split_ring(32);
    set_stat(0);
    assert_eq!(s.write(&[1, 2, 3]), Ok(3));
    assert_eq!(s.tx_timeouts(), 1);
    set_stat(STAT_TBE); // TBE back, TC still never: flush is the one that waits for TC now
    assert_eq!(s.flush(), Ok(()));
    assert_eq!(s.tx_timeouts(), 2);
}

/// The budget is a WALL-CLOCK bound, which is the whole reason it is derived from the core clock
/// rather than being a magic iteration count: the same count is 4 ms at 72 MHz and 36 ms at
/// 8 MHz, and 36 ms is past the consumer's 16 ms motor coast.
#[test]
fn the_write_budget_is_a_wall_clock_bound_at_every_clock() {
    for &hz in &[8_000_000u32, 48_000_000, 72_000_000] {
        let us = worst_case_write_block_us(hz);
        // Within one poll iteration of the target either way: the iteration count is an integer,
        // so the microseconds it buys truncate.
        assert!(
            us.abs_diff(WRITE_BUDGET_US) <= WRITE_POLL_CYCLES,
            "at {hz} Hz the bound is {us} us, not ~{WRITE_BUDGET_US}"
        );
        assert!(
            us < 16_000,
            "the bound must stay inside the 16 ms motor coast"
        );
    }
    // Derived, not fixed: a faster clock spends more iterations to buy the same microseconds.
    assert!(write_budget(72_000_000) > write_budget(8_000_000));
    assert_eq!(write_budget(0), 1, "a nonsensical clock still terminates");
}
