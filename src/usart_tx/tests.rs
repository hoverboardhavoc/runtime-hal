//! Host (mock) tests for the one-shot DMA transmit ([`DmaTx`]), G-DMA-TX cases 1-7
//! (`specs/uart-tx-dma.md` section 5.1).
//!
//! The same shape as the DMA-ring RX cases: bring the USART up against the mock register space, arm
//! the transmitter on the split TX half, then drive the state machine by staging DMA `INTF` and
//! firing the real vector through `mock_vtor::dispatch`, which resolves the slot the way hardware
//! does after the `VTOR` flip. Nothing here calls an ISR body directly, so the grouped-vector case
//! exercises the routing and not just the handler.
//!
//! Two things the mock backend supplies that silicon supplies for free, both declared by the
//! harness rather than manufactured by the driver: the DMA `INTC` -> `INTF` write-1-to-clear pair,
//! and the cycle counter the force-restart deadline is measured against
//! (`irq::mock_advance_cycles`, since there is no DWT under `cargo test`).
#![cfg(feature = "mock")]

use super::{DmaTx, TxBusy};
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, UsartConfig, UsartFrame};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::dma::{DmaTxMap, DMA0_BASE};
use crate::error::DescriptorError;
use crate::irq::{
    install_mock, mock_advance_cycles, mock_reset_cycles, mock_vtor, F10X_DMA0_CH1_IRQ,
    F10X_DMA0_CH6_IRQ, F1X0_DMA_CH1_2_IRQ, F1X0_DMA_CH3_4_IRQ,
};
use crate::reg::{mock, Reg32};
use crate::usart::Usart;
use crate::usart_rx::RingBufferedRx;

use std::sync::MutexGuard;

/// The bench USART1 base in the mock window (the offsets within it are what assertions key on).
const USART_BASE: u32 = 0x4000_4400;
/// The F10x BLE-module USART (GD `USART2`) base.
const MODULE_BASE: u32 = 0x4000_4800;
/// The F1x0 BLE-module USART (GD `USART0`, PB6/PB7) base.
const USART0_BASE: u32 = 0x4001_3800;
/// A non-zero RAM-table address for `install_mock` (stands in for the section).
const RAM_ADDR: u32 = 0x2000_4000;

/// The core clock and baud the deadline is computed from: the inter-board link's own numbers, so
/// the budgets the cases step over are the ones the port runs with.
const CORE_HZ: u32 = 72_000_000;
const BAUD: u32 = 460_800;
/// The BLE module's baud (`ble::at::BAUD` in the consuming firmware).
const MODULE_BAUD: u32 = 9_600;

// CHxCTL bits (identical on both families).
const CHEN: u32 = 1 << 0;
const FTFIE: u32 = 1 << 1;
const HTFIE: u32 = 1 << 2;
const DIR: u32 = 1 << 4;
const CMEN: u32 = 1 << 5;
const MNAGA: u32 = 1 << 7;
/// CTL2 `DENT`, the DMA-transmission enable the bring-up sets.
const CTL2_DENT: u32 = 1 << 7;

/// Per-family register offsets + the matching IRQ layout / transmit vectors.
struct Fam {
    ctl2: u32,
    clock: ClockPath,
    irq: IrqLayout,
    /// The vector the link's transmit completion arrives on: its own Ch6 vector on F10x, the
    /// grouped Ch3/4 vector on F1x0.
    dispatch: usize,
    /// The BLE module port's USART on this family (GD `USART2` on F10x, `USART0` on F1x0), its
    /// register base, and the vector its transmit completion arrives on (its own Ch1 vector on
    /// F10x, the grouped Ch1/2 vector on F1x0).
    module: PeriphLabel,
    module_base: u32,
    module_dispatch: usize,
}

fn f10x() -> Fam {
    Fam {
        ctl2: 0x14,
        clock: ClockPath::F10xRcc,
        irq: IrqLayout::F10xSeparate,
        dispatch: F10X_DMA0_CH6_IRQ,
        module: PeriphLabel::Usart2,
        module_base: MODULE_BASE,
        module_dispatch: F10X_DMA0_CH1_IRQ,
    }
}

fn f1x0() -> Fam {
    Fam {
        ctl2: 0x08,
        clock: ClockPath::F1x0Rcu,
        irq: IrqLayout::F1x0Grouped,
        dispatch: F1X0_DMA_CH3_4_IRQ,
        module: PeriphLabel::Usart0,
        module_base: USART0_BASE,
        module_dispatch: F1X0_DMA_CH1_2_IRQ,
    }
}

/// The module port's transmit channel: 1 on both families.
const MODULE_CH: u8 = 1;

fn chip_for(fam: &Fam) -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    addrs.set(PeriphLabel::Usart2, MODULE_BASE);
    addrs.set(PeriphLabel::Usart0, USART0_BASE);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    Chip::from_descriptor(McuDescriptor {
        gpio: if fam.clock == ClockPath::F1x0Rcu {
            GpioPath::AhbCtlAfsel
        } else {
            GpioPath::ApbCrlCrh
        },
        clock: fam.clock,
        adc: AdcPath::Single,
        irq: fam.irq,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

fn link_cfg() -> UsartConfig {
    UsartConfig {
        usart: PeriphLabel::Usart1,
        baud: BAUD,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    }
}

fn module_cfg(usart: PeriphLabel) -> UsartConfig {
    UsartConfig {
        usart,
        baud: MODULE_BAUD,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    }
}

/// Acquire the whole-case serialization lock and zero the register space, the DMA contexts, the
/// recorded VTOR and the stand-in cycle counter (a fresh world per case). Declares the one silicon
/// rule these cases depend on: writing DMA `INTC` clears the written bits in `INTF`.
fn setup() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    mock::w1c_pair(DMA0_BASE + 0x04, DMA0_BASE);
    crate::usart_rx::reset_for_test();
    crate::dma::reset_for_test();
    mock_vtor::reset();
    mock_reset_cycles();
    g
}

fn ch_ctl(ch: u8) -> u32 {
    0x08 + 0x14 * ch as u32
}
fn ch_cnt(ch: u8) -> u32 {
    0x0C + 0x14 * ch as u32
}
fn ch_paddr(ch: u8) -> u32 {
    0x10 + 0x14 * ch as u32
}
fn ch_maddr(ch: u8) -> u32 {
    0x14 + 0x14 * ch as u32
}
/// INTF/INTC per-channel full-transfer flag: `flag << 4*channel`, FTFIF at +1.
fn ftf_flag(ch: u8) -> u32 {
    1 << (4 * ch as u32 + 1)
}
fn gif_flag(ch: u8) -> u32 {
    1 << (4 * ch as u32)
}

/// Bring up USART1 and arm a `DmaTx` of capacity `N` on its TX half, with the RAM table flipped for
/// `fam`'s layout. Returns the transmitter, the resolved channel, and the `TDATA` address the
/// channel should have been pointed at.
fn arm<const N: usize>(fam: &Fam) -> (DmaTx, u8, u32, *const u8) {
    let chip = chip_for(fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &link_cfg()).unwrap();
    let (tx_half, _rx_half) = usart.split();
    let tdata = tx_half.regs().tdata_addr();
    let ch = DmaTxMap::usart1_tx(&chip).channel;
    let buf: &'static mut [u8] = Box::leak(Box::new([0u8; N]));
    // The real host pointer: `CHxMADDR` holds it truncated to 32 bits (the register is 32-bit and
    // the host's addresses are not), so the buffer is inspected through this and the register is
    // only ever compared against the same truncation.
    let ptr: *const u8 = buf.as_ptr();
    let tx = DmaTx::new(&chip, tx_half, buf, CORE_HZ, BAUD).unwrap();
    install_mock(fam.irq, RAM_ADDR);
    (tx, ch, tdata, ptr)
}

/// Bring up the family's module USART and arm a `DmaTx` of capacity `N` on its TX half. Does NOT
/// install the RAM table, so a case can arm the link's transmitter as well before flipping it.
/// Returns the transmitter, the `TDATA` address and the owned buffer's pointer.
fn arm_module<const N: usize>(fam: &Fam) -> (DmaTx, u32, *const u8) {
    let chip = chip_for(fam);
    let usart = Usart::bring_up(
        &chip,
        &ClockConfig::REFERENCE_72M_IRC8M,
        &module_cfg(fam.module),
    )
    .unwrap();
    let (tx_half, _rx_half) = usart.split();
    let tdata = tx_half.regs().tdata_addr();
    let buf: &'static mut [u8] = Box::leak(Box::new([0u8; N]));
    let ptr: *const u8 = buf.as_ptr();
    let tx = DmaTx::new(&chip, tx_half, buf, CORE_HZ, MODULE_BAUD).unwrap();
    (tx, tdata, ptr)
}

/// Fire the link's transmit completion vector the way hardware would.
fn fire(fam: &Fam) {
    // SAFETY: the RAM table is installed (install_mock) and the slot holds a handler.
    unsafe { mock_vtor::dispatch(fam.dispatch) };
}

/// Fire the module port's transmit completion vector the way hardware would.
fn fire_module(fam: &Fam) {
    // SAFETY: the RAM table is installed (install_mock) and the slot holds a handler.
    unsafe { mock_vtor::dispatch(fam.module_dispatch) };
}

/// Raise the channel's full-transfer flag in `INTF`, as the hardware does when `CHxCNT` hits zero.
fn complete(ch: u8) {
    Reg32::new(DMA0_BASE, 0x00).modify(ftf_flag(ch), ftf_flag(ch));
}

/// The deadline, in cycles, for a `len`-byte frame: the same wire-time-times-slack the transmitter
/// computes, written out here so a case steps the stand-in counter to a known side of it rather
/// than to a number copied from the implementation.
fn deadline(len: usize) -> u32 {
    let per_byte = (CORE_HZ as u64) * 10 / (BAUD as u64) * 4;
    (per_byte * len as u64) as u32
}

// --- The map resolves the family's transmit channel -------------------------------------------

#[test]
fn tx_map_resolves_channel_base_and_grouping() {
    let f103 = DmaTxMap::usart1_tx(&chip_for(&f10x()));
    assert_eq!(f103.channel, 6, "USART1_TX is DMA0 Ch6 on F10x");
    assert_eq!(f103.base, DMA0_BASE);
    assert!(!f103.grouped, "F10x DMA0_Channel6 has its own vector");

    let f130 = DmaTxMap::usart1_tx(&chip_for(&f1x0()));
    assert_eq!(f130.channel, 3, "USART1_TX is Ch3 on F1x0");
    assert_eq!(f130.base, DMA0_BASE);
    assert!(f130.grouped, "F1x0 Ch3/4 share a grouped vector");
}

// --- Case 1: an idle channel takes the frame, reports the count, reports in-flight -------------

#[test]
fn c1_idle_channel_takes_the_frame_and_reports_in_flight() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let (mut tx, ch, tdata, src) = arm::<32>(&fam);
        assert!(
            !tx.in_flight(),
            "nothing is in flight before the first send"
        );

        let frame = [0xA0u8, 0xA1, 0xA2, 0xA3, 0xA4];
        assert_eq!(tx.try_send(&frame), Ok(5), "the whole frame was taken");
        assert!(tx.in_flight(), "the transfer is in flight");

        // The channel points at TDATA (fixed), reads from the owned buffer (incrementing), for
        // exactly the frame's length.
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_paddr(ch)).read(),
            tdata,
            "CHxPADDR = USART TDATA address"
        );
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_cnt(ch)).read(),
            5,
            "CHxCNT = the frame length"
        );
        let ctl = Reg32::new(DMA0_BASE, ch_ctl(ch)).read();
        assert_eq!(ctl & DIR, DIR, "DIR=1 mem->periph");
        assert_eq!(ctl & MNAGA, MNAGA, "memory-increment set");
        assert_eq!(ctl & (1 << 6), 0, "PNAGA=0 peripheral fixed (TDATA)");
        assert_eq!(ctl & CMEN, 0, "CMEN clear: one shot, not circular");
        assert_eq!(ctl & FTFIE, FTFIE, "full-transfer interrupt enabled");
        assert_eq!(ctl & HTFIE, 0, "half-transfer interrupt off");
        assert_eq!(ctl & (0xF << 8), 0, "8-bit widths");
        assert_eq!(ctl & CHEN, CHEN, "CHEN set (channel started)");

        // The USART routes its transmit fetches to the DMA controller.
        assert_eq!(
            Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENT,
            CTL2_DENT,
            "DENT set"
        );

        // The channel reads out of the owned buffer, and the buffer holds the frame in order.
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_maddr(ch)).read(),
            src as u32,
            "CHxMADDR = the owned buffer"
        );
        for (i, &b) in frame.iter().enumerate() {
            // SAFETY: the address is the leaked 32-byte buffer the transmitter owns.
            assert_eq!(unsafe { *src.add(i) }, b, "buffer byte {i}");
        }
    }
}

// --- Case 2: a busy channel refuses and touches nothing ---------------------------------------

#[test]
fn c2_busy_channel_refuses_and_does_not_touch_the_buffer() {
    let fam = f10x();
    let _g = setup();
    let (mut tx, ch, _tdata, src) = arm::<32>(&fam);

    assert_eq!(tx.try_send(&[1, 2, 3, 4]), Ok(4));
    let before: Vec<u8> = (0..4).map(|i| unsafe { *src.add(i) }).collect();

    // Well inside the deadline (no cycles have passed at all).
    assert_eq!(
        tx.try_send(&[9, 9, 9, 9]),
        Err(TxBusy),
        "a transfer is in flight"
    );
    let after: Vec<u8> = (0..4).map(|i| unsafe { *src.add(i) }).collect();
    assert_eq!(before, after, "the refused frame did not reach the buffer");
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_cnt(ch)).read(),
        4,
        "the running transfer was not reprogrammed"
    );
    assert_eq!(tx.timeouts(), 0, "a refusal is not a timeout");
}

// --- Case 3: the completion clears in-flight and the next send is taken ------------------------

#[test]
fn c3_completion_clears_in_flight_and_the_next_send_is_taken() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let (mut tx, ch, _tdata, _src) = arm::<32>(&fam);
        assert_eq!(tx.try_send(&[1, 2, 3]), Ok(3));

        complete(ch);
        fire(&fam);

        assert!(!tx.in_flight(), "the completion cleared the in-flight flag");
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
            0,
            "the completion cleared CHEN"
        );
        let intf = Reg32::new(DMA0_BASE, 0x00).read();
        assert_eq!(intf & ftf_flag(ch), 0, "the channel's FTFIF was cleared");

        assert_eq!(tx.try_send(&[4, 5]), Ok(2), "the next frame is taken");
        assert!(tx.in_flight());
        assert_eq!(tx.timeouts(), 0, "a clean completion is not a timeout");
    }
}

// --- Case 4: a frame longer than N is truncated, not refused -----------------------------------

#[test]
fn c4_over_long_frame_is_truncated_to_n() {
    let fam = f10x();
    let _g = setup();
    let (mut tx, ch, _tdata, src) = arm::<4>(&fam);

    let frame = [1u8, 2, 3, 4, 5, 6, 7];
    assert_eq!(tx.try_send(&frame), Ok(4), "N bytes taken, not an error");
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_cnt(ch)).read(),
        4,
        "the transfer is N long"
    );
    for i in 0..4usize {
        assert_eq!(unsafe { *src.add(i) }, frame[i], "the first N bytes");
    }
}

// --- Case 5: an empty frame starts no transfer -------------------------------------------------

#[test]
fn c5_empty_frame_starts_no_transfer() {
    let fam = f10x();
    let _g = setup();
    let (mut tx, ch, _tdata, _src) = arm::<32>(&fam);

    assert_eq!(tx.try_send(&[]), Ok(0));
    assert!(!tx.in_flight(), "nothing is in flight");
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "no channel was armed"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_cnt(ch)).read(),
        0,
        "CHxCNT was never programmed"
    );
}

// --- Case 6: the deadline passes, so the next send force-restarts and counts it ----------------

#[test]
fn c6_elapsed_budget_force_restarts_counts_and_takes_the_frame() {
    let fam = f10x();
    let _g = setup();
    let (mut tx, ch, _tdata, _src) = arm::<32>(&fam);

    // A transfer that never completes: no completion vector is ever fired for it.
    assert_eq!(tx.try_send(&[1, 2, 3, 4, 5, 6, 7, 8]), Ok(8));
    // Hardware also left a stale flag behind on the abandoned channel; the restart must clear it,
    // or the next transfer's completion ISR fires on this one's flag.
    complete(ch);

    // Just inside the deadline: still refused, still not counted.
    mock_advance_cycles(deadline(8) - 1);
    assert_eq!(tx.try_send(&[0xEE]), Err(TxBusy), "inside the deadline");
    assert_eq!(tx.timeouts(), 0);

    // Past it: the wedged transfer is abandoned, counted, and the new frame goes.
    mock_advance_cycles(1);
    assert_eq!(tx.try_send(&[0xEE, 0xEF]), Ok(2), "the new frame is taken");
    assert_eq!(tx.timeouts(), 1, "one abandoned transfer counted");
    assert!(tx.in_flight(), "the replacement transfer is running");
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_cnt(ch)).read(),
        2,
        "the channel carries the new frame"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, 0x00).read() & (0xF << (4 * ch as u32)),
        0,
        "the abandoned transfer's flags were cleared"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "the replacement is armed"
    );
}

// --- Case 8: readiness turns true at the deadline, so a ready-gated caller reaches the restart --

/// A caller that only sends when the transmitter is ready (the consuming firmware's BLE owner) must
/// still recover a transfer whose completion never arrives. `ready()` is false while the transfer
/// is inside its deadline and true from the deadline on, WITHOUT touching anything itself; the send
/// it lets through is what abandons the transfer and counts it. Against `!in_flight()` as the
/// readiness, this caller would never send again.
#[test]
fn c8_ready_turns_true_at_the_deadline_so_a_ready_gated_caller_recovers() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let (mut tx, ch, _tdata, _src) = arm::<32>(&fam);
        assert!(tx.ready(), "idle: ready");

        assert_eq!(tx.try_send(&[1, 2, 3, 4]), Ok(4));
        assert!(!tx.ready(), "in flight inside the deadline: not ready");

        // The completion is lost: nothing fires. Just inside the deadline, still not ready.
        mock_advance_cycles(deadline(4) - 1);
        assert!(!tx.ready());
        // At the deadline: ready, and asking changed nothing.
        mock_advance_cycles(1);
        assert!(
            tx.ready(),
            "{:?}: past the deadline a send would go",
            fam.clock
        );
        assert!(tx.in_flight(), "ready() abandons nothing by itself");
        assert_eq!(tx.timeouts(), 0, "and counts nothing by itself");

        // The send it lets through restarts the channel and counts the lost transfer.
        assert_eq!(tx.try_send(&[5, 6]), Ok(2));
        assert_eq!(tx.timeouts(), 1);
        assert_eq!(Reg32::new(DMA0_BASE, ch_cnt(ch)).read(), 2);
        assert!(!tx.ready(), "the new transfer has a fresh deadline");
    }
}

// --- Case 7: the F1x0 grouped vector services whichever channels are flagged, including both ---

/// The case the shape of the handler is chosen for. Ch3 (the link's TX) and Ch4 (its RX) share
/// `DMA_Channel3_4_IRQn` = 11. One entry must service whichever of them has a flag set, and an
/// entry with BOTH set must service BOTH: a demux that returned after its first match would pass
/// the two single-channel assertions here and silently drop the other channel's event when they
/// coincide.
#[test]
fn c7_grouped_vector_services_ch3_ch4_and_both() {
    let fam = f1x0();
    let _g = setup();

    // Both halves of the link live on the one vector, as they are in the shipping image.
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &link_cfg()).unwrap();
    let (tx_half, rx_half) = usart.split();
    let dma_buf: &'static mut [u8] = vec![0u8; 16].leak();
    let rx = RingBufferedRx::new(&chip, rx_half, PeriphLabel::Usart1, dma_buf).unwrap();
    let txbuf: &'static mut [u8] = Box::leak(Box::new([0u8; 32]));
    let mut tx = DmaTx::new(&chip, tx_half, txbuf, CORE_HZ, BAUD).unwrap();
    install_mock(fam.irq, RAM_ADDR);
    let (tx_ch, rx_ch) = (3u8, 4u8);
    assert_eq!(DmaTxMap::usart1_tx(&chip).channel, tx_ch);

    // (a) Ch3 alone: the transmit completion is serviced, the receive wrap counter is untouched.
    assert_eq!(tx.try_send(&[1, 2, 3]), Ok(3));
    complete(tx_ch);
    fire(&fam);
    assert!(!tx.in_flight(), "the Ch3 completion was serviced");
    assert_eq!(crate::dma::wraps(0), 0, "no receive wrap was invented");

    // (b) Ch4 alone: the receive wrap is counted and the transmitter is left alone, mid-transfer.
    assert_eq!(tx.try_send(&[4, 5, 6]), Ok(3));
    complete(rx_ch);
    fire(&fam);
    assert_eq!(crate::dma::wraps(0), 1, "the Ch4 wrap was counted");
    assert!(
        tx.in_flight(),
        "a receive-only entry must not clear the transmit in-flight flag"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(tx_ch)).read() & CHEN,
        CHEN,
        "the running transmit channel is still armed"
    );

    // (c) BOTH flagged in one entry: both are serviced.
    complete(tx_ch);
    complete(rx_ch);
    fire(&fam);
    assert!(!tx.in_flight(), "the transmit completion was serviced");
    assert_eq!(crate::dma::wraps(0), 2, "the receive wrap was counted too");
    let intf = Reg32::new(DMA0_BASE, 0x00).read();
    assert_eq!(intf & ftf_flag(tx_ch), 0, "Ch3 FTFIF cleared");
    assert_eq!(intf & ftf_flag(rx_ch), 0, "Ch4 FTFIF cleared");
    assert_eq!(intf & gif_flag(tx_ch), 0, "Ch3 GIF cleared");
    assert_eq!(intf & gif_flag(rx_ch), 0, "Ch4 GIF cleared");

    // The receiver is held to the end of the case on purpose: both halves must be live for the
    // shared vector to be the shared vector.
    let _ = rx;
}

// --- Bring-up: fail loud, and refuse a handle this HAL has no channel for ----------------------

/// The write-back self-check is the one reliable DMA probe on this silicon: a channel that does not
/// respond must fail the bring-up with nothing armed and `DENT` left clear, rather than hand back a
/// transmitter whose frames vanish.
#[test]
fn self_check_failure_arms_nothing() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &link_cfg()).unwrap();
    let ch = DmaTxMap::usart1_tx(&chip).channel;
    let buf: &'static mut [u8] = Box::leak(Box::new([0u8; 32]));

    mock::freeze(DMA0_BASE + ch_maddr(ch));

    assert!(
        matches!(
            DmaTx::new(&chip, usart.split().0, buf, CORE_HZ, BAUD),
            Err(DescriptorError::SelfCheckFailed)
        ),
        "a channel that does not respond fails loud"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "no channel armed on a failed self-check"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENT,
        0,
        "DENT not set on a failed self-check"
    );
}

/// The module port's instance is per family, as on the receive side: `USART2` is the module on F10x
/// and `USART0` on F1x0. The OTHER family's module label resolves to no transmit channel here (on
/// F10x, `USART0`'s PB6/PB7 mapping needs an AFIO remap this HAL does not implement; on F1x0 there is
/// no `USART2`), so a half from it is refused instead of being armed onto channel 1, whose request
/// line belongs to a different USART.
#[test]
fn a_half_with_no_mapped_transmit_channel_is_refused() {
    for (fam, other, other_base) in [
        (f10x(), PeriphLabel::Usart0, USART0_BASE),
        (f1x0(), PeriphLabel::Usart2, MODULE_BASE),
    ] {
        let _g = setup();
        let chip = chip_for(&fam);
        // The USART itself comes up (the instance exists in the descriptor); it is the transmit
        // channel that has no mapping.
        let usart =
            Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg(other)).unwrap();
        let buf: &'static mut [u8] = Box::leak(Box::new([0u8; 32]));

        assert!(
            matches!(
                DmaTx::new(&chip, usart.split().0, buf, CORE_HZ, MODULE_BAUD),
                Err(DescriptorError::Unsupported)
            ),
            "{other:?} has no transmit channel on {:?}",
            fam.clock
        );
        assert_eq!(
            Reg32::new(other_base, fam.ctl2).read() & CTL2_DENT,
            0,
            "the refused instance's DENT is untouched"
        );
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_ctl(MODULE_CH)).read() & CHEN,
            0,
            "channel 1 was not armed for a USART that does not own it"
        );
    }
}

// --- The module port: its own channel, vector and in-flight state ------------------------------

#[test]
fn tx_map_resolves_the_module_port_channel_per_family() {
    let f103 = DmaTxMap::module_tx();
    assert_eq!(f103.channel, 1, "GD USART2_TX is DMA0 Ch1 on F10x");
    assert_eq!(f103.base, DMA0_BASE);
    assert!(!f103.grouped, "F10x DMA0_Channel1 has its own vector");

    let f130 = DmaTxMap::usart0_tx();
    assert_eq!(f130.channel, 1, "USART0_TX is Ch1 on F1x0, unremapped");
    assert_eq!(f130.base, DMA0_BASE);
    assert!(f130.grouped, "F1x0 Ch1/2 share a grouped vector");

    // Never the link's channel on either family, so neither port can program the other's.
    assert_ne!(
        f103.channel,
        DmaTxMap::usart1_tx(&chip_for(&f10x())).channel
    );
    assert_ne!(
        f130.channel,
        DmaTxMap::usart1_tx(&chip_for(&f1x0())).channel
    );
}

/// The module port's half arms channel 1 against its own `TDATA`, and its completion arrives on its
/// own vector: Ch1 = IRQ 12 on F10x, the grouped Ch1/2 = IRQ 10 on F1x0.
#[test]
fn m1_module_port_sends_on_channel_1_and_completes_on_its_own_vector() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let (mut tx, tdata, src) = arm_module::<20>(&fam);
        install_mock(fam.irq, RAM_ADDR);

        assert_eq!(
            Reg32::new(fam.module_base, fam.ctl2).read() & CTL2_DENT,
            CTL2_DENT,
            "{:?}: DENT set on the module USART",
            fam.clock
        );

        let frame: Vec<u8> = (0..20).map(|i| 0x30 + i).collect();
        assert_eq!(
            tx.try_send(&frame),
            Ok(20),
            "a 20 B wire frame in one transfer"
        );
        assert!(tx.in_flight());
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_paddr(MODULE_CH)).read(),
            tdata,
            "CH1PADDR = the module USART's TDATA"
        );
        assert_eq!(Reg32::new(DMA0_BASE, ch_cnt(MODULE_CH)).read(), 20);
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_maddr(MODULE_CH)).read(),
            src as u32,
            "CH1MADDR = the owned buffer"
        );
        let ctl = Reg32::new(DMA0_BASE, ch_ctl(MODULE_CH)).read();
        assert_eq!(
            ctl & (DIR | MNAGA | FTFIE | CHEN),
            DIR | MNAGA | FTFIE | CHEN
        );
        assert_eq!(
            ctl & (CMEN | HTFIE),
            0,
            "one shot, no half-transfer interrupt"
        );

        // The LINK's vector does not complete it: its context is not this one.
        complete(MODULE_CH);
        if fam.dispatch != fam.module_dispatch {
            fire(&fam);
            assert!(
                tx.in_flight(),
                "{:?}: the link's vector must not service the module channel",
                fam.clock
            );
        }
        fire_module(&fam);
        assert!(
            !tx.in_flight(),
            "{:?}: the module vector serviced Ch1",
            fam.clock
        );
        assert_eq!(
            Reg32::new(DMA0_BASE, 0x00).read() & ftf_flag(MODULE_CH),
            0,
            "Ch1 FTFIF cleared"
        );
        assert_eq!(tx.try_send(&[1, 2]), Ok(2), "the next frame is taken");
    }
}

/// **The reason the transmit context became one per port.** With both transmitters live, each one's
/// in-flight state is its own: a busy link never refuses a BLE frame, a busy BLE port never refuses
/// a link frame, and a completion on one never clears the other.
#[test]
fn m2_link_and_module_transmitters_are_independent() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        // Both transmitters live under the one installed table (`arm` installs it last).
        let (mut module, _mt, _ms) = arm_module::<20>(&fam);
        let (mut link, link_ch, _t, _s) = arm::<32>(&fam);

        assert_eq!(link.try_send(&[1, 2, 3]), Ok(3));
        assert!(link.in_flight());
        assert!(
            !module.in_flight(),
            "{:?}: the link's claim is not the module's",
            fam.clock
        );
        assert_eq!(
            module.try_send(&[9; 20]),
            Ok(20),
            "{:?}: a busy link does not refuse a BLE frame",
            fam.clock
        );
        assert!(module.in_flight());

        // The module port completes; the link is still mid-transfer.
        complete(MODULE_CH);
        fire_module(&fam);
        assert!(
            !module.in_flight(),
            "{:?}: the module completion serviced",
            fam.clock
        );
        assert!(
            link.in_flight(),
            "{:?}: and did not clear the link",
            fam.clock
        );
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_ctl(link_ch)).read() & CHEN,
            CHEN,
            "the link's channel is still running"
        );

        // A busy module port does not refuse a link frame either.
        assert_eq!(module.try_send(&[7; 20]), Ok(20));
        complete(link_ch);
        fire(&fam);
        assert!(
            !link.in_flight(),
            "{:?}: the link completion serviced",
            fam.clock
        );
        assert!(
            module.in_flight(),
            "{:?}: and did not clear the module port",
            fam.clock
        );
        assert_eq!(link.try_send(&[4]), Ok(1));

        // The force-restart deadlines are each port's own: past the link's 4-byte budget at
        // 460800, the 20-byte module frame at 9600 is still well inside its own.
        mock_advance_cycles(deadline(1) + 1);
        assert_eq!(
            link.try_send(&[5]),
            Ok(1),
            "the link's wedged transfer restarts"
        );
        assert_eq!(link.timeouts(), 1);
        assert_eq!(
            module.try_send(&[6]),
            Err(TxBusy),
            "the module port's does not"
        );
        assert_eq!(module.timeouts(), 0);
    }
}

/// The F1x0 grouped Ch1/2 vector carries both halves of the module port, `USART0_TX` on Ch1 and
/// `USART0_RX` on Ch2: the [`c7_grouped_vector_services_ch3_ch4_and_both`] case for the other
/// grouped vector. One entry services whichever channel is flagged, and both when both are.
#[test]
fn m3_grouped_ch1_2_services_the_module_tx_rx_and_both() {
    let fam = f1x0();
    let _g = setup();

    let chip = chip_for(&fam);
    let usart = Usart::bring_up(
        &chip,
        &ClockConfig::REFERENCE_72M_IRC8M,
        &module_cfg(PeriphLabel::Usart0),
    )
    .unwrap();
    let (tx_half, rx_half) = usart.split();
    let ring: &'static mut [u8] = vec![0u8; 16].leak();
    let rx = RingBufferedRx::new(&chip, rx_half, PeriphLabel::Usart0, ring).unwrap();
    let txbuf: &'static mut [u8] = Box::leak(Box::new([0u8; 20]));
    let mut tx = DmaTx::new(&chip, tx_half, txbuf, CORE_HZ, MODULE_BAUD).unwrap();
    install_mock(fam.irq, RAM_ADDR);
    let (tx_ch, rx_ch) = (MODULE_CH, 2u8);
    // The USART0 ring is the DMA-RX context 2 (`usart_rx`'s slot index for USART0).
    let usart0_wraps = || crate::dma::wraps(2);

    // (a) Ch1 alone: the transmit completion is serviced, no receive wrap is invented.
    assert_eq!(tx.try_send(&[1, 2, 3]), Ok(3));
    complete(tx_ch);
    fire_module(&fam);
    assert!(!tx.in_flight(), "the Ch1 completion was serviced");
    assert_eq!(usart0_wraps(), 0, "no receive wrap was invented");

    // (b) Ch2 alone: the receive wrap is counted and the transmitter is left mid-transfer.
    assert_eq!(tx.try_send(&[4, 5, 6]), Ok(3));
    complete(rx_ch);
    fire_module(&fam);
    assert_eq!(usart0_wraps(), 1, "the Ch2 wrap was counted");
    assert!(
        tx.in_flight(),
        "a receive-only entry must not clear the transmit in-flight flag"
    );

    // (c) BOTH flagged in one entry: both are serviced.
    complete(tx_ch);
    complete(rx_ch);
    fire_module(&fam);
    assert!(!tx.in_flight(), "the transmit completion was serviced");
    assert_eq!(usart0_wraps(), 2, "the receive wrap was counted too");
    let intf = Reg32::new(DMA0_BASE, 0x00).read();
    assert_eq!(intf & (0xF << (4 * tx_ch as u32)), 0, "Ch1 flags cleared");
    assert_eq!(intf & ftf_flag(rx_ch), 0, "Ch2 FTFIF cleared");

    let _ = rx;
}

/// Releasing one port's transmitter leaves the other's armed: `release` uninstalls its OWN context
/// only, so the BLE port can be torn down for a reconfigure without silencing the link.
#[test]
fn m4_releasing_the_module_port_leaves_the_link_armed() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let (module, _mt, _ms) = arm_module::<20>(&fam);
        let (mut link, link_ch, _t, _s) = arm::<32>(&fam);

        let _half = module.release();
        assert_eq!(
            Reg32::new(fam.module_base, fam.ctl2).read() & CTL2_DENT,
            0,
            "the module USART's DENT is cleared"
        );
        assert_eq!(
            Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENT,
            CTL2_DENT,
            "the link's DENT is not"
        );

        assert_eq!(link.try_send(&[1, 2]), Ok(2));
        complete(link_ch);
        fire(&fam);
        assert!(
            !link.in_flight(),
            "{:?}: the link's completion is still serviced",
            fam.clock
        );
    }
}

/// `release` gives the polled half back: the channel is stopped, `DENT` is cleared, and the context
/// is inert, so a later completion cannot act on a port nobody owns.
#[test]
fn release_quiesces_and_the_half_comes_back() {
    let fam = f10x();
    let _g = setup();
    let (mut tx, ch, _tdata, _src) = arm::<32>(&fam);
    assert_eq!(tx.try_send(&[1, 2, 3]), Ok(3));

    let half = tx.release();

    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "the channel was stopped"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENT,
        0,
        "DENT cleared: the half is polled again"
    );
    // A completion arriving after the release is a no-op on an uninstalled context.
    complete(ch);
    fire(&fam);
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "a late completion did not re-arm anything"
    );

    // The returned half is a polled transmitter again: with TBE staged it takes a byte.
    Reg32::new(USART_BASE, 0x00).write(1 << 7);
    assert!(half.try_write_byte(0x55), "the half writes polled again");
    assert_eq!(
        Reg32::new(USART_BASE, 0x04).read() & 0xFF,
        0x55,
        "the byte reached the data register"
    );
}
