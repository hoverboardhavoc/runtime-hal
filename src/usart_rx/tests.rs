//! Host (mock) tests for the interrupt-buffered RX (`BufferedRx`), G-DMA-UART Gate A cases B1-B5.
//!
//! The same shape as the irq host tests: build the RAM vector table with `install_mock`, then fire
//! the USART1 RX vector through `mock_vtor::dispatch`, which resolves the slot to `usart1_rx_isr` and
//! calls the registered ISR body exactly the way hardware would after the `VTOR` flip. The mock
//! register backend is a passive array with no UART core, so `Usart::read_rbne_byte` models the
//! hardware "reading the data register clears RBNE" side effect under the `mock` feature (so the ISR
//! drain loop terminates); each staged byte is one dispatch, mirroring the polled serial RX tests.
#![cfg(feature = "mock")]

use super::{reset_for_test, supports_rx, BufferedRx, RingBufferedRx, RxRing};
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, UsartConfig, UsartFrame};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::dma::{DmaRxMap, DMA0_BASE};
use crate::error::{DescriptorError, UsartError};
use crate::irq::{
    install_mock, mock_vtor, F10X_DMA0_CH2_IRQ, F10X_DMA0_CH5_IRQ, F10X_USART1_IRQ,
    F10X_USART_MODULE_IRQ, F1X0_DMA_CH3_4_IRQ, F1X0_USART1_IRQ,
};
use crate::reg::{mock, Reg32};
use crate::usart::Usart;

use std::sync::MutexGuard;

/// The bench USART1 base in the mock window (the offsets within it are what assertions key on).
const USART_BASE: u32 = 0x4000_4400;
/// The BLE-module USART base (HAL `Usart2` = GD USART2, the F10x second instance). Distinct from
/// `USART_BASE` in the mock window (idx 0x4800 vs 0x4400), so the two slots' register spaces never
/// alias: this is what the two-instances-live coexistence case keys on.
const MODULE_BASE: u32 = 0x4000_4800;
/// A non-zero RAM-table address for `install_mock` (stands in for the section).
const RAM_ADDR: u32 = 0x2000_4000;

// STAT bit positions (identical on both families).
const RBNE: u32 = 1 << 5;
const IDLEF: u32 = 1 << 4;
const ORERR: u32 = 1 << 3;
const FERR: u32 = 1 << 1;
const PERR: u32 = 1 << 0;

/// Per-family register offsets + the matching IRQ layout / dispatch vector.
struct Fam {
    stat: u32,
    data: u32,
    ctl0: u32,
    ctl2: u32,
    intc: Option<u32>,
    clock: ClockPath,
    irq: IrqLayout,
    dispatch: usize,
}

fn f10x() -> Fam {
    Fam {
        stat: 0x00,
        data: 0x04,
        ctl0: 0x0C,
        ctl2: 0x14,
        intc: None,
        clock: ClockPath::F10xRcc,
        irq: IrqLayout::F10xSeparate,
        dispatch: F10X_USART1_IRQ,
    }
}

fn f1x0() -> Fam {
    Fam {
        stat: 0x1C,
        data: 0x24,
        ctl0: 0x00,
        ctl2: 0x08,
        intc: Some(0x20),
        clock: ClockPath::F1x0Rcu,
        irq: IrqLayout::F1x0Grouped,
        dispatch: F1X0_USART1_IRQ,
    }
}

fn chip_for(fam: &Fam) -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    addrs.set(PeriphLabel::Usart2, MODULE_BASE);
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

fn bench_cfg() -> UsartConfig {
    UsartConfig {
        usart: PeriphLabel::Usart1,
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    }
}

/// Acquire the whole-case serialization lock and zero the register space + the static RX context +
/// the recorded VTOR (a fresh world per case).
fn setup() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    reset_for_test();
    crate::dma::reset_for_test();
    mock_vtor::reset();
    g
}

/// Bring up the USART, install `BufferedRx` over a leaked `'static` ring of capacity word `N`, and
/// flip the RAM table for `fam`'s layout so `dispatch` can route the RX vector.
fn install<const N: usize>(fam: &Fam) -> BufferedRx {
    let chip = chip_for(fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let storage: &'static RxRing<N> = Box::leak(Box::new(RxRing::new()));
    let rx = BufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, storage).unwrap();
    install_mock(fam.irq, RAM_ADDR);
    rx
}

fn stage_byte(fam: &Fam, b: u8) {
    stage_byte_at(USART_BASE, fam, b);
}

/// Stage a ready RBNE byte at an arbitrary USART base (so the module instance's register space, at
/// `MODULE_BASE`, can be driven independently of USART1's).
fn stage_byte_at(base: u32, fam: &Fam, b: u8) {
    Reg32::new(base, fam.data).write(b as u32);
    Reg32::new(base, fam.stat).write(RBNE);
}

/// A `UsartConfig` for the BLE-module USART (HAL `Usart2`): the F10x-only second instance. `bring_up`
/// uses only `usart`/`baud`/`frame`/`oversampling` (not the pin bytes), so the placeholder PB10/PB11
/// values are inert here.
fn module_cfg() -> UsartConfig {
    UsartConfig {
        usart: PeriphLabel::Usart2,
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    }
}

/// Bring up a module-USART `BufferedRx` (slot 1) over a leaked `'static` ring of capacity word `N`.
/// Does NOT flip the RAM table (callers that need `dispatch` flip it themselves, so a coexistence case
/// can install both instances before one flip).
fn bring_up_module<const N: usize>(chip: &Chip) -> BufferedRx {
    let usart = Usart::bring_up(chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg()).unwrap();
    let storage: &'static RxRing<N> = Box::leak(Box::new(RxRing::new()));
    BufferedRx::new(chip, usart.split().1, PeriphLabel::Usart2, storage).unwrap()
}

fn fire(fam: &Fam) {
    // SAFETY: the RAM table is installed (install_mock) and the slot holds a handler.
    unsafe { mock_vtor::dispatch(fam.dispatch) };
}

// --- B1: bring-up programs RBNEIE + IDLEIE, leaves DENR clear ----------------------------------

#[test]
fn b1_bringup_sets_rbneie_idleie_and_leaves_denr_clear() {
    let fam = f10x();
    let _g = setup();
    let _rx = install::<8>(&fam);

    let ctl0 = Reg32::new(USART_BASE, fam.ctl0).read();
    assert_eq!(ctl0 & (1 << 5), 1 << 5, "RBNEIE set");
    assert_eq!(ctl0 & (1 << 4), 1 << 4, "IDLEIE set");
    // The polled enable bits from bring_up survive (REN+TEN+UEN; UEN is bit 13 on F10x).
    assert_eq!(
        ctl0 & ((1 << 2) | (1 << 3) | (1 << 13)),
        (1 << 2) | (1 << 3) | (1 << 13),
        "REN+TEN+UEN preserved"
    );
    // No DMA: CTL2 DENR (bit 6) stays clear (the interrupt-buffered mode spends no DMA channel).
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & (1 << 6),
        0,
        "CTL2 DENR clear (no DMA)"
    );
}

// --- B2: the ISR drains staged RBNE bytes into the ring, in order -----------------------------

#[test]
fn b2_isr_pushes_rbne_bytes_into_the_ring_in_order() {
    let fam = f10x();
    let _g = setup();
    let mut rx = install::<8>(&fam);

    let pattern = [0x11u8, 0x22, 0x33, 0x44, 0x55];
    for &b in &pattern {
        stage_byte(&fam, b);
        fire(&fam);
    }
    assert!(rx.ready(), "the ring has buffered bytes after the ISR ran");

    let mut buf = [0u8; 8];
    let n = rx.read(&mut buf).unwrap();
    assert_eq!(n, pattern.len(), "all staged bytes drained");
    assert_eq!(&buf[..n], &pattern, "bytes land in arrival order");
    // Ring now empty: a follow-up read is the non-blocking empty case, not an error.
    assert_eq!(rx.read(&mut buf), Ok(0));
}

// --- B3: IDLE handling, family-correct clear + library-owned latch, BOTH layouts --------------

#[test]
fn b3_idle_latches_clears_family_correct_and_is_consumed_by_take_idle() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let mut rx = install::<8>(&fam);

        // No byte ready, just the IDLE-line flag set.
        Reg32::new(USART_BASE, fam.stat).write(IDLEF);
        fire(&fam);

        match fam.intc {
            // F1x0: the IDLE flag is cleared by writing IDLEC (bit 4) to INTC.
            Some(intc) => assert_eq!(
                Reg32::new(USART_BASE, intc).read() & (1 << 4),
                1 << 4,
                "F1x0 wrote INTC IDLEC"
            ),
            // F10x: the clear is the STAT-then-data read pair (no INTC register to assert); the
            // latch below is the observable.
            None => {}
        }

        // `read` must NOT consume the IDLE latch: drain (nothing buffered) and the boundary survives.
        let mut buf = [0u8; 4];
        assert_eq!(rx.read(&mut buf), Ok(0));

        // `take_idle` consumes the boundary exactly once: true now, false after.
        assert!(
            rx.take_idle(),
            "the IDLE boundary latched and survived read ({:?})",
            fam.clock
        );
        assert!(
            !rx.take_idle(),
            "take_idle consumed the latch (one boundary)"
        );
    }
}

// --- B4: ring-full overflow sets the sticky Overrun surfaced by the next read -----------------

#[test]
fn b4_ring_full_overflow_surfaces_overrun() {
    let fam = f10x();
    let _g = setup();
    // Capacity word 4 => the ring holds 3 bytes (heapless N-1). The 4th byte overflows.
    let mut rx = install::<4>(&fam);

    for b in [0xA0u8, 0xA1, 0xA2, 0xA3] {
        stage_byte(&fam, b);
        fire(&fam);
    }

    // The next read surfaces the sticky overflow as Overrun (and clears it).
    let mut buf = [0u8; 8];
    assert_eq!(
        rx.read(&mut buf),
        Err(UsartError::Overrun),
        "ring-full overflow is surfaced, never a silent drop"
    );
    // After surfacing, the buffered bytes (the 3 that fit, in order) drain normally.
    let n = rx.read(&mut buf).unwrap();
    assert_eq!(&buf[..n], &[0xA0, 0xA1, 0xA2]);
}

// --- B5: line errors (ORERR/FERR/PERR) cleared + surfaced, RX not latched, BOTH layouts --------
//
// Looped over both families because the clear is family-divergent (F1x0 writes the matching `*CF`
// bit to INTC, F10x uses the STAT-then-data read pair), so the buffered ISR's F1x0 line-error clear
// is exercised here. The three staged errors map to Overrun / Framing / Parity respectively (the
// spec's "ORERR/FERR/PERR").

#[test]
fn b5_line_error_is_cleared_surfaced_and_rx_recovers() {
    for fam in [f10x(), f1x0()] {
        for (err_bit, expected) in [
            (ORERR, UsartError::Overrun),
            (FERR, UsartError::Framing),
            (PERR, UsartError::Parity),
        ] {
            let _g = setup();
            let mut rx = install::<8>(&fam);

            // Stage a line error (no RBNE): the ISR records it sticky and clears it family-correct.
            Reg32::new(USART_BASE, fam.stat).write(err_bit);
            fire(&fam);

            // The family-correct clear ran: on F1x0 the matching `*CF` bit (same position as the
            // STAT flag: ORECF/FECF/PECF) was written to INTC. F10x clears via the read pair (no
            // INTC register), so there the surfaced error below is the observable.
            if let Some(intc) = fam.intc {
                assert_eq!(
                    Reg32::new(USART_BASE, intc).read() & err_bit,
                    err_bit,
                    "F1x0 wrote the matching *CF bit to INTC ({:?})",
                    expected
                );
            }

            let mut buf = [0u8; 8];
            assert_eq!(rx.read(&mut buf), Err(expected), "line error surfaced once");
            // Surfaced once only: with no new data the follow-up read is empty, not Err again.
            assert_eq!(rx.read(&mut buf), Ok(0), "the sticky error did not latch");

            // RX is not stranded: a subsequent clean byte still arrives through the ISR.
            stage_byte(&fam, 0x5A);
            fire(&fam);
            let n = rx.read(&mut buf).unwrap();
            assert_eq!(&buf[..n], &[0x5A], "RX still alive after the line error");
        }
    }
}

// ============================================================================================
// Second-instance (module USART, F10x) cases B14-B17 (uart-rx-multi-instance.md S1)
// ============================================================================================
//
// The interrupt-path generalization: a BufferedRx can target the BLE-module USART (HAL `Usart2`),
// which has its OWN static slot (index 1) and its OWN F10x vector (`USART2_IRQn` = 39,
// `F10X_USART_MODULE_IRQ`), independent of USART1's slot 0 / vector 38. The module instance is
// F10x-only; constructing it on F1x0 (or naming any other label) fails loud.

// --- B14: the module instance uses its own slot, driven by its own vector (IRQ 39) ------------

#[test]
fn b14_module_instance_uses_its_own_slot_and_vector() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let mut rx = bring_up_module::<8>(&chip);
    install_mock(fam.irq, RAM_ADDR);

    // Stage bytes in the MODULE register space and fire the MODULE vector (39), never USART1's (38).
    let pattern = [0x71u8, 0x72, 0x73];
    for &b in &pattern {
        stage_byte_at(MODULE_BASE, &fam, b);
        // SAFETY: the RAM table is installed and slot 39 holds the module RX handler.
        unsafe { mock_vtor::dispatch(F10X_USART_MODULE_IRQ) };
    }
    assert!(
        rx.ready(),
        "the module slot buffered bytes after its ISR ran"
    );

    let mut buf = [0u8; 8];
    let n = rx.read(&mut buf).unwrap();
    assert_eq!(
        &buf[..n],
        &pattern,
        "module slot received its own bytes via IRQ 39"
    );
}

// --- B15: two instances live at once, no slot/vector collision (coexistence) ------------------

#[test]
fn b15_two_instances_live_no_slot_or_vector_collision() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);

    // USART1 BufferedRx (slot 0, vector 38) and module BufferedRx (slot 1, vector 39), both live.
    let u1 = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let s1: &'static RxRing<8> = Box::leak(Box::new(RxRing::new()));
    let mut rx1 = BufferedRx::new(&chip, u1.split().1, PeriphLabel::Usart1, s1).unwrap();
    let mut rxm = bring_up_module::<8>(&chip);
    install_mock(fam.irq, RAM_ADDR);

    // Drive USART1 (byte in the USART1 space + the USART1 vector), then the module (a DIFFERENT byte
    // in the module space + the module vector).
    stage_byte_at(USART_BASE, &fam, 0xA1);
    // SAFETY: table installed; slot 38 holds the USART1 RX handler.
    unsafe { mock_vtor::dispatch(F10X_USART1_IRQ) };
    stage_byte_at(MODULE_BASE, &fam, 0xB2);
    // SAFETY: table installed; slot 39 holds the module RX handler.
    unsafe { mock_vtor::dispatch(F10X_USART_MODULE_IRQ) };

    // Each receiver holds exactly its own byte: the ISRs filled different slots, no cross-talk.
    let mut b1 = [0u8; 4];
    let n1 = rx1.read(&mut b1).unwrap();
    assert_eq!(&b1[..n1], &[0xA1], "USART1 slot got only its own byte");
    let mut bm = [0u8; 4];
    let nm = rxm.read(&mut bm).unwrap();
    assert_eq!(&bm[..nm], &[0xB2], "module slot got only its own byte");

    // And neither vector leaked into the other's ring: a re-read of each is the empty case.
    assert_eq!(
        rx1.read(&mut b1),
        Ok(0),
        "USART1 ring drained, no stray byte"
    );
    assert_eq!(
        rxm.read(&mut bm),
        Ok(0),
        "module ring drained, no stray byte"
    );
}

// --- B16: the module instance is F10x-only; F1x0 fails loud ------------------------------------

#[test]
fn b16_module_instance_on_f1x0_is_unsupported() {
    let fam = f1x0();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg()).unwrap();
    let storage: &'static RxRing<8> = Box::leak(Box::new(RxRing::new()));
    assert_eq!(
        BufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart2, storage).map(|_| ()),
        Err(DescriptorError::Unsupported),
        "the module USART is F10x-only; F1x0 returns DescriptorError (no silent untested mapping)"
    );
}

// --- B17: a selector that is neither USART1 nor the module USART is rejected -------------------

#[test]
fn b17_unknown_instance_selector_is_rejected() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let storage: &'static RxRing<8> = Box::leak(Box::new(RxRing::new()));
    // USART0 is a real label but not a supported buffered-RX instance (only USART1 + the module are).
    assert_eq!(
        BufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart0, storage).map(|_| ()),
        Err(DescriptorError::UnknownSelector),
        "an unsupported instance selector fails loud, not a silent wrong mapping"
    );
}

// ============================================================================================
// DMA-ring (RingBufferedRx) cases B6-B12
// ============================================================================================
//
// The DMA register block is at DMA0_BASE; in the mock window the per-channel registers are at the
// confirmed offsets (stride 0x14: CHCTL 0x08, CHCNT 0x0C, CHPADDR 0x10, CHMADDR 0x14; INTF 0x00,
// INTC 0x04). Tests read/write those directly (the same hardcoded-offset style the USART tests use),
// drive the DMA ISR via `mock_vtor::dispatch`, and write the "DMA buffer" bytes through the raw
// pointer (the buffer is plain RAM, not the mock register space, so reads use the real pointer).

// CHxCTL bits (confirmed identical both families).
const CHEN: u32 = 1 << 0;
const FTFIE: u32 = 1 << 1;
const HTFIE: u32 = 1 << 2;
const CMEN: u32 = 1 << 5;
const MNAGA: u32 = 1 << 7;
// CTL2 / CTL0 enables the DMA bring-up sets.
const CTL2_DENR: u32 = 1 << 6;
const CTL2_ERRIE: u32 = 1 << 0;
const CTL0_IDLEIE: u32 = 1 << 4;

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
// INTF/INTC per-channel full-transfer (wrap) flag bit: flag << 4*channel.
fn ftf_flag(ch: u8) -> u32 {
    1 << (4 * ch as u32 + 1)
}

/// The DMA RX IRQ vector for a family's layout (separate Ch5 = 16 on F10x, grouped Ch3/4 = 11 F1x0).
fn dma_dispatch(fam: &Fam) -> usize {
    match fam.irq {
        IrqLayout::F10xSeparate => F10X_DMA0_CH5_IRQ,
        IrqLayout::F1x0Grouped => F1X0_DMA_CH3_4_IRQ,
    }
}

/// Bring up the USART + RingBufferedRx over a leaked `'static` DMA buffer of `len` bytes, with the RAM
/// table flipped for `fam`'s layout. Returns the receiver, the resolved channel, and the buffer ptr
/// (so the test can simulate DMA writes into it).
fn install_ring(fam: &Fam, len: usize) -> (RingBufferedRx, u8, *mut u8) {
    let chip = chip_for(fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let buf: &'static mut [u8] = vec![0u8; len].leak();
    let ptr = buf.as_mut_ptr();
    let ch = DmaRxMap::usart1_rx(&chip).channel;
    let rx = RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, buf).unwrap();
    install_mock(fam.irq, RAM_ADDR);
    (rx, ch, ptr)
}

/// Write a byte into the (raw) DMA buffer, modelling a DMA store. The buffer is host RAM.
fn dma_write(ptr: *mut u8, pos: usize, val: u8) {
    // SAFETY: `pos` is within the leaked buffer the test sized.
    unsafe { *ptr.add(pos) = val };
}

/// Fire the DMA ISR (a buffer wrap / transfer completion).
fn fire_dma(fam: &Fam) {
    // SAFETY: the RAM table is installed and the DMA slot holds the handler.
    unsafe { mock_vtor::dispatch(dma_dispatch(fam)) };
}

// --- B6: DmaRxMap resolves channel 5 (F10x) / 4 (F1x0), base + grouping ----------------------

#[test]
fn b6_dma_rx_map_resolves_channel_base_and_grouping() {
    let f103 = DmaRxMap::usart1_rx(&chip_for(&f10x()));
    assert_eq!(f103.channel, 5, "USART1_RX is DMA0 Ch5 on F10x");
    assert_eq!(f103.base, DMA0_BASE);
    assert!(!f103.grouped, "F10x DMA0_Channel5 has its own vector");

    let f130 = DmaRxMap::usart1_rx(&chip_for(&f1x0()));
    assert_eq!(f130.channel, 4, "USART1_RX is Ch4 on F1x0");
    assert_eq!(f130.base, DMA0_BASE);
    assert!(f130.grouped, "F1x0 Ch3/4 share a grouped vector");
}

// --- B7: channel programming (PADDR/MADDR/CNT, CMEN, CHEN last) + DENR ------------------------

#[test]
fn b7_channel_programming_and_denr() {
    for fam in [f10x(), f1x0()] {
        let _g = setup();
        let chip = chip_for(&fam);
        let usart =
            Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
        let buf: &'static mut [u8] = vec![0u8; 32].leak();
        let maddr = buf.as_ptr() as u32;
        let ch = DmaRxMap::usart1_rx(&chip).channel;
        let rx_half = usart.split().1;
        let rdata = rx_half.regs().rdata_addr();
        let _rx = RingBufferedRx::new(&chip, rx_half, PeriphLabel::Usart1, buf).unwrap();

        // PADDR = the RDATA address; MADDR = buffer ptr; CNT = len.
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_paddr(ch)).read(),
            rdata,
            "CHxPADDR = USART RDATA address"
        );
        assert_eq!(
            Reg32::new(DMA0_BASE, ch_maddr(ch)).read(),
            maddr,
            "CHxMADDR = buffer"
        );
        assert_eq!(Reg32::new(DMA0_BASE, ch_cnt(ch)).read(), 32, "CHxCNT = len");

        // CHCTL: circular + mem-incr + half/full IRQ + CHEN (all set after configure).
        let ctl = Reg32::new(DMA0_BASE, ch_ctl(ch)).read();
        assert_eq!(ctl & CMEN, CMEN, "CMEN (circular) set");
        assert_eq!(ctl & MNAGA, MNAGA, "memory-increment set");
        assert_eq!(
            ctl & (FTFIE | HTFIE),
            FTFIE | HTFIE,
            "half+full IRQ enabled"
        );
        assert_eq!(ctl & CHEN, CHEN, "CHEN set (channel started)");
        // DIR = 0 (periph->mem), PNAGA = 0 (peripheral fixed), widths 0 (8-bit).
        assert_eq!(ctl & (1 << 4), 0, "DIR=0 periph->mem");
        assert_eq!(ctl & (1 << 6), 0, "PNAGA=0 peripheral fixed");
        assert_eq!(ctl & (0xF << 8), 0, "8-bit widths");

        // USART: DMA reception enabled (DENR), IDLE + error interrupts on.
        assert_eq!(
            Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENR,
            CTL2_DENR,
            "DENR set"
        );
        assert_eq!(
            Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_ERRIE,
            CTL2_ERRIE,
            "ERRIE set"
        );
        assert_eq!(
            Reg32::new(USART_BASE, fam.ctl0).read() & CTL0_IDLEIE,
            CTL0_IDLEIE,
            "IDLEIE set"
        );
    }
}

// --- B8: read math (len - CHxCNT), advancing the cursor, wrap across the buffer end -----------

#[test]
fn b8_read_returns_len_minus_chxcnt_bytes_and_wraps() {
    let fam = f10x();
    let _g = setup();
    let (mut rx, ch, ptr) = install_ring(&fam, 8);

    // The DMA "wrote" 6 bytes: CHxCNT = len - 6 = 2 (write index 6).
    let first = [10u8, 11, 12, 13, 14, 15];
    for (i, &b) in first.iter().enumerate() {
        dma_write(ptr, i, b);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 6);

    let mut out = [0u8; 16];
    let n = rx.read(&mut out).unwrap();
    assert_eq!(n, 6, "len - CHxCNT bytes available");
    assert_eq!(&out[..6], &first, "the bytes behind the DMA head, in order");

    // Now the DMA wrote 4 more bytes, wrapping: buf[6], buf[7], buf[0], buf[1]. One full-transfer
    // completion (wrap) fires the DMA ISR, then CHxCNT = len - 2 = 6 (write index 2 this lap).
    dma_write(ptr, 6, 16);
    dma_write(ptr, 7, 17);
    dma_write(ptr, 0, 20);
    dma_write(ptr, 1, 21);
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch)); // INTF full-transfer for our channel
    fire_dma(&fam); // wrap counted
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 2);

    let n = rx.read(&mut out).unwrap();
    assert_eq!(n, 4, "4 more bytes, spanning the buffer wrap");
    assert_eq!(
        &out[..4],
        &[16, 17, 20, 21],
        "bytes read across the end of the buffer"
    );
}

// --- B9: lap detection -> RingOverrun (recoverable in place, distinct from the disabling Overrun) --

#[test]
fn b9_lap_past_cursor_is_ring_overrun() {
    let fam = f10x();
    let _g = setup();
    let (mut rx, ch, _ptr) = install_ring(&fam, 8);

    // Model the DMA lapping the cursor: two full wraps + write index 1, cursor still 0 => the head
    // is len*2 + 1 = 17 bytes ahead of the cursor, far more than the buffer, so data was overwritten.
    // The flag is re-set before each dispatch (the ISR clears it), so the wrap counter reaches 2.
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    fire_dma(&fam);
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    fire_dma(&fam); // wraps = 2
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 1);

    let mut out = [0u8; 16];
    // The lap is a non-silent loss signal, but RECOVERABLE in place: it is `RingOverrun`, NOT the
    // channel-disabling `Overrun` the ERRIE path raises (b12b). The channel must stay live so the very
    // next read keeps draining.
    assert_eq!(
        rx.read(&mut out),
        Err(UsartError::RingOverrun),
        "the DMA lapping the cursor by more than the buffer is a non-silent, in-place RingOverrun"
    );
    // Channel still live: after the resync, the next read returns the freshest bytes (no re-arm). Stage
    // a fresh lap's worth of progress (CHxCNT = len - 2 within the current lap) and confirm draining
    // continues rather than the channel being dead.
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 3);
    assert!(
        rx.read(&mut out).is_ok(),
        "RingOverrun leaves the channel live: reads keep draining without a re-arm"
    );
}

// --- B13: wrap boundary with a PENDING (uncounted) FTF is not a spurious Overrun --------------
//
// The false-positive twin of B9. At a circular wrap the hardware reloads `CHxCNT` to `len` and sets
// `FTFIF`, but the wrap-counter ISR runs slightly later. If `read` snapshots the reloaded
// `CHxCNT == len` with the OLD wrap count it would undercount the write index by `len`, underflow
// `available`, and report a spurious `Overrun` though the cursor was never lapped. The fixed snapshot
// reads the pending `FTFIF` and attributes the wrap, so `read` returns the new bytes, not `Overrun`.

#[test]
fn b13_wrap_boundary_pending_ftf_is_not_spurious_overrun() {
    let fam = f10x();
    let _g = setup();
    let (mut rx, ch, ptr) = install_ring(&fam, 8);

    // First read: the DMA wrote 5 bytes (CHxCNT = len - 5); cursor advances to 5.
    for (i, &b) in [10u8, 11, 12, 13, 14].iter().enumerate() {
        dma_write(ptr, i, b);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 5);
    let mut out = [0u8; 8];
    assert_eq!(rx.read(&mut out), Ok(5));
    assert_eq!(&out[..5], &[10, 11, 12, 13, 14]);

    // Now the lap completes (the remaining 3 bytes land at buf[5..8]); CHxCNT reloads to `len` and the
    // channel's FTFIF is set, but the wrap-counter ISR has NOT run (the wrap counter is still 0). The
    // DMA wrote exactly 8 bytes total vs cursor 5: it did NOT lap the cursor.
    for (i, &b) in [15u8, 16, 17].iter().enumerate() {
        dma_write(ptr, 5 + i, b);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8); // reloaded to len
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch)); // FTF pending, ISR not yet run (wraps == 0)

    // `read` must attribute the pending wrap (write index = 8) and return the 3 new bytes, NOT a
    // spurious Overrun. `.unwrap()` asserts it is Ok (the old, buggy snapshot returned Err(Overrun)).
    let n = rx.read(&mut out).unwrap();
    assert_eq!(
        n, 3,
        "the 3 bytes of the just-completed lap, no spurious overrun"
    );
    assert_eq!(&out[..3], &[15, 16, 17]);
}

// --- B10: F1x0 grouped DMA demux routes Ch4, ignores Ch3 -------------------------------------

#[test]
fn b10_grouped_demux_routes_ch4_not_ch3() {
    let fam = f1x0();
    let _g = setup();
    let (_rx, ch, _ptr) = install_ring(&fam, 16);
    assert_eq!(ch, 4, "F1x0 USART1_RX is Ch4");

    // Both the unrelated Ch3 and our Ch4 have a full-transfer flag pending in INTF.
    let ch3_ftf = ftf_flag(3);
    Reg32::new(DMA0_BASE, 0x00).write(ch3_ftf | ftf_flag(4));
    fire_dma(&fam); // the grouped Ch3/4 vector

    // Exactly one wrap counted (Ch4's), and INTC cleared ONLY Ch4's bits (GIF+FTFIF at 4*4): Ch3 was
    // not serviced and no Ch3 event was invented.
    assert_eq!(
        crate::dma::wraps(0),
        1,
        "only the Ch4 full-transfer was counted"
    );
    let intc = Reg32::new(DMA0_BASE, 0x04).read();
    assert_eq!(intc & ftf_flag(4), ftf_flag(4), "Ch4 FTF cleared");
    assert_eq!(intc & (1 << (4 * 4)), 1 << (4 * 4), "Ch4 GIF cleared");
    assert_eq!(intc & ch3_ftf, 0, "Ch3 flags untouched (not dispatched)");
    assert_eq!(intc & (1 << (4 * 3)), 0, "Ch3 GIF untouched");
}

// --- B11: write-back self-check fails loud, arming nothing ------------------------------------

#[test]
fn b11_self_check_failure_arms_nothing() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let buf: &'static mut [u8] = vec![0u8; 32].leak();
    let ch = DmaRxMap::usart1_rx(&chip).channel;

    // Model a non-responsive channel: writes to CHxMADDR do not stick, so the write-back self-check
    // reads back something other than the sentinel.
    mock::freeze(DMA0_BASE + ch_maddr(ch));

    assert!(
        matches!(
            RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, buf),
            Err(DescriptorError::SelfCheckFailed)
        ),
        "a channel that does not respond fails loud"
    );
    // Nothing was armed: CHEN stays clear (configure_circular never ran).
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "no channel armed on a failed self-check"
    );
}

// --- B12: line error stops reception; a fresh new re-arms (no auto-restart) -------------------

#[test]
fn b12_line_error_stops_reception_and_re_new_rearms() {
    let fam = f10x();
    let _g = setup();
    let (mut rx, ch, _ptr) = install_ring(&fam, 32);
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "armed at start"
    );

    // A USART line error (ERRIE path) during background reception: stage FERR and fire the shared
    // USART ISR, which records the sticky line error.
    Reg32::new(USART_BASE, fam.stat).write(FERR);
    fire(&fam);

    // read surfaces the error AND stops background reception (CHEN cleared) - fail-loud, no restart.
    let mut out = [0u8; 8];
    assert_eq!(
        rx.read(&mut out),
        Err(UsartError::Framing),
        "line error surfaced"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "reception stopped (channel disabled), no silent auto-restart"
    );

    // A fresh `new` re-arms: CHEN set again, and DENR/IDLEIE/ERRIE re-enabled.
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let buf2: &'static mut [u8] = vec![0u8; 32].leak();
    let _rx2 = RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, buf2).unwrap();
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "re-armed: CHEN set"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_DENR,
        CTL2_DENR,
        "DENR re-enabled"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & CTL2_ERRIE,
        CTL2_ERRIE,
        "ERRIE re-enabled"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl0).read() & CTL0_IDLEIE,
        CTL0_IDLEIE,
        "IDLEIE re-enabled"
    );
}

// --- B12b: an ERRIE *overrun* line error returns the DISABLING `Overrun`, distinct from a lap ----

#[test]
fn b12b_errie_overrun_disables_channel_and_is_overrun_not_ring_overrun() {
    let fam = f10x();
    let _g = setup();
    let (mut rx, ch, _ptr) = install_ring(&fam, 32);
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "armed at start"
    );

    // A USART ORERR (overrun) line error on the ERRIE path: the shared ISR records it sticky. Unlike
    // the b9 lap-overrun, this is a HARDWARE overrun that the read path treats as channel-disabling.
    Reg32::new(USART_BASE, fam.stat).write(ORERR);
    fire(&fam);

    let mut out = [0u8; 8];
    // It surfaces as the channel-disabling `Overrun` (NOT `RingOverrun`): the two overrun conditions
    // map to distinct values so the caller re-arms here but recovers in place after a lap.
    assert_eq!(
        rx.read(&mut out),
        Err(UsartError::Overrun),
        "an ERRIE overrun is the disabling `Overrun`, not the in-place `RingOverrun`"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "the ERRIE overrun disabled the channel (caller must re-arm)"
    );
}

// ============================================================================================
// Second-instance DMA-ring (module USART, F10x) cases B18-B21 (uart-rx-multi-instance.md S2)
// ============================================================================================
//
// The DMA-path generalization: a RingBufferedRx can target the BLE-module USART (HAL `Usart2`), whose
// RX is GD `USART2_RX` = DMA0 Channel 2 (UM §9.4.9 Table 9-3), with its OWN DMA vector
// (`DMA0_Channel2_IRQn` = 13, `F10X_DMA0_CH2_IRQ`) and its OWN wrap-counter context (index 1),
// independent of USART1's channel 5 / vector 16 / context 0. F10x-only; F1x0 fails loud.

/// Bring up a module-USART `RingBufferedRx` (channel 2, context 1) over a leaked `'static` DMA buffer
/// of `len` bytes, with the F10x RAM table flipped. Returns the receiver, its channel, and the buf ptr.
fn install_ring_module(len: usize) -> (RingBufferedRx, u8, *mut u8) {
    let chip = chip_for(&f10x());
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg()).unwrap();
    let buf: &'static mut [u8] = vec![0u8; len].leak();
    let ptr = buf.as_mut_ptr();
    let ch = DmaRxMap::module_rx().channel;
    let rx = RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart2, buf).unwrap();
    install_mock(IrqLayout::F10xSeparate, RAM_ADDR);
    (rx, ch, ptr)
}

// --- B18: DmaRxMap::module_rx resolves channel 2 (GD USART2_RX), base, separate vector ---------

#[test]
fn b18_dma_rx_map_module_resolves_channel2() {
    let m = DmaRxMap::module_rx();
    assert_eq!(
        m.channel, 2,
        "GD USART2_RX is DMA0 Ch2 on F10x (UM Table 9-3, the USART row)"
    );
    assert_eq!(m.base, DMA0_BASE);
    assert!(!m.grouped, "F10x DMA0_Channel2 has its own vector (IRQ 13)");
}

// --- B19: module RingBufferedRx reads channel 2's CHxCNT; its vector bumps its own context ------

#[test]
fn b19_module_ring_reads_channel2_and_its_own_context() {
    let _g = setup();
    let (mut rx, ch, ptr) = install_ring_module(8);
    assert_eq!(ch, 2, "module RingBufferedRx armed DMA0 Ch2");

    // The DMA "wrote" 6 bytes into the module buffer; CHxCNT(2) = len - 6 = 2 (write index 6).
    let first = [20u8, 21, 22, 23, 24, 25];
    for (i, &b) in first.iter().enumerate() {
        dma_write(ptr, i, b);
    }
    Reg32::new(DMA0_BASE, ch_cnt(ch)).write(8 - 6);
    let mut out = [0u8; 16];
    let n = rx.read(&mut out).unwrap();
    assert_eq!(
        n, 6,
        "len - CHxCNT(2) bytes available from the module channel"
    );
    assert_eq!(&out[..6], &first, "the module channel's bytes, in order");

    // A buffer wrap: fire the MODULE DMA vector (IRQ 13). Only the module wrap context (1) increments;
    // USART1's context (0) is untouched (no cross-talk between the two DMA contexts).
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch));
    // SAFETY: the F10x table is installed and slot 13 holds the module DMA handler.
    unsafe { mock_vtor::dispatch(F10X_DMA0_CH2_IRQ) };
    assert_eq!(crate::dma::wraps(1), 1, "module wrap counted on context 1");
    assert_eq!(crate::dma::wraps(0), 0, "USART1 context untouched");
}

// --- B20: two DMA receivers live; each vector services only its own channel/context ------------

#[test]
fn b20_two_dma_instances_live_no_channel_or_context_collision() {
    let _g = setup();
    let chip = chip_for(&f10x());

    // USART1 RingBufferedRx (channel 5, context 0, vector 16) and module RingBufferedRx (channel 2,
    // context 1, vector 13), both live.
    let u1 = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let b1: &'static mut [u8] = vec![0u8; 16].leak();
    let _r1 = RingBufferedRx::new(&chip, u1.split().1, PeriphLabel::Usart1, b1).unwrap();
    let ch_u1 = DmaRxMap::usart1_rx(&chip).channel;
    let um = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg()).unwrap();
    let bm: &'static mut [u8] = vec![0u8; 16].leak();
    let _rm = RingBufferedRx::new(&chip, um.split().1, PeriphLabel::Usart2, bm).unwrap();
    let ch_mod = DmaRxMap::module_rx().channel;
    assert_eq!((ch_u1, ch_mod), (5, 2), "distinct channels, no collision");
    install_mock(IrqLayout::F10xSeparate, RAM_ADDR);

    // Both channels' full-transfer flags pending. Fire ONLY the USART1 DMA vector: it counts only its
    // own channel (5 / context 0) and leaves the module channel's flag pending.
    Reg32::new(DMA0_BASE, 0x00).write(ftf_flag(ch_u1) | ftf_flag(ch_mod));
    // SAFETY: table installed; slot 16 holds the USART1 DMA handler.
    unsafe { mock_vtor::dispatch(F10X_DMA0_CH5_IRQ) };
    assert_eq!(crate::dma::wraps(0), 1, "USART1 context counted its wrap");
    assert_eq!(
        crate::dma::wraps(1),
        0,
        "module context untouched by the USART1 vector"
    );
    assert_ne!(
        Reg32::new(DMA0_BASE, 0x00).read() & ftf_flag(ch_mod),
        0,
        "the module channel's FTF is still pending (USART1 vector did not service Ch2)"
    );

    // Now fire the module DMA vector: it services only its own channel (2 / context 1).
    // SAFETY: table installed; slot 13 holds the module DMA handler.
    unsafe { mock_vtor::dispatch(F10X_DMA0_CH2_IRQ) };
    assert_eq!(crate::dma::wraps(1), 1, "module context counted its wrap");
    assert_eq!(
        crate::dma::wraps(0),
        1,
        "USART1 context unchanged by the module vector"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, 0x00).read() & ftf_flag(ch_mod),
        0,
        "the module channel's FTF is now cleared by its own vector"
    );
    assert_eq!(
        Reg32::new(DMA0_BASE, 0x00).read() & ftf_flag(ch_u1),
        0,
        "the USART1 channel's FTF stayed cleared (its vector cleared it, the module vector left it)"
    );
}

// --- B21: the module DMA path is F10x-only; F1x0 fails loud ------------------------------------

#[test]
fn b21_module_ring_on_f1x0_is_unsupported() {
    let _g = setup();
    let chip = chip_for(&f1x0());
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &module_cfg()).unwrap();
    let buf: &'static mut [u8] = vec![0u8; 16].leak();
    assert_eq!(
        RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart2, buf).map(|_| ()),
        Err(DescriptorError::Unsupported),
        "the module DMA path is F10x-only; F1x0 returns DescriptorError (no silent untested mapping)"
    );
}

// --- release / re-arm / the reprogram sequence (specs/usart-split.md section 5) ----------------

#[test]
fn buffered_release_quiesces_and_the_half_rearms() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let storage: &'static RxRing<8> = Box::leak(Box::new(RxRing::new()));
    let mut rx = BufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, storage).unwrap();
    install_mock(fam.irq, RAM_ADDR);

    // Live: the IRQ enables are set and a staged byte flows through the ISR into the ring.
    let ctl0 = Reg32::new(USART_BASE, fam.ctl0).read();
    assert_eq!(ctl0 & ((1 << 5) | (1 << 4)), (1 << 5) | (1 << 4));
    stage_byte(&fam, 0xA1);
    fire(&fam);
    let mut buf = [0u8; 4];
    assert_eq!(rx.read(&mut buf), Ok(1));
    assert_eq!(buf[0], 0xA1);

    // Release: RBNEIE + IDLEIE cleared (quiesced), everything else untouched.
    let half = rx.release();
    let ctl0 = Reg32::new(USART_BASE, fam.ctl0).read();
    assert_eq!(ctl0 & ((1 << 5) | (1 << 4)), 0, "RBNEIE+IDLEIE cleared");
    assert_eq!(
        ctl0 & ((1 << 2) | (1 << 3) | (1 << 13)),
        (1 << 2) | (1 << 3) | (1 << 13),
        "REN+TEN+UEN untouched"
    );

    // The returned half re-arms and receives again.
    let storage2: &'static RxRing<8> = Box::leak(Box::new(RxRing::new()));
    let mut rx2 = BufferedRx::new(&chip, half, PeriphLabel::Usart1, storage2).unwrap();
    stage_byte(&fam, 0xB2);
    fire(&fam);
    assert_eq!(rx2.read(&mut buf), Ok(1));
    assert_eq!(buf[0], 0xB2);
}

#[test]
fn ring_release_disables_channel_and_the_half_rearms() {
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let usart = Usart::bring_up(&chip, &ClockConfig::REFERENCE_72M_IRC8M, &bench_cfg()).unwrap();
    let ch = DmaRxMap::usart1_rx(&chip).channel;
    let buf: &'static mut [u8] = vec![0u8; 32].leak();
    let rx = RingBufferedRx::new(&chip, usart.split().1, PeriphLabel::Usart1, buf).unwrap();

    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        CHEN,
        "armed: CHEN set"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & ((1 << 6) | (1 << 0)),
        (1 << 6) | (1 << 0),
        "armed: DENR + ERRIE set"
    );

    let half = rx.release();
    assert_eq!(
        Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN,
        0,
        "released: channel disabled"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl2).read() & ((1 << 6) | (1 << 0)),
        0,
        "released: DENR + ERRIE cleared"
    );
    assert_eq!(
        Reg32::new(USART_BASE, fam.ctl0).read() & (1 << 4),
        0,
        "released: IDLEIE cleared"
    );

    // Re-arm on the returned half: the channel comes back up.
    let buf2: &'static mut [u8] = vec![0u8; 32].leak();
    let _rx2 = RingBufferedRx::new(&chip, half, PeriphLabel::Usart1, buf2).unwrap();
    assert_eq!(Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN, CHEN);
}

#[test]
fn reprogram_sequence_release_rejoin_set_baud_split_rearm() {
    // The bench's baud-change path (specs/usart-split.md D4), end to end on the mock: a live DMA
    // receiver is released, the halves rejoin, set_baud reprograms, and a fresh split re-arms.
    let fam = f10x();
    let _g = setup();
    let chip = chip_for(&fam);
    let clock = ClockConfig::REFERENCE_72M_IRC8M;
    let usart = Usart::bring_up(&chip, &clock, &bench_cfg()).unwrap();
    let baud_off = 0x08; // F10x BAUD offset
    assert_eq!(Reg32::new(USART_BASE, baud_off).read(), 313); // 115200 @ 36 MHz

    let (tx, rx) = usart.split();
    let buf: &'static mut [u8] = vec![0u8; 32].leak();
    let ring = RingBufferedRx::new(&chip, rx, PeriphLabel::Usart1, buf).unwrap();

    // Reconfigure: impossible while split (no set_baud on the halves; the type system enforces
    // it), so release -> rejoin -> set_baud -> split -> re-arm.
    let rx = ring.release();
    let mut whole = Usart::rejoin(tx, rx);
    whole.set_baud(&clock, 9_600);
    assert_eq!(Reg32::new(USART_BASE, baud_off).read(), 3750);
    let (_tx, rx) = whole.split();
    let buf2: &'static mut [u8] = vec![0u8; 32].leak();
    let ch = DmaRxMap::usart1_rx(&chip).channel;
    let _ring = RingBufferedRx::new(&chip, rx, PeriphLabel::Usart1, buf2).unwrap();
    assert_eq!(Reg32::new(DMA0_BASE, ch_ctl(ch)).read() & CHEN, CHEN);
}

// --- supports_rx: the public capability query (uart-rx-multi-instance.md acceptance) ------------

/// `supports_rx` is pure and tracks `resolve_instance` exactly: Usart1 on both families, Usart2 on
/// F10x only, everything else (incl. the not-yet-expressible Usart0-remap) false. No mock setup is
/// performed: a register access would panic the fresh mock lock discipline, pinning purity.
#[test]
fn supports_rx_answers_from_the_model() {
    let _g = setup();
    let f10x_chip = chip_for(&f10x());
    let f1x0_chip = chip_for(&f1x0());

    assert!(
        supports_rx(&f10x_chip, PeriphLabel::Usart1),
        "Usart1 on F10x"
    );
    assert!(
        supports_rx(&f1x0_chip, PeriphLabel::Usart1),
        "Usart1 on F1x0"
    );
    assert!(
        supports_rx(&f10x_chip, PeriphLabel::Usart2),
        "the module USART resolves on F10x"
    );
    assert!(
        !supports_rx(&f1x0_chip, PeriphLabel::Usart2),
        "the module USART is F10x-only (the chip-blind-flag bug this query fixes)"
    );
    // Usart0: false until the AFIO remap primitive exists (specs/usart-pin-remap.md).
    assert!(!supports_rx(&f10x_chip, PeriphLabel::Usart0));
    assert!(!supports_rx(&f1x0_chip, PeriphLabel::Usart0));
    // A non-USART label is never RX-capable.
    assert!(!supports_rx(&f10x_chip, PeriphLabel::Gpioa));
}
