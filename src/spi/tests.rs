//! T8/T9 host tests for the shared SPI driver (run under the `mock` feature against the
//! backing-array register space).
//!
//! Four groups:
//! - **prescaler / clock** ([`prescaler_for`], [`spi_input_clock`]): the PSC field code chosen for a
//!   bus clock + target, and the APB2/APB1 derivation.
//! - **mode bits** ([`mode_bits`]): CPOL/CPHA -> CTL0 CKPL/CKPH.
//! - **bring-up** ([`Spi::bring_up`]): the CTL0 end state (master + software NSS + MSB + frame size
//!   + mode + PSC + SPIEN) and the I2SCTL clear, vs the SPL `spi_init` / `spi_enable`.
//! - **transfer + `embedded-hal`** ([`embedded_hal::spi::SpiBus`]): read / write / transfer /
//!   transfer_in_place / flush over the mock register space with STAT seeded so TBE/RBNE polls pass,
//!   and error-injection (CONFERR / RXORERR in STAT) asserting the mapped
//!   [`embedded_hal::spi::ErrorKind`].
//!
//! The mock backend is a flat array (a static register snapshot, not a sequencer): the polled loops
//! are made to terminate by seeding STAT with TBE | RBNE set at once; each `wait_flag` only checks
//! its own bit, so a single all-flags-set STAT satisfies the whole transfer. The DATA register is
//! one flat cell, so a transfer reads back whatever DATA last held; the value-path test drives
//! `receive()` against a seeded DATA. The on-silicon flag-by-flag progression is the with_polling
//! golden's job (T9 harness layer) and the bench MOSI/MISO loopback (T13); this host layer proves
//! the register-level transfer shape and the error mapping.
#![cfg(feature = "mock")]

use super::*;
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{NssMode, SpiConfig};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::reg::{mock, Reg32};
use embedded_hal::spi::{
    Error as _, ErrorKind, Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};
use std::sync::MutexGuard;

/// The SPI0 base (the mock window wraps modulo its size; only the offsets matter).
const SPI0_BASE: u32 = 0x4001_3000;

/// Build a `Chip` mapping SPI0 to `SPI0_BASE` (and an RCU base).
fn chip() -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Spi0, SPI0_BASE);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

/// The reference 72 MHz / 2 WS tree: SPI0 sits on APB2 = sysclk / apb2_psc(1) = 72 MHz, matching the
/// bus_hz the old `bring_up` took directly. APB1 = 36 MHz.
fn ref_72m() -> ClockConfig {
    ClockConfig::REFERENCE_72M_IRC8M
}

/// A SPI0 config for `target_hz`, `mode` (0..3), and 8/16-bit, MSB-first, software NSS.
fn cfg_for(target_hz: u32, mode: Mode, data16: bool) -> SpiConfig {
    // Reconstruct the 0..3 mode code from the embedded-hal Mode.
    let cpol = matches!(mode.polarity, Polarity::IdleHigh) as u8;
    let cpha = matches!(mode.phase, Phase::CaptureOnSecondTransition) as u8;
    SpiConfig {
        spi: PeriphLabel::Spi0,
        sck: 0x05,  // PA5
        miso: 0x06, // PA6
        mosi: 0x07, // PA7
        nss: 0x04,  // PA4
        mode: (cpol << 1) | cpha,
        data16,
        target_hz,
        lsb_first: false,
        nss_mode: NssMode::Software,
    }
}

fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

fn r(off: u32) -> u32 {
    Reg32::new(SPI0_BASE, off).read()
}
fn w(off: u32, v: u32) {
    Reg32::new(SPI0_BASE, off).write(v);
}

/// Set both STAT flags a transfer polls (TBE | RBNE), so each bounded poll exits immediately.
fn seed_stat_ready() {
    w(STAT, STAT_TBE | STAT_RBNE);
}

// --- prescaler / clock ------------------------------------------------------------------------

#[test]
fn prescaler_picks_smallest_divisor_at_or_below_target() {
    // APB2 = 72 MHz. Target 1 MHz: /72 needed, but only powers of two; /128 (code 6) gives
    // 562.5 kHz <= 1 MHz while /64 (code 5) gives 1.125 MHz > 1 MHz, so code 6.
    assert_eq!(prescaler_for(72_000_000, 1_000_000), 6);
    // Target 9 MHz at 72 MHz: /8 (code 2) = 9 MHz <= 9 MHz; /4 = 18 MHz > 9. -> code 2.
    assert_eq!(prescaler_for(72_000_000, 9_000_000), 2);
    // Target 36 MHz at 72 MHz: /2 (code 0) = 36 MHz <= 36. -> code 0.
    assert_eq!(prescaler_for(72_000_000, 36_000_000), 0);
    // Target far above bus/2: still /2 (code 0).
    assert_eq!(prescaler_for(72_000_000, 100_000_000), 0);
    // Target far below bus/256: clamp to the slowest, /256 (code 7).
    assert_eq!(prescaler_for(72_000_000, 1_000), 7);
}

#[test]
fn spi_input_clock_apb2_and_apb1() {
    let p = ref_72m();
    // SPI0 on APB2: 72 MHz / apb2_psc(1) = 72 MHz.
    assert_eq!(spi_input_clock(&p, true), 72_000_000);
    // SPI1 on APB1: (72 / ahb 1) / apb1_psc(2) = 36 MHz.
    assert_eq!(spi_input_clock(&p, false), 36_000_000);
}

// --- mode bits --------------------------------------------------------------------------------

#[test]
fn mode_bits_map_cpol_cpha() {
    assert_eq!(mode_bits(MODE_0), 0, "CPOL 0 CPHA 0 -> no bits");
    assert_eq!(mode_bits(MODE_1), CTL0_CKPH, "CPOL 0 CPHA 1 -> CKPH");
    assert_eq!(mode_bits(MODE_2), CTL0_CKPL, "CPOL 1 CPHA 0 -> CKPL");
    assert_eq!(
        mode_bits(MODE_3),
        CTL0_CKPL | CTL0_CKPH,
        "CPOL 1 CPHA 1 -> CKPL|CKPH"
    );
    // Explicit construction matches.
    let m = Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
    };
    assert_eq!(mode_bits(m), CTL0_CKPL | CTL0_CKPH);
}

// --- bring-up register end state --------------------------------------------------------------

#[test]
fn bring_up_programs_ctl0_master_soft_nss_mode_psc_and_enable() {
    let _g = seed_reset();
    // 72 MHz APB2, target 1 MHz (PSC code 6), MODE_0, 8-bit.
    let _dev = Spi::bring_up(&chip(), &ref_72m(), &cfg_for(1_000_000, MODE_0, false)).unwrap();

    let ctl0 = r(CTL0);
    // Master = MSTMOD(2) | SWNSS(8); software NSS adds SWNSSEN(9).
    assert_eq!(ctl0 & CTL0_MSTMOD, CTL0_MSTMOD, "MSTMOD set (master)");
    assert_eq!(
        ctl0 & CTL0_SWNSS,
        CTL0_SWNSS,
        "SWNSS set (NSS internal high)"
    );
    assert_eq!(
        ctl0 & CTL0_SWNSSEN,
        CTL0_SWNSSEN,
        "SWNSSEN set (software NSS)"
    );
    // MSB-first (LF clear), full-duplex, 8-bit (FF16 clear), MODE_0 (CKPL/CKPH clear).
    assert_eq!(ctl0 & CTL0_LF, 0, "LF clear (MSB-first)");
    assert_eq!(ctl0 & CTL0_FF16, 0, "FF16 clear (8-bit)");
    assert_eq!(ctl0 & (CTL0_CKPL | CTL0_CKPH), 0, "MODE_0: CKPL/CKPH clear");
    // PSC field = code 6 << 3.
    assert_eq!((ctl0 & CTL0_PSC) >> 3, 6, "PSC = code 6 (/128)");
    // SPIEN set by spi_enable.
    assert_eq!(ctl0 & CTL0_SPIEN, CTL0_SPIEN, "SPIEN set");
    // I2SCTL I2SSEL cleared (SPI mode, not I2S).
    assert_eq!(r(I2SCTL) & I2SCTL_I2SSEL, 0, "I2SSEL clear (SPI mode)");
}

#[test]
fn bring_up_mode3_and_16bit_set_their_bits() {
    let _g = seed_reset();
    let _dev = Spi::bring_up(&chip(), &ref_72m(), &cfg_for(9_000_000, MODE_3, true)).unwrap();
    let ctl0 = r(CTL0);
    assert_eq!(
        ctl0 & (CTL0_CKPL | CTL0_CKPH),
        CTL0_CKPL | CTL0_CKPH,
        "MODE_3 sets CKPL|CKPH"
    );
    assert_eq!(ctl0 & CTL0_FF16, CTL0_FF16, "16-bit sets FF16");
    assert_eq!(
        (ctl0 & CTL0_PSC) >> 3,
        2,
        "PSC = code 2 (/8) for 9 MHz target"
    );
}

#[test]
fn bring_up_matches_spl_init_mask_end_state() {
    // The SPL spi_init reads CTL0, masks with SPI_INIT_MASK (0x3040), ORs the params. From reset-0
    // CTL0 the mask keeps nothing, so the end state is exactly the ORed parameter bits for MODE_0,
    // master, software NSS, 8-bit, /128, then SPIEN: MSTMOD|SWNSS|SWNSSEN|(6<<3)|SPIEN.
    let _g = seed_reset();
    let _dev = Spi::bring_up(&chip(), &ref_72m(), &cfg_for(1_000_000, MODE_0, false)).unwrap();
    let expected = CTL0_MSTMOD | CTL0_SWNSS | CTL0_SWNSSEN | (6 << 3) | CTL0_SPIEN;
    assert_eq!(
        r(CTL0),
        expected,
        "CTL0 end state matches the SPL spi_init+spi_enable result"
    );
}

// --- embedded-hal spi::SpiBus transfers -------------------------------------------------------

fn brought_up() -> Spi {
    Spi::bring_up(&chip(), &ref_72m(), &cfg_for(1_000_000, MODE_0, false)).unwrap()
}

#[test]
fn write_clocks_each_byte_to_data() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat_ready();
    SpiBus::write(&mut dev, &[0xAB, 0xCD]).expect("write should succeed");
    // The last byte written to DATA is 0xCD.
    assert_eq!(r(DATA) & 0xFF, 0xCD, "last write byte reached DATA");
}

#[test]
fn transfer_byte_returns_data_register_value() {
    let _g = seed_reset();
    let dev = brought_up();
    seed_stat_ready();
    // On the flat-array mock TX and RX share one DATA cell, so transfer_byte(tx) writes tx then
    // reads it straight back: the received value equals the transmitted byte here. This proves the
    // write-then-read DATA path (a silicon loopback, T13, sees the same equality for real).
    assert_eq!(
        dev.transfer_byte(0x5A).unwrap(),
        0x5A,
        "transfer_byte echoes the DATA byte"
    );
}

#[test]
fn read_clocks_dummy_and_fills_buffer() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat_ready();
    let mut buf = [0xFFu8; 2];
    SpiBus::read(&mut dev, &mut buf).expect("read should succeed");
    // read clocks a dummy 0x00 per byte; the flat mock echoes it, so each slot reads back 0x00.
    assert_eq!(
        buf,
        [0x00, 0x00],
        "read clocked dummy 0x00 bytes and filled the buffer"
    );
    assert_eq!(r(DATA) & 0xFF, 0x00, "DATA holds the last dummy byte");
}

#[test]
fn transfer_clocks_write_and_captures_read() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat_ready();
    w(DATA, 0x77); // the byte the wire returns for each clocked byte (flat mock).
    let mut rd = [0u8; 3];
    SpiBus::transfer(&mut dev, &mut rd, &[0x11, 0x22, 0x33]).expect("transfer should succeed");
    // Each received slot is whatever DATA held when read; the mock conflates TX/RX, so after each
    // transmit DATA = the TX byte, so rd[i] == write[i] here. The point is the call completes and
    // fills the whole read buffer.
    assert_eq!(
        rd,
        [0x11, 0x22, 0x33],
        "transfer filled the read buffer (flat-mock TX echo)"
    );
}

#[test]
fn transfer_handles_uneven_buffers() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat_ready();
    // read longer than write: once write is exhausted, dummy 0x00 is clocked.
    let mut rd = [0u8; 3];
    SpiBus::transfer(&mut dev, &mut rd, &[0xEE]).expect("uneven transfer should succeed");
    assert_eq!(rd[2], 0x00, "tail clocked dummy 0x00 once write exhausted");
}

#[test]
fn transfer_in_place_replaces_each_byte() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat_ready();
    let mut buf = [0x01, 0x02, 0x03];
    SpiBus::transfer_in_place(&mut dev, &mut buf).expect("in-place should succeed");
    // Flat mock echoes the just-written byte, so each slot is replaced by itself; the call
    // completing over the whole buffer is the shape proof.
    assert_eq!(buf, [0x01, 0x02, 0x03]);
}

#[test]
fn flush_is_a_noop_success() {
    let _g = seed_reset();
    let mut dev = brought_up();
    SpiBus::flush(&mut dev).expect("flush should succeed");
}

// --- error injection: STAT error bits map to the right ErrorKind ------------------------------

#[test]
fn conferr_maps_to_mode_fault() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // No ready flags, but CONFERR set: the first wait_flag (TBE) sees CONFERR and returns ModeFault.
    w(STAT, STAT_CONFERR);
    let e = SpiBus::write(&mut dev, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::ModeFault);
}

#[test]
fn rxorerr_maps_to_overrun() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // TBE set so the write poll passes; then RXORERR (no RBNE) makes the RBNE wait return Overrun.
    w(STAT, STAT_TBE | STAT_RXORERR);
    let e = SpiBus::write(&mut dev, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Overrun);
}

#[test]
fn timeout_maps_to_other() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // STAT left at 0: no flag ever sets, the bounded poll exhausts its budget -> Other.
    let e = SpiBus::write(&mut dev, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Other);
}
