//! Shared SPI driver (T8 bring-up + T9 full-duplex transfer / `embedded-hal` `spi::SpiBus`).
//!
//! This is the single SPI bring-up + transfer path. SPEC.md: SPI is one shared path parameterised
//! by base address. Like [`crate::i2c`], the SPI peripheral register model is **identical on F10x
//! and F1x0** (verified against `gd32f10x_spi.h` and `gd32f1x0_spi.h`: CTL0 0x00, CTL1 0x04,
//! STAT 0x08, DATA 0x0C, and the same bit positions), so there is **one register model shared by
//! both families**; the path is parameterised only by the base address (data, from
//! [`crate::addr::AddrTable`]) and by the bus clock (from the [`ClockConfig`]). There is no
//! [`crate::descriptor::GpioPath`]-style selector here.
//!
//! SPI is NOT exercised by the hoverboard's own firmware, so the M2 SPI target is **general
//! `embedded-hal` correctness** (so off-the-shelf drivers work), anchored later (T13, NOT this
//! task) by a MOSI/MISO loopback.
//!
//! # Register model (identical on both families)
//!
//! | reg    | offset | what                                                                 |
//! |--------|--------|----------------------------------------------------------------------|
//! | `CTL0` | `0x00` | CKPH(0) CKPL(1) MSTMOD(2) `PSC[5:3]` SPIEN(6) LF(7) SWNSS(8) SWNSSEN(9) FF16(11) |
//! | `CTL1` | `0x04` | NSSDRV(2) etc. (left at reset; software NSS needs no NSSDRV)          |
//! | `STAT` | `0x08` | RBNE(0) TBE(1) CRCERR(4) CONFERR(5) RXORERR(6) TRANS(7)               |
//! | `DATA` | `0x0C` | transfer buffer (TX write / RX read)                                 |
//! | `I2SCTL`| `0x1C`| I2SSEL(11); `spi_init` clears it to select SPI (not I2S) mode        |
//!
//! # Bring-up (T8, the SPL `spi_init` / `spi_enable`)
//!
//! [`Spi::bring_up`] reproduces the SPL `spi_init` exactly: it reads `CTL0`, masks it with
//! `SPI_INIT_MASK` (`0x3040`, keeping only the SPIEN/CRCEN/reserved bits the SPL preserves) and ORs
//! in the parameter struct fields, then writes `CTL0`; then clears `I2SCTL` I2SSEL to select SPI
//! mode; then `spi_enable` sets `CTL0` SPIEN. The parameter struct M2 fixes:
//!
//! - **device_mode** = `SPI_MASTER` (`MSTMOD | SWNSS`, bits 2 and 8: master, NSS internally high).
//! - **trans_mode** = `SPI_TRANSMODE_FULLDUPLEX` (0).
//! - **frame_size** = 8 or 16 bit ([`DataSize`]; `FF16` bit 11 set for 16-bit).
//! - **nss** = `SPI_NSS_SOFT` (`SWNSSEN`, bit 9): **software-managed NSS** (open item SPI-2). The
//!   `embedded-hal` `SpiBus` trait does NOT own CS, so the caller owns chip-select; software NSS
//!   (`SSM` = `SWNSSEN` set, `SSI` = `SWNSS` set, which `SPI_MASTER` already carries) keeps the
//!   master out of a spurious mode fault when no hardware NSS line is wired. This is the
//!   `embedded-hal`-correct default and the one a loopback (T13) needs.
//! - **endian** = `SPI_ENDIAN_MSB` (0): MSB-first.
//! - **clock_polarity_phase** = from the `embedded-hal` [`spi::Mode`] (CPOL -> `CKPL` bit 1,
//!   CPHA -> `CKPH` bit 0), via [`mode_bits`].
//! - **prescale** = the smallest `SPI_PSC_n` (2,4,..256) that keeps the SPI clock at or below the
//!   target ([`prescaler_for`]), from the bus clock (SPI0 on APB2, SPI1 on APB1).
//!
//! # Transfer (T9, the polled full-duplex handshake)
//!
//! Full-duplex byte transfer mirrors the SPL polled example: wait `STAT` TBE, write `DATA`, wait
//! `STAT` RBNE, read `DATA`. Every poll is bounded ([`SPI_TIMEOUT`]) so a stuck bus cannot hang.
//! A concurrently-set `CONFERR` (mode fault) or `RXORERR` (overrun) during a poll surfaces as the
//! mapped [`SpiError`]; a budget exhaustion is [`SpiError::Other`] (a polled timeout, which
//! `embedded-hal` 1.0 `spi::ErrorKind` does not name, so it folds into `Other`).
//!
//! The GD/ST register naming stays on this side of the trait boundary (SPEC.md): what crosses into
//! `embedded-hal` `spi::SpiBus` is bytes and a [`SpiError`].

use embedded_hal::spi::{self, Mode, SpiBus, MODE_0};

use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{NssMode, SpiConfig};
use crate::error::{DescriptorError, SpiError};
use crate::reg::Reg32;

// --- register offsets (identical on both families) --------------------------------------------

const CTL0: u32 = 0x00;
#[allow(dead_code)]
const CTL1: u32 = 0x04;
const STAT: u32 = 0x08;
const DATA: u32 = 0x0C;
const I2SCTL: u32 = 0x1C;

// CTL0 bits (gd32f1x0_spi.h / gd32f10x_spi.h, identical).
const CTL0_CKPH: u32 = 1 << 0;
const CTL0_CKPL: u32 = 1 << 1;
const CTL0_MSTMOD: u32 = 1 << 2;
const CTL0_PSC: u32 = 0x7 << 3; // BITS(3,5)
const CTL0_SPIEN: u32 = 1 << 6;
/// LF (LSB-first) bit; set for LSB-first, clear for MSB-first (the application's `lsb_first` choice).
const CTL0_LF: u32 = 1 << 7;
const CTL0_SWNSS: u32 = 1 << 8;
const CTL0_SWNSSEN: u32 = 1 << 9;
const CTL0_FF16: u32 = 1 << 11;

/// `SPI_INIT_MASK` from the SPL `spi_init`: the CTL0 bits preserved across an init (the rest are
/// cleared and re-ORed from the parameter struct). `0x3040` = bits 6 (SPIEN), 12 (CRCNT),
/// 13 (CRCEN). At a reset-0 CTL0 the mask keeps nothing, so the end state is purely the ORed
/// parameter bits; reproduced verbatim so the trace matches the SPL byte-for-byte.
const SPI_INIT_MASK: u32 = 0x0000_3040;

/// `SPI_MASTER` = `MSTMOD | SWNSS` (the SPL macro): master mode with NSS internally driven high.
const SPI_MASTER: u32 = CTL0_MSTMOD | CTL0_SWNSS;
/// `SPI_NSS_SOFT` = `SWNSSEN` (software NSS management).
const SPI_NSS_SOFT: u32 = CTL0_SWNSSEN;
/// `SPI_ENDIAN_MSB` = 0 (MSB-first).
const SPI_ENDIAN_MSB: u32 = 0;
/// `SPI_ENDIAN_LSB` = `LF` (LSB-first).
const SPI_ENDIAN_LSB: u32 = CTL0_LF;
/// `SPI_TRANSMODE_FULLDUPLEX` = 0.
const SPI_TRANSMODE_FULLDUPLEX: u32 = 0;

// I2SCTL I2SSEL bit (cleared to select SPI, not I2S, mode).
const I2SCTL_I2SSEL: u32 = 1 << 11;

// STAT flags.
const STAT_RBNE: u32 = 1 << 0;
const STAT_TBE: u32 = 1 << 1;
const STAT_CONFERR: u32 = 1 << 5;
const STAT_RXORERR: u32 = 1 << 6;

/// Bounded poll budget for a single status-flag wait (TBE / RBNE). Counts loop iterations, not
/// cycles, so it is clock-independent; generous enough never to false-time a working byte timing
/// at any representative SPI clock, but always escaping a dead bus (the F130 hang-if-done-wrong
/// class). Mirrors [`crate::i2c::I2C_TIMEOUT`].
pub const SPI_TIMEOUT: u32 = 100_000;

/// SPI frame size: 8 or 16 bit (the SPL `SPI_FRAMESIZE_8BIT` / `SPI_FRAMESIZE_16BIT`).
///
/// `embedded-hal` `SpiBus<u8>` (the impl this module provides) is 8-bit; the 16-bit option is
/// carried in the descriptor and the bring-up so a future `SpiBus<u16>` impl can use it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataSize {
    /// 8-bit frames (`SPI_FRAMESIZE_8BIT`, FF16 clear). The M2 default.
    #[default]
    Eight,
    /// 16-bit frames (`SPI_FRAMESIZE_16BIT`, FF16 set).
    Sixteen,
}

impl DataSize {
    /// The `CTL0` bit this frame size contributes (FF16 for 16-bit, nothing for 8-bit).
    #[inline]
    const fn ctl0_bit(self) -> u32 {
        match self {
            DataSize::Eight => 0,
            DataSize::Sixteen => CTL0_FF16,
        }
    }
}

/// Map an `embedded-hal` [`spi::Mode`] (CPOL + CPHA) to the GD `CTL0` polarity/phase bits, exactly
/// as the SPL `SPI_CK_PL_*_PH_*EDGE` macros do: CPOL high -> `CKPL` (bit 1), CPHA "second edge"
/// (capture on the second clock transition) -> `CKPH` (bit 0).
#[inline]
pub fn mode_bits(mode: Mode) -> u32 {
    use spi::{Phase, Polarity};
    let mut bits = 0;
    if mode.polarity == Polarity::IdleHigh {
        bits |= CTL0_CKPL;
    }
    if mode.phase == Phase::CaptureOnSecondTransition {
        bits |= CTL0_CKPH;
    }
    bits
}

/// Choose the smallest `SPI_PSC_n` prescaler (divisor 2, 4, .. 256) that keeps the resulting SPI
/// clock (`bus_hz / div`) at or below `target_hz`, and return the **`PSC` field code** (0..=7) the
/// GD `CTL0` PSC bits take (`SPI_PSC_2` = 0, `SPI_PSC_4` = 1, .. `SPI_PSC_256` = 7), matching the
/// SPL `CTL0_PSC(regval)` encoding `(regval << 3)`.
///
/// The SPI baud prescaler divides only by powers of two from 2 to 256, so this walks the eight
/// divisors and picks the first that does not exceed the target. If even /256 is too fast for the
/// target (i.e. `bus_hz / 256 > target_hz`), the slowest (/256, code 7) is used (best effort,
/// clamped); if the bus is already at or below the target at /2, code 0 is used.
pub fn prescaler_for(bus_hz: u32, target_hz: u32) -> u32 {
    // Divisor = 2 << code; code 0 -> /2, code 7 -> /256.
    let mut code = 0u32;
    while code < 7 {
        let div = 2u32 << code; // 2, 4, 8, ... 256
        if bus_hz / div <= target_hz {
            break;
        }
        code += 1;
    }
    code
}

/// Derive the SPI peripheral clock in Hz from a [`ClockConfig`] and which APB bus the instance
/// sits on. SPI0 is on **APB2**, SPI1 on **APB1** (both families). APB2 = AHB / `apb2_psc`,
/// APB1 = AHB / `apb1_psc`, with AHB = `sysclk / ahb_psc` (the same chain the SPL
/// `rcu_clock_freq_get` walks and that [`crate::i2c::i2c_input_clock`] uses for APB1).
#[inline]
pub fn spi_input_clock(clock: &ClockConfig, on_apb2: bool) -> u32 {
    let ahb = clock.sysclk_hz / clock.ahb_psc.max(1) as u32;
    if on_apb2 {
        ahb / clock.apb2_psc.max(1) as u32
    } else {
        ahb / clock.apb1_psc.max(1) as u32
    }
}

// --- the handle -------------------------------------------------------------------------------

/// A configured SPI master, resolved once at bring-up: just the base (the register model is shared,
/// so there is no per-family field). The polled transfer primitives and the `embedded-hal`
/// `spi::SpiBus` impl hang off this (DECISIONS.md #4: resolve once into a concrete handle).
#[derive(Debug, Clone, Copy)]
pub struct Spi {
    base: u32,
}

impl Spi {
    /// Bring up the SPI master at `base` for the supplied bus clock + target SPI clock, mode, and
    /// frame size, reproducing the SPL `spi_init` then `spi_enable`.
    ///
    /// `bus_hz` is the SPI peripheral clock (APB2 for SPI0, APB1 for SPI1; derive it with
    /// [`spi_input_clock`]). `target_hz` is the wanted SCK frequency; the prescaler is the smallest
    /// power-of-two divisor that keeps SCK at or below it ([`prescaler_for`]). `mode` is the
    /// `embedded-hal` CPOL/CPHA. NSS is **software-managed** (the `embedded-hal` default; open item
    /// SPI-2).
    pub fn bring_up(
        chip: &Chip,
        clock: &ClockConfig,
        cfg: &SpiConfig,
    ) -> Result<Spi, DescriptorError> {
        let base = chip.base(cfg.spi)?;
        let on_apb2 = matches!(cfg.spi, crate::addr::PeriphLabel::Spi0);
        let bus_hz = spi_input_clock(clock, on_apb2);
        let dev = Spi { base };
        let psc_code = prescaler_for(bus_hz, cfg.target_hz);
        dev.configure(
            cfg.mode(),
            cfg.data_size(),
            psc_code,
            cfg.lsb_first,
            cfg.nss_mode,
        );
        Ok(dev)
    }

    /// Program `CTL0` (the SPL `spi_init` parameter assembly), clear `I2SCTL` I2SSEL, then set
    /// `CTL0` SPIEN (the SPL `spi_enable`). `psc_code` is the `PSC` field value (0..=7); `lsb_first`
    /// and `nss_mode` are the application's explicit bit-order / NSS choices (no baked MSB / soft).
    fn configure(
        &self,
        mode: Mode,
        data_size: DataSize,
        psc_code: u32,
        lsb_first: bool,
        nss_mode: NssMode,
    ) {
        // SPL spi_init: reg = CTL0 & SPI_INIT_MASK; then OR the parameter-struct fields.
        let mut reg = self.ctl0().read() & SPI_INIT_MASK;
        reg |= SPI_MASTER; // device_mode (MSTMOD | SWNSS)
        reg |= SPI_TRANSMODE_FULLDUPLEX; // trans_mode (0)
        reg |= data_size.ctl0_bit(); // frame_size (FF16 for 16-bit)
                                     // nss: software-managed (SWNSSEN) vs hardware (NSSDRV in CTL1, no SWNSSEN here).
        if let NssMode::Software = nss_mode {
            reg |= SPI_NSS_SOFT;
        }
        // endian: MSB-first (0) vs LSB-first (LF bit).
        reg |= if lsb_first {
            SPI_ENDIAN_LSB
        } else {
            SPI_ENDIAN_MSB
        };
        reg |= mode_bits(mode); // clock_polarity_phase (CKPL/CKPH)
        reg |= (psc_code << 3) & CTL0_PSC; // prescale (PSC field, CTL0_PSC(regval))
        self.ctl0().write(reg);

        // SPL spi_init tail: select SPI (not I2S) mode by clearing I2SCTL I2SSEL.
        self.i2sctl().modify(I2SCTL_I2SSEL, 0);

        // SPL spi_enable: set SPIEN.
        self.ctl0().modify(CTL0_SPIEN, CTL0_SPIEN);
    }

    /// The underlying base address (for code that needs the register-level view).
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    // --- register accessors -------------------------------------------------------------------

    #[inline]
    fn ctl0(&self) -> Reg32 {
        Reg32::new(self.base, CTL0)
    }
    #[inline]
    fn stat(&self) -> Reg32 {
        Reg32::new(self.base, STAT)
    }
    #[inline]
    fn data(&self) -> Reg32 {
        Reg32::new(self.base, DATA)
    }
    #[inline]
    fn i2sctl(&self) -> Reg32 {
        Reg32::new(self.base, I2SCTL)
    }

    // --- low-level primitives (the SPL `spi_i2s_*` calls, GD-named) ---------------------------

    /// `spi_i2s_data_transmit`: write a byte to `DATA`.
    #[inline]
    fn transmit(&self, byte: u8) {
        self.data().write(byte as u32);
    }

    /// `spi_i2s_data_receive`: read a byte from `DATA`.
    #[inline]
    fn receive(&self) -> u8 {
        (self.data().read() & 0xFF) as u8
    }

    /// Poll `STAT` until `flag` (TBE or RBNE) is set, mapping a concurrently-set error flag
    /// (`CONFERR` mode fault / `RXORERR` overrun) to the corresponding [`SpiError`], and a budget
    /// exhaustion to [`SpiError::Other`] (a polled timeout, unnamed by `embedded-hal` 1.0
    /// `spi::ErrorKind`). The error checks are the same `spi_i2s_flag_get` reads a polled SPL
    /// example does before trusting the data register.
    fn wait_flag(&self, flag: u32) -> Result<(), SpiError> {
        let mut budget = SPI_TIMEOUT;
        loop {
            let s = self.stat().read();
            if s & flag != 0 {
                return Ok(());
            }
            // Surface a mode fault / overrun if it appears while waiting (CONFERR before RXORERR;
            // a mode fault is the more fundamental misconfiguration).
            if s & STAT_CONFERR != 0 {
                return Err(SpiError::ModeFault);
            }
            if s & STAT_RXORERR != 0 {
                return Err(SpiError::Overrun);
            }
            budget -= 1;
            if budget == 0 {
                // A polled timeout has no dedicated embedded-hal 1.0 spi::ErrorKind, so it folds
                // into Other (the same policy i2c uses for its Timeout variant).
                return Err(SpiError::Other);
            }
        }
    }

    /// Full-duplex transfer of one byte: wait TBE, write `tx` to `DATA`, wait RBNE, read the
    /// received byte. This is the polled full-duplex unit the SPL example shifts a byte with; every
    /// `embedded-hal` method below is built from it.
    pub fn transfer_byte(&self, tx: u8) -> Result<u8, SpiError> {
        self.wait_flag(STAT_TBE)?;
        self.transmit(tx);
        self.wait_flag(STAT_RBNE)?;
        Ok(self.receive())
    }
}

// --- embedded-hal 1.0 spi::SpiBus -------------------------------------------------------------

impl spi::ErrorType for Spi {
    type Error = SpiError;
}

impl SpiBus<u8> for Spi {
    /// Read `words.len()` bytes: clock out a dummy `0x00` per byte (full-duplex needs a write to
    /// generate the clock) and store each received byte.
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(0x00)?;
        }
        Ok(())
    }

    /// Write `words`: clock each byte out, discarding the simultaneously-received bytes (the
    /// full-duplex read still happens on the wire; `SpiBus::write` drops it).
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &w in words.iter() {
            let _ = self.transfer_byte(w)?;
        }
        Ok(())
    }

    /// Full-duplex transfer: clock out `write`, capturing into `read`. The buffers may differ in
    /// length (per the trait): once `write` is exhausted, clock `0x00` to keep reading; once `read`
    /// is full, the received byte is discarded but the write byte is still clocked.
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let n = read.len().max(write.len());
        for i in 0..n {
            let tx = write.get(i).copied().unwrap_or(0x00);
            let rx = self.transfer_byte(tx)?;
            if let Some(slot) = read.get_mut(i) {
                *slot = rx;
            }
        }
        Ok(())
    }

    /// In-place full-duplex transfer: each byte is clocked out and replaced by the received byte.
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(*w)?;
        }
        Ok(())
    }

    /// Flush: the polled byte transfers complete synchronously (each waits RBNE before returning),
    /// so there is no buffered word in flight to drain; flush is a no-op success. (A DMA/interrupt
    /// implementation would wait for TRANS to clear here.)
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// MODE_0 (CPOL = 0, CPHA = 0) re-exported for callers and tests that want the common default.
pub const DEFAULT_MODE: Mode = MODE_0;

#[cfg(test)]
mod tests;
