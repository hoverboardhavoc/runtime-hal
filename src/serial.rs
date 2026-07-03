//! HAL-owned non-blocking `embedded-io` serial adapters (`specs/serial-adapters.md`).
//!
//! Two adapters, one contract:
//!
//! - [`PolledSerial`] over a whole [`Usart`] (polled RBNE RX + polled TX): the bring-up / low-rate /
//!   BLE-module shape.
//! - [`SplitSerial<R>`] over a split [`UsartTx`] + an owned RX backend ([`RingBufferedRx`] or
//!   [`BufferedRx`]): the DMA / interrupt link shape.
//!
//! The shared contract (`specs/serial-adapters.md` D2-D4):
//!
//! - **Non-blocking `Read`, `Ok(0)` = nothing available.** A deliberate, documented deviation from
//!   strict `embedded-io` EOF semantics (a UART has no EOF); consumers gate on `ReadReady`
//!   (`link::SerialTransport` and `ble` both do).
//! - **Errors are owned below the API line; `Error = Infallible`.** The IDLE latch is consumed
//!   internally (framing is the L2 framer's job; IDLE is never a byte-stream boundary), and line
//!   errors / overruns are cleared-and-absorbed by the paths underneath (which never latch), with a
//!   saturating `line_errors` diagnostic counter outside the `embedded-io` traits. A consumer never
//!   calls `take_idle` and never learns ORE exists.
//! - **`ReadReady` = "a read will make progress"** (data available, or a pending condition the read
//!   would clear), answered from real state, consuming nothing.
//! - **`Write` is polled-blocking per byte** (TBE/TC waits; DMA TX is out of scope), `flush` a
//!   no-op beyond the per-byte TC wait; **`WriteReady` = TBE**.
//!
//! The pre-slice-4 BLOCKING `UsartSerial` is superseded by [`PolledSerial`]
//! (`specs/serial-adapters.md` D6); the [`Serial`] alias and its pin-handle `new` constructor carry
//! over. The GD/ST register names live in [`crate::usart`]; what crosses into `embedded-io` is just
//! bytes.

use core::convert::Infallible;

use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::error::{DescriptorError, UsartError};
use crate::gpio::Pin;
use crate::usart::{Usart, UsartTx};
use crate::usart_rx::{BufferedRx, RingBufferedRx};

/// The headline alias: application code reads `Serial::new(&chip, &clock, instance, (pa2, pa3),
/// baud)` (the pin-handle constructor), the [`crate::i2c::I2c`] analogue for the serial port.
pub type Serial = PolledSerial;

// --- the polled adapter -------------------------------------------------------------------------

/// Non-blocking `embedded-io` endpoint over a whole (unsplit) [`Usart`]: polled RBNE receive +
/// polled transmit. See the module docs for the contract. Like [`Usart`] it is NOT `Copy`/`Clone`
/// (`specs/usart-split.md` D1): it IS the one handle to its peripheral, wrapped in the
/// `embedded-io` seam.
#[derive(Debug)]
pub struct PolledSerial {
    usart: Usart,
    line_errors: u16,
}

impl PolledSerial {
    /// Bring up a USART CONSUMING the TX/RX [`Pin`] handles from `split()` and wrap it in the
    /// `embedded-io` serial seam, the headline pin-handle constructor.
    ///
    /// The application passes the named pins from `chip.gpioa().split()` (e.g. `gpioa.pa2` /
    /// `gpioa.pa3`), never a packed `(port << 4) | pin` byte and never the
    /// [`crate::descriptor::GpioPath`] register model: this is the serial-port analogue of
    /// [`crate::i2c::I2c::new`]. The frame is the M1 default 8N1 with oversample /16. It delegates
    /// to [`Usart::new`] (which enables the USART clock, configures the AF pins, and programs the
    /// BAUD + frame) and returns the endpoint ready for the `embedded-io` traits. Generic over the
    /// pins' current mode markers (they arrive in their reset state from `split()`).
    #[inline]
    pub fn new<TX, RX>(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        pins: (Pin<TX>, Pin<RX>),
        baud: u32,
    ) -> Result<Self, DescriptorError> {
        Ok(Self::from_usart(Usart::new(
            chip, clock, instance, pins, baud,
        )?))
    }

    /// Wrap an already-configured [`Usart`] handle (the low-level path for code that brought the
    /// USART up itself, e.g. via [`Usart::bring_up`] or after a [`Usart::rejoin`]).
    #[inline]
    pub const fn from_usart(usart: Usart) -> Self {
        Self {
            usart,
            line_errors: 0,
        }
    }

    /// Unwrap into the underlying [`Usart`] handle (for code that needs the register-level
    /// primitives, or the split path). Consumes the endpoint: the `Usart` is the one handle to the
    /// peripheral.
    #[inline]
    pub fn into_usart(self) -> Usart {
        self.usart
    }

    /// Diagnostic: line errors (framing / parity) absorbed by `read` so far, saturating. Overrun
    /// recovery happens inside [`Usart::try_read_byte`] and never surfaces at all (it is not
    /// separately countable here). Outside the `embedded-io` traits by design
    /// (`specs/serial-adapters.md` D3).
    #[inline]
    pub fn line_errors(&self) -> u16 {
        self.line_errors
    }
}

impl ErrorType for PolledSerial {
    type Error = Infallible;
}

impl Read for PolledSerial {
    /// Non-blocking: drain every byte currently available (each [`Usart::try_read_byte`] also
    /// clears an overrun the family-correct way), stopping at the first empty poll; `Ok(0)` when
    /// nothing is available. A line error (framing/parity) is absorbed: the suspect byte is
    /// dropped, the counter ticks, and this read returns what it already has (`try_read_byte`
    /// cleared the flag, so the next read continues cleanly).
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        while n < out.len() {
            match self.usart.try_read_byte() {
                Ok(Some(b)) => {
                    out[n] = b;
                    n += 1;
                }
                Ok(None) => break,
                Err(_) => {
                    self.line_errors = self.line_errors.saturating_add(1);
                    break;
                }
            }
        }
        Ok(n)
    }
}

impl ReadReady for PolledSerial {
    /// RX-not-empty (RBNE), or an overrun pending (which `read`'s `try_read_byte` will clear):
    /// either way a `read` makes progress. A non-consuming status-flag check.
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        let s = self.usart.read_status();
        Ok(s.rx_ready || s.overrun)
    }
}

impl Write for PolledSerial {
    /// Blocking polled write: send every byte via [`Usart::write_byte`] (TBE before, TC after).
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        for &b in data {
            self.usart.write_byte(b);
        }
        Ok(data.len())
    }

    /// `write_byte` already polled TC per byte, so the line is drained; re-confirm TC so a caller
    /// that wrote by some other route still gets a real flush.
    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.usart.read_status().tx_complete {}
        Ok(())
    }
}

impl WriteReady for PolledSerial {
    /// `TBE`: the transmit data register is empty, so the next byte can be written without waiting
    /// on TBE.
    #[inline]
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.usart.read_status().tx_empty)
    }
}

// --- the split adapter (DMA / interrupt RX + polled TX) ------------------------------------------

mod private {
    /// Seals [`super::RxBackend`]: exactly the two in-crate receivers implement it.
    pub trait Sealed {}
    impl Sealed for crate::usart_rx::RingBufferedRx {}
    impl Sealed for crate::usart_rx::BufferedRx {}
}

/// The RX side a [`SplitSerial`] adapts: [`RingBufferedRx`] (DMA ring) or [`BufferedRx`]
/// (interrupt SPSC ring). Sealed (`specs/serial-adapters.md` D1): the quirk ownership the adapter
/// promises (IDLE latch, overrun clearing) is a property of these two implementations, not an
/// extension point.
pub trait RxBackend: private::Sealed {
    /// Drain available bytes (the backend's non-blocking `read`).
    #[doc(hidden)]
    fn rx_read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError>;
    /// True if a read will make progress (data, or a pending condition it would clear).
    #[doc(hidden)]
    fn rx_ready(&self) -> bool;
    /// Read-and-clear the IDLE latch (the adapter consumes it; no consumer ever sees it).
    #[doc(hidden)]
    fn rx_take_idle(&self) -> bool;
}

impl RxBackend for RingBufferedRx {
    fn rx_read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError> {
        self.read(buf)
    }
    fn rx_ready(&self) -> bool {
        self.ready()
    }
    fn rx_take_idle(&self) -> bool {
        self.take_idle()
    }
}

impl RxBackend for BufferedRx {
    fn rx_read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError> {
        self.read(buf)
    }
    fn rx_ready(&self) -> bool {
        self.ready() || self.condition_pending()
    }
    fn rx_take_idle(&self) -> bool {
        self.take_idle()
    }
}

/// Non-blocking `embedded-io` endpoint over a split port: a [`UsartTx`] (polled transmit) plus an
/// owned RX backend (`R`: DMA ring or interrupt ring). The link/firmware shape: RX never blocks the
/// scheduler, TX is polled. See the module docs for the contract.
pub struct SplitSerial<R: RxBackend> {
    tx: UsartTx,
    rx: R,
    line_errors: u16,
}

impl<R: RxBackend> SplitSerial<R> {
    /// Adapt an already-armed receiver + the TX half. The adapter brings nothing up; construction
    /// is explicit from the slice-3 ownership pieces (`specs/serial-adapters.md` D5):
    /// `usart.split()`, arm the backend on the RX half, wrap both here.
    #[inline]
    pub fn new(tx: UsartTx, rx: R) -> Self {
        SplitSerial {
            tx,
            rx,
            line_errors: 0,
        }
    }

    /// Take the adapter apart (the reconfigure path: `into_parts` -> backend `release` ->
    /// [`Usart::rejoin`] -> `set_baud` -> `split` -> re-arm -> `new`).
    #[inline]
    pub fn into_parts(self) -> (UsartTx, R) {
        (self.tx, self.rx)
    }

    /// Diagnostic: conditions absorbed by `read` so far (ring lap, ring overflow, line errors),
    /// saturating. A channel-DISABLING hardware overrun (the ERRIE fail-loud path) also lands here
    /// once, after which reads return 0 until the owner re-arms via [`Self::into_parts`]
    /// (`specs/serial-adapters.md` D3: no silent auto-restart). Outside the `embedded-io` traits by
    /// design.
    #[inline]
    pub fn line_errors(&self) -> u16 {
        self.line_errors
    }
}

impl<R: RxBackend> ErrorType for SplitSerial<R> {
    type Error = Infallible;
}

impl<R: RxBackend> Read for SplitSerial<R> {
    /// Non-blocking: drain what the backend has; `Ok(0)` when nothing is available. The IDLE latch
    /// is consumed (dropped) here so it can never leak to a consumer; an absorbed condition (lap /
    /// overflow / line error) ticks the counter and the drain is retried, so a single `read` still
    /// returns post-recovery bytes where the backend has them.
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        // The adapter owns the IDLE latch: consume it unconditionally (framing belongs to the
        // framer above; the latch must never sit as un-modeled state a consumer misreads).
        let _ = self.rx.rx_take_idle();

        // Each surfaced condition is consume-on-read (the backend clears it as it returns Err), so
        // this loop is finite in practice; bound it anyway so a misbehaving state cannot spin.
        for _ in 0..4 {
            match self.rx.rx_read(out) {
                Ok(n) => return Ok(n),
                Err(_) => {
                    self.line_errors = self.line_errors.saturating_add(1);
                }
            }
        }
        Ok(0)
    }
}

impl<R: RxBackend> ReadReady for SplitSerial<R> {
    /// The backend's own progress state (buffered bytes / bytes behind the live DMA write position,
    /// or a pending condition a read would clear). Consumes nothing.
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.rx_ready())
    }
}

impl<R: RxBackend> Write for SplitSerial<R> {
    /// Blocking polled write on the TX half (TBE before, TC after, per byte).
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        for &b in data {
            self.tx.write_byte(b);
        }
        Ok(data.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(()) // write_byte already waited for TC per byte
    }
}

impl<R: RxBackend> WriteReady for SplitSerial<R> {
    /// `TBE` on the TX half.
    #[inline]
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx.tx_empty())
    }
}

#[cfg(test)]
mod tests;
