//! `embedded-io` serial impl over the polled USART driver (T7, the cold-path seam).
//!
//! [`UsartSerial`] wraps a configured [`Usart`] handle (from T6) and implements the
//! `embedded-io` blocking traits on top of its polled byte primitives ([`Usart::write_byte`],
//! [`Usart::try_read_byte`], [`Usart::read_status`]). This is the "cold path": application byte
//! transfer (the M1 inter-board handshake), not a per-cycle ISR, so a polled blocking impl is the
//! right shape and there is no DMA/IRQ here (DECISIONS.md #4 keeps the hot path elsewhere).
//!
//! The naming convention stays on this side of the trait boundary: the GD/ST register names live
//! in [`crate::usart`]; what crosses into `embedded-io` is just bytes and a [`UsartError`].
//!
//! # error mapping (M1 open item 4, pinned here)
//!
//! `embedded-io`'s `Read` is **blocking**: its contract (`embedded-io` 0.6, `Read::read`) is "if no
//! bytes are currently available to read, this function blocks until at least one byte is
//! available." It does **not** have a `WouldBlock` return. So [`Read::read`] below polls `RBNE`
//! and blocks for the first byte rather than returning a spurious error or `Ok(0)` (`Ok(0)` would
//! be misread as EOF). Only a *line error* (overrun / framing / parity) seen while polling cuts
//! the blocking short and is returned as the mapped [`UsartError`].
//!
//! The [`UsartError`] variants map to [`embedded_io::ErrorKind`] via the `embedded_io::Error` impl
//! in [`crate::error`]: overrun, framing, and parity all fold to `ErrorKind::Other`, because
//! `embedded-io` has no dedicated kinds for those recoverable line conditions. This is the same
//! mapping T1 pinned on [`UsartError`]; it is not re-decided here, just consumed. The
//! `serial_error_kind_is_other` test below ties the two together so the mapping cannot drift.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::error::{DescriptorError, UsartError};
use crate::gpio::Pin;
use crate::usart::Usart;
use embedded_io::{ErrorType, Read, ReadReady, Write, WriteReady};

/// The headline serial-port type: an `embedded-io` endpoint over a configured USART. Alias of
/// [`UsartSerial`] so application code reads `Serial::new(&chip, &clock, instance, (pa2, pa3), baud)`
/// (the pin-handle constructor), the [`crate::i2c::I2c`] analogue for the serial port.
pub type Serial = UsartSerial;

/// `embedded-io` serial endpoint over a configured [`Usart`].
///
/// Construct one with the pin-handle [`UsartSerial::new`] (the headline path), or wrap a handle you
/// brought up yourself via [`UsartSerial::from_usart`]. Like [`Usart`] it is NOT `Copy`/`Clone`
/// (`specs/usart-split.md` D1): it IS the one handle to its peripheral, wrapped in the
/// `embedded-io` seam. The blocking `Read`/`Write` impls poll the USART status flags (RBNE / TBE /
/// TC) and the line-error flags (overrun / framing / parity).
#[derive(Debug)]
pub struct UsartSerial {
    usart: Usart,
}

impl UsartSerial {
    /// Bring up a USART CONSUMING the TX/RX [`Pin`] handles from `split()` and wrap it in the
    /// `embedded-io` serial seam, the headline pin-handle constructor.
    ///
    /// The application passes the named pins from `chip.gpioa().split()` (e.g. `gpioa.pa2` /
    /// `gpioa.pa3`), never a packed `(port << 4) | pin` byte and never the
    /// [`crate::descriptor::GpioPath`] register model: this is the serial-port analogue of
    /// [`crate::i2c::I2c::new`]. The frame is the M1 default 8N1 with oversample /16. It delegates to
    /// [`Usart::new`] (which enables the USART clock, configures the AF pins, and programs the BAUD +
    /// frame) and returns the endpoint ready for the `embedded-io` `Read`/`Write` traits.
    ///
    /// Generic over the pins' current mode markers `TX` / `RX` (they arrive in their reset
    /// [`crate::gpio::Input`] state from `split()`); `new` reconfigures them and so takes them by
    /// value.
    #[inline]
    pub fn new<TX, RX>(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        pins: (Pin<TX>, Pin<RX>),
        baud: u32,
    ) -> Result<Self, DescriptorError> {
        Ok(Self {
            usart: Usart::new(chip, clock, instance, pins, baud)?,
        })
    }

    /// Wrap an already-configured [`Usart`] handle in the `embedded-io` serial seam (the low-level
    /// path for code that brought the USART up itself, e.g. via [`Usart::bring_up`]). The pin-handle
    /// [`UsartSerial::new`] is the headline constructor.
    #[inline]
    pub const fn from_usart(usart: Usart) -> Self {
        Self { usart }
    }

    /// Unwrap into the underlying [`Usart`] handle (for code that needs the register-level
    /// primitives). Consumes the endpoint: the `Usart` is the one handle to the peripheral.
    #[inline]
    pub fn into_usart(self) -> Usart {
        self.usart
    }
}

impl ErrorType for UsartSerial {
    type Error = UsartError;
}

impl Read for UsartSerial {
    /// Blocking read, per the `embedded-io` `Read` contract: block until at least one byte is
    /// available, then drain whatever is immediately ready up to `buf.len()`.
    ///
    /// Empty-buffer fast path: `read([])` returns `Ok(0)` without blocking (the contract says a
    /// zero-length buffer must not block and its `Ok(0)` is not EOF).
    ///
    /// A line error (overrun / framing / parity) observed while polling is returned immediately as
    /// the mapped [`UsartError`]; if it surfaces after some bytes were already taken, those bytes
    /// would be lost to the error, so the error is checked before each byte and short-circuits.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        // Block for the first byte (embedded-io Read blocks; it has no WouldBlock). `try_read_byte`
        // returns Err on a line error, Ok(None) when RBNE is clear (keep polling), Ok(Some) on a
        // byte. The first slot is filled by this loop, so we always return at least one byte unless
        // a line error intervenes.
        let mut filled = 0usize;
        loop {
            if let Some(b) = self.usart.try_read_byte()? {
                buf[filled] = b;
                filled += 1;
                break;
            }
        }

        // Drain what is already ready, up to buf.len(), without blocking. Stop at the first slot
        // where no byte is ready (Ok(None)); a line error short-circuits with `?`.
        while filled < buf.len() {
            match self.usart.try_read_byte()? {
                Some(b) => {
                    buf[filled] = b;
                    filled += 1;
                }
                None => break,
            }
        }

        Ok(filled)
    }
}

impl Write for UsartSerial {
    /// Blocking write: send every byte of `buf` via the polled [`Usart::write_byte`] (which polls
    /// TBE before writing and TC after), then return `buf.len()`.
    ///
    /// `write_byte` already waits for TBE/TC per byte, so the whole buffer is sent here rather than
    /// returning a short count. Empty buffer returns `Ok(0)` without touching the peripheral.
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &b in buf {
            self.usart.write_byte(b);
        }
        Ok(buf.len())
    }

    /// Flush: wait for transmission to complete (TC). [`Usart::write_byte`] already polls TC after
    /// each byte, so by the time `write` returns the line is drained; this re-confirms TC so a
    /// caller that wrote bytes by some other route still gets a real flush.
    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.usart.read_status().tx_complete {}
        Ok(())
    }
}

impl ReadReady for UsartSerial {
    /// `RBNE`: a byte is in the read data register, so the next [`Read::read`] will not block.
    #[inline]
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.usart.read_status().rx_ready)
    }
}

impl WriteReady for UsartSerial {
    /// `TBE`: the transmit data register is empty, so the next byte can be written without blocking
    /// on TBE.
    #[inline]
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.usart.read_status().tx_empty)
    }
}

#[cfg(test)]
mod tests;
