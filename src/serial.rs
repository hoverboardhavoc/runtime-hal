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
//!   errors / overruns are cleared-and-absorbed by the paths underneath (which never latch), with
//!   saturating diagnostic counters outside the `embedded-io` traits. A consumer never calls
//!   `take_idle` and never learns ORE exists.
//! - **Absorbed is not unmeasured, and the two causes stay apart (the OQ1 split).** Both adapters
//!   count a WIRE disturbance (framing / parity / noise) separately from a LOST-BYTE condition (the
//!   consumer fell behind): `line_errors` vs [`SplitSerial::lap_overruns`] on the DMA/interrupt
//!   shape, `line_errors` vs [`PolledSerial::rx_overruns`] on the polled shape. Keeping them apart
//!   is what lets a bench read attribute a loss: a noisy wire must never be able to masquerade as a
//!   consumer that stopped reading for too long, which is the whole question on the BLE port.
//! - **`ReadReady` = "a read will make progress"** (data available, or a pending condition the read
//!   would clear), answered from real state, consuming nothing.
//! - **`Write` is polled, `WriteReady` = TBE**, and DMA TX is out of scope. The two adapters differ
//!   in what they wait for, because their carriers do: [`SplitSerial`] waits `TBE` then `TC` per
//!   byte (the inter-board UART at 115200, where a character time is 87 us and the send is the
//!   whole point of the pass), so its `flush` is a no-op. [`PolledSerial`] waits `TBE` only, taking
//!   as many bytes as the register accepts and reporting how many, so a caller can meter a frame
//!   onto a slow wire without blocking (the 9600-baud BLE module: 1.04 ms a byte against a 4 ms
//!   control tick); its `flush` is the real `TC` wait, and is what a caller owes before
//!   reconfiguring or tearing down the port.
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
    rx_overruns: u16,
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
            rx_overruns: 0,
        }
    }

    /// Unwrap into the underlying [`Usart`] handle (for code that needs the register-level
    /// primitives, or the split path). Consumes the endpoint: the `Usart` is the one handle to the
    /// peripheral.
    #[inline]
    pub fn into_usart(self) -> Usart {
        self.usart
    }

    /// Diagnostic: WIRE disturbances (framing / parity) absorbed by `read` so far, saturating.
    /// Counted SEPARATELY from [`rx_overruns`](Self::rx_overruns) (a lost-byte condition), the OQ1
    /// split as it applies to the polled path. Outside the `embedded-io` traits by design
    /// (`specs/serial-adapters.md` D3).
    #[inline]
    pub fn line_errors(&self) -> u16 {
        self.line_errors
    }

    /// Diagnostic: receive OVERRUN conditions absorbed by `read` so far, saturating: the polled
    /// path's lost-byte counter, and the polled half of the OQ1 split that [`SplitSerial`] answers
    /// with `lap_overruns`.
    ///
    /// A one-byte receive register IS the polled path's buffer, so an overrun means exactly what a
    /// lap means on the DMA ring: the consumer did not come back in time and at least one inbound
    /// byte is gone. The recovery itself stays silent and automatic ([`Usart::try_read_byte`]
    /// clears ORE and keeps reading, so RX can never strand), but silent is not the same as
    /// unmeasured: this counts the conditions the recovery absorbed.
    ///
    /// One condition, not one byte. ORE latches on the first byte lost and stays set until cleared,
    /// so an overrun observed across a long blind window scores 1 however many characters fell in
    /// it. It is a yes/no answer about whether the consumer's absence cost bytes, and how often,
    /// which is the question a control loop that occupies the CPU for longer than a character time
    /// raises.
    #[inline]
    pub fn rx_overruns(&self) -> u16 {
        self.rx_overruns
    }

    /// Model ONE character arriving from the wire (host `mock` only): passthrough to
    /// [`crate::usart::Usart::inject_rx_byte`]. Deliberately the SAME name and shape as
    /// [`SplitSerial::inject_rx_byte`], so one caller-away property test can be written once and
    /// compiled against whichever carrier a port actually ships.
    #[cfg(feature = "mock")]
    #[inline]
    pub fn inject_rx_byte(&self, b: u8) {
        self.usart.inject_rx_byte(b);
    }

    /// Declare the wrapped instance's device read side effects to the mock register space
    /// ([`crate::usart::UsartRegs::mock_declare_device_reads`]; host `mock` only).
    #[cfg(feature = "mock")]
    #[inline]
    pub fn mock_declare_device_reads(&self) {
        self.usart.mock_declare_device_reads();
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
    /// cleared the flag, so the next read continues cleanly). An overrun is absorbed too, and
    /// ticks [`rx_overruns`](PolledSerial::rx_overruns) instead: the drain continues either way.
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        let mut n = 0;
        while n < out.len() {
            // Observe ORE BEFORE `try_read_byte` recovers it. The recovery is deliberately
            // invisible to the caller (it must never surface as `Err` or strand RX), so this
            // status read is the ONLY point at which the lost byte is still evidence. It costs one
            // extra register read per drained byte on a polled port, which is the port that runs
            // at 9600 baud.
            if self.usart.read_status().overrun {
                self.rx_overruns = self.rx_overruns.saturating_add(1);
            }
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
    /// Hand the peripheral as many bytes as its transmit register will take, blocking only for the
    /// FIRST one, and return how many went in. No byte is waited out on the wire: `TBE` (the data
    /// register is free) is the whole condition, so a caller that already saw
    /// [`write_ready`](WriteReady::write_ready) pays microseconds per byte instead of a character
    /// time (1.04 ms at 9600 baud, the BLE module's rate).
    ///
    /// The partial return is the `embedded-io` contract, not a deviation: an implementation may
    /// write fewer bytes than asked while staying ready, and must never answer `Ok(0)` for a
    /// non-empty buffer, which is why the first byte blocks on `TBE`. `write_all` loops on this and
    /// so still delivers whole buffers, at the wire's pace.
    ///
    /// **On return the last byte is in the peripheral, not yet out on the wire.** A caller that
    /// then reprograms the baud, splits/rejoins the port, or tears it down must
    /// [`flush`](Write::flush) first or truncate that byte mid-frame; every AT command in the `ble`
    /// bring-up and every frame in `link`'s serial transport does exactly that.
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        let Some((&first, rest)) = data.split_first() else {
            return Ok(0);
        };
        // At least one byte, per the trait: this is the only wait in the whole path, and a caller
        // that gated on `write_ready` never enters it.
        while !self.usart.try_write_byte(first) {}
        let mut n = 1;
        for &b in rest {
            if !self.usart.try_write_byte(b) {
                break;
            }
            n += 1;
        }
        Ok(n)
    }

    /// Wait for `TC`: the last byte written has fully shifted out and the line is idle. This is
    /// where the character-time wait lives now that [`write`](Write::write) does not pay it per
    /// byte, and it is what a caller owes before reconfiguring or tearing down the port.
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
    #[inline]
    fn rx_read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError> {
        self.read(buf)
    }
    #[inline]
    fn rx_ready(&self) -> bool {
        self.ready()
    }
    #[inline]
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
    lap_overruns: u16,
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
            lap_overruns: 0,
        }
    }

    /// Take the adapter apart (the reconfigure path: `into_parts` -> backend `release` ->
    /// [`Usart::rejoin`] -> `set_baud` -> `split` -> re-arm -> `new`).
    #[inline]
    pub fn into_parts(self) -> (UsartTx, R) {
        (self.tx, self.rx)
    }

    /// Diagnostic: **line-level disturbances** absorbed by `read` so far, saturating: a DMA-ring
    /// [`LineError`](crate::error::UsartError::LineError) (a USART `ERRIE` overrun / framing / noise
    /// glitch, i.e. a wire disturbance) or a `BufferedRx` framing / parity error. Each is absorbed once
    /// with the channel left LIVE and reads resume with no re-arm (the backend self-heals in place). A
    /// persistent bad line is left for the protocol layer to notice as `comms_loss`, not a disabled
    /// peripheral. Counted SEPARATELY from [`lap_overruns`](Self::lap_overruns) so a consumer can tell
    /// a wire glitch from a slow-consumer loss (they were conflated before). Outside the `embedded-io`
    /// traits by design.
    #[inline]
    pub fn line_errors(&self) -> u16 {
        self.line_errors
    }

    /// Diagnostic: **buffer-overrun losses** absorbed by `read` so far, saturating: a DMA-ring
    /// [`RingOverrun`](crate::error::UsartError::RingOverrun) (the circular DMA lapped the read cursor)
    /// or a `BufferedRx` ring-full [`Overrun`](crate::error::UsartError::Overrun). Both mean the
    /// consumer fell behind and bytes were dropped; each is absorbed with reception continuing (no
    /// re-arm). Counted SEPARATELY from [`line_errors`](Self::line_errors) (a wire disturbance) so
    /// "the consumer is too slow" is distinguishable from "the line glitched". Outside the
    /// `embedded-io` traits by design.
    #[inline]
    pub fn lap_overruns(&self) -> u16 {
        self.lap_overruns
    }
}

impl SplitSerial<RingBufferedRx> {
    /// **Validation hook** (DMA-ring backend only): inject one line error into the RX path exactly as
    /// the ERRIE ISR would record it, so the next [`read`](Read::read) surfaces it (counted into
    /// [`line_errors`](Self::line_errors)) and self-heals in place. Passthrough to
    /// [`RingBufferedRx::inject_line_error`]; see its docs. Concrete to the DMA-ring backend because
    /// that is the always-on framed inter-board UART link the firmware's Gate-1 self-heal sign-off
    /// drives over SWD (`BufferedRx` has no such exercised hook).
    #[inline]
    pub fn inject_line_error(&self) {
        self.rx.inject_line_error();
    }

    /// Model ONE character arriving from the wire (host `mock` only): passthrough to
    /// [`RingBufferedRx::inject_rx_byte`]. Deliberately the SAME name and shape as
    /// [`PolledSerial::inject_rx_byte`], so one caller-away property test can be written once and
    /// compiled against whichever carrier a port actually ships.
    #[cfg(feature = "mock")]
    #[inline]
    pub fn inject_rx_byte(&self, b: u8) {
        self.rx.inject_rx_byte(b);
    }
}

impl<R: RxBackend> ErrorType for SplitSerial<R> {
    type Error = Infallible;
}

impl<R: RxBackend> Read for SplitSerial<R> {
    /// Non-blocking: drain what the backend has; `Ok(0)` when nothing is available. The IDLE latch
    /// is consumed (dropped) here so it can never leak to a consumer; an absorbed condition ticks the
    /// matching counter (a wire disturbance -> [`line_errors`](Self::line_errors), a buffer-overrun
    /// loss -> [`lap_overruns`](Self::lap_overruns)) and the drain is retried, so a single `read` still
    /// returns post-recovery bytes where the backend has them.
    #[inline]
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        // The adapter owns the IDLE latch: consume it unconditionally (framing belongs to the
        // framer above; the latch must never sit as un-modeled state a consumer misreads).
        let _ = self.rx.rx_take_idle();

        // Each surfaced condition is consume-on-read (the backend clears it as it returns Err), so
        // this loop is finite in practice; bound it anyway so a misbehaving state cannot spin. Classify
        // by the returned variant so the two recoverable causes are counted apart (the OQ1 split): a
        // slow-consumer buffer-overrun loss vs a wire/line disturbance.
        for _ in 0..4 {
            match self.rx.rx_read(out) {
                Ok(n) => return Ok(n),
                // Buffer-overrun losses (the consumer fell behind): a DMA lap or a BufferedRx ring-full.
                Err(UsartError::RingOverrun) | Err(UsartError::Overrun) => {
                    self.lap_overruns = self.lap_overruns.saturating_add(1);
                }
                // Wire/line disturbances: a DMA LineError or a BufferedRx framing/parity error.
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
    #[inline]
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
