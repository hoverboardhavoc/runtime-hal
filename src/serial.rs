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
//! - **`Write` is polled, `WriteReady` = TBE**, and DMA TX is out of scope. BOTH adapters wait
//!   `TBE` only, take as many bytes as the register accepts and report how many, so a caller can
//!   meter a frame onto a slow wire without blocking (the 9600-baud BLE module: 1.04 ms a byte
//!   against a 4 ms control tick). `flush` is the real `TC` wait on both, and is what a caller
//!   owes before reconfiguring or tearing down the port. [`SplitSerial`] used to wait `TBE` then
//!   `TC` on every byte and call its `flush` a no-op; that was corrected on 2026-10-22 after the
//!   measurement, because this adapter carries the metered 9600-baud port as well as the
//!   inter-board one and the per-byte wait was costing the consumer's main loop a character time
//!   per metered byte (see [`SplitSerial`]'s `write`).
//! - **Every wait on the write path is BOUNDED**, by a budget derived from the core clock so the
//!   bound means something in microseconds ("the polled-write time budget" below). A main loop
//!   with a motor coast and a watchdog cannot contain an unbounded spin on a hardware flag; a
//!   timed-out write drops the frame and counts it (`tx_timeouts`), the same way the RX side
//!   absorbs and counts a line error.
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

// --- the polled-write time budget ---------------------------------------------------------------
//
// Every wait on the write path is bounded, and the bound is derived from the core clock so it
// means something in microseconds. This is the [`crate::i2c`] acquisition budget applied to the
// other polled peripheral, for the same reason and in the same shape; read that module's
// "polled-transfer time budget" for the long form of the argument.
//
// Why a bound at all. These adapters are what the consuming firmware's MAIN LOOP writes through,
// and that loop has two deadlines: the 16 ms motor coast (a commutation ISR floats all three
// phases if the 250 Hz control task, dispatched from the same pass, goes quiet) and a 500 ms
// watchdog. An unbounded `while !flag {}` respects neither. It is not a theoretical hazard: on
// 2026-10-22 the offroad master took eleven independent-watchdog resets in one bench session
// (`RCU_RSTSCK` bit 29 set on the master, clear on its peer), and the only waits in its loop with
// no bound were these. A wait whose failure mode is "spin until the watchdog reboots the board"
// is strictly worse than one whose failure mode is a dropped frame on a best-effort link.
//
// Why per WRITE CALL rather than per flag wait: the `i2c` argument transfers verbatim. A per-wait
// budget detects "no progress" but cannot bound a transfer, because a port that makes progress on
// every byte to just inside a per-wait limit runs the call arbitrarily long. One countdown shared
// by every wait in the call bounds the whole of it by construction.
//
// What a timed-out write DOES is the TX twin of how this module already handles RX line errors
// (see the module docs): the condition is absorbed below the `embedded-io` line, the rest of the
// buffer is dropped, a saturating diagnostic counter records it, and the call reports the buffer
// as consumed. Reporting it as consumed is deliberate and is not a convenience: `Ok(0)` on a
// non-empty buffer makes `embedded-io`'s `write_all` PANIC, and the consumer is a panic-free
// image, so the choice is between a counted drop and a panic. The callers already assume the
// drop: `link::SerialTransport::send_l2_frame` discards the result because L2 is best-effort and
// a truncated frame fails its receiver's CRC.

/// Core cycles one write-path poll iteration costs, as an UPPER bound.
///
/// The upper bound is the correct end to take, exactly as in [`crate::i2c::POLL_CYCLES`]:
/// over-stating it under-states the iteration count, so the real block finishes EARLY.
/// The loop is an APB load of `STAT`, one flag test, a load/store of the countdown and a taken
/// back-branch; it is shorter than the I2C's five-flag test, and 24 leaves room for the
/// AHB-to-APB bridge stall and the flash wait states on the fetch.
pub const WRITE_POLL_CYCLES: u32 = 24;

/// Wall-clock ceiling, in microseconds, on the polling ONE write call may do.
///
/// Sized between the same two pincers the I2C budget sits between. Below it has to clear the
/// legitimate waits: one character time is 1.04 ms at 9600 (the BLE module's rate, where a
/// metered caller writes one byte per call) and 21.7 us at 460800 (the inter-board link, where a
/// 24-byte frame is 24 of those), so 4 ms is ~3.8x the slowest single legitimate wait and ~7x a
/// whole inter-board frame. Above, it must stay well inside the 16 ms motor coast, and at 4 ms it
/// is a quarter of it.
///
/// A caller wanting to push a WHOLE frame at 9600 baud is asking for ~21 ms of wire time and will
/// time out here, which is correct: that call was already a coast violation before this bound
/// existed, and it is why the firmware meters that port a byte at a time instead.
pub const WRITE_BUDGET_US: u32 = 4_000;

/// Poll iterations that fit in [`WRITE_BUDGET_US`] at `core_hz`: the per-call write budget.
///
/// Saturates at 1 so a nonsensically slow `core_hz` still terminates.
pub const fn write_budget(core_hz: u32) -> u32 {
    let n = (core_hz / 1_000_000) * WRITE_BUDGET_US / WRITE_POLL_CYCLES;
    if n == 0 {
        1
    } else {
        n
    }
}

/// Worst-case microseconds one failing write call can block the caller. Const, so a consumer with
/// a real-time deadline can `const_assert` its own budget against it and fail the BUILD if either
/// moves ([`crate::i2c::worst_case_block_us`] is the precedent).
pub const fn worst_case_write_block_us(core_hz: u32) -> u32 {
    if core_hz == 0 {
        return WRITE_BUDGET_US;
    }
    let mhz = core_hz / 1_000_000;
    write_budget(core_hz) * WRITE_POLL_CYCLES / if mhz == 0 { 1 } else { mhz }
}

/// Spin on `cond` until it holds or `left` runs out. `true` = the flag came up, `false` = the
/// budget expired. The ONE place a write-path wait is written, so no wait can be added without a
/// bound.
#[inline]
fn wait_bounded(left: &mut u32, mut cond: impl FnMut() -> bool) -> bool {
    while !cond() {
        if *left == 0 {
            return false;
        }
        *left -= 1;
    }
    true
}

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
    /// Poll iterations ONE write call may spend ([`write_budget`] at the bring-up core clock),
    /// resolved once where the clock is known. See "the polled-write time budget" above.
    write_budget: u32,
    tx_timeouts: u16,
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
        Ok(Self::from_usart(
            Usart::new(chip, clock, instance, pins, baud)?,
            clock.sysclk_hz,
        ))
    }

    /// Wrap an already-configured [`Usart`] handle (the low-level path for code that brought the
    /// USART up itself, e.g. via [`Usart::bring_up`] or after a [`Usart::rejoin`]).
    #[inline]
    /// `core_hz` is the system clock the write budget is derived against, and it is a parameter
    /// rather than a constant because the bound is a WALL-CLOCK one: the same iteration count is
    /// 4 ms at 72 MHz and 36 ms at 8 MHz, and 36 ms is past the consumer's motor coast. See "the
    /// polled-write time budget" above.
    pub const fn from_usart(usart: Usart, core_hz: u32) -> Self {
        Self {
            usart,
            line_errors: 0,
            rx_overruns: 0,
            write_budget: write_budget(core_hz),
            tx_timeouts: 0,
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

    /// Diagnostic: write calls abandoned because a transmit flag did not come up inside the
    /// write budget, saturating. The TX twin of the absorbed RX conditions above: the frame is
    /// dropped (L2 is best-effort and a truncated frame fails its receiver's CRC), the caller is
    /// told the buffer was consumed, and this is where the loss is visible. Expected to read ZERO;
    /// a non-zero count means the port stopped accepting bytes, which on the consuming board used
    /// to be a watchdog reset instead of a number.
    #[inline]
    pub fn tx_timeouts(&self) -> u16 {
        self.tx_timeouts
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
        // that gated on `write_ready` never enters it. BOUNDED: see "the polled-write time
        // budget" above for why, and for why a timeout reports the buffer as consumed.
        let mut left = self.write_budget;
        if !wait_bounded(&mut left, || self.usart.try_write_byte(first)) {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
            return Ok(data.len());
        }
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
        let mut left = self.write_budget;
        if !wait_bounded(&mut left, || self.usart.read_status().tx_complete) {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
        }
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
    /// Poll iterations ONE write call may spend ([`write_budget`] at the bring-up core clock).
    write_budget: u32,
    tx_timeouts: u16,
}

impl<R: RxBackend> SplitSerial<R> {
    /// Adapt an already-armed receiver + the TX half. The adapter brings nothing up; construction
    /// is explicit from the slice-3 ownership pieces (`specs/serial-adapters.md` D5):
    /// `usart.split()`, arm the backend on the RX half, wrap both here.
    #[inline]
    /// `core_hz` is the system clock the write budget is derived against; see
    /// [`PolledSerial::from_usart`] for why it is a parameter and not a constant.
    pub fn new(tx: UsartTx, rx: R, core_hz: u32) -> Self {
        SplitSerial {
            tx,
            rx,
            line_errors: 0,
            lap_overruns: 0,
            write_budget: write_budget(core_hz),
            tx_timeouts: 0,
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
    pub fn tx_timeouts(&self) -> u16 {
        self.tx_timeouts
    }

    /// Diagnostic: **buffer-overrun losses** absorbed by `read` so far, saturating (see
    /// [`PolledSerial::tx_timeouts`] for the TX-side twin's contract).
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
    /// Hand the peripheral as many bytes as its transmit register will take, blocking only for
    /// the FIRST one, and return how many went in. Identical in shape to
    /// [`PolledSerial`]'s, because the two adapters face the same caller.
    ///
    /// **It used to wait `TBE` then `TC` on EVERY byte, and that was wrong on both of this
    /// adapter's ports** (corrected 2026-10-22, measured). `TC` says the byte has finished
    /// shifting out on the wire, which no caller on this path needs: `TBE` alone is the whole
    /// condition for not losing data, and the only thing a line going idle is required for is
    /// reconfiguring or tearing the port down, which is what [`flush`](Write::flush) is for and
    /// is now where that wait lives.
    ///
    /// What the per-byte `TC` wait cost, on the two ports this adapter actually carries:
    ///
    /// - The 9600-baud BLE module, where the consuming firmware meters ONE byte per main-loop
    ///   pass precisely so a frame does not block the pass. Each of those one-byte writes was
    ///   paying a full 1.04 ms character time in the spin, which is the whole cost the metering
    ///   exists to avoid, and 32 bytes of one telemetry sample was 33 ms of main loop spent
    ///   waiting on a wire nothing was waiting for.
    /// - The 460800 inter-board link at 125 frames a second, where serialising every byte against
    ///   `TC` gives up the data register's double buffering and spends ~0.5 ms of CPU per frame;
    ///   PC sampling the consuming board put ~10 percent of its whole CPU inside this function.
    ///
    /// The wire behaviour changes: bytes now go out back to back with no inter-byte gap, which is
    /// ordinary UART operation and is what both receivers (a DMA ring, and a CC2541 bridge that
    /// re-chunks the stream anyway) are built for.
    ///
    /// Bounded, and a timeout reports the buffer as consumed: see "the polled-write time budget".
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        let Some((&first, rest)) = data.split_first() else {
            return Ok(0);
        };
        let mut left = self.write_budget;
        if !wait_bounded(&mut left, || self.tx.try_write_byte(first)) {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
            return Ok(data.len());
        }
        let mut n = 1;
        for &b in rest {
            if !self.tx.try_write_byte(b) {
                break;
            }
            n += 1;
        }
        Ok(n)
    }

    /// Wait for `TC`: the last byte written has fully shifted out and the line is idle. The real
    /// wait now that [`write`](Write::write) does not pay it per byte, and what a caller owes
    /// before a baud change, a rejoin or a teardown. Bounded like every other wait here.
    fn flush(&mut self) -> Result<(), Self::Error> {
        let mut left = self.write_budget;
        if !wait_bounded(&mut left, || self.tx.tx_complete()) {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
        }
        Ok(())
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
