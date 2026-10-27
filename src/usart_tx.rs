//! One-shot DMA USART transmit: [`DmaTx`] (G-DMA-TX, `specs/uart-tx-dma.md`).
//!
//! The transmit twin of [`crate::usart_rx::RingBufferedRx`], and the first non-blocking transmit in
//! this HAL. Every writer in the fleet was a `try_write_byte` poll, which is adequate while the wire
//! time of a frame is small against the loop period and stops being adequate when it is not: the
//! inter-board link's 27 B frame at 460800 is 0.43 ms of CPU inside the write, 125 times a second.
//! Here the CPU copies the frame into an owned buffer, arms a channel, and leaves; the DMA feeds
//! `TDATA` byte by byte and raises one interrupt at the end.
//!
//! # Two ports
//!
//! Two USARTs can transmit on DMA, each with its own channel, completion vector and in-flight state,
//! so both can be live at once: the inter-board link (`USART1`), and the BLE module port (`USART2` on
//! F10x, `USART0` on its PB6/PB7 mapping on F1x0; the same per-family split the receive side
//! resolves). The module port is 9600 baud, where a 20 B frame is 21 ms of wire: polled, it can only
//! go out a byte per main-loop visit, so the loop's pass rate sets the port's byte rate. On DMA the
//! frame goes in one transfer and the pass rate stops mattering.
//!
//! # What it is NOT
//!
//! It queues nothing and drops nothing. A `try_send` while a transfer is in flight is REFUSED, and
//! that refusal is the entire backpressure surface: whether a frame should wait, be dropped, or
//! replace the one in flight is the caller's policy, and the callers already have one.
//!
//! # Completion means the last byte reached `TDATA`, not the wire
//!
//! The full-transfer flag fires when `CHxCNT` reaches zero, which means the last byte was written
//! INTO the transmit data register; one character time of shifting remains after that. Nothing on
//! this port needs the distinction for correctness, because it is full duplex with no direction pin
//! to turn around. A caller about to change the baud, rejoin the halves or tear the port down DOES
//! need it, and that wait is `TC`, which is where [`crate::serial::SplitSerial`]'s `flush` puts it.
//!
//! # The transfer that never completes
//!
//! A polled write cannot wedge a port: it either makes progress or its bounded wait expires. A DMA
//! transfer can, because the in-flight flag is cleared by an interrupt that a wedged channel, a lost
//! vector or a peer holding the line never produces. So the in-flight refusal carries a deadline:
//! the wire time of the frame at the configured baud with the same kind of slack the polled waits
//! take, measured in CPU cycles off the DWT counter. Past it, the next `try_send` abandons the
//! transfer (disable `CHEN`, clear the channel's flags), counts it, and takes the new frame. The
//! count is what makes a wedging port visible rather than merely survived.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::ClockPath;
use crate::dma::{self, DmaTxMap};
use crate::error::DescriptorError;
use crate::irq;
use crate::usart::UsartTx;

/// The refusal from [`DmaTx::try_send`]: a transfer is in flight and its deadline has not passed, so
/// the port took nothing. Not an error condition: it is the ordinary "not now" of a one-frame-deep
/// transmitter, and the caller's policy decides between waiting and dropping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxBusy;

/// Bit times one 8N1 character costs on the wire: a start bit, eight data bits and a stop bit. The
/// frame this HAL configures ([`crate::config::UsartFrame::EIGHT_N_ONE`]) is the only one any port
/// in the fleet uses, so the wire-time budget is computed from it rather than parametrised on a
/// frame nothing selects.
const BITS_PER_BYTE: u32 = 10;

/// The slack the transmit deadline takes over the bare wire time, matching what
/// [`crate::serial::WRITE_BUDGET_US`] takes over the longest legitimate polled wait (4 ms against a
/// 1.04 ms character time at 9600). It has to absorb a transfer that is real but slow: a peer
/// clock-stretching nothing, an interrupt storm delaying the completion vector, the character still
/// shifting when `CHxCNT` hit zero. It must stay small enough that a WEDGED port recovers inside a
/// handful of main-loop passes rather than a human-visible stall: at 460800 a 27 B frame's deadline
/// is 2.3 ms, well inside the 16 ms motor coast.
const BUDGET_SLACK: u32 = 4;

/// Half the counter's range, the ceiling a computed deadline is clamped to. The deadline is compared
/// as a wrapping difference against the free-running 32-bit `CYCCNT`, which is only meaningful while
/// the interval is less than one wrap; a budget approaching `u32::MAX` would be a deadline that
/// never reliably arrives. Nothing reaches it in practice (the largest frame this HAL's ports send,
/// at the slowest baud they use, is far below it), so the clamp is the arithmetic's guard rail, not
/// a policy.
const MAX_BUDGET_CYCLES: u32 = u32::MAX / 2;

/// CPU cycles one byte of wire time costs at `baud` on a `core_hz` core, with [`BUDGET_SLACK`]
/// applied. Saturating throughout: a nonsensical `baud` of 0 yields the clamp rather than a divide
/// by zero or a wrap, and `core_hz` is at most 72 MHz on this fleet so `core_hz * BITS_PER_BYTE`
/// cannot overflow a `u32` for any real clock.
const fn cycles_per_byte(core_hz: u32, baud: u32) -> u32 {
    if baud == 0 {
        return MAX_BUDGET_CYCLES;
    }
    let per_byte = (core_hz as u64) * (BITS_PER_BYTE as u64) / (baud as u64);
    let slacked = per_byte * (BUDGET_SLACK as u64);
    if slacked > MAX_BUDGET_CYCLES as u64 {
        MAX_BUDGET_CYCLES
    } else if slacked == 0 {
        // A clock so slow that a byte costs under one cycle: still give the deadline a cycle, so it
        // is a deadline and not an immediate expiry.
        1
    } else {
        slacked as u32
    }
}

/// The deadline, in CPU cycles, for a transfer of `len` bytes. Clamped as above.
const fn budget_cycles(per_byte: u32, len: usize) -> u32 {
    let total = (per_byte as u64) * (len as u64);
    if total > MAX_BUDGET_CYCLES as u64 {
        MAX_BUDGET_CYCLES
    } else if total == 0 {
        1
    } else {
        total as u32
    }
}

/// Which transmit context a handle resolves to, and so which channel, vector and in-flight flag it
/// owns. The index is the [`crate::dma`] context index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxPort {
    /// The inter-board link's `USART1`: context 0.
    Link,
    /// The BLE module port: `USART2` on F10x, `USART0` (PB6/PB7) on F1x0. Context 1, shared by the
    /// two instances because no part has both.
    Module,
}

impl TxPort {
    #[inline]
    const fn ctx(self) -> usize {
        match self {
            TxPort::Link => 0,
            TxPort::Module => 1,
        }
    }
}

/// Resolve a TX half's register base to the port it transmits for, and that port's channel map.
///
/// Keyed on the handle's base rather than on a selector the caller passes, so a channel can only be
/// armed for the USART whose `TDATA` it feeds: the instance is what the caller actually handed over.
/// The module instance follows the receive side's family rule (`usart_rx::resolve_instance`):
/// `USART2` is the module on F10x and does not exist on F1x0; `USART0` is the module on F1x0 and is
/// refused on F10x, where its PB6/PB7 mapping needs an AFIO remap this HAL does not implement. A
/// label the descriptor has no address for simply does not match. Anything else is
/// [`DescriptorError::Unsupported`].
fn resolve_port(chip: &Chip, base: u32) -> Result<(TxPort, DmaTxMap), DescriptorError> {
    let is = |label: PeriphLabel| chip.base(label).ok() == Some(base);
    if is(PeriphLabel::Usart1) {
        return Ok((TxPort::Link, DmaTxMap::usart1_tx(chip)));
    }
    match chip.clock() {
        ClockPath::F10xRcc if is(PeriphLabel::Usart2) => {
            Ok((TxPort::Module, DmaTxMap::module_tx()))
        }
        ClockPath::F1x0Rcu if is(PeriphLabel::Usart0) => {
            Ok((TxPort::Module, DmaTxMap::usart0_tx()))
        }
        _ => Err(DescriptorError::Unsupported),
    }
}

/// The DMA transmit IRQ number for `port` on `chip`'s family. The link: its own `DMA0_Channel6` = 17
/// on F10x, the grouped `DMA_Channel3_4` = 11 on F1x0. The module port: its own `DMA0_Channel1` = 12
/// on F10x, the grouped `DMA_Channel1_2` = 10 on F1x0. A grouped vector is shared with the port's
/// receive channel, where unmasking it a second time is idempotent. Hardware-build-only (drives the
/// NVIC).
#[cfg(not(feature = "mock"))]
#[inline]
fn tx_irq_num(chip: &Chip, port: TxPort) -> usize {
    match port {
        TxPort::Link => irq::dma_tx_irq(chip.irq()),
        TxPort::Module => irq::module_tx_irq(chip.irq()),
    }
}

/// One-shot DMA USART transmitter for the inter-board link's port or the BLE module port (G-DMA-TX).
///
/// Built from the TX half of a split [`crate::usart::Usart`], consuming it the way
/// [`crate::usart_rx::RingBufferedRx`] consumes the RX half: the transmit path now owns that
/// instance's `TDATA` and its DMA channel, and [`release`](DmaTx::release) gives the half back
/// polled. The buffer's length is the caller's largest frame.
///
/// The `'static` buffer is OWNED rather than borrowed per call, because a DMA transfer reads its
/// source for as long as it runs and a frame handed in by reference does not live that long. The
/// owned copy plus the in-flight refusal are together what makes [`try_send`](DmaTx::try_send) safe
/// from safe code: there is no window in which the caller can touch the bytes the channel is reading.
///
/// **Not generic over the buffer length**, for the same reason [`crate::usart_rx::RingBufferedRx`]
/// is not: the type is named by [`crate::serial::SplitSerial`]'s transmit half, which every port
/// shares, and a const generic there would split that type (and with it the whole generic stack
/// above it) once per buffer size. The length is a runtime field, checked once at bring-up.
pub struct DmaTx {
    map: DmaTxMap,
    /// The [`crate::dma`] transmit context this port owns ([`TxPort::ctx`]): its in-flight flag and
    /// its completion body.
    ctx: u8,
    /// The owned source buffer. `&'static mut` rather than an inline array so the application places
    /// it (DECISIONS.md #10: buffers are application code) and so moving the handle never moves the
    /// memory a running channel is reading.
    buf: &'static mut [u8],
    /// The consumed TX half, held for `TC` waits and for [`release`](DmaTx::release).
    tx: UsartTx,
    /// The `TDATA` address resolved once from the register model (it differs by family).
    paddr: u32,
    /// CPU cycles of deadline per byte at the configured baud ([`cycles_per_byte`]).
    per_byte: u32,
    /// `CYCCNT` when the in-flight transfer was armed, and its deadline in cycles. Meaningful only
    /// while the in-flight flag is set.
    started: u32,
    budget: u32,
    /// Transfers abandoned by the force-restart path, saturating. Surfaced through
    /// [`timeouts`](DmaTx::timeouts) and summed into [`crate::serial::SplitSerial::tx_timeouts`].
    timeouts: u16,
}

impl DmaTx {
    /// Arm DMA transmit on the TX half of a brought-up USART, sending out of `buf`
    /// (`specs/uart-tx-dma.md` section 3.2). In order: resolve [`DmaTxMap`]; enable the DMA clock;
    /// stop any channel left running; **write-back self-check** (fail loud, arming nothing); set
    /// `DENT`; install the completion context + ISR body; unmask the vector. The same bring-up order
    /// and the same fail-loud posture as [`crate::usart_rx::RingBufferedRx::new`], and like it the
    /// caller must have flipped `VTOR` to the RAM table first, since this is what lets the vector
    /// fire.
    ///
    /// No channel is programmed here: a transmitter has nothing to send until it is asked, so
    /// `CHEN` stays clear until the first [`try_send`](DmaTx::try_send).
    ///
    /// `core_hz` and `baud` are what the force-restart deadline is computed from (the wire time of a
    /// frame in CPU cycles); `core_hz` is a parameter for the same reason
    /// [`crate::serial::PolledSerial::from_usart`]'s is.
    ///
    /// **`tx` must be the half of a USART this HAL maps a transmit channel for**: `USART1` (the link)
    /// on either family, `USART2` (the module port) on F10x, `USART0` (the module port) on F1x0. Any
    /// other handle is refused with [`DescriptorError::Unsupported`] rather than armed onto a
    /// channel that feeds a different peripheral's `TDATA`. A buffer that is empty or longer than
    /// `CHxCNT`'s 16-bit range is [`DescriptorError::Unsupported`] too, and a failed self-check is
    /// [`DescriptorError::SelfCheckFailed`].
    #[cfg_attr(runtime_hal_nightly, optimize(size))]
    pub fn new(
        chip: &Chip,
        tx: UsartTx,
        buf: &'static mut [u8],
        core_hz: u32,
        baud: u32,
    ) -> Result<DmaTx, DescriptorError> {
        // A zero-length buffer is a transmitter that can never take a byte, and a length past
        // `CHxCNT`'s 16 bits cannot be programmed. Checked here rather than by a const assert,
        // because the length is runtime data now (see the type's docs).
        if buf.is_empty() || buf.len() > u16::MAX as usize {
            return Err(DescriptorError::Unsupported);
        }

        // A channel's request line belongs to one USART, so a handle armed on another instance's
        // channel would either never advance or feed the wrong port. Resolve the channel from the
        // handle the caller actually passed (the `resolve_instance` posture on the RX side).
        let (port, map) = resolve_port(chip, tx.regs().base())?;

        // The DMA controller clock (shared with the receive ring, which may already have enabled it;
        // the write is idempotent).
        crate::clock::enable_dma(chip.rcu_base()?);

        // Stop anything the channel was doing before the self-check pokes `CHxMADDR`: a live channel
        // must not have its source address briefly pointed at the sentinel.
        map.disable();

        // Write-back self-check (fail-loud): confirm the resolved channel responds before arming
        // anything. As on the receive side it proves the channel RESPONDS, not that it is the one
        // this USART's transmitter feeds; only the bench proves the mapping.
        if !map.self_check() {
            return Err(DescriptorError::SelfCheckFailed);
        }

        // Route the transmitter's byte fetches to the DMA controller.
        tx.regs().enable_dma_tx();

        // Install the completion context + ISR body, then let the vector fire. The deadline's clock
        // is the DWT cycle counter, so arming it is part of bring-up rather than an obligation left
        // to the application: a counter that was never started reads a constant, and a constant
        // clock is a deadline that never arrives. It ZEROES the counter, which is why it belongs to
        // bring-up and nowhere else: a consumer timing a window across this call would see its
        // start stamp move.
        dma::install_tx(&map, port.ctx());
        irq::enable_cycle_counter();
        #[cfg(not(feature = "mock"))]
        nvic_unmask(tx_irq_num(chip, port));

        Ok(DmaTx {
            map,
            ctx: port.ctx() as u8,
            buf,
            paddr: tx.regs().tdata_addr(),
            tx,
            per_byte: cycles_per_byte(core_hz, baud),
            started: 0,
            budget: 0,
            timeouts: 0,
        })
    }

    /// Hand a frame to the channel: copy as much of it as the owned buffer holds, program the
    /// channel and start it, returning how many bytes were taken. A frame longer than the buffer is
    /// TRUNCATED rather than refused (the caller sized the buffer to its largest frame, so an
    /// over-long one is a caller bug the return value reports; failing the whole write would lose
    /// the part that fits as well). An empty frame starts no transfer and reports zero.
    ///
    /// While a transfer is in flight it refuses with [`TxBusy`] and touches neither the buffer nor
    /// the channel, UNTIL that transfer's deadline has passed; then it force-restarts (disable
    /// `CHEN`, clear the channel's flags, count a timeout) and takes the new frame. See the module
    /// docs for why the deadline exists and how it is sized.
    pub fn try_send(&mut self, frame: &[u8]) -> Result<usize, TxBusy> {
        if frame.is_empty() {
            return Ok(0);
        }
        if dma::tx_in_flight(self.ctx()) {
            if !self.expired() {
                return Err(TxBusy);
            }
            // The deadline passed: abandon the transfer. Order matters: stop the channel before
            // clearing its flags, so a completion racing the restart cannot leave a flag set behind
            // the clear, and clear the in-flight claim last, as the ISR does.
            self.map.disable();
            self.map.clear_flags();
            dma::clear_tx_in_flight(self.ctx());
            self.timeouts = self.timeouts.saturating_add(1);
        }

        let n = frame.len().min(self.buf.len());
        self.buf[..n].copy_from_slice(&frame[..n]);
        self.budget = budget_cycles(self.per_byte, n);
        self.started = irq::cycle_count();
        // Claim BEFORE `CHEN` (inside `configure_oneshot`): a completion landing between the start
        // and this store would clear a flag that was not yet set, stranding the port.
        dma::mark_tx_in_flight(self.ctx());
        // SAFETY: `self.buf` is the caller's `'static` buffer, owned by this handle for as long as
        // it lives, and the in-flight refusal is what keeps the CPU out of the first `n` bytes until
        // the channel reports it is done with them.
        unsafe {
            self.map
                .configure_oneshot(self.paddr, self.buf.as_ptr() as u32, n as u16)
        };
        Ok(n)
    }

    /// True while a programmed transfer has not reported its completion. The whole observable state
    /// of the transmitter: a caller that gets [`TxBusy`] can poll this instead of retrying blindly.
    #[inline]
    pub fn in_flight(&self) -> bool {
        dma::tx_in_flight(self.ctx())
    }

    /// True if a [`try_send`](DmaTx::try_send) now would take a frame: the channel is idle, OR the
    /// transfer in flight is past its deadline, so the send would abandon it, count it and go.
    ///
    /// This, not `!in_flight()`, is the readiness a caller that only writes when ready must gate
    /// on. The deadline is checked inside `try_send`, so a caller that waits for the in-flight flag
    /// to clear before calling it never reaches the deadline: a completion that never arrives (a
    /// wedged channel, a lost vector) then silences the port for good, and silently, because the
    /// count that would have reported it is never taken. Reporting ready at the deadline is what
    /// lets that caller recover the port and see the fault in [`timeouts`](DmaTx::timeouts).
    #[inline]
    pub fn ready(&self) -> bool {
        !self.in_flight() || self.expired()
    }

    /// The transfer in flight has outlived its deadline. Meaningful only while one is in flight.
    #[inline]
    fn expired(&self) -> bool {
        // Wrapping difference: `CYCCNT` is free-running and the deadline is far below one wrap.
        irq::cycle_count().wrapping_sub(self.started) >= self.budget
    }

    /// Transfers the force-restart path abandoned, saturating: a wedged channel, a completion
    /// interrupt that never arrived, a peer that stopped clocking. A non-zero count on a working
    /// port means a real transmit fault, not a slow consumer.
    #[inline]
    pub fn timeouts(&self) -> u16 {
        self.timeouts
    }

    /// `TC`: the last byte has fully shifted out and the line is idle. Passthrough to the owned TX
    /// half, because what "the port is quiet" means does not change with how the bytes got there.
    #[inline]
    pub fn tx_complete(&self) -> bool {
        self.tx.tx_complete()
    }

    /// Quiesce the transmitter and give the TX half back (`specs/usart-split.md` D3): stop the
    /// channel, clear `DENT`, and uninstall the completion context so the vector becomes inert. The
    /// returned [`UsartTx`] is polled again, and rejoins its RX half
    /// ([`crate::usart::Usart::rejoin`]) for a reconfiguration.
    ///
    /// A transfer in flight is ABANDONED, not waited for: a caller tearing the port down has already
    /// waited for what it cared about (`flush`), and a `release` that could block on a wedged
    /// channel would reintroduce the unbounded wait this whole path exists to remove.
    pub fn release(self) -> UsartTx {
        self.map.disable();
        self.map.clear_flags();
        self.tx.regs().disable_dma_tx();
        dma::uninstall_tx(self.ctx());
        self.tx
    }

    /// This port's [`crate::dma`] transmit context index.
    #[inline]
    fn ctx(&self) -> usize {
        self.ctx as usize
    }
}

// --- NVIC interrupt number (hardware build only) ----------------------------------------------

/// Unmask the transmit vector through the crate's single owner of NVIC set-enable writes
/// ([`irq::unmask_irq`]). Hardware only: there is no NVIC under `cargo test`, where the host suite
/// fires the ISR via `mock_vtor::dispatch` instead.
#[cfg(not(feature = "mock"))]
#[inline]
fn nvic_unmask(irq_num: usize) {
    irq::unmask_irq(irq_num);
}

#[cfg(test)]
mod tests;
