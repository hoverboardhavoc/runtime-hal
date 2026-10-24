//! One-shot DMA USART transmit: [`DmaTx`] (G-DMA-TX, `specs/uart-tx-dma.md`).
//!
//! The transmit twin of [`crate::usart_rx::RingBufferedRx`], and the first non-blocking transmit in
//! this HAL. Every writer in the fleet was a `try_write_byte` poll, which is adequate while the wire
//! time of a frame is small against the loop period and stops being adequate when it is not: the
//! inter-board link's 27 B frame at 460800 is 0.43 ms of CPU inside the write, 125 times a second.
//! Here the CPU copies the frame into an owned buffer, arms a channel, and leaves; the DMA feeds
//! `TDATA` byte by byte and raises one interrupt at the end.
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

use crate::chip::Chip;
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

/// The DMA transmit IRQ number for `chip`'s family: its own `DMA0_Channel6` = 17 on F10x, the
/// grouped `DMA_Channel3_4` = 11 on F1x0 (shared with the link's receive channel, where unmasking it
/// a second time is idempotent). Hardware-build-only (drives the NVIC).
#[cfg(not(feature = "mock"))]
#[inline]
fn tx_irq_num(chip: &Chip) -> usize {
    irq::dma_tx_irq(chip.irq())
}

/// One-shot DMA USART transmitter for the inter-board link's port (G-DMA-TX).
///
/// Built from the TX half of a split [`crate::usart::Usart`], consuming it the way
/// [`crate::usart_rx::RingBufferedRx`] consumes the RX half: the transmit path now owns that
/// instance's `TDATA` and its DMA channel, and [`release`](DmaTx::release) gives the half back
/// polled. `N` is the caller's largest frame.
///
/// The `'static` buffer is OWNED rather than borrowed per call, because a DMA transfer reads its
/// source for as long as it runs and a frame handed in by reference does not live that long. The
/// owned copy plus the in-flight refusal are together what makes [`try_send`](DmaTx::try_send) safe
/// from safe code: there is no window in which the caller can touch the bytes the channel is reading.
pub struct DmaTx<const N: usize> {
    map: DmaTxMap,
    /// The owned source buffer. `&'static mut` rather than an inline array so the application places
    /// it (DECISIONS.md #10: buffers are application code) and so moving the handle never moves the
    /// memory a running channel is reading.
    buf: &'static mut [u8; N],
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

impl<const N: usize> DmaTx<N> {
    /// Compile-time rejection of a degenerate buffer: `N = 0` is a transmitter that can never take a
    /// byte, and `N` above `CHxCNT`'s 16-bit range cannot be programmed. Referenced from
    /// [`DmaTx::new`], so a bad `N` fails the BUILD of the instantiating crate (the
    /// [`crate::usart_rx::RxRing`] precedent); no runtime test is possible or needed.
    const VALID: () = assert!(
        N >= 1 && N <= u16::MAX as usize,
        "DmaTx needs 1 <= N <= 65535 (N is the largest frame, and CHxCNT is 16-bit)"
    );

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
    /// **`tx` must be USART1's half**, the only transmit channel this HAL maps (section 3.1): any
    /// other handle is refused with [`DescriptorError::Unsupported`] rather than armed onto a
    /// channel that feeds a different peripheral's `TDATA`. A failed self-check is
    /// [`DescriptorError::SelfCheckFailed`].
    pub fn new(
        chip: &Chip,
        tx: UsartTx,
        buf: &'static mut [u8; N],
        core_hz: u32,
        baud: u32,
    ) -> Result<DmaTx<N>, DescriptorError> {
        // Force the compile-time N check at every instantiation site.
        #[allow(clippy::let_unit_value)]
        let () = Self::VALID;

        // The map is `USART1_TX` and nothing else, so a handle on another instance would arm a
        // channel whose peripheral request line belongs to a different USART: the transfer would
        // either never advance or feed the wrong port. Check the handle against the descriptor's
        // USART1 base rather than trusting the caller (the `resolve_instance` posture on the RX
        // side, expressed against the handle the caller actually passed).
        if tx.regs().base() != chip.base(crate::addr::PeriphLabel::Usart1)? {
            return Err(DescriptorError::Unsupported);
        }

        let map = DmaTxMap::usart1_tx(chip);

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
        dma::install_tx(&map);
        irq::enable_cycle_counter();
        #[cfg(not(feature = "mock"))]
        nvic_unmask(tx_irq_num(chip));

        Ok(DmaTx {
            map,
            buf,
            paddr: tx.regs().tdata_addr(),
            tx,
            per_byte: cycles_per_byte(core_hz, baud),
            started: 0,
            budget: 0,
            timeouts: 0,
        })
    }

    /// Hand a frame to the channel: copy `min(frame.len(), N)` bytes into the owned buffer, program
    /// the channel and start it, returning how many bytes were taken. A frame longer than `N` is
    /// TRUNCATED rather than refused (the caller sized `N` to its largest frame, so an over-long one
    /// is a caller bug the return value reports; failing the whole write would lose the part that
    /// fits as well). An empty frame starts no transfer and reports zero.
    ///
    /// While a transfer is in flight it refuses with [`TxBusy`] and touches neither the buffer nor
    /// the channel, UNTIL that transfer's deadline has passed; then it force-restarts (disable
    /// `CHEN`, clear the channel's flags, count a timeout) and takes the new frame. See the module
    /// docs for why the deadline exists and how it is sized.
    pub fn try_send(&mut self, frame: &[u8]) -> Result<usize, TxBusy> {
        if frame.is_empty() {
            return Ok(0);
        }
        if dma::tx_in_flight() {
            // Wrapping difference: `CYCCNT` is free-running and the deadline is far below one wrap.
            let elapsed = irq::cycle_count().wrapping_sub(self.started);
            if elapsed < self.budget {
                return Err(TxBusy);
            }
            // The deadline passed: abandon the transfer. Order matters: stop the channel before
            // clearing its flags, so a completion racing the restart cannot leave a flag set behind
            // the clear, and clear the in-flight claim last, as the ISR does.
            self.map.disable();
            self.map.clear_flags();
            dma::clear_tx_in_flight();
            self.timeouts = self.timeouts.saturating_add(1);
        }

        let n = frame.len().min(N);
        self.buf[..n].copy_from_slice(&frame[..n]);
        self.budget = budget_cycles(self.per_byte, n);
        self.started = irq::cycle_count();
        // Claim BEFORE `CHEN` (inside `configure_oneshot`): a completion landing between the start
        // and this store would clear a flag that was not yet set, stranding the port.
        dma::mark_tx_in_flight();
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
        dma::tx_in_flight()
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
        dma::uninstall_tx();
        self.tx
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
