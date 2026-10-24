//! DMA channel driver for the USART ports: the circular RX (G-DMA-UART Gate B) and the one-shot TX
//! (G-DMA-TX, `specs/uart-tx-dma.md`).
//!
//! Only what the two USART users need: resolve which DMA controller / channel / IRQ grouping carries
//! `USART1_RX` ([`DmaRxMap`]) and `USART1_TX` ([`DmaTxMap`]) for the detected family, program the
//! channel (circular periph->mem for receive, one-shot mem->periph for transmit), read the live
//! remaining-count `CHxCNT`, and the two tiny ISR contexts: the receive one counts buffer wraps (for
//! lap-overrun detection), the transmit one clears the in-flight flag on a completion. Each services
//! ONLY its own channel's `INTF` bits, which is what lets the F1x0 grouped Ch3/4 vector carry both
//! (the DMA twin of the grouped advanced-timer demux).
//!
//! # The F1x0 grouped vector carries BOTH halves of the link
//!
//! On F1x0 the link's transmit (Ch3) and receive (Ch4) sit on one vector, `DMA_Channel3_4_IRQn` = 11.
//! Its table slot ([`crate::irq`]'s `dma_ch3_4_isr`) calls BOTH registered bodies on every entry and
//! each reads `INTF` for its own channel, so an entry with both channels flagged services both. A
//! demux that returned after the first match would drop the other channel's completion, and on the
//! transmit side a dropped completion is an in-flight flag that never clears: a port that never
//! transmits again until the force-restart budget expires.
//!
//! # Register layout (confirmed against the GD SPL `dma_*` + manuals)
//!
//! STM32F1-style on both families (`gd32f10x_dma.h` / `gd32f1x0_dma.h`): the controller is `DMA0` at
//! `0x4002_0000` on both (`gd32f10x.h`: AHB1 `0x4001_8000 + 0x8000`; `gd32f1x0.h`: AHB1
//! `0x4002_0000 + 0`). `INTF` at `0x00`, `INTC` at `0x04`; per-channel registers at a `0x14` stride
//! from channel 0: `CHCTL` `0x08`, `CHCNT` `0x0C`, `CHPADDR` `0x10`, `CHMADDR` `0x14`. `INTF`/`INTC`
//! carry 4 bits per channel (`flag << 4*channel`: GIF/FTFIF/HTFIF/ERRIF at +0/+1/+2/+3). `CHxCTL`
//! bits: `CHEN` 0, `FTFIE` 1, `HTFIE` 2, `ERRIE` 3, `DIR` 4 (0 = periph->mem), `CMEN` 5 (circular),
//! `PNAGA` 6 (peripheral increment), `MNAGA` 7 (memory increment), `PWIDTH` 8-9, `MWIDTH` 10-11
//! (0 = 8-bit). The DMA clock is `RCU_AHBEN` bit 0 on both families ([`crate::clock::enable_dma`]).

use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU32, AtomicU8, Ordering};

use crate::chip::Chip;
use crate::descriptor::IrqLayout;
use crate::irq;
use crate::reg::Reg32;

/// The single DMA controller (`DMA0`) base, identical on both families.
pub const DMA0_BASE: u32 = 0x4002_0000;

// Global register offsets.
const DMA_INTF: u32 = 0x00;
const DMA_INTC: u32 = 0x04;

// Per-channel registers: base offset + 0x14 * channel.
const CH_STRIDE: u32 = 0x14;
const CHCTL_OFF: u32 = 0x08;
const CHCNT_OFF: u32 = 0x0C;
const CHPADDR_OFF: u32 = 0x10;
const CHMADDR_OFF: u32 = 0x14;

// CHxCTL bit positions (identical on both families).
const CHCTL_CHEN: u32 = 1 << 0;
const CHCTL_FTFIE: u32 = 1 << 1;
const CHCTL_HTFIE: u32 = 1 << 2;
/// Transfer direction, `CHxCTL` bit 4: 0 = periph->mem (the receive path, which therefore never
/// writes it), 1 = mem->periph (the transmit path's `configure_oneshot`).
const CHCTL_DIR: u32 = 1 << 4;
const CHCTL_CMEN: u32 = 1 << 5;
const CHCTL_MNAGA: u32 = 1 << 7;
// DIR = 0 (periph->mem), PNAGA = 0 (peripheral address fixed = RDATA), PWIDTH/MWIDTH = 0 (8-bit):
// all left clear, so the programmed CHxCTL value is exactly the bits set below.

/// The sentinel for the write-back self-check (section 5.1 step 3 / section 2.5): an arbitrary
/// non-zero value written to `CHxMADDR` and read back to confirm the resolved channel responds.
const SELF_CHECK_SENTINEL: u32 = 0xA5A5_5A5A;

// --- the per-channel register window, direction-agnostic ---------------------------------------
//
// The channel block and the ops over it (arm/disarm, the write-back probe, the flag clear) are the
// same registers whichever way the data flows, so they are written ONCE here as free functions over
// `(base, channel)` and the two direction-specific maps ([`DmaRxMap`] / [`DmaTxMap`]) delegate to
// them. Direction lives only where it differs: the `CHxCTL` value each map programs.

#[inline]
fn chctl(base: u32, ch: u32) -> Reg32 {
    Reg32::new(base, CHCTL_OFF + CH_STRIDE * ch)
}
#[inline]
fn chcnt(base: u32, ch: u32) -> Reg32 {
    Reg32::new(base, CHCNT_OFF + CH_STRIDE * ch)
}
#[inline]
fn chpaddr(base: u32, ch: u32) -> Reg32 {
    Reg32::new(base, CHPADDR_OFF + CH_STRIDE * ch)
}
#[inline]
fn chmaddr(base: u32, ch: u32) -> Reg32 {
    Reg32::new(base, CHMADDR_OFF + CH_STRIDE * ch)
}

/// Stop the channel (clear `CHEN`). The one disarm, shared by both directions.
#[inline]
fn channel_disable(base: u32, ch: u32) {
    chctl(base, ch).modify(CHCTL_CHEN, 0);
}

/// Write-back self-check: write a sentinel to `CHxMADDR` and read it back. `true` if it sticks (the
/// DMA clock is on and the resolved base/channel is real); the one reliable DMA probe on this
/// silicon. It confirms the channel RESPONDS, not that the mapping is correct (that is family
/// detection's job). The caller's real `CHxMADDR` is written when the channel is programmed, so the
/// sentinel is overwritten rather than left behind.
#[inline]
fn channel_self_check(base: u32, ch: u32) -> bool {
    chmaddr(base, ch).write(SELF_CHECK_SENTINEL);
    chmaddr(base, ch).read() == SELF_CHECK_SENTINEL
}

/// Clear every one of this channel's `INTF` flags (GIF / FTFIF / HTFIF / ERRIF) by writing them to
/// `INTC`, touching no other channel's nibble. The recovery path's flag reset (a force-restarted
/// transmit): the stale completion of the abandoned transfer must not fire the ISR into the new one.
#[inline]
fn clear_channel_flags(base: u32, ch: u32) {
    Reg32::new(base, DMA_INTC).write(0xF << (4 * ch));
}

/// The chip-resolved DMA mapping for `USART1_RX`: controller base, channel index, and whether the
/// channel's IRQ is grouped (F1x0) or separate (F10x). The DMA analogue of `UsartModel`/`ClockPath`,
/// resolved from the detected family at bring-up (section 2.4 / 2.5), never probed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaRxMap {
    /// DMA controller base ([`DMA0_BASE`] on both families).
    pub base: u32,
    /// The `USART1_RX` channel index: **5 on F10x (F103), 4 on F1x0 (F130)** (section 2.4).
    pub channel: u8,
    /// True if the channel's DMA IRQ is the grouped `DMA_Channel3_4` vector (F1x0); false if it has
    /// its own `DMA0_Channel5` vector (F10x).
    pub grouped: bool,
}

impl DmaRxMap {
    /// Resolve the `USART1_RX` mapping for `chip` from the detected family (the [`IrqLayout`] selector
    /// carries the separate-vs-grouped fact; the channel index follows the same family split).
    #[inline]
    pub fn usart1_rx(chip: &Chip) -> DmaRxMap {
        match chip.irq() {
            // F10x: USART1_RX on DMA0 channel 5, its own vector.
            IrqLayout::F10xSeparate => DmaRxMap {
                base: DMA0_BASE,
                channel: 5,
                grouped: false,
            },
            // F1x0: USART1_RX on the single DMA channel 4, grouped Ch3/4 vector.
            IrqLayout::F1x0Grouped => DmaRxMap {
                base: DMA0_BASE,
                channel: 4,
                grouped: true,
            },
        }
    }

    /// Resolve the BLE-module USART's RX mapping (HAL `Usart2` = GD `USART2`, the F10x second instance).
    /// **F10x-only**: the GD32F10x User Manual §9.4.9 (Figure 9-4 / Table 9-3, the "USART" row) maps GD
    /// `USART2_RX` to **DMA0 Channel 2**, a separate per-channel vector (`DMA0_Channel2_IRQn` = 13). The
    /// caller ([`crate::usart_rx::RingBufferedRx::new`]) only reaches this after resolving the instance
    /// to the module, which is rejected on F1x0, so this is unconditionally the F10x Ch2 mapping (the
    /// module USART has no F1x0 silicon to validate a channel against).
    #[inline]
    pub fn module_rx() -> DmaRxMap {
        DmaRxMap {
            base: DMA0_BASE,
            channel: 2,
            grouped: false,
        }
    }

    /// Resolve USART0's RX mapping (the PB6/PB7 mapping, `pincap::usart_pins`). **F1x0-only**: the
    /// GD32F1x0 User Manual Rev3.6 `DMA_RMP` register's `USART0_RX_DMA_RMP` field (bit 10) maps
    /// `USART0_RX` to **DMA channel 2** when not remapped, which is the reset state this HAL never
    /// changes (remapping would move it to channel 4, colliding with `USART1_RX`). Channel 2 shares
    /// the grouped `DMA_Channel1_2_IRQn` vector, so `grouped` is true and the handler demuxes Ch1
    /// out by reading `INTF`. The caller ([`crate::usart_rx::RingBufferedRx::new`]) only reaches
    /// this after resolving the instance to USART0, which is rejected on F10x, so this is
    /// unconditionally the F1x0 Ch2 mapping.
    ///
    /// Independently corroborated by the offroad stock image, whose DMA CH2 carries USART0's RX
    /// (`hoverboard-firmware/specs/offroad-pinmap.md` section 4.3).
    #[inline]
    pub fn usart0_rx() -> DmaRxMap {
        DmaRxMap {
            base: DMA0_BASE,
            channel: 2,
            grouped: true,
        }
    }

    #[inline]
    fn ch(&self) -> u32 {
        self.channel as u32
    }
    #[inline]
    fn chctl(&self) -> Reg32 {
        chctl(self.base, self.ch())
    }
    #[inline]
    fn chcnt(&self) -> Reg32 {
        chcnt(self.base, self.ch())
    }
    #[inline]
    fn chpaddr(&self) -> Reg32 {
        chpaddr(self.base, self.ch())
    }
    #[inline]
    fn chmaddr(&self) -> Reg32 {
        chmaddr(self.base, self.ch())
    }

    /// The live remaining-transfer count `CHxCNT` (counts DOWN as the DMA writes). The DMA-ring read
    /// path derives the write index as `len - remaining()`. Read through `volatile` (it changes under
    /// hardware); the low 16 bits are the count.
    #[inline]
    pub fn remaining(&self) -> u16 {
        (self.chcnt().read() & 0xFFFF) as u16
    }

    /// Write-back self-check (section 5.1 step 3 / section 2.5): see [`channel_self_check`], the
    /// direction-agnostic implementation both maps share. The real `CHxMADDR` is written in
    /// [`Self::configure_circular`].
    #[inline]
    pub fn self_check(&self) -> bool {
        channel_self_check(self.base, self.ch())
    }

    /// Program the channel for circular periph->mem byte reception and start it (section 5.1 steps 4-5).
    /// `paddr` = `USART_RDATA` address, `maddr` = the `'static` buffer pointer, `len` = its length.
    /// Direction periph->mem (`DIR=0`), peripheral address fixed (`PNAGA=0`), memory-increment on
    /// (`MNAGA=1`), 8-bit both sides (widths 0), circular (`CMEN`), half + full-transfer interrupts
    /// enabled. `CHEN` is set LAST, after a `compiler_fence(Release)` so the channel setup is not
    /// reordered after the DMA starts (section 6).
    ///
    /// # Safety
    /// `maddr` must point at a `'static` buffer of at least `len` bytes that lives for as long as the
    /// channel runs; the DMA writes it concurrently (the `RingBufferedRx` ownership contract).
    pub unsafe fn configure_circular(&self, paddr: u32, maddr: u32, len: u16) {
        // Disable before reconfiguring (a running channel ignores config writes).
        self.chctl().modify(CHCTL_CHEN, 0);
        self.chpaddr().write(paddr);
        self.chmaddr().write(maddr);
        self.chcnt().write(len as u32);
        // Everything except CHEN; CHCTL reset value is 0, so a plain write sets exactly these bits.
        self.chctl()
            .write(CHCTL_MNAGA | CHCTL_CMEN | CHCTL_FTFIE | CHCTL_HTFIE);
        // Section 6: buffer/channel setup must not be reordered after the DMA starts.
        compiler_fence(Ordering::Release);
        self.chctl().modify(CHCTL_CHEN, CHCTL_CHEN);
    }

    /// Stop the channel (clear `CHEN`): background reception stops until a fresh `configure_circular`.
    /// The DMA-ring fail-loud path calls this when surfacing a line error (section 5.4).
    #[inline]
    pub fn disable(&self) {
        channel_disable(self.base, self.ch());
    }

    /// True if the channel's full-transfer (wrap) flag is currently set in DMA `INTF` (`FTFIF`, bit
    /// `4*channel + 1`): hardware sets it when `CHxCNT` reloads at a circular wrap, and the wrap-counter
    /// ISR clears it. So a set flag here means a wrap the counter has NOT yet counted. The read-position
    /// snapshot reads this together with `CHxCNT` so a just-reloaded `CHxCNT == len` is attributed to
    /// the pending wrap (section 5.2), never undercounting the write index into a spurious overrun.
    #[inline]
    pub fn ftf_pending(&self) -> bool {
        let intf = Reg32::new(self.base, DMA_INTF).read();
        intf & (1 << (4 * self.ch() + 1)) != 0
    }

    /// Advance the channel by ONE transfer, as the DMA engine does when a character arrives (host
    /// `mock` only): `CHxCNT` counts down, and at the circular wrap hardware reloads it to `len` and
    /// raises this channel's `FTFIF` for the wrap-counter ISR to service.
    ///
    /// The byte itself is stored by the caller ([`crate::usart_rx::RingBufferedRx::inject_rx_byte`]),
    /// which owns the buffer; this half is the channel bookkeeping the read path's snapshot reads.
    #[cfg(feature = "mock")]
    #[inline]
    pub fn mock_advance(&self, len: u16) {
        let rem = self.remaining();
        if rem <= 1 {
            // The lap boundary: CHxCNT reloads to the full length and FTFIF latches (uncounted until
            // the wrap ISR runs, which is exactly the pending-wrap case the snapshot handles).
            self.chcnt().write(len as u32);
            let intf = Reg32::new(self.base, DMA_INTF);
            intf.modify(1 << (4 * self.ch() + 1), 1 << (4 * self.ch() + 1));
        } else {
            self.chcnt().write((rem - 1) as u32);
        }
    }
}

// --- the transmit map ---------------------------------------------------------------------------

/// The chip-resolved DMA mapping for `USART1_TX`: controller base, channel index, and whether the
/// channel's IRQ is grouped (F1x0) or separate (F10x). The transmit twin of [`DmaRxMap`], resolved
/// from the detected family at bring-up, never probed (`specs/uart-tx-dma.md` section 3.1).
///
/// There is no `usart0_tx` and no `module_tx`: neither has a consumer, and an unexercised mapping is
/// a wrong mapping waiting to be trusted. The receive side carries those two because its ports use
/// them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaTxMap {
    /// DMA controller base ([`DMA0_BASE`] on both families).
    pub base: u32,
    /// The `USART1_TX` channel index: **6 on F10x (F103), 3 on F1x0 (F130)**.
    pub channel: u8,
    /// True if the channel's DMA IRQ is the grouped `DMA_Channel3_4` vector it shares with the
    /// link's RX channel (F1x0); false if it has its own `DMA0_Channel6` vector (F10x).
    pub grouped: bool,
}

impl DmaTxMap {
    /// Resolve the `USART1_TX` mapping for `chip` from the detected family.
    ///
    /// The channel numbers are the USART row of the DMA request tables: GD32F10x User Manual Rev2.6
    /// Table 9-3 (`USART1_TX` = DMA0 Channel 6, its own vector `DMA0_Channel6_IRQn` = 17) and
    /// GD32F1x0 User Manual Rev3.6 Table 8-3 (`USART1_TX` = Channel 3, on the grouped
    /// `DMA_Channel3_4_IRQn` = 11 it shares with `USART1_RX` on Channel 4). Neither needs a remap.
    #[inline]
    pub fn usart1_tx(chip: &Chip) -> DmaTxMap {
        match chip.irq() {
            // F10x: USART1_TX on DMA0 channel 6, its own vector.
            IrqLayout::F10xSeparate => DmaTxMap {
                base: DMA0_BASE,
                channel: 6,
                grouped: false,
            },
            // F1x0: USART1_TX on channel 3, the grouped Ch3/4 vector (shared with USART1_RX on Ch4).
            IrqLayout::F1x0Grouped => DmaTxMap {
                base: DMA0_BASE,
                channel: 3,
                grouped: true,
            },
        }
    }

    #[inline]
    fn ch(&self) -> u32 {
        self.channel as u32
    }

    /// Write-back self-check before arming: [`channel_self_check`], the same implementation the
    /// receive map uses (the probe is a property of the channel, not of the direction).
    #[inline]
    pub fn self_check(&self) -> bool {
        channel_self_check(self.base, self.ch())
    }

    /// Stop the channel (clear `CHEN`). The completion ISR calls it at the end of a transfer and the
    /// force-restart path calls it to abandon one.
    #[inline]
    pub fn disable(&self) {
        channel_disable(self.base, self.ch());
    }

    /// Clear every one of this channel's `INTF` flags ([`clear_channel_flags`]): the force-restart
    /// path's reset, so the abandoned transfer's stale completion cannot fire into the new one.
    #[inline]
    pub fn clear_flags(&self) {
        clear_channel_flags(self.base, self.ch());
    }

    /// Program the channel for a ONE-SHOT mem->periph byte transfer and start it
    /// (`specs/uart-tx-dma.md` section 3.1). `paddr` = the `USART_TDATA` address, `maddr` = the
    /// `'static` source buffer pointer, `len` = how many bytes to send.
    ///
    /// Direction mem->periph (`DIR = 1`, the one bit the receive path never set because it was
    /// always zero), peripheral address fixed (`PNAGA = 0`: every byte goes to the same `TDATA`),
    /// memory-increment on (`MNAGA = 1`), 8-bit both sides (widths 0), **`CMEN` clear** (one shot:
    /// a circular transmit would re-send the buffer forever), full-transfer interrupt on
    /// (`FTFIE`), half-transfer interrupt off (`HTFIE`: nothing acts on half a frame). `CHEN` is set
    /// LAST, after a `compiler_fence(Release)` so the buffer fill and the channel setup cannot be
    /// reordered after the DMA starts reading (the receive spec's section 6 argument, unchanged by
    /// the direction).
    ///
    /// # Safety
    /// `maddr` must point at a `'static` buffer of at least `len` bytes that lives, unmodified, for
    /// as long as the channel runs; the DMA reads it concurrently (the [`crate::usart_tx::DmaTx`]
    /// ownership contract, which is what the owned buffer and the in-flight refusal enforce).
    pub unsafe fn configure_oneshot(&self, paddr: u32, maddr: u32, len: u16) {
        let (base, ch) = (self.base, self.ch());
        // Disable before reconfiguring (a running channel ignores config writes).
        channel_disable(base, ch);
        chpaddr(base, ch).write(paddr);
        chmaddr(base, ch).write(maddr);
        chcnt(base, ch).write(len as u32);
        // Everything except CHEN. A plain write (not a modify) so CMEN and HTFIE are left CLEAR
        // whatever the previous transfer or the reset value put there.
        chctl(base, ch).write(CHCTL_DIR | CHCTL_MNAGA | CHCTL_FTFIE);
        // The buffer fill + channel setup must not be reordered after the DMA starts.
        compiler_fence(Ordering::Release);
        chctl(base, ch).modify(CHCTL_CHEN, CHCTL_CHEN);
    }

    /// The live remaining-transfer count `CHxCNT` (counts DOWN as the DMA reads bytes out of the
    /// buffer). Not part of the transmit state machine, which runs off the completion interrupt; it
    /// is what a host case and a bench read use to see how far a transfer got.
    #[inline]
    pub fn remaining(&self) -> u16 {
        (chcnt(self.base, self.ch()).read() & 0xFFFF) as u16
    }

    /// Raise this channel's full-transfer flag in `INTF` as the hardware does when `CHxCNT` reaches
    /// zero (host `mock` only): the whole buffer has been written into `TDATA`. The completion ISR
    /// is what the test then dispatches.
    #[cfg(feature = "mock")]
    #[inline]
    pub fn mock_complete(&self) {
        let bit = 1 << (4 * self.ch() + 1);
        Reg32::new(self.base, DMA_INTF).modify(bit, bit);
    }
}

// --- DMA-RX ISR context (wrap counter for lap-overrun detection) -------------------------------

/// The static DMA-RX context: the resolved channel (so the ISR knows which `INTF` bits to service,
/// ignoring the other grouped channel) plus a buffer-wrap counter. Inert until [`RingBufferedRx::new`]
/// installs it, like the other RX contexts.
struct DmaRxCtx {
    installed: AtomicBool,
    base: AtomicU32,
    channel: AtomicU8,
    /// Full-transfer completions (buffer wraps) the ISR has counted; the read path uses
    /// `wraps * len + (len - CHxCNT)` as the monotonic write position for lap-overrun detection.
    wraps: AtomicU32,
}

impl DmaRxCtx {
    const fn new() -> Self {
        DmaRxCtx {
            installed: AtomicBool::new(false),
            base: AtomicU32::new(0),
            channel: AtomicU8::new(0),
            wraps: AtomicU32::new(0),
        }
    }
}

/// The DMA-RX contexts, one independent wrap-counter per supported DMA-ring instance: index 0 = USART1
/// (its channel: Ch5 F10x / Ch4 F1x0), index 1 = the BLE-module USART (Ch2, F10x-only), index 2 =
/// USART0 on its PB6/PB7 mapping (Ch2, F1x0-only). Several [`RingBufferedRx`] may be live at once, so
/// each services ONLY its own channel's `INTF` bits and bumps ONLY its own wrap counter, with no
/// channel/context collision (`uart-rx-multi-instance.md` items 2-3). Contexts 1 and 2 name the same
/// channel INDEX (2) but never coexist: one is F10x-only and the other F1x0-only.
/// Same shape as the interrupt path's `RX_SLOTS`. Index convention matches `RxInstance::slot_index`.
static DMA_RX: [DmaRxCtx; 3] = [DmaRxCtx::new(), DmaRxCtx::new(), DmaRxCtx::new()];

/// Install the DMA-RX context `ctx_index` for `map` and register the matching ISR body (each instance
/// has its own DMA vector + handler, so the two contexts never collide). Called by `RingBufferedRx::new`.
pub(crate) fn install(map: &DmaRxMap, ctx_index: usize) {
    let ctx = &DMA_RX[ctx_index];
    ctx.wraps.store(0, Ordering::Release);
    ctx.base.store(map.base, Ordering::Release);
    ctx.channel.store(map.channel, Ordering::Release);
    ctx.installed.store(true, Ordering::Release);
    // Register the body for THIS instance's DMA vector. The module DMA channel is F10x-only, so its
    // handler only ever fires there.
    match ctx_index {
        0 => irq::register_dma_rx_handler(dma_rx_handler),
        1 => irq::register_dma_rx_handler2(module_dma_rx_handler),
        _ => irq::register_dma_rx_handler3(usart0_dma_rx_handler),
    }
}

/// The buffer-wrap count DMA-RX context `ctx_index` has observed (monotonic; wraps on `u32` overflow,
/// far beyond any bench run). The read path combines it with `CHxCNT` to detect a lapped cursor.
#[inline]
pub(crate) fn wraps(ctx_index: usize) -> u32 {
    DMA_RX[ctx_index].wraps.load(Ordering::Acquire)
}

/// The registered ISR body for USART1's DMA channel (context 0). Reached via that channel's DMA vector
/// slot through `call_dma_rx_handler`.
extern "C" fn dma_rx_handler() {
    service_dma_rx(&DMA_RX[0]);
}

/// The registered ISR body for the module USART's DMA channel (context 1, F10x-only). Reached via the
/// module DMA vector slot (`module_dma_rx_isr`) through `call_dma_rx_handler2`. Independent of
/// [`dma_rx_handler`]: it services only Ch2's flags and bumps only context 1's wrap counter.
extern "C" fn module_dma_rx_handler() {
    service_dma_rx(&DMA_RX[1]);
}

/// The registered ISR body for USART0's DMA channel (context 2, F1x0-only). Reached via the grouped
/// Ch1/2 vector slot (`usart0_dma_rx_isr`) through `call_dma_rx_handler3`. Independent of the other
/// two: it services only Ch2's flags and bumps only context 2's wrap counter, which is also the Ch1
/// demux on that grouped vector.
extern "C" fn usart0_dma_rx_handler() {
    service_dma_rx(&DMA_RX[2]);
}

/// The DMA-RX ISR / demux logic, factored out so a host test drives it against the mock `INTF`.
///
/// Reads DMA `INTF` and services ONLY the resolved channel's flags: a full-transfer (`FTFIF`) is a
/// buffer wrap, so bump the wrap counter; clear the serviced flags in `INTC`. On the F1x0 grouped
/// Ch3/4 vector this is the demux: it touches only channel 4's bits and never the other channel's, so
/// a Ch3 event is not invented (the DMA twin of `demux_grouped_timer`).
fn service_dma_rx(ctx: &DmaRxCtx) {
    if !ctx.installed.load(Ordering::Acquire) {
        return;
    }
    let base = ctx.base.load(Ordering::Acquire);
    let ch = ctx.channel.load(Ordering::Acquire) as u32;
    let intf = Reg32::new(base, DMA_INTF).read();

    let gif = 1u32 << (4 * ch);
    let ftf = 1u32 << (4 * ch + 1);
    let htf = 1u32 << (4 * ch + 2);

    let mut clear = 0u32;
    if intf & ftf != 0 {
        ctx.wraps.fetch_add(1, Ordering::AcqRel);
        clear |= ftf | gif;
    }
    if intf & htf != 0 {
        clear |= htf;
    }
    if clear != 0 {
        // INTC: writing the flag bit clears it (same bit positions as INTF). On silicon this write
        // clears the matching INTF bits; the host tests model that as a registered mock W1C pair
        // (`mock::w1c_pair`), never here.
        Reg32::new(base, DMA_INTC).write(clear);
    }
}

/// Run the wrap-counter ISR body for context `ctx_index` (host `mock` only), exactly as this
/// instance's DMA vector would: service its `FTFIF`, bump its wrap counter, clear the flags.
///
/// The injector ([`crate::usart_rx::RingBufferedRx::inject_rx_byte`]) calls this after a lap so a
/// modelled character stream reaches the steady state a real one does, where the vector fires. The
/// window BEFORE the vector runs (`FTFIF` set, wrap uncounted) is a distinct hazard with its own
/// coverage, driven by dispatching the vector directly rather than through here.
#[cfg(feature = "mock")]
#[inline]
pub(crate) fn mock_service(ctx_index: usize) {
    service_dma_rx(&DMA_RX[ctx_index]);
}

// --- DMA-TX ISR context (the in-flight flag) ---------------------------------------------------

/// The static DMA-TX context: the resolved channel (so the ISR knows which `INTF` bits are its own,
/// ignoring the other channel on the F1x0 grouped vector) plus the in-flight flag. Inert until
/// [`crate::usart_tx::DmaTx::new`] installs it, like the receive contexts.
///
/// ONE context, not an array: the only mapped transmit channel is `USART1_TX` (section 3.1), so a
/// second would be a context for a channel nothing resolves.
struct DmaTxCtx {
    installed: AtomicBool,
    base: AtomicU32,
    channel: AtomicU8,
    /// True from the moment a transfer is programmed until its completion ISR runs. It is the whole
    /// transmit state machine: `try_send` refuses while it is set, the ISR clears it. Set BEFORE
    /// `CHEN` so a completion cannot land between the start and the store and leave it stuck true.
    in_flight: AtomicBool,
}

impl DmaTxCtx {
    const fn new() -> Self {
        DmaTxCtx {
            installed: AtomicBool::new(false),
            base: AtomicU32::new(0),
            channel: AtomicU8::new(0),
            in_flight: AtomicBool::new(false),
        }
    }
}

static DMA_TX: DmaTxCtx = DmaTxCtx::new();

/// Install the DMA-TX context for `map` and register the completion ISR body. Called by
/// [`crate::usart_tx::DmaTx::new`] before it unmasks the vector. Clears any in-flight flag left by a
/// previous arming of the port (a `release` + fresh `new` starts from idle).
pub(crate) fn install_tx(map: &DmaTxMap) {
    DMA_TX.in_flight.store(false, Ordering::Release);
    DMA_TX.base.store(map.base, Ordering::Release);
    DMA_TX.channel.store(map.channel, Ordering::Release);
    DMA_TX.installed.store(true, Ordering::Release);
    irq::register_dma_tx_handler(dma_tx_handler);
}

/// Mark the context uninstalled (the transmit half is being given back): the ISR becomes a no-op and
/// nothing is left claiming a transfer is running.
pub(crate) fn uninstall_tx() {
    DMA_TX.installed.store(false, Ordering::Release);
    DMA_TX.in_flight.store(false, Ordering::Release);
}

/// True while a programmed transfer has not yet reported its completion.
#[inline]
pub(crate) fn tx_in_flight() -> bool {
    DMA_TX.in_flight.load(Ordering::Acquire)
}

/// Claim the channel for a transfer about to be programmed (stored before `CHEN`, see the field).
#[inline]
pub(crate) fn mark_tx_in_flight() {
    DMA_TX.in_flight.store(true, Ordering::Release);
}

/// Release the claim without a completion: the force-restart path, which has already disabled the
/// channel and cleared its flags.
#[inline]
pub(crate) fn clear_tx_in_flight() {
    DMA_TX.in_flight.store(false, Ordering::Release);
}

/// The registered ISR body for the transmit channel. Reached through `call_dma_tx_handler` from the
/// F10x Ch6 vector or the F1x0 grouped Ch3/4 vector.
extern "C" fn dma_tx_handler() {
    service_dma_tx(&DMA_TX);
}

/// The DMA-TX completion ISR, factored out so a host test drives it against the mock `INTF`.
///
/// Reads DMA `INTF` and services ONLY the resolved channel's full-transfer flag: clear the flag in
/// `INTC`, clear `CHEN`, clear the in-flight flag. That is all it does. It starts nothing, for the
/// reason the receive path does not auto-restart: what to do about a transmit that did not go out is
/// the caller's policy, and a driver that re-sends behind the caller's back makes that policy
/// unobservable.
///
/// The in-flight flag is cleared LAST: it is the gate `try_send` reads, so clearing it before `CHEN`
/// would let a new transfer be programmed and then have its freshly-set `CHEN` torn down here.
fn service_dma_tx(ctx: &DmaTxCtx) {
    if !ctx.installed.load(Ordering::Acquire) {
        return;
    }
    let base = ctx.base.load(Ordering::Acquire);
    let ch = ctx.channel.load(Ordering::Acquire) as u32;
    let intf = Reg32::new(base, DMA_INTF).read();

    let gif = 1u32 << (4 * ch);
    let ftf = 1u32 << (4 * ch + 1);
    if intf & ftf == 0 {
        // Not our channel's event. On the F1x0 grouped vector this is the demux: the entry belongs
        // to the receive channel alone, and inventing a completion here would clear an in-flight
        // flag for a transfer still running.
        return;
    }
    // INTC: writing the flag bit clears it (same bit positions as INTF). On silicon this write
    // clears the matching INTF bits; the host tests model that as a registered mock W1C pair.
    Reg32::new(base, DMA_INTC).write(ftf | gif);
    channel_disable(base, ch);
    ctx.in_flight.store(false, Ordering::Release);
}

/// Reset every DMA-RX context + the DMA-TX context + every registered handler (host-test teardown
/// between cases). All three receive contexts are reset so a case that installed the module or
/// USART0 channel cannot leak into the next.
#[cfg(feature = "mock")]
pub fn reset_for_test() {
    for ctx in &DMA_RX {
        ctx.installed.store(false, Ordering::Release);
        ctx.base.store(0, Ordering::Release);
        ctx.channel.store(0, Ordering::Release);
        ctx.wraps.store(0, Ordering::Release);
    }
    DMA_TX.installed.store(false, Ordering::Release);
    DMA_TX.base.store(0, Ordering::Release);
    DMA_TX.channel.store(0, Ordering::Release);
    DMA_TX.in_flight.store(false, Ordering::Release);
    irq::clear_dma_rx_handler();
    irq::clear_dma_rx_handler2();
    // The third slot is USART0's DMA handler (F1x0 PB6/PB7, the offroad BLE wiring). It was omitted
    // while nothing installed it; the BLE RX ring does, so a case that arms that instance would
    // otherwise leak its handler into the next case.
    irq::clear_dma_rx_handler3();
    irq::clear_dma_tx_handler();
}
