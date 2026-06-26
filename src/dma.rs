//! DMA channel driver for the circular USART RX (G-DMA-UART Gate B).
//!
//! Only what the DMA-ring RX ([`crate::usart_rx::RingBufferedRx`]) needs: resolve which DMA controller
//! / channel / IRQ grouping carries `USART1_RX` for the detected family ([`DmaRxMap`]), program the
//! channel for circular periph->mem reception, read the live remaining-count `CHxCNT`, and a tiny ISR
//! context that counts buffer wraps (for lap-overrun detection) while ignoring the other channel on the
//! F1x0 grouped vector (the DMA twin of the grouped advanced-timer demux).
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
const CHCTL_CMEN: u32 = 1 << 5;
const CHCTL_MNAGA: u32 = 1 << 7;
// DIR = 0 (periph->mem), PNAGA = 0 (peripheral address fixed = RDATA), PWIDTH/MWIDTH = 0 (8-bit):
// all left clear, so the programmed CHxCTL value is exactly the bits set below.

/// The sentinel for the write-back self-check (section 5.1 step 3 / section 2.5): an arbitrary
/// non-zero value written to `CHxMADDR` and read back to confirm the resolved channel responds.
const SELF_CHECK_SENTINEL: u32 = 0xA5A5_5A5A;

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

    #[inline]
    fn ch(&self) -> u32 {
        self.channel as u32
    }
    #[inline]
    fn chctl(&self) -> Reg32 {
        Reg32::new(self.base, CHCTL_OFF + CH_STRIDE * self.ch())
    }
    #[inline]
    fn chcnt(&self) -> Reg32 {
        Reg32::new(self.base, CHCNT_OFF + CH_STRIDE * self.ch())
    }
    #[inline]
    fn chpaddr(&self) -> Reg32 {
        Reg32::new(self.base, CHPADDR_OFF + CH_STRIDE * self.ch())
    }
    #[inline]
    fn chmaddr(&self) -> Reg32 {
        Reg32::new(self.base, CHMADDR_OFF + CH_STRIDE * self.ch())
    }

    /// The live remaining-transfer count `CHxCNT` (counts DOWN as the DMA writes). The DMA-ring read
    /// path derives the write index as `len - remaining()`. Read through `volatile` (it changes under
    /// hardware); the low 16 bits are the count.
    #[inline]
    pub fn remaining(&self) -> u16 {
        (self.chcnt().read() & 0xFFFF) as u16
    }

    /// Write-back self-check (section 5.1 step 3 / section 2.5): write a sentinel to `CHxMADDR` and read
    /// it back. `true` if it sticks (the DMA clock is on and the resolved base/channel is real); the
    /// one reliable DMA probe on this silicon. It confirms the channel responds, NOT that the mapping
    /// is correct (that is family detection's job). The real `CHxMADDR` is written in [`Self::configure_circular`].
    #[inline]
    pub fn self_check(&self) -> bool {
        self.chmaddr().write(SELF_CHECK_SENTINEL);
        self.chmaddr().read() == SELF_CHECK_SENTINEL
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
        self.chctl().modify(CHCTL_CHEN, 0);
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
/// (its channel: Ch5 F10x / Ch4 F1x0), index 1 = the BLE-module USART (Ch2, F10x-only). Two
/// [`RingBufferedRx`] may be live at once, so each services ONLY its own channel's `INTF` bits and bumps
/// ONLY its own wrap counter, with no channel/context collision (`uart-rx-multi-instance.md` items 2-3).
/// Same shape as the interrupt path's `RX_SLOTS`. Index convention matches `RxInstance::slot_index`.
static DMA_RX: [DmaRxCtx; 2] = [DmaRxCtx::new(), DmaRxCtx::new()];

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
        _ => irq::register_dma_rx_handler2(module_dma_rx_handler),
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
        // INTC: writing the flag bit clears it (same bit positions as INTF).
        Reg32::new(base, DMA_INTC).write(clear);
        // On silicon, that INTC write clears the matching `INTF` bits. The host-test backend is a
        // passive array with no DMA core, so model that side effect under `mock` (else a later
        // `ftf_pending` read would see the flag still set); on real MMIO the hardware did it.
        #[cfg(feature = "mock")]
        Reg32::new(base, DMA_INTF).modify(clear, 0);
    }
}

/// Reset every DMA-RX context + both registered handlers (host-test teardown between cases). Both
/// contexts are reset so a case that installed the module channel cannot leak into the next.
#[cfg(feature = "mock")]
pub fn reset_for_test() {
    for ctx in &DMA_RX {
        ctx.installed.store(false, Ordering::Release);
        ctx.base.store(0, Ordering::Release);
        ctx.channel.store(0, Ordering::Release);
        ctx.wraps.store(0, Ordering::Release);
    }
    irq::clear_dma_rx_handler();
    irq::clear_dma_rx_handler2();
}
