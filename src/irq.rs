//! RAM vector table + per-`irq`-selector handler registration + grouped demux (M3 T2).
//!
//! DECISIONS.md #6 (RAM vector table), #7 (static handler-pointer registration), and SPEC.md
//! "Interrupts: RAM vector table". The device IRQ layout differs by part (the [`IrqLayout`]
//! selector): the F1x0 GROUPED layout bundles the advanced-timer break/update/trigger/commutation
//! into ONE IRQ and groups EXTI lines; the F10x SEPARATE layout has them as distinct IRQs at
//! different positions. A single static flash table would need a layout-aware dispatcher at every
//! divergent slot, branching on every interrupt including the control loop. So runtime-hal builds a
//! vector table in RAM specialized per the selector (each slot points straight at the right
//! handler) and sets `VTOR`, with no per-ISR branch on the hot path.
//!
//! # HP-4: the GD SPL vector layout (the authority), and where the SPEC wording is off
//!
//! Resolved against the GD SPL CMSIS headers' `IRQn_Type` enum (`gd32f1x0.h` / `gd32f10x.h`) and
//! the `startup_*.s` vector tables (NOT any other library):
//!
//! - **F1x0 (grouped)** matches the SPEC `f1x0_grouped` wording: the advanced timer's break,
//!   update, trigger AND commutation are ALL bundled into one vector, `TIMER0_BRK_UP_TRG_COM_IRQn`
//!   = 13, with the capture/compare channels on a separate vector `TIMER0_Channel_IRQn` = 14. EXTI
//!   is grouped: `EXTI0_1` = 5, `EXTI2_3` = 6, `EXTI4_15` = 7. The ADC (which carries the injected
//!   end-of-conversion, the control loop) is `ADC_CMP_IRQn` = 12 (shared with the comparators).
//!   The part's highest external IRQ is 73, but the table reserves [`MAX_IRQS`] = 48 (64 entries,
//!   256 B): only an IRQ this HAL unmasks can fire, and the highest it installs is 40.
//! - **F10x (separate)** is where the SPEC's implied "fully separate" wording is INACCURATE, and
//!   this is the HP-4 discrepancy to record: the four advanced-timer sub-sources are NOT four
//!   separate vectors. `TIMER0_BRK_IRQn` = 24 and `TIMER0_UP_IRQn` = 25 are individual, but trigger
//!   and commutation SHARE one vector `TIMER0_TRG_CMT_IRQn` = 26, and the channel capture/compare is
//!   `TIMER0_Channel_IRQn` = 27. So F10x is a 4-vector layout {BRK}{UP}{TRG+CMT}{CH}, not a 4-way
//!   split of {BRK}{UP}{TRG}{CMT}. Trigger+commutation always share a vector even on the "separate"
//!   layout. EXTI on F10x: lines 0..4 individual (`EXTI0..4` = 6..10), `EXTI5_9` = 23, `EXTI10_15`
//!   = 40. The ADC is `ADC0_1_IRQn` = 18 (shared by ADC0/ADC1). Highest external IRQ (MD/HD parts)
//!   = 59, so the table is sized to cover it.
//!
//! So the demux burden is on the F1x0 grouped layout (one combined handler at IRQ 13 routing
//! break/update/trigger/commutation by reading the TIMER0 interrupt-flag register), while F10x has
//! separate slots and needs no advanced-timer demux. The control loop itself runs in the
//! injected-EOC ISR (the ADC vector: IRQ 12 on F1x0, IRQ 18 on F10x), per the reference firmware
//! and SPEC.md; T8 wires that ADC slot to the registered control handler. (No timer/ADC peripheral
//! is enabled by this module: it is the substrate.)
//!
//! # Static handler registration (DECISIONS.md #7)
//!
//! The firmware registers a `'static` control-loop handler at boot ([`register_control_handler`]);
//! the per-layout ISR bodies (and the grouped demux) call through it via [`call_control_handler`],
//! with a no-op default ([`default_control_handler`]) guarding the pre-registration window. The
//! control law stays entirely in the `control` crate.
//!
//! # Testability
//!
//! The table is built as plain data ([`build_table`]) so a host test asserts it slot-by-slot
//! against the expected handler addresses (cross-checked against the GD SPL layout above). The
//! early-boot handoff is modelled by `mock_vtor::dispatch` (a `mock`-feature module), which looks up
//! a slot and calls it the way hardware would after `VTOR` points at the table, so the flip ordering
//! and the grouped demux are host-testable; see `irq/tests.rs` for the gap vs a real Unicorn
//! exception injection. (Plain-code, not an intra-doc link: `mock_vtor` is compiled only under the
//! `mock` feature, so the default `cargo doc` build has no such item to link to.)

use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use crate::descriptor::IrqLayout;

/// A vector-table entry: a Cortex-M exception/IRQ handler function pointer.
pub type Handler = unsafe extern "C" fn();

/// Number of Cortex-M system-exception slots before the external IRQs (slot 0 = initial SP, 1 =
/// reset, 2 = NMI, 3 = HardFault, ... 15 = SysTick). The external IRQ `n` lives at table index
/// `SYSTEM_VECTORS + n`.
pub const SYSTEM_VECTORS: usize = 16;

/// External IRQ count the table reserves: 48, so the table is 64 slots = 256 B and takes a 256 B
/// VTOR alignment ([`RamVectorTable`]).
///
/// It used to reserve 74 (F1x0's highest external IRQ, `CAN1_SCE_IRQn` = 73) on the "over-provision
/// is fine" argument, which was wrong about the cost: 90 slots is 360 B, but a 360 B table needs a
/// 512 B alignment, and Rust pads the static to its alignment, so the table COST 512 B of the 8 KiB
/// part's RAM, 152 B of it padding behind slots nothing could ever route to. Only an IRQ this HAL
/// unmasks can ever fire, and the highest vector either family's table installs is
/// [`F10X_EXTI10_15_IRQ`] = 40. The 256 B this gives back is stack: the DMA transmit on the
/// inter-board link (`specs/uart-tx-dma.md`) costs the consuming firmware 160 B of RAM and stack,
/// and on an image whose armed margin was 84 B that overflowed the stack into the motor state the
/// moment the board was armed (hoverboard-firmware, `specs/bench-evidence/2026-10-24/dma-tx-drive/`).
///
/// Every IRQ number this module names is asserted below to fit; a new vector past the table fails
/// the build there, not at the first spurious entry into RAM past the table.
pub const MAX_IRQS: usize = 48;

/// Total vector-table entries (system + external IRQ).
pub const MAX_VECTORS: usize = SYSTEM_VECTORS + MAX_IRQS;

// Every vector this module can install or unmask fits the table. Compile-time: a number past the
// table is a build failure naming the constant, never a RAM table entered past its end.
const _: () = {
    assert!(F1X0_ADC_CMP_IRQ < MAX_IRQS);
    assert!(F1X0_TIMER0_BRK_UP_TRG_COM_IRQ < MAX_IRQS);
    assert!(F1X0_TIMER0_CHANNEL_IRQ < MAX_IRQS);
    assert!(F1X0_EXTI0_1_IRQ < MAX_IRQS);
    assert!(F1X0_EXTI2_3_IRQ < MAX_IRQS);
    assert!(F1X0_EXTI4_15_IRQ < MAX_IRQS);
    assert!(F10X_ADC0_1_IRQ < MAX_IRQS);
    assert!(F10X_TIMER0_BRK_IRQ < MAX_IRQS);
    assert!(F10X_TIMER0_UP_IRQ < MAX_IRQS);
    assert!(F10X_TIMER0_TRG_CMT_IRQ < MAX_IRQS);
    assert!(F10X_TIMER0_CHANNEL_IRQ < MAX_IRQS);
    assert!(F10X_EXTI5_9_IRQ < MAX_IRQS);
    assert!(F10X_EXTI10_15_IRQ < MAX_IRQS);
    assert!(F1X0_USART1_IRQ < MAX_IRQS);
    assert!(F10X_USART1_IRQ < MAX_IRQS);
    assert!(F1X0_USART0_IRQ < MAX_IRQS);
    assert!(F10X_USART_MODULE_IRQ < MAX_IRQS);
    assert!(F10X_DMA0_CH5_IRQ < MAX_IRQS);
    assert!(F10X_DMA0_CH2_IRQ < MAX_IRQS);
    assert!(F1X0_DMA_CH1_2_IRQ < MAX_IRQS);
    assert!(F1X0_DMA_CH3_4_IRQ < MAX_IRQS);
    assert!(F10X_DMA0_CH6_IRQ < MAX_IRQS);
    assert!(F10X_DMA0_CH1_IRQ < MAX_IRQS);
};

// --- GD SPL IRQ numbers (the authority; see the HP-4 note above) ------------------------------

/// F1x0 `ADC_CMP_IRQn` (the injected-EOC / control-loop vector on F1x0).
pub const F1X0_ADC_CMP_IRQ: usize = 12;
/// F1x0 `TIMER0_BRK_UP_TRG_COM_IRQn`: the combined advanced-timer vector the grouped demux serves.
pub const F1X0_TIMER0_BRK_UP_TRG_COM_IRQ: usize = 13;
/// F1x0 `TIMER0_Channel_IRQn` (the advanced-timer capture/compare channel vector).
pub const F1X0_TIMER0_CHANNEL_IRQ: usize = 14;
/// F1x0 `EXTI0_1_IRQn`.
pub const F1X0_EXTI0_1_IRQ: usize = 5;
/// F1x0 `EXTI2_3_IRQn`.
pub const F1X0_EXTI2_3_IRQ: usize = 6;
/// F1x0 `EXTI4_15_IRQn`.
pub const F1X0_EXTI4_15_IRQ: usize = 7;

/// F10x `ADC0_1_IRQn` (the injected-EOC / control-loop vector on F10x).
pub const F10X_ADC0_1_IRQ: usize = 18;
/// F10x `TIMER0_BRK_IRQn`.
pub const F10X_TIMER0_BRK_IRQ: usize = 24;
/// F10x `TIMER0_UP_IRQn`.
pub const F10X_TIMER0_UP_IRQ: usize = 25;
/// F10x `TIMER0_TRG_CMT_IRQn` (trigger AND commutation share this vector; see HP-4).
pub const F10X_TIMER0_TRG_CMT_IRQ: usize = 26;
/// F10x `TIMER0_Channel_IRQn`.
pub const F10X_TIMER0_CHANNEL_IRQ: usize = 27;
/// F10x `EXTI5_9_IRQn`.
pub const F10X_EXTI5_9_IRQ: usize = 23;
/// F10x `EXTI10_15_IRQn`.
pub const F10X_EXTI10_15_IRQ: usize = 40;

/// F1x0 `USART1_IRQn` (the bench port USART1, PA2/PA3): the vector the interrupt-buffered RX
/// ([`crate::usart_rx::BufferedRx`]) routes through. From the GD SPL `IRQn_Type` (`gd32f1x0.h:159`),
/// the same authority as the timer/ADC IRQ consts above.
pub const F1X0_USART1_IRQ: usize = 28;
/// F10x `USART1_IRQn` (`gd32f10x.h:159`). Note the SAME logical USART1 sits on a DIFFERENT vector
/// number per family (28 vs 38), so the layout-specific `build_table` arm picks the right slot.
pub const F10X_USART1_IRQ: usize = 38;
/// F1x0 `USART0_IRQn` (`gd32f1x0.h:158`): USART0 on its PB6/PB7 mapping (`pincap::usart_pins`),
/// the offroad chassis's onboard CC2541 BLE port. The interrupt-buffered RX's THIRD instance routes
/// through this vector. **F1x0-only**: on F10x those pins reach USART0 only through the AFIO
/// `USART0_REMAP` field the HAL does not implement, so `resolve_instance` refuses `Usart0` there and
/// no F10x USART0 vector const exists.
pub const F1X0_USART0_IRQ: usize = 27;
/// F10x `USART2_IRQn` (`gd32f10x.h:160`): the BLE-module USART (HAL [`crate::addr::PeriphLabel::Usart2`]
/// = GD USART2 register block at 0x4000_4800, PB10/PB11) on the F10x master. The interrupt-buffered RX's
/// SECOND instance routes through this vector. **F10x-only**: the module USART does not exist on F1x0
/// (the `uart-rx-multi-instance.md` family scope), so there is no F1x0 module-USART IRQ const and no
/// module slot in the F1x0 `build_table` arm.
pub const F10X_USART_MODULE_IRQ: usize = 39;

/// F10x `DMA0_Channel5_IRQn` (`gd32f10x.h:135`): on F10x the DMA channel carrying `USART1_RX` (Ch5)
/// has its OWN vector (separate-vector layout). The DMA-ring RX ([`crate::usart_rx::RingBufferedRx`])
/// routes through it.
pub const F10X_DMA0_CH5_IRQ: usize = 16;
/// F10x `DMA0_Channel2_IRQn` (`gd32f10x.h:132`): the DMA channel carrying the BLE-module USART's RX.
/// GD32F10x User Manual §9.4.9 (Figure 9-4 / Table 9-3, the "USART" row) maps **GD `USART2_RX` to DMA0
/// Channel 2** (the same table that maps `USART1_RX` to Ch5, the bench-proven first instance). The
/// module USART's DMA-ring RX (the [`crate::usart_rx::RingBufferedRx`] second instance) routes through
/// this vector. **F10x-only** (the module USART does not exist on F1x0), so there is no F1x0 const and
/// no module DMA slot in the F1x0 `build_table` arm. NOTE: on F1x0 this same IRQ number (13) is the
/// unrelated grouped `TIMER0_BRK_UP_TRG_COM` vector; the layout-specific `build_table` arms keep them
/// distinct.
pub const F10X_DMA0_CH2_IRQ: usize = 13;
/// F1x0 `DMA_Channel1_2_IRQn` (`gd32f1x0.h:142`): channels 1 and 2 SHARE one vector (grouped layout).
/// GD32F1x0 User Manual Rev3.6 (the `DMA_RMP` `USART0_RX_DMA_RMP` field, bit 10) maps **`USART0_RX` to
/// DMA channel 2** when not remapped (the reset state; remap would move it to channel 4), so USART0's
/// DMA-ring RX arrives here. `USART0_TX` is Ch1 unremapped, so USART0's DMA transmit arrives here too:
/// BOTH halves of the module port share this vector, exactly as both halves of the link share
/// [`F1X0_DMA_CH3_4_IRQ`], and the slot (`dma_ch1_2_isr`) runs both bodies, each demuxing by reading
/// DMA `INTF` for its own channel. Independently corroborated by the offroad stock image, whose CH2 is
/// USART0's RX channel (`hoverboard-firmware/specs/offroad-pinmap.md` section 4.3). **F1x0-only**,
/// like [`F1X0_USART0_IRQ`].
pub const F1X0_DMA_CH1_2_IRQ: usize = 10;
/// F1x0 `DMA_Channel3_4_IRQn` (`gd32f1x0.h:143`): channels 3 and 4 SHARE one vector (grouped layout).
/// BOTH halves of the inter-board link arrive here: `USART1_RX` is Ch4 and `USART1_TX` is Ch3
/// (GD32F1x0 User Manual Rev3.6 Table 8-3), so the slot ([`dma_ch3_4_isr`]) runs both registered
/// bodies and each demuxes by reading DMA `INTF` for its own channel (the DMA twin of the grouped
/// advanced-timer vector).
pub const F1X0_DMA_CH3_4_IRQ: usize = 11;
/// F10x `DMA0_Channel6_IRQn` (`gd32f10x.h:136`): on F10x the DMA channel carrying `USART1_TX` (Ch6,
/// GD32F10x User Manual Rev2.6 Table 9-3) has its OWN vector, so the transmit completion
/// ([`crate::usart_tx::DmaTx`]) needs no demux there and nothing about receive changes. **F10x-only**
/// as a transmit vector: on F1x0 the transmit channel is 3, on the grouped vector above.
pub const F10X_DMA0_CH6_IRQ: usize = 17;
/// F10x `DMA0_Channel1_IRQn` (`gd32f10x.h:131`): the DMA channel carrying the BLE-module USART's
/// TRANSMIT (GD `USART2_TX` = DMA0 Ch1, GD32F10x User Manual Rev2.6 Table 9-3, the same row that
/// puts `USART2_RX` on Ch2). Its own vector, so the module port's transmit completion needs no demux
/// on F10x. **F10x-only**: on F1x0 the module port's transmit is `USART0_TX` on Ch1, on the grouped
/// [`F1X0_DMA_CH1_2_IRQ`] it shares with that port's receive. On F1x0 IRQ 12 is the unrelated
/// `ADC_CMP` vector; the layout-specific `build_table` arms keep them distinct.
pub const F10X_DMA0_CH1_IRQ: usize = 12;

// --- TIMER0 interrupt-flag register (INTF, offset 0x10) for the grouped demux ------------------
//
// The F1x0 combined handler routes its bundled sub-sources by reading TIMER0_INTF (gd32f1x0_timer.h:
// `TIMER_INTF(timerx)` at 0x10). The bit positions are identical on both families.

/// TIMER0 INTF offset.
pub const TIMER_INTF: u32 = 0x10;
/// INTF update flag (`TIMER_INTF_UPIF`, bit 0).
pub const INTF_UPIF: u32 = 1 << 0;
/// INTF trigger flag (`TIMER_INTF_TRGIF`, bit 6).
pub const INTF_TRGIF: u32 = 1 << 6;
/// INTF commutation flag (`TIMER_INTF_CMTIF`, bit 5).
pub const INTF_CMTIF: u32 = 1 << 5;
/// INTF break flag (`TIMER_INTF_BRKIF`, bit 7).
pub const INTF_BRKIF: u32 = 1 << 7;

// --- Static control-loop handler registration (DECISIONS.md #7) -------------------------------

/// A registered control-loop handler: a `'static` extern-C function the firmware installs at boot.
pub type ControlHandler = extern "C" fn();

/// The no-op default that guards the pre-registration window (DECISIONS.md #7). Before the firmware
/// registers its control handler, the ISR path calls this, so an interrupt that fires early is a
/// safe no-op rather than a jump through a null/garbage pointer.
pub extern "C" fn default_control_handler() {}

/// The registered control handler pointer. `AtomicPtr` so registration is a single atomic store and
/// the ISR read is lock-free; starts at the no-op default.
static CONTROL_HANDLER: AtomicPtr<()> = AtomicPtr::new(default_control_handler as *mut ());

/// Register the firmware's `'static` control-loop handler (DECISIONS.md #7). Called once at boot,
/// before the peripheral interrupts that drive the loop are enabled. Replaces the no-op default.
#[inline]
pub fn register_control_handler(handler: ControlHandler) {
    CONTROL_HANDLER.store(handler as *mut (), Ordering::Release);
}

/// Reset the registered handler back to the no-op default (host tests; also models a clean teardown).
#[inline]
pub fn clear_control_handler() {
    CONTROL_HANDLER.store(default_control_handler as *mut (), Ordering::Release);
}

/// Call the registered control handler (or the no-op default if none is registered yet). The
/// per-layout ISR bodies and the grouped demux call through this for the injected-EOC / update
/// sub-source that runs the control loop.
#[inline]
pub fn call_control_handler() {
    let p = CONTROL_HANDLER.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_control_handler` or a `'static` fn the firmware
    // registered via `register_control_handler`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static periodic-tick handler registration (G7, symmetric with the control handler) -------
//
// The SysTick exception slot (system vector 15) routes through this pair, exactly the way the ADC
// vector routes through `call_control_handler`. G-TICK (`crate::timebase::Timebase`) drives SysTick
// in interrupt mode; the cortex-m-rt `#[exception] SysTick` symbol is owned by the firmware/example,
// which delegates to `crate::on_systick()` (one line, mirroring how detection's BusFault is a
// one-line `#[exception]` delegating into the HAL). `on_systick()` bumps `TICK_COUNT` and calls the
// registered tick handler. A firmware that flips `VTOR` to the RAM table instead reaches the same
// body through the `systick_handler` slot below.

/// A registered periodic-tick handler: a `'static` extern-C function the firmware installs at boot,
/// called from the SysTick exception (G-TICK). Mirrors [`ControlHandler`].
pub type TickHandler = extern "C" fn();

/// The no-op default that guards the pre-registration window (mirrors [`default_control_handler`]).
/// Before the firmware registers a tick handler, the SysTick path calls this, so a tick that fires
/// early is a safe no-op rather than a jump through a null/garbage pointer.
pub extern "C" fn default_tick_handler() {}

/// The registered tick handler pointer. `AtomicPtr` so registration is a single atomic store and the
/// ISR read is lock-free; starts at the no-op default.
static TICK_HANDLER: AtomicPtr<()> = AtomicPtr::new(default_tick_handler as *mut ());

/// A free-running count of SysTick ticks, bumped once per SysTick exception by [`on_systick`].
/// `AtomicU32` so the application can poll it from the main loop lock-free (the alternative to
/// registering a tick handler). Wraps on overflow.
static TICK_COUNT: AtomicU32 = AtomicU32::new(0);

/// Register the firmware's `'static` periodic-tick handler (G7). Called once at boot. Replaces the
/// no-op default. Symmetric with [`register_control_handler`].
#[inline]
pub fn register_tick_handler(handler: TickHandler) {
    TICK_HANDLER.store(handler as *mut (), Ordering::Release);
}

/// Reset the registered tick handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_tick_handler() {
    TICK_HANDLER.store(default_tick_handler as *mut (), Ordering::Release);
}

/// Call the registered tick handler (or the no-op default if none is registered yet).
#[inline]
pub fn call_tick_handler() {
    let p = TICK_HANDLER.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_tick_handler` or a `'static` fn the firmware registered
    // via `register_tick_handler`; both are valid `extern "C" fn()` pointers.
    let f: TickHandler = unsafe { core::mem::transmute::<*mut (), TickHandler>(p) };
    f();
}

/// The current free-running SysTick tick count (lock-free read). Bumped once per SysTick exception
/// by [`on_systick`]; an application that does not register a tick handler can instead poll this
/// from `main` to drive a tone toggle or a beep envelope. Wraps on overflow.
#[inline]
pub fn tick_count() -> u32 {
    TICK_COUNT.load(Ordering::Acquire)
}

/// Reset the tick count to zero (host tests / a fresh measurement window).
#[inline]
pub fn clear_tick_count() {
    TICK_COUNT.store(0, Ordering::Release);
}

/// The HAL's SysTick exception body: bump the tick count, then call the registered tick handler.
///
/// This is the single entry every SysTick route reaches. A firmware/example that uses the
/// cortex-m-rt flash vector table provides a one-line `#[exception] fn SysTick() { on_systick() }`
/// (the same pattern detection uses for its one-line BusFault delegate); a firmware that flips
/// `VTOR` to the RAM table reaches it through the [`build_table`] `systick_handler` slot, which calls
/// this. Either way the body lives here, in the HAL, so the wiring is one line in the application.
#[inline]
pub fn on_systick() {
    TICK_COUNT.fetch_add(1, Ordering::Release);
    call_tick_handler();
    SYSTICK_ISR_METRIC.record();
}

// --- Static USART RX handler registration (G-DMA-UART Gate A) ---------------------------------
//
// The USART1 RX vector routes through this pair, exactly the way the ADC vector routes through
// `call_control_handler`. The interrupt-buffered RX ([`crate::usart_rx::BufferedRx`]) registers its
// `'static` ISR body at bring-up via [`register_usart_rx_handler`]; the `usart1_rx_isr` slot
// ([`build_table`]) calls through it via [`call_usart_rx_handler`], with a no-op default
// ([`default_usart_rx_handler`]) guarding the pre-registration window. The ISR body (reading the
// USART, filling the ring) lives in `usart_rx`, not here, so this module stays the substrate.

/// The no-op default that guards the pre-registration window (mirrors [`default_control_handler`]).
pub extern "C" fn default_usart_rx_handler() {}

/// The registered USART RX handler pointer. `AtomicPtr` so registration is one atomic store and the
/// ISR read is lock-free; starts at the no-op default.
static USART_RX_HANDLER: AtomicPtr<()> = AtomicPtr::new(default_usart_rx_handler as *mut ());

/// Register the interrupt-buffered RX `'static` ISR body (G-DMA-UART). Called once by
/// [`crate::usart_rx::BufferedRx::new`], before it unmasks the USART IRQ. Replaces the no-op default.
/// Symmetric with [`register_control_handler`].
#[inline]
pub fn register_usart_rx_handler(handler: ControlHandler) {
    USART_RX_HANDLER.store(handler as *mut (), Ordering::Release);
}

/// Reset the registered USART RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_usart_rx_handler() {
    USART_RX_HANDLER.store(default_usart_rx_handler as *mut (), Ordering::Release);
}

/// Call the registered USART RX handler (or the no-op default if none is registered yet). The
/// `usart1_rx_isr` slot calls through this.
#[inline]
pub(crate) fn call_usart_rx_handler() {
    let p = USART_RX_HANDLER.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_usart_rx_handler` or a `'static` fn the RX bring-up
    // registered via `register_usart_rx_handler`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static USART RX handler registration for the SECOND instance (module USART, F10x) ----------
//
// The module USART's RX vector (F10x `USART2_IRQn` = 39, `module_usart_rx_isr`) routes through this
// SECOND, fully-independent handler pair so two interrupt-buffered receivers (USART1 + the module
// USART) can be live at once with no vector collision (`uart-rx-multi-instance.md` items 1-2). Same
// shape as the USART1 pair above; the body (its own static RX slot) lives in [`crate::usart_rx`].

/// The no-op default that guards the module-USART RX vector before its handler is registered (mirrors
/// [`default_usart_rx_handler`]).
pub extern "C" fn default_usart_rx_handler2() {}

/// The registered module-USART RX handler pointer. `AtomicPtr`, starts at the no-op default.
static USART_RX_HANDLER2: AtomicPtr<()> = AtomicPtr::new(default_usart_rx_handler2 as *mut ());

/// Register the module-USART interrupt-buffered RX `'static` ISR body. Called by
/// [`crate::usart_rx::BufferedRx::new`] when the second (module) instance is brought up, before it
/// unmasks the module USART IRQ. Symmetric with [`register_usart_rx_handler`].
#[inline]
pub fn register_usart_rx_handler2(handler: ControlHandler) {
    USART_RX_HANDLER2.store(handler as *mut (), Ordering::Release);
}

/// Reset the module-USART RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_usart_rx_handler2() {
    USART_RX_HANDLER2.store(default_usart_rx_handler2 as *mut (), Ordering::Release);
}

/// Call the registered module-USART RX handler (or the no-op default). The `module_usart_rx_isr` slot
/// calls through this.
#[inline]
pub(crate) fn call_usart_rx_handler2() {
    let p = USART_RX_HANDLER2.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_usart_rx_handler2` or a `'static` fn the RX bring-up
    // registered via `register_usart_rx_handler2`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static USART RX handler registration: the THIRD instance (USART0 on PB6/PB7) --------------
//
// USART0's RX vector (F1x0 `USART0_IRQn` = 27, `usart0_rx_isr`) routes through this THIRD, fully
// independent handler pair, so a third interrupt-buffered receiver can be live alongside USART1 and
// the module USART with no vector collision. Same shape as the two pairs above; the body (its own
// static RX slot) lives in [`crate::usart_rx`]. **F1x0-only** (see [`F1X0_USART0_IRQ`]).

/// The no-op default that guards the USART0 RX vector before its handler is registered (mirrors
/// [`default_usart_rx_handler`]).
pub extern "C" fn default_usart_rx_handler3() {}

/// The registered USART0 RX handler pointer. `AtomicPtr`, starts at the no-op default.
static USART_RX_HANDLER3: AtomicPtr<()> = AtomicPtr::new(default_usart_rx_handler3 as *mut ());

/// Register the USART0 interrupt-buffered RX `'static` ISR body. Called by
/// [`crate::usart_rx::BufferedRx::new`] when the USART0 instance is brought up, before it unmasks the
/// USART0 IRQ. Symmetric with [`register_usart_rx_handler`].
#[inline]
pub fn register_usart_rx_handler3(handler: ControlHandler) {
    USART_RX_HANDLER3.store(handler as *mut (), Ordering::Release);
}

/// Reset the USART0 RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_usart_rx_handler3() {
    USART_RX_HANDLER3.store(default_usart_rx_handler3 as *mut (), Ordering::Release);
}

/// Call the registered USART0 RX handler (or the no-op default). The `usart0_rx_isr` slot calls
/// through this.
#[inline]
pub(crate) fn call_usart_rx_handler3() {
    let p = USART_RX_HANDLER3.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_usart_rx_handler3` or a `'static` fn the RX bring-up
    // registered via `register_usart_rx_handler3`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static DMA-RX handler registration (G-DMA-UART Gate B) ------------------------------------
//
// The DMA-channel vector that carries USART1_RX (its own vector on F10x, the grouped Ch3/4 vector on
// F1x0) routes through this pair, mirroring the USART RX pair above. The DMA-ring RX
// ([`crate::usart_rx::RingBufferedRx`]) registers a `'static` handler ([`crate::dma`]) that reads the
// DMA `INTF`, services only the resolved channel's flags (the demux), and counts buffer wraps.

/// The no-op default that guards the pre-registration window (mirrors [`default_usart_rx_handler`]).
pub extern "C" fn default_dma_rx_handler() {}

/// The registered DMA-RX handler pointer. `AtomicPtr`, starts at the no-op default.
static DMA_RX_HANDLER: AtomicPtr<()> = AtomicPtr::new(default_dma_rx_handler as *mut ());

/// Register the DMA-ring RX `'static` ISR body (G-DMA-UART). Called by
/// [`crate::usart_rx::RingBufferedRx::new`] before it unmasks the DMA IRQ. Replaces the no-op default.
#[inline]
pub fn register_dma_rx_handler(handler: ControlHandler) {
    DMA_RX_HANDLER.store(handler as *mut (), Ordering::Release);
}

/// Reset the registered DMA-RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_dma_rx_handler() {
    DMA_RX_HANDLER.store(default_dma_rx_handler as *mut (), Ordering::Release);
}

/// Call the registered DMA-RX handler (or the no-op default if none is registered yet). The
/// `dma_rx_isr` slot calls through this.
#[inline]
pub fn call_dma_rx_handler() {
    let p = DMA_RX_HANDLER.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_dma_rx_handler` or a `'static` fn the RX bring-up
    // registered via `register_dma_rx_handler`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static DMA-RX handler registration for the SECOND instance (module USART, F10x) -----------
//
// The module USART's DMA channel (GD `USART2_RX` = DMA0 Ch2, its own F10x vector `DMA0_Channel2_IRQn`
// = 13, `module_dma_rx_isr`) routes through this SECOND, fully-independent DMA-RX handler pair so two
// `RingBufferedRx` (USART1 + module) can be live at once with no DMA-channel/context collision
// (`uart-rx-multi-instance.md` items 2-3). Same shape as the USART1 DMA-RX pair above; the body (its
// own wrap-counter context, servicing ONLY Ch2's `INTF` bits) lives in [`crate::dma`].

/// The no-op default that guards the module DMA-RX vector before its handler is registered (mirrors
/// [`default_dma_rx_handler`]).
pub extern "C" fn default_dma_rx_handler2() {}

/// The registered module DMA-RX handler pointer. `AtomicPtr`, starts at the no-op default.
static DMA_RX_HANDLER2: AtomicPtr<()> = AtomicPtr::new(default_dma_rx_handler2 as *mut ());

/// Register the module DMA-ring RX `'static` ISR body. Called by
/// [`crate::usart_rx::RingBufferedRx::new`] when the second (module) instance is brought up, before it
/// unmasks the module DMA IRQ. Symmetric with [`register_dma_rx_handler`].
#[inline]
pub fn register_dma_rx_handler2(handler: ControlHandler) {
    DMA_RX_HANDLER2.store(handler as *mut (), Ordering::Release);
}

/// Reset the module DMA-RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_dma_rx_handler2() {
    DMA_RX_HANDLER2.store(default_dma_rx_handler2 as *mut (), Ordering::Release);
}

/// Call the registered module DMA-RX handler (or the no-op default). The `module_dma_rx_isr` slot
/// calls through this.
#[inline]
pub fn call_dma_rx_handler2() {
    let p = DMA_RX_HANDLER2.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_dma_rx_handler2` or a `'static` fn the RX bring-up
    // registered via `register_dma_rx_handler2`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static DMA-RX handler registration: the THIRD instance (USART0's channel) ----------------
//
// USART0's DMA channel (`USART0_RX` = DMA Ch2 unremapped, on the grouped F1x0 vector
// `DMA_Channel1_2_IRQn` = 10, `dma_ch1_2_isr`) routes through this THIRD, fully-independent
// DMA-RX handler pair, so a third `RingBufferedRx` can be live with no DMA-channel/context collision.
// Same shape as the two pairs above; the body (its own wrap-counter context, servicing ONLY Ch2's
// `INTF` bits) lives in [`crate::dma`]. **F1x0-only** (see [`F1X0_DMA_CH1_2_IRQ`]).

/// The no-op default that guards the USART0 DMA-RX vector before its handler is registered.
pub extern "C" fn default_dma_rx_handler3() {}

/// The registered USART0 DMA-RX handler pointer. `AtomicPtr`, starts at the no-op default.
static DMA_RX_HANDLER3: AtomicPtr<()> = AtomicPtr::new(default_dma_rx_handler3 as *mut ());

/// Register the USART0 DMA-ring RX `'static` ISR body. Called by
/// [`crate::usart_rx::RingBufferedRx::new`] when the USART0 instance is brought up, before it unmasks
/// the grouped Ch1/2 DMA IRQ. Symmetric with [`register_dma_rx_handler`].
#[inline]
pub fn register_dma_rx_handler3(handler: ControlHandler) {
    DMA_RX_HANDLER3.store(handler as *mut (), Ordering::Release);
}

/// Reset the USART0 DMA-RX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_dma_rx_handler3() {
    DMA_RX_HANDLER3.store(default_dma_rx_handler3 as *mut (), Ordering::Release);
}

/// Call the registered USART0 DMA-RX handler (or the no-op default). The `dma_ch1_2_isr` slot
/// calls through this.
#[inline]
pub fn call_dma_rx_handler3() {
    let p = DMA_RX_HANDLER3.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_dma_rx_handler3` or a `'static` fn the RX bring-up
    // registered via `register_dma_rx_handler3`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static DMA-TX handler registration (G-DMA-TX) ---------------------------------------------
//
// The DMA channel that carries `USART1_TX` (its own Ch6 vector on F10x, the grouped Ch3/4 vector it
// shares with the link's RX channel on F1x0) routes through this pair, mirroring the receive pairs
// above. The transmit path ([`crate::usart_tx::DmaTx`]) registers a `'static` handler
// ([`crate::dma`]) that reads DMA `INTF`, services only its own channel's completion, and clears the
// in-flight flag.

/// The no-op default that guards the DMA-TX vector before a transmitter is armed.
pub extern "C" fn default_dma_tx_handler() {}

/// The registered DMA-TX handler pointer. `AtomicPtr`, starts at the no-op default.
static DMA_TX_HANDLER: AtomicPtr<()> = AtomicPtr::new(default_dma_tx_handler as *mut ());

/// Register the DMA-TX completion ISR body (G-DMA-TX). Called by [`crate::usart_tx::DmaTx::new`]
/// before it unmasks the transmit vector. Symmetric with [`register_dma_rx_handler`].
#[inline]
pub fn register_dma_tx_handler(handler: ControlHandler) {
    DMA_TX_HANDLER.store(handler as *mut (), Ordering::Release);
}

/// Reset the DMA-TX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_dma_tx_handler() {
    DMA_TX_HANDLER.store(default_dma_tx_handler as *mut (), Ordering::Release);
}

/// Call the registered DMA-TX handler (or the no-op default). The [`dma_tx_isr`] slot (F10x) and the
/// [`dma_ch3_4_isr`] slot (F1x0) call through this.
#[inline]
pub fn call_dma_tx_handler() {
    let p = DMA_TX_HANDLER.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_dma_tx_handler` or a `'static` fn the TX bring-up
    // registered via `register_dma_tx_handler`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Static DMA-TX handler registration: the SECOND transmitter (the module port's channel) -----
//
// The BLE module port's transmit channel (`USART2_TX` on its own DMA0 Ch1 vector on F10x, `USART0_TX`
// on the grouped Ch1/2 vector on F1x0) routes through this SECOND, fully-independent pair, so the
// link's and the module port's transmitters can both be live with no handler or context collision.
// Same shape as the pair above; the body (its own in-flight context, servicing ONLY Ch1's `INTF`
// bits) lives in [`crate::dma`].

/// The no-op default that guards the module port's DMA-TX vector before its transmitter is armed.
pub extern "C" fn default_dma_tx_handler2() {}

/// The registered module-port DMA-TX handler pointer. `AtomicPtr`, starts at the no-op default.
static DMA_TX_HANDLER2: AtomicPtr<()> = AtomicPtr::new(default_dma_tx_handler2 as *mut ());

/// Register the module port's DMA-TX completion ISR body. Called by [`crate::usart_tx::DmaTx::new`]
/// when the module port's transmitter is armed, before it unmasks the vector.
#[inline]
pub fn register_dma_tx_handler2(handler: ControlHandler) {
    DMA_TX_HANDLER2.store(handler as *mut (), Ordering::Release);
}

/// Reset the module port's DMA-TX handler back to the no-op default (host tests / clean teardown).
#[inline]
pub fn clear_dma_tx_handler2() {
    DMA_TX_HANDLER2.store(default_dma_tx_handler2 as *mut (), Ordering::Release);
}

/// Call the registered module-port DMA-TX handler (or the no-op default). The `module_dma_tx_isr`
/// slot (F10x) and the `dma_ch1_2_isr` slot (F1x0) call through this.
#[inline]
pub fn call_dma_tx_handler2() {
    let p = DMA_TX_HANDLER2.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_dma_tx_handler2` or a `'static` fn the TX bring-up
    // registered via `register_dma_tx_handler2`; both are valid `extern "C" fn()` pointers.
    let f: ControlHandler = unsafe { core::mem::transmute::<*mut (), ControlHandler>(p) };
    f();
}

// --- Per-vector ISR entry counting (permanent observability) ----------------------------------
//
// A small, always-on facility that counts entries into the interrupt vectors the inter-board-link
// CPU budget rides on (the SysTick tick, the USART1 RX IDLE/error vector, and the DMA-RX wrap
// vector). Each instrumented vector bumps a per-vector entry counter; the application publishes
// these (e.g. the firmware's `CTRL_OBS`) so a bench read over SWD, taking a delta over a timed
// window, sees each vector's entry RATE WITHOUT a throwaway probe build. The overhead is one
// relaxed atomic add per instrumented ISR; the hot control-loop ADC vector is deliberately NOT
// instrumented so the PWM-rate path is untouched.
//
// Only entries are counted, not in-ISR cycles: the round-7a bench characterisation found the
// NVIC-handler-mode DWT `CYCCNT` delta carries a ~19x per-entry inflation artifact, so below-pass
// cycle attribution is not a trustworthy observable. Entry counts alone discriminate a per-byte
// storm (thousands/s) from the expected IDLE/wrap rate (hundreds/s). [`enable_cycle_counter`] /
// [`cycle_count`] remain for wall-anchored, pass-scale timing windows taken by an attached bench.

/// A per-vector ISR metric: a free-running entry count (wrapping `u32`). The application reads it
/// and takes a delta over a timed window to get a per-vector entry rate. Written only from the
/// instrumented ISR body (a single writer per vector); read lock-free from the main thread.
pub struct IsrMetric {
    entries: AtomicU32,
}

impl IsrMetric {
    const fn new() -> Self {
        Self {
            entries: AtomicU32::new(0),
        }
    }

    /// Count one ISR invocation: bump the entry count.
    #[inline]
    fn record(&self) {
        self.entries.fetch_add(1, Ordering::Relaxed);
    }

    /// The free-running entry count (wraps on overflow). Take a delta over a timed window for a rate.
    #[inline]
    pub fn entries(&self) -> u32 {
        self.entries.load(Ordering::Relaxed)
    }
}

/// SysTick ISR metric (the tick body [`on_systick`]).
pub static SYSTICK_ISR_METRIC: IsrMetric = IsrMetric::new();
/// USART1 RX ISR metric (the [`usart1_rx_isr`] vector: IDLE boundary + line errors under DMA RX).
pub static USART1_RX_ISR_METRIC: IsrMetric = IsrMetric::new();
/// DMA-RX ISR metric (the [`dma_rx_isr`] vector: the circular-DMA half/full wrap servicing).
pub static DMA_RX_ISR_METRIC: IsrMetric = IsrMetric::new();

/// Enable the DWT cycle counter (`CYCCNT`) so [`cycle_count`] is live for wall-anchored, pass-scale
/// timing windows taken by an attached bench. Call once at boot, after the clock tree is up. Sets
/// `DEMCR.TRCENA` then
/// `DWT_CTRL.CYCCNTENA` and zeroes `CYCCNT`. Idempotent. A no-op under the host `mock` build.
#[cfg(not(feature = "mock"))]
#[inline]
pub fn enable_cycle_counter() {
    const DEMCR: *mut u32 = 0xE000_EDFC as *mut u32; // Debug Exception and Monitor Control Register
    const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
    const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;
    // SAFETY: three word-aligned core debug/DWT registers; single-writer boot-time init.
    unsafe {
        DEMCR.write_volatile(DEMCR.read_volatile() | (1 << 24)); // TRCENA
        DWT_CYCCNT.write_volatile(0);
        DWT_CTRL.write_volatile(DWT_CTRL.read_volatile() | 1); // CYCCNTENA
    }
}

/// Host stand-in: no DWT under `cargo test`.
#[cfg(feature = "mock")]
#[inline]
pub fn enable_cycle_counter() {}

/// Read the DWT cycle counter (`CYCCNT`, `0xE000_1004`). A free-running CPU-cycle counter once
/// [`enable_cycle_counter`] ran; deltas of two reads bound the CPU cycles between them (including
/// any ISR that preempted the window, since the CPU runs it). Reads 0 under the host `mock` build.
#[cfg(not(feature = "mock"))]
#[inline]
pub fn cycle_count() -> u32 {
    // SAFETY: a single volatile read of the word-aligned DWT CYCCNT register.
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

/// The harness-driven cycle counter under the host `mock` build (there is no DWT): it reads 0 until
/// a case advances it with [`mock_advance_cycles`]. It is the TEST's clock, not a modelled one, so a
/// driver whose recovery is time-based (the transmit force-restart budget) can be driven to its
/// deadline without a wall clock, the way `mock_vtor` drives a vector without an NVIC.
#[cfg(feature = "mock")]
static MOCK_CYCLES: AtomicU32 = AtomicU32::new(0);

/// Host stand-in: no DWT under `cargo test`, so the count is whatever the harness advanced it to.
#[cfg(feature = "mock")]
#[inline]
pub fn cycle_count() -> u32 {
    MOCK_CYCLES.load(Ordering::Relaxed)
}

/// Advance the host stand-in cycle counter by `n` (host `mock` only): the harness making time pass.
#[cfg(feature = "mock")]
#[inline]
pub fn mock_advance_cycles(n: u32) {
    MOCK_CYCLES.fetch_add(n, Ordering::Relaxed);
}

/// Zero the host stand-in cycle counter (host-test teardown between cases).
#[cfg(feature = "mock")]
#[inline]
pub fn mock_reset_cycles() {
    MOCK_CYCLES.store(0, Ordering::Relaxed);
}

// --- NVIC / SysTick interrupt-priority configuration (R7) --------------------------------------
//
// The motor-era execution model (`specs/motor-integration.md`) requires a strict interrupt-priority
// ordering the HAL previously could not express: every external IRQ runs at reset priority 0 and the
// only priority ever set is SysTick (poked into SCB directly by the firmware). R7 is the missing
// priority-configuration API, verified at silicon stage 1 by reading the NVIC IPR registers back over
// SWD.
//
// Register facts (Cortex-M3, identical on both bench families):
// - External-IRQ priorities live in the NVIC IPR byte array at 0xE000_E400: IRQ `n`'s 8-bit priority
//   is the byte at `0xE000_E400 + n` (four IRQs per 32-bit IPR word).
// - SysTick (a system exception, not an external IRQ) has its priority in SCB SHPR3 (0xE000_ED20),
//   the top byte (`bits[31:24]`).
// - GD32F10x/F1x0 implement `__NVIC_PRIO_BITS = 4`: only the TOP 4 bits of each 8-bit field are
//   significant, so priorities move in steps of 0x10 and LOWER numeric value = HIGHER urgency.
//
// GROUPING DECISION (documented, no write): the priority-group split (AIRCR PRIGROUP) is left at its
// reset value 0. On a 4-priority-bit part PRIGROUP = 0 makes all four implemented bits PREEMPTION
// priority with zero sub-priority bits, so any two distinct priority values strictly preempt one
// another, which is exactly the ordering the execution model needs. No AIRCR write is emitted (there
// is nothing to change from reset), so this module never touches PRIGROUP.
//
// The writes go through [`crate::reg::Reg32`] (word RMW), so they are host-testable against the mock
// register space and read back with [`irq_priority`] / [`systick_priority`] the same way a bench SWD
// read confirms the IPR bytes on silicon.

use crate::reg::Reg32;

/// NVIC IPR byte-array base (`0xE000_E400`): external IRQ `n`'s priority byte is at `base + n`.
pub const NVIC_IPR_BASE: u32 = 0xE000_E400;
/// SCB SHPR3 (`0xE000_ED20`): the SysTick priority is its top byte (`bits[31:24]`).
pub const SCB_SHPR3: u32 = 0xE000_ED20;

/// The period (motor hot-path) ISR priority: the HIGHEST urgency in the image (lowest numeric). It
/// must outrank the link RX vectors and SysTick, per the execution model (a link ISR delaying a
/// commutation step is a torque glitch; the reverse is a few resync-tolerant dropped bytes).
pub const MOTOR_ISR_PRIORITY: u8 = 0x10;
/// The USART1 RX vector priority: below the period ISR, above the DMA-RX vector.
pub const USART1_RX_PRIORITY: u8 = 0x40;
/// The DMA-RX vector priority: below USART1 RX, above SysTick.
pub const DMA_RX_PRIORITY: u8 = 0x70;
/// The SysTick priority: the lowest urgency (highest numeric), the stock 0xF0 the scheduler runs at.
pub const SYSTICK_PRIORITY: u8 = 0xF0;

/// The NVIC IPR word holding IRQ `irq`'s priority byte, and the byte's shift within that word.
#[inline]
const fn ipr_word(irq: usize) -> (Reg32, u32) {
    let word = (irq / 4) as u32 * 4;
    let shift = (irq % 4) as u32 * 8;
    (Reg32::new(NVIC_IPR_BASE, word), shift)
}

/// Set external IRQ `irq`'s NVIC priority (R7). RMW of the containing IPR word so the three
/// neighbouring IRQs' priority bytes are preserved. Only the top `__NVIC_PRIO_BITS` (4) bits are
/// significant on these parts; lower numeric value = higher urgency. Reads back via [`irq_priority`]
/// (the same byte a bench SWD read of the IPR confirms).
#[inline]
pub fn set_irq_priority(irq: usize, priority: u8) {
    let (reg, shift) = ipr_word(irq);
    reg.modify(0xFF << shift, (priority as u32) << shift);
}

/// Read external IRQ `irq`'s NVIC priority byte back (the host-test / read-back companion to
/// [`set_irq_priority`]; the bench reads the same byte over SWD).
#[inline]
pub fn irq_priority(irq: usize) -> u8 {
    let (reg, shift) = ipr_word(irq);
    ((reg.read() >> shift) & 0xFF) as u8
}

/// Set the SysTick exception priority (SCB SHPR3 top byte). RMW so the other three system-handler
/// priority bytes in SHPR3 (bits[23:0], reserved on these parts) are preserved. This is the HAL
/// surface for the SysTick priority the firmware otherwise pokes into SCB directly.
#[inline]
pub fn set_systick_priority(priority: u8) {
    Reg32::new(SCB_SHPR3, 0).modify(0xFF00_0000, (priority as u32) << 24);
}

/// Read the SysTick exception priority back (SCB SHPR3 top byte).
#[inline]
pub fn systick_priority() -> u8 {
    (Reg32::new(SCB_SHPR3, 0).read() >> 24) as u8
}

/// The motor-era vector IRQ numbers for `layout`, in the order the priority ordering names them:
/// `(period_isr, usart1_rx, dma_rx)`. The PERIOD ISR is the ADC injected-EOC vector (the reference's
/// current-sense driving model: the loop rate IS the PWM rate, so the period vector completes an
/// injected conversion). The no-current-sense update-vector period path is not wired here (no such
/// board is in the validation loop; `set_irq_priority` serves it directly when it lands).
#[inline]
pub const fn motor_era_irqs(layout: IrqLayout) -> (usize, usize, usize) {
    match layout {
        IrqLayout::F1x0Grouped => (F1X0_ADC_CMP_IRQ, F1X0_USART1_IRQ, F1X0_DMA_CH3_4_IRQ),
        IrqLayout::F10xSeparate => (F10X_ADC0_1_IRQ, F10X_USART1_IRQ, F10X_DMA0_CH5_IRQ),
    }
}

/// The link's DMA TRANSMIT vector for `layout`: its own `DMA0_Channel6` vector on F10x, the grouped
/// Ch3/4 vector (shared with the link's receive channel) on F1x0.
///
/// Resolved here rather than named by the caller for the same reason the receive vectors are, and
/// named at all because [`apply_motor_era_priorities`] must cover it: a transmitter unmasks it, and
/// an NVIC priority byte RESETS TO 0, the most urgent value there is, so an unwritten one preempts
/// the period ISR. On F1x0 it resolves to the same vector as the receive side, where writing the
/// same priority twice costs a register write and nothing else.
#[inline]
pub const fn dma_tx_irq(layout: IrqLayout) -> usize {
    match layout {
        IrqLayout::F1x0Grouped => F1X0_DMA_CH3_4_IRQ,
        IrqLayout::F10xSeparate => F10X_DMA0_CH6_IRQ,
    }
}

/// The MODULE port's DMA TRANSMIT vector for `layout`: its own `DMA0_Channel1` vector on F10x (GD
/// `USART2_TX`), the grouped Ch1/2 vector (shared with that port's receive channel) on F1x0 (GD
/// `USART0_TX`). The module-port twin of [`dma_tx_irq`], named for the same reason: a transmitter
/// unmasks it, so [`apply_motor_era_priorities`] must write its priority byte.
#[inline]
pub const fn module_tx_irq(layout: IrqLayout) -> usize {
    match layout {
        IrqLayout::F1x0Grouped => F1X0_DMA_CH1_2_IRQ,
        IrqLayout::F10xSeparate => F10X_DMA0_CH1_IRQ,
    }
}

/// The MODULE-port RX vector pair for `layout`: `(usart_rx, dma_rx)` for the USART carrying the
/// onboard CC2541 BLE module. Mirrored across the fleet, which is why it is resolved from the layout
/// rather than named by the caller: the standard family puts the module on USART2 (PB10/PB11, its own
/// DMA0 Ch2 vector), the classywalk offroad family on USART0 (PB6/PB7, the grouped Ch1/2 vector).
///
/// The companion of [`motor_era_irqs`], and the second half of what
/// [`apply_motor_era_priorities`] must cover: these two vectors are unmasked by a
/// [`crate::usart_rx::RingBufferedRx`] armed on the module port.
#[inline]
pub const fn module_rx_irqs(layout: IrqLayout) -> (usize, usize) {
    match layout {
        IrqLayout::F1x0Grouped => (F1X0_USART0_IRQ, F1X0_DMA_CH1_2_IRQ),
        IrqLayout::F10xSeparate => (F10X_USART_MODULE_IRQ, F10X_DMA0_CH2_IRQ),
    }
}

/// Apply the motor-era interrupt-priority ordering for `layout` (R7, the single-owner surface): the
/// period (ADC injected-EOC) ISR above the USART RX vectors above the DMA-RX vectors above SysTick.
/// Resolves the per-family vector numbers from [`motor_era_irqs`] + [`module_rx_irqs`] and writes the
/// priorities; silicon stage 1 reads the IPR + SHPR3 bytes back over SWD to confirm the ordering is
/// real. Call once at boot, before the vectors are unmasked.
///
/// **Both RX ports, not just the inter-board one.** An NVIC priority byte resets to 0, which is the
/// MOST urgent value there is: any vector left unwritten preempts the period ISR, the tightest budget
/// on the part. While only the inter-board port carried a receiver that was harmless, because nothing
/// unmasked the module port's vectors. A BLE RX ring unmasks both of them, so the ordering has to name
/// them or arming the module port would silently outrank the motor. The policy covers every vector
/// this HAL's receivers can unmask, on both families, so it does not depend on which wiring a given
/// board turns out to have; writing a priority enables nothing, so covering a vector that stays masked
/// costs a register write and nothing else.
pub fn apply_motor_era_priorities(layout: IrqLayout) {
    let (period, usart1, dma) = motor_era_irqs(layout);
    set_irq_priority(period, MOTOR_ISR_PRIORITY);
    set_irq_priority(usart1, USART1_RX_PRIORITY);
    set_irq_priority(dma, DMA_RX_PRIORITY);
    let (module_usart, module_dma) = module_rx_irqs(layout);
    set_irq_priority(module_usart, USART1_RX_PRIORITY);
    set_irq_priority(module_dma, DMA_RX_PRIORITY);
    // The link's DMA TRANSMIT vector, on the same rung as the DMA receive vectors: it is a
    // completion flag being cleared, the least urgent interrupt work in the image, and
    // [`crate::usart_tx::DmaTx::new`] unmasks it, so leaving its reset priority of 0 in place would
    // put a transmit completion above the motor's period ISR.
    set_irq_priority(dma_tx_irq(layout), DMA_RX_PRIORITY);
    // The module port's DMA transmit vector, on the same rung and for the same reason. On F1x0 it is
    // the module receive's grouped vector again, written twice for nothing worse than a store.
    set_irq_priority(module_tx_irq(layout), DMA_RX_PRIORITY);
    set_systick_priority(SYSTICK_PRIORITY);
}

/// A device IRQ number as a [`cortex_m::interrupt::InterruptNumber`] for `NVIC::unmask`.
#[cfg(not(feature = "mock"))]
#[derive(Clone, Copy)]
struct IrqNum(u16);

#[cfg(not(feature = "mock"))]
// SAFETY: the value is a GD SPL `IRQn_Type` number, a valid device IRQ number for this part.
unsafe impl cortex_m::interrupt::InterruptNumber for IrqNum {
    #[inline]
    fn number(self) -> u16 {
        self.0
    }
}

/// Unmask (enable) an external IRQ in the NVIC: the "enable its vector" half of registering a
/// handler, and the single owner of NVIC set-enable writes (as [`set_irq_priority`] is of the
/// priority bytes).
///
/// The caller is responsible for the vector's table routing (the RAM table's slot) and for its
/// priority BEFORE unmasking: unmasking is what first allows the IRQ to fire. Hardware-only; under
/// the host `mock` build there is no NVIC and this is a no-op (the host suite drives ISR bodies
/// through `mock_vtor::dispatch` instead).
#[cfg(not(feature = "mock"))]
#[inline]
pub fn unmask_irq(irq: usize) {
    // SAFETY: enabling a vector whose table slot + priority the caller has already set up; the
    // masking-based critical sections this crate uses do not rely on this IRQ staying masked.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(IrqNum(irq as u16));
    }
}

/// Host stand-in: no NVIC under `cargo test`.
#[cfg(feature = "mock")]
#[inline]
pub fn unmask_irq(_irq: usize) {}

// --- The owned RAM vector table (DECISIONS.md #6) ---------------------------------------------

/// The owned RAM vector table (DECISIONS.md #6): an alignment-correct `static` in a dedicated
/// section. The Cortex-M `VTOR` requires the table to be aligned to a power of two at least as large
/// as the table: `MAX_VECTORS * 4 = 256` bytes, so the table is `align(256)` (the `align(R) >=
/// TBLOFF` rule, the exact power-of-two granule), and the struct's size is exactly its alignment, so
/// the static carries no padding.
///
/// The alignment has come down twice, and each step was RAM for the 8 KiB part's stack. 1024 -> 512
/// reclaimed 1 KiB (512 B of struct padding plus 512 B the linker no longer spent padding up to the
/// boundary), one half of the two-part fix for the deep `store` flash-write path (with the firmware's
/// `store::MAX_RECORD = 272`). 512 -> 256 came with [`MAX_IRQS`] 74 -> 48 and reclaims 256 B of
/// struct padding plus whatever the consuming linker script spent reaching the 512 boundary (see
/// [`MAX_IRQS`] for the trigger). The detect-probe table is a SEPARATE `align(128)`
/// `AlignedVectorTable` in [`crate::detect`] holding only the 16 reachable system vectors, unaffected
/// by this. `#[no_mangle]` + the section let the linker place it in RAM; the flash table still covers
/// reset + the first exceptions until [`install`] flips `VTOR`.
#[repr(C, align(256))]
pub struct RamVectorTable {
    /// The vector slots (slot 0 reserved for the initial SP value the hardware reads; slots 1..15
    /// the system exceptions; 16.. the external IRQs).
    pub slots: [usize; MAX_VECTORS],
}

// VTOR alignment guard: on the 32-bit target each slot is 4 bytes, and the table base must be aligned
// to a power of two >= its byte size. `align(256)` is valid only while the table fits in 256 bytes
// (<= 64 slots); if MAX_VECTORS ever exceeds that, bump align(..) to the next power of two or VTOR
// would reject the (then-unaligned) table base. (Asserted on the slot COUNT so it is the same on the
// 32-bit firmware target and the 64-bit host test build.)
const _: () = assert!(
    MAX_VECTORS <= 64,
    "RamVectorTable exceeds its 256-byte VTOR alignment; bump align(..) to the next power of two"
);

/// Build the RAM vector table contents for the given [`IrqLayout`], as plain data (so it is
/// host-testable slot-by-slot). Every slot defaults to [`default_isr`]; the layout then overwrites
/// the timer/ADC/EXTI slots with the right handler per the GD SPL layout (the HP-4 note).
///
/// Slot 0 (initial SP) and slot 1 (reset) are left as the default placeholder here: the flash table
/// owns reset, and the RAM table is only ever entered AFTER reset + RAM init (DECISIONS.md #6), so
/// those two slots are never used from the RAM table. The system exceptions (NMI..SysTick) and the
/// IRQs are filled.
pub fn build_table(layout: IrqLayout) -> [usize; MAX_VECTORS] {
    let mut t = [handler_addr(default_isr); MAX_VECTORS];

    // System exceptions common to both layouts (the handlers are runtime-hal-provided defaults; the
    // firmware can register the control handler that the ADC IRQ routes to).
    t[2] = handler_addr(nmi_handler); // NMI
    t[3] = handler_addr(hardfault_handler); // HardFault
    t[14] = handler_addr(pendsv_handler); // PendSV
    t[15] = handler_addr(systick_handler); // SysTick

    match layout {
        IrqLayout::F1x0Grouped => {
            // The ADC vector carries the injected-EOC (the control loop), per the reference.
            t[SYSTEM_VECTORS + F1X0_ADC_CMP_IRQ] = handler_addr(adc_isr);
            // The combined advanced-timer vector: ONE demux handler routes break/update/trigger/
            // commutation (HP-4: all four bundled here on F1x0).
            t[SYSTEM_VECTORS + F1X0_TIMER0_BRK_UP_TRG_COM_IRQ] = handler_addr(timer0_grouped_demux);
            t[SYSTEM_VECTORS + F1X0_TIMER0_CHANNEL_IRQ] = handler_addr(timer0_channel_isr);
            // The grouped EXTI lines.
            t[SYSTEM_VECTORS + F1X0_EXTI0_1_IRQ] = handler_addr(exti_isr);
            t[SYSTEM_VECTORS + F1X0_EXTI2_3_IRQ] = handler_addr(exti_isr);
            t[SYSTEM_VECTORS + F1X0_EXTI4_15_IRQ] = handler_addr(exti_isr);
            // The interrupt-buffered RX vector (USART1 = IRQ 28 on F1x0).
            t[SYSTEM_VECTORS + F1X0_USART1_IRQ] = handler_addr(usart1_rx_isr);
            // The link's DMA vector: grouped DMA Ch3/4 = IRQ 11 on F1x0, carrying the RX channel
            // (Ch4) AND the TX channel (Ch3). The slot runs both bodies, each flag-gated on its own
            // channel, so an entry with both flagged services both.
            t[SYSTEM_VECTORS + F1X0_DMA_CH3_4_IRQ] = handler_addr(dma_ch3_4_isr);
            // The THIRD interrupt-buffered RX vector: USART0 on its PB6/PB7 mapping (IRQ 27),
            // F1x0-only (the F10x arm installs no USART0 slot: that mapping needs an AFIO remap).
            t[SYSTEM_VECTORS + F1X0_USART0_IRQ] = handler_addr(usart0_rx_isr);
            // The module port's DMA vector: grouped DMA Ch1/2 = IRQ 10, F1x0-only, carrying USART0's
            // RX channel (Ch2, the THIRD DMA-ring RX) AND its TX channel (Ch1). Both bodies run on
            // every entry, each flag-gated on its own channel, as on the Ch3/4 slot.
            t[SYSTEM_VECTORS + F1X0_DMA_CH1_2_IRQ] = handler_addr(dma_ch1_2_isr);
        }
        IrqLayout::F10xSeparate => {
            t[SYSTEM_VECTORS + F10X_ADC0_1_IRQ] = handler_addr(adc_isr);
            // Separate advanced-timer vectors (HP-4: BRK / UP / TRG+CMT / CH; trigger+commutation
            // share slot 26, so no demux is needed, each slot is a direct handler).
            t[SYSTEM_VECTORS + F10X_TIMER0_BRK_IRQ] = handler_addr(timer0_brk_isr);
            t[SYSTEM_VECTORS + F10X_TIMER0_UP_IRQ] = handler_addr(timer0_up_isr);
            t[SYSTEM_VECTORS + F10X_TIMER0_TRG_CMT_IRQ] = handler_addr(timer0_trg_cmt_isr);
            t[SYSTEM_VECTORS + F10X_TIMER0_CHANNEL_IRQ] = handler_addr(timer0_channel_isr);
            // EXTI: 0..4 individual + two grouped vectors.
            for irq in 6..=10 {
                t[SYSTEM_VECTORS + irq] = handler_addr(exti_isr);
            }
            t[SYSTEM_VECTORS + F10X_EXTI5_9_IRQ] = handler_addr(exti_isr);
            t[SYSTEM_VECTORS + F10X_EXTI10_15_IRQ] = handler_addr(exti_isr);
            // The interrupt-buffered RX vector (USART1 = IRQ 38 on F10x).
            t[SYSTEM_VECTORS + F10X_USART1_IRQ] = handler_addr(usart1_rx_isr);
            // The SECOND interrupt-buffered RX vector: the BLE-module USART (GD USART2 = IRQ 39),
            // F10x-only (the F1x0 arm installs no module-USART slot).
            t[SYSTEM_VECTORS + F10X_USART_MODULE_IRQ] = handler_addr(module_usart_rx_isr);
            // The DMA-ring RX vector: separate DMA0 Ch5 = IRQ 16 on F10x.
            t[SYSTEM_VECTORS + F10X_DMA0_CH5_IRQ] = handler_addr(dma_rx_isr);
            // The DMA transmit vector: separate DMA0 Ch6 = IRQ 17 on F10x (on F1x0 the transmit
            // channel is on the grouped Ch3/4 slot above, which is why there is no F1x0 arm here).
            t[SYSTEM_VECTORS + F10X_DMA0_CH6_IRQ] = handler_addr(dma_tx_isr);
            // The SECOND DMA-ring RX vector: the module USART's DMA channel (GD USART2_RX = DMA0 Ch2 =
            // IRQ 13 on F10x), F10x-only (the F1x0 arm installs no module DMA slot).
            t[SYSTEM_VECTORS + F10X_DMA0_CH2_IRQ] = handler_addr(module_dma_rx_isr);
            // The module USART's DMA TRANSMIT vector (GD USART2_TX = DMA0 Ch1 = IRQ 12 on F10x), its
            // own slot, F10x-only (on F1x0 the module port's transmit is on the grouped Ch1/2 slot).
            t[SYSTEM_VECTORS + F10X_DMA0_CH1_IRQ] = handler_addr(module_dma_tx_isr);
        }
    }

    t
}

/// The numeric address of a handler function, the value a vector-table slot holds. Routing the
/// `fn`-to-`usize` cast through a pointer first is the lint-clean form and makes "this slot holds
/// the address of that handler" explicit. The slot-by-slot tests compare against the same
/// [`handler_addr`] of the expected handler.
#[inline]
pub fn handler_addr(f: Handler) -> usize {
    f as *const () as usize
}

// --- Per-layout ISR bodies --------------------------------------------------------------------
//
// These are the runtime-hal-provided handlers the table slots point at. T8 wires the injected-EOC
// (the ADC ISR's update sub-source) through to the registered control handler; until then they are
// minimal: the ADC ISR and the timer update path call through `call_control_handler`, the rest are
// safe placeholders. No peripheral is enabled here, so on the substrate they never actually fire.

/// The catch-all default ISR for an un-routed slot. A spurious interrupt at an unrouted vector
/// lands here; it is a safe no-op (a real fault path would log, but the substrate keeps it inert).
pub extern "C" fn default_isr() {}

extern "C" fn nmi_handler() {}
extern "C" fn hardfault_handler() {
    // A real build would record the fault; on the substrate, spin so the state is inspectable. The
    // empty spin is intentional for a fault handler (a debugger halt inspects the stacked frame).
    #[allow(clippy::empty_loop)]
    loop {}
}
extern "C" fn pendsv_handler() {}
/// The SysTick exception body in the RAM vector table: route to the HAL's [`on_systick`] (bump the
/// tick count + call the registered tick handler), exactly mirroring how [`adc_isr`] routes to
/// [`call_control_handler`]. A firmware on the cortex-m-rt flash table instead delegates from a
/// one-line `#[exception] SysTick`; both reach the same `on_systick` body.
extern "C" fn systick_handler() {
    on_systick();
}

/// The ADC injected-EOC vector body (F1x0 IRQ 12 / F10x IRQ 18). This is where the control loop
/// runs at the PWM rate (the reference + SPEC.md): it routes to the registered control handler.
///
/// The EOIC flag is cleared by the REGISTERED handler, through
/// [`crate::adc::InjectedHandle::clear_eoic`], not here: this body holds no ADC base (the base is
/// resolved into the handle the bring-up returns), and the handle is the injected group's per-cycle
/// register surface. A registered handler that returns without clearing EOIC re-enters immediately
/// (EOIC is a level source into the NVIC).
extern "C" fn adc_isr() {
    call_control_handler();
}

/// The USART1 RX vector body (F1x0 IRQ 28 / F10x IRQ 38): route to the registered interrupt-buffered
/// RX handler, exactly mirroring how [`adc_isr`] routes to [`call_control_handler`]. The handler
/// (which fills the ring from the USART) lives in [`crate::usart_rx`]; until it is registered this is
/// a safe no-op.
extern "C" fn usart1_rx_isr() {
    call_usart_rx_handler();
    USART1_RX_ISR_METRIC.record();
}

/// The module-USART RX vector body (F10x `USART2_IRQn` = 39): route to the registered second-instance
/// interrupt-buffered RX handler, mirroring [`usart1_rx_isr`]. **F10x-only** (the module USART does
/// not exist on F1x0), so this slot is installed by [`build_table`]'s F10x arm and never its F1x0 arm.
/// Until the handler is registered this is a safe no-op.
extern "C" fn module_usart_rx_isr() {
    call_usart_rx_handler2();
}

/// The USART0 RX vector body (F1x0 `USART0_IRQn` = 27): route to the registered third-instance
/// interrupt-buffered RX handler, mirroring [`usart1_rx_isr`]. **F1x0-only** (on F10x, USART0's
/// PB6/PB7 mapping needs an AFIO remap the HAL does not implement), so this slot is installed by
/// [`build_table`]'s F1x0 arm and never its F10x arm. Until the handler is registered this is a
/// safe no-op.
extern "C" fn usart0_rx_isr() {
    call_usart_rx_handler3();
}

/// The F1x0 grouped DMA vector body (`DMA_Channel1_2_IRQn` = 10), which carries BOTH halves of the
/// module port: `USART0_TX` on Ch1 and `USART0_RX` on Ch2.
///
/// The [`dma_ch3_4_isr`] shape for the same reason: it calls BOTH registered bodies on every entry
/// (the third DMA-ring RX handler, which services only Ch2 and bumps its own wrap counter, and the
/// module port's DMA-TX handler, which services only Ch1 and clears its own in-flight flag), so an
/// entry with both channels flagged services both. **F1x0-only**, installed by [`build_table`]'s
/// F1x0 arm alone. Each body is a no-op until registered.
extern "C" fn dma_ch1_2_isr() {
    call_dma_rx_handler3();
    call_dma_tx_handler2();
}

/// The DMA-RX vector body (F10x `DMA0_Channel5` IRQ 16): route to the registered DMA-ring RX handler
/// ([`crate::dma`]), which reads DMA `INTF` and services only the resolved `USART1_RX` channel. On
/// F10x every DMA channel has its own vector, so this body serves receive alone; the F1x0 grouped
/// slot is [`dma_ch3_4_isr`]. Until that handler is registered this is a safe no-op.
extern "C" fn dma_rx_isr() {
    call_dma_rx_handler();
    DMA_RX_ISR_METRIC.record();
}

/// The F1x0 grouped DMA vector body (`DMA_Channel3_4_IRQn` = 11), which carries BOTH halves of the
/// inter-board link: `USART1_TX` on Ch3 and `USART1_RX` on Ch4.
///
/// It calls BOTH registered bodies on every entry, and each one reads DMA `INTF` and acts only on its
/// own channel's flags. So an entry with one channel flagged services that one, and an entry with
/// BOTH flagged services both. The alternative shape, a demux that inspects `INTF` once and returns
/// after the first channel it matches, passes every single-channel case and silently drops the other
/// channel's event when they coincide: on the transmit side that is an in-flight flag that never
/// clears, i.e. a port that stops transmitting until the force-restart budget expires. Two
/// independent, flag-gated bodies make the both-at-once case the ordinary path rather than a case to
/// remember.
///
/// The metric counts entries into THIS VECTOR, which on F1x0 includes transmit completions; that is
/// what the vector is, and a bench reading the rate is reading the Ch3/4 vector's rate.
extern "C" fn dma_ch3_4_isr() {
    call_dma_rx_handler();
    call_dma_tx_handler();
    DMA_RX_ISR_METRIC.record();
}

/// The DMA-TX vector body (F10x `DMA0_Channel6_IRQn` = 17): route to the registered transmit
/// completion handler ([`crate::dma`]), which services only Ch6's flags. F10x-only as a slot of its
/// own (on F1x0 the transmit channel shares [`dma_ch3_4_isr`]). A no-op until registered.
extern "C" fn dma_tx_isr() {
    call_dma_tx_handler();
}

/// The module DMA-TX vector body (F10x `DMA0_Channel1_IRQn` = 12): route to the registered module-port
/// transmit completion handler ([`crate::dma`]), which services only Ch1's flags. F10x-only as a slot
/// of its own (on F1x0 the module port's transmit shares [`dma_ch1_2_isr`]). A no-op until registered.
extern "C" fn module_dma_tx_isr() {
    call_dma_tx_handler2();
}

/// The module DMA-RX vector body (F10x `DMA0_Channel2_IRQn` = 13): route to the registered second
/// DMA-ring RX handler ([`crate::dma`]), which services ONLY the module channel's (Ch2) `INTF` bits and
/// bumps its own wrap counter, mirroring [`dma_rx_isr`]. **F10x-only** (the module USART does not exist
/// on F1x0), so this slot is installed by [`build_table`]'s F10x arm and never its F1x0 arm (where IRQ
/// 13 is the unrelated grouped advanced-timer vector). Until the handler is registered this is a no-op.
extern "C" fn module_dma_rx_isr() {
    call_dma_rx_handler2();
}

/// The advanced-timer capture/compare-channel vector body (separate on both layouts).
extern "C" fn timer0_channel_isr() {}

/// F10x separate advanced-timer break vector body.
extern "C" fn timer0_brk_isr() {
    grouped_inner::on_break();
}
/// F10x separate advanced-timer update vector body. The update event is the PWM-period boundary; on
/// F10x the control loop is on the ADC vector, so this is the timebase/update path.
extern "C" fn timer0_up_isr() {
    grouped_inner::on_update();
}
/// F10x separate advanced-timer trigger+commutation vector body (HP-4: these share slot 26).
extern "C" fn timer0_trg_cmt_isr() {
    grouped_inner::on_trigger();
    grouped_inner::on_commutation();
}

/// The grouped EXTI vector body (placeholder; the reference reads its rotor-position lines as polled
/// GPIO, not EXTI, per HP-9, so no EXTI line drives the control loop in M3).
extern "C" fn exti_isr() {}

/// The F1x0 GROUPED combined advanced-timer demux (the heart of HP-4). One vector
/// (`TIMER0_BRK_UP_TRG_COM_IRQn` = 13) carries break + update + trigger + commutation; this handler
/// reads TIMER0_INTF and routes each pending sub-source to its inner routine. A demux bug silently
/// drops a sub-source (e.g. the update event), which no register diff would show, so the routing is
/// host-tested directly (`irq/tests.rs`).
///
/// The TIMER0 base is taken from [`grouped_demux_timer_base`], set by the bring-up once the timer is
/// resolved (T3). Until set it is 0 and the demux reads nothing (the substrate is inert).
extern "C" fn timer0_grouped_demux() {
    let base = grouped_demux_timer_base();
    if base == 0 {
        return;
    }
    demux_grouped_timer(base);
}

/// The base address the grouped demux reads TIMER0_INTF from. Set once by the timer bring-up (T3)
/// via [`set_grouped_demux_timer_base`]; 0 until then. An `AtomicU32` so the ISR read is lock-free.
static GROUPED_DEMUX_TIMER_BASE: AtomicU32 = AtomicU32::new(0);

/// Set the TIMER0 base the F1x0 grouped demux reads its INTF from (called by the timer bring-up).
#[inline]
pub fn set_grouped_demux_timer_base(base: u32) {
    GROUPED_DEMUX_TIMER_BASE.store(base, Ordering::Release);
}

/// The TIMER0 base the grouped demux reads (0 if unset).
#[inline]
pub fn grouped_demux_timer_base() -> u32 {
    GROUPED_DEMUX_TIMER_BASE.load(Ordering::Acquire)
}

/// The grouped-demux routing logic, factored out of the ISR so a host test can drive it against the
/// mock register space. Reads TIMER0_INTF at `base` and dispatches each pending sub-source to its
/// inner routine, in a fixed order (break first as the safety-critical one, then update which runs
/// the timebase, then trigger, then commutation). A sub-source whose flag is clear is NOT
/// dispatched, so the demux never invents an event.
pub fn demux_grouped_timer(base: u32) {
    use crate::reg::Reg32;
    let intf = Reg32::new(base, TIMER_INTF).read();
    if intf & INTF_BRKIF != 0 {
        grouped_inner::on_break();
    }
    if intf & INTF_UPIF != 0 {
        grouped_inner::on_update();
    }
    if intf & INTF_TRGIF != 0 {
        grouped_inner::on_trigger();
    }
    if intf & INTF_CMTIF != 0 {
        grouped_inner::on_commutation();
    }
}

/// The inner routines the demux (and the F10x separate handlers) route to. Kept in one module so
/// both layouts share the same sub-source bodies, and so a host test can observe which were called
/// via the test-only call counters (DECISIONS.md #7's demux-routing assertion). On the real build
/// these are minimal: break is the safety kill path (T4 wires the disarm), update is the PWM-period
/// boundary, trigger/commutation are reserved (the reference uses neither).
pub mod grouped_inner {
    #[cfg(feature = "mock")]
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Per-sub-source call counters (mock/host only), so a test asserts the demux routed each
    /// pending sub-source to exactly its routine.
    #[cfg(feature = "mock")]
    pub static BREAK_CALLS: AtomicU32 = AtomicU32::new(0);
    /// Update sub-source call counter (mock/host only); see [`BREAK_CALLS`].
    #[cfg(feature = "mock")]
    pub static UPDATE_CALLS: AtomicU32 = AtomicU32::new(0);
    /// Trigger sub-source call counter (mock/host only); see [`BREAK_CALLS`].
    #[cfg(feature = "mock")]
    pub static TRIGGER_CALLS: AtomicU32 = AtomicU32::new(0);
    /// Commutation sub-source call counter (mock/host only); see [`BREAK_CALLS`].
    #[cfg(feature = "mock")]
    pub static COMMUTATION_CALLS: AtomicU32 = AtomicU32::new(0);

    /// Zero all counters (host test setup).
    #[cfg(feature = "mock")]
    pub fn reset_counts() {
        BREAK_CALLS.store(0, Ordering::SeqCst);
        UPDATE_CALLS.store(0, Ordering::SeqCst);
        TRIGGER_CALLS.store(0, Ordering::SeqCst);
        COMMUTATION_CALLS.store(0, Ordering::SeqCst);
    }

    /// Break sub-source: the hardware-kill / fault path (T4 wires the disarm).
    #[inline]
    pub fn on_break() {
        #[cfg(feature = "mock")]
        BREAK_CALLS.fetch_add(1, Ordering::SeqCst);
    }

    /// Update sub-source: the PWM-period boundary (the timebase tick lives here on the grouped
    /// layout; the PWM-rate control loop is on the ADC injected-EOC vector, per the reference).
    #[inline]
    pub fn on_update() {
        #[cfg(feature = "mock")]
        UPDATE_CALLS.fetch_add(1, Ordering::SeqCst);
    }

    /// Trigger sub-source (reserved; the reference does not use the timer trigger interrupt).
    #[inline]
    pub fn on_trigger() {
        #[cfg(feature = "mock")]
        TRIGGER_CALLS.fetch_add(1, Ordering::SeqCst);
    }

    /// Commutation sub-source (reserved; the reference does software commutation in the loop).
    #[inline]
    pub fn on_commutation() {
        #[cfg(feature = "mock")]
        COMMUTATION_CALLS.fetch_add(1, Ordering::SeqCst);
    }
}

// --- VTOR install (the early-boot handoff, DECISIONS.md #6) ------------------------------------

/// Install the RAM vector table and point `VTOR` at it (DECISIONS.md #6). MUST be called AFTER
/// `.data`/`.bss` init and BEFORE any peripheral interrupt is enabled: the flash table covers reset
/// and the first exceptions, and the post-init flip is the tested handoff. A premature or wrong
/// `VTOR` bricks boot, so this is sequencing-critical.
///
/// `table` is the owned RAM table the caller built with [`build_table`] (stored in the dedicated
/// RAM section). This fills its slots and writes its address to `SCB.VTOR`.
///
/// # Safety
/// The caller must guarantee (a) RAM init is complete, (b) no peripheral IRQ is enabled yet, and
/// (c) `table` lives for the rest of the program (a `'static`), since the hardware will read it on
/// every exception. The table must be aligned per [`RamVectorTable`].
pub unsafe fn install(table: &'static mut RamVectorTable, layout: IrqLayout) {
    table.slots = build_table(layout);
    set_vtor(table.slots.as_ptr() as u32);
}

/// Write `VTOR` (the vector-table offset register, SCB at `0xE000_ED08`). Split out so the host
/// test can stub the SCB write; on the real build it is the cortex-m `SCB::vtor` write.
#[cfg(not(feature = "mock"))]
#[inline]
unsafe fn set_vtor(addr: u32) {
    // SAFETY: VTOR is a single 32-bit SCB register; the caller guarantees the table is aligned and
    // RAM init is complete (the `install` contract).
    let scb = &*cortex_m::peripheral::SCB::PTR;
    scb.vtor.write(addr);
}

/// Mock VTOR: record the written address so the host test can assert the flip happened (and the
/// ordering), since there is no real SCB under `cargo test`.
#[cfg(feature = "mock")]
#[inline]
unsafe fn set_vtor(addr: u32) {
    mock_vtor::set(addr);
}

/// Host-test VTOR shim + a faithful exception-dispatch model (mock feature only). There is no real
/// Cortex-M SCB or NVIC under `cargo test`, so the early-boot handoff (DECISIONS.md #6) is modelled:
/// [`mock_vtor::set`] records the `VTOR` value, and [`mock_vtor::dispatch`] looks up an IRQ's slot in the currently
/// installed table and CALLS it the way the hardware would after `VTOR` points at the RAM table.
/// This exercises the flip ordering and the grouped demux faithfully on the host; the gap vs a real
/// Unicorn exception injection (it cannot drive the NVIC) is noted in `irq/tests.rs`.
#[cfg(feature = "mock")]
pub mod mock_vtor {
    use super::{Handler, MAX_VECTORS, SYSTEM_VECTORS};
    use core::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Mutex;

    /// The recorded `VTOR` value (0 = still on the flash table; non-zero = flipped to RAM).
    static VTOR: AtomicU32 = AtomicU32::new(0);
    /// The currently installed RAM table slots (set when `install` runs under mock), so `dispatch`
    /// can resolve a slot to a handler the way hardware reads `VTOR[slot]`.
    static INSTALLED: Mutex<Option<[usize; MAX_VECTORS]>> = Mutex::new(None);

    /// Record a `VTOR` write (and capture the table the address points into).
    pub fn set(addr: u32) {
        VTOR.store(addr, Ordering::SeqCst);
    }

    /// The recorded `VTOR` value.
    pub fn get() -> u32 {
        VTOR.load(Ordering::SeqCst)
    }

    /// Reset the shim (host test setup): VTOR back to the flash table, no installed RAM table.
    /// Poison-tolerant: a prior `dispatch`-before-flip test panics by design, so recover the guard.
    pub fn reset() {
        VTOR.store(0, Ordering::SeqCst);
        *INSTALLED.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    /// Record the installed RAM table slots (called by the mock `install` path).
    pub fn set_installed(slots: [usize; MAX_VECTORS]) {
        *INSTALLED.lock().unwrap_or_else(|e| e.into_inner()) = Some(slots);
    }

    /// True if `VTOR` has been flipped to a RAM table.
    pub fn is_flipped() -> bool {
        VTOR.load(Ordering::SeqCst) != 0
    }

    /// Dispatch external IRQ `irq` the way hardware would AFTER the flip: resolve the slot in the
    /// installed RAM table and call the handler. Panics if no table is installed (modelling that an
    /// interrupt before the flip would run the FLASH handler, not the RAM one); the test uses
    /// [`is_flipped`] to assert the ordering.
    ///
    /// # Safety
    /// The slot holds a valid `Handler` (built by `build_table`).
    pub unsafe fn dispatch(irq: usize) {
        // Copy the installed table out and DROP the guard before any panic, so a dispatch-before-flip
        // (which panics by design) does not poison the INSTALLED mutex for the next test.
        let maybe = *INSTALLED.lock().unwrap_or_else(|e| e.into_inner());
        let slots = maybe.expect("no RAM table installed: VTOR not flipped yet");
        let idx = SYSTEM_VECTORS + irq;
        let f: Handler = core::mem::transmute::<usize, Handler>(slots[idx]);
        f();
    }

    /// Dispatch a system-exception slot (index 0..15) the way hardware would after the flip.
    ///
    /// # Safety
    /// As [`dispatch`].
    pub unsafe fn dispatch_system(slot: usize) {
        let maybe = *INSTALLED.lock().unwrap_or_else(|e| e.into_inner());
        let slots = maybe.expect("no RAM table installed: VTOR not flipped yet");
        let f: Handler = core::mem::transmute::<usize, Handler>(slots[slot]);
        f();
    }
}

// Under the mock feature, the `install` path also records the installed table for `dispatch`.
#[cfg(feature = "mock")]
/// Build the table for `layout`, record it as installed, and flip `VTOR` to it (the host-test
/// stand-in for the unsafe `install`, modelling the post-RAM-init handoff without a real SCB). The
/// caller asserts `mock_vtor::is_flipped()` and then `mock_vtor::dispatch(..)`.
pub fn install_mock(layout: IrqLayout, ram_table_addr: u32) {
    let slots = build_table(layout);
    mock_vtor::set_installed(slots);
    // Mirror the real ordering: build + record, THEN flip VTOR (a non-zero RAM address).
    unsafe { set_vtor(ram_table_addr) };
}

#[cfg(test)]
mod tests;
