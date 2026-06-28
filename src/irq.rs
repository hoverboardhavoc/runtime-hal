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
//!   Highest external IRQ = 73, so the table is 16 system + 74 IRQ = 90 entries.
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

/// External IRQ count the table reserves. Sized to cover BOTH families: F1x0's highest external
/// IRQ is `CAN1_SCE_IRQn` = 73 (so 74 IRQ slots), which also covers F10x (highest 59 on MD/HD).
/// Over-provisioning is fine (DECISIONS.md / the milestone's "over-provision is fine").
pub const MAX_IRQS: usize = 74;

/// Total vector-table entries (system + external IRQ).
pub const MAX_VECTORS: usize = SYSTEM_VECTORS + MAX_IRQS;

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
/// F1x0 `DMA_Channel3_4_IRQn` (`gd32f1x0.h:143`): channels 3 and 4 SHARE one vector (grouped layout),
/// so `USART1_RX` (Ch4 on F1x0) arrives here alongside Ch3 and the handler demuxes by reading DMA
/// `INTF` (the DMA twin of the grouped advanced-timer vector).
pub const F1X0_DMA_CH3_4_IRQ: usize = 11;

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
pub fn call_usart_rx_handler() {
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
pub fn call_usart_rx_handler2() {
    let p = USART_RX_HANDLER2.load(Ordering::Acquire);
    // SAFETY: `p` is always either `default_usart_rx_handler2` or a `'static` fn the RX bring-up
    // registered via `register_usart_rx_handler2`; both are valid `extern "C" fn()` pointers.
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

// --- The owned RAM vector table (DECISIONS.md #6) ---------------------------------------------

/// The owned RAM vector table (DECISIONS.md #6): an alignment-correct `static` in a dedicated
/// section. The Cortex-M `VTOR` requires the table to be aligned to a power of two at least as large
/// as the table: `MAX_VECTORS * 4 = 360` bytes rounds up to **512**, so the table is `align(512)` (the
/// `align(R) >= TBLOFF` rule, the exact power-of-two granule).
///
/// Aligning to 512 (not 1024) reclaims 1 KiB of RAM from the universal firmware's single
/// `RamVectorTable` static (its `VECTORS`): 512 B because the struct size is padded to its alignment
/// (1024 -> 512), plus 512 B because the linker no longer pads the preceding RAM up to a 1024-byte
/// boundary. That 1 KiB goes to the 8 KiB-RAM image's stack, which the deep `store` flash-write path
/// needs (a too-small stack overflows into `.bss` and corrupts the ISR state). It is one half of a
/// **two-part** stack-overflow fix that together keep the peak under the region; neither is sufficient
/// alone: this `align(512)` (region 3004 -> 4028 B) AND the firmware's `store::MAX_RECORD = 272`
/// (peak 4820 -> 3044 B). (The detect-probe table is a SEPARATE `align(1024)` `AlignedVectorTable` in
/// [`crate::detect`], unaffected by this.) `#[no_mangle]` + the section let the linker place it in RAM;
/// the flash table still covers reset + the first exceptions until [`install`] flips `VTOR`.
#[repr(C, align(512))]
pub struct RamVectorTable {
    /// The vector slots (slot 0 reserved for the initial SP value the hardware reads; slots 1..15
    /// the system exceptions; 16.. the external IRQs).
    pub slots: [usize; MAX_VECTORS],
}

// VTOR alignment guard: on the 32-bit target each slot is 4 bytes, and the table base must be aligned
// to a power of two >= its byte size. `align(512)` is valid only while the table fits in 512 bytes
// (<= 128 slots); if MAX_VECTORS ever exceeds that, bump align(..) to the next power of two or VTOR
// would reject the (then-unaligned) table base. (Asserted on the slot COUNT so it is the same on the
// 32-bit firmware target and the 64-bit host test build.)
const _: () = assert!(
    MAX_VECTORS <= 128,
    "RamVectorTable exceeds its 512-byte VTOR alignment; bump align(..) to the next power of two"
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
            // The DMA-ring RX vector: grouped DMA Ch3/4 = IRQ 11 on F1x0 (the handler demuxes Ch4).
            t[SYSTEM_VECTORS + F1X0_DMA_CH3_4_IRQ] = handler_addr(dma_rx_isr);
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
            // The SECOND DMA-ring RX vector: the module USART's DMA channel (GD USART2_RX = DMA0 Ch2 =
            // IRQ 13 on F10x), F10x-only (the F1x0 arm installs no module DMA slot).
            t[SYSTEM_VECTORS + F10X_DMA0_CH2_IRQ] = handler_addr(module_dma_rx_isr);
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
/// runs at the PWM rate (the reference + SPEC.md). T8 enables the injected group and this body
/// clears the EOIC flag then calls the registered control handler; the T2 substrate just routes to
/// the registered handler so the wiring is in place.
extern "C" fn adc_isr() {
    call_control_handler();
}

/// The USART1 RX vector body (F1x0 IRQ 28 / F10x IRQ 38): route to the registered interrupt-buffered
/// RX handler, exactly mirroring how [`adc_isr`] routes to [`call_control_handler`]. The handler
/// (which fills the ring from the USART) lives in [`crate::usart_rx`]; until it is registered this is
/// a safe no-op.
extern "C" fn usart1_rx_isr() {
    call_usart_rx_handler();
}

/// The module-USART RX vector body (F10x `USART2_IRQn` = 39): route to the registered second-instance
/// interrupt-buffered RX handler, mirroring [`usart1_rx_isr`]. **F10x-only** (the module USART does
/// not exist on F1x0), so this slot is installed by [`build_table`]'s F10x arm and never its F1x0 arm.
/// Until the handler is registered this is a safe no-op.
extern "C" fn module_usart_rx_isr() {
    call_usart_rx_handler2();
}

/// The DMA-RX vector body (F10x `DMA0_Channel5` IRQ 16 / F1x0 grouped `DMA_Channel3_4` IRQ 11): route
/// to the registered DMA-ring RX handler ([`crate::dma`]), which reads DMA `INTF` and services only the
/// resolved `USART1_RX` channel (the demux that ignores the other grouped channel on F1x0). Until that
/// handler is registered this is a safe no-op.
extern "C" fn dma_rx_isr() {
    call_dma_rx_handler();
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
