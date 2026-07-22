//! Interrupt-buffered USART receive: [`BufferedRx`] (G-DMA-UART Gate A).
//!
//! The polled USART (`usart.rs`, `serial.rs`) stays the silicon-proven low-rate path; this adds a
//! non-blocking, IDLE-framed RX mode on top of it without spending a DMA channel. The model is
//! embassy-stm32's `BufferedUart` RX: an ISR pushes each `RBNE` byte into a lock-free SPSC ring and
//! the IDLE-line interrupt marks the variable-length frame boundary; the main loop drains the ring.
//!
//! # The shared-ISR ownership problem (and how it is solved)
//!
//! The USART RX vector is an argument-less `extern "C" fn` (it reaches the ISR body via
//! `irq::call_usart_rx_handler`, crate-internal), yet the body needs the USART registers and the
//! ring. The
//! crate already solved this for the control loop (a static handler pointer, DECISIONS.md #7) and the
//! grouped demux (a static base). This module uses the same shape: a `static` RX context (a slot in
//! [`RX_SLOTS`], one per instance) holds the USART base + the family bit + the ring's queue pointer +
//! the monomorphised push, and is inert until [`BufferedRx::new`] installs it. The ISR does NO
//! per-family branching beyond reading the one already-resolved family bit (DECISIONS.md #4).
//!
//! # The ring is HAL-owned ([`RxRing`]), all ops on `&self`
//!
//! The ISR side reaches the ring through a type-erased pointer in the static slot, so the ring's
//! operations must be callable from a SHARED reference reconstructed from that pointer. A
//! third-party queue whose producer/consumer handles must be `split()` out cannot do that without
//! either aliasing `&mut` (UB) or transmuting the crate's PRIVATE handle layout (what this module
//! did before debt-paydown slice 9: `transmute_copy` of a queue pointer into a
//! `heapless::spsc::Producer`, sound only while that crate kept its handle a single pointer). So
//! the HAL owns a minimal SPSC ring instead: head/tail atomics + an `UnsafeCell` buffer, every op
//! (`push`/`pop`/`ready`) on `&self`, the layout OURS. The ISR is the sole pusher and the main
//! loop the sole popper (the SPSC contract); the erasure is a plain `&*(ptr as *const RxRing<N>)`
//! shared-reference reconstruction with no layout assumption beyond our own type.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU8, AtomicUsize, Ordering};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::ClockPath;
// `IrqLayout` is used only by the NVIC-IRQ-number helpers, which are hardware-build-only.
#[cfg(not(feature = "mock"))]
use crate::descriptor::IrqLayout;
use crate::error::{DescriptorError, UsartError};
use crate::irq;
use crate::usart::{UsartRegs, UsartRx};

// --- the static RX contexts (one independent slot per supported instance: USART1 + module) -----

/// Sticky line-error codes stored in [`RxSlot::line_error`] (0 = none). The buffered path surfaces a
/// line error through the next [`BufferedRx::read`], the way the polled path surfaces it inline.
const ERR_NONE: u8 = 0;
const ERR_FRAMING: u8 = 1;
const ERR_PARITY: u8 = 2;
const ERR_OVERRUN: u8 = 3;

/// One static RX context (the bring-up of a [`BufferedRx`] installs one per instance; see
/// [`RX_SLOTS`]). Every field is an atomic, so the whole `static` is `Sync` and the ISR reads it
/// lock-free. Inert (`installed == false`, `base == 0`) until [`BufferedRx::new`] installs it,
/// exactly like the grouped demux's base is 0 until set.
struct RxSlot {
    /// `true` once `new` has installed a context; the ISR is a no-op before this.
    installed: AtomicBool,
    /// The USART base the ISR rebuilds its handle from.
    base: AtomicU32,
    /// Family bit (`true` = F1x0 register model), for the IDLE/error clears.
    is_f1x0: AtomicBool,
    /// The application's `'static` ring, as a type-erased pointer (the producer/consumer are
    /// reconstructed from it; see the module docs).
    queue: AtomicPtr<()>,
    /// The monomorphised `push_impl::<N>`, type-erased as `fn(*mut (), u8) -> bool` (same erase the
    /// control-handler pointer uses).
    push: AtomicPtr<()>,
    /// Sticky ring-full overflow, surfaced as [`UsartError::Overrun`] by the next `read`.
    overflow: AtomicBool,
    /// Set by the ISR on an IDLE boundary, cleared only by [`BufferedRx::take_idle`] (NOT by `read`):
    /// a library-owned latch the caller explicitly consumes, the frame-complete hint.
    idle_seen: AtomicBool,
    /// Sticky line error (see the `ERR_*` codes), surfaced by the next `read`.
    line_error: AtomicU8,
}

impl RxSlot {
    const fn new() -> Self {
        RxSlot {
            installed: AtomicBool::new(false),
            base: AtomicU32::new(0),
            is_f1x0: AtomicBool::new(false),
            queue: AtomicPtr::new(core::ptr::null_mut()),
            push: AtomicPtr::new(core::ptr::null_mut()),
            overflow: AtomicBool::new(false),
            idle_seen: AtomicBool::new(false),
            line_error: AtomicU8::new(ERR_NONE),
        }
    }
}

/// The static RX contexts, one independent slot per supported buffered-RX instance: index 0 = USART1
/// (family-generic), index 1 = the BLE-module USART (HAL [`PeriphLabel::Usart2`], F10x-only). Two
/// [`BufferedRx`] may be live at once (USART1 + module), so the slots must not share state
/// (`uart-rx-multi-instance.md` item 1). Each slot is inert until a `new` installs it.
static RX_SLOTS: [RxSlot; 2] = [RxSlot::new(), RxSlot::new()];

/// Which buffered-RX instance a receiver targets, resolved from the caller's [`PeriphLabel`] selector.
/// Selects the static RX slot, the NVIC vector, and (S2) the DMA channel. The module USART is F10x-only
/// (`uart-rx-multi-instance.md` family scope); any other selector, or the module on F1x0, fails loud in
/// [`resolve_instance`].
#[derive(Clone, Copy, PartialEq, Eq)]
enum RxInstance {
    /// USART1 (HAL [`PeriphLabel::Usart1`], base 0x4000_4400): slot 0, the family-generic first instance.
    Usart1,
    /// The BLE-module USART (HAL [`PeriphLabel::Usart2`], base 0x4000_4800, F10x-only): slot 1.
    Module,
}

impl RxInstance {
    /// The static-slot index for this instance (`RX_SLOTS[..]`).
    #[inline]
    const fn slot_index(self) -> usize {
        match self {
            RxInstance::Usart1 => 0,
            RxInstance::Module => 1,
        }
    }

    /// The `'static` RX slot for this instance.
    #[inline]
    fn slot(self) -> &'static RxSlot {
        &RX_SLOTS[self.slot_index()]
    }
}

/// Resolve a [`PeriphLabel`] selector to a supported [`RxInstance`] for `chip`'s family, fail-loud
/// otherwise (`uart-rx-multi-instance.md` item 4):
/// - `Usart1` -> the family-generic first instance.
/// - `Usart2` on F10x -> the BLE-module instance; on F1x0 -> [`DescriptorError::Unsupported`] (the
///   module USART is F10x-only; no F1x0 board validates its vector/channel).
/// - anything else -> [`DescriptorError::UnknownSelector`].
fn resolve_instance(chip: &Chip, selector: PeriphLabel) -> Result<RxInstance, DescriptorError> {
    match selector {
        PeriphLabel::Usart1 => Ok(RxInstance::Usart1),
        PeriphLabel::Usart2 => match chip.clock() {
            ClockPath::F10xRcc => Ok(RxInstance::Module),
            ClockPath::F1x0Rcu => Err(DescriptorError::Unsupported),
        },
        _ => Err(DescriptorError::UnknownSelector),
    }
}

/// The public RX capability query (`specs/uart-rx-multi-instance.md`, "The public capability
/// query"; mandated by the hoverboard firmware's `specs/l3.md`: capability answers come from the
/// HAL model, never a baked consumer flag): can THIS chip bring up the instance-bound
/// buffered/DMA RX path on `selector`'s USART?
///
/// **Pure** (no register or GPIO access) and answered from the model: it returns exactly
/// [`resolve_instance`]`.is_ok()`, so it can never drift from what [`BufferedRx::new`] /
/// [`RingBufferedRx::new`] will actually accept. Today: `Usart1` -> `true` on both families;
/// `Usart2` -> `true` on F10x, `false` on F1x0 (the module USART is F10x-only); everything else
/// (including `Usart0`) -> `false`. **`Usart0` stays `false` until the AFIO USART0-remap primitive
/// exists** (`specs/usart-pin-remap.md`, out of scope for gate-pin adjacency: its default mapping
/// is PA9/PA10): when that lands, `resolve_instance` grows the arm and this query updates with it
/// automatically - the honest "not yet expressible" answer, owned here, not in a consumer table.
pub fn supports_rx(chip: &Chip, selector: PeriphLabel) -> bool {
    resolve_instance(chip, selector).is_ok()
}

// --- the HAL-owned SPSC ring -------------------------------------------------------------------

/// The application-owned `'static` RX ring a [`BufferedRx`] fills: a minimal lock-free SPSC byte
/// ring, HAL-owned so the ISR-side type erasure rests on OUR layout guarantees, not a third-party
/// crate's private handle layout (debt-paydown slice 9; the predecessor transmuted
/// `heapless::spsc::Producer`'s single-pointer layout).
///
/// Capacity: `N - 1` usable bytes (one slot distinguishes full from empty), matching the previous
/// `heapless` semantics, so existing ring sizings keep their meaning. Single-producer (the RX ISR)
/// / single-consumer (the main loop); every operation takes `&self` (the state lives in the
/// atomics), which is exactly what lets both sides work through shared references reconstructed
/// from a type-erased pointer.
pub struct RxRing<const N: usize> {
    /// Producer index (next write position). Written only by the ISR side.
    head: AtomicUsize,
    /// Consumer index (next read position). Written only by the main-loop side.
    tail: AtomicUsize,
    /// The exclusivity claim: `heapless` carried it in the type (`&'static mut Queue`), but the
    /// `&'static RxRing` ergonomics let safe code hand ONE ring to TWO receivers (two ISR
    /// producers = interleaved corruption), so the claim is a runtime fact instead:
    /// [`BufferedRx::new`] takes it fail-loud, [`BufferedRx::release`] gives it back.
    taken: AtomicBool,
    /// The byte storage. A cell: the producer writes `buf[head]` while the consumer reads
    /// `buf[tail]`, never the same index (guarded by the index protocol below).
    buf: UnsafeCell<[u8; N]>,
}

// SAFETY: all shared mutation goes through the head/tail atomics; the buffer cell is accessed only
// at indices the protocol makes exclusive (producer writes at `head` before publishing it with a
// Release store; the consumer reads at `tail` only after an Acquire load of `head` shows the byte
// published). Single-producer/single-consumer by contract (the ISR vs the owning thread).
unsafe impl<const N: usize> Sync for RxRing<N> {}

impl<const N: usize> RxRing<N> {
    /// Compile-time rejection of degenerate rings: `N = 0` would divide by zero in the index math
    /// and `N = 1` is a silent zero-capacity ring (one slot distinguishes full from empty).
    /// Referenced from [`RxRing::new`], so a bad `N` fails the BUILD of the instantiating crate;
    /// no runtime test is possible or needed.
    const VALID: () = assert!(N >= 2, "RxRing needs N >= 2 (usable capacity is N - 1)");

    /// An empty, unclaimed ring (usable capacity `N - 1`; `N >= 2` enforced at compile time).
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        // Force the compile-time N check at every instantiation site.
        #[allow(clippy::let_unit_value)]
        let () = Self::VALID;
        RxRing {
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            taken: AtomicBool::new(false),
            buf: UnsafeCell::new([0; N]),
        }
    }

    /// Claim exclusive receiver ownership of this ring: `true` exactly once until
    /// [`RxRing::unclaim`]. The type-discipline replacement for `&'static mut` (see the `taken`
    /// field docs).
    fn claim(&self) -> bool {
        !self.taken.swap(true, Ordering::AcqRel)
    }

    /// Release the receiver claim (the ring may be re-claimed by a fresh receiver).
    fn unclaim(&self) {
        self.taken.store(false, Ordering::Release);
    }

    /// Producer side (the RX ISR): append one byte; `false` if the ring is full.
    fn push(&self, b: u8) -> bool {
        let head = self.head.load(Ordering::Relaxed); // own index
        let next = (head + 1) % N;
        if next == self.tail.load(Ordering::Acquire) {
            return false; // full: overwriting would corrupt the unread byte at `tail`
        }
        // SAFETY: `head` is exclusively the producer's write slot until the Release store below
        // publishes it; the consumer never reads past the published head.
        unsafe { (*self.buf.get())[head] = b };
        self.head.store(next, Ordering::Release);
        true
    }

    /// Consumer side (the main loop): take the oldest byte, if any.
    fn pop(&self) -> Option<u8> {
        let tail = self.tail.load(Ordering::Relaxed); // own index
        if tail == self.head.load(Ordering::Acquire) {
            return None; // empty
        }
        // SAFETY: `head != tail`, so `buf[tail]` was fully written before the producer's Release
        // store of `head` (paired with the Acquire load above); the producer never writes at `tail`.
        let b = unsafe { (*self.buf.get())[tail] };
        self.tail.store((tail + 1) % N, Ordering::Release);
        Some(b)
    }

    /// True if at least one byte is buffered. Consumes nothing.
    fn ready(&self) -> bool {
        self.tail.load(Ordering::Acquire) != self.head.load(Ordering::Acquire)
    }
}

// --- monomorphised ring access (the type erasure boundary) ------------------------------------

/// Push one byte into the ring (the ISR side). Installed into the slot as `push_impl::<N>`.
fn push_impl<const N: usize>(q: *mut (), b: u8) -> bool {
    // SAFETY: `q` is the `&'static RxRing<N>` pointer `new::<N>` stored (valid for 'static); a
    // shared reference to a `Sync` type, no layout assumption beyond our own.
    let ring: &RxRing<N> = unsafe { &*(q as *const RxRing<N>) };
    ring.push(b)
}

/// Pop one byte from the ring (the main-loop side).
fn pop_impl<const N: usize>(q: *mut ()) -> Option<u8> {
    // SAFETY: as `push_impl`.
    let ring: &RxRing<N> = unsafe { &*(q as *const RxRing<N>) };
    ring.pop()
}

/// True if the ring has at least one buffered byte.
fn ready_impl<const N: usize>(q: *mut ()) -> bool {
    // SAFETY: as `push_impl`.
    let ring: &RxRing<N> = unsafe { &*(q as *const RxRing<N>) };
    ring.ready()
}

/// Release the ring's exclusivity claim ([`BufferedRx::release`]).
fn unclaim_impl<const N: usize>(q: *mut ()) {
    // SAFETY: as `push_impl`.
    let ring: &RxRing<N> = unsafe { &*(q as *const RxRing<N>) };
    ring.unclaim();
}

// --- the ISR bodies (one per instance, each reached via its own vector slot) -------------------

/// The registered ISR body for USART1 (slot 0). Reached from the `usart1_rx_isr` vector slot through
/// `irq::call_usart_rx_handler` (crate-internal).
extern "C" fn rx_irq_handler() {
    on_usart_rx_irq(&RX_SLOTS[0]);
}

/// The registered ISR body for the module USART (slot 1, F10x-only). Reached from the
/// `module_usart_rx_isr` vector slot through the crate-internal `irq::call_usart_rx_handler2`.
/// Independent of
/// [`rx_irq_handler`] so the two instances never touch each other's slot (item 1/2 coexistence).
extern "C" fn module_rx_irq_handler() {
    on_usart_rx_irq(&RX_SLOTS[1]);
}

/// The interrupt-buffered RX ISR logic, factored out so a host test drives it against the mock
/// register space (the [`crate::irq::mock_vtor::dispatch`] path runs this via the registered handler).
///
/// Per the spec §4.2: read STAT once; clear+record any line error; drain every ready `RBNE` byte
/// into the ring (flag overflow if full); then clear+flag IDLE. The drain is a loop, not a single
/// read, matching the FIFO-less discipline the polled path documents.
fn on_usart_rx_irq(slot: &RxSlot) {
    if !slot.installed.load(Ordering::Acquire) {
        return;
    }
    let u = UsartRegs::from_parts(
        slot.base.load(Ordering::Acquire),
        slot.is_f1x0.load(Ordering::Acquire),
    );

    // Line error (overrun / framing / parity): record sticky + clear the family-correct way so it
    // cannot latch and strand RX (the regression the polled path fixed, carried over).
    let st = u.read_status();
    if let Some(e) = st.line_error() {
        record_line_error(slot, e);
        u.clear_line_errors(&st);
    }

    // Drain every byte that is ready this entry.
    loop {
        if !u.read_status().rx_ready {
            break;
        }
        let byte = u.read_rbne_byte();
        if !push_byte(slot, byte) {
            slot.overflow.store(true, Ordering::Release);
            break;
        }
    }

    // IDLE boundary: clear it the family-correct way and flag it for the reader.
    if u.idle_flag() {
        u.clear_idle();
        slot.idle_seen.store(true, Ordering::Release);
    }
}

/// Record a line error stickily, keeping the FIRST one seen (precedence matches
/// [`crate::usart::Status::line_error`]: overrun, then framing, then parity).
fn record_line_error(slot: &RxSlot, e: UsartError) {
    let code = match e {
        UsartError::Overrun => ERR_OVERRUN,
        UsartError::Framing => ERR_FRAMING,
        UsartError::Parity => ERR_PARITY,
        _ => ERR_NONE,
    };
    if code != ERR_NONE {
        let _ =
            slot.line_error
                .compare_exchange(ERR_NONE, code, Ordering::AcqRel, Ordering::Relaxed);
    }
}

/// Push through the slot's installed (type-erased) producer.
fn push_byte(slot: &RxSlot, b: u8) -> bool {
    let q = slot.queue.load(Ordering::Acquire);
    let p = slot.push.load(Ordering::Acquire);
    if q.is_null() || p.is_null() {
        return false;
    }
    // SAFETY: `p` is the `push_impl::<N>` fn pointer `new` stored, erased as `*mut ()` exactly the
    // way the irq.rs handler pointers are (DECISIONS.md #7: a fn pointer round-trips through the
    // AtomicPtr, our own erasure, no foreign layout involved); `q` is the matching ring pointer.
    let push = unsafe { core::mem::transmute::<*mut (), fn(*mut (), u8) -> bool>(p) };
    push(q, b)
}

fn decode_error(code: u8) -> UsartError {
    match code {
        ERR_FRAMING => UsartError::Framing,
        ERR_PARITY => UsartError::Parity,
        _ => UsartError::Overrun,
    }
}

// --- shared RX-context install + NVIC unmask --------------------------------------------------

/// Install the static USART RX context and register the shared `usart1_rx_isr` body. Both
/// [`BufferedRx`] (with a real ring `queue`/`push`) and [`RingBufferedRx`] (with null `queue`/`push`:
/// under DMA the ISR's RBNE drain never runs, only its IDLE + line-error path) use this, so the IDLE
/// latch and line-error sticky live in one place (section 3.2 / 5.3).
fn install_usart_ctx(regs: &UsartRegs, instance: RxInstance, queue: *mut (), push: *mut ()) {
    let slot = instance.slot();
    slot.base.store(regs.base(), Ordering::Release);
    slot.is_f1x0.store(regs.is_f1x0(), Ordering::Release);
    slot.queue.store(queue, Ordering::Release);
    slot.push.store(push, Ordering::Release);
    slot.overflow.store(false, Ordering::Release);
    slot.idle_seen.store(false, Ordering::Release);
    slot.line_error.store(ERR_NONE, Ordering::Release);
    slot.installed.store(true, Ordering::Release);
    // Register the ISR body for THIS instance's vector (each instance has its own handler pair, so the
    // two slots never collide). The module USART is F10x-only, so its handler only ever fires there.
    match instance {
        RxInstance::Usart1 => irq::register_usart_rx_handler(rx_irq_handler),
        RxInstance::Module => irq::register_usart_rx_handler2(module_rx_irq_handler),
    }
}

/// The USART RX IRQ number for `chip`'s family + `instance`. The vector differs by family for USART1
/// (28 on F1x0, 38 on F10x) and is the F10x-only module vector (39) for the module instance (the module
/// USART cannot exist on F1x0, so its IRQ is unconditionally the F10x one). Hardware-build-only (drives
/// the NVIC).
#[cfg(not(feature = "mock"))]
#[inline]
fn usart_irq_num(chip: &Chip, instance: RxInstance) -> usize {
    match instance {
        RxInstance::Usart1 => match chip.irq() {
            IrqLayout::F1x0Grouped => irq::F1X0_USART1_IRQ,
            IrqLayout::F10xSeparate => irq::F10X_USART1_IRQ,
        },
        // `resolve_instance` only yields `Module` on F10x, so its vector is always the F10x module IRQ.
        RxInstance::Module => irq::F10X_USART_MODULE_IRQ,
    }
}

/// The DMA-RX IRQ number for `chip`'s family + `instance`. USART1's channel: separate `DMA0_Channel5`
/// = 16 on F10x, grouped `DMA_Channel3_4` = 11 on F1x0. The module's channel: F10x-only `DMA0_Channel2`
/// = 13 (`resolve_instance` only yields `Module` on F10x, so its vector is unconditionally that one).
/// Hardware-build-only (drives the NVIC).
#[cfg(not(feature = "mock"))]
#[inline]
fn dma_irq_num(chip: &Chip, instance: RxInstance) -> usize {
    match instance {
        RxInstance::Usart1 => match chip.irq() {
            IrqLayout::F1x0Grouped => irq::F1X0_DMA_CH3_4_IRQ,
            IrqLayout::F10xSeparate => irq::F10X_DMA0_CH5_IRQ,
        },
        RxInstance::Module => irq::F10X_DMA0_CH2_IRQ,
    }
}

// --- NVIC interrupt number (hardware build only) ----------------------------------------------

/// A device IRQ number as a [`cortex_m::interrupt::InterruptNumber`] for [`NVIC::unmask`]. This is the
/// first place the crate drives the NVIC (the firmware owns enabling its interrupts).
#[cfg(not(feature = "mock"))]
#[derive(Clone, Copy)]
struct IrqNum(u16);

#[cfg(not(feature = "mock"))]
// SAFETY: the value is a GD SPL `IRQn_Type` number (USART1 / DMA channel), a valid device IRQ number.
unsafe impl cortex_m::interrupt::InterruptNumber for IrqNum {
    #[inline]
    fn number(self) -> u16 {
        self.0
    }
}

/// Unmask a device IRQ in the NVIC. Hardware only: there is no NVIC under `cargo test`, where the host
/// suite fires the ISR via `mock_vtor::dispatch` instead.
#[cfg(not(feature = "mock"))]
#[inline]
fn nvic_unmask(irq_num: usize) {
    // SAFETY: the RAM vector table routing this IRQ is the caller's pre-`new` responsibility; unmasking
    // now is what first allows the IRQ to fire.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(IrqNum(irq_num as u16));
    }
}

// --- the public type --------------------------------------------------------------------------

/// Interrupt-buffered, IDLE-framed USART receiver (G-DMA-UART Gate A).
///
/// Built from the RX half of a split [`crate::usart::Usart`] (consuming it: the RX path now owns
/// that instance's RX interrupt; the TX half stays with its owner, `specs/usart-split.md` D3). The
/// ISR fills a `'static` SPSC ring the application owns (DECISIONS.md #10: buffers are application
/// code, not a HAL default); [`read`](BufferedRx::read) drains it non-blocking. No DMA channel is
/// spent, so this is the cheap option for moderate-rate framed-protocol RX. The RX half is given
/// back by [`release`](BufferedRx::release).
pub struct BufferedRx {
    /// The application ring, type-erased (the consumer is reconstructed from this per `read`).
    queue: *mut (),
    /// Monomorphised pop / ready / claim-release for this ring's `N` (concrete fn pointers; no
    /// erasure needed since they live in non-generic fields set by the generic `new`).
    pop: fn(*mut ()) -> Option<u8>,
    ready: fn(*mut ()) -> bool,
    unclaim: fn(*mut ()),
    /// This receiver's `'static` RX slot (USART1 or the module USART): `read` / `take_idle` reference
    /// it directly, so two `BufferedRx` on different instances never touch each other's state.
    slot: &'static RxSlot,
    /// The consumed RX half, held for [`release`](BufferedRx::release).
    rx: UsartRx,
}

impl BufferedRx {
    /// Install interrupt-buffered RX on the RX half of a brought-up USART, using `storage` as the
    /// `'static` ring.
    ///
    /// Performs, in order: install the static RX context (USART base + family bit + the ring); enable
    /// `RBNEIE` + `IDLEIE` on the USART; register the ISR body; then unmask the USART IRQ in the NVIC.
    /// The application must have flipped `VTOR` to the RAM table (the [`crate::irq::install`] contract)
    /// BEFORE calling this, since `new` is the thing that enables the IRQ the table must already
    /// route. `N` is the ring capacity word (the ring holds `N - 1` bytes, [`RxRing`]).
    pub fn new<const N: usize>(
        chip: &Chip,
        rx: UsartRx,
        instance: PeriphLabel,
        storage: &'static RxRing<N>,
    ) -> Result<BufferedRx, DescriptorError> {
        // Resolve the instance selector to a slot + vector (fail-loud on F1x0 module / unknown label),
        // before touching any hardware (`uart-rx-multi-instance.md` item 4).
        let inst = resolve_instance(chip, instance)?;

        // Exclusivity claim (fail loud): one ring, one receiver. Two receivers pushing into one
        // ring from two ISRs is interleaved corruption; heapless carried this in `&'static mut`,
        // the RxRing carries it as a runtime claim released by `release()`. A double-claim is a
        // programming error, so it panics (the `Usart::rejoin` mismatch precedent).
        assert!(
            storage.claim(),
            "RxRing is already claimed by another receiver (one ring, one BufferedRx)"
        );

        let queue = storage as *const RxRing<N> as *mut ();

        // 1. Install this instance's RX slot (base + family bit + this ring's queue + monomorphised
        //    push) and register the ISR body the instance's vector routes to.
        install_usart_ctx(&rx.regs(), inst, queue, push_impl::<N> as *mut ());

        // 2. Enable the RX interrupt sources on the USART.
        rx.regs().listen_rbne();
        rx.regs().listen_idle();

        // 3. Unmask this instance's USART IRQ in the NVIC (hardware only).
        let _ = chip;
        #[cfg(not(feature = "mock"))]
        nvic_unmask(usart_irq_num(chip, inst));

        Ok(BufferedRx {
            queue,
            pop: pop_impl::<N>,
            ready: ready_impl::<N>,
            unclaim: unclaim_impl::<N>,
            slot: inst.slot(),
            rx,
        })
    }

    /// Quiesce this receiver and give the RX half back (`specs/usart-split.md` D3): clear the
    /// `RBNEIE` + `IDLEIE` interrupt sources it enabled, mark its RX slot uninstalled (a pending
    /// IRQ finds an inert slot), and RELEASE the ring's exclusivity claim (the [`RxRing`] may be
    /// re-claimed by a fresh receiver). The returned [`UsartRx`] re-arms via a fresh `new`, or
    /// rejoins its TX half ([`crate::usart::Usart::rejoin`]) for reconfiguration.
    pub fn release(self) -> UsartRx {
        self.rx.regs().unlisten_rx_irqs();
        self.slot.installed.store(false, Ordering::Release);
        (self.unclaim)(self.queue);
        self.rx
    }

    /// Drain buffered bytes into `buf`; returns the count (0 if the ring is currently empty,
    /// non-blocking). A sticky condition is surfaced FIRST as an `Err`, then cleared, so the next
    /// `read` resumes draining:
    /// - [`UsartError::Overrun`]: the ring filled (a byte was dropped) or the USART raised `ORERR`.
    /// - [`UsartError::Framing`] / [`UsartError::Parity`]: a line error the ISR cleared so it cannot
    ///   latch.
    ///
    /// `Ok(0)` means "nothing buffered right now", never an error (matching the `try_read_byte`
    /// contract that the empty case is not a failure).
    ///
    /// `read` does NOT touch the IDLE latch: the frame boundary is consumed only by
    /// [`take_idle`](BufferedRx::take_idle), so a drain-then-check poll loop cannot race the boundary
    /// away (the caller owns when the latch is consumed, not the byte path).
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError> {
        let slot = self.slot;

        // Surface a sticky condition first (overflow as Overrun, then any line error), clearing it.
        if slot.overflow.swap(false, Ordering::AcqRel) {
            return Err(UsartError::Overrun);
        }
        let code = slot.line_error.swap(ERR_NONE, Ordering::AcqRel);
        if code != ERR_NONE {
            return Err(decode_error(code));
        }

        let mut n = 0;
        while n < buf.len() {
            match (self.pop)(self.queue) {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        Ok(n)
    }

    /// True if at least one byte is buffered (the `embedded-io` `ReadReady` shape).
    pub fn ready(&self) -> bool {
        (self.ready)(self.queue)
    }

    /// True if a sticky condition (ring overflow or a recorded line error) is pending, i.e. the next
    /// [`read`](BufferedRx::read) will surface-and-clear it rather than return bytes. The serial
    /// adapter's `ReadReady` counts this as "a read makes progress" (`specs/serial-adapters.md` D4).
    pub(crate) fn condition_pending(&self) -> bool {
        self.slot.overflow.load(Ordering::Acquire)
            || self.slot.line_error.load(Ordering::Acquire) != ERR_NONE
    }

    /// Atomically read-and-clear the IDLE latch: returns `true` exactly once per IDLE boundary the
    /// ISR has flagged since the previous `take_idle`, and consumes it. This is the frame-complete
    /// hint, and it is the ONLY thing that clears the latch ([`read`](BufferedRx::read) does not), so
    /// the natural pattern is sound without any read-gating discipline by the caller:
    ///
    /// ```ignore
    /// let n = rx.read(&mut buf[len..])?;   // drain whatever is buffered
    /// len += n;
    /// if rx.take_idle() && len > 0 {       // boundary consumed: this frame is complete
    ///     // ... handle the frame ...
    /// }
    /// ```
    ///
    /// Because the latch persists across `read` calls until taken, an IDLE that the ISR serviced in
    /// the same entry as the final bytes (or between two polls) is still observed on the next
    /// `take_idle`, not silently eaten.
    pub fn take_idle(&self) -> bool {
        self.slot.idle_seen.swap(false, Ordering::AcqRel)
    }
}

// --- DMA-ring mode: RingBufferedRx (circular DMA + IDLE) --------------------------------------

/// DMA-ring USART receiver (G-DMA-UART Gate B): a circular DMA continuously refills a `'static` byte
/// buffer while the CPU does no per-byte work; [`read`](RingBufferedRx::read) drains bytes behind the
/// live DMA write position (`len - CHxCNT`). The IDLE boundary (shared USART ISR) and lap-overrun are
/// surfaced explicitly. This is the low-CPU high-rate mode; [`take_idle`](RingBufferedRx::take_idle)
/// gives the same frame-complete hint as [`BufferedRx`].
pub struct RingBufferedRx {
    map: crate::dma::DmaRxMap,
    /// The application-owned `'static` DMA buffer (the DMA writes it; the CPU only reads bytes strictly
    /// behind the live write index, section 6).
    buf: *mut u8,
    len: usize,
    /// Monotonic count of bytes the application has consumed (the read cursor). `% len` is the buffer
    /// position; comparing it to `wraps * len + (len - CHxCNT)` detects a lapped (overwritten) cursor.
    cursor: u64,
    /// This receiver's shared USART RX slot (USART1 or the module): `read` reads its sticky line error,
    /// `take_idle` its IDLE latch. Two `RingBufferedRx` on different instances never touch each other's.
    slot: &'static RxSlot,
    /// This receiver's DMA-RX wrap-counter context index (`crate::dma::wraps`), independent per instance.
    dma_ctx: usize,
    /// The consumed RX half, held for [`release`](RingBufferedRx::release).
    rx: UsartRx,
}

impl RingBufferedRx {
    /// Arm circular DMA RX on the RX half of a brought-up USART, writing into `dma_buf` (section 5.1). In order:
    /// resolve [`DmaRxMap`](crate::dma::DmaRxMap); enable the DMA clock; **write-back self-check** (fail
    /// loud if the channel does not respond, arming nothing); program the channel (PADDR = RDATA,
    /// MADDR = buf, CNT = len, circular, half+full IRQ) and start it (`CHEN` last, after a Release
    /// fence); on the USART set `DENR` + `IDLEIE` + `ERRIE`; install the shared RX context + the DMA-RX
    /// context; unmask both IRQs. The caller must have flipped `VTOR` to the RAM table first.
    ///
    /// `dma_buf` length must fit `CHxCNT` (<= 65535) and be non-empty; an out-of-range length is a
    /// [`DescriptorError::Unsupported`]. A failed self-check is [`DescriptorError::SelfCheckFailed`].
    pub fn new(
        chip: &Chip,
        rx: UsartRx,
        instance: PeriphLabel,
        dma_buf: &'static mut [u8],
    ) -> Result<RingBufferedRx, DescriptorError> {
        // Resolve the instance selector to a slot + DMA channel + vectors (fail-loud on F1x0 module /
        // unknown label) before touching hardware (`uart-rx-multi-instance.md` items 3-4).
        let inst = resolve_instance(chip, instance)?;

        let len = dma_buf.len();
        if len == 0 || len > u16::MAX as usize {
            return Err(DescriptorError::Unsupported);
        }
        let buf = dma_buf.as_mut_ptr();

        // 1. Resolve the DMA mapping for this instance: USART1's channel is family-generic; the module's
        //    is the F10x-only GD `USART2_RX` channel (DMA0 Ch2).
        let map = match inst {
            RxInstance::Usart1 => crate::dma::DmaRxMap::usart1_rx(chip),
            RxInstance::Module => crate::dma::DmaRxMap::module_rx(),
        };

        // 2. Enable the DMA controller clock.
        crate::clock::enable_dma(chip.rcu_base()?);

        // 2a. Stop any channel that may still be running (re-arming after a prior `new`): a live channel
        //     must not be poked by the self-check's `CHxMADDR` write, which would briefly point its
        //     writes at the sentinel address. Disable it first; the line is quiescent at re-arm time.
        map.disable();

        // 3. Write-back self-check (fail-loud): confirm the resolved channel responds before arming.
        //    NOTE: the self-check proves the channel RESPONDS, not that it is the one the USART feeds;
        //    only the live bench loopback confirms the resolved channel mapping (a wrong channel passes
        //    this but receives zero bytes).
        if !map.self_check() {
            return Err(DescriptorError::SelfCheckFailed);
        }

        // 4-5. Program + start the channel (CHEN last, Release fence inside).
        // SAFETY: `buf` points at the caller's `'static` `dma_buf` of `len` bytes, which outlives the
        // channel (the `&'static mut` contract); the DMA writes it while we only read behind its head.
        unsafe { map.configure_circular(rx.regs().rdata_addr(), buf as u32, len as u16) };

        // 6. USART: route bytes to DMA, raise the IDLE boundary, raise line errors.
        rx.regs().enable_dma_rx();
        rx.regs().listen_idle();
        rx.regs().listen_errors();

        // Install this instance's shared RX context (no SPSC ring under DMA: the ISR's RBNE drain never
        // runs, since the DMA controller's RDATA read auto-clears RBNE; only its IDLE + line-error path
        // is used) and the per-instance DMA-RX wrap-counter context + DMA ISR body.
        install_usart_ctx(
            &rx.regs(),
            inst,
            core::ptr::null_mut(),
            core::ptr::null_mut(),
        );
        crate::dma::install(&map, inst.slot_index());

        // 7. Unmask both the DMA-channel and the USART IRQ for this instance (hardware only).
        let _ = chip;
        #[cfg(not(feature = "mock"))]
        {
            nvic_unmask(dma_irq_num(chip, inst));
            nvic_unmask(usart_irq_num(chip, inst));
        }

        Ok(RingBufferedRx {
            map,
            buf,
            len,
            cursor: 0,
            slot: inst.slot(),
            dma_ctx: inst.slot_index(),
            rx,
        })
    }

    /// Quiesce this receiver and give the RX half back (`specs/usart-split.md` D3): disable the DMA
    /// channel, clear the `DENR` + `IDLEIE` + `ERRIE` sources it enabled, and mark its RX slot
    /// uninstalled. The returned [`UsartRx`] re-arms via a fresh `new`, or rejoins its TX half
    /// ([`crate::usart::Usart::rejoin`]) for reconfiguration (the bench's baud-change path).
    pub fn release(self) -> UsartRx {
        self.map.disable();
        self.rx.regs().disable_dma_rx();
        self.slot.installed.store(false, Ordering::Release);
        self.rx
    }

    /// A consistent snapshot of (effective wrap count, remaining) for the live DMA write position.
    ///
    /// Two hazards at a circular wrap, both handled:
    /// 1. A wrap the ISR HAS counted landing mid-snapshot: re-read the wrap counter around the rest and
    ///    retry on a change (`w1 == w2`).
    /// 2. A wrap the ISR has NOT yet counted: at the wrap, hardware reloads `CHxCNT` to `len` and sets
    ///    the channel's `FTFIF`, but the wrap-counter ISR runs slightly later. If the snapshot reads the
    ///    reloaded `CHxCNT == len` with the old wrap count, `write_total` would undercount by `len`,
    ///    underflow `available`, and report a SPURIOUS overrun though the cursor was never lapped. So the
    ///    snapshot also reads the pending `FTFIF` and attributes that wrap (`+1`) to the position. The
    ///    `CHxCNT` re-read (`rem_b <= rem_a`) rejects a reload that slipped in after the flag read.
    #[inline]
    fn snapshot(&self) -> (u32, u16) {
        loop {
            let w1 = crate::dma::wraps(self.dma_ctx);
            let rem_a = self.map.remaining();
            let ftf = self.map.ftf_pending();
            let rem_b = self.map.remaining();
            let w2 = crate::dma::wraps(self.dma_ctx);
            if w1 != w2 {
                continue; // a counted wrap landed during the snapshot: retry
            }
            if ftf {
                // An uncounted wrap is pending; `rem_b` (read after the flag) reflects the new lap.
                return (w1 + 1, rem_b);
            }
            if rem_b <= rem_a {
                // No wrap in progress (`CHxCNT` only counted down): consistent.
                return (w1, rem_b);
            }
            // `CHxCNT` increased (a reload) but the flag was read clear just before it: retry so the
            // next pass observes `FTFIF` set and attributes the wrap.
        }
    }

    /// Drain bytes that have arrived since the last read (between the cursor and the live DMA write
    /// position `len - CHxCNT`), into `buf`; returns the count, 0 if none (non-blocking).
    ///
    /// Both loss conditions are **recoverable in place** with the DMA channel left LIVE (the caller
    /// drops the lost bytes and keeps reading, it does NOT re-arm), but they are DISTINCT error values
    /// so a diagnostic can classify them (section 5.4, the self-heal default an always-on framed link
    /// needs: a transient line disturbance must not strand the receiver):
    /// - A **USART line error** ([`UsartError::LineError`], the `ERRIE` path recorded by the shared
    ///   ISR: overrun / framing / noise) is a transient wire glitch on an always-on link (a peer
    ///   rebooting, a cable connecting, the peer re-initialising its USART, all of which momentarily
    ///   disturb the line). The shared ISR already cleared the hardware flag; `read` drops the
    ///   disturbed bytes by resyncing the cursor to the live write position and leaves the channel
    ///   running, so the [`crate::serial::SplitSerial`]/`StreamFramer` above resyncs on the next frame
    ///   boundary and the link self-heals with no reset. A PERSISTENT bad line is a protocol-layer
    ///   concern (it shows up upstream as the link's `comms_loss`), NOT a disabled peripheral.
    /// - A **lapped cursor** ([`UsartError::RingOverrun`], the DMA wrote more than `len` bytes ahead of
    ///   the cursor: the consumer fell behind and unread bytes were overwritten) likewise resyncs to
    ///   the freshest position and leaves the channel live (section 5.2).
    #[inline]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, UsartError> {
        // Line error (the ERRIE path): SELF-HEAL in place, do NOT disable the channel. The shared ISR
        // already cleared the hardware flag; drop the disturbed bytes by resyncing the cursor to the
        // live DMA write position and keep the channel running, exactly as the lap-overrun path below.
        // Returns the DISTINCT `LineError` (not `RingOverrun`) so a diagnostic can tell a wire glitch
        // from a slow-consumer lap. A transient line glitch (a peer rebooting / a cable connecting)
        // must not strand an always-on link; a persistent bad line surfaces upstream as `comms_loss`,
        // never a dead peripheral. Reads THIS instance's shared RX slot (independent of the module's).
        if self.slot.line_error.swap(ERR_NONE, Ordering::AcqRel) != ERR_NONE {
            let (wraps, rem) = self.snapshot();
            let write_total = wraps as u64 * self.len as u64 + (self.len as u64 - rem as u64);
            self.cursor = write_total; // drop the disturbed bytes; resume from the live position
            return Err(UsartError::LineError);
        }

        let (wraps, rem) = self.snapshot();
        // Live monotonic write position: full laps + this lap's progress (CHxCNT counts down from len).
        let write_total = wraps as u64 * self.len as u64 + (self.len as u64 - rem as u64);
        let available = write_total - self.cursor;

        // Lap-overrun: the head passed the cursor by more than a full buffer (section 5.2). Recoverable
        // in place (the channel stays live), so it is `RingOverrun`, NOT the channel-disabling
        // `Overrun` the ERRIE path above returns.
        if available > self.len as u64 {
            self.cursor = write_total; // drop the overwritten data; resume from the live position
            return Err(UsartError::RingOverrun);
        }

        let n = core::cmp::min(available as usize, buf.len());
        // Section 6: the buffer reads must not be hoisted before the CHxCNT snapshot above.
        core::sync::atomic::compiler_fence(Ordering::Acquire);
        // ONE u64 modulo per CALL, then a conditional-reset wrap per byte. The previous per-byte
        // `(cursor + i) % len` was a software `__aeabi_uldivmod` on every byte, and on the M3 that
        // per-byte division was the dominant term of the flood-drain cost (round-4 slice-1 PC
        // profile: 35% of CPU in u64 division; the per-call hoist removes all but one).
        let mut pos = (self.cursor % self.len as u64) as usize;
        let len = self.len;
        for slot in buf.iter_mut().take(n) {
            // SAFETY: `pos < len`, within the `'static` buffer; the byte is strictly behind the live
            // DMA write index, so it is fully written and not being overwritten now (section 6).
            *slot = unsafe { core::ptr::read_volatile(self.buf.add(pos)) };
            pos += 1;
            if pos == len {
                pos = 0;
            }
        }
        self.cursor += n as u64;
        Ok(n)
    }

    /// Atomically read-and-clear the IDLE latch (the frame-complete hint), exactly as
    /// [`BufferedRx::take_idle`]: the IDLE boundary is delivered by the shared USART IRQ (section 5.3),
    /// so the DMA-ring reader uses the same latch to know a variable-length frame just ended.
    #[inline]
    pub fn take_idle(&self) -> bool {
        self.slot.idle_seen.swap(false, Ordering::AcqRel)
    }

    /// **Validation hook**: inject one line error into this receiver's RX slot EXACTLY as the shared
    /// ERRIE ISR records a real one ([`record_line_error`], into `slot.line_error`), so the next
    /// [`read`](RingBufferedRx::read) surfaces [`UsartError::LineError`] and self-heals in place (the
    /// cursor resync in `read`, channel left LIVE) -- the identical recovery path a real wire glitch
    /// takes. It fabricates NO DMA state: only the sticky slot byte the ISR itself sets; `read`'s own
    /// snapshot handles the cursor. One call injects one error (a real error already pending is kept,
    /// per `record_line_error`'s first-wins). This is the controlled-injection stimulus the firmware's
    /// Gate-1 UART-RX-self-heal sign-off drives over SWD (poke a flag, observe `line_errors` increment
    /// while the framed link stays live); it is spec'd, permanent, and drives the shipping self-heal
    /// path rather than a stand-in.
    #[inline]
    pub fn inject_line_error(&self) {
        record_line_error(self.slot, UsartError::Overrun);
    }

    /// True if the next [`read`](RingBufferedRx::read) will make progress: bytes are available
    /// behind the live DMA write position (the same wrap-consistent [`snapshot`](Self::snapshot)
    /// `read` uses, so the B13 pending-wrap case never reads false-negative), or a pending
    /// condition (a recorded line error, or a lap) that `read` would surface-and-clear. Consumes
    /// nothing (the cursor does not advance). The `embedded-io` `ReadReady` shape
    /// (`specs/serial-adapters.md` D4).
    #[inline]
    pub fn ready(&self) -> bool {
        if self.slot.line_error.load(Ordering::Acquire) != ERR_NONE {
            return true;
        }
        let (wraps, rem) = self.snapshot();
        let write_total = wraps as u64 * self.len as u64 + (self.len as u64 - rem as u64);
        write_total > self.cursor
    }
}

/// Reset every static RX slot and both registered handlers (host-test teardown between cases). Both
/// instances' slots are reset so a case that installed the module slot cannot leak into the next.
#[cfg(feature = "mock")]
pub fn reset_for_test() {
    for slot in &RX_SLOTS {
        slot.installed.store(false, Ordering::Release);
        slot.base.store(0, Ordering::Release);
        slot.is_f1x0.store(false, Ordering::Release);
        slot.queue.store(core::ptr::null_mut(), Ordering::Release);
        slot.push.store(core::ptr::null_mut(), Ordering::Release);
        slot.overflow.store(false, Ordering::Release);
        slot.idle_seen.store(false, Ordering::Release);
        slot.line_error.store(ERR_NONE, Ordering::Release);
    }
    irq::clear_usart_rx_handler();
    irq::clear_usart_rx_handler2();
}

#[cfg(test)]
mod tests;
