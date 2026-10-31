//! Host tests for the RAM vector table + registration + grouped demux (M3 T2).
//!
//! Validates DECISIONS.md #6 (table flip + VTOR set after RAM init) and #7 (static handler
//! registration with a no-op default guarding the pre-registration window), and the HP-4 grouped
//! demux routing, exactly the checks TESTING.md "Interrupts: RAM vector-table correctness" calls
//! for: slot-by-slot, exception-after-flip, and the combined-demux sub-source routing.
//!
//! # Unicorn gap (recorded, per the milestone)
//!
//! A real Unicorn exception injection would set VTOR via the emulated SCB and pend an IRQ via the
//! NVIC so the CPU vectors through the RAM table on hardware semantics. Unicorn is the QEMU CPU core
//! with NO peripheral models (TESTING.md): it has no NVIC and cannot pend/take an external
//! interrupt, so it cannot drive the vector fetch. The harness extractor (`harness/regcmp`) is
//! built to TRACE MMIO writes from a snippet to a sentinel return, not to take interrupts. So the
//! exception-after-flip is modelled HOST-SIDE here: `install_mock` records the installed RAM table
//! and flips a recorded VTOR, and `mock_vtor::dispatch` resolves an IRQ's slot in the installed
//! table and calls it the way hardware reads `VTOR[slot]` after the flip. This faithfully exercises
//! the SEQUENCING (a dispatch before the flip has no installed table and panics, modelling "the
//! flash handler, not the RAM one, would run") and the slot->handler resolution; the gap vs real
//! silicon is that the host model does not exercise the CPU's actual vector fetch / stacking, only
//! the table contents + flip ordering. That gap is covered on-silicon implicitly once T8+ enables
//! the injected-EOC IRQ on the bench.

#![cfg(feature = "mock")]

use super::grouped_inner;
use super::{
    build_table, call_control_handler, clear_control_handler, clear_tick_count, clear_tick_handler,
    default_isr, handler_addr, install_mock, mock_vtor, on_systick, register_control_handler,
    register_tick_handler, set_grouped_demux_timer_base, tick_count, ControlHandler, TickHandler,
    F10X_ADC0_1_IRQ, F10X_DMA0_CH1_IRQ, F10X_DMA0_CH2_IRQ, F10X_DMA0_CH5_IRQ, F10X_DMA0_CH6_IRQ,
    F10X_EXTI10_15_IRQ, F10X_EXTI5_9_IRQ, F10X_TIMER0_BRK_IRQ, F10X_TIMER0_CHANNEL_IRQ,
    F10X_TIMER0_TRG_CMT_IRQ, F10X_TIMER0_UP_IRQ, F1X0_ADC_CMP_IRQ, F1X0_EXTI0_1_IRQ,
    F1X0_EXTI2_3_IRQ, F1X0_EXTI4_15_IRQ, F1X0_TIMER0_BRK_UP_TRG_COM_IRQ, F1X0_TIMER0_CHANNEL_IRQ,
    INTF_BRKIF, INTF_CMTIF, INTF_TRGIF, INTF_UPIF, MAX_VECTORS, SYSTEM_VECTORS, TIMER_INTF,
};
use super::{
    DMA_RX_ISR_METRIC, F10X_DMA0_CH3_IRQ, F10X_DMA0_CH4_IRQ, F10X_USART0_IRQ, F10X_USART2_IRQ,
    F1X0_DMA_CH1_2_IRQ, F1X0_DMA_CH3_4_IRQ, F1X0_USART0_IRQ, F1X0_USART1_IRQ, SYSTICK_ISR_METRIC,
    USART1_RX_ISR_METRIC,
};
use crate::addr::PeriphLabel;
use crate::descriptor::IrqLayout;
use crate::reg::{mock, Reg32};
use core::sync::atomic::{AtomicU32, Ordering};

const TIMER0_BASE: u32 = 0x4001_2C00;

fn slot(table: &[usize; MAX_VECTORS], irq: usize) -> usize {
    table[SYSTEM_VECTORS + irq]
}

// --- Slot-by-slot, F1x0 grouped (cross-checked against the GD SPL IRQn_Type) ------------------

#[test]
fn f1x0_grouped_slots_match_spl_layout() {
    let t = build_table(IrqLayout::F1x0Grouped);

    // The ADC vector (IRQ 12 = ADC_CMP) carries the injected-EOC / control loop.
    assert_eq!(slot(&t, F1X0_ADC_CMP_IRQ), handler_addr(super::adc_isr));
    // The combined advanced-timer vector (IRQ 13) is the grouped demux (all four sub-sources).
    assert_eq!(
        slot(&t, F1X0_TIMER0_BRK_UP_TRG_COM_IRQ),
        handler_addr(super::timer0_grouped_demux)
    );
    // The advanced-timer channel vector (IRQ 14) is separate even on the grouped layout.
    assert_eq!(
        slot(&t, F1X0_TIMER0_CHANNEL_IRQ),
        handler_addr(super::timer0_channel_isr)
    );
    // The three grouped EXTI vectors.
    for irq in [F1X0_EXTI0_1_IRQ, F1X0_EXTI2_3_IRQ, F1X0_EXTI4_15_IRQ] {
        assert_eq!(slot(&t, irq), handler_addr(super::exti_isr));
    }

    // The link's DMA vector (IRQ 11, grouped Ch3/4) is the slot that runs BOTH halves' bodies:
    // receive on Ch4 and transmit on Ch3 share it, so it is not the F10x receive-only body.
    assert_eq!(
        slot(&t, F1X0_DMA_CH3_4_IRQ),
        handler_addr(super::dma_ch3_4_isr)
    );
    assert_ne!(
        slot(&t, F1X0_DMA_CH3_4_IRQ),
        handler_addr(super::dma_rx_isr),
        "the grouped slot must not be the receive-only body: it would drop every transmit completion"
    );
    // F10x's separate transmit vector (IRQ 17) is not routed here: on F1x0 that number is the
    // unrelated TIMER1 vector and the transmit channel is on the grouped slot above.
    assert_eq!(slot(&t, F10X_DMA0_CH6_IRQ), handler_addr(default_isr));

    // USART0's DMA vector (IRQ 10, grouped Ch1/2) is the same two-body shape: USART0's receive on
    // Ch2 and its transmit on Ch1 share it, so the slot must not be a receive-only body.
    assert_eq!(
        slot(&t, F1X0_DMA_CH1_2_IRQ),
        handler_addr(super::dma_ch1_2_isr)
    );
    // USART0's own RX vector is IRQ 27 here.
    assert_eq!(
        slot(&t, F1X0_USART0_IRQ),
        handler_addr(super::usart0_rx_isr)
    );
    // The F10x-only bodies (USART2's three vectors, USART0's separate DMA vectors) must not appear
    // anywhere in this table: on this layout their F10x numbers are other vectors (12 = ADC,
    // 14 = TIMER0 channel, 13 = the grouped timer demux).
    for (body, name) in [
        (handler_addr(super::usart2_rx_isr), "USART2 RX"),
        (handler_addr(super::usart2_dma_rx_isr), "USART2 DMA RX"),
        (handler_addr(super::usart2_dma_tx_isr), "USART2 DMA TX"),
        (
            handler_addr(super::usart0_dma_rx_isr),
            "USART0 DMA RX (Ch4)",
        ),
        (
            handler_addr(super::usart0_dma_tx_isr),
            "USART0 DMA TX (Ch3)",
        ),
    ] {
        assert!(
            !t.iter().any(|&s| s == body),
            "the F10x {name} body must not appear in the F1x0 grouped table"
        );
    }

    // A slot the layout does not route (the table's last, an unused gap) stays the default ISR. This is
    // the "misplaced slot points at the wrong handler" guard: only the named slots diverge.
    assert_eq!(slot(&t, super::MAX_IRQS - 1), handler_addr(default_isr));
    // The F10x-only ADC slot (18) is NOT routed on F1x0 (its ADC is at 12).
    assert_eq!(slot(&t, F10X_ADC0_1_IRQ), handler_addr(default_isr));
}

// --- Slot-by-slot, F10x separate (HP-4: BRK/UP/TRG+CMT/CH distinct, no demux) -----------------

#[test]
fn f10x_separate_slots_match_spl_layout() {
    let t = build_table(IrqLayout::F10xSeparate);

    assert_eq!(slot(&t, F10X_ADC0_1_IRQ), handler_addr(super::adc_isr));
    // The four separate advanced-timer vectors. HP-4: trigger+commutation share slot 26, so it is a
    // single direct handler, NOT a demux.
    assert_eq!(
        slot(&t, F10X_TIMER0_BRK_IRQ),
        handler_addr(super::timer0_brk_isr)
    );
    assert_eq!(
        slot(&t, F10X_TIMER0_UP_IRQ),
        handler_addr(super::timer0_up_isr)
    );
    assert_eq!(
        slot(&t, F10X_TIMER0_TRG_CMT_IRQ),
        handler_addr(super::timer0_trg_cmt_isr)
    );
    assert_eq!(
        slot(&t, F10X_TIMER0_CHANNEL_IRQ),
        handler_addr(super::timer0_channel_isr)
    );
    // EXTI: lines 0..4 individual, plus the two grouped vectors.
    for irq in 6..=10 {
        assert_eq!(slot(&t, irq), handler_addr(super::exti_isr));
    }
    assert_eq!(slot(&t, F10X_EXTI5_9_IRQ), handler_addr(super::exti_isr));
    assert_eq!(slot(&t, F10X_EXTI10_15_IRQ), handler_addr(super::exti_isr));

    // IRQ 13 differs by family: on F1x0 it is the grouped TIMER0_BRK_UP_TRG_COM demux, but on F10x it
    // is DMA0_Channel2 = USART2's DMA-ring RX vector. The advanced timer does NOT use slot 13 on
    // F10x; USART2's DMA channel does.
    assert_eq!(
        F10X_DMA0_CH2_IRQ, F1X0_TIMER0_BRK_UP_TRG_COM_IRQ,
        "both are IRQ 13"
    );
    assert_eq!(
        slot(&t, F10X_DMA0_CH2_IRQ),
        handler_addr(super::usart2_dma_rx_isr)
    );
    // The link's two DMA channels have a vector each on this layout: receive on Ch5 (IRQ 16),
    // transmit on Ch6 (IRQ 17), no demux between them.
    assert_eq!(slot(&t, F10X_DMA0_CH5_IRQ), handler_addr(super::dma_rx_isr));
    assert_eq!(slot(&t, F10X_DMA0_CH6_IRQ), handler_addr(super::dma_tx_isr));
    // USART2's transmit (GD USART2_TX) is DMA0 Ch1, its own vector (IRQ 12), next to its receive on
    // Ch2 (IRQ 13) above, and its RX vector is 39.
    assert_eq!(
        slot(&t, F10X_DMA0_CH1_IRQ),
        handler_addr(super::usart2_dma_tx_isr)
    );
    assert_eq!(
        slot(&t, F10X_USART2_IRQ),
        handler_addr(super::usart2_rx_isr)
    );
    // USART0 on its remapped PB6/PB7: RX vector 37, DMA receive Ch4 (IRQ 15) and transmit Ch3
    // (IRQ 14), each its own slot (`gd32f10x.h`: USART0_IRQn = 37, DMA0_Channel3/4_IRQn = 14/15).
    assert_eq!(
        slot(&t, F10X_USART0_IRQ),
        handler_addr(super::usart0_rx_isr)
    );
    assert_eq!(
        slot(&t, F10X_DMA0_CH4_IRQ),
        handler_addr(super::usart0_dma_rx_isr)
    );
    assert_eq!(
        slot(&t, F10X_DMA0_CH3_IRQ),
        handler_addr(super::usart0_dma_tx_isr)
    );
    // The grouped Ch3/4 and Ch1/2 bodies belong to the other layout and must not appear here.
    for (grouped_dma, name) in [
        (handler_addr(super::dma_ch3_4_isr), "Ch3/4"),
        (handler_addr(super::dma_ch1_2_isr), "Ch1/2"),
    ] {
        assert!(
            !t.iter().any(|&s| s == grouped_dma),
            "the grouped {name} DMA body must not appear in the F10x separate table"
        );
    }
    // The grouped demux is never installed on the separate layout.
    let demux = handler_addr(super::timer0_grouped_demux);
    assert!(
        !t.iter().any(|&s| s == demux),
        "the grouped demux must not appear in the F10x separate table"
    );
}

#[test]
fn system_exception_slots_are_filled_on_both_layouts() {
    for layout in [IrqLayout::F1x0Grouped, IrqLayout::F10xSeparate] {
        let t = build_table(layout);
        assert_eq!(t[2], handler_addr(super::nmi_handler), "NMI");
        assert_eq!(t[3], handler_addr(super::hardfault_handler), "HardFault");
        assert_eq!(t[14], handler_addr(super::pendsv_handler), "PendSV");
        assert_eq!(t[15], handler_addr(super::systick_handler), "SysTick");
    }
}

// --- Static handler registration: no-op default before, registered after (DECISIONS.md #7) ----

static TEST_HANDLER_CALLS: AtomicU32 = AtomicU32::new(0);
extern "C" fn test_control_handler() {
    TEST_HANDLER_CALLS.fetch_add(1, Ordering::SeqCst);
}

#[test]
fn registration_swaps_the_noop_default_for_the_registered_handler() {
    let _serial = mock::lock();
    clear_control_handler();
    TEST_HANDLER_CALLS.store(0, Ordering::SeqCst);

    // Before registration: the ISR path calls the no-op default (the pre-registration guard). The
    // test handler's counter must NOT move.
    call_control_handler();
    call_control_handler();
    assert_eq!(
        TEST_HANDLER_CALLS.load(Ordering::SeqCst),
        0,
        "before registration the no-op default runs, not the firmware handler"
    );

    // After registration: the ISR path calls through to the registered handler.
    register_control_handler(test_control_handler as ControlHandler);
    call_control_handler();
    call_control_handler();
    call_control_handler();
    assert_eq!(
        TEST_HANDLER_CALLS.load(Ordering::SeqCst),
        3,
        "after registration the firmware control handler runs"
    );

    clear_control_handler();
}

/// The tick-seam test handler's call counter (mirrors `TEST_HANDLER_CALLS`).
static TEST_TICK_CALLS: AtomicU32 = AtomicU32::new(0);

extern "C" fn test_tick_handler() {
    TEST_TICK_CALLS.fetch_add(1, Ordering::SeqCst);
}

#[test]
fn tick_seam_registration_and_dispatch_through_on_systick() {
    // The G7 tick seam end to end (the integration firmware's SysTick wiring: it registers its
    // scheduler tick through this seam because `install()` flips VTOR to the RAM table, whose
    // slot-15 `systick_handler` reaches `on_systick`; a firmware-side `#[exception] SysTick`
    // on the flash table would be dead code after the flip).
    let _serial = mock::lock();
    clear_tick_handler();
    clear_tick_count();
    TEST_TICK_CALLS.store(0, Ordering::SeqCst);

    // Before registration: `on_systick` (the single body every SysTick route reaches) bumps the
    // free-running count and calls the no-op default; the pre-registration window is safe.
    on_systick();
    on_systick();
    assert_eq!(tick_count(), 2, "the free-running count always advances");
    assert_eq!(
        TEST_TICK_CALLS.load(Ordering::SeqCst),
        0,
        "before registration the no-op default runs, not the firmware tick"
    );

    // After registration (the firmware's boot-time act): every tick reaches the handler AND the
    // count keeps advancing (the handler is additive, not a replacement).
    register_tick_handler(test_tick_handler as TickHandler);
    on_systick();
    on_systick();
    on_systick();
    assert_eq!(
        TEST_TICK_CALLS.load(Ordering::SeqCst),
        3,
        "after registration the firmware tick handler runs once per tick"
    );
    assert_eq!(tick_count(), 5);

    clear_tick_handler();
    clear_tick_count();
}

// --- The exception-after-flip sequencing test (DECISIONS.md #6) -------------------------------

#[test]
fn ram_handler_runs_after_vtor_flip_not_before() {
    let _serial = mock::lock();
    mock_vtor::reset();
    clear_control_handler();
    TEST_HANDLER_CALLS.store(0, Ordering::SeqCst);
    register_control_handler(test_control_handler as ControlHandler);

    // Before the flip: VTOR is still on the flash table (recorded as 0), no RAM table installed.
    assert!(
        !mock_vtor::is_flipped(),
        "VTOR must still point at the flash table before install (the reset/early-exception window)"
    );

    // Flip: build the F1x0 table, record it installed, set VTOR to the RAM address (modelling
    // `install` running AFTER RAM init). A non-zero RAM table address stands in for the section.
    let ram_addr = 0x2000_4000u32;
    install_mock(IrqLayout::F1x0Grouped, ram_addr);
    assert!(
        mock_vtor::is_flipped(),
        "VTOR flipped to the RAM table after install"
    );
    assert_eq!(mock_vtor::get(), ram_addr);

    // Raise the ADC injected-EOC exception (IRQ 12 on F1x0): the RAM table's adc_isr runs, which
    // calls through to the registered control handler. This is the RAM handler running, not flash.
    unsafe { mock_vtor::dispatch(F1X0_ADC_CMP_IRQ) };
    assert_eq!(
        TEST_HANDLER_CALLS.load(Ordering::SeqCst),
        1,
        "the RAM table's ADC ISR ran the registered control handler after the flip"
    );

    mock_vtor::reset();
    clear_control_handler();
}

#[test]
#[should_panic(expected = "VTOR not flipped")]
fn dispatch_before_flip_has_no_ram_table() {
    let _serial = mock::lock();
    mock_vtor::reset();
    // Dispatching before any install: there is no RAM table, modelling that the FLASH table (not a
    // RAM handler) would service the exception. The model panics rather than running a RAM handler,
    // which is the sequencing assertion: nothing in RAM is live until the flip.
    unsafe { mock_vtor::dispatch(F1X0_ADC_CMP_IRQ) };
}

// --- The grouped demux routes each sub-source to the right inner routine (HP-4) ----------------

#[test]
fn grouped_demux_routes_each_pending_subsource() {
    let _serial = mock::lock();
    mock::reset();
    grouped_inner::reset_counts();
    set_grouped_demux_timer_base(TIMER0_BASE);

    // All four sub-source flags pending in INTF: break + update + trigger + commutation.
    Reg32::new(TIMER0_BASE, TIMER_INTF).write(INTF_BRKIF | INTF_UPIF | INTF_TRGIF | INTF_CMTIF);
    super::demux_grouped_timer(TIMER0_BASE);
    assert_eq!(grouped_inner::BREAK_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(grouped_inner::UPDATE_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(grouped_inner::TRIGGER_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(grouped_inner::COMMUTATION_CALLS.load(Ordering::SeqCst), 1);
}

#[test]
fn grouped_demux_does_not_invent_a_cleared_subsource() {
    let _serial = mock::lock();
    mock::reset();
    grouped_inner::reset_counts();

    // Only the UPDATE flag pending (the sub-source that runs the timebase). A demux bug that ran
    // others, or dropped update, is exactly the silent-drop class TESTING.md warns about.
    Reg32::new(TIMER0_BASE, TIMER_INTF).write(INTF_UPIF);
    super::demux_grouped_timer(TIMER0_BASE);
    assert_eq!(
        grouped_inner::UPDATE_CALLS.load(Ordering::SeqCst),
        1,
        "update routed"
    );
    assert_eq!(
        grouped_inner::BREAK_CALLS.load(Ordering::SeqCst),
        0,
        "break not invented"
    );
    assert_eq!(
        grouped_inner::TRIGGER_CALLS.load(Ordering::SeqCst),
        0,
        "trigger not invented"
    );
    assert_eq!(
        grouped_inner::COMMUTATION_CALLS.load(Ordering::SeqCst),
        0,
        "commutation not invented"
    );

    // Only BREAK pending (the safety-critical sub-source): only break routes.
    grouped_inner::reset_counts();
    Reg32::new(TIMER0_BASE, TIMER_INTF).write(INTF_BRKIF);
    super::demux_grouped_timer(TIMER0_BASE);
    assert_eq!(grouped_inner::BREAK_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(grouped_inner::UPDATE_CALLS.load(Ordering::SeqCst), 0);
}

#[test]
fn table_size_covers_both_families() {
    // 64 entries = 256 B at a 256 B VTOR alignment, with no padding. The part's highest external
    // IRQ (73, CAN1_SCE) is NOT covered on purpose: only an unmasked IRQ can fire, the highest the
    // HAL installs is 40, and the const asserts beside MAX_IRQS hold every named vector inside.
    assert_eq!(MAX_VECTORS, SYSTEM_VECTORS + super::MAX_IRQS);
    assert_eq!(MAX_VECTORS, 16 + 48);
    assert_eq!(core::mem::align_of::<super::RamVectorTable>(), 256);
    // On the 32-bit target the struct is exactly its alignment; on the 64-bit host each slot is 8 B.
    assert_eq!(
        core::mem::size_of::<super::RamVectorTable>(),
        (MAX_VECTORS * core::mem::size_of::<usize>()).next_multiple_of(256)
    );
}

// --- Planted-bug self-test: a misplaced vector-table slot is caught (M3 T12) -------------------
//
// The slot-by-slot tests above prove the built table is correct, but the planted-bug discipline
// (mirroring the harness trace self-tests) demands we also prove a CORRUPTED table is FLAGGED, so a
// silently-wrong slot cannot pass. A misplaced vector slot is a boot brick or, worse, the event that
// runs the control loop routed to the wrong handler (the half-bridge keeps the last duties with no
// ISR re-arming or disarming it). This perturbs exactly one slot and asserts a slot-by-slot diff
// against the golden table flags that one slot and only it; the unperturbed control run is clean.

fn slot_diff(golden: &[usize; MAX_VECTORS], live: &[usize; MAX_VECTORS]) -> SlotDiff {
    let mut diffs = 0usize;
    let mut first = usize::MAX;
    for i in 0..MAX_VECTORS {
        if golden[i] != live[i] {
            diffs += 1;
            if first == usize::MAX {
                first = i;
            }
        }
    }
    SlotDiff { diffs, first }
}

struct SlotDiff {
    diffs: usize,
    first: usize,
}

#[test]
fn misplaced_vector_slot_is_flagged() {
    for layout in [IrqLayout::F1x0Grouped, IrqLayout::F10xSeparate] {
        let golden = build_table(layout);

        // Control run: a freshly built table matches the golden slot-by-slot (no diffs).
        let clean = build_table(layout);
        let cr = slot_diff(&golden, &clean);
        assert_eq!(cr.diffs, 0, "the unperturbed table must match slot-by-slot");

        // Plant the bug: point the ADC slot (the injected-EOC / control-loop vector) at the wrong
        // handler (the timer channel ISR). On silicon this drops the control loop while the bridge
        // stays driven.
        let adc_irq = match layout {
            IrqLayout::F1x0Grouped => F1X0_ADC_CMP_IRQ,
            IrqLayout::F10xSeparate => F10X_ADC0_1_IRQ,
        };
        let mut bad = build_table(layout);
        let bad_slot = SYSTEM_VECTORS + adc_irq;
        assert_eq!(bad[bad_slot], handler_addr(super::adc_isr));
        bad[bad_slot] = handler_addr(super::timer0_channel_isr);

        let cr = slot_diff(&golden, &bad);
        assert_eq!(
            cr.diffs, 1,
            "exactly the misplaced slot must diverge ({layout:?})"
        );
        assert_eq!(
            cr.first, bad_slot,
            "the flagged slot must be the misplaced one"
        );
    }
}

// --- Per-vector ISR entry counting (permanent observability) ----------------------------------

#[test]
fn isr_metrics_count_entries_per_vector() {
    // The permanent CTRL_OBS instrumentation seam: each instrumented vector body bumps its own
    // entry counter exactly once per invocation, and the three counters are independent. Entry
    // counts alone are what discriminates a per-byte storm from the expected IDLE/wrap rate on
    // silicon (only entries are published; below-pass cycle attribution is not a trusted
    // observable). Deltas (not absolutes) under the serial lock, since the metrics are process-wide.
    let _serial = mock::lock();

    // SysTick: on_systick is the single body every SysTick route reaches; it must record one entry.
    let s0 = SYSTICK_ISR_METRIC.entries();
    on_systick();
    on_systick();
    assert_eq!(
        SYSTICK_ISR_METRIC.entries().wrapping_sub(s0),
        2,
        "SysTick metric bumps once per tick body"
    );

    // USART1 RX + DMA RX: dispatch their vectors through the installed F1x0 RAM table the way
    // hardware would after the VTOR flip, and assert only the dispatched vector's counter moved.
    install_mock(IrqLayout::F1x0Grouped, 0x2000_4000);
    let u0 = USART1_RX_ISR_METRIC.entries();
    let d0 = DMA_RX_ISR_METRIC.entries();

    unsafe { mock_vtor::dispatch(F1X0_USART1_IRQ) };
    assert_eq!(
        USART1_RX_ISR_METRIC.entries().wrapping_sub(u0),
        1,
        "the USART1 RX vector records one entry"
    );
    assert_eq!(
        DMA_RX_ISR_METRIC.entries().wrapping_sub(d0),
        0,
        "dispatching USART1 RX must not touch the DMA metric"
    );

    let d1 = DMA_RX_ISR_METRIC.entries();
    unsafe { mock_vtor::dispatch(F1X0_DMA_CH3_4_IRQ) };
    assert_eq!(
        DMA_RX_ISR_METRIC.entries().wrapping_sub(d1),
        1,
        "the DMA RX vector records one entry"
    );

    mock_vtor::reset();
}

// --- R7: NVIC / SysTick interrupt-priority configuration --------------------------------------

use super::{
    apply_motor_era_priorities, irq_priority, period_irq, set_irq_priority, set_systick_priority,
    systick_priority, usart_irqs, UsartIrqs, DMA_RX_PRIORITY, F10X_ADC0_1_IRQ as R7_F10X_ADC,
    F10X_USART1_IRQ, MOTOR_ISR_PRIORITY, NVIC_IPR_BASE, SCB_SHPR3, SYSTICK_PRIORITY,
    USART_RX_PRIORITY,
};

/// The priority levels are strictly ordered (lower numeric = higher urgency): period ISR ABOVE
/// USART RX ABOVE DMA ABOVE SysTick, the execution-model ordering the IPR read-back gate checks.
#[test]
fn r7_priority_constants_strictly_ordered() {
    assert!(MOTOR_ISR_PRIORITY < USART_RX_PRIORITY);
    assert!(USART_RX_PRIORITY < DMA_RX_PRIORITY);
    assert!(DMA_RX_PRIORITY < SYSTICK_PRIORITY);
}

/// `set_irq_priority` writes IRQ `n`'s byte in the containing IPR word and preserves the three
/// neighbouring IRQs' priority bytes (a byte-in-word RMW), and `irq_priority` reads it back.
#[test]
fn r7_set_irq_priority_round_trips_and_preserves_neighbours() {
    let _g = mock::lock();
    mock::reset();

    // IRQ 12 (F1x0 ADC): its byte is IPR word 3 (12/4), shift 0 (12%4). Pre-seed the whole word
    // with a sentinel so we can prove the RMW touched only IRQ 12's byte.
    let word_addr = NVIC_IPR_BASE + (12 / 4) * 4;
    Reg32::new(word_addr, 0).write(0xAABBCCDD);

    set_irq_priority(12, MOTOR_ISR_PRIORITY);
    assert_eq!(irq_priority(12), MOTOR_ISR_PRIORITY, "IRQ 12 priority set");
    // Neighbours (IRQs 13/14/15 = bytes 1/2/3 of the same word) unchanged.
    let w = Reg32::new(word_addr, 0).read();
    assert_eq!((w >> 8) & 0xFF, 0xCC, "IRQ 13 byte preserved");
    assert_eq!((w >> 16) & 0xFF, 0xBB, "IRQ 14 byte preserved");
    assert_eq!((w >> 24) & 0xFF, 0xAA, "IRQ 15 byte preserved");
    assert_eq!(
        w & 0xFF,
        MOTOR_ISR_PRIORITY as u32,
        "IRQ 12 byte is the new priority"
    );

    // A byte at a non-zero shift (IRQ 28 = word 7, shift 0; IRQ 30 = word 7, shift 16).
    set_irq_priority(30, USART_RX_PRIORITY);
    assert_eq!(irq_priority(30), USART_RX_PRIORITY);
}

/// `set_systick_priority` writes SHPR3's top byte and `systick_priority` reads it back; the low
/// three bytes of SHPR3 are preserved.
#[test]
fn r7_set_systick_priority_round_trips_top_byte_only() {
    let _g = mock::lock();
    mock::reset();
    Reg32::new(SCB_SHPR3, 0).write(0x0011_2233);
    set_systick_priority(SYSTICK_PRIORITY);
    assert_eq!(systick_priority(), SYSTICK_PRIORITY);
    assert_eq!(
        Reg32::new(SCB_SHPR3, 0).read() & 0x00FF_FFFF,
        0x0011_2233,
        "SHPR3 low three bytes preserved"
    );
}

/// `period_irq` resolves the ADC injected-EOC vector per family: the period ISR on both.
#[test]
fn r7_period_irq_resolves_per_family() {
    assert_eq!(period_irq(IrqLayout::F1x0Grouped), F1X0_ADC_CMP_IRQ);
    assert_eq!(period_irq(IrqLayout::F10xSeparate), R7_F10X_ADC);
}

/// `usart_irqs` against the GD SPL `IRQn_Type` values (`gd32f1x0.h` / `gd32f10x.h`) and the DMA
/// request tables (GD32F1x0 UM Rev3.6 Table 8-3, GD32F10x UM Rev2.6 Figure 9-4): every instance's
/// three vectors on each family, USART2 absent on the F1x0, and no non-USART label.
#[test]
fn usart_irqs_match_the_spl_per_instance_and_family() {
    let v = |rx, dma_rx, dma_tx| Some(UsartIrqs { rx, dma_rx, dma_tx });
    // F1x0: the numbers themselves, then the resolver.
    assert_eq!((F1X0_USART0_IRQ, F1X0_USART1_IRQ), (27, 28));
    assert_eq!((F1X0_DMA_CH1_2_IRQ, F1X0_DMA_CH3_4_IRQ), (10, 11));
    let f1 = IrqLayout::F1x0Grouped;
    assert_eq!(usart_irqs(f1, PeriphLabel::Usart0), v(27, 10, 10));
    assert_eq!(usart_irqs(f1, PeriphLabel::Usart1), v(28, 11, 11));
    assert_eq!(
        usart_irqs(f1, PeriphLabel::Usart2),
        None,
        "no USART2 on F1x0"
    );
    // F10x: USART0_IRQn 37, USART1_IRQn 38, USART2_IRQn 39; DMA0_Channel1..6_IRQn = 12..17.
    assert_eq!(
        (F10X_USART0_IRQ, F10X_USART1_IRQ, F10X_USART2_IRQ),
        (37, 38, 39)
    );
    assert_eq!(
        (
            F10X_DMA0_CH1_IRQ,
            F10X_DMA0_CH2_IRQ,
            F10X_DMA0_CH3_IRQ,
            F10X_DMA0_CH4_IRQ,
            F10X_DMA0_CH5_IRQ,
            F10X_DMA0_CH6_IRQ
        ),
        (12, 13, 14, 15, 16, 17)
    );
    let f10 = IrqLayout::F10xSeparate;
    assert_eq!(usart_irqs(f10, PeriphLabel::Usart0), v(37, 15, 14));
    assert_eq!(usart_irqs(f10, PeriphLabel::Usart1), v(38, 16, 17));
    assert_eq!(usart_irqs(f10, PeriphLabel::Usart2), v(39, 13, 12));
    for layout in [f1, f10] {
        assert_eq!(usart_irqs(layout, PeriphLabel::Gpioa), None);
        assert_eq!(usart_irqs(layout, PeriphLabel::I2c0), None);
    }
}

/// No two instances share a vector on either family (a grouped DMA vector is shared by the two
/// halves of ONE instance only), and none is the period vector, so the instances' receivers and
/// transmitters cannot arrive on each other's slots.
#[test]
fn usart_irqs_never_collide_across_instances_or_with_the_period_vector() {
    for layout in [IrqLayout::F1x0Grouped, IrqLayout::F10xSeparate] {
        let all: Vec<(PeriphLabel, UsartIrqs)> = [
            PeriphLabel::Usart0,
            PeriphLabel::Usart1,
            PeriphLabel::Usart2,
        ]
        .into_iter()
        .filter_map(|l| usart_irqs(layout, l).map(|v| (l, v)))
        .collect();
        for (i, (la, a)) in all.iter().enumerate() {
            for n in [a.rx, a.dma_rx, a.dma_tx] {
                assert_ne!(n, period_irq(layout), "{layout:?} {la:?}");
            }
            for (lb, b) in &all[i + 1..] {
                for x in [a.rx, a.dma_rx, a.dma_tx] {
                    for y in [b.rx, b.dma_rx, b.dma_tx] {
                        assert_ne!(x, y, "{layout:?}: {la:?} and {lb:?} share vector {x}");
                    }
                }
            }
        }
    }
}

/// `specs/usart-pin-remap.md` section 5, host test 7: `apply_motor_era_priorities` sets every
/// instance's three vectors on both families, the F10x USART0 ones (37, 15, 14) included, below the
/// period ISR and above SysTick. Every byte starts at its reset value 0 (the most urgent there is),
/// so a vector the policy missed would show up as outranking the period ISR.
#[test]
fn r7_apply_motor_era_priorities_covers_every_instance_on_both_families() {
    for layout in [IrqLayout::F1x0Grouped, IrqLayout::F10xSeparate] {
        let _g = mock::lock();
        mock::reset();
        let instances: Vec<UsartIrqs> = [
            PeriphLabel::Usart0,
            PeriphLabel::Usart1,
            PeriphLabel::Usart2,
        ]
        .into_iter()
        .filter_map(|l| usart_irqs(layout, l))
        .collect();
        for v in &instances {
            assert_eq!(irq_priority(v.rx), 0, "{layout:?}: reset byte is 0");
        }

        apply_motor_era_priorities(layout);

        let period = period_irq(layout);
        assert_eq!(
            irq_priority(period),
            MOTOR_ISR_PRIORITY,
            "period ISR highest"
        );
        assert_eq!(systick_priority(), SYSTICK_PRIORITY, "systick lowest");
        for v in &instances {
            assert_eq!(irq_priority(v.rx), USART_RX_PRIORITY, "{layout:?} {v:?}");
            assert_eq!(irq_priority(v.dma_rx), DMA_RX_PRIORITY, "{layout:?} {v:?}");
            assert_eq!(irq_priority(v.dma_tx), DMA_RX_PRIORITY, "{layout:?} {v:?}");
            assert!(irq_priority(period) < irq_priority(v.rx));
            assert!(irq_priority(v.rx) < irq_priority(v.dma_rx));
            assert!(irq_priority(v.dma_tx) < systick_priority());
        }
    }
    // The F10x USART0 vectors by number, the ones that sat at reset priority 0 before this change.
    let _g = mock::lock();
    mock::reset();
    apply_motor_era_priorities(IrqLayout::F10xSeparate);
    assert_eq!(irq_priority(37), USART_RX_PRIORITY);
    assert_eq!(irq_priority(15), DMA_RX_PRIORITY);
    assert_eq!(irq_priority(14), DMA_RX_PRIORITY);
}

/// `unmask_irq` is the crate's NVIC set-enable owner and is a no-op under the host `mock` build
/// (there is no NVIC there), so calling it from host-test code is safe and changes no register.
/// The hardware body is exercised on silicon by every RX bring-up and by the motor period vector.
#[test]
fn unmask_irq_is_inert_under_mock() {
    let _g = mock::lock();
    mock::reset();
    // Both the F1x0 and F10x period-vector numbers; nothing is written anywhere.
    super::unmask_irq(F1X0_ADC_CMP_IRQ);
    super::unmask_irq(R7_F10X_ADC);
    assert_eq!(
        Reg32::new(NVIC_IPR_BASE, 0).read(),
        0,
        "unmask writes no IPR byte"
    );
}
