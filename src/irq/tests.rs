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
    F10X_ADC0_1_IRQ, F10X_DMA0_CH2_IRQ, F10X_EXTI10_15_IRQ, F10X_EXTI5_9_IRQ, F10X_TIMER0_BRK_IRQ,
    F10X_TIMER0_CHANNEL_IRQ, F10X_TIMER0_TRG_CMT_IRQ, F10X_TIMER0_UP_IRQ, F1X0_ADC_CMP_IRQ,
    F1X0_EXTI0_1_IRQ, F1X0_EXTI2_3_IRQ, F1X0_EXTI4_15_IRQ, F1X0_TIMER0_BRK_UP_TRG_COM_IRQ,
    F1X0_TIMER0_CHANNEL_IRQ, INTF_BRKIF, INTF_CMTIF, INTF_TRGIF, INTF_UPIF, MAX_VECTORS,
    SYSTEM_VECTORS, TIMER_INTF,
};
use super::{
    DMA_RX_ISR_METRIC, F1X0_DMA_CH3_4_IRQ, F1X0_USART1_IRQ, SYSTICK_ISR_METRIC,
    USART1_RX_ISR_METRIC,
};
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

    // A slot the layout does not route (e.g. IRQ 50, an unused gap) stays the default ISR. This is
    // the "misplaced slot points at the wrong handler" guard: only the named slots diverge.
    assert_eq!(slot(&t, 50), handler_addr(default_isr));
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
    // is DMA0_Channel2 = the module USART's DMA-ring RX vector (GD USART2_RX). The advanced timer does
    // NOT use slot 13 on F10x; the module DMA channel does.
    assert_eq!(
        F10X_DMA0_CH2_IRQ, F1X0_TIMER0_BRK_UP_TRG_COM_IRQ,
        "both are IRQ 13"
    );
    assert_eq!(
        slot(&t, F10X_DMA0_CH2_IRQ),
        handler_addr(super::module_dma_rx_isr)
    );
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
    // The table covers F1x0's highest external IRQ (73, CAN1_SCE) -> 90 entries total.
    assert_eq!(MAX_VECTORS, SYSTEM_VECTORS + super::MAX_IRQS);
    assert_eq!(MAX_VECTORS, 16 + 74);
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
