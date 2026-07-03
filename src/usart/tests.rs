//! T6 host tests for the shared USART driver (run under the `mock` feature against the
//! backing-array register space).
//!
//! The bench config is the proven GD SPL link: **USART1, 72 MHz sysclk, 115200 8N1**. USART1 is on
//! APB1, and at 72 MHz sysclk APB1 is clocked at 36 MHz (AHB/2, the "APB1 max 36 MHz" arrangement
//! the GD `system_72m_*` setup programs and `rcu_clock_freq_get` reads back). So the USART1 input
//! clock is 36 MHz and the SPL BAUD formula yields:
//!
//! ```text
//! udiv = (36_000_000 + 115_200/2) / 115_200 = (36_000_000 + 57_600) / 115_200 = 313
//! BAUD = udiv & 0xFFFF = 313 = 0x139
//! ```
//!
//! That exact value (`313`) is hardcoded below as the expected BRR. The baud sweep re-implements
//! the SPL formula as an independent oracle and diffs runtime-hal's BAUD against it across a range
//! of baud and clock inputs, so a clock-source or rounding mistake shows up as a wrong divisor
//! without standing up a live SPL build.
#![cfg(feature = "mock")]

use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::{ClockConfig, ClockSource};
use crate::config::{Oversampling, UsartConfig, UsartFrame};
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::error::UsartError;
use crate::reg::{mock, Reg32};
use crate::usart::{compute_brr, usart_input_clock, Usart, UsartBus};
use std::sync::MutexGuard;

/// Build a `Chip` whose addr table maps USART1 to `USART_BASE` (and the RCU base), with the given
/// clock-tree path (which selects the USART register model in `bring_up`).
fn chip_for(path: ClockPath) -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    Chip::from_descriptor(McuDescriptor {
        gpio: if path == ClockPath::F1x0Rcu {
            GpioPath::AhbCtlAfsel
        } else {
            GpioPath::ApbCrlCrh
        },
        clock: path,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

/// The bench USART1 (8N1, /16) config; the base is resolved via the chip from `PeriphLabel::Usart1`.
fn bench_cfg() -> UsartConfig {
    UsartConfig {
        usart: PeriphLabel::Usart1,
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    }
}

/// The reference 72 MHz / 2 WS tree (IRC8M, pll18, ahb1, apb1 /2, apb2 /1).
fn ref_72m() -> ClockConfig {
    ClockConfig::REFERENCE_72M_IRC8M
}

/// A USART base in the mock space (the numeric offsets within it are what the assertions key on).
/// The real GD USART1 base is `0x4000_4400`; the mock window wraps modulo its size, so the value
/// only matters in that we read back the same offsets we wrote.
const USART_BASE: u32 = 0x4000_4400;

/// The bench-config expected BAUD value, computed by hand from the SPL formula (see module docs):
/// USART1, 36 MHz input (72 MHz sysclk, APB1 = /2), 115200 baud.
const EXPECTED_BRR_USART1_72M_115200: u32 = 313; // 0x139

/// Acquire the whole-case serialization lock and zero the USART register window.
fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

/// The register offsets a family's model uses, taken from the documented per-family layout. Kept
/// in lockstep with `UsartModel::{F10X,F1X0}` by construction; the test asserts against these
/// rather than reaching into the model's private fields.
struct Offsets {
    baud: u32,
    ctl0: u32,
    ctl1: u32,
    ctl2: u32,
}

/// Offsets for a clock path, taken from the documented per-family layout.
fn offsets_for_path(path: ClockPath) -> Offsets {
    match path {
        // gd32f10x_usart.h: STAT 0x00, DATA 0x04, BAUD 0x08, CTL0 0x0C, CTL1 0x10, CTL2 0x14.
        ClockPath::F10xRcc => Offsets {
            baud: 0x08,
            ctl0: 0x0C,
            ctl1: 0x10,
            ctl2: 0x14,
        },
        // gd32f1x0_usart.h: CTL0 0x00, CTL1 0x04, CTL2 0x08, BAUD 0x0C, STAT 0x1C, RDATA 0x24, TDATA 0x28.
        ClockPath::F1x0Rcu => Offsets {
            baud: 0x0C,
            ctl0: 0x00,
            ctl1: 0x04,
            ctl2: 0x08,
        },
    }
}

// --- (a) exact BRR for the bench config -------------------------------------------------------

#[test]
fn brr_usart1_72mhz_115200_is_the_spl_value() {
    // Pure formula: USART1 is on APB1; 72 MHz sysclk -> 36 MHz APB1.
    let clock = ref_72m();
    let uclk = usart_input_clock(&clock, UsartBus::Apb1);
    assert_eq!(
        uclk, 36_000_000,
        "USART1 input clock is APB1 = sysclk/2 at 72 MHz"
    );
    assert_eq!(
        compute_brr(uclk, 115_200) as u32,
        EXPECTED_BRR_USART1_72M_115200
    );
}

#[test]
fn bring_up_writes_expected_brr_to_the_baud_register() {
    let _g = seed_reset();
    let off = offsets_for_path(ClockPath::F10xRcc);
    Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    assert_eq!(
        Reg32::new(USART_BASE, off.baud).read(),
        EXPECTED_BRR_USART1_72M_115200,
        "BAUD register must hold the exact SPL divisor for USART1 @ 72 MHz / 115200"
    );
}

// --- (b) 8N1 frame + TX/RX + UART enable bits -------------------------------------------------

#[test]
fn f10x_bring_up_sets_8n1_and_enable_bits() {
    let _g = seed_reset();
    let off = offsets_for_path(ClockPath::F10xRcc);
    Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();

    // CTL0: REN (BIT2) + TEN (BIT3) + UEN (BIT13 on F10x). WL (BIT12), PM (BIT9), PCEN (BIT10)
    // all clear = 8 bits, no parity.
    let ctl0 = Reg32::new(USART_BASE, off.ctl0).read();
    assert_eq!(
        ctl0,
        (1 << 2) | (1 << 3) | (1 << 13),
        "REN+TEN+UEN, WL/PM/PCEN clear (8N1)"
    );
    assert_eq!(ctl0 & (1 << 12), 0, "WL = 0 (8 data bits)");
    assert_eq!(ctl0 & ((1 << 9) | (1 << 10)), 0, "PM/PCEN = 0 (no parity)");

    // CTL1: STB (BITS 12,13) = 0 (1 stop bit). Nothing else set.
    let ctl1 = Reg32::new(USART_BASE, off.ctl1).read();
    assert_eq!(ctl1 & (0b11 << 12), 0, "STB = 0 (1 stop bit)");
    assert_eq!(ctl1, 0, "no other CTL1 bits touched");

    // CTL2 left at reset (no flow control / DMA / IrDA for the M1 polled path).
    assert_eq!(Reg32::new(USART_BASE, off.ctl2).read(), 0, "CTL2 untouched");
}

#[test]
fn f1x0_bring_up_sets_8n1_and_enable_bits_with_uen_at_bit0() {
    let _g = seed_reset();
    // F1x0 is the divergent register model: same logical 8N1 but UEN is BIT(0), and the register
    // offsets differ. The link is still USART1 on APB1.
    let off = offsets_for_path(ClockPath::F1x0Rcu);
    Usart::bring_up(&chip_for(ClockPath::F1x0Rcu), &ref_72m(), &bench_cfg()).unwrap();

    // CTL0: REN (BIT2) + TEN (BIT3) + UEN (BIT0 on F1x0). WL/PM/PCEN clear.
    let ctl0 = Reg32::new(USART_BASE, off.ctl0).read();
    assert_eq!(
        ctl0,
        (1 << 0) | (1 << 2) | (1 << 3),
        "REN+TEN+UEN(bit0), WL/PM/PCEN clear"
    );
    assert_eq!(ctl0 & (1 << 12), 0, "WL = 0 (8 data bits)");

    let ctl1 = Reg32::new(USART_BASE, off.ctl1).read();
    assert_eq!(ctl1 & (0b11 << 12), 0, "STB = 0 (1 stop bit)");

    // BAUD is the same 36 MHz / 115200 divisor (the input clock derivation is family-independent).
    assert_eq!(
        Reg32::new(USART_BASE, off.baud).read(),
        EXPECTED_BRR_USART1_72M_115200
    );
}

#[test]
fn bring_up_preserves_unrelated_ctl0_bits() {
    let _g = seed_reset();
    let off = offsets_for_path(ClockPath::F10xRcc);
    // Seed an unrelated CTL0 bit (e.g. an interrupt-enable a higher layer set): the RMW bring-up
    // must not clobber it. RBNEIE = BIT(5).
    Reg32::new(USART_BASE, off.ctl0).write(1 << 5);
    Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    let ctl0 = Reg32::new(USART_BASE, off.ctl0).read();
    assert_eq!(
        ctl0 & (1 << 5),
        1 << 5,
        "pre-existing RBNEIE survives the RMW bring-up"
    );
    assert_eq!(
        ctl0 & ((1 << 2) | (1 << 3) | (1 << 13)),
        (1 << 2) | (1 << 3) | (1 << 13)
    );
}

// --- input-clock derivation -------------------------------------------------------------------

#[test]
fn usart0_input_clock_tracks_apb2_not_apb1() {
    // USART0 is on APB2, which is not divided down at 72 MHz, so its input clock is the full
    // sysclk, unlike USART1.
    let clock = ref_72m();
    assert_eq!(usart_input_clock(&clock, UsartBus::Apb2), 72_000_000);
    assert_eq!(usart_input_clock(&clock, UsartBus::Apb1), 36_000_000);
}

#[test]
fn apb1_input_clock_uses_profile_prescaler() {
    // M2 reconciliation: the APB1 input clock = AHB / profile.apb1_psc (and AHB = sysclk /
    // ahb_psc), read straight from the prescalers configure_tree programs, not a 36 MHz ceiling
    // heuristic. The default profile (apb1_psc = 2) gives sysclk/2; a non-default profile divides
    // by whatever it carries.
    // Default 72 MHz tree: APB1 = 36 MHz.
    let def = ref_72m();
    assert_eq!(usart_input_clock(&def, UsartBus::Apb1), 36_000_000);

    // An explicit config with apb1_psc = 4 gives sysclk/4.
    let p = ClockConfig {
        sysclk_hz: 72_000_000,
        wait_states: 2,
        source: ClockSource::Irc8m,
        pll_mul: 18,
        ahb_psc: 1,
        apb1_psc: 4,
        apb2_psc: 1,
    };
    assert_eq!(usart_input_clock(&p, UsartBus::Apb1), 18_000_000);
    // AHB prescaler also feeds through: ahb_psc = 2 halves AHB before the APB divide.
    let p2 = ClockConfig { ahb_psc: 2, ..p };
    assert_eq!(usart_input_clock(&p2, UsartBus::Apb1), 9_000_000);
    assert_eq!(usart_input_clock(&p2, UsartBus::Apb2), 36_000_000); // (72/2)/1
}

// --- polled byte primitives -------------------------------------------------------------------

#[test]
fn write_byte_writes_data_register_when_ready() {
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    // Pre-seed STAT with TBE+TC set so write_byte does not spin (the mock has no UART core that
    // would set them). STAT is at 0x00 on F10x. TBE = BIT(7), TC = BIT(6).
    Reg32::new(USART_BASE, 0x00).write((1 << 7) | (1 << 6));
    u.write_byte(0x5A);
    // DATA register at 0x04 holds the byte (F10x TX = RX = DATA at 0x04).
    assert_eq!(Reg32::new(USART_BASE, 0x04).read() & 0xFF, 0x5A);
}

#[test]
fn try_read_byte_returns_none_when_not_ready_and_data_when_rbne() {
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    // STAT 0x00, RBNE = BIT(5), DATA 0x04. Bring_up may have left STAT zero; force it.
    Reg32::new(USART_BASE, 0x00).write(0);
    assert_eq!(u.try_read_byte(), Ok(None), "no RBNE -> no byte");

    Reg32::new(USART_BASE, 0x04).write(0xA5);
    Reg32::new(USART_BASE, 0x00).write(1 << 5); // RBNE
    assert_eq!(u.try_read_byte(), Ok(Some(0xA5)));
}

#[test]
fn try_read_byte_surfaces_framing_and_parity_after_clearing() {
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    // FERR = BIT(1), PERR = BIT(0) in STAT (0x00). Framing/parity still surface as Err (the byte they
    // describe is suspect), but the HAL clears them first so they cannot latch (overrun is handled
    // separately, see the recovery tests below). On F10x the clear is the STAT+data read pair, which
    // does not zero the mock STAT (no UART core), so each case re-seeds STAT explicitly.
    Reg32::new(USART_BASE, 0x00).write(1 << 1);
    assert_eq!(u.try_read_byte(), Err(UsartError::Framing));
    Reg32::new(USART_BASE, 0x00).write(1 << 0);
    assert_eq!(u.try_read_byte(), Err(UsartError::Parity));
}

// --- (d) overrun (ORE) self-recovery, both families -------------------------------------------

#[test]
fn overrun_recovers_via_stat_data_read_pair_on_f10x() {
    // F10x clears a sticky ORERR with the STAT-then-data-register read pair (no INTC). An overrun
    // must NOT return Err and strand RX (the link_bench latch bug): the HAL clears it and returns the
    // freshest byte if one is still ready, else Ok(None). Here we model the silicon: STAT has ORERR
    // (+ RBNE, a byte waiting), the data register holds the byte, and after the HAL clears + reads we
    // confirm it surfaced the byte rather than erroring.
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    // STAT 0x00: ORERR (BIT 3) + RBNE (BIT 5). DATA 0x04 holds the byte.
    Reg32::new(USART_BASE, 0x04).write(0x7E);
    Reg32::new(USART_BASE, 0x00).write((1 << 3) | (1 << 5));
    // Not an Err: the overrun is recoverable. With RBNE still set, the byte is returned.
    assert_eq!(
        u.try_read_byte(),
        Ok(Some(0x7E)),
        "F10x overrun is cleared and the fresh byte returned, not Err"
    );

    // And RX keeps working afterwards: a plain RBNE byte (no error) still reads back.
    Reg32::new(USART_BASE, 0x04).write(0x33);
    Reg32::new(USART_BASE, 0x00).write(1 << 5);
    assert_eq!(
        u.try_read_byte(),
        Ok(Some(0x33)),
        "RX still alive after overrun"
    );
}

#[test]
fn overrun_recovers_via_intc_orecf_write_on_f1x0() {
    // F1x0 clears a sticky ORERR by writing ORECF (BIT 3) to INTC at 0x20 (not a data-register read).
    // The HAL must run that exact write and not return Err. STAT is at 0x1C, RDATA at 0x24 on F1x0.
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F1x0Rcu), &ref_72m(), &bench_cfg()).unwrap();
    // STAT 0x1C: ORERR (BIT 3) + RBNE (BIT 5). RDATA 0x24 holds the byte.
    Reg32::new(USART_BASE, 0x24).write(0xC4);
    Reg32::new(USART_BASE, 0x1C).write((1 << 3) | (1 << 5));
    assert_eq!(
        u.try_read_byte(),
        Ok(Some(0xC4)),
        "F1x0 overrun is cleared (ORECF -> INTC) and the fresh byte returned, not Err"
    );
    // The HAL wrote ORECF (BIT 3) into INTC at 0x20 (the family-correct clear sequence ran).
    assert_eq!(
        Reg32::new(USART_BASE, 0x20).read() & (1 << 3),
        1 << 3,
        "ORECF was written to INTC (0x20) to clear the overrun"
    );

    // RX keeps working: a plain RBNE byte still reads back after the recovery.
    Reg32::new(USART_BASE, 0x24).write(0x55);
    Reg32::new(USART_BASE, 0x1C).write(1 << 5);
    assert_eq!(
        u.try_read_byte(),
        Ok(Some(0x55)),
        "RX still alive after overrun"
    );
}

#[test]
fn overrun_with_no_fresh_byte_returns_none_not_err_on_f1x0() {
    // An overrun with no byte still queued (RBNE clear after the clear) returns Ok(None), never Err:
    // the receiver self-recovers and the caller simply polls again. Models F1x0 where INTC clears
    // ORERR and STAT then shows no RBNE.
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F1x0Rcu), &ref_72m(), &bench_cfg()).unwrap();
    // STAT 0x1C: ORERR only (no RBNE). Note: the mock has no UART core, so the INTC write does not
    // auto-clear STAT; clear it by hand to model the silicon's post-clear state (no RBNE).
    Reg32::new(USART_BASE, 0x1C).write(1 << 3);
    // The HAL clears ORE (writes ORECF). Re-reading STAT, the mock still shows ORERR set because no
    // core cleared it; to model silicon we drop STAT to 0 (the clear took effect, no byte ready).
    // Run try_read_byte, then assert it did not Err. The first call clears + re-reads STAT: with the
    // mock's sticky ORERR, the re-read still shows ORERR but no RBNE -> Ok(None). Either way: not Err.
    let r = u.try_read_byte();
    assert!(
        matches!(r, Ok(None)),
        "overrun with no fresh byte is Ok(None), never Err (got {r:?})"
    );
    // And ORECF reached INTC.
    assert_eq!(Reg32::new(USART_BASE, 0x20).read() & (1 << 3), 1 << 3);
}

// --- (c) baud sweep against the SPL formula oracle --------------------------------------------

/// The SPL `usart_baudrate_set` BAUD value, re-implemented independently as the test oracle
/// (`gd32f10x_usart.c:115-118`): oversampling-by-16, round-to-nearest divide, BAUD = low 16 bits.
fn spl_brr_oracle(uclk_hz: u32, baud: u32) -> u16 {
    let uclk = uclk_hz as u64;
    let b = baud as u64;
    let udiv = (uclk + b / 2) / b;
    let intdiv = udiv & 0x0000_FFF0;
    let fradiv = udiv & 0x0000_000F;
    ((intdiv | fradiv) & 0xFFFF) as u16
}

proptest::proptest! {
    /// For a sweep of input clocks and baud rates, runtime-hal's `compute_brr` must equal the SPL
    /// formula oracle exactly. A clock-source or rounding mistake would diverge here.
    #[test]
    fn compute_brr_matches_spl_oracle(
        uclk in 1_000_000u32..=72_000_000u32,
        baud in 1_200u32..=921_600u32,
    ) {
        proptest::prop_assume!(baud <= uclk); // a baud above the input clock is not a real config
        proptest::prop_assert_eq!(compute_brr(uclk, baud), spl_brr_oracle(uclk, baud));
    }

    /// End-to-end through the input-clock derivation: for any sysclk on the bus, the BAUD the
    /// driver computes for USART1 equals the oracle fed the derived APB1 clock.
    #[test]
    fn usart1_brr_end_to_end_matches_oracle(
        sysclk in 8_000_000u32..=72_000_000u32,
        baud in 9_600u32..=460_800u32,
    ) {
        // The M1 default tree shape (ahb /1, apb1 /2, apb2 /1), arbitrary sysclk: APB1 = sysclk/2.
        let clock = ClockConfig {
            sysclk_hz: sysclk,
            wait_states: 0,
            source: ClockSource::Irc8m,
            pll_mul: 18,
            ahb_psc: 1,
            apb1_psc: 2,
            apb2_psc: 1,
        };
        let uclk = usart_input_clock(&clock, UsartBus::Apb1);
        proptest::prop_assume!(baud <= uclk);
        proptest::prop_assert_eq!(compute_brr(uclk, baud), spl_brr_oracle(uclk, baud));
    }
}

// --- (e) split / rejoin / set_baud ownership rules (specs/usart-split.md section 5) ------------

#[test]
fn split_halves_address_the_same_peripheral_and_tx_writes() {
    let _g = seed_reset();
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    let (tx, _rx) = u.split();
    // Pre-seed TBE+TC so write_byte does not spin (F10x STAT 0x00, TBE BIT7, TC BIT6).
    Reg32::new(USART_BASE, 0x00).write((1 << 7) | (1 << 6));
    tx.write_byte(0x42);
    // The byte landed in THIS peripheral's data register (F10x DATA 0x04): the halves alias the
    // one configured base, not a copy.
    assert_eq!(Reg32::new(USART_BASE, 0x04).read() & 0xFF, 0x42);
}

#[test]
fn set_baud_reprograms_baud_and_leaves_frame_and_uen() {
    let _g = seed_reset();
    let off = offsets_for_path(ClockPath::F10xRcc);
    let mut u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    assert_eq!(
        Reg32::new(USART_BASE, off.baud).read(),
        EXPECTED_BRR_USART1_72M_115200
    );
    let ctl0_before = Reg32::new(USART_BASE, off.ctl0).read();

    u.set_baud(&ref_72m(), 9_600);

    // BAUD = (36 MHz + 4800) / 9600 = 3750 (the SPL round-to-nearest divisor).
    assert_eq!(Reg32::new(USART_BASE, off.baud).read(), 3750, "9600 BRR");
    // UEN re-set, frame/enable bits untouched (the RMW disabled then re-enabled only UEN).
    assert_eq!(
        Reg32::new(USART_BASE, off.ctl0).read(),
        ctl0_before,
        "CTL0 ends exactly as it started (REN+TEN+UEN, 8N1 fields)"
    );
}

#[test]
fn rejoin_matching_halves_restores_reconfigurability() {
    let _g = seed_reset();
    let off = offsets_for_path(ClockPath::F10xRcc);
    let u = Usart::bring_up(&chip_for(ClockPath::F10xRcc), &ref_72m(), &bench_cfg()).unwrap();
    let (tx, rx) = u.split();
    let mut u = Usart::rejoin(tx, rx);
    u.set_baud(&ref_72m(), 57_600);
    // (36 MHz + 28800) / 57600 = 625.5 -> 625.
    assert_eq!(Reg32::new(USART_BASE, off.baud).read(), 625);
}

#[test]
#[should_panic(expected = "different peripherals")]
fn rejoin_of_mismatched_halves_panics() {
    let _g = seed_reset();
    // A chip with TWO USART instances at distinct bases.
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART_BASE);
    addrs.set(PeriphLabel::Usart2, 0x4000_4800);
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    let chip = Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    });
    let u1 = Usart::bring_up(&chip, &ref_72m(), &bench_cfg()).unwrap();
    let cfg2 = UsartConfig {
        usart: PeriphLabel::Usart2,
        ..bench_cfg()
    };
    let u2 = Usart::bring_up(&chip, &ref_72m(), &cfg2).unwrap();
    let (tx1, _rx1) = u1.split();
    let (_tx2, rx2) = u2.split();
    let _ = Usart::rejoin(tx1, rx2); // cross-peripheral rejoin: programming error, fail loud
}
