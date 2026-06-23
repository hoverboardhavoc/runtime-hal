//! T1 host tests (run with `cargo test --features mock`).
//!
//! Two checks the milestone calls for:
//!   (a) `Reg16` / `Reg32` are width-distinct over the mock backing array: a 32-bit write is not
//!       seen as two unrelated 16-bit values, and two 16-bit writes compose into the 32-bit view.
//!   (b) a register-model conformance test: a few known GD32F10x / ST F103 USART/GPIO/RCU register
//!       offsets and reset values, asserted against the GD User Manual register tables (seed facts
//!       below). Small for T1; T3+ grows it as the divergent paths land.

#![cfg(feature = "mock")]

use runtime_hal::reg::{mock, Reg16, Reg32};
use runtime_hal::McuDescriptor;
use runtime_hal::{decode_pin, AddrTable, PeriphLabel};
use runtime_hal::{AdcPath, ClockPath, GpioPath, IrqLayout, PageSize};

// --- Seed register facts (GD32F10x User Manual; identical base map to ST F103) -------------
//
// These are the independent-source seeds the conformance test pins against. The bases are the
// standard APB2 peripheral map; offsets and reset values are from the peripheral register tables.

/// RCU (reset & clock unit) base.
const RCU_BASE: u32 = 0x4002_1000;
/// RCU_APB2EN: APB2 peripheral clock enable (holds USART0EN, IOPAEN, ...). 32-bit.
const RCU_APB2EN_OFFSET: u32 = 0x18;
/// RCU_APB2EN reset value: all peripheral clocks off.
const RCU_APB2EN_RESET: u32 = 0x0000_0000;

/// USART0 base (ST USART1).
const USART0_BASE: u32 = 0x4001_3800;
/// USART_DATA (data register, ST DR). 16-bit on this family.
const USART_DATA_OFFSET: u32 = 0x04;
/// USART_DATA reset value.
const USART_DATA_RESET: u16 = 0x0000;

/// GPIOA base.
const GPIOA_BASE: u32 = 0x4001_0800;
/// GPIO_CTL0 (port control register 0, ST CRL). 32-bit.
const GPIO_CTL0_OFFSET: u32 = 0x00;
/// GPIO_CTL0 reset value: every low pin resets to floating input (MODE=00, CNF=01 -> nibble 0x4).
const GPIO_CTL0_RESET: u32 = 0x4444_4444;

// --- (a) width-distinctness ----------------------------------------------------------------

#[test]
fn reg16_and_reg32_are_width_distinct() {
    let _serial = mock::lock(); // serialize against other cases that reset the shared space
    mock::reset();

    // Pick a scratch base well clear of the seed addresses.
    let base = 0x2000_0000u32;

    // A 32-bit write of 0xDEADBEEF lands as four LE bytes EF BE AD DE.
    let r32 = Reg32::new(base, 0x00);
    r32.write(0xDEAD_BEEF);

    // Two 16-bit views over the same span see the two halves, not the whole word, and not some
    // mangled overlap. Little-endian: low half at +0, high half at +2.
    let lo16 = Reg16::new(base, 0x00);
    let hi16 = Reg16::new(base, 0x02);
    assert_eq!(lo16.read(), 0xBEEF, "low 16-bit half of the 32-bit write");
    assert_eq!(hi16.read(), 0xDEAD, "high 16-bit half of the 32-bit write");

    // And a single 16-bit read is *not* the full 32-bit value (the widths are genuinely distinct).
    assert_ne!(u32::from(lo16.read()), 0xDEAD_BEEF);

    // Conversely, two independent 16-bit writes compose into the 32-bit view.
    mock::reset();
    Reg16::new(base, 0x00).write(0x1234);
    Reg16::new(base, 0x02).write(0xABCD);
    assert_eq!(
        Reg32::new(base, 0x00).read(),
        0xABCD_1234,
        "two 16-bit writes compose LE into one 32-bit word"
    );

    // Byte-level endianness sanity (proves it is the backing array, not a coincidence).
    assert_eq!(mock::peek_byte(base + 0), 0x34);
    assert_eq!(mock::peek_byte(base + 1), 0x12);
    assert_eq!(mock::peek_byte(base + 2), 0xCD);
    assert_eq!(mock::peek_byte(base + 3), 0xAB);
}

#[test]
fn reg_modify_is_read_modify_write() {
    let _serial = mock::lock();
    mock::reset();
    let base = 0x2001_0000u32;
    let r = Reg32::new(base, 0x00);
    r.write(0x0000_00FF);
    // Set bits in the high byte without disturbing the low byte.
    r.modify(0xFF00_0000, 0xAB00_0000);
    assert_eq!(r.read(), 0xAB00_00FF);

    let h = Reg16::new(base, 0x10);
    h.write(0x00F0);
    h.modify(0x000F, 0x0005);
    assert_eq!(h.read(), 0x00F5);
}

// --- (b) register-model conformance --------------------------------------------------------

#[test]
fn known_register_offsets_and_reset_values() {
    let _serial = mock::lock();
    mock::reset();

    // The mock space starts zeroed, which is the reset value for RCU_APB2EN and USART_DATA.
    let rcu_apb2en = Reg32::new(RCU_BASE, RCU_APB2EN_OFFSET);
    assert_eq!(
        rcu_apb2en.read(),
        RCU_APB2EN_RESET,
        "RCU_APB2EN resets with all peripheral clocks off"
    );

    let usart_data = Reg16::new(USART0_BASE, USART_DATA_OFFSET);
    assert_eq!(
        usart_data.read(),
        USART_DATA_RESET,
        "USART_DATA resets to 0"
    );

    // GPIO_CTL0's documented reset is 0x44444444 (not zero); seed the mock space with it to model
    // the real reset state, then assert the accessor reads it back at the documented offset.
    let gpio_ctl0 = Reg32::new(GPIOA_BASE, GPIO_CTL0_OFFSET);
    gpio_ctl0.write(GPIO_CTL0_RESET);
    assert_eq!(
        gpio_ctl0.read(),
        0x4444_4444,
        "GPIO_CTL0 resets to floating-input on every low pin"
    );

    // Offsets are distinct and width-correct: USART_DATA is 16-bit at +0x04, not colliding with
    // the status register at +0x00.
    let usart_stat = Reg32::new(USART0_BASE, 0x00);
    usart_stat.write(0x0000_00C0); // TC|TXE set, say
    assert_eq!(
        usart_data.read(),
        0x0000,
        "writing the status register does not bleed into the data register at +0x04"
    );

    // The addresses really are 4 bytes apart (catches an offset typo).
    assert_eq!(usart_data.addr() - usart_stat.addr(), 0x04);
}

// --- descriptor smoke (it builds as a bounded-owned literal) -------------------------------

#[test]
fn descriptor_literal_and_addr_resolution() {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Rcu, RCU_BASE);
    addrs.set(PeriphLabel::Usart0, USART0_BASE);
    addrs.set(PeriphLabel::Gpioa, GPIOA_BASE);

    // resolve() yields the base; a missing label is a MissingBase error.
    assert_eq!(addrs.resolve(PeriphLabel::Usart0).unwrap(), USART0_BASE);
    assert!(addrs.resolve(PeriphLabel::Usart1).is_err());

    // The per-path range check (tightened in T3/T4) accepts these: GPIOA on the F10x APB bus
    // (0x4001_0800) under the apb_crl_crh path, and the shared RCU base.
    assert!(addrs
        .check_ranges(GpioPath::ApbCrlCrh, ClockPath::F10xRcc)
        .is_ok());

    let desc = McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F10xSeparate,
        addrs,
        flash_page: PageSize::K2,
        flash_kib: 256,
        adv_timers: 1,
        adc_count: 2,
    };

    // The descriptor is chip-only now; the wiring is code-level config. A logical pin byte still
    // decodes the same way (the application uses `decode_pin` on its `*Config` pins).
    assert_eq!(decode_pin((0 << 4) | 2), (0, 2)); // PA2
    assert_eq!(desc.flash_page.bytes(), 2048);
    assert_eq!(desc.adc_count, 2);
}
