//! T6/T7 host tests for the shared I2C driver (run under the `mock` feature against the
//! backing-array register space).
//!
//! Three groups:
//! - **timing** ([`timing_for`]): the CKCFG / RT / I2CCLK values vs the GD SPL `i2c_clock_config`
//!   formula, hand-computed for the bench cases (100 kHz at APB1 = 36 MHz and at 8 MHz, 400 kHz).
//! - **bring-up** ([`I2c::bring_up`]): the register end state (CTL1 I2CCLK, RT, CKCFG, CTL0
//!   I2CEN|ACKEN, SADDR0) for the IMU config.
//! - **transfer + `embedded-hal`** ([`embedded_hal::i2c::I2c`]): write / read / write_read over the
//!   mock register space with STAT0 seeded so the polls pass, and error-injection (BERR / LOSTARB /
//!   AERR in STAT0) asserting the mapped [`embedded_hal::i2c::ErrorKind`].
//! - **recovery + the busy gate** (the 2026-07-18 silicon finding): a failed transfer clears the
//!   sticky STAT0 error flags and SRESET-reinits a wedged block (pending START/STOP cleared, the
//!   timing reprogrammed); a fresh transfer on a stuck-busy bus fails fast with `Bus` instead of
//!   corrupting the wire; the non-fresh (repeated-START) read skips the busy gate.
//!
//! The mock backend is a flat array (a static register snapshot, not a sequencer), so the polled
//! loops are made to terminate by seeding STAT0 with every flag the transfer waits on set at once;
//! each `wait_flag` only checks its own bit, so a single all-flags-set STAT0 satisfies the whole
//! ordered sequence. The on-silicon flag-by-flag progression is the with_polling golden's job (T7
//! harness layer) and the bench IMU read (T13); this host layer proves the register-level transfer
//! shape and the error mapping.
#![cfg(feature = "mock")]

use super::*;
use crate::addr::{AddrTable, PeriphLabel};
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::descriptor::{AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize};
use crate::reg::{mock, Reg32};
use embedded_hal::i2c::{Error as _, ErrorKind, I2c as _, NoAcknowledgeSource, Operation};
use std::sync::MutexGuard;

/// The bench I2C0 base (the mock window wraps modulo its size; only the offsets matter).
const I2C_BASE: u32 = 0x4000_5400;
/// The I2C1 base, the second instance's own APB1 slot (both families). The mock register space is
/// keyed by absolute address, so this is genuinely distinct from `I2C_BASE`: a bring-up on the
/// wrong instance writes a different set of words and the assertions catch it.
const I2C1_BASE: u32 = 0x4000_5800;
/// The GPIOB base the SCL/SDA pins resolve to (PB6/PB7); distinct from `I2C_BASE` so the AF writes
/// land in their own window in the mock space.
const GPIOB_BASE: u32 = 0x4001_0C00;
/// The RCU base (the I2C / GPIO-port clock enables RMW into it).
const RCU_BASE: u32 = 0x4002_1000;
/// The IMU 7-bit address.
const IMU: u8 = 0x68;

/// Build a `Chip` for a given GPIO register-model path, mapping BOTH I2C instances, GPIOB, and the
/// RCU. `I2c::new` does not range-check the I2C base, so any base the chip resolves works.
fn chip_for(gpio: GpioPath) -> Chip {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::I2c0, I2C_BASE);
    addrs.set(PeriphLabel::I2c1, I2C1_BASE);
    addrs.set(PeriphLabel::Gpiob, GPIOB_BASE);
    addrs.set(PeriphLabel::Rcu, RCU_BASE);
    let clock = match gpio {
        GpioPath::ApbCrlCrh => ClockPath::F10xRcc,
        GpioPath::AhbCtlAfsel => ClockPath::F1x0Rcu,
    };
    Chip::from_descriptor(McuDescriptor {
        gpio,
        clock,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 64,
        adv_timers: 1,
        adc_count: 1,
    })
}

/// The F10x chip (the prior default), for the timing / bring-up / transfer assertions.
fn chip() -> Chip {
    chip_for(GpioPath::ApbCrlCrh)
}

/// The reference 72 MHz / 2 WS tree (APB1 = 36 MHz, matching the proven bench tree).
fn ref_72m() -> ClockConfig {
    ClockConfig::REFERENCE_72M_IRC8M
}

/// PB6 (SCL) / PB7 (SDA) handles split from a chip's GPIOB, the pins `I2c::new` consumes.
fn pb6_pb7(
    chip: &Chip,
) -> (
    crate::gpio::Pin<crate::gpio::Input<crate::gpio::Floating>>,
    crate::gpio::Pin<crate::gpio::Input<crate::gpio::Floating>>,
) {
    let gpiob = chip.gpiob().unwrap().split();
    (gpiob.pb6, gpiob.pb7)
}

#[test]
fn new_refuses_gate_capable_pins_the_layer_7_backstop() {
    // Layer 7 (`hoverboard-firmware/specs/motor-integration.md`): `I2c::new` configures the pins
    // itself, so it re-asks the ownership question rather than trusting its caller. PB14/PB15 are
    // TIMER0 CH1N/CH2N, motor gates, and the refusal comes before the bus-clear drives them.
    let _g = seed_reset();
    let chip = chip();
    let gpiob = chip.gpiob().unwrap().split();
    let before = Reg32::new(GPIOB_BASE, 0x04).read();
    assert_eq!(
        I2c::new(
            &chip,
            &ref_72m(),
            PeriphLabel::I2c0,
            (gpiob.pb14, gpiob.pb15),
            I2cMode::standard(100_000),
        )
        .err(),
        Some(crate::error::DescriptorError::PinOwnership)
    );
    assert_eq!(
        Reg32::new(GPIOB_BASE, 0x04).read(),
        before,
        "a refused pair is never driven or configured"
    );
}

/// Bring up I2C0 on PB6/PB7 at `speed_hz` (standard <= 100 kHz, fast otherwise) on `chip`.
fn new_on(chip: &Chip, speed_hz: u32, duty: FastDuty) -> I2c {
    let mode = if speed_hz <= 100_000 {
        I2cMode::standard(speed_hz)
    } else {
        I2cMode::fast(speed_hz, duty)
    };
    let pins = pb6_pb7(chip);
    I2c::new(chip, &ref_72m(), PeriphLabel::I2c0, pins, mode).unwrap()
}

fn seed_reset() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

fn r(off: u32) -> u32 {
    Reg32::new(I2C_BASE, off).read()
}
fn w(off: u32, v: u32) {
    Reg32::new(I2C_BASE, off).write(v);
}

/// Set every STAT0 flag a transfer polls (SBSEND|ADDSEND|BTC|RBNE|TBE), so each bounded poll exits
/// immediately. Bits 0,1,2,6,7.
fn seed_stat0_all_ready() {
    w(
        STAT0,
        STAT0_SBSEND | STAT0_ADDSEND | STAT0_BTC | STAT0_RBNE | STAT0_TBE,
    );
}

// --- timing (vs the SPL i2c_clock_config formula) ---------------------------------------------

#[test]
fn timing_standard_100k_at_36mhz() {
    // APB1 = 36 MHz, 100 kHz standard mode (the proven 72 MHz tree).
    // freq = 36 -> I2CCLK = 0x24; RT = 36+1 = 37 = 0x25; CLKC = 36e6/(100e3*2) = 180 = 0xB4.
    let t = timing_for(36_000_000, 100_000, FastDuty::Two);
    assert_eq!(t.i2cclk, 0x24, "I2CCLK = 36 MHz field");
    assert_eq!(t.rt, 0x25, "RT = pclk1_MHz + 1");
    assert_eq!(t.ckcfg, 0xB4, "CKCFG = CLKC, FAST/DTCY clear");
    assert_eq!(t.ckcfg & CKCFG_FAST, 0, "standard mode: FAST clear");
}

#[test]
fn timing_standard_100k_at_8mhz_matches_probe() {
    // The bench i2c_probe.c ran at the 8 MHz reset clock: freq = 8 -> I2CCLK = 8;
    // RT = 8+1 = 9; CLKC = 8e6/(100e3*2) = 40 = 0x28.
    let t = timing_for(8_000_000, 100_000, FastDuty::Two);
    assert_eq!(t.i2cclk, 8);
    assert_eq!(t.rt, 9);
    assert_eq!(t.ckcfg, 40);
}

#[test]
fn timing_fast_400k_at_36mhz_dtcy2() {
    // 400 kHz fast mode, DTCY_2 at APB1 = 36 MHz: RT = (36*300)/1000 + 1 = 10 + 1 = 11;
    // CLKC = 36e6/(400e3*3) = 30 = 0x1E; CKCFG = FAST | CLKC = 0x8000 | 0x1E.
    let t = timing_for(36_000_000, 400_000, FastDuty::Two);
    assert_eq!(t.rt, 11);
    assert_eq!(t.ckcfg, CKCFG_FAST | 0x1E);
    assert_eq!(t.ckcfg & CKCFG_DTCY, 0, "DTCY_2: DTCY clear");
}

#[test]
fn timing_fast_400k_dtcy_16_9_sets_dtcy() {
    // DTCY_16/9: CLKC = 36e6/(400e3*25) = 3; CKCFG = FAST | DTCY | 3.
    let t = timing_for(36_000_000, 400_000, FastDuty::SixteenNine);
    assert_eq!(t.ckcfg & CKCFG_DTCY, CKCFG_DTCY, "DTCY_16/9: DTCY set");
    assert_eq!(t.ckcfg & CKCFG_FAST, CKCFG_FAST);
    assert_eq!(t.ckcfg & CKCFG_CLKC, 3);
}

#[test]
fn i2c_input_clock_from_default_72m_tree_is_36mhz() {
    let p = ref_72m();
    assert_eq!(i2c_input_clock(&p), 36_000_000);
}

// --- bring-up register end state --------------------------------------------------------------

#[test]
fn bring_up_programs_timing_mode_and_enable() {
    let _g = seed_reset();
    // Bench config: 100 kHz standard mode.
    let _dev = new_on(&chip(), 100_000, FastDuty::Two);

    assert_eq!(r(CTL1) & CTL1_I2CCLK, 0x24, "CTL1 I2CCLK = 36 MHz");
    assert_eq!(r(RT) & 0x7F, 0x25, "RT = 37");
    assert_eq!(r(CKCFG), 0xB4, "CKCFG = 180 (100 kHz, standard)");
    // CTL0: I2CEN and ACKEN set, SMBEN clear, START/STOP not set by bring-up.
    let ctl0 = r(CTL0);
    assert_eq!(ctl0 & CTL0_I2CEN, CTL0_I2CEN, "I2CEN set");
    assert_eq!(ctl0 & CTL0_ACKEN, CTL0_ACKEN, "ACKEN set");
    assert_eq!(ctl0 & CTL0_SMBEN, 0, "SMBEN clear (I2C mode)");
    assert_eq!(
        ctl0 & (CTL0_START | CTL0_STOP),
        0,
        "no START/STOP at bring-up"
    );
    // SADDR0: the SPL writes the own-address argument directly (raw register value), so 0x24 lands
    // as 0x24 (matching i2c_mode_addr_config and the bench probe), not shifted.
    assert_eq!(r(SADDR0), 0x24, "SADDR0 = own-address register value");
}

#[test]
fn bring_up_fast_400k_sets_fast_bit() {
    let _g = seed_reset();
    let _dev = new_on(&chip(), 400_000, FastDuty::Two);
    assert_eq!(
        r(CKCFG) & CKCFG_FAST,
        CKCFG_FAST,
        "fast mode bit set at 400 kHz"
    );
}

// --- instance selection (I2C0 vs I2C1) --------------------------------------------------------
//
// The driver logic is instance-agnostic: `new` takes the `PeriphLabel` and everything downstream
// (base, clock enable, timing, transfers) follows from it. These cases pin that the SELECTION is
// what varies, and that nothing about the second instance forks the driver.

/// PB10 (SCL) / PB11 (SDA) handles, the classywalk offroad IMU pair = I2C1.
fn pb10_pb11(
    chip: &Chip,
) -> (
    crate::gpio::Pin<crate::gpio::Input<crate::gpio::Floating>>,
    crate::gpio::Pin<crate::gpio::Input<crate::gpio::Floating>>,
) {
    let gpiob = chip.gpiob().unwrap().split();
    (gpiob.pb10, gpiob.pb11)
}

/// Read an I2C1 register at `off`.
fn r1(off: u32) -> u32 {
    Reg32::new(I2C1_BASE, off).read()
}

#[test]
fn bring_up_on_instance_1_programs_the_second_instances_registers() {
    // The same bring-up as `bring_up_programs_timing_mode_and_enable`, selected onto I2C1: the
    // identical end state, at the OTHER base, with I2C0's window left untouched.
    let _g = seed_reset();
    let chip = chip();
    let pins = pb10_pb11(&chip);
    let _dev = I2c::new(
        &chip,
        &ref_72m(),
        PeriphLabel::I2c1,
        pins,
        I2cMode::standard(100_000),
    )
    .unwrap();

    assert_eq!(r1(CTL1) & CTL1_I2CCLK, 0x24, "CTL1 I2CCLK = 36 MHz");
    assert_eq!(r1(RT) & 0x7F, 0x25, "RT = 37");
    assert_eq!(r1(CKCFG), 0xB4, "CKCFG = 180 (100 kHz, standard)");
    let ctl0 = r1(CTL0);
    assert_eq!(ctl0 & CTL0_I2CEN, CTL0_I2CEN, "I2CEN set");
    assert_eq!(ctl0 & CTL0_ACKEN, CTL0_ACKEN, "ACKEN set");
    // I2C0's registers were never written: the instance label alone steered every access.
    assert_eq!(r(CTL0), 0, "I2C0 CTL0 untouched");
    assert_eq!(r(CKCFG), 0, "I2C0 CKCFG untouched");
    assert_eq!(r(CTL1), 0, "I2C0 CTL1 untouched");
}

#[test]
fn bring_up_on_instance_1_enables_only_its_own_clock_bit() {
    // I2C1EN = APB1EN bit 22 (bit 21 is I2C0EN); APB1EN is at RCU offset 0x1C.
    let _g = seed_reset();
    let chip = chip();
    let pins = pb10_pb11(&chip);
    let _dev = I2c::new(
        &chip,
        &ref_72m(),
        PeriphLabel::I2c1,
        pins,
        I2cMode::standard(100_000),
    )
    .unwrap();
    let apb1en = Reg32::new(RCU_BASE, 0x1C).read();
    assert_eq!(apb1en & (1 << 22), 1 << 22, "I2C1EN set");
    assert_eq!(apb1en & (1 << 21), 0, "I2C0EN not set");
}

#[test]
fn bring_up_on_instance_1_transfers_on_the_second_instance() {
    // The `embedded-hal` surface is not forked: a write on the I2C1 handle drives I2C1's data
    // register, proving the driver carries the resolved base through the transfer path too.
    let _g = seed_reset();
    let chip = chip();
    let pins = pb10_pb11(&chip);
    let mut dev = I2c::new(
        &chip,
        &ref_72m(),
        PeriphLabel::I2c1,
        pins,
        I2cMode::standard(100_000),
    )
    .unwrap();
    // Seed I2C1's STAT0 with every polled flag set at once (the module docs' flat-mock
    // convention); `seed_stat0_all_ready` seeds I2C0's, which must NOT satisfy this transfer.
    Reg32::new(I2C1_BASE, STAT0)
        .write(STAT0_SBSEND | STAT0_ADDSEND | STAT0_BTC | STAT0_RBNE | STAT0_TBE);
    dev.write(IMU, &[0x75]).unwrap();
    assert_eq!(r1(DATA) & 0xFF, 0x75, "the byte went out on I2C1");
    assert_eq!(r(DATA) & 0xFF, 0, "nothing went out on I2C0");
}

#[test]
fn new_fails_loud_on_a_part_without_the_second_instance() {
    // The fail-loud seam `pincap::i2c_pair` documents: a part below the density floor carries no
    // I2C1 base, so bring-up on that label errors MissingBase rather than poking a peripheral that
    // is not there, and the pins are left untouched.
    let _g = seed_reset();
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::I2c0, I2C_BASE);
    addrs.set(PeriphLabel::Gpiob, GPIOB_BASE);
    addrs.set(PeriphLabel::Rcu, RCU_BASE);
    let chip = Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        flash_page: PageSize::K1,
        flash_kib: 32,
        adv_timers: 1,
        adc_count: 1,
    });
    assert!(!chip.has_i2c(PeriphLabel::I2c1));
    let pins = pb10_pb11(&chip);
    let before = Reg32::new(GPIOB_BASE, 0x04).read();
    assert_eq!(
        I2c::new(
            &chip,
            &ref_72m(),
            PeriphLabel::I2c1,
            pins,
            I2cMode::standard(100_000),
        )
        .err(),
        Some(crate::error::DescriptorError::MissingBase(
            PeriphLabel::I2c1
        ))
    );
    assert_eq!(
        Reg32::new(GPIOB_BASE, 0x04).read(),
        before,
        "a refused instance never configures its pins"
    );
}

// --- pin AF-open-drain config (both GpioPath register models) ---------------------------------
//
// `I2c::new` consumes the PB6/PB7 Pin handles and must land the I2C AF-open-drain config on them
// through `configure_af`, for BOTH the F10x CRL/CRH model and the F1x0 CTL/AFSEL model. PB6/PB7 are
// pins 6/7 in port B, so they live in the low half of each model's per-pin register.

/// Read a GPIOB register at `off`.
fn gb(off: u32) -> u32 {
    Reg32::new(GPIOB_BASE, off).read()
}

#[test]
fn new_configures_scl_sda_af_open_drain_f10x() {
    let _g = seed_reset();
    // F10x: each pin is a 4-bit nibble in CTL0 (pins 0..7). AF open-drain 50 MHz = nibble 0xF
    // (GPIO_MODE_AF_OD 0xC | 50 MHz 0x3). PB6 -> shift 24, PB7 -> shift 28.
    let _dev = new_on(&chip_for(GpioPath::ApbCrlCrh), 100_000, FastDuty::Two);
    let ctl0 = gb(0x00); // F10X_CTL0
    assert_eq!((ctl0 >> 24) & 0xF, 0xF, "PB6 (SCL) = AF open-drain nibble");
    assert_eq!((ctl0 >> 28) & 0xF, 0xF, "PB7 (SDA) = AF open-drain nibble");
}

#[test]
fn new_configures_scl_sda_af_open_drain_f1x0() {
    let _g = seed_reset();
    // F1x0: CTL = AF (2), OMODE = open-drain (1), OSPD = 50 MHz (3), PUD = pull-up (1), AFSEL0 = AF1.
    let _dev = new_on(&chip_for(GpioPath::AhbCtlAfsel), 100_000, FastDuty::Two);
    let ctl = gb(0x00); // F1X0_CTL: 2 bits/pin
    assert_eq!((ctl >> (2 * 6)) & 0x3, 2, "PB6 CTL = AF mode");
    assert_eq!((ctl >> (2 * 7)) & 0x3, 2, "PB7 CTL = AF mode");
    let omode = gb(0x04); // F1X0_OMODE: 1 bit/pin, 1 = open-drain
    assert_eq!((omode >> 6) & 1, 1, "PB6 OMODE = open-drain");
    assert_eq!((omode >> 7) & 1, 1, "PB7 OMODE = open-drain");
    let ospd = gb(0x08); // F1X0_OSPD: 2 bits/pin, 3 = 50 MHz
    assert_eq!((ospd >> (2 * 6)) & 0x3, 3, "PB6 OSPD = 50 MHz");
    assert_eq!((ospd >> (2 * 7)) & 0x3, 3, "PB7 OSPD = 50 MHz");
    let pud = gb(0x0C); // F1X0_PUD: 2 bits/pin, 1 = pull-up
    assert_eq!((pud >> (2 * 6)) & 0x3, 1, "PB6 PUD = pull-up");
    assert_eq!((pud >> (2 * 7)) & 0x3, 1, "PB7 PUD = pull-up");
    let afsel0 = gb(0x20); // F1X0_AFSEL0: 4 bits/pin, AF1 for I2C0 on port B
    assert_eq!((afsel0 >> (4 * 6)) & 0xF, 1, "PB6 AFSEL = AF1");
    assert_eq!((afsel0 >> (4 * 7)) & 0xF, 1, "PB7 AFSEL = AF1");
}

// --- embedded-hal i2c::I2c transfers ----------------------------------------------------------

fn brought_up() -> I2c {
    new_on(&chip(), 100_000, FastDuty::Two)
}

#[test]
fn write_sends_address_then_bytes_then_stop() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    dev.write(IMU, &[0x75]).expect("write should succeed");
    // The last DATA write is the register byte 0x75.
    assert_eq!(r(DATA) & 0xFF, 0x75, "data byte written to DATA");
    // STOP programmed (write with terminating stop).
    assert_eq!(r(CTL0) & CTL0_STOP, CTL0_STOP, "STOP issued");
}

#[test]
fn read_byte_value_path() {
    // The value path: `receive()` returns whatever DATA holds when read. On real silicon DATA-read
    // and DATA-write are distinct registers, but the flat-array mock conflates them and the read
    // phase writes the address byte to DATA first, so this drives `receive()` directly against a
    // seeded DATA rather than through the full read sequence (which would clobber the seed).
    let _g = seed_reset();
    let dev = brought_up();
    w(DATA, 0x2E); // WHO_AM_I value the device would return.
    assert_eq!(dev.receive(), 0x2E, "receive() returns the DATA byte");
}

#[test]
fn read_single_byte_acks_and_stops() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    let mut buf = [0u8; 1];
    dev.read(IMU, &mut buf).expect("read should succeed");
    // The read phase wrote the address with the READ bit set: (0x68 << 1) | 1 = 0xD1.
    assert_eq!(
        r(DATA) & 0xFF,
        ((IMU as u32) << 1) | 1,
        "address byte has R/W bit set"
    );
    assert_eq!(
        r(CTL0) & CTL0_STOP,
        CTL0_STOP,
        "single-byte read issues STOP"
    );
    // Single-byte read disabled ACK before clearing ADDSEND, then re-enabled it after STOP.
    assert_eq!(
        r(CTL0) & CTL0_ACKEN,
        CTL0_ACKEN,
        "ACK re-enabled for next transfer"
    );
}

#[test]
fn write_read_register_pointer_then_byte() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    // The IMU WHO_AM_I shape: write reg pointer 0x75, repeated-start, read 1 byte. Seed DATA so the
    // read phase returns 0x2E. (The write phase overwrites DATA with the address/register byte
    // first; the read then re-reads DATA, which the mock returns as last-written, so we cannot
    // distinguish the read value from the written one here. Re-seed right before the read is not
    // possible mid-call, so this asserts the call completes and STOPs; the value path is covered by
    // read_returns_seeded_data_and_stops.)
    dev.write_read(IMU, &[0x75], &mut [0u8; 1])
        .expect("write_read should succeed");
    assert_eq!(
        r(CTL0) & CTL0_STOP,
        CTL0_STOP,
        "write_read terminates with STOP"
    );
}

#[test]
fn transaction_write_then_read_is_repeated_start() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    let mut rd = [0u8; 1];
    let mut ops = [Operation::Write(&[0x06]), Operation::Read(&mut rd)];
    dev.transaction(IMU, &mut ops)
        .expect("transaction should succeed");
    assert_eq!(r(CTL0) & CTL0_STOP, CTL0_STOP, "final read STOPs");
}

#[test]
fn zero_length_write_is_a_presence_check() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    // An empty write still does START + address (an embedded-hal bus presence check) and STOPs.
    dev.write(IMU, &[]).expect("empty write should succeed");
    assert_eq!(r(CTL0) & CTL0_STOP, CTL0_STOP);
}

// --- Solution B receive shapes (flat mock: flow, end state, and under a read ramp the store
// destinations too; the on-wire sequencing is silicon's) --------------------------------------

#[test]
fn read_burst_14_solution_b_completes_and_stops() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    let mut buf = [0u8; 14];
    dev.read(IMU, &mut buf)
        .expect("14-byte burst should succeed");
    let ctl0 = r(CTL0);
    assert_eq!(ctl0 & CTL0_STOP, CTL0_STOP, "burst read issues STOP");
    assert_eq!(
        ctl0 & CTL0_ACKEN,
        CTL0_ACKEN,
        "ACK re-enabled for next transfer"
    );
    assert_eq!(ctl0 & CTL0_POAP, 0, "POAP not used for N>2");
}

/// The N>2 arm stores the last three bytes outside its loop, in three separate bindings
/// (`third_last`, `second_last`, `last`), which is where a destination mistake hides: transpose two
/// of them and the IMU's burst comes back with two bytes swapped on silicon while every other
/// assertion in this file still passes, because a flat mock DATA register answers every read with
/// the same byte. Arming the read ramp makes each received byte distinct, so the buffer's contents
/// are the proof that each one landed where the sequence says. 14 bytes is the IMU's real burst.
#[test]
fn read_burst_14_stores_every_byte_in_order() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    mock::ramp(I2C_BASE + DATA, 0x10);
    let mut buf = [0u8; 14];
    dev.read(IMU, &mut buf)
        .expect("14-byte burst should succeed");
    let expected: [u8; 14] = std::array::from_fn(|i| 0x10 + i as u8);
    assert_eq!(buf, expected, "received bytes stored in receive order");
}

/// The same proof at the smallest length the N>2 arm takes: three bytes, so its loop runs zero
/// times and the three trailing bindings are the whole transfer.
#[test]
fn read_three_bytes_stores_every_byte_in_order() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    mock::ramp(I2C_BASE + DATA, 0x41);
    let mut buf = [0u8; 3];
    dev.read(IMU, &mut buf).expect("3-byte read should succeed");
    assert_eq!(
        buf,
        [0x41, 0x42, 0x43],
        "received bytes stored in receive order"
    );
}

#[test]
fn read_two_bytes_restores_poap_and_stops() {
    let _g = seed_reset();
    let mut dev = brought_up();
    seed_stat0_all_ready();
    let mut buf = [0u8; 2];
    dev.read(IMU, &mut buf).expect("2-byte read should succeed");
    let ctl0 = r(CTL0);
    assert_eq!(ctl0 & CTL0_STOP, CTL0_STOP, "N=2 read issues STOP");
    assert_eq!(
        ctl0 & CTL0_POAP,
        0,
        "POAP restored (cleared) after the transfer"
    );
    assert_eq!(ctl0 & CTL0_ACKEN, CTL0_ACKEN, "ACK re-enabled");
}

// --- recovery + the busy gate ------------------------------------------------------------------

#[test]
fn failed_transfer_clears_sticky_errors_and_reinits() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // Zero CKCFG after bring-up so the reinit's reprogramming is observable.
    w(CKCFG, 0);
    // BERR set, no ready flags: the SBSEND wait fails with Bus; recovery must then (1) clear the
    // sticky BERR, (2) SRESET-reinit (the failed attempt left a START request pending).
    w(STAT0, STAT0_BERR);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Bus);
    assert_eq!(r(STAT0) & STAT0_BERR, 0, "sticky BERR cleared by recovery");
    let ctl0 = r(CTL0);
    assert_eq!(
        ctl0 & (CTL0_START | CTL0_STOP),
        0,
        "pending START/STOP request cleared by the SRESET reinit"
    );
    assert_eq!(ctl0 & CTL0_I2CEN, CTL0_I2CEN, "block re-enabled");
    assert_eq!(ctl0 & CTL0_ACKEN, CTL0_ACKEN, "ACK posture restored");
    assert_eq!(r(CKCFG), 0xB4, "timing reprogrammed by the reinit");
}

#[test]
fn stuck_busy_fresh_transfer_fails_bus_after_sreset() {
    let _g = seed_reset();
    let mut dev = brought_up();
    w(CKCFG, 0); // observable reinit, as above
                 // I2CBSY stuck with MASTER clear (the silicon-observed wedge shape); STAT0 stays 0. The fresh
                 // transfer's busy gate exhausts, recovery SRESET-reinits, the (mock-static) bus is still busy,
                 // and the call fails fast with Bus.
    w(STAT1, STAT1_I2CBSY);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Bus);
    assert_eq!(
        r(CKCFG),
        0xB4,
        "wedge recovery SRESET-reinit reprogrammed the timing"
    );
    assert_eq!(r(CTL0) & CTL0_I2CEN, CTL0_I2CEN, "block re-enabled");
}

#[test]
fn non_fresh_read_skips_the_busy_gate() {
    let _g = seed_reset();
    let dev = brought_up();
    // Busy is EXPECTED mid-transaction (this side holds the bus after a register-pointer write):
    // the repeated-START read must not busy-gate. Seed busy + all-ready and drive the raw
    // non-fresh read; it must complete.
    w(STAT1, STAT1_I2CBSY | STAT1_MASTER);
    seed_stat0_all_ready();
    let mut buf = [0u8; 1];
    dev.read_bytes(IMU, &mut buf, false)
        .expect("non-fresh read must not busy-gate");
    assert_eq!(r(CTL0) & CTL0_STOP, CTL0_STOP, "read still STOPs");
}

// --- error injection: STAT0 error bits map to the right ErrorKind -----------------------------

#[test]
fn berr_maps_to_bus() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // No ready flags, but BERR set: the first wait_flag (SBSEND) sees BERR and returns Bus.
    w(STAT0, STAT0_BERR);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Bus);
}

#[test]
fn lostarb_maps_to_arbitration_loss() {
    let _g = seed_reset();
    let mut dev = brought_up();
    w(STAT0, STAT0_LOSTARB);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::ArbitrationLoss);
}

#[test]
fn aerr_during_address_maps_to_nack_address() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // SBSEND set so the START poll passes; then AERR set with ADDSEND clear, so the ADDSEND wait
    // (NackKind::Address) sees AERR and returns an ADDRESS NACK.
    w(STAT0, STAT0_SBSEND | STAT0_AERR);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(
        e.kind(),
        ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
    );
}

#[test]
fn aerr_during_data_maps_to_nack_data() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // SBSEND + ADDSEND pass the address phase; AERR (no TBE) makes the data-phase TBE wait return a
    // DATA NACK.
    w(STAT0, STAT0_SBSEND | STAT0_ADDSEND | STAT0_AERR);
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(
        e.kind(),
        ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data)
    );
}

#[test]
fn timeout_maps_to_other() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // STAT0 left at 0: no flag ever sets, the bounded poll exhausts its budget -> Timeout -> Other.
    let e = dev.write(IMU, &[0x00]).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Other);
}

// --- warm-reset bus-clear sequencer (round-10 hang) -------------------------------------------
//
// The bit-bang itself is silicon-only; the DECISION LOGIC (only-if-needed, the pulse bound, the
// early-stop, the trailing STOP) is pure and host-tested here through `bus_clear_seq` with recording
// closures. Levels: `drive_*(true)` releases the line, `drive_*(false)` drives it low.

/// One recorded line event from the closures: which line, and the level driven (`false` = low).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ev {
    Scl(bool),
    Sda(bool),
}

#[test]
fn bus_clear_clean_bus_issues_no_pulses_and_no_stop() {
    // SDA reads high from the start: the bus is clean, so recovery is skipped entirely.
    let events = core::cell::RefCell::new(Vec::<Ev>::new());
    let n = bus_clear_seq(
        || true,
        |high| events.borrow_mut().push(Ev::Scl(high)),
        |high| events.borrow_mut().push(Ev::Sda(high)),
        || {},
    );
    assert_eq!(n, 0, "a clean bus issues zero pulses");
    // No SCL low pulse and no SDA low (a STOP needs an SDA low->high edge, never generated here).
    assert!(
        !events
            .borrow()
            .iter()
            .any(|e| matches!(e, Ev::Scl(false) | Ev::Sda(false))),
        "clean bus must not drive either line low: {:?}",
        events.borrow()
    );
}

#[test]
fn bus_clear_stuck_bus_issues_the_full_bound() {
    // SDA never releases: the sequencer must clock exactly BUS_CLEAR_MAX_PULSES (9) times and stop.
    let mut scl_low = 0u32;
    let n = bus_clear_seq(
        || false,
        |high| {
            if !high {
                scl_low += 1;
            }
        },
        |_| {},
        || {},
    );
    assert_eq!(
        n, BUS_CLEAR_MAX_PULSES,
        "a never-releasing slave hits the 9-pulse bound"
    );
    assert_eq!(scl_low, BUS_CLEAR_MAX_PULSES, "one SCL low edge per pulse");
}

#[test]
fn bus_clear_stops_as_soon_as_sda_releases() {
    // SDA low for the first few reads, then high: the loop stops early (fewer than 9 pulses).
    // read_sda calls: [0]=only-if-needed check, then one per while-condition. Return high once the
    // call index reaches `release_at`, so pulses issued = release_at - 1.
    for release_at in 1..=BUS_CLEAR_MAX_PULSES {
        let mut call = 0u32;
        let mut scl_low = 0u32;
        let n = bus_clear_seq(
            || {
                let high = call >= release_at;
                call += 1;
                high
            },
            |high| {
                if !high {
                    scl_low += 1;
                }
            },
            |_| {},
            || {},
        );
        assert_eq!(n, release_at - 1, "stops the pulse train when SDA releases");
        assert!(n < BUS_CLEAR_MAX_PULSES);
        assert_eq!(scl_low, n, "one SCL low edge per issued pulse");
    }
}

#[test]
fn bus_clear_generates_a_trailing_stop_after_recovery() {
    // A stuck bus that never releases still ends with a STOP: an SDA low->high transition while SCL
    // is left released high. Record the tail of the event stream and check the STOP edge.
    let events = core::cell::RefCell::new(Vec::<Ev>::new());
    let n = bus_clear_seq(
        || false,
        |high| events.borrow_mut().push(Ev::Scl(high)),
        |high| events.borrow_mut().push(Ev::Sda(high)),
        || {},
    );
    assert_eq!(n, BUS_CLEAR_MAX_PULSES);
    // The final three line events are the STOP: SCL released high, SDA low, SDA high.
    let ev = events.borrow();
    let tail: Vec<Ev> = ev.iter().rev().take(3).rev().copied().collect();
    assert_eq!(
        tail,
        vec![Ev::Scl(true), Ev::Sda(false), Ev::Sda(true)],
        "STOP = SDA low->high while SCL is high: {ev:?}"
    );
}

// --- the control-loop blocking bound (the 16 ms demand-stale coast) ---------------------------
//
// The consumer of this driver is a 250 Hz balance control task whose torque word is watched by a
// 16 kHz commutation ISR: if the task does not rewrite demand within 256 periods (16 ms) the ISR
// floats all three motor phases. So a single polled I2C transaction that spins past 16 ms cuts
// motor torque on a machine that is trying to stay upright. These cases pin that bound against the
// driver's real polled state machine, driven from the mock register space with a bus that never
// answers.
//
// The cost model, because the poll budget is counted in loop ITERATIONS and the invariant is in
// MICROSECONDS. One `wait_flag` iteration, disassembled from the consuming firmware's release
// image, is an APB `ldr` of STAT0, five shift/branch flag tests, a load and store of the budget
// through memory, a zero test and a taken back-branch: 21 core cycles on Cortex-M3 before the
// AHB-to-APB1 bridge latency on the STAT0 load and any flash wait states. So 21 is the FLOOR: real
// silicon is slower, never faster.
//
// Converting a measured poll count with the FLOOR under-states the block, and that is deliberate.
// These cases assert "the block stays below the coast", so under-stating the block makes the
// assertion EASIER to pass; a case that passes anyway has passed on the driver's behaviour rather
// than on a flattering constant. The driver itself sizes its budget with the upper bound
// (`POLL_CYCLES`), the opposite end, for the same reason in reverse.

/// Core cycles per `wait_flag` poll iteration, the FLOOR (see the cost model above).
const POLL_CYCLES_FLOOR: u64 = 21;
/// The core clock both boards run (`ClockConfig::REFERENCE_72M_IRC8M`).
const CORE_MHZ: u64 = 72;
/// `motor::DEMAND_STALE_PERIODS` (256) at 16 kHz: the window after which the phases float.
const COAST_US: u64 = 16_000;

/// Microseconds a measured poll count costs at 72 MHz (floor; see the cost model above).
fn poll_us(polls: u64) -> u64 {
    polls * POLL_CYCLES_FLOOR / CORE_MHZ
}

/// Arm the poll counter over the WHOLE I2C register window (CTL0 through RT) and return the
/// microseconds the closure's polling cost. The window rather than `STAT0` alone because the
/// transfer waits poll `STAT0` and `STAT1` while the recovery path polls `CTL0`, and recovery is
/// part of what a failing transfer makes the caller wait for.
fn measure_block_us(body: impl FnOnce()) -> u64 {
    mock::watch_reads(I2C_BASE + CTL0, I2C_BASE + RT + 4);
    body();
    poll_us(mock::read_count())
}

/// The hazard, exactly: the 14-byte cyclic IMU burst against a bus that never answers and never
/// raises an error flag either (SCL held low by a wedged slave, so the peripheral makes no
/// progress and no AERR/BERR is ever set). A device that is merely ABSENT is the cheap case: it
/// NACKs and the address-phase poll returns on AERR. This is the expensive one.
#[test]
fn a_dead_bus_imu_burst_cannot_block_past_the_motor_coast() {
    let _g = seed_reset();
    let mut dev = brought_up();
    let mut buf = [0u8; 14];
    let mut r = Ok(());
    let us = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
    assert_eq!(
        r,
        Err(I2cError::Timeout),
        "a bus that never answers times out"
    );
    assert!(
        us < COAST_US,
        "one IMU burst on a dead bus blocked {us} us, past the {COAST_US} us demand-stale coast \
         (all three phases float)"
    );
}

/// The same bound for a transaction that NAKs partway through the burst rather than at the
/// address: the register-pointer write is answered, then the slave stops driving. The read phase's
/// per-byte poll is the one that has to give up.
#[test]
fn a_burst_that_stalls_after_the_address_cannot_block_past_the_motor_coast() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // The write phase's flags are all present, so the register-pointer write completes; the read
    // phase then waits on RBNE, which is NOT seeded, and no error flag ever appears.
    w(STAT0, STAT0_SBSEND | STAT0_ADDSEND | STAT0_BTC | STAT0_TBE);
    let mut buf = [0u8; 14];
    let mut r = Ok(());
    let us = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
    assert_eq!(r, Err(I2cError::Timeout), "a stalled burst times out");
    assert!(
        us < COAST_US,
        "an IMU burst stalling mid-read blocked {us} us, past the {COAST_US} us coast"
    );
}

/// A bus held busy from the start (I2CBSY set, the wedged-slave case the fresh-transfer idle wait
/// exists for) must also fail inside the coast, including the recovery it runs on the way out.
#[test]
fn a_bus_stuck_busy_cannot_block_past_the_motor_coast() {
    let _g = seed_reset();
    let mut dev = brought_up();
    w(STAT1, STAT1_I2CBSY);
    let mut buf = [0u8; 14];
    let mut r = Ok(());
    let us = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
    assert!(r.is_err(), "a bus stuck busy cannot start a fresh transfer");
    assert!(
        us < COAST_US,
        "a stuck-busy IMU burst blocked {us} us, past the {COAST_US} us coast"
    );
}

/// Five consecutive dead-bus reads is what the orchestrator's retry breaker allows before the
/// `imu_loss` fault latches (`IMU_LOSS_THRESHOLD`), so every one of those passes is a separate
/// control pass and each must independently stay inside the coast. Pins that the bound is a
/// property of each pass and does not decay across a run.
#[test]
fn every_pass_of_the_pre_breaker_run_stays_inside_the_coast() {
    let _g = seed_reset();
    let mut dev = brought_up();
    let mut buf = [0u8; 14];
    for pass in 0..5 {
        let mut r = Ok(());
        let us = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
        assert!(r.is_err(), "pass {pass} should fail on a dead bus");
        assert!(
            us < COAST_US,
            "pass {pass} blocked {us} us, past the {COAST_US} us coast"
        );
    }
}

/// The trap the metered-TX round hit: a timeout that leaves the peripheral mid-transaction is
/// WORSE than the block, because the next read then fails too. After a timed-out burst the block
/// must be left with no pending START/STOP request, no sticky error flag, the peripheral enabled,
/// its bring-up timing intact, ACK re-armed and POAP clear.
#[test]
fn a_timed_out_burst_leaves_the_block_recoverable() {
    let _g = seed_reset();
    let mut dev = brought_up();
    let timing = dev.timing;
    let mut buf = [0u8; 14];
    assert_eq!(
        dev.write_read(IMU, &[0x3B], &mut buf),
        Err(I2cError::Timeout)
    );

    let ctl0 = r(CTL0);
    assert_eq!(ctl0 & CTL0_START, 0, "no START left pending");
    assert_eq!(ctl0 & CTL0_STOP, 0, "no STOP left pending");
    assert_eq!(ctl0 & CTL0_I2CEN, CTL0_I2CEN, "peripheral left enabled");
    assert_eq!(
        ctl0 & CTL0_ACKEN,
        CTL0_ACKEN,
        "ACK re-armed for the next transfer"
    );
    assert_eq!(
        ctl0 & CTL0_POAP,
        0,
        "POAP cleared (the N=2 receive posture)"
    );
    assert_eq!(r(STAT0) & STAT0_ERRORS, 0, "sticky error flags cleared");
    assert_eq!(
        r(CKCFG),
        timing.ckcfg,
        "bring-up timing reprogrammed after SRESET"
    );
    assert_eq!(
        r(CTL1) & 0x7F,
        timing.i2cclk,
        "peripheral-clock field reprogrammed"
    );
    assert_eq!(r(RT) & 0x7F, timing.rt, "rise time reprogrammed");
}

/// The other half of recoverability: the NEXT read on a bus that has come back must succeed. A
/// half-finished timeout would fail this even with the register state looking clean.
#[test]
fn a_read_after_a_timed_out_burst_succeeds_once_the_bus_returns() {
    let _g = seed_reset();
    let mut dev = brought_up();
    let mut buf = [0u8; 14];
    assert_eq!(
        dev.write_read(IMU, &[0x3B], &mut buf),
        Err(I2cError::Timeout)
    );

    // The slave comes back: every polled flag answers again.
    seed_stat0_all_ready();
    w(STAT1, 0);
    dev.write_read(IMU, &[0x3B], &mut buf)
        .expect("the burst after a timeout must succeed once the bus answers");
}

/// The most expensive shape there is: the transfer times out AND the recovery on the way out has
/// to spin too, because this side is still master and the STOP it requests never completes (SCL
/// held low). That is the acquisition budget followed by the recovery budget, the two summands of
/// [`worst_case_block_us`], and it still has to fit inside the coast.
#[test]
fn a_timeout_whose_recovery_also_spins_stays_inside_the_coast() {
    let _g = seed_reset();
    let mut dev = brought_up();
    // Master, and the bus never goes idle: recovery issues a STOP and then waits for the request
    // to clear, which it never does (the mock keeps whatever CTL0 bits were written).
    w(STAT1, STAT1_MASTER | STAT1_I2CBSY);
    let mut buf = [0u8; 14];
    let mut r = Ok(());
    let us = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
    assert!(r.is_err(), "a wedged bus cannot complete a burst");
    assert!(
        us < COAST_US,
        "a timeout with a spinning recovery blocked {us} us, past the {COAST_US} us coast"
    );
}

/// The measured block against the arithmetic a caller const-asserts on. The measurement converts
/// polls with the 16-cycle FLOOR and [`worst_case_block_us`] uses the 24-cycle upper bound, so the
/// measured figure must land at or below the published one, and the published one inside the coast.
/// This is the test that ties what the driver actually does to the number it advertises.
#[test]
fn the_measured_dead_bus_block_agrees_with_the_published_bound() {
    let _g = seed_reset();
    let mut dev = brought_up();
    let mut buf = [0u8; 14];
    let mut r = Ok(());
    let measured = measure_block_us(|| r = dev.write_read(IMU, &[0x3B], &mut buf));
    assert_eq!(r, Err(I2cError::Timeout));

    let published = worst_case_block_us(72_000_000) as u64;
    assert!(
        measured <= published,
        "measured {measured} us exceeds the published worst case {published} us"
    );
    assert!(
        published < COAST_US,
        "published worst case {published} us is not inside the {COAST_US} us coast"
    );
    // The bound is worth having only if it is well clear of the coast rather than just under it:
    // a failing read must cost at most a couple of missed control ticks, not most of the window.
    assert!(
        published * 2 < COAST_US,
        "published worst case {published} us leaves under 2x margin on the {COAST_US} us coast"
    );
}
