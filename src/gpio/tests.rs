//! T4 host tests for the gpio path (run under the `mock` feature against the backing-array
//! register space). The milestone link is `USART1 TX = PA2, RX = PA3`; these configure exactly
//! those pins on both families and assert the exact resulting register bits the GD SPL produces
//! for "AF push-pull 50MHz TX" / "input RX". Width-strict: every GPIO config register here is
//! 32-bit and is read back as `Reg32` (catching a 16-vs-32-bit AF-register slip).
#![cfg(feature = "mock")]

use crate::descriptor::GpioPath;
use crate::gpio::{configure_af, configure_output, set_pin, GpioOutput, PinRole};
use crate::reg::{mock, Reg32};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use std::sync::MutexGuard;

/// Acquire the whole-case serialization lock and zero the register space. Hold the guard for the
/// rest of the case (the mock register space is shared across the multi-threaded test runner).
fn seed() -> MutexGuard<'static, ()> {
    let g = mock::lock();
    mock::reset();
    g
}

// A port base; the offsets are what the mock keys on.
const PORT_BASE: u32 = 0x4001_0800; // shape of an F10x GPIOA base; value irrelevant to the mock.

// F10x offsets.
const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
// F1x0 offsets.
const CTL: u32 = 0x00;
const OMODE: u32 = 0x04;
const OSPD: u32 = 0x08;
const PUD: u32 = 0x0C;
const AFSEL0: u32 = 0x20;
const AFSEL1: u32 = 0x24;

fn read(off: u32) -> u32 {
    Reg32::new(PORT_BASE, off).read()
}

// PA2 / PA3 logical pin bytes (port A = 0).
const PA2: u8 = 0x02;
const PA3: u8 = 0x03;

// --- F10x (apb_crl_crh) -----------------------------------------------------------------------

#[test]
fn f10x_tx_pa2_is_af_pp_50mhz_nibble() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA2, PinRole::Tx);
    // PA2 -> CTL0 nibble at bits [11:8]. AF push-pull (0x8) | 50MHz (0x3) = 0xB.
    assert_eq!(read(CTL0), 0xB << 8);
    assert_eq!(read(CTL1), 0);
}

#[test]
fn f10x_rx_pa3_is_floating_input_nibble() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA3, PinRole::Rx);
    // PA3 -> CTL0 nibble at bits [15:12]. Floating input = 0x4.
    assert_eq!(read(CTL0), 0x4 << 12);
}

#[test]
fn f10x_tx_pa2_plus_rx_pa3_combined() {
    // Seed the documented F10x CTL0 reset value (0x4444_4444: every pin floating input). The RMW
    // must leave all the other pins at 0x4 and place TX/RX nibbles correctly.
    let _g = seed();
    Reg32::new(PORT_BASE, CTL0).write(0x4444_4444);
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA2, PinRole::Tx);
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA3, PinRole::Rx);
    // PA2 nibble -> 0xB at [11:8]; PA3 -> 0x4 at [15:12] (which equals the reset nibble). Every
    // other pin keeps its reset nibble 0x4.
    let expected = (0x4444_4444u32 & !(0xFu32 << 8) & !(0xFu32 << 12)) | (0xB << 8) | (0x4 << 12);
    assert_eq!(read(CTL0), expected);
}

#[test]
fn f10x_high_pin_uses_ctl1() {
    let _g = seed();
    // PB10 (port irrelevant to mock): pin 10 -> CTL1 nibble at bits [(10-8)*4 = 8].
    let pb10 = (1u8 << 4) | 10;
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, pb10, PinRole::Tx);
    assert_eq!(read(CTL1), 0xB << 8);
    assert_eq!(read(CTL0), 0);
}

// --- F1x0 (ahb_ctl_afsel) ---------------------------------------------------------------------

#[test]
fn f1x0_tx_pa2_sets_ctl_af_afsel_omode_ospd() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, PA2, PinRole::Tx);
    // CTL: pin 2, 2 bits at [5:4], AF = 2.
    assert_eq!(read(CTL), 2 << 4);
    // AFSEL0: pin 2, 4 bits at [11:8], AF1 (USART).
    assert_eq!(read(AFSEL0), 1 << 8);
    assert_eq!(read(AFSEL1), 0);
    // OMODE: pin 2 bit cleared (push-pull = 0).
    assert_eq!(read(OMODE) & (1 << 2), 0);
    // OSPD: pin 2, 2 bits at [5:4], 50MHz = 3.
    assert_eq!(read(OSPD), 3 << 4);
    // PUD: pin 2, no pull = 0.
    assert_eq!(read(PUD) & (0x3 << 4), 0);
}

#[test]
fn f1x0_rx_pa3_is_af_input_no_output_options() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, PA3, PinRole::Rx);
    // CTL: pin 3, AF = 2 at bits [7:6].
    assert_eq!(read(CTL), 2 << 6);
    // AFSEL0: pin 3, AF1 at bits [15:12].
    assert_eq!(read(AFSEL0), 1 << 12);
    // RX does not drive the line: OSPD untouched (0), OMODE untouched (0).
    assert_eq!(read(OSPD), 0);
    assert_eq!(read(OMODE), 0);
    // PUD: pin 3, no pull.
    assert_eq!(read(PUD) & (0x3 << 6), 0);
}

#[test]
fn f1x0_tx_pa2_plus_rx_pa3_combined() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, PA2, PinRole::Tx);
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, PA3, PinRole::Rx);
    // CTL: PA2 AF at [5:4], PA3 AF at [7:6] -> (2<<4) | (2<<6).
    assert_eq!(read(CTL), (2 << 4) | (2 << 6));
    // AFSEL0: PA2 AF1 at [11:8], PA3 AF1 at [15:12].
    assert_eq!(read(AFSEL0), (1 << 8) | (1 << 12));
    // Only TX (PA2) gets output options.
    assert_eq!(read(OSPD), 3 << 4);
    assert_eq!(read(OMODE), 0);
}

#[test]
fn f1x0_high_pin_uses_afsel1() {
    let _g = seed();
    // Pin 9 -> AFSEL1 nibble at [(9-8)*4 = 4].
    let p9 = 9u8;
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, p9, PinRole::Tx);
    assert_eq!(read(AFSEL1), 1 << 4);
    assert_eq!(read(AFSEL0), 0);
    // CTL: pin 9, AF at [19:18].
    assert_eq!(read(CTL), 2 << 18);
}

// --- M2 T5: bus-pin AF config (I2C open-drain + pull-up, SPI AF push-pull) --------------------

// PB6 / PB7 logical pin bytes (port B = 1) for the IMU I2C0 link.
const PB6: u8 = (1u8 << 4) | 6;
const PB7: u8 = (1u8 << 4) | 7;
// SPI0 representative pins (port A): PA5 = SCK, PA6 = MISO, PA7 = MOSI.
const PA5: u8 = 0x05;
const PA6: u8 = 0x06;
const PA7: u8 = 0x07;

// --- F10x I2C open-drain ----------------------------------------------------------------------

#[test]
fn f10x_i2c_pb6_pb7_are_af_open_drain_nibble() {
    let _g = seed();
    // F10x I2C AF open-drain 50MHz = nibble 0xF (GPIO_MODE_AF_OD 0x1C -> 0xC | speed 0x3).
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PB6, PinRole::I2cAfOpenDrain);
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PB7, PinRole::I2cAfOpenDrain);
    // PB6 -> CTL0 nibble [27:24], PB7 -> [31:28].
    assert_eq!(read(CTL0), (0xF << 24) | (0xFu32 << 28));
    assert_eq!(read(CTL1), 0);
}

// --- F1x0 I2C open-drain + pull-up ------------------------------------------------------------

#[test]
fn f1x0_i2c_pb6_is_af_open_drain_pullup() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PB6,
        PinRole::I2cAfOpenDrain,
    );
    // CTL: pin 6, AF = 2 at [13:12].
    assert_eq!(read(CTL), 2 << 12);
    // AFSEL0: pin 6, AF1 at [27:24].
    assert_eq!(read(AFSEL0), 1 << 24);
    assert_eq!(read(AFSEL1), 0);
    // OMODE: pin 6 bit SET (open-drain = 1).
    assert_eq!(read(OMODE) & (1 << 6), 1 << 6);
    // OSPD: pin 6, 50MHz = 3 at [13:12].
    assert_eq!(read(OSPD), 3 << 12);
    // PUD: pin 6, pull-up = 1 at [13:12].
    assert_eq!(read(PUD) & (0x3 << 12), 1 << 12);
}

#[test]
fn f1x0_i2c_pb6_pb7_combined() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PB6,
        PinRole::I2cAfOpenDrain,
    );
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PB7,
        PinRole::I2cAfOpenDrain,
    );
    assert_eq!(read(CTL), (2 << 12) | (2 << 14));
    assert_eq!(read(AFSEL0), (1 << 24) | (1u32 << 28));
    // Both open-drain: OMODE bits 6 and 7 set.
    assert_eq!(read(OMODE), (1 << 6) | (1 << 7));
    // Both 50MHz: OSPD pins 6,7.
    assert_eq!(read(OSPD), (3 << 12) | (3 << 14));
    // Both pull-up: PUD pins 6,7.
    assert_eq!(read(PUD), (1 << 12) | (1 << 14));
}

// --- F1x0 SPI AF push-pull (AF0) --------------------------------------------------------------

#[test]
fn f1x0_spi_sck_mosi_are_af_pp_af0() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PA5,
        PinRole::SpiAfPushPull,
    ); // SCK
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PA7,
        PinRole::SpiAfPushPull,
    ); // MOSI
       // CTL: PA5 AF at [11:10], PA7 AF at [15:14].
    assert_eq!(read(CTL), (2 << 10) | (2 << 14));
    // AFSEL0: SPI0 is AF0, so the nibbles are 0 (no AF bits set). The CTL=AF still routes them.
    assert_eq!(read(AFSEL0), 0);
    // Push-pull: OMODE untouched (0).
    assert_eq!(read(OMODE), 0);
    // 50MHz: OSPD pins 5,7.
    assert_eq!(read(OSPD), (3 << 10) | (3 << 14));
    // No pull.
    assert_eq!(read(PUD), 0);
}

#[test]
fn f1x0_spi_miso_is_af_input_af0() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, PA6, PinRole::SpiInput);
    // CTL: PA6 AF at [13:12].
    assert_eq!(read(CTL), 2 << 12);
    assert_eq!(read(AFSEL0), 0); // AF0
                                 // Input: no output options.
    assert_eq!(read(OSPD), 0);
    assert_eq!(read(OMODE), 0);
    assert_eq!(read(PUD), 0);
}

// --- F10x SPI AF push-pull --------------------------------------------------------------------

#[test]
fn f10x_spi_sck_mosi_are_af_pp_nibble() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA5, PinRole::SpiAfPushPull); // SCK
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA7, PinRole::SpiAfPushPull); // MOSI
                                                                               // AF push-pull 50MHz = 0xB. PA5 -> [23:20], PA7 -> [31:28].
    assert_eq!(read(CTL0), (0xB << 20) | (0xBu32 << 28));
}

#[test]
fn f10x_spi_miso_is_floating_input_nibble() {
    let _g = seed();
    configure_af(PORT_BASE, GpioPath::ApbCrlCrh, PA6, PinRole::SpiInput);
    // Floating input = 0x4. PA6 -> CTL0 [27:24].
    assert_eq!(read(CTL0), 0x4 << 24);
}

// --- M3 DR-T4: advanced-timer complementary-PWM gate pins (AF2 on F1x0) -----------------------
//
// The six gate pins (high CH0/1/2 on PA8/9/10, low CH0N/1N/2N on PB13/14/15) are AF push-pull
// 50 MHz on AF2 (TIMER0 on F1x0). Same drive as SPI SCK/MOSI; only the F1x0 AF-mux number differs
// (AF2 vs AF0). The M3 bench firmware set this AF mux with a raw Reg32 write; this role closes that
// gap. PA8/PA10 (high pins) and PB13 are the representative pins exercised here.

const PA8: u8 = 0x08;
const PA10: u8 = 0x0A;
// PB13 logical byte (port B = 1).
const PB13: u8 = (1u8 << 4) | 13;

#[test]
fn f1x0_timer_gate_pa8_is_af_pp_af2() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PA8,
        PinRole::TimerAfPushPull,
    );
    // CTL: pin 8, 2 bits at [17:16], AF = 2.
    assert_eq!(read(CTL), 2 << 16);
    // AFSEL1: pin 8 -> (8-8)*4 = nibble at [3:0], AF2.
    assert_eq!(read(AFSEL1), F1X0_AF2);
    assert_eq!(read(AFSEL0), 0);
    // Push-pull: OMODE pin 8 cleared.
    assert_eq!(read(OMODE) & (1 << 8), 0);
    // OSPD: pin 8, 50MHz = 3 at [17:16].
    assert_eq!(read(OSPD), 3 << 16);
    // No pull.
    assert_eq!(read(PUD) & (0x3 << 16), 0);
}

#[test]
fn f1x0_timer_gate_pb13_low_side_is_af_pp_af2() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PB13,
        PinRole::TimerAfPushPull,
    );
    // CTL: pin 13, 2 bits at [27:26], AF = 2.
    assert_eq!(read(CTL), 2 << 26);
    // AFSEL1: pin 13 -> (13-8)*4 = 20, nibble at [23:20], AF2.
    assert_eq!(read(AFSEL1), F1X0_AF2 << 20);
    assert_eq!(read(AFSEL0), 0);
    // Push-pull, 50MHz, no pull.
    assert_eq!(read(OMODE) & (1 << 13), 0);
    assert_eq!(read(OSPD), 3 << 26);
    assert_eq!(read(PUD) & (0x3 << 26), 0);
}

// AF2 numeric value (the F1x0 TIMER0 AF mux), local to the tests for readability.
const F1X0_AF2: u32 = 2;

// --- G3: general-purpose-timer PWM routing (TIMER1_CH1 -> PB3) ---------------------------------
//
// The cold-path general PWM fades the green LED on PB3. The two families route the SAME timer
// channel to the SAME pin by DIFFERENT mechanisms (the deliberate visible difference):
//  - F1x0: per-pin AFSEL mux. PB3 -> TIMER1_CH1 is AF2 (GD32F130xx Datasheet Port B AF summary).
//  - F10x: AFIO TIMER1_REMAP partial remap (AFIO_PCF0[9:8] = 01 = SPL GPIO_TIMER1_PARTIAL_REMAP0)
//    + the CRL AF push-pull nibble.

// PB3 logical byte (port B = 1, pin 3).
const PB3: u8 = (1u8 << 4) | 3;

#[test]
fn f1x0_gen_timer_pb3_is_af_pp_af2() {
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PB3,
        PinRole::GenTimerAfPushPull,
    );
    // CTL: pin 3, 2 bits at [7:6], AF = 2.
    assert_eq!(read(CTL), 2 << 6);
    // AFSEL0: pin 3 -> 4*3 = 12, nibble at [15:12], AF2 (PB3 = TIMER1_CH1 on AF2).
    assert_eq!(read(AFSEL0), F1X0_AF2 << 12);
    assert_eq!(read(AFSEL1), 0);
    // Push-pull, 50 MHz, no pull.
    assert_eq!(read(OMODE) & (1 << 3), 0);
    assert_eq!(read(OSPD), 3 << 6);
    assert_eq!(read(PUD) & (0x3 << 6), 0);
}

#[test]
fn f10x_gen_timer_pb3_is_af_pp_nibble() {
    let _g = seed();
    // On F10x the AF is implied by the CRL nibble (AF push-pull 50 MHz = 0xB); the per-pin AF mux
    // does not exist. PB3 is pin 3 -> CTL0 nibble at [15:12].
    configure_af(
        PORT_BASE,
        GpioPath::ApbCrlCrh,
        PB3,
        PinRole::GenTimerAfPushPull,
    );
    assert_eq!(read(CTL0), 0xB << 12);
    assert_eq!(read(CTL1), 0);
}

// --- general-purpose push-pull output (configure_output / set_pin / GpioOutput) ---------------
//
// The blinky / firmware indicator path: a plain digital output that owns the F10x/F1x0 register
// branch internally. Assert the exact GPIO config bits for BOTH register models, plus the atomic
// BOP set/reset write that drives the pin. PB9 (the firmware's indicator pin) is the worked pin.

// BOP (bit set/reset) register offsets per family.
const F10X_BOP: u32 = 0x10;
const F1X0_BOP: u32 = 0x18;

// PB9 logical byte (port B = 1); pin number 9 is in the high (CTL1 / AFSEL1) half.
const PB9: u8 = (1u8 << 4) | 9;

#[test]
fn f10x_output_pb9_is_gp_push_pull_50mhz_nibble() {
    let _g = seed();
    configure_output(PORT_BASE, GpioPath::ApbCrlCrh, PB9);
    // Pin 9 -> CTL1 nibble at (9-8)*4 = [7:4]. GP output push-pull 50MHz = 0x3.
    assert_eq!(read(CTL1), 0x3 << 4);
    assert_eq!(read(CTL0), 0);
}

#[test]
fn f10x_output_low_pin_uses_ctl0() {
    let _g = seed();
    // PA2 (pin 2) -> CTL0 nibble at [11:8].
    configure_output(PORT_BASE, GpioPath::ApbCrlCrh, PA2);
    assert_eq!(read(CTL0), 0x3 << 8);
    assert_eq!(read(CTL1), 0);
}

#[test]
fn f1x0_output_pb9_sets_ctl_omode_ospd() {
    let _g = seed();
    configure_output(PORT_BASE, GpioPath::AhbCtlAfsel, PB9);
    // CTL: pin 9, 2 bits at [19:18], output mode = 1.
    assert_eq!(read(CTL), 1 << 18);
    // OMODE: pin 9 bit cleared (push-pull = 0).
    assert_eq!(read(OMODE) & (1 << 9), 0);
    // OSPD: pin 9, 2 bits at [19:18], 50MHz = 3.
    assert_eq!(read(OSPD), 3 << 18);
    // PUD untouched (plain output, no pull configured).
    assert_eq!(read(PUD), 0);
    // No AF mux written for a plain output.
    assert_eq!(read(AFSEL1), 0);
    assert_eq!(read(AFSEL0), 0);
}

#[test]
fn set_pin_f10x_uses_bop_set_and_reset_halves() {
    let _g = seed();
    // High: BSx in the low half (1 << pin).
    set_pin(PORT_BASE, GpioPath::ApbCrlCrh, PB9, true);
    assert_eq!(read(F10X_BOP), 1 << 9);
    // Low: BCx in the high half (1 << (pin + 16)). BOP is write-only, so this overwrites.
    set_pin(PORT_BASE, GpioPath::ApbCrlCrh, PB9, false);
    assert_eq!(read(F10X_BOP), 1 << (9 + 16));
}

#[test]
fn set_pin_f1x0_uses_bop_at_0x18() {
    let _g = seed();
    set_pin(PORT_BASE, GpioPath::AhbCtlAfsel, PB9, true);
    assert_eq!(read(F1X0_BOP), 1 << 9);
    set_pin(PORT_BASE, GpioPath::AhbCtlAfsel, PB9, false);
    assert_eq!(read(F1X0_BOP), 1 << (9 + 16));
}

#[test]
fn gpio_output_handle_drives_bop_and_tracks_state_f10x() {
    let _g = seed();
    let mut out = GpioOutput::new(PORT_BASE, GpioPath::ApbCrlCrh, PB9);
    // Fresh handle starts low.
    assert_eq!(out.is_set_low(), Ok(true));
    out.set_high().unwrap();
    assert_eq!(read(F10X_BOP), 1 << 9);
    assert_eq!(out.is_set_high(), Ok(true));
    out.set_low().unwrap();
    assert_eq!(read(F10X_BOP), 1 << (9 + 16));
    assert_eq!(out.is_set_low(), Ok(true));
}

#[test]
fn gpio_output_handle_drives_bop_f1x0() {
    let _g = seed();
    let mut out = GpioOutput::new(PORT_BASE, GpioPath::AhbCtlAfsel, PB9);
    out.set_high().unwrap();
    assert_eq!(read(F1X0_BOP), 1 << 9);
    assert_eq!(out.pin(), PB9);
}

#[test]
fn f10x_timer_gate_pa8_is_af_pp_nibble() {
    let _g = seed();
    // F10x has no AF mux: AF push-pull 50 MHz = nibble 0xB, the SAME as any AF-PP output. PA8 is in
    // CTL1 (pins 8..15), nibble at (8-8)*4 = [3:0].
    configure_af(
        PORT_BASE,
        GpioPath::ApbCrlCrh,
        PA8,
        PinRole::TimerAfPushPull,
    );
    assert_eq!(read(CTL1), 0xB << 0);
    assert_eq!(read(CTL0), 0);
}

#[test]
fn f10x_timer_gate_pa10_uses_ctl1_high_nibble() {
    let _g = seed();
    // PA10 -> CTL1 nibble at (10-8)*4 = [11:8].
    configure_af(
        PORT_BASE,
        GpioPath::ApbCrlCrh,
        PA10,
        PinRole::TimerAfPushPull,
    );
    assert_eq!(read(CTL1), 0xB << 8);
    assert_eq!(read(CTL0), 0);
}

#[test]
fn timer_gate_role_matches_spi_drive_only_af_differs() {
    // The timer gate and SPI SCK/MOSI roles share AF-push-pull drive; on F1x0 they differ ONLY in
    // the AFSEL number (AF2 vs AF0). Configure the same pin under each role and compare.
    let _g = seed();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PA8,
        PinRole::TimerAfPushPull,
    );
    let timer_ctl = read(CTL);
    let timer_ospd = read(OSPD);
    let timer_afsel1 = read(AFSEL1);

    mock::reset();
    configure_af(
        PORT_BASE,
        GpioPath::AhbCtlAfsel,
        PA8,
        PinRole::SpiAfPushPull,
    );
    // CTL (AF mode) and OSPD (50MHz) are identical; only AFSEL differs (AF2 vs AF0 = 0).
    assert_eq!(timer_ctl, read(CTL));
    assert_eq!(timer_ospd, read(OSPD));
    assert_eq!(timer_afsel1, F1X0_AF2);
    assert_eq!(read(AFSEL1), 0); // SPI AF0
}

// --- type-state Pin API (split() pins + into_push_pull_output + the stateful read-back) ---------

use crate::gpio::{Floating, Input, Output, Pin, PortPins, PushPull};

#[test]
fn pin_into_push_pull_output_writes_f1x0_ctl_omode_ospd() {
    let _g = seed();
    // Build a reset-state PA15 directly (PortAPins::from_port would do the same; here we isolate the
    // Pin transition). Port A nibble 0, pin 15.
    let pin: Pin<Input<Floating>> = make_pin(PORT_BASE, GpioPath::AhbCtlAfsel, 15);
    let _out: Pin<Output<PushPull>> = pin.into_push_pull_output();
    // Same bits as configure_output for pin 15: CTL [31:30]=1, OMODE bit15=0, OSPD [31:30]=3.
    assert_eq!(read(CTL), 1u32 << 30);
    assert_eq!(read(OMODE) & (1 << 15), 0);
    assert_eq!(read(OSPD), 3u32 << 30);
}

#[test]
fn pin_output_set_high_low_drives_bop_f10x() {
    let _g = seed();
    let mut out = make_pin(PORT_BASE, GpioPath::ApbCrlCrh, 15).into_push_pull_output();
    out.set_high().unwrap();
    assert_eq!(read(F10X_BOP), 1 << 15);
    out.set_low().unwrap();
    assert_eq!(read(F10X_BOP), 1 << (15 + 16));
}

#[test]
fn pin_output_is_set_high_reads_back_octl() {
    let _g = seed();
    // is_set_high reads the family output-data register (F1x0 OCTL at 0x14), not the write-only BOP.
    let mut out = make_pin(PORT_BASE, GpioPath::AhbCtlAfsel, 15).into_push_pull_output();
    assert_eq!(out.is_set_low(), Ok(true)); // OCTL still 0
    Reg32::new(PORT_BASE, 0x14).write(1 << 15);
    assert_eq!(out.is_set_high(), Ok(true));
    assert_eq!(out.is_set_low(), Ok(false));
}

#[test]
fn split_yields_pins_with_correct_pin_bytes() {
    let _g = seed();
    // PortAPins: pa15 carries logical (0<<4)|15. PortBPins: pb3 carries (1<<4)|3.
    let a = crate::gpio::PortAPins::from_port(PORT_BASE, GpioPath::ApbCrlCrh);
    assert_eq!(a.pa15.pin(), 15);
    assert_eq!(a.pa0.pin(), 0);
    let b = crate::gpio::PortBPins::from_port(PORT_BASE, GpioPath::ApbCrlCrh);
    assert_eq!(b.pb3.pin(), (1 << 4) | 3);
    let c = crate::gpio::PortCPins::from_port(PORT_BASE, GpioPath::AhbCtlAfsel);
    assert_eq!(c.pc13.pin(), (2 << 4) | 13);
}

/// Build a reset-state `Pin<Input<Floating>>` from a port base + path + pin number. The `Pin::new`
/// constructor is private to the `gpio` module, which this test submodule is part of, so it is
/// reachable here via `super`.
fn make_pin(base: u32, path: GpioPath, pin: u8) -> Pin<Input<Floating>> {
    super::Pin::new(base, path, pin)
}

// --- general-purpose digital input (configure_input / read_pin / Pin<Input<_>> InputPin) -------
//
// The switches example path: a plain digital input that owns the F10x/F1x0 register branch
// internally. Assert the exact GPIO config bits for BOTH register models (the F10x CRL/CRH input
// nibble + the GPIO_OCTL pull-direction bit via BOP; the F1x0 CTL input mode + the PUD pull field),
// plus the GPIO_ISTAT read-back the InputPin trait exposes. The foot pads are PA11 (pad A) and PC15
// (pad B); PA11 (pin 11) is in the high half, exercising CTL1 / the PUD high bits.

use crate::gpio::{configure_input, read_pin, InputPull, PullDown, PullUp};
use embedded_hal::digital::InputPin;

// F1x0 ISTAT at 0x10, F10x ISTAT at 0x08 (the offsets the input-group reader uses).
const F10X_ISTAT: u32 = 0x08;
const F1X0_ISTAT: u32 = 0x10;

// PA11 logical byte (port A = 0, pin 11) = pad A; PC15 (port C = 2, pin 15) = pad B.
const PA11: u8 = 0x0B;

#[test]
fn f10x_floating_input_pa11_is_cnf01_nibble_no_octl() {
    let _g = seed();
    configure_input(PORT_BASE, GpioPath::ApbCrlCrh, PA11, InputPull::Floating);
    // Pin 11 -> CTL1 nibble at (11-8)*4 = [15:12]. Floating input = CNF 01 / MODE 00 = 0x4.
    assert_eq!(read(CTL1), 0x4 << 12);
    assert_eq!(read(CTL0), 0);
    // Floating: the OCTL pull bit is meaningless, so no BOP write happened.
    assert_eq!(read(F10X_BOP), 0);
}

#[test]
fn f10x_pull_up_input_pa11_is_cnf10_nibble_octl_set() {
    let _g = seed();
    configure_input(PORT_BASE, GpioPath::ApbCrlCrh, PA11, InputPull::PullUp);
    // Input with pull = CNF 10 / MODE 00 = 0x8, at [15:12].
    assert_eq!(read(CTL1), 0x8 << 12);
    // Pull-up direction = OCTL bit set, written via BOP set half (1 << 11).
    assert_eq!(read(F10X_BOP), 1 << 11);
}

#[test]
fn f10x_pull_down_input_pa11_is_cnf10_nibble_octl_clear() {
    let _g = seed();
    configure_input(PORT_BASE, GpioPath::ApbCrlCrh, PA11, InputPull::PullDown);
    // Input with pull = CNF 10 / MODE 00 = 0x8, at [15:12].
    assert_eq!(read(CTL1), 0x8 << 12);
    // Pull-down direction = OCTL bit clear, written via BOP reset half (1 << (11 + 16)).
    assert_eq!(read(F10X_BOP), 1 << (11 + 16));
}

#[test]
fn f1x0_floating_input_pa11_is_ctl_input_pud_none() {
    let _g = seed();
    configure_input(PORT_BASE, GpioPath::AhbCtlAfsel, PA11, InputPull::Floating);
    // CTL: pin 11, 2 bits at [23:22], input mode = 0.
    assert_eq!(read(CTL), 0);
    // PUD: pin 11, 2 bits at [23:22], floating = 0.
    assert_eq!(read(PUD), 0);
    // Input drives nothing: no output options.
    assert_eq!(read(OMODE), 0);
    assert_eq!(read(OSPD), 0);
}

#[test]
fn f1x0_pull_up_input_pa11_is_ctl_input_pud_01() {
    let _g = seed();
    // Seed CTL with a non-zero value at the pin's field to prove input-mode CLEARS it to 0.
    Reg32::new(PORT_BASE, CTL).write(0x3 << 22);
    configure_input(PORT_BASE, GpioPath::AhbCtlAfsel, PA11, InputPull::PullUp);
    // CTL: pin 11 field at [23:22] -> input mode = 0 (cleared from the seeded 0x3).
    assert_eq!(read(CTL), 0);
    // PUD: pin 11, pull-up = 01 at [23:22].
    assert_eq!(read(PUD), 1 << 22);
}

#[test]
fn f1x0_pull_down_input_pa11_is_ctl_input_pud_10() {
    let _g = seed();
    configure_input(PORT_BASE, GpioPath::AhbCtlAfsel, PA11, InputPull::PullDown);
    // CTL: input mode = 0.
    assert_eq!(read(CTL), 0);
    // PUD: pin 11, pull-down = 10 at [23:22].
    assert_eq!(read(PUD), 2 << 22);
    assert_eq!(read(OMODE), 0);
    assert_eq!(read(OSPD), 0);
}

#[test]
fn read_pin_reads_istat_at_family_offset() {
    let _g = seed();
    // F10x: ISTAT at 0x08, bit 11 high.
    Reg32::new(PORT_BASE, F10X_ISTAT).write(1 << 11);
    assert!(read_pin(PORT_BASE, GpioPath::ApbCrlCrh, PA11));
    assert!(!read_pin(PORT_BASE, GpioPath::ApbCrlCrh, 0x0A)); // pin 10 low
    mock::reset();
    // F1x0: ISTAT at 0x10, bit 11 high.
    Reg32::new(PORT_BASE, F1X0_ISTAT).write(1 << 11);
    assert!(read_pin(PORT_BASE, GpioPath::AhbCtlAfsel, PA11));
    assert!(!read_pin(PORT_BASE, GpioPath::AhbCtlAfsel, 0x0A));
}

#[test]
fn input_pin_is_high_is_low_read_istat_f10x() {
    let _g = seed();
    let mut pad: Pin<Input<PullDown>> =
        make_pin(PORT_BASE, GpioPath::ApbCrlCrh, PA11).into_pull_down_input();
    // ISTAT bit 11 clear: low.
    assert_eq!(pad.is_low(), Ok(true));
    assert_eq!(pad.is_high(), Ok(false));
    // Drive ISTAT (0x08) bit 11 high: now high.
    Reg32::new(PORT_BASE, F10X_ISTAT).write(1 << 11);
    assert_eq!(pad.is_high(), Ok(true));
    assert_eq!(pad.is_low(), Ok(false));
}

#[test]
fn input_pin_is_high_reads_istat_f1x0() {
    let _g = seed();
    let mut pad: Pin<Input<PullUp>> =
        make_pin(PORT_BASE, GpioPath::AhbCtlAfsel, PA11).into_pull_up_input();
    // ISTAT (0x10) bit 11 high -> is_high.
    Reg32::new(PORT_BASE, F1X0_ISTAT).write(1 << 11);
    assert_eq!(pad.is_high(), Ok(true));
}

#[test]
fn pin_into_floating_input_writes_f1x0_ctl_input_pud_none() {
    let _g = seed();
    // Round-trip an output back to a floating input: CTL field clears to 0, PUD stays 0.
    let out = make_pin(PORT_BASE, GpioPath::AhbCtlAfsel, 11).into_push_pull_output();
    let _in: Pin<Input<Floating>> = out.into_floating_input();
    assert_eq!(read(CTL) & (0x3 << 22), 0);
    assert_eq!(read(PUD) & (0x3 << 22), 0);
}

#[test]
fn width_strict_afsel_is_32_bit() {
    // The F1x0 AFSEL registers are 32-bit. Configuring pin 7 lands its nibble in the top 4 bits of
    // AFSEL0 (bits [31:28]); a 16-bit accessor would miss the high half. Reading back as Reg32
    // confirms the full-width write.
    let _g = seed();
    let p7 = 7u8;
    configure_af(PORT_BASE, GpioPath::AhbCtlAfsel, p7, PinRole::Tx);
    assert_eq!(read(AFSEL0), 1u32 << 28);
}

#[test]
fn f1x0_is_set_high_reads_octl_at_0x14_not_pud_at_0x0c() {
    // Regression: on the F1x0 the output-data register OCTL is at 0x14; offset 0x0C is PUD. is_set_high
    // (which backs StatefulOutputPin::toggle) must read OCTL (0x14), not PUD (0x0C). Reading 0x0C made
    // toggle() never see the pin as high on the F1x0, so it stayed solid instead of blinking.
    let _g = seed();
    let mut out = make_pin(PORT_BASE, GpioPath::AhbCtlAfsel, 5).into_push_pull_output();
    // A 1 in PUD (0x0C, the WRONG register) must NOT read back as "set high".
    Reg32::new(PORT_BASE, 0x0C).write(1u32 << 5);
    assert!(
        out.is_set_low().unwrap(),
        "F1x0 is_set_high must not read PUD at 0x0C"
    );
    // A 1 in OCTL (0x14, the correct register) MUST read back as "set high".
    Reg32::new(PORT_BASE, 0x14).write(1u32 << 5);
    assert!(
        out.is_set_high().unwrap(),
        "F1x0 is_set_high must read OCTL at 0x14"
    );
}

#[test]
fn f10x_is_set_high_reads_odr_at_0x0c() {
    // On the F10x the output-data register ODR is at 0x0C.
    let _g = seed();
    let mut out = make_pin(PORT_BASE, GpioPath::ApbCrlCrh, 5).into_push_pull_output();
    Reg32::new(PORT_BASE, 0x0C).write(1u32 << 5);
    assert!(
        out.is_set_high().unwrap(),
        "F10x is_set_high must read ODR at 0x0C"
    );
}

/// Host tests for the resolve-once multi-pin input reader ([`InputGroup`]).
/// A neutral N-pin GPIO read: it samples the pins and packs them into a code,
/// reading the family's `GPIO_ISTAT` offset (0x10 on F1x0 AHB, 0x08 on F10x APB).
mod input_group {
    use crate::descriptor::GpioPath;
    use crate::gpio::InputGroup;
    use crate::reg::{mock, Reg32};

    /// The reader samples three input pins (e.g. PC13 / PA1 / PC14) and packs them into a 3-bit code
    /// `(p2<<2)|(p1<<1)|p0`, reading the family's GPIO_ISTAT offset (0x10 on F1x0 AHB).
    #[test]
    fn input_group_reads_three_lines_into_code() {
        let _serial = mock::lock();
        mock::reset();

        const GPIOA: u32 = 0x4800_0000;
        const GPIOC: u32 = 0x4800_0800;
        // Lines in code order: PC13, PA1, PC14.
        let reader = InputGroup::resolve(
            GpioPath::AhbCtlAfsel,
            [(GPIOC, 13), (GPIOA, 1), (GPIOC, 14)],
        );

        // F1x0 ISTAT is at 0x10. Set PC13 (bit 13) and PA1 (bit 1) high, PC14 (bit 14) low.
        Reg32::new(GPIOC, 0x10).write(1 << 13);
        Reg32::new(GPIOA, 0x10).write(1 << 1); // PA1 = 1
                                               // code = (PC14<<2)|(PA1<<1)|PC13 = (0<<2)|(1<<1)|1 = 0b011 = 3.
        assert_eq!(reader.read(), 0b011);

        // Now drive PC14 high too -> code = (1<<2)|(1<<1)|1 = 0b111 = 7.
        Reg32::new(GPIOC, 0x10).write((1 << 13) | (1 << 14));
        assert_eq!(reader.read(), 0b111);

        // All low.
        Reg32::new(GPIOC, 0x10).write(0);
        Reg32::new(GPIOA, 0x10).write(0);
        assert_eq!(reader.read(), 0);
    }

    /// The F10x GPIO_ISTAT is at 0x08 (APB), not 0x10: the reader picks the offset from the GpioPath.
    #[test]
    fn input_group_uses_apb_istat_offset_on_f10x() {
        let _serial = mock::lock();
        mock::reset();

        const GPIOC: u32 = 0x4001_1000;
        const GPIOA: u32 = 0x4001_0800;
        let reader =
            InputGroup::resolve(GpioPath::ApbCrlCrh, [(GPIOC, 13), (GPIOA, 1), (GPIOC, 14)]);
        // APB ISTAT at 0x08: set PC13 high.
        Reg32::new(GPIOC, 0x08).write(1 << 13);
        assert_eq!(reader.read(), 0b001);
    }
}
