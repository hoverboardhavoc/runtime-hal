//! M1 inter-board UART bench firmware: F103 master + F130 slave.
//!
//! ONE binary boots on either board and configures **everything through runtime-hal from the
//! runtime-detected chip**, not from a compile-time chip selection or a descriptor blob:
//!
//! 1. `runtime_hal::detect_chip()` runs the bus-fault-safe family probe + peripheral-presence
//!    measurement and synthesizes the `Chip` (the register model, base addresses, and measured
//!    timer/ADC counts). A part that matches neither family halts (fail-loud) rather than guessing.
//! 2. Resolve the RCU and GPIOA bases from the chip's `AddrTable` (via `Chip`).
//! 3. `clock::enable_gpio_port` / `clock::enable_usart` using the chip's clock path.
//! 4. `gpio::configure_af` the TX/RX pins (from the code-level `UsartConfig`) using the chip's gpio
//!    path.
//! 5. `usart::Usart::bring_up(&chip, &clock, &UsartConfig{ .. })` with a code-constructed
//!    `ClockConfig` + `UsartConfig`, wrapped in `UsartSerial`.
//!
//! The two families diverge by the DETECTED chip, not by a compile-time chip ifdef in the bring-up
//! logic. The peripheral config (USART1, PA2/PA3, 115200, 8N1) and the clock tree are constructed in
//! code; the chip identity is discovered at runtime.
//!
//! Clock: M1's clock path is peripheral-enable only and does not bring up the PLL, so the firmware
//! runs at the reset-default HSI 8 MHz (no `SystemInit`/PLL, no `configure_tree`). The code-level
//! `ClockConfig` describes that same 8 MHz tree (sysclk 8 MHz, AHB/APB /1), so runtime-hal computes
//! BRR = 69 for an 8 MHz USART1 input clock at 115200. Both boards run identically, so the link
//! agrees end-to-end.
//!
//! Handshake (matches the SPL demo so the human can verify the same way), with the role chosen by the
//! DETECTED family (F10x = master, F1x0 = slave, matching the proven bench setup):
//! - **Master (F103 / F10x):** sends 0x5A periodically; on a correct 0x5A echo, pulses PB9 (buzzer).
//! - **Slave (F130 / F1x0):** echoes each received 0x5A; pulses PB9 on each valid byte.
//!
//! PB9 is driven as a plain push-pull output through runtime-hal's `gpio` output API
//! (`Chip::output_pin` -> a `GpioOutput` that implements `embedded_hal::digital::OutputPin`). The
//! HAL owns the F10x/F1x0 register-model branch internally (the same way `gpio::configure_af` does
//! for alternate-function pins), so the firmware drives the indicator through the standard trait
//! without a compile-time chip ifdef and without raw register writes.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal::digital::OutputPin;

use runtime_hal::{
    clock,
    clock::{ClockConfig, ClockSource},
    config::{Oversampling, UsartConfig, UsartFrame},
    detect_chip,
    gpio::PinRole,
    usart::Usart,
    Chip, ClockPath, GpioOutput, PeriphLabel,
};

/// The code-level clock tree for the M1 bench firmware: the reset-default HSI 8 MHz, AHB/APB /1, no
/// PLL, 0 flash wait states. The firmware does not run `configure_tree` (it stays at reset), so this
/// only feeds the USART input-clock derivation (8 MHz on APB1 -> BRR = 69 at 115200).
const M1_CLOCK: ClockConfig = ClockConfig {
    sysclk_hz: 8_000_000,
    wait_states: 0,
    source: ClockSource::Irc8m,
    pll_mul: 2,
    ahb_psc: 1,
    apb1_psc: 1,
    apb2_psc: 1,
};

/// The handshake probe byte, matching the SPL demo.
const PROBE: u8 = 0x5A;

/// The buzzer pin: PB9 (pin number within GPIOB).
const BUZZER_PIN: u8 = 9;

#[entry]
fn main() -> ! {
    // 1. Detect the chip at runtime (family probe + peripheral measurement -> synthesized Chip). A
    //    part that matches neither family halts (fail-loud) rather than misconfiguring.
    let chip: Chip = detect_chip().unwrap_or_else(|_| halt());

    // 2. The USART config, constructed IN CODE: USART1, PA2/PA3, 115200, 8N1, /16. The chip resolves
    //    USART1 to its base; this supplies the behavior.
    let usart_cfg = UsartConfig {
        usart: PeriphLabel::Usart1,
        tx: 0x02, // PA2
        rx: 0x03, // PA3
        baud: 115_200,
        frame: UsartFrame::EIGHT_N_ONE,
        oversampling: Oversampling::By16,
    };

    // 3. Resolve the RCU base and enable peripheral clocks through runtime-hal's clock path (the
    //    chip's clock selector): the USART instance, its GPIO port (GPIOA, for PA2/PA3), and GPIOB
    //    (the buzzer port).
    let rcu = chip.rcu_base().unwrap_or_else(|_| halt());
    let clk = chip.clock();
    clock::enable_usart(rcu, clk, usart_cfg.usart).unwrap_or_else(|_| halt());
    clock::enable_gpio_port(rcu, clk, PeriphLabel::Gpioa).unwrap_or_else(|_| halt());
    clock::enable_gpio_port(rcu, clk, PeriphLabel::Gpiob).unwrap_or_else(|_| halt());

    // 4. Configure TX/RX alternate-function through runtime-hal's public USART routing (the chip
    //    resolves the pin's port + family AF mux internally). The pin bytes are `(port << 4) | pin`.
    chip.route_usart_pin(usart_cfg.tx, PinRole::Tx)
        .unwrap_or_else(|_| halt());
    chip.route_usart_pin(usart_cfg.rx, PinRole::Rx)
        .unwrap_or_else(|_| halt());

    // Configure PB9 as a plain push-pull output indicator through runtime-hal's gpio output API.
    // `Chip::output_pin` resolves GPIOB from the chip's address table, configures the pin in the
    // detected family's register model, and returns an `embedded_hal::digital::OutputPin` handle, so
    // the firmware drives the indicator through the standard trait (no raw register writes, no
    // compile-time chip ifdef).
    let buzzer = chip
        .output_pin(PeriphLabel::Gpiob, BUZZER_PIN)
        .unwrap_or_else(|_| halt());

    // 5. Bring up the USART peripheral from the code-level ClockConfig + UsartConfig. The AF pins were
    //    already routed in step 4 (`route_usart_pin`); `bring_up` programs only the USART registers.
    let usart = Usart::bring_up(&chip, &M1_CLOCK, &usart_cfg).unwrap_or_else(|_| halt());

    // Run the role chosen by the detected family, matching the PUBLIC clock-path capability directly
    // (no family discriminator): F10x (F103) = master, F1x0 (F130) = slave.
    match chip.clock() {
        ClockPath::F10xRcc => run_master(usart, buzzer),
        ClockPath::F1x0Rcu => run_slave(usart, buzzer),
    }
}

// --- handshake roles --------------------------------------------------------------------------

/// Master: send 0x5A periodically; pulse PB9 on a correct 0x5A echo.
fn run_master(usart: Usart, mut buzzer: GpioOutput) -> ! {
    loop {
        // Send the probe byte (polled TBE/TC inside write_byte).
        usart.write_byte(PROBE);

        // Wait for the echo, then check it. A line error or a wrong byte is ignored (keep probing).
        loop {
            match usart.try_read_byte() {
                Ok(Some(b)) => {
                    if b == PROBE {
                        pulse_buzzer(&mut buzzer);
                    }
                    break;
                }
                Ok(None) => {}   // no byte yet, keep polling
                Err(_) => break, // line error: drop this round, send again
            }
        }

        // Inter-probe gap so the handshake is visible/audible rather than a continuous stream.
        delay(400_000);
    }
}

/// Slave: echo each received 0x5A; pulse PB9 on each valid byte.
fn run_slave(usart: Usart, mut buzzer: GpioOutput) -> ! {
    loop {
        match usart.try_read_byte() {
            Ok(Some(b)) => {
                if b == PROBE {
                    usart.write_byte(b); // echo it back to the master
                    pulse_buzzer(&mut buzzer);
                }
            }
            Ok(None) => {} // nothing received yet
            Err(_) => {}   // line error: ignore and keep listening
        }
    }
}

// --- indicator (PB9 push-pull output) ---------------------------------------------------------

/// Drive the indicator high then low (a visible/audible pulse on the buzzer), through the
/// `embedded_hal::digital::OutputPin` handle from `Chip::output_pin`. The drive is infallible (a
/// single atomic `GPIO_BOP` write), so the `set_high`/`set_low` results are ignored.
fn pulse_buzzer(buzzer: &mut GpioOutput) {
    let _ = buzzer.set_high();
    delay(120_000);
    let _ = buzzer.set_low();
    delay(120_000);
}

// --- helpers ----------------------------------------------------------------------------------

/// Crude busy-wait. At HSI 8 MHz a few-cycle loop body gives a coarse delay; exact timing is not
/// needed, only a visible/audible gap.
#[inline(never)]
fn delay(count: u32) {
    let mut i = count;
    while i > 0 {
        cortex_m::asm::nop();
        i -= 1;
    }
}

/// Halt forever on an unrecoverable error (no family detected, a missing base). The bench operator
/// sees "no handshake" rather than a misconfigured peripheral scribbling on the bus.
fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
