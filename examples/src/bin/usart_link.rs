//! Inter-board USART link: one image on both boards, each shows the OTHER board's counter on its LEDs.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family).
//! There is no compile-time chip selection: `detect_chip()` works out the family at boot, so the
//! `Serial::new` bring-up and the `Pin` calls below drive the F10x register model on one board and the
//! F1x0 model on the other. This is the M1 inter-board link, validated on silicon, expressed through
//! the pin-handle serial API.
//!
//! What it demonstrates:
//! - runtime-hal's USART pin-handle bring-up (`Serial::new(&chip, &clock, instance, (pa2, pa3), baud)`,
//!   the `I2c::new` analogue): the application passes the named `gpioa.pa2` / `gpioa.pa3` pins it got
//!   from `split()`, never a packed `(port << 4) | pin` byte and never the register model,
//! - byte I/O over the link via the polled USART primitives (`try_read_byte` to drain, `write_byte`
//!   to send); the same `Serial` endpoint also implements the `embedded-io` `Read`/`Write` traits,
//! - the FIFO-less drain discipline that lets a polled RX keep up with a streaming peer,
//! - the HAL's overrun (ORE) self-recovery: the application does NO manual overrun handling. The
//!   FIFO-less USART overruns if a byte is not taken within one character time (~87 us at 115200);
//!   the HAL's `try_read_byte` now clears ORE the family-correct way and keeps RX alive, so a missed
//!   byte never strands the receiver (the link_bench had to clear ORE by hand; that is gone).
//!
//! # Wiring (the cross-wire assumption)
//!
//! The two boards are cross-connected on the inter-board UART, the existing M1 link on PA2/PA3:
//!
//! ```text
//!   board A  PA2 (TX) -----> PA3 (RX)  board B
//!   board A  PA3 (RX) <----- PA2 (TX)  board B
//!   (common ground between the two boards)
//! ```
//!
//! So each board's TX (PA2) drives the other board's RX (PA3), and vice versa. Both boards run this
//! same image; the link is symmetric, so neither is "master".
//!
//! # Behavior
//!
//! Each board periodically increments a local 2-bit counter (0, 1, 2, 3, 0, ...) and sends it as a
//! single byte to the sibling. Every loop pass it drains all available RX bytes and drives its two
//! LEDs from the MOST RECENT received byte: upper = bit 0, lower = bit 1. Net effect: each board's
//! upper + lower LEDs display the OTHER board's 2-bit counter, so both pairs of LEDs count in binary
//! (00, 01, 10, 11, ...), driven by packets from the sibling.
//!
//! # LEDs
//!
//! Upper = PB2, lower = PB5, push-pull outputs (the bench board LED map, RoboDurden `UPPER_LED` /
//! `LOWER_LED` on both 2-1-20 and 2-2-20). These are independent LEDs on separate drive pins, so the
//! pair can show all four states 00/01/10/11 (unlike the green/red `-/g/r` module, which shares one
//! current-limit resistor and so cannot light both at once). Neither PB2 nor PB5 is a JTAG-overlay pin,
//! so this example needs NO `free_jtag_pins()` call at all; PA2/PA3 (USART) are not JTAG-overlay either.
//!
//! # Clock
//!
//! This runs at the 8 MHz reset IRC8M clock: it never brings up the PLL (no `configure_tree` call), so
//! the core stays on the internal RC oscillator. At 8 MHz the USART1 BRR comes out 0x45 (8e6 / 115200),
//! matching the M1 note. The SysTick `Delay` is also built for 8 MHz. The drain cadence is counted in
//! short (1 ms) loop ticks, NOT one long blocking delay, so the FIFO-less RX is never starved.
//!
//! The application defines NO fault handler: `detect_chip()` owns its discrimination BusFault entirely
//! (own probe-scoped vector table via VTOR, restored before it returns).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::clock::{ClockConfig, ClockSource};
use runtime_hal::{detect_chip, PeriphLabel, Serial};

/// The inter-board link rate: 115200 (the M1 link rate; BRR = 0x45 at the 8 MHz reset clock).
const LINK_BAUD: u32 = 115_200;

/// Loop ticks (at ~1 ms each) between counter increments: ~500 ms, in the requested 400-600 ms band.
/// Counted in short ticks rather than one blocking delay so the FIFO-less RX keeps draining.
const SEND_PERIOD_TICKS: u32 = 500;

/// The clock the example runs at: the 8 MHz reset IRC8M, no PLL. We never call `configure_tree`, so
/// the chip stays here; the USART BRR is computed for this clock (8e6 / 115200 = 0x45, the M1 value).
/// It is NOT a clock-tree to program, only the source of truth for `Serial::new`'s BAUD math.
const RESET_8M: ClockConfig = ClockConfig {
    sysclk_hz: 8_000_000,
    wait_states: 0,
    source: ClockSource::Irc8m,
    pll_mul: 2, // unused (no PLL brought up); a legal placeholder value.
    ahb_psc: 1,
    apb1_psc: 1,
    apb2_psc: 1,
};

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the loop delay. `detect_chip()` uses
    // `cortex_m::Peripherals::steal()` internally for its probe, so this `take()` still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();

    // 1. Detect the chip at runtime (family probe + peripheral-presence measurement). A part matching
    //    neither known family panics here and `panic_halt` halts: fail-loud, not a guessed layout.
    let chip = detect_chip().unwrap();

    // 2. Split GPIOA into its named pins (this enables the GPIOA port clock). PA2 (TX) / PA3 (RX) go
    //    to the serial bring-up. The LEDs live on GPIOB, split below. No `free_jtag_pins()` here:
    //    the LEDs are PB2/PB5 (not JTAG-overlay) and PA2/PA3 are not JTAG-overlay either.
    let gpioa = chip.gpioa().unwrap().split();
    let gpiob = chip.gpiob().unwrap().split();

    // 3. Bring up the inter-board link on USART1, PA2 (TX) / PA3 (RX), 115200 8N1. `Serial::new`
    //    CONSUMES the `gpioa.pa2` / `gpioa.pa3` handles, configures them AF push-pull / AF input,
    //    enables the USART1 peripheral clock, and programs the BRR from the running 8 MHz clock. No
    //    packed `(port << 4) | pin` byte: the application passes the named pins.
    let serial = Serial::new(
        &chip,
        &RESET_8M,
        PeriphLabel::Usart1,
        (gpioa.pa2, gpioa.pa3),
        LINK_BAUD,
    )
    .unwrap();
    // The raw USART handle for the polled byte primitives: drain via `try_read_byte` (which clears
    // any overrun and self-recovers, so the app needs no manual ORE handling), send via `write_byte`.
    let usart = serial.usart();

    // 4. LEDs as push-pull outputs from the GPIOB split: upper = PB2, lower = PB5.
    let mut led_upper = gpiob.pb2.into_push_pull_output();
    let mut led_lower = gpiob.pb5.into_push_pull_output();

    // The bench board gates the UPPER/LOWER LED rail behind the SELF_HOLD power latch (PB12): the LEDs
    // stay dark unless PB12 is driven high (the stock firmware does this in main.c). Green/red are on a
    // different rail and do not need it. Drive SELF_HOLD high so upper/lower are actually powered;
    // harmless on boards that do not gate the LEDs this way. Keep the handle alive for the program's life.
    let mut self_hold = gpiob.pb12.into_push_pull_output();
    let _ = self_hold.set_high();

    // 5. SysTick-backed delay at the 8 MHz reset clock.
    let mut delay = runtime_hal::Delay::new(cp.SYST, 8_000_000);

    // Local 2-bit counter we advertise to the sibling, the tick counter that paces it, and the last
    // byte we received from the sibling (drives our LEDs).
    let mut my_counter: u8 = 0;
    let mut tick: u32 = 0;
    let mut last_rx: Option<u8> = None;

    // 6. Main loop, the FIFO-less drain discipline: drain ALL ready RX bytes every pass (never block
    //    long enough to starve the polled RX), keep the freshest byte, periodically send our counter,
    //    and use a short 1 ms delay per pass. The HAL clears any overrun (ORE) inside `try_read_byte`,
    //    so there is NO manual overrun handling here (contrast link_bench's hand-rolled clear).
    loop {
        // --- Drain RX every pass: take every byte that is ready, remember the most recent ---
        loop {
            match usart.try_read_byte() {
                Ok(Some(b)) => last_rx = Some(b),
                // Nothing more ready this instant (or the HAL just self-recovered from an overrun
                // with no fresh byte): stop draining until the next pass.
                Ok(None) => break,
                // A framing/parity error surfaces here, already CLEARED by the HAL so it cannot
                // latch; drop this byte and keep the loop alive (the next pass recovers). NOTE: an
                // overrun never reaches this arm, the HAL clears ORE and returns Ok internally.
                Err(_) => break,
            }
        }

        // --- Drive the LEDs from the most recent received byte: upper = bit 0, lower = bit 1 ---
        if let Some(b) = last_rx {
            let _ = led_upper.set_state((b & 0b01 != 0).into());
            let _ = led_lower.set_state((b & 0b10 != 0).into());
        }

        // --- Periodically advance our 2-bit counter and send it to the sibling ---
        tick = tick.wrapping_add(1);
        if tick >= SEND_PERIOD_TICKS {
            tick = 0;
            my_counter = (my_counter + 1) & 0b11;
            // One byte to the sibling. `write_byte` polls TBE/TC; at this rare cadence it does not
            // starve the RX drain (the loop spends almost all its time draining).
            usart.write_byte(my_counter);
        }

        // Short per-pass delay: keeps the RX drained (one character at 115200 is ~87 us, far under
        // a tick), and the tick count paces the ~500 ms send cadence without one long blocking wait.
        delay.delay_ms(1);
    }
}
