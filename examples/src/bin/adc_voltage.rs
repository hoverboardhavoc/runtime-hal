//! ADC voltage monitor: one runtime-detected image (F10x / F1x0), reads bus voltage and shows the
//! level on two LEDs, publishing the raw + converted values to a fixed RAM block for SWD readback:
//!
//! - **VREFINT** (the internal ~1.2 V bandgap, ADC channel 17): a known voltage, so it yields the real
//!   supply `VDDA = 1.2 V * 4095 / vrefint_raw`. This is the ADC-alive / accuracy anchor (no external
//!   wiring). A healthy reading is roughly `vrefint_raw ~ 1489` at VDDA ~ 3.3 V.
//! - **VBATT** (the bus/battery divider on PA4 = ADC channel 4): converted to real bus volts via the
//!   divider ratio below.
//!
//! # Requires a board that wires the battery to PA4
//!
//! This only produces a real bus reading on a board whose PA4 is on the VBATT divider, i.e. a board
//! that actually senses the pack voltage. A board that does not sense battery leaves PA4 on some
//! other fixed node, so `vbatt_raw` reads a stuck value that does not track the bus. VREFINT / VDDA
//! still read correctly regardless (no external pin), so the SWD block proves the ADC is alive
//! either way.
//!
//! # LEDs
//!
//! - **Lower LED (PB5)** lit when the bus is **under 20 V**.
//! - **Upper LED (PB2)** lit when the bus is **over 25 V**.
//! - Between 20 V and 25 V, neither is lit.
//!
//! Both LEDs (PB2 upper, PB5 lower) sit behind the SELF_HOLD power latch, so the example drives
//! SELF_HOLD (PB12) high to power their rail. Neither pin is JTAG-overlaid, so no JTAG freeing needed.
//!
//! # SWD readback
//!
//! The `ADC_OBS` block (a `#[no_mangle]` static, find it by symbol or read its RAM address) carries
//! `{ magic, seq, vrefint_raw, vbatt_raw, vdda_mv, bus_mv }`, updated every pass. `magic` (0xADC00B5E)
//! marks it; `seq` increments each update so a reader can tell it is live.
//!
//! # Calibration (board-specific, do this once)
//!
//! `VBATT_DIVIDER` is the battery-volts-per-pin-volt ratio of the VBATT resistor divider (factor 30
//! from the RoboDurden defines). Bus volts are reconstructed VDDA-referenced (VDDA measured live from
//! VREFINT), so only the one divider constant is board-specific. Recalibrate from a known PSU point:
//! `DIVIDER = bus_mv * 4095 / (vbatt_raw * vdda_mv)`. The 20 V / 25 V LED thresholds depend on it.
//!
//! # Safety
//!
//! Read-only ADC + two LEDs. This NEVER touches the motor bridge, the advanced timer, or the MOE gate.
//! Runs on the 8 MHz reset clock (no PLL). The application defines no fault handler (`detect_chip()`
//! owns its probe BusFault).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::{detect_chip, enable_adc, AdcCapability, Delay, PeriphLabel};

/// ADC channel for the internal VREFINT bandgap (16 = temperature, 17 = VREFINT on these parts).
const VREFINT_CH: u8 = 17;
/// ADC channel for VBATT: PA4 = ADC_IN4 (RoboDurden `defines`: VBATT on PA4). Only meaningful on a
/// board that wires the battery divider to PA4 (see the module "Requires" note).
const VBATT_CH: u8 = 4;
/// Sample-time code 7 (239.5 cycles): needed for the high-impedance internal VREFINT, and safe for the
/// VBATT resistor divider. Set once per channel before the read loop.
const SAMPLE_LONG: u8 = 7;
/// The GD32 internal reference is ~1.2 V (1200 mV) nominal.
const VREFINT_MV: u32 = 1200;

// --- VBATT calibration: one divider ratio, VDDA-referenced. ------------------------------------
// Bus voltage is reconstructed as `bus = (vbatt_raw / 4095) * VDDA * DIVIDER`, with VDDA measured
// live from VREFINT each pass (so a drifting 3.3 V rail does not skew the reading). DIVIDER is the
// battery-volts-per-pin-volt ratio of the VBATT resistor divider, factor 30 from the RoboDurden
// defines (`ADC_BATTERY_VOLT`). One constant suffices across chip families: the battery-sensing
// boards measured ~31x on both F10x and F1x0 silicon (within resistor tolerance of 30). Recalibrate
// from a known supply point with DIVIDER = bus_mv * 4095 / (vbatt_raw * vdda_mv).
const VBATT_DIVIDER: u32 = 30;

/// Bus voltage below which the lower LED lights (millivolts).
const LOW_MV: u32 = 20_000;
/// Bus voltage above which the upper LED lights (millivolts).
const HIGH_MV: u32 = 25_000;

/// The reset IRC8M core clock this example runs on (no PLL bring-up); the `sysclk_hz` for `Delay`.
const SYSCLK_HZ: u32 = 8_000_000;

/// SWD-readable observation block. Find it by the `ADC_OBS` symbol (nm the elf) or read its RAM
/// address. `magic` marks it; `seq` increments each update (liveness); the rest are the live readings.
#[repr(C)]
struct AdcObs {
    magic: u32,
    seq: u32,
    vrefint_raw: u32,
    vbatt_raw: u32,
    vdda_mv: u32,
    bus_mv: u32,
}

/// Sentinel in `ADC_OBS.magic` so an SWD reader can locate / sanity-check the block.
const OBS_MAGIC: u32 = 0xADC0_0B5E;

#[no_mangle]
static mut ADC_OBS: AdcObs = AdcObs {
    magic: 0,
    seq: 0,
    vrefint_raw: 0,
    vbatt_raw: 0,
    vdda_mv: 0,
    bus_mv: 0,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    // Detect the chip; a part matching neither family panics (panic_halt halts): fail-loud.
    let chip = detect_chip().unwrap();

    // Port clocks: A (PA4 = VBATT) and B (the LEDs + SELF_HOLD).
    let _ = chip.gpioa();
    let _ = chip.gpiob();

    // PA4 must be ANALOG, not its reset digital-input state, or the ADC samples through the live
    // digital input buffer and reads a stuck/clamped value (the stock firmware does the same with
    // `pinMode(VBATT, GPIO_MODE_ANALOG)`). The reset state is a digital input on BOTH families.
    chip.analog_pin(PeriphLabel::Gpioa, 4).unwrap(); // PA4 (ADC channel 4 = VBATT)

    // SELF_HOLD (PB12) high to power the rail both LEDs (PB2 upper, PB5 lower) sit behind. Neither pin
    // is JTAG-overlaid, so no JTAG freeing is needed (PB3/PB4/PA15 are the overlay pins; we avoid them).
    let mut self_hold = chip.output_pin(PeriphLabel::Gpiob, 12).unwrap();
    let _ = self_hold.set_high();
    let mut led_upper = chip.output_pin(PeriphLabel::Gpiob, 2).unwrap();
    let mut led_lower = chip.output_pin(PeriphLabel::Gpiob, 5).unwrap();
    let _ = led_upper.set_low();
    let _ = led_lower.set_low();

    // Enable the ADC0 peripheral clock (the chip.adc() fruit resolves handles, not the bus clock).
    let rcu = chip.rcu_base().unwrap();
    enable_adc(rcu, chip.clock(), PeriphLabel::Adc0).unwrap();

    // Voltage uses the SINGLE-ADC path: take the primary ADC of the capability fruit (dual-
    // simultaneous is for phase currents, not voltages).
    let adc = match chip.adc().unwrap() {
        AdcCapability::Single(a) => a,
        AdcCapability::Dual(dual) => dual.primary(),
    };
    // Bring up + calibrate on VREFINT (sets TSVREN for the internal channel + its long sample time),
    // then set VBATT's sample time once. The read loop only re-points rank 0, no re-calibration.
    adc.bring_up(VREFINT_CH, SAMPLE_LONG).unwrap();
    adc.configure_single(VBATT_CH, SAMPLE_LONG);

    let mut delay = Delay::new(cp.SYST, SYSCLK_HZ);
    let mut seq: u32 = 0;

    loop {
        // VREFINT -> VDDA (the absolute-reference anchor).
        let vrefint_raw = adc.read_channel(VREFINT_CH).unwrap_or(0);
        let vdda_mv = if vrefint_raw > 0 {
            VREFINT_MV * 4095 / u32::from(vrefint_raw)
        } else {
            0
        };

        // VBATT -> bus volts, VDDA-referenced: bus = (raw / 4095) * VDDA * DIVIDER.
        let vbatt_raw = adc.read_channel(VBATT_CH).unwrap_or(0);
        let bus_mv = u32::from(vbatt_raw) * vdda_mv * VBATT_DIVIDER / 4095;

        // Publish for SWD readback (one volatile struct store; seq proves liveness).
        seq = seq.wrapping_add(1);
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(ADC_OBS),
                AdcObs {
                    magic: OBS_MAGIC,
                    seq,
                    vrefint_raw: u32::from(vrefint_raw),
                    vbatt_raw: u32::from(vbatt_raw),
                    vdda_mv,
                    bus_mv,
                },
            );
        }

        // LEDs: lower under 20 V, upper over 25 V, neither in between.
        if bus_mv < LOW_MV {
            let _ = led_lower.set_high();
            let _ = led_upper.set_low();
        } else if bus_mv > HIGH_MV {
            let _ = led_upper.set_high();
            let _ = led_lower.set_low();
        } else {
            let _ = led_lower.set_low();
            let _ = led_upper.set_low();
        }

        delay.delay_ms(100);
    }
}
