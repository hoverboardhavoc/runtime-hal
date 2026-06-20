//! ADC voltage monitor on the bench twin-board (F103 master / F130 slave), one image, runtime-detected.
//!
//! Reads two voltages with the on-chip ADC and shows the bus level on two LEDs, while publishing the
//! raw + converted values to a fixed RAM block for SWD readback:
//!
//! - **VREFINT** (the internal ~1.2 V bandgap, ADC channel 17): a known voltage, so it yields the real
//!   supply `VDDA = 1.2 V * 4095 / vrefint_raw`. This is the ADC-alive / accuracy anchor (no external
//!   wiring, both boards). A healthy reading is roughly `vrefint_raw ~ 1489` at VDDA ~ 3.3 V.
//! - **VBATT** (the bus/battery divider on PA4 = ADC channel 4): converted to real bus volts via the
//!   board calibration pair below.
//!
//! # LEDs
//!
//! - **Lower LED (PB5)** lit when the bus is **under 20 V**.
//! - **Green LED (PB3)** lit when the bus is **over 25 V**.
//! - Between 20 V and 25 V, neither is lit.
//!
//! PB3 (green) sits on the always-on rail; PB5 (lower) sits behind the SELF_HOLD power latch, so the
//! example drives SELF_HOLD (PB12) high to power its rail.
//!
//! # SWD readback
//!
//! The `ADC_OBS` block (a `#[no_mangle]` static, find it by symbol or read its RAM address) carries
//! `{ magic, seq, vrefint_raw, vbatt_raw, vdda_mv, bus_mv }`, updated every pass. `magic` (0xADC00B5E)
//! marks it; `seq` increments each update so a reader can tell it is live.
//!
//! # Calibration (board-specific, do this once)
//!
//! `BAT_CALIB_*` maps a raw VBATT count to real bus volts and is specific to the board's divider (the
//! defaults are EFeru's config.h values, NOT necessarily this board's). To calibrate: set a known PSU
//! voltage, read `ADC_OBS.vbatt_raw` over SWD, then set `BAT_CALIB_ADC` = that raw and `BAT_CALIB_CV` =
//! the PSU volts * 100. The 20 V / 25 V LED thresholds are only meaningful once this is calibrated.
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
/// ADC channel for VBATT: PA4 = ADC_IN4 (RoboDurden master `defines_2-2-20.h`: VBATT on PA4). Confirm
/// the F130 slave wires the same channel before trusting its reading.
const VBATT_CH: u8 = 4;
/// Sample-time code 7 (239.5 cycles): needed for the high-impedance internal VREFINT, and safe for the
/// VBATT resistor divider. Set once per channel before the read loop.
const SAMPLE_LONG: u8 = 7;
/// The GD32 internal reference is ~1.2 V (1200 mV) nominal.
const VREFINT_MV: u32 = 1200;

// --- VBATT calibration (raw ADC count <-> real bus volts). BOARD-SPECIFIC; see the module docs. -----
// Defaults from EFeru config.h (`BAT_CALIB_ADC` / `BAT_CALIB_REAL_VOLTAGE`): 1492 counts == 39.70 V.
/// Raw ADC count measured at `BAT_CALIB_CV`.
const BAT_CALIB_ADC: u32 = 1492;
/// The bus voltage at `BAT_CALIB_ADC`, in centivolts (volts * 100). 3970 = 39.70 V.
const BAT_CALIB_CV: u32 = 3970;

/// Bus voltage below which the lower LED lights (millivolts).
const LOW_MV: u32 = 20_000;
/// Bus voltage above which the green LED lights (millivolts).
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

    // Port clocks: A (PA4 = VBATT) and B (the LEDs + SELF_HOLD). PA4 is left at its reset floating-input
    // state; the ADC mux samples it. (An explicit analog-mode pin config is a HAL gap, noted; for a
    // divider node the reset state reads correctly enough for this monitor.)
    let _ = chip.gpioa();
    let _ = chip.gpiob();

    // SELF_HOLD (PB12) high to power the lower-LED rail; green (PB3) is on the always-on rail.
    let mut self_hold = chip.output_pin(PeriphLabel::Gpiob, 12).unwrap();
    let _ = self_hold.set_high();
    let mut led_green = chip.output_pin(PeriphLabel::Gpiob, 3).unwrap();
    let mut led_lower = chip.output_pin(PeriphLabel::Gpiob, 5).unwrap();
    let _ = led_green.set_low();
    let _ = led_lower.set_low();

    // Enable the ADC0 peripheral clock (the chip.adc() fruit resolves handles, not the bus clock).
    let rcu = chip.rcu_base().unwrap();
    enable_adc(rcu, chip.clock(), PeriphLabel::Adc0).unwrap();

    // Voltage uses the SINGLE-ADC path: take the primary ADC of the capability fruit (both families;
    // dual-simultaneous is for phase currents, not voltages).
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

        // VBATT -> bus volts via the board calibration (raw * volts-per-count).
        let vbatt_raw = adc.read_channel(VBATT_CH).unwrap_or(0);
        let bus_mv = u32::from(vbatt_raw) * BAT_CALIB_CV * 10 / BAT_CALIB_ADC;

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

        // LEDs: lower under 20 V, green over 25 V, neither in between.
        if bus_mv < LOW_MV {
            let _ = led_lower.set_high();
            let _ = led_green.set_low();
        } else if bus_mv > HIGH_MV {
            let _ = led_green.set_high();
            let _ = led_lower.set_low();
        } else {
            let _ = led_lower.set_low();
            let _ = led_green.set_low();
        }

        delay.delay_ms(100);
    }
}
