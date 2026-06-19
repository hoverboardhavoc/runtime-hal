//! Read the on-board IMU over I2C, run the Mahony attitude filter, and light an LED by pitch.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family).
//! There is no compile-time chip selection: `detect_chip()` works out the family at boot, so the
//! I2C0 bring-up and the `Pin` calls below drive the F10x register model on one board and the F1x0
//! model on the other.
//!
//! What it demonstrates:
//! - runtime-hal's hardware I2C (`runtime_hal::I2c`, an `embedded-hal` 1.0 `i2c::I2c` implementer),
//! - a generic, vendor-neutral MPU-6050-class driver (`imu::Mpu6050`, generic over that trait),
//! - the Mahony complementary attitude filter (`attitude::Mahony`).
//!
//! All `no_std` and fixed-point Q: the IMU scaling, the filter body, and the trig run in `I32F32` /
//! `I16F16` with cordic, never software float, because the Cortex-M3 has no FPU. The per-tick loop
//! introduces no `f32`/`f64` math.
//!
//! Bus / device: I2C0 on SCL = PB6, SDA = PB7 (AF1, open-drain with pull-up), 100 kHz standard mode,
//! IMU 7-bit address 0x68. This bring-up recipe (pins, AF, clocks, speed) is the one validated on the
//! bench F130 by `bench-fw-m2`; it is reused here verbatim.
//!
//! LEDs (push-pull outputs, board LED map: green PB3, red PB4):
//! - green = PB3: ON when pitch < -2 deg ("pitch down"),
//! - red   = PB4: ON when pitch > +2 deg ("pitch up"),
//! - both OFF within +/-2 deg of level.
//!
//! Both LED pins are JTAG-overlay pins on the F103 (PB3 = JTDO, PB4 = NJTRST), so `free_jtag_pins()`
//! is needed before they can drive GPIO (it disables JTAG-DP but keeps SWD live). On the F1x0 that
//! call is a no-op.
//!
//! SIGN-CONVENTION CAVEAT: the pitch sign/axis convention (which physical tilt is "pitch up") depends
//! on the IMU mounting and the per-board sign map; it may be inverted on this hardware. To flip it,
//! negate `pitch` on the one line marked below, OR swap the two LED comparisons. That is the only
//! change needed; the threshold magnitude (2 deg) stays the same.
//!
//! Clock: this runs at the 8 MHz reset IRC8M clock (like `blinky`/`switches`): it never brings up the
//! PLL, so the I2C bring-up is told APB1 = 8 MHz and computes its timing for that (matching the bench
//! probe's 8 MHz timing). The SysTick `Delay` is also built for 8 MHz.
//!
//! BOARD NOTE: the IMU at 0x68 is validated present on the F130 bench board. The F103 IMU presence is
//! to be confirmed on the bench; if the device does not ACK there, the read errors and the loop holds
//! both LEDs off (a failed read is treated as "no tilt info this tick"), so the image is still safe to
//! flash to both boards.
//!
//! The application defines NO fault handler: `detect_chip()` owns its discrimination BusFault entirely
//! (own probe-scoped vector table via VTOR, restored before it returns), so there is no
//! `#[exception] BusFault` here.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::clock::{ClockConfig, ClockSource};
use runtime_hal::i2c::{I2c, I2cMode};
use runtime_hal::{detect_chip, PeriphLabel};

use attitude::{Config as AttConfig, Mahony};
use imu::{Config as ImuConfig, Mpu6050};

/// The IMU bus speed: 100 kHz standard mode (the rate the imu crate / bench probe use).
const I2C_SPEED_HZ: u32 = 100_000;

/// The clock the example actually runs at: the 8 MHz reset IRC8M, no PLL. We never call
/// `configure_tree`, so the chip stays here; the I2C timing must be computed for this clock, so this
/// describes it (sysclk 8 MHz, all prescalers /1 => APB1 = 8 MHz, the I2C peripheral clock). It is
/// NOT a clock-tree to program, only the source of truth for `I2c::bring_up`'s timing math.
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

    // 2. Free the JTAG-overlay pins so PB3 (green) and PB4 (red) can drive their LEDs. F10x: disables
    //    JTAG-DP, keeps SW-DP (SWD stays attached). F1x0: no-op.
    chip.free_jtag_pins().ok();

    // 3. Split GPIOB into its named pins (this enables the GPIOB port clock). PB6/PB7 go to the I2C
    //    bring-up; PB3/PB4 become the LED outputs below.
    let gpiob = chip.gpiob().unwrap().split();

    // 4. I2C0 bring-up on PB6/PB7, exactly as the validated `bench-fw-m2` does it. `I2c::new`
    //    CONSUMES the `gpiob.pb6` (SCL) / `gpiob.pb7` (SDA) handles, configures them AF open-drain
    //    with pull-up (AF1), enables the I2C0 peripheral clock, and computes the 100 kHz timing from
    //    the running clock. No packed `(port << 4) | pin` byte: the application passes the named pins.
    //    `.unwrap()`s here are the same fail-loud posture as the other examples.
    let mut i2c: I2c = I2c::new(
        &chip,
        &RESET_8M,
        PeriphLabel::I2c0,
        (gpiob.pb6, gpiob.pb7),
        I2cMode::standard(I2C_SPEED_HZ),
    )
    .unwrap();

    // 5. Bring up the IMU (wake + full-scale config writes) and build the Mahony filter. Both use the
    //    crates' reference default calibration (sign map / gyro bias / Kp). If the IMU does not ACK
    //    (e.g. an F103 board whose IMU presence is unconfirmed), `init` errors; we keep going and let
    //    the per-tick read drive the LEDs (a failed read = both LEDs off this tick).
    let mut imu = Mpu6050::new(ImuConfig::default());
    let _ = imu.init(&mut i2c);
    let mut mahony = Mahony::new(AttConfig::default());

    // 6. LEDs as push-pull outputs from the same GPIOB split: green = PB3, red = PB4.
    let mut led_green = gpiob.pb3.into_push_pull_output();
    let mut led_red = gpiob.pb4.into_push_pull_output();

    // The +/-2 degree level band, as the filter's output Q type (I16F16 degrees). Constructed once.
    let pitch_threshold = attitude::Out::from_num(2);

    // 7. SysTick-backed delay at the 8 MHz reset clock.
    let mut delay = runtime_hal::Delay::new(cp.SYST, 8_000_000);

    // 8. The control loop, at ~250 Hz (4 ms), the imu/attitude design rate (matching `control.rs`).
    //    Mirror that binary's data flow: read the calibrated IMU sample, feed gyro (rad/s, already in
    //    the filter's body Q) + accel (sign-applied direction counts, widened to the body Q) into
    //    `mahony.update`, take `pitch_deg` from the Output, and drive the LEDs by the +/-2 deg band.
    loop {
        match imu.read(&mut i2c) {
            Ok(sample) => {
                // Accel direction counts widened to the filter's body Q (the `control.rs` recipe).
                let accel = [
                    attitude::Fix::from_num(sample.accel_raw[0]),
                    attitude::Fix::from_num(sample.accel_raw[1]),
                    attitude::Fix::from_num(sample.accel_raw[2]),
                ];
                let out = mahony.update(sample.gyro, accel);

                // Pitch in degrees (I16F16). SIGN-CONVENTION CAVEAT: to invert the convention, change
                // this to `-out.pitch_deg` (the one-line flip), or swap the two comparisons below.
                let pitch = out.pitch_deg;

                if pitch < -pitch_threshold {
                    // Pitch down: green on, red off.
                    let _ = led_green.set_high();
                    let _ = led_red.set_low();
                } else if pitch > pitch_threshold {
                    // Pitch up: red on, green off.
                    let _ = led_green.set_low();
                    let _ = led_red.set_high();
                } else {
                    // Within +/-2 deg of level: both off.
                    let _ = led_green.set_low();
                    let _ = led_red.set_low();
                }
            }
            Err(_) => {
                // A failed read (no/stuck device, e.g. an F103 without the IMU populated): hold both
                // LEDs off this tick. The bus polls are bounded, so this cannot hang the loop.
                let _ = led_green.set_low();
                let _ = led_red.set_low();
            }
        }

        delay.delay_ms(4); // ~250 Hz, the imu/attitude design tick rate.
    }
}
