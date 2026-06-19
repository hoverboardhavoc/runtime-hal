//! Configure the motor hot path, then VERIFY it by reading the registers back.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It exercises the HAL's read-only hot-path
//! register-dump surface (`runtime_hal::HotpathConfig`): the "configure then confirm" self-check that
//! a shipping firmware runs before it ever arms the bridge.
//!
//! # What it does
//!
//! 1. Detects the chip and resolves the advanced-timer (TIMER0) base.
//! 2. Runs the advanced-timer complementary-PWM bring-up (`PwmController::configure`) from a
//!    reference `PwmConfig`. This is CONFIG-ONLY: it programs the time base, the three complementary
//!    channel pairs, dead-time and the break/off-state word, and leaves MOE (the main-output-enable
//!    arming gate) OFF. The bridge is configured but DISARMED, so no current can flow.
//! 3. Reads the configured registers back with `HotpathConfig::dump(timer_base, adc_base)` (pure
//!    reads, no writes, never an MOE write) and checks them against what was configured:
//!    - the period (CAR) and prescaler (PSC) match the config,
//!    - the dead-time field in CCHP matches,
//!    - and, the load-bearing SAFETY check, MOE is CLEAR (`!regs.moe()`): a config-only bring-up
//!      must never have armed the bridge.
//! 4. Reports the verdict on an LED: a SLOW blink means "verified, all fields match and the bridge
//!    is disarmed"; a FAST blink means "the read-back did not match the configured state" (a fault
//!    a bench operator can see without a debugger or a UART).
//!
//! # Why this is electrically safe to run on a board
//!
//! Nothing here arms the bridge: `PwmController::configure` leaves MOE off, and `HotpathConfig::dump`
//! only READS registers. With MOE clear the timer counts and the compare events toggle the internal
//! channels, but the gate driver outputs stay at their idle state, so no phase current flows. This
//! is the disarmed-but-configured state the M3 SAFETY section calls electrically safe to scope. The
//! example never builds an `ArmGate` and never calls `arm()`.
//!
//! # Pins
//!
//! Verdict LED = PB3 (green LED on the bench board map). On the F10x, PB3 is JTDO after reset and is
//! freed with `chip.free_jtag_pins()` (which keeps SWD live); on the F1x0 that call is a no-op. The
//! TIMER0 gate pins named in the config (PA8/9/10 high, PB13/14/15 low) are NOT driven as outputs
//! here: the bring-up programs the timer registers, but this example does not configure the gate-pin
//! alternate functions (it never arms), so the half-bridges are not driven. Runs on the 8 MHz reset
//! IRC8M clock (no PLL bring-up).
//!
//! The application defines NO fault handler: `detect_chip()` owns its discrimination BusFault.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use runtime_hal::config::TrgoSource;
use runtime_hal::{
    detect_chip, BreakConfig, ClockDiv, ComplementaryPwm, HotpathConfig, OcMode, PeriphLabel,
    PwmAlign, PwmChannelConfig, PwmConfig, PwmController,
};

/// The reference PWM period (TIMER0 CAR): the stock board's 16 kHz center-aligned value.
const PERIOD: u16 = 2250;
/// The reference prescaler (TIMER0 PSC): the timer runs from the bus clock undivided.
const PRESCALER: u16 = 0;
/// The reference dead-time field code (CCHP DTCFG).
const DEAD_TIME: u8 = 0x1C;

/// The reference complementary-PWM config: three half-bridge pairs (high CH0/1/2 on PA8/9/10, low
/// CH0N/1N/2N on PB13/14/15), center-aligned mode 2, ARSE on, CKDIV/2, break disabled, the CH3
/// ADC-trigger compare near the top, TRGO = update. The pin bytes are `(port << 4) | pin`
/// (port 0=A,1=B): PA8 = 0x08, PB13 = 0x1D, etc.
fn reference_config() -> PwmConfig {
    let ch = |high: u8, low: u8| PwmChannelConfig {
        high,
        low,
        polarity: true,
        idle_high: true,
        idle_high_n: true,
    };
    PwmConfig {
        timer: PeriphLabel::Timer0,
        channels: [ch(0x08, 0x1D), ch(0x09, 0x1E), ch(0x0A, 0x1F)],
        period: PERIOD,
        prescaler: PRESCALER,
        dead_time: DEAD_TIME,
        brk: BreakConfig {
            enabled: false,
            level: false,
        },
        trigger_compare: PERIOD - 1,
        align: PwmAlign::Center2,
        arse: true,
        trigger_oc_mode: OcMode::Pwm0,
        trigger_ch_enable: false,
        crep: 0,
        ckdiv: ClockDiv::Div2,
        trgo_src: TrgoSource::Update,
    }
}

/// Check a register-dump snapshot against the configured state. Returns `true` only if every checked
/// field matches AND the bridge is disarmed (MOE clear). This is the verification gate in miniature:
/// the same fields a host golden or a bench SWD read would diff.
fn verify(regs: &runtime_hal::TimerRegs) -> bool {
    // The bring-up wrote these values; the read-back must agree.
    let period_ok = regs.car as u16 == PERIOD;
    let prescaler_ok = regs.psc as u16 == PRESCALER;
    let dead_time_ok = (regs.cchp & 0xFF) as u8 == DEAD_TIME;
    // The load-bearing SAFETY check: a config-only bring-up must leave MOE clear (disarmed).
    let disarmed = !regs.moe();
    period_ok && prescaler_ok && dead_time_ok && disarmed
}

#[entry]
fn main() -> ! {
    // Take the core peripherals to claim SysTick for the blink delay. detect_chip() steals what it
    // needs internally, so this take() still succeeds.
    let cp = cortex_m::Peripherals::take().unwrap();

    // Detect the chip; a part matching neither family panics (panic_halt halts): fail-loud.
    let chip = detect_chip().unwrap();

    // Free the JTAG-overlay pins so PB3 can drive the verdict LED (F10x: keeps SWD live; F1x0: no-op).
    chip.free_jtag_pins().ok();

    // Verdict LED on PB3 (green), push-pull output, parked low.
    let gpiob = chip.gpiob().unwrap().split();
    let mut led = gpiob.pb3.into_push_pull_output();
    let _ = led.set_low();

    // Resolve the advanced-timer (TIMER0) base, and use it for the ADC base too: this example does
    // not bring up the injected ADC, so the ADC half of the dump is read from a harmless base (the
    // verdict only checks the timer half). A full verify example would resolve the real ADC base.
    let timer_base = chip.base(PeriphLabel::Timer0).unwrap();

    // CONFIG-ONLY bring-up: programs the timer, leaves MOE OFF (the bridge stays disarmed). We hold
    // only the per-cycle handle; arming is a separate ArmGate call this example never makes.
    let cfg = reference_config();
    let verified = match PwmController::new().configure(&chip, &cfg) {
        Ok(_handle) => {
            // Read the configured registers back (pure reads, no MOE write) and check them.
            let snap = HotpathConfig::dump(timer_base, timer_base);
            verify(&snap.timer)
        }
        // A config failure (e.g. the timer base did not resolve) is itself a verification failure.
        Err(_) => false,
    };

    // Build the SysTick-backed delay on the 8 MHz reset clock for the verdict blink.
    let mut delay = runtime_hal::Delay::new(cp.SYST, 8_000_000);

    // Report the verdict forever: SLOW blink (500 ms) = verified + disarmed; FAST blink (80 ms) =
    // the read-back did not match the configured state.
    let on_off_ms: u32 = if verified { 500 } else { 80 };
    loop {
        let _ = led.set_high();
        delay.delay_ms(on_off_ms);
        let _ = led.set_low();
        delay.delay_ms(on_off_ms);
    }
}
