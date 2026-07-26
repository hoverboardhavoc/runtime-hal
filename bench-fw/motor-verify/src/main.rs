//! Configure the motor hot path, then VERIFY it by reading the registers back.
//!
//! ONE binary, flashed unchanged to a GD32F103C8T6 (F10x family) or a GD32F130C8T6 (F1x0 family);
//! `detect_chip()` picks the register model at boot. It exercises the HAL's read-only hot-path
//! register-dump surface (`runtime_hal::RegDumpConfig`): the "configure then confirm" self-check that
//! a shipping firmware runs before it ever arms the bridge.
//!
//! # What it does
//!
//! 1. Detects the chip.
//! 2. Runs the advanced-timer complementary-PWM bring-up (`PwmController::configure`) from a
//!    reference `PwmConfig`. This is CONFIG-ONLY: it programs the time base, the three complementary
//!    channel pairs, dead-time and the break/off-state word, and leaves MOE (the main-output-enable
//!    arming gate) OFF. The bridge is configured but DISARMED, so no current can flow.
//! 3. Applies the motor-era NVIC interrupt-priority ordering (`irq::apply_motor_era_priorities`,
//!    HAL requirement R7): the period (ADC injected-EOC) vector above the USART1 RX vector above the
//!    DMA-RX vector above SysTick. This is the surface `specs/motor-integration.md`'s silicon stage 1
//!    verifies by reading the NVIC IPR bytes + SCB SHPR3 back over SWD, so this image is the vehicle
//!    that carries it onto silicon on BOTH families (the per-family vector numbers come from the
//!    detected `IrqLayout`). Writing priorities enables nothing: the vectors stay masked.
//! 4. Reads the configured registers back with `chip.dump_config(timer, adc)` (pure
//!    reads, no writes, never an MOE write) and checks them against what was configured:
//!    - the period (CAR) and prescaler (PSC) match the config,
//!    - the dead-time field in CCHP matches,
//!    - and, the load-bearing SAFETY check, MOE is CLEAR (`!regs.moe()`): a config-only bring-up
//!      must never have armed the bridge.
//! 5. Reports the verdict on an LED: a SLOW blink means "verified, all fields match and the bridge
//!    is disarmed"; a FAST blink means "the read-back did not match the configured state" (a fault
//!    a bench operator can see without a debugger or a UART).
//!
//! # Why this is electrically safe to run on a board
//!
//! Nothing here arms the bridge: `PwmController::configure` leaves MOE off, and `Chip::dump_config`
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
use runtime_hal::irq;
use runtime_hal::{
    detect_chip, BreakConfig, ClockDiv, ComplementaryPwm, OcMode, PeriphLabel, PwmAlign,
    PwmChannelConfig, PwmConfig, PwmController,
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

/// Check the motor-era interrupt-priority ordering back out of the NVIC/SCB (R7's read-back
/// companion): the period (ADC injected-EOC) vector, the USART1 RX vector and the DMA-RX vector at
/// their named priorities, and SysTick at the stock 0xF0. The per-family vector numbers come from the
/// detected [`runtime_hal::descriptor::IrqLayout`], so the same check is meaningful on both families.
/// The bench reads the same IPR bytes over SWD; this is the on-board half of that gate.
fn verify_priorities(layout: runtime_hal::descriptor::IrqLayout) -> bool {
    let (period, usart1, dma) = irq::motor_era_irqs(layout);
    irq::irq_priority(period) == irq::MOTOR_ISR_PRIORITY
        && irq::irq_priority(usart1) == irq::USART1_RX_PRIORITY
        && irq::irq_priority(dma) == irq::DMA_RX_PRIORITY
        && irq::systick_priority() == irq::SYSTICK_PRIORITY
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

    // Apply the motor-era NVIC priority ordering (R7) for the DETECTED layout, at boot, before any
    // vector is unmasked. Pure NVIC IPR + SCB SHPR3 writes: it enables no interrupt and arms nothing.
    // Silicon stage 1 reads these bytes back over SWD; `verify_priorities` below is the on-board half.
    let layout = chip.irq();
    irq::apply_motor_era_priorities(layout);
    let priorities_ok = verify_priorities(layout);

    // Verdict LED on PB3 (green), push-pull output, parked low.
    let gpiob = chip.gpiob().unwrap().split();
    let mut led = gpiob.pb3.into_push_pull_output();
    let _ = led.set_low();

    // CONFIG-ONLY bring-up: programs the timer, leaves MOE OFF (the bridge stays disarmed). We hold
    // only the per-cycle handle; arming is a separate ArmGate call this example never makes.
    let cfg = reference_config();
    let timer_ok = match PwmController::new().configure(&chip, &cfg) {
        Ok(_handle) => {
            // Read the configured registers back through the chip-based dump builder (pure reads, no
            // MOE write), which resolves the bases internally. This example does not bring up the
            // injected ADC, so TIMER0 is passed for the ADC base too: the verdict only checks the
            // timer half. A full verify example would pass the real injected-ADC label.
            match chip.dump_config(PeriphLabel::Timer0, PeriphLabel::Timer0) {
                Ok(snap) => verify(&snap.timer),
                Err(_) => false,
            }
        }
        // A config failure (e.g. the timer base did not resolve) is itself a verification failure.
        Err(_) => false,
    };
    // The verdict is the conjunction: the timer read-back matched AND the bridge is disarmed AND the
    // motor-era priority ordering reads back as written.
    let verified = timer_ok && priorities_ok;

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
