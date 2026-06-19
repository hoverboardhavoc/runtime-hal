//! GPIO path (T4): alternate-function config for a USART's TX/RX pins.
//!
//! The second divergent path a USART needs (the first is [`crate::clock`]). The [`GpioPath`]
//! selector chooses the register model at runtime:
//!
//! - [`GpioPath::ApbCrlCrh`] (`f10x_rcc` family): the F10x model. Each pin is a 4-bit nibble in
//!   `CTL0` (pins 0..7) or `CTL1` (pins 8..15): low 2 bits = MODE, high 2 bits = CNF. Alternate
//!   function is *implied* by the mode/cnf encoding (there is no separate AF-mux register).
//! - [`GpioPath::AhbCtlAfsel`] (`f1x0_rcu` family): the F1x0 model. Mode lives in `CTL` (2 bits per
//!   pin), output type in `OMODE` (1 bit), speed in `OSPD` (2 bits), pull in `PUD` (2 bits), and a
//!   separate per-pin AF mux in `AFSEL0` (pins 0..7) / `AFSEL1` (pins 8..15) (4 bits per pin).
//!
//! A pin is a logical `(port << 4) | pin` byte (SPEC.md pin model); the caller resolves the port
//! to its `port_base` from [`crate::addr::AddrTable`] and passes the base in. Only the pin number
//! (low nibble) matters here, the port is already resolved.
//!
//! [`PinRole`] distinguishes **TX** (alternate-function output, push-pull) from **RX** (input on
//! F10x, alternate-function input on F1x0). The milestone target is `USART1` `TX = PA2`, `RX = PA3`
//! at AF1 on F1x0.
//!
//! # M2 (T5) bus-pin roles (the open-drain + pull-up + AF-number growth)
//!
//! M1's USART pins were always push-pull, AF1. M2's buses need two new combinations the role set
//! grows to express (each carrying its own AF-mux number for the F1x0 path, which M1 hardcoded):
//!
//! - [`PinRole::I2cAfOpenDrain`] (I2C SCL/SDA): **AF, open-drain, with pull-up**. The IMU link is
//!   SCL = PB6 / SDA = PB7 at **AF1** on F1x0 (`gd32f1x0_gpio.c` AF table: I2C0 is on AF1 for
//!   port B). On F10x this is `GPIO_MODE_AF_OD` (CRL/CRH nibble `0xF` = AF-OD `0xC` | 50 MHz `0x3`,
//!   from `GPIO_MODE_AF_OD = 0x1C`); F10x has no per-pin pull control, so the bus pull-up is the
//!   board's external resistor (the I2C convention), which is why the SPL `gpio_init(AF_OD)` writes
//!   no BOP/BC. On F1x0 it is CTL = AF, `OMODE` open-drain, `OSPD` 50 MHz, `PUD` pull-up, AFSEL = AF1.
//! - [`PinRole::SpiAfPushPull`] (SPI SCK/MOSI): AF push-pull (same drive as USART TX) but on the
//!   SPI alternate function (**AF0** on F1x0 for SPI0; `gd32f1x0_gpio.c` AF table: SPI0 is on AF0).
//! - [`PinRole::SpiInput`] (SPI MISO): input, same encoding as a USART RX pin (floating input on
//!   F10x, AF input on F1x0); kept as a distinct role for descriptor readability.
//!
//! All these registers are **32-bit**; access is [`Reg32`] (the testing spec is width-strict, and
//! a 16-vs-32-bit AF-register slip is exactly what the gpio vector must catch).
//!
//! # Register facts (sourced from the GD SPL the vendor library uses)
//!
//! F10x (`framework-spl-gd32/.../gd32f10x/inc/gd32f10x_gpio.h`, and `..._gpio.c::gpio_init`):
//! - `GPIO_CTL0` at offset `0x00`, `GPIO_CTL1` at `0x04` (lines 58-59), 4 bits per pin.
//! - The 4-bit nibble = `(mode & 0x0F)`, OR the 2-bit speed iff the mode is an output
//!   (`mode & 0x10 != 0`) (`gpio_init`, lines 143-148).
//! - `GPIO_MODE_AF_PP = 0x18` (line 315) -> nibble low = `0x8`; output, so OR speed.
//!   `GPIO_OSPEED_50MHZ = 0x03` (line 320) -> nibble = `0x8 | 0x3 = 0xB`. (TX = AF push-pull 50MHz.)
//! - `GPIO_MODE_IN_FLOATING = 0x04` (line 309) -> nibble = `0x4`; not output, no speed. (RX.)
//!
//! F1x0 (`framework-spl-gd32/.../gd32f1x0/inc/gd32f1x0_gpio.h`, and `..._gpio.c`):
//! - `GPIO_CTL` at `0x00`, `GPIO_OMODE` at `0x04`, `GPIO_OSPD` at `0x08`, `GPIO_PUD` at `0x0C`,
//!   `GPIO_AFSEL0` at `0x20`, `GPIO_AFSEL1` at `0x24` (lines 53-62).
//! - `CTL`: 2 bits/pin, `GPIO_MODE_AF = CTL_CLTR(2) = 2`, `GPIO_MODE_INPUT = 0` (lines 294-296).
//! - `OMODE`: 1 bit/pin, `GPIO_OTYPE_PP = 0` (line 337).
//! - `OSPD`: 2 bits/pin, `GPIO_OSPEED_50MHZ = OSPD_OSPD(3) = 3` (line 344).
//! - `PUD`: 2 bits/pin, `GPIO_PUPD_NONE = 0` (line 301).
//! - `AFSEL{0,1}`: 4 bits/pin (`GPIO_AFR_SET(n, af) = af << (4*n)`, line 347). `GPIO_AF_1` is the
//!   USART0/USART1 alternate function on these pins (`gd32f1x0_gpio.c:321` AF table).
//!
//! Both families set mode then AF then output options in the order the SPL does: F10x writes the
//! single CTL nibble; F1x0 writes CTL, then AFSEL, then OMODE+OSPD (and PUD).

use crate::descriptor::GpioPath;
use crate::reg::Reg32;

// --- F10x register offsets --------------------------------------------------------------------
const F10X_CTL0: u32 = 0x00;
const F10X_CTL1: u32 = 0x04;

// --- F1x0 register offsets --------------------------------------------------------------------
const F1X0_CTL: u32 = 0x00;
const F1X0_OMODE: u32 = 0x04;
const F1X0_OSPD: u32 = 0x08;
const F1X0_PUD: u32 = 0x0C;
const F1X0_AFSEL0: u32 = 0x20;
const F1X0_AFSEL1: u32 = 0x24;

// --- bit set/reset register offsets (the atomic drive register, both families) ----------------
// Both models expose a single 32-bit bit-operate register: writing `1 << pin` (low half) sets the
// pin, `1 << (pin + 16)` (high half) resets it. The offset differs by family.
// F10x `GPIO_BOP` at 0x10 (`gd32f10x_gpio.h`); F1x0 `GPIO_BOP` at 0x18 (`gd32f1x0_gpio.h`).
const F10X_BOP: u32 = 0x10;
const F1X0_BOP: u32 = 0x18;

// --- GPIO input-status register (read-back of the live pin level) -----------------------------
// `GPIO_ISTAT` is at a family-dependent offset: 0x08 on the F10x (APB) GPIO and 0x10 on the F1x0
// (AHB) GPIO (verified against `gd32f10x_gpio.h` / `gd32f1x0_gpio.h`, the same offsets the hot-path
// hall reader uses in `crate::hotpath::hall`). Bit `n` is the live level of pin `n`.
const F10X_ISTAT: u32 = 0x08;
const F1X0_ISTAT: u32 = 0x10;

// --- F1x0 field values ------------------------------------------------------------------------
const F1X0_MODE_INPUT: u32 = 0;
/// CTL = general-purpose output (`GPIO_MODE_OUTPUT = 1`, `gd32f1x0_gpio.h:295`).
const F1X0_MODE_OUTPUT: u32 = 1;
const F1X0_MODE_AF: u32 = 2;
const F1X0_OSPEED_50MHZ: u32 = 3;
const F1X0_PUPD_NONE: u32 = 0;
const F1X0_PUPD_PULLUP: u32 = 1;
/// `PUD` pull-down (`GPIO_PUPD_PULLDOWN = PUD_PUPD(2) = 2`, `gd32f1x0_gpio.h`).
const F1X0_PUPD_PULLDOWN: u32 = 2;
const F1X0_OTYPE_PP: bool = false;
const F1X0_OTYPE_OD: bool = true;
/// AF1 = USART0/USART1 and I2C0 (port B) on the milestone pins. (`gd32f1x0_gpio.c` AF table.)
const F1X0_AF_1: u32 = 1;
/// AF0 = SPI0 on F1x0 (`gd32f1x0_gpio.c` AF table: SPI0 is on AF0).
const F1X0_AF_0: u32 = 0;
/// AF2 = TIMER0 (and TIMER1/15/16) on F1x0 (`gd32f1x0_gpio.c:322` AF table: `GPIO_AF_2: TIMER0,
/// TIMER1, TIMER15, TIMER16, EVENTOUT`). The advanced-timer complementary-PWM gate pins
/// (high CH0/1/2 on PA8/9/10, low CH0N/1N/2N on PB13/14/15) route to TIMER0 through AF2.
const F1X0_AF_2: u32 = 2;

/// The role a bus pin plays, which fixes its direction, drive, pull, and AF-mux number.
///
/// M1's USART roles ([`PinRole::Tx`] / [`PinRole::Rx`]) were always AF push-pull / input at AF1.
/// M2 grows the set so I2C can express **open-drain + pull-up** and SPI its **AF push-pull** on a
/// different AF number (see the module docs):
///
/// - [`PinRole::Tx`]: USART transmit, AF push-pull 50 MHz, AF1.
/// - [`PinRole::Rx`]: USART receive, input (F10x floating / F1x0 AF input), AF1.
/// - [`PinRole::I2cAfOpenDrain`]: I2C SCL/SDA, AF **open-drain with pull-up**, 50 MHz, AF1.
/// - [`PinRole::SpiAfPushPull`]: SPI SCK/MOSI, AF push-pull 50 MHz, AF0 (the SPI AF on F1x0).
/// - [`PinRole::SpiInput`]: SPI MISO, input (same encoding as RX), AF0.
///
/// M3 (DR-T4) adds the advanced-timer complementary-PWM gate role, the last raw-`Reg32` bypass the
/// M3 bench firmware needed (it set the six gate pins' AF mux by hand because the role set had no
/// timer AF):
///
/// - [`PinRole::TimerAfPushPull`]: the six PWM gate pins (high CH0/1/2 on PA8/9/10, low CH0N/1N/2N
///   on PB13/14/15), AF push-pull 50 MHz, **AF2** (TIMER0 on F1x0). Same drive as
///   [`PinRole::SpiAfPushPull`]; only the F1x0 AF-mux number differs (AF2 vs AF0). On F10x the AF is
///   implied by the AF-push-pull nibble (0xB), identical to any other AF-PP output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinRole {
    /// USART transmit: alternate-function push-pull output, 50 MHz.
    Tx,
    /// USART receive: input (F10x floating input; F1x0 AF input).
    Rx,
    /// I2C SCL/SDA: alternate-function open-drain output with pull-up, 50 MHz.
    I2cAfOpenDrain,
    /// SPI SCK/MOSI: alternate-function push-pull output, 50 MHz.
    SpiAfPushPull,
    /// SPI MISO: input (F10x floating input; F1x0 AF input).
    SpiInput,
    /// Advanced-timer complementary-PWM gate (CHx / CHxN): alternate-function push-pull output,
    /// 50 MHz, AF2 (TIMER0 on F1x0).
    TimerAfPushPull,
}

/// The role-independent configuration a role resolves to, shared by both register models.
///
/// `af` is the F1x0 AF-mux number (ignored by F10x, where AF is implied by the mode/cnf nibble).
struct PinConfig {
    /// True if this pin drives the line as an output (push-pull or open-drain); false for inputs.
    output: bool,
    /// True for open-drain output drive (only meaningful when `output`); false for push-pull.
    open_drain: bool,
    /// True to enable the internal pull-up (I2C). USART/SPI pins float.
    pull_up: bool,
    /// The F1x0 alternate-function number (AF1 for USART/I2C, AF0 for SPI).
    af: u32,
}

impl PinRole {
    /// Resolve this role to its shared, model-independent pin configuration.
    fn config(self) -> PinConfig {
        match self {
            PinRole::Tx => PinConfig {
                output: true,
                open_drain: false,
                pull_up: false,
                af: F1X0_AF_1,
            },
            PinRole::Rx => PinConfig {
                output: false,
                open_drain: false,
                pull_up: false,
                af: F1X0_AF_1,
            },
            PinRole::I2cAfOpenDrain => PinConfig {
                output: true,
                open_drain: true,
                pull_up: true,
                af: F1X0_AF_1,
            },
            PinRole::SpiAfPushPull => PinConfig {
                output: true,
                open_drain: false,
                pull_up: false,
                af: F1X0_AF_0,
            },
            PinRole::SpiInput => PinConfig {
                output: false,
                open_drain: false,
                pull_up: false,
                af: F1X0_AF_0,
            },
            // Advanced-timer gate: same AF push-pull drive as SPI SCK/MOSI, but on the TIMER0 AF
            // (AF2 on F1x0; F10x has no AF mux, so the AF-PP nibble carries it). The AF number is
            // carried by the role here, exactly as the SPI/USART/I2C roles carry theirs.
            PinRole::TimerAfPushPull => PinConfig {
                output: true,
                open_drain: false,
                pull_up: false,
                af: F1X0_AF_2,
            },
        }
    }
}

/// Configure a logical pin's alternate-function for the given role.
///
/// `port_base` is the resolved base of the pin's port (from the address table). `pin` is the
/// logical `(port << 4) | pin` byte; only the low nibble (pin number 0..15) is used here, the port
/// having already been resolved to `port_base`.
pub fn configure_af(port_base: u32, path: GpioPath, pin: u8, role: PinRole) {
    let n = (pin & 0x0F) as u32;
    let cfg = role.config();
    match path {
        GpioPath::ApbCrlCrh => configure_f10x(port_base, n, &cfg),
        GpioPath::AhbCtlAfsel => configure_f1x0(port_base, n, &cfg),
    }
}

// --- F10x (apb_crl_crh) -----------------------------------------------------------------------

/// Configure pin `n` on the F10x CRL/CRH model.
///
/// Pins 0..7 are in `CTL0`, pins 8..15 in `CTL1`; within the register each pin owns a 4-bit nibble
/// at `4 * (n % 8)`. The CNF/MODE nibble follows the SPL `gpio_init` (see the module docs):
/// - AF push-pull 50 MHz = `0xB` (`GPIO_MODE_AF_PP = 0x18` -> `0x8` | speed `0x3`).
/// - AF open-drain 50 MHz = `0xF` (`GPIO_MODE_AF_OD = 0x1C` -> `0xC` | speed `0x3`).
/// - Floating input = `0x4` (`GPIO_MODE_IN_FLOATING`), no speed.
///
/// F10x has no per-pin pull register: the I2C pull-up is the board's external resistor (the SPL's
/// `gpio_init(AF_OD)` likewise writes no pull / BOP), so `pull_up` does not change the F10x write.
fn configure_f10x(port_base: u32, n: u32, cfg: &PinConfig) {
    let nibble: u32 = if !cfg.output {
        // Floating input (0x4), no speed.
        0x4
    } else if cfg.open_drain {
        // AF open-drain (0xC) | 50MHz speed (0x3).
        0xF
    } else {
        // AF push-pull (0x8) | 50MHz speed (0x3).
        0xB
    };
    let (offset, within) = if n < 8 {
        (F10X_CTL0, n)
    } else {
        (F10X_CTL1, n - 8)
    };
    let shift = 4 * within;
    let mask = 0xFu32 << shift;
    Reg32::new(port_base, offset).modify(mask, nibble << shift);
}

// --- F1x0 (ahb_ctl_afsel) ---------------------------------------------------------------------

/// Configure pin `n` on the F1x0 CTL/AFSEL model.
///
/// Order matches the SPL (`gpio_mode_set` then `gpio_af_set` then `gpio_output_options_set`):
/// 1. `CTL`: AF mode for every role (the line goes through the AF mux either way).
/// 2. `PUD`: pull-up for an I2C pin, no pull for USART/SPI.
/// 3. `AFSEL{0,1}`: the role's AF-mux number (AF1 for USART/I2C, AF0 for SPI).
/// 4. For output roles, the output options: `OMODE` push-pull or open-drain, and `OSPD` 50 MHz.
fn configure_f1x0(port_base: u32, n: u32, cfg: &PinConfig) {
    // 1. CTL: 2 bits/pin. AF mode for every role: the line goes through the AF mux either way, and
    // the SPL drives USART/SPI inputs as AF too (F1X0_MODE_INPUT is the documented non-AF
    // alternative, not used for a bus pin).
    let _ = F1X0_MODE_INPUT;
    let mode = F1X0_MODE_AF;
    let ctl_shift = 2 * n;
    Reg32::new(port_base, F1X0_CTL).modify(0x3u32 << ctl_shift, mode << ctl_shift);

    // 2. PUD: 2 bits/pin. Pull-up for I2C, no pull otherwise.
    let pud_shift = 2 * n;
    let pupd = if cfg.pull_up {
        F1X0_PUPD_PULLUP
    } else {
        F1X0_PUPD_NONE
    };
    Reg32::new(port_base, F1X0_PUD).modify(0x3u32 << pud_shift, pupd << pud_shift);

    // 3. AFSEL: 4 bits/pin, split AFSEL0 (0..7) / AFSEL1 (8..15).
    let (af_off, af_within) = if n < 8 {
        (F1X0_AFSEL0, n)
    } else {
        (F1X0_AFSEL1, n - 8)
    };
    let af_shift = 4 * af_within;
    Reg32::new(port_base, af_off).modify(0xFu32 << af_shift, cfg.af << af_shift);

    // 4. Output options for output roles (drive type + 50 MHz). Input roles do not drive the line.
    if cfg.output {
        // OMODE: 1 bit/pin, 0 = push-pull, 1 = open-drain.
        let od = if cfg.open_drain {
            F1X0_OTYPE_OD
        } else {
            F1X0_OTYPE_PP
        };
        let omode_val = if od { 1u32 << n } else { 0 };
        Reg32::new(port_base, F1X0_OMODE).modify(1u32 << n, omode_val);
        // OSPD: 2 bits/pin, 50 MHz = 3.
        let sp_shift = 2 * n;
        Reg32::new(port_base, F1X0_OSPD).modify(0x3u32 << sp_shift, F1X0_OSPEED_50MHZ << sp_shift);
    }
}

// --- general-purpose digital input ------------------------------------------------------------

/// The pull configuration a digital input pin is set to.
///
/// The model-independent selector that [`configure_input`] resolves to the family's register write
/// (the F10x CRL/CRH input nibble + `GPIO_OCTL` pull-direction bit, or the F1x0 `GPIO_PUD` field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InputPull {
    /// No internal pull resistor (the reset state); the line floats.
    Floating,
    /// Internal pull-up: the pin idles high.
    PullUp,
    /// Internal pull-down: the pin idles low.
    PullDown,
}

/// Configure a logical pin as a general-purpose digital input with the given [`InputPull`].
///
/// The input counterpart to [`configure_output`]: it owns the F10x/F1x0 register-model branch
/// internally so callers never see the [`GpioPath`] split (the same standard [`configure_output`]
/// and [`configure_af`] hold). `port_base` is the resolved base of the pin's port; `pin` is the
/// logical `(port << 4) | pin` byte, only the low nibble (pin number 0..15) is used.
///
/// - F10x (CRL/CRH): a floating input is the nibble CNF `0b01` / MODE `0b00` = `0x4`; an input with
///   a pull is CNF `0b10` / MODE `0b00` = `0x8`, and the pull DIRECTION is selected by the pin's
///   `GPIO_OCTL` bit (1 = pull-up, 0 = pull-down), per `gd32f10x_gpio.c::gpio_init`.
/// - F1x0 (CTL/PUD): `CTL` = input mode (`0`), pull via `GPIO_PUD` (2 bits/pin: `01` = pull-up,
///   `10` = pull-down, `00` = floating), per `gd32f1x0_gpio.c::gpio_mode_set`.
pub(crate) fn configure_input(port_base: u32, path: GpioPath, pin: u8, pull: InputPull) {
    let n = (pin & 0x0F) as u32;
    match path {
        GpioPath::ApbCrlCrh => configure_input_f10x(port_base, n, pull),
        GpioPath::AhbCtlAfsel => configure_input_f1x0(port_base, n, pull),
    }
}

/// Configure pin `n` as a digital input on the F10x CRL/CRH model.
///
/// Pins 0..7 are in `CTL0`, pins 8..15 in `CTL1`; within the register each pin owns a 4-bit nibble
/// at `4 * (n % 8)`. Floating input = nibble `0x4` (CNF `0b01` / MODE `0b00`); input with a pull =
/// nibble `0x8` (CNF `0b10` / MODE `0b00`), and the pull DIRECTION is then set by the pin's
/// `GPIO_OCTL` bit (1 = pull-up, 0 = pull-down), written through the atomic `GPIO_BOP` register
/// (set half for pull-up, reset half for pull-down), exactly as `gd32f10x_gpio.c::gpio_init` does.
fn configure_input_f10x(port_base: u32, n: u32, pull: InputPull) {
    let nibble: u32 = match pull {
        // Floating input (CNF 01 / MODE 00).
        InputPull::Floating => 0x4,
        // Input with pull-up/down (CNF 10 / MODE 00); the direction is the OCTL bit, set below.
        InputPull::PullUp | InputPull::PullDown => 0x8,
    };
    let (offset, within) = if n < 8 {
        (F10X_CTL0, n)
    } else {
        (F10X_CTL1, n - 8)
    };
    let shift = 4 * within;
    let mask = 0xFu32 << shift;
    Reg32::new(port_base, offset).modify(mask, nibble << shift);

    // The pull direction is the GPIO_OCTL bit for the pin, driven via the atomic GPIO_BOP register:
    // 1 (BOP set half, `1 << n`) = pull-up, 0 (BOP reset half, `1 << (n + 16)`) = pull-down. The
    // OCTL bit is meaningless for a floating input, so leave it alone.
    match pull {
        InputPull::PullUp => Reg32::new(port_base, F10X_BOP).write(1u32 << n),
        InputPull::PullDown => Reg32::new(port_base, F10X_BOP).write(1u32 << (n + 16)),
        InputPull::Floating => {}
    }
}

/// Configure pin `n` as a digital input on the F1x0 CTL/PUD model.
///
/// `CTL` = input mode (2 bits/pin, `0`); the pull lives in `GPIO_PUD` (2 bits/pin: `00` = floating,
/// `01` = pull-up, `10` = pull-down), per `gd32f1x0_gpio.c::gpio_mode_set`. No output options
/// (`OMODE`/`OSPD`) are written: an input does not drive the line.
fn configure_input_f1x0(port_base: u32, n: u32, pull: InputPull) {
    // CTL: 2 bits/pin, input mode = 0.
    let ctl_shift = 2 * n;
    Reg32::new(port_base, F1X0_CTL).modify(0x3u32 << ctl_shift, F1X0_MODE_INPUT << ctl_shift);
    // PUD: 2 bits/pin.
    let pupd = match pull {
        InputPull::Floating => F1X0_PUPD_NONE,
        InputPull::PullUp => F1X0_PUPD_PULLUP,
        InputPull::PullDown => F1X0_PUPD_PULLDOWN,
    };
    let pud_shift = 2 * n;
    Reg32::new(port_base, F1X0_PUD).modify(0x3u32 << pud_shift, pupd << pud_shift);
}

/// Read the live level of a logical input pin from the family's `GPIO_ISTAT` register.
///
/// Owns the family offset branch internally (F10x `GPIO_ISTAT` at `0x08`, F1x0 at `0x10`, the same
/// offsets the hot-path hall reader uses), so callers never see the [`GpioPath`] split. Returns
/// `true` if the pin reads high. `port_base` is the resolved port base; only `pin`'s low nibble is
/// used.
pub fn read_pin(port_base: u32, path: GpioPath, pin: u8) -> bool {
    let n = (pin & 0x0F) as u32;
    let istat = match path {
        GpioPath::ApbCrlCrh => F10X_ISTAT,
        GpioPath::AhbCtlAfsel => F1X0_ISTAT,
    };
    Reg32::new(port_base, istat).read() & (1u32 << n) != 0
}

// --- general-purpose digital output -----------------------------------------------------------

/// Configure a logical pin as a general-purpose push-pull output, 50 MHz.
///
/// The plain-output counterpart to [`configure_af`]: it owns the F10x/F1x0 register-model branch
/// internally so callers never see the [`GpioPath`] split (the same way [`configure_af`] hides it
/// for alternate-function pins). `port_base` is the resolved base of the pin's port (from the
/// address table); `pin` is the logical `(port << 4) | pin` byte, only the low nibble (pin number
/// 0..15) is used here, the port having already been resolved to `port_base`.
///
/// - F10x (CRL/CRH): the pin's 4-bit nibble = general-purpose output push-pull (CNF = `0b00`) at
///   50 MHz (MODE = `0b11`) -> `0x3` (`gd32f10x_gpio.c::gpio_init`).
/// - F1x0 (CTL/OMODE/OSPD): `CTL` = output mode (`1`), `OMODE` = push-pull (`0`), `OSPD` = 50 MHz
///   (`3`) (`gd32f1x0_gpio.c`).
pub fn configure_output(port_base: u32, path: GpioPath, pin: u8) {
    let n = (pin & 0x0F) as u32;
    match path {
        GpioPath::ApbCrlCrh => configure_output_f10x(port_base, n),
        GpioPath::AhbCtlAfsel => configure_output_f1x0(port_base, n),
    }
}

/// Configure pin `n` as a push-pull output on the F10x CRL/CRH model.
///
/// Pins 0..7 are in `CTL0`, pins 8..15 in `CTL1`; within the register each pin owns a 4-bit nibble
/// at `4 * (n % 8)`. General-purpose output push-pull 50 MHz = nibble `0x3` (CNF `0b00` | MODE
/// `0b11`).
fn configure_output_f10x(port_base: u32, n: u32) {
    const NIBBLE: u32 = 0x3;
    let (offset, within) = if n < 8 {
        (F10X_CTL0, n)
    } else {
        (F10X_CTL1, n - 8)
    };
    let shift = 4 * within;
    let mask = 0xFu32 << shift;
    Reg32::new(port_base, offset).modify(mask, NIBBLE << shift);
}

/// Configure pin `n` as a push-pull output on the F1x0 CTL/OMODE/OSPD model.
///
/// `CTL` = output mode (2 bits/pin), `OMODE` = push-pull (1 bit/pin, `0`), `OSPD` = 50 MHz (2
/// bits/pin, `3`). Written in the SPL order (mode, then output options).
fn configure_output_f1x0(port_base: u32, n: u32) {
    // CTL: 2 bits/pin, output mode.
    let ctl_shift = 2 * n;
    Reg32::new(port_base, F1X0_CTL).modify(0x3u32 << ctl_shift, F1X0_MODE_OUTPUT << ctl_shift);
    // OMODE: 1 bit/pin, push-pull = 0.
    let omode_val = if F1X0_OTYPE_PP { 1u32 << n } else { 0 };
    Reg32::new(port_base, F1X0_OMODE).modify(1u32 << n, omode_val);
    // OSPD: 2 bits/pin, 50 MHz = 3.
    let sp_shift = 2 * n;
    Reg32::new(port_base, F1X0_OSPD).modify(0x3u32 << sp_shift, F1X0_OSPEED_50MHZ << sp_shift);
}

/// Drive a logical output pin high or low via the family's atomic bit set/reset register.
///
/// Both register models expose one 32-bit bit-operate register (`GPIO_BOP`): writing `1 << pin`
/// sets the pin, `1 << (pin + 16)` resets it, no read-modify-write. This owns the family offset
/// branch internally (F10x `BOP` at `0x10`, F1x0 `BOP` at `0x18`), so callers never see the
/// [`GpioPath`] split. `port_base` is the resolved port base; only `pin`'s low nibble is used.
pub fn set_pin(port_base: u32, path: GpioPath, pin: u8, high: bool) {
    let n = (pin & 0x0F) as u32;
    let bop = match path {
        GpioPath::ApbCrlCrh => F10X_BOP,
        GpioPath::AhbCtlAfsel => F1X0_BOP,
    };
    let value = if high { 1u32 << n } else { 1u32 << (n + 16) };
    Reg32::new(port_base, bop).write(value);
}

/// A configured general-purpose push-pull output pin, as an [`embedded_hal::digital::OutputPin`].
///
/// This is the headline output API: application code drives the pin through the standard
/// `embedded-hal` trait and never touches the [`GpioPath`] split or a raw register base. Build one
/// with [`crate::Chip::output_pin`] (which resolves the port base from the chip's address table and
/// configures the pin via [`configure_output`]), then call [`embedded_hal::digital::OutputPin::set_high`]
/// / [`embedded_hal::digital::OutputPin::set_low`].
///
/// The handle is `Copy` and carries no ownership: it holds the resolved port base, the chip's
/// [`GpioPath`], and the logical pin byte. Driving the pin is the single atomic `GPIO_BOP` write of
/// [`set_pin`], so it is infallible (the [`embedded_hal::digital::ErrorType::Error`] is
/// [`core::convert::Infallible`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioOutput {
    port_base: u32,
    path: GpioPath,
    pin: u8,
    /// The last commanded level, tracked so [`embedded_hal::digital::StatefulOutputPin`] can report
    /// it (the `GPIO_BOP` register is write-only, so the state is not read back from hardware).
    state: bool,
}

impl GpioOutput {
    /// Wrap an already-configured pin as an output handle, starting in the low state.
    ///
    /// `port_base` is the resolved port base, `path` the chip's GPIO register-model selector, and
    /// `pin` the logical `(port << 4) | pin` byte. This does NOT configure the pin: callers go
    /// through [`crate::Chip::output_pin`], which configures it with [`configure_output`] first.
    #[inline]
    pub const fn new(port_base: u32, path: GpioPath, pin: u8) -> Self {
        Self {
            port_base,
            path,
            pin,
            state: false,
        }
    }

    /// The logical `(port << 4) | pin` byte this handle drives.
    #[inline]
    pub const fn pin(&self) -> u8 {
        self.pin
    }
}

impl embedded_hal::digital::ErrorType for GpioOutput {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for GpioOutput {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_pin(self.port_base, self.path, self.pin, true);
        self.state = true;
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_pin(self.port_base, self.path, self.pin, false);
        self.state = false;
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for GpioOutput {
    /// Report the last commanded level (the `GPIO_BOP` drive register is write-only, so this is the
    /// tracked state, not a hardware read-back of the input data register).
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
}

// --- type-state pin API (the stm32f1xx-hal-style `split()` ergonomics) -------------------------
//
// The headline application API mirrors stm32f1xx-hal's `gpioa.split()` -> named pins ->
// `pin.into_push_pull_output()`, but WITHOUT its `&mut crh` config-register-handle wart: the chip
// is detected at runtime, so the register model (CRL/CRH vs CTL/OMODE/OSPD) is a runtime `GpioPath`
// the `Pin` carries, and `into_push_pull_output` drives the existing `configure_output` branch
// itself. Unlike a compile-time PAC, a `Pin` is NOT zero-sized: it carries its resolved port base,
// the runtime `GpioPath`, and its pin number. That few-bytes-per-pin cost is the price of runtime
// detection (a compile-time HAL folds all of that into the type).

/// Marker: a floating input (the pin's reset configuration). Used as the `PULL` of [`Input`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Floating;

/// Marker: an input with the internal pull-up enabled (the pin idles high). Used as the `PULL` of
/// [`Input`]; reached from any [`Pin`] via [`Pin::into_pull_up_input`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PullUp;

/// Marker: an input with the internal pull-down enabled (the pin idles low). Used as the `PULL` of
/// [`Input`]; reached from any [`Pin`] via [`Pin::into_pull_down_input`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PullDown;

/// Marker: a push-pull output drive. Used as the `OTYPE` of [`Output`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushPull;

/// Type-state marker for a pin in INPUT mode, parameterised by its pull configuration `PULL`
/// (e.g. [`Floating`]). A freshly [`GpioPort::split`] pin is `Input<Floating>` (the reset state).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Input<PULL> {
    _pull: core::marker::PhantomData<PULL>,
}

/// Type-state marker for a pin in OUTPUT mode, parameterised by its output drive type `OTYPE`
/// (e.g. [`PushPull`]). Reached from any [`Pin`] via [`Pin::into_push_pull_output`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Output<OTYPE> {
    _otype: core::marker::PhantomData<OTYPE>,
}

/// A single GPIO pin in a type-state mode `MODE`, the runtime-detection analogue of a
/// stm32f1xx-hal pin.
///
/// It carries (at runtime, not in the type) its resolved port base, the chip's [`GpioPath`]
/// register-model selector, and its pin number 0..15. `MODE` tracks the configured mode in the type
/// ([`Input<Floating>`] at reset, [`Output<PushPull>`] after [`Pin::into_push_pull_output`]); the
/// `embedded-hal` output traits are implemented only for the output mode, so calling `set_high` on
/// an unconfigured input pin is a compile error.
///
/// This is NOT zero-sized (it holds the runtime base + path + pin); see the module note on the
/// runtime-detection cost. For the simple "resolve and drive" path that does not need type-state,
/// [`crate::Chip::output_pin`] returns a [`GpioOutput`] directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin<MODE> {
    port_base: u32,
    path: GpioPath,
    pin: u8,
    _mode: core::marker::PhantomData<MODE>,
}

impl<MODE> Pin<MODE> {
    /// Construct a pin handle in a given mode (internal; the typed constructors below are the API).
    #[inline]
    const fn new(port_base: u32, path: GpioPath, pin: u8) -> Self {
        Self {
            port_base,
            path,
            pin,
            _mode: core::marker::PhantomData,
        }
    }

    /// The pin number (0..15) within its port.
    #[inline]
    pub const fn pin(&self) -> u8 {
        self.pin
    }

    /// The resolved base address of this pin's port (HAL-internal).
    ///
    /// Used by peripheral bring-ups that CONSUME `Pin` handles (e.g. [`crate::i2c::I2c::new`]) to
    /// drive [`configure_af`] without exposing the [`GpioPath`] register model to the application.
    #[inline]
    pub(crate) const fn port_base(&self) -> u32 {
        self.port_base
    }

    /// The chip's GPIO register-model selector this pin carries (HAL-internal). See
    /// [`Pin::port_base`]. The logical `(port << 4) | pin` byte is [`Pin::pin`].
    #[inline]
    pub(crate) const fn path(&self) -> GpioPath {
        self.path
    }

    /// Reconfigure this pin as a general-purpose push-pull output, consuming the old typed value.
    ///
    /// Drives the existing [`configure_output`] (which owns the F10x CRL/CRH vs F1x0
    /// CTL/OMODE/OSPD branch internally), so NO config-register handle is passed by the caller
    /// (the difference from stm32f1xx-hal's `into_push_pull_output(&mut gpioc.crh)`). Returns the
    /// pin re-typed as [`Pin<Output<PushPull>>`], which implements the `embedded-hal`
    /// [`embedded_hal::digital::OutputPin`] / [`embedded_hal::digital::StatefulOutputPin`] traits.
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<Output<PushPull>> {
        configure_output(self.port_base, self.path, self.pin);
        Pin::new(self.port_base, self.path, self.pin)
    }

    /// Reconfigure this pin as a floating digital input (no internal pull), consuming the old typed
    /// value.
    ///
    /// The input counterpart to [`Pin::into_push_pull_output`]: it configures the pin as a floating
    /// input through the HAL's family-internal input config (which owns the F10x CRL/CRH vs F1x0
    /// CTL/PUD branch), so the caller passes NO config-register handle and never sees the
    /// [`GpioPath`] split. Returns the pin re-typed as [`Pin<Input<Floating>>`], which implements
    /// the `embedded-hal` [`embedded_hal::digital::InputPin`] trait.
    #[inline]
    pub fn into_floating_input(self) -> Pin<Input<Floating>> {
        configure_input(self.port_base, self.path, self.pin, InputPull::Floating);
        Pin::new(self.port_base, self.path, self.pin)
    }

    /// Reconfigure this pin as a digital input with the internal pull-up enabled (the pin idles
    /// high), consuming the old typed value.
    ///
    /// Like [`Pin::into_floating_input`], configures the pin with the family branch internal. On
    /// F10x the pull direction is the pin's `GPIO_OCTL` bit (set = pull-up); on F1x0 it is the
    /// `GPIO_PUD` field (`01`). Returns the pin re-typed as [`Pin<Input<PullUp>>`], an
    /// [`embedded_hal::digital::InputPin`].
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<Input<PullUp>> {
        configure_input(self.port_base, self.path, self.pin, InputPull::PullUp);
        Pin::new(self.port_base, self.path, self.pin)
    }

    /// Reconfigure this pin as a digital input with the internal pull-down enabled (the pin idles
    /// low), consuming the old typed value.
    ///
    /// Like [`Pin::into_floating_input`], configures the pin with the family branch internal. On
    /// F10x the pull direction is the pin's `GPIO_OCTL` bit (clear = pull-down); on F1x0 it is the
    /// `GPIO_PUD` field (`10`). Returns the pin re-typed as [`Pin<Input<PullDown>>`], an
    /// [`embedded_hal::digital::InputPin`].
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<Input<PullDown>> {
        configure_input(self.port_base, self.path, self.pin, InputPull::PullDown);
        Pin::new(self.port_base, self.path, self.pin)
    }
}

impl Pin<Output<PushPull>> {
    /// Borrow this configured output as a [`GpioOutput`] handle (the shared `GPIO_BOP` drive logic).
    ///
    /// The output trait impls below delegate to this so the BOP set/reset write lives in exactly one
    /// place ([`set_pin`], via [`GpioOutput`]); the `Pin` does not duplicate it.
    #[inline]
    fn as_output(&self) -> GpioOutput {
        GpioOutput::new(self.port_base, self.path, self.pin)
    }
}

impl embedded_hal::digital::ErrorType for Pin<Output<PushPull>> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Pin<Output<PushPull>> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        // Delegate to the shared GpioOutput::set_high (the single GPIO_BOP set write of `set_pin`).
        embedded_hal::digital::OutputPin::set_high(&mut self.as_output())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        embedded_hal::digital::OutputPin::set_low(&mut self.as_output())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Pin<Output<PushPull>> {
    /// Report the last commanded level by reading back the family's GPIO output-data register
    /// (`GPIO_OCTL`, offset `0x0C` on both register models): the `Pin` is `Copy` and carries no
    /// tracked state field, so the live level is read from hardware rather than remembered.
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        let octl = Reg32::new(self.port_base, GPIO_OCTL).read();
        Ok(octl & (1u32 << (self.pin as u32 & 0x0F)) != 0)
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!embedded_hal::digital::StatefulOutputPin::is_set_high(
            self,
        )?)
    }
}

// --- type-state input read (embedded-hal InputPin) ---------------------------------------------
//
// A configured `Pin<Input<PULL>>` reads its live level through the standard embedded-hal
// `digital::InputPin` trait. The read is the single `GPIO_ISTAT` access of `read_pin`, which owns
// the family offset branch (F10x 0x08 / F1x0 0x10) internally, so the application never sees the
// `GpioPath` split. The impl is generic over the pull marker `PULL` (Floating / PullUp / PullDown):
// the pull only affected how the pin was CONFIGURED; reading the level back is identical.

impl<PULL> embedded_hal::digital::ErrorType for Pin<Input<PULL>> {
    type Error = core::convert::Infallible;
}

impl<PULL> embedded_hal::digital::InputPin for Pin<Input<PULL>> {
    /// Read the pin's live level from the family's `GPIO_ISTAT` register (the single [`read_pin`]
    /// access; F10x at `0x08`, F1x0 at `0x10`). Infallible (the
    /// [`embedded_hal::digital::ErrorType::Error`] is [`core::convert::Infallible`]).
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_pin(self.port_base, self.path, self.pin))
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_pin(self.port_base, self.path, self.pin))
    }
}

/// `GPIO_OCTL` (output-data control) register offset, identical on both register models (F10x
/// `GPIO_OCTL` and F1x0 `GPIO_OCTL` are both at offset `0x0C`). Read back to report the set level
/// of a stateless [`Pin<Output<PushPull>>`].
const GPIO_OCTL: u32 = 0x0C;

/// A resolved GPIO port, parameterised by which port's pin bag it [`split`](GpioPort::split)s into.
///
/// Obtained from the chip via the named getters ([`crate::Chip::gpioa`] .. [`crate::Chip::gpiof`]),
/// each of which resolves the base and enables the port clock first (the stm32f1xx-hal
/// `split(&mut rcc)` clock-enable, done in the getter so the application passes no clock handle) and
/// returns the port pre-typed: `chip.gpioa()` yields `GpioPort<PortAPins>`. [`GpioPort::split`] then
/// hands back that port's named pins directly (so `chip.gpioa()?.split().pa15` resolves). Whether a
/// given port exists at all is a RUNTIME `Result` from the getter (the chip is detected at runtime),
/// NOT a compile-time guarantee.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioPort<P: PortPins> {
    base: u32,
    path: GpioPath,
    _pins: core::marker::PhantomData<P>,
}

impl<P: PortPins> GpioPort<P> {
    /// Construct a port handle (internal; built by the chip's port getters, which enable the clock).
    #[inline]
    pub(crate) const fn new(base: u32, path: GpioPath) -> Self {
        Self {
            base,
            path,
            _pins: core::marker::PhantomData,
        }
    }

    /// The resolved port base address.
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    /// Consume the port and hand back its named pins, each in its reset [`Input<Floating>`] state.
    ///
    /// The runtime-detection analogue of stm32f1xx-hal's `gpioa.split()`. The port clock was already
    /// enabled when the chip handed back this `GpioPort` (the chip's port getter does it), so this is
    /// a pure type transition: it constructs the port's pin bag (`PortAPins`, `PortBPins`, ...). Each
    /// pin starts as a [`Pin<Input<Floating>>`]; reconfigure with [`Pin::into_push_pull_output`].
    #[inline]
    pub fn split(self) -> P {
        P::from_port(self.base, self.path)
    }
}

/// A port's pin bag (the `PortXPins` structs). Implemented by the per-port structs the
/// `gpio_port_pins!` macro generates so [`GpioPort::split`] can build the right one generically.
pub trait PortPins {
    /// Build the pin bag for a resolved port base + register-model path (internal to `split`).
    fn from_port(base: u32, path: GpioPath) -> Self;
}

/// Generate a per-port pin-bag struct with one `Input<Floating>` field per pin, plus the
/// [`PortPins::from_port`] constructor that fills it. One invocation per port (this keeps the
/// 16-fields-per-port boilerplate from being hand-written 5 times). Parameters: the struct name, the
/// port letter (a string literal, for the doc comment), the port-number nibble (the high nibble of
/// the logical pin byte: A=0, B=1, C=2, D=3, F=5), then the 16 `pin => field` pairs. Stable Rust has
/// no identifier concatenation, so the field names (`pa0`, `pa1`, ...) are spelled out at the call
/// site rather than synthesized from the letter.
macro_rules! gpio_pin_struct {
    ($struct_name:ident, $letter:literal, $port_nibble:literal,
        [$($pin:literal => $field:ident),+ $(,)?]) => {
        #[doc = concat!("The 16 named pins of GPIO port ", $letter, ", each in its reset")]
        /// [`Input<Floating>`] state. Produced by [`GpioPort::split`] for that port. Reconfigure a
        /// pin with [`Pin::into_push_pull_output`].
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[allow(missing_docs)]
        pub struct $struct_name {
            $(pub $field: Pin<Input<Floating>>,)+
        }

        impl PortPins for $struct_name {
            /// Build the pin bag for a resolved port base + register-model path. Each pin carries the
            /// logical `(port << 4) | pin` byte; only the low nibble drives the register writes.
            #[inline]
            fn from_port(base: u32, path: GpioPath) -> Self {
                Self {
                    $($field: Pin::new(base, path, ($pin as u8) | (($port_nibble as u8) << 4)),)+
                }
            }
        }
    };
}

gpio_pin_struct! { PortAPins, "A", 0,
[0 => pa0, 1 => pa1, 2 => pa2, 3 => pa3, 4 => pa4, 5 => pa5, 6 => pa6, 7 => pa7,
 8 => pa8, 9 => pa9, 10 => pa10, 11 => pa11, 12 => pa12, 13 => pa13, 14 => pa14, 15 => pa15] }
gpio_pin_struct! { PortBPins, "B", 1,
[0 => pb0, 1 => pb1, 2 => pb2, 3 => pb3, 4 => pb4, 5 => pb5, 6 => pb6, 7 => pb7,
 8 => pb8, 9 => pb9, 10 => pb10, 11 => pb11, 12 => pb12, 13 => pb13, 14 => pb14, 15 => pb15] }
gpio_pin_struct! { PortCPins, "C", 2,
[0 => pc0, 1 => pc1, 2 => pc2, 3 => pc3, 4 => pc4, 5 => pc5, 6 => pc6, 7 => pc7,
 8 => pc8, 9 => pc9, 10 => pc10, 11 => pc11, 12 => pc12, 13 => pc13, 14 => pc14, 15 => pc15] }
gpio_pin_struct! { PortDPins, "D", 3,
[0 => pd0, 1 => pd1, 2 => pd2, 3 => pd3, 4 => pd4, 5 => pd5, 6 => pd6, 7 => pd7,
 8 => pd8, 9 => pd9, 10 => pd10, 11 => pd11, 12 => pd12, 13 => pd13, 14 => pd14, 15 => pd15] }
gpio_pin_struct! { PortFPins, "F", 5,
[0 => pf0, 1 => pf1, 2 => pf2, 3 => pf3, 4 => pf4, 5 => pf5, 6 => pf6, 7 => pf7,
 8 => pf8, 9 => pf9, 10 => pf10, 11 => pf11, 12 => pf12, 13 => pf13, 14 => pf14, 15 => pf15] }

#[cfg(test)]
mod tests;
