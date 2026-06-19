//! Shared USART driver (T6): baud divisor, 8N1 frame, TX/RX + peripheral enable, polled byte I/O.
//!
//! This is the single USART bring-up + transfer path. It is parameterised by the USART base (data,
//! from [`crate::addr::AddrTable`]) and by which register model the part uses ([`UsartModel`],
//! chosen from the runtime [`ClockPath`] selector, the same way [`crate::clock`] / [`crate::gpio`]
//! pick a model). One binary carries both models; the descriptor picks (DECISIONS.md #8). The
//! polled byte primitives ([`Usart::write_byte`], [`Usart::try_read_byte`], [`Usart::read_status`]) are what the T7
//! `embedded-io` layer sits on.
//!
//! # Two register models (the families' USART blocks genuinely differ)
//!
//! GD32F10x and GD32F1x0 do **not** share a USART register layout (F1x0's block is the newer
//! STM32F0-style peripheral). The offsets *and* the CTL0 bit positions differ; only the STAT error
//! /ready bit positions happen to coincide. So [`UsartModel`] carries the per-family offsets/bits:
//!
//! | thing            | F10x (`gd32f10x_usart.h`) | F1x0 (`gd32f1x0_usart.h`) |
//! |------------------|---------------------------|---------------------------|
//! | `STAT`           | `0x00` (`:51`)            | `0x1C` (`:57`)            |
//! | data: TX / RX    | `DATA 0x04` (`:52`)       | `TDATA 0x28` / `RDATA 0x24` (`:59,60`) |
//! | `BAUD`           | `0x08` (`:53`)            | `0x0C` (`:53`)            |
//! | `CTL0`           | `0x0C` (`:54`)            | `0x00` (`:50`)            |
//! | `CTL1`           | `0x10` (`:55`)            | `0x04` (`:51`)            |
//! | `CTL2`           | `0x14` (`:56`)            | `0x08` (`:52`)            |
//! | `CTL0_UEN`       | `BIT(13)` (`:93`)         | `BIT(0)` (`:64`)          |
//! | `CTL0_REN`/`TEN` | `BIT(2)` / `BIT(3)` (`:82,83`) | `BIT(2)` / `BIT(3)` (`:66,67`) |
//! | `CTL0_WL`        | `BIT(12)` (`:92`)        | `BIT(12)` (`:76`)         |
//! | `CTL0_PM`/`PCEN` | `BIT(9)` / `BIT(10)` (`:89,90`) | `BIT(9)` / `BIT(10)` (`:73,74`) |
//! | `CTL1_STB`       | `BITS(12,13)` (`:103`)   | `BITS(12,13)` (`:91` STB) |
//! | `STAT` PERR/FERR/ORERR/RBNE/TC/TBE | `BIT(0/1/3/5/6/7)` (`:61-68`) | `BIT(0/1/3/5/6/7)` (`:146-153`) |
//!
//! 8N1 is `WL = 0` (8 bits), `STB = 0` (1 stop), `PM/PCEN = 0` (no parity), exactly the SPL's
//! `USART_WL_8BIT` / `USART_STB_1BIT` / `USART_PM_NONE` defaults.
//!
//! # Input clock + BAUD divisor (the per-family clock subtlety)
//!
//! The USART input clock is derived from [`ClockConfig::sysclk_hz`] and which APB bus the USART
//! sits on, the way the SPL `usart_baudrate_set` does: it reads `rcu_clock_freq_get(CK_APB2)` for
//! USART0 and `CK_APB1` for USART1/USART2 (`gd32f10x_usart.c:91-101`). On the GD SPL's proven
//! 72 MHz link the clock tree is AHB = sysclk/1, APB2 = AHB/1, APB1 = AHB/2 (`system_gd32f10x.c`
//! `system_clock_72m_*`: `RCU_AHB_CKSYS_DIV1`, `RCU_APB2_CKAHB_DIV1`, `RCU_APB1_CKAHB_DIV2`), the
//! standard "APB1 max is 36 MHz" arrangement that `rcu_clock_freq_get` then reproduces from the
//! prescaler bits. So at 72 MHz sysclk the USART1 (APB1) input clock is **36 MHz**.
//!
//! `apb1_div_for` reproduces that policy: the smallest power-of-two APB1 prescaler that keeps
//! APB1 at or below 36 MHz (so /1 up to 36 MHz, /2 up to 72 MHz, ...). APB2 has no such ceiling
//! at these clocks, so it tracks AHB (= sysclk; M1 runs AHB = sysclk). This matches the dividers a
//! 72 MHz `system_*` setup programs and that `rcu_clock_freq_get` reads back.
//!
//! The BAUD value (oversampling-by-16) is the SPL formula in [`compute_brr`]:
//! `udiv = (uclk + baud/2) / baud`, `BAUD = (udiv & 0xFFF0) | (udiv & 0xF)` = `udiv & 0xFFFF`
//! (`gd32f10x_usart.c:115-118`). For USART1 at 72 MHz sysclk (uclk = 36 MHz) and 115200 baud:
//! `udiv = (36_000_000 + 57_600) / 115_200 = 313`, so `BAUD = 313 = 0x139`.

use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, Parity, UsartConfig, UsartFrame, WordLen};
use crate::descriptor::ClockPath;
use crate::error::{DescriptorError, UsartError};
use crate::gpio::{configure_af, Pin, PinRole};
use crate::reg::Reg32;

// --- bus + clock derivation -------------------------------------------------------------------

/// Which APB bus a USART instance sits on, fixing which APB clock feeds its baud generator.
///
/// GD `USART0` is on APB2; GD `USART1` and `USART2` are on APB1 (the same split [`crate::clock`]
/// encodes as the enable register). This is family-independent (the bus assignment is the same on
/// F10x and F1x0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsartBus {
    /// APB1 (USART1, USART2). Clocked at or below 36 MHz, so a prescaler may apply.
    Apb1,
    /// APB2 (USART0). Tracks AHB / sysclk at M1's clocks.
    Apb2,
}

impl UsartBus {
    /// The APB bus a USART label belongs to. `None` if the label is not a USART instance.
    #[inline]
    pub fn of(usart: crate::addr::PeriphLabel) -> Option<Self> {
        use crate::addr::PeriphLabel::*;
        match usart {
            Usart0 => Some(UsartBus::Apb2),
            Usart1 | Usart2 => Some(UsartBus::Apb1),
            _ => None,
        }
    }
}

/// The power-of-two APB1 prescaler the SPL's 72 MHz-class clock setup implies for `ahb_hz`.
///
/// The GD `system_*` clock configs keep APB1 at or below its 36 MHz ceiling: at AHB <= 36 MHz the
/// prescaler is 1, at AHB <= 72 MHz it is 2, then 4, 8, 16 (the prescaler is a power of two,
/// matching `RCU_CFG0` APB1PSC and the `apb1_exp` table in `rcu_clock_freq_get`). M1 runs AHB =
/// sysclk, so `ahb_hz` is the descriptor's sysclk.
// Retained for reference: M1 derived the APB1 prescaler from a 36 MHz ceiling here. M2 reads the
// prescaler from the profile instead (see `usart_input_clock`), so this is no longer on the path.
#[allow(dead_code)]
#[inline]
fn apb1_div_for(ahb_hz: u32) -> u32 {
    const APB1_MAX_HZ: u32 = 36_000_000;
    let mut div = 1u32;
    while ahb_hz / div > APB1_MAX_HZ && div < 16 {
        div *= 2;
    }
    div
}

/// Derive a USART's input clock (Hz) from the [`ClockConfig`] and the bus it sits on.
///
/// M2 reconciliation (T2): the profile now carries the AHB / APB1 / APB2 prescaler DIVISORS that
/// [`crate::clock::configure_tree`] actually programs into `RCU_CFG0`, so the per-USART input clock
/// is read straight from them (`AHB = sysclk / ahb_psc`, `APBx = AHB / apbx_psc`), exactly the
/// chain the SPL `rcu_clock_freq_get` walks from the same prescaler bits. This keeps the BRR
/// consistent with the tree this build sets up, rather than re-deriving the APB1 prescaler from a
/// 36 MHz ceiling heuristic.
///
/// For an M1-era descriptor (decoded with the defaults `ahb_psc = 1`, `apb1_psc = 2`,
/// `apb2_psc = 1`), this yields APB2 = sysclk and APB1 = sysclk/2 (36 MHz at 72 MHz), identical to
/// the old `apb1_div_for` result, so M1's `BAUD = 0x139` is unchanged.
#[inline]
pub fn usart_input_clock(clock: &ClockConfig, bus: UsartBus) -> u32 {
    let ahb = clock.sysclk_hz / clock.ahb_psc.max(1) as u32;
    match bus {
        UsartBus::Apb2 => ahb / clock.apb2_psc.max(1) as u32,
        UsartBus::Apb1 => ahb / clock.apb1_psc.max(1) as u32,
    }
}

/// Compute the `BAUD` register value (oversampling-by-16) from the input clock and target baud.
///
/// The SPL formula (`gd32f10x_usart.c:115-118`): round-to-nearest divide, then the 12-bit integer
/// part (`INTDIV`, `BITS(4,15)`) and 4-bit fraction (`FRADIV`, `BITS(0,3)`) are just the low 16
/// bits of that divisor, so `BAUD = udiv & 0xFFFF`. `baud` must be non-zero.
#[inline]
pub fn compute_brr(input_clock_hz: u32, baud: u32) -> u16 {
    // (uclk + baud/2) / baud, the SPL's round-to-nearest. u64 math so a high clock cannot overflow.
    let uclk = input_clock_hz as u64;
    let baud = baud as u64;
    let udiv = (uclk + baud / 2) / baud;
    (udiv & 0xFFFF) as u16
}

// --- register model ---------------------------------------------------------------------------

/// The per-family USART register model: register offsets and the CTL0 enable-bit positions that
/// differ between F10x and F1x0. Selected from the [`ClockPath`] selector.
#[derive(Debug, Clone, Copy)]
pub struct UsartModel {
    stat: u32,
    tx_data: u32,
    rx_data: u32,
    baud: u32,
    ctl0: u32,
    ctl1: u32,
    ctl2: u32,
    /// CTL0 `UEN` bit position (differs: F10x bit 13, F1x0 bit 0).
    uen_bit: u8,
    /// F1x0-only interrupt/status-flag-clear register `INTC` offset (`Some` on F1x0, `None` on
    /// F10x). On F1x0 a sticky line flag (overrun/framing/parity) is cleared by writing its `*CF`
    /// bit to `INTC`; F10x has no such register (it clears by reading STAT then the data register),
    /// so this is `None` and [`Usart::clear_line_errors`] takes the read-pair path instead.
    intc: Option<u32>,
}

// Shared CTL0 bit positions (identical on both families).
const CTL0_REN: u32 = 1 << 2;
const CTL0_TEN: u32 = 1 << 3;
/// Parity selection `PM` (0 = even, 1 = odd) and parity-control enable `PCEN`.
const CTL0_PM: u32 = 1 << 9;
const CTL0_PCEN: u32 = 1 << 10;
const CTL0_WL: u32 = 1 << 12;
/// Oversampling-mode bit `OVSMOD` (CTL0 bit 15; identical on both families). 0 = /16, 1 = /8.
const CTL0_OVSMOD: u32 = 1 << 15;
/// CTL1 stop-bit field `STB = BITS(12,13)`.
const CTL1_STB: u32 = 0b11 << 12;

// Shared STAT bit positions (identical on both families).
const STAT_PERR: u32 = 1 << 0;
const STAT_FERR: u32 = 1 << 1;
const STAT_ORERR: u32 = 1 << 3;
const STAT_RBNE: u32 = 1 << 5;
const STAT_TC: u32 = 1 << 6;
const STAT_TBE: u32 = 1 << 7;

// F1x0 `INTC` flag-clear bits (`gd32f1x0_usart.h`): writing the bit clears the matching sticky STAT
// flag. The bit positions mirror the STAT positions for these line errors (PERR/FERR/ORERR at
// 0/1/3), so PECF/FECF/ORECF reuse the same masks. (F10x has no INTC: see `UsartModel::intc`.)
const INTC_PECF: u32 = STAT_PERR;
const INTC_FECF: u32 = STAT_FERR;
const INTC_ORECF: u32 = STAT_ORERR;

impl UsartModel {
    /// The F10x (`gd32f10x_usart.h`) register model.
    pub const F10X: UsartModel = UsartModel {
        stat: 0x00,
        tx_data: 0x04,
        rx_data: 0x04,
        baud: 0x08,
        ctl0: 0x0C,
        ctl1: 0x10,
        ctl2: 0x14,
        uen_bit: 13,
        // F10x (F1-style) has no INTC flag-clear register: a sticky overrun (ORERR) is cleared by
        // the STAT-then-data read pair, so there is no INTC offset here.
        intc: None,
    };

    /// The F1x0 (`gd32f1x0_usart.h`) register model. Separate TX/RX data registers, STAT moved to
    /// `0x1C`, CTL block at the start, `UEN` at bit 0.
    pub const F1X0: UsartModel = UsartModel {
        stat: 0x1C,
        tx_data: 0x28,
        rx_data: 0x24,
        baud: 0x0C,
        ctl0: 0x00,
        ctl1: 0x04,
        ctl2: 0x08,
        uen_bit: 0,
        // F1x0 (F0-style) clears a sticky line flag by writing its `*CF` bit to INTC at 0x20
        // (`gd32f1x0_usart.h`): ORECF (overrun) = BIT(3), FECF (framing) = BIT(1), PECF (parity) =
        // BIT(0). This is the family-correct overrun clear link_bench used (`Reg32(base, 0x20)`).
        intc: Some(0x20),
    };

    /// The register model for a clock path (USART register layout maps 1:1 to the family the clock
    /// path selects).
    #[inline]
    pub const fn for_path(path: ClockPath) -> UsartModel {
        match path {
            ClockPath::F10xRcc => UsartModel::F10X,
            ClockPath::F1x0Rcu => UsartModel::F1X0,
        }
    }

    #[inline]
    const fn uen(&self) -> u32 {
        1 << self.uen_bit
    }
}

// --- bring-up ---------------------------------------------------------------------------------

/// Status snapshot of the line/transfer flags T7 maps to `embedded-io` error kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// Receive overrun (`STAT_ORERR`).
    pub overrun: bool,
    /// Framing error (`STAT_FERR`).
    pub framing: bool,
    /// Parity error (`STAT_PERR`).
    pub parity: bool,
    /// Read data buffer not empty (`STAT_RBNE`): a byte is available.
    pub rx_ready: bool,
    /// Transmit data buffer empty (`STAT_TBE`): the next byte can be written.
    pub tx_empty: bool,
    /// Transmission complete (`STAT_TC`): the shift register has finished.
    pub tx_complete: bool,
}

impl Status {
    /// The first line error present, as a [`UsartError`], if any (overrun, then framing, then
    /// parity). T7 surfaces this through the `embedded-io` seam.
    #[inline]
    pub fn line_error(&self) -> Option<UsartError> {
        if self.overrun {
            Some(UsartError::Overrun)
        } else if self.framing {
            Some(UsartError::Framing)
        } else if self.parity {
            Some(UsartError::Parity)
        } else {
            None
        }
    }
}

/// A configured USART, resolved once at bring-up: the base and the register model. The polled
/// byte primitives hang off this (DECISIONS.md #4: resolve once into a concrete handle).
#[derive(Debug, Clone, Copy)]
pub struct Usart {
    base: u32,
    model: UsartModel,
}

impl Usart {
    /// Bring up the USART at `base` for `path`'s register model: disable, program the BAUD divisor
    /// (from the input clock the [`ClockConfig`] + bus imply and the target `baud`), set the 8N1
    /// frame, enable TX + RX, then enable the peripheral. Returns the configured handle.
    ///
    /// The register writes match the sequence the SPL `usart_*` calls produce: `usart_disable`
    /// (clear `UEN`), `usart_baudrate_set` (`BAUD`), `usart_word_length_set` (`WL = 0`),
    /// `usart_stop_bit_set` (`STB = 0`), `usart_parity_config` (`PM/PCEN = 0`),
    /// `usart_receive_config` / `usart_transmit_config` (`REN` / `TEN`), `usart_enable` (`UEN`).
    pub fn bring_up(
        chip: &Chip,
        clock: &ClockConfig,
        cfg: &UsartConfig,
    ) -> Result<Usart, DescriptorError> {
        let base = chip.base(cfg.usart)?;
        let path = chip.clock();
        let bus = UsartBus::of(cfg.usart).ok_or(DescriptorError::UnknownSelector)?;
        let model = UsartModel::for_path(path);
        Ok(Usart::program(
            base,
            model,
            clock,
            bus,
            cfg.baud,
            cfg.frame,
            cfg.oversampling,
        ))
    }

    /// Bring up a USART CONSUMING the TX/RX [`Pin`] handles from `split()`, the headline pin-handle
    /// constructor (the [`crate::i2c::I2c::new`] analogue for the serial port).
    ///
    /// The application passes the named pins it got from `chip.gpioa().split()` (e.g. `gpioa.pa2` /
    /// `gpioa.pa3`), never a packed `(port << 4) | pin` byte and never the [`crate::descriptor::GpioPath`]
    /// register model. The frame is the M1 default 8N1 with oversample /16; for a different frame use
    /// [`Usart::bring_up`] with a [`UsartConfig`]. Generic over the pins' current mode markers `TX` /
    /// `RX` (they arrive in their reset [`crate::gpio::Input`] state from `split()`); `new`
    /// reconfigures them and so takes them by value.
    ///
    /// Steps, in order (mirroring `I2c::new`):
    /// 1. Resolve the USART `instance` to its base and pick the register model from the chip's clock
    ///    path; the instance fixes the APB bus (so the BAUD input clock is correct).
    /// 2. Enable the USART peripheral clock. The TX/RX **GPIO port** clock was already enabled when
    ///    the chip handed back the [`crate::gpio::GpioPort`] the pins were split from, so it is not
    ///    re-enabled here.
    /// 3. Configure TX as AF push-pull ([`PinRole::Tx`]) and RX as AF input ([`PinRole::Rx`]) via
    ///    [`configure_af`], taking each consumed pin's resolved port base / register-model path /
    ///    logical pin byte (the application never built them).
    /// 4. Program BRR from `clock` + `baud`, the 8N1 /16 frame, and enable TX + RX + the peripheral.
    pub fn new<TX, RX>(
        chip: &Chip,
        clock: &ClockConfig,
        instance: crate::addr::PeriphLabel,
        pins: (Pin<TX>, Pin<RX>),
        baud: u32,
    ) -> Result<Usart, DescriptorError> {
        let base = chip.base(instance)?;
        let path = chip.clock();
        let bus = UsartBus::of(instance).ok_or(DescriptorError::UnknownSelector)?;
        let model = UsartModel::for_path(path);
        let (tx, rx) = pins;

        // 2. Enable the USART peripheral clock (the GPIO-port clock was enabled by `chip.gpioa()`).
        crate::clock::enable_usart(chip.rcu_base()?, path, instance)?;

        // 3. TX as AF push-pull, RX as AF input. Take the resolved port base + register-model path +
        //    logical pin byte straight from each consumed Pin; the application never built them.
        configure_af(tx.port_base(), tx.path(), tx.pin(), PinRole::Tx);
        configure_af(rx.port_base(), rx.path(), rx.pin(), PinRole::Rx);

        // 4. BRR + 8N1 /16 frame + enable. The default M1 settings (the link-validated frame).
        Ok(Usart::program(
            base,
            model,
            clock,
            bus,
            baud,
            UsartFrame::EIGHT_N_ONE,
            Oversampling::By16,
        ))
    }

    /// The shared register-programming body for both bring-up paths: disable, program the BAUD
    /// divisor (from the input clock the [`ClockConfig`] + `bus` imply and the target `baud`), set
    /// the `frame` + `oversampling`, enable TX + RX, then enable the peripheral. Returns the handle.
    ///
    /// The register writes match the sequence the SPL `usart_*` calls produce: `usart_disable`
    /// (clear `UEN`), `usart_baudrate_set` (`BAUD`), `usart_word_length_set` (`WL`),
    /// `usart_stop_bit_set` (`STB`), `usart_parity_config` (`PM/PCEN`), `usart_oversample_config`
    /// (`OVSMOD`), `usart_receive_config` / `usart_transmit_config` (`REN` / `TEN`), `usart_enable`
    /// (`UEN`). For an 8N1 /16 frame every frame field value is 0 (the reset frame).
    fn program(
        base: u32,
        model: UsartModel,
        clock: &ClockConfig,
        bus: UsartBus,
        baud: u32,
        frame: UsartFrame,
        oversampling: Oversampling,
    ) -> Usart {
        let u = Usart { base, model };

        // 1. Disable the peripheral before reconfiguring (usart_disable: clear UEN).
        u.ctl0().modify(model.uen(), 0);

        // 2. BAUD divisor from the per-bus input clock (the ClockConfig + bus imply) + target baud
        //    (usart_baudrate_set).
        let uclk = usart_input_clock(clock, bus);
        let brr = compute_brr(uclk, baud) as u32;
        u.baud_reg().write(brr);

        // 3. Line frame (no baked 8N1): word length (WL), stop bits (STB), parity (PM/PCEN),
        //    oversampling (OVSMOD). Done as the SPL's clear-then-set RMW per field; for an 8N1 /16
        //    frame every field value is 0, so this clears the fields (the reset frame), matching the
        //    SPL `usart_word_length_set`/`_stop_bit_set`/`_parity_config` end state.
        let wl = match frame.word_len {
            WordLen::Eight => 0,
            WordLen::Nine => CTL0_WL,
        };
        u.ctl0().modify(CTL0_WL, wl); // usart_word_length_set
        let stb = (frame.stop as u32) << 12;
        u.ctl1().modify(CTL1_STB, stb & CTL1_STB); // usart_stop_bit_set
        let parity = match frame.parity {
            Parity::None => 0,
            Parity::Even => CTL0_PCEN,
            Parity::Odd => CTL0_PCEN | CTL0_PM,
        };
        u.ctl0().modify(CTL0_PM | CTL0_PCEN, parity); // usart_parity_config
        let ovs = match oversampling {
            Oversampling::By16 => 0,
            Oversampling::By8 => CTL0_OVSMOD,
        };
        u.ctl0().modify(CTL0_OVSMOD, ovs); // usart_oversample_config

        // 4. Enable receiver and transmitter (usart_receive_config / usart_transmit_config).
        u.ctl0().modify(CTL0_REN, CTL0_REN);
        u.ctl0().modify(CTL0_TEN, CTL0_TEN);

        // 5. Enable the peripheral (usart_enable: set UEN).
        u.ctl0().modify(model.uen(), model.uen());

        u
    }

    #[inline]
    fn ctl0(&self) -> Reg32 {
        Reg32::new(self.base, self.model.ctl0)
    }
    #[inline]
    fn ctl1(&self) -> Reg32 {
        Reg32::new(self.base, self.model.ctl1)
    }
    #[inline]
    fn baud_reg(&self) -> Reg32 {
        Reg32::new(self.base, self.model.baud)
    }
    #[inline]
    fn stat(&self) -> Reg32 {
        Reg32::new(self.base, self.model.stat)
    }

    /// CTL2 accessor (exposed for completeness / T7; the M1 8N1 polled path leaves CTL2 at reset).
    #[inline]
    pub fn ctl2(&self) -> Reg32 {
        Reg32::new(self.base, self.model.ctl2)
    }

    /// Read the line/transfer status flags.
    #[inline]
    pub fn read_status(&self) -> Status {
        let s = self.stat().read();
        Status {
            overrun: s & STAT_ORERR != 0,
            framing: s & STAT_FERR != 0,
            parity: s & STAT_PERR != 0,
            rx_ready: s & STAT_RBNE != 0,
            tx_empty: s & STAT_TBE != 0,
            tx_complete: s & STAT_TC != 0,
        }
    }

    /// Write one byte, polling for the transmit buffer to drain then for transmission to complete.
    ///
    /// Polls `TBE` (transmit buffer empty) before writing the data register, then `TC`
    /// (transmission complete) after, the textbook SPL polled-send loop. The data register is the
    /// 9-bit `DATA`/`TDATA` field; the high bits are unused at 8 data bits.
    #[inline]
    pub fn write_byte(&self, byte: u8) {
        while self.stat().read() & STAT_TBE == 0 {}
        Reg32::new(self.base, self.model.tx_data).write(byte as u32);
        while self.stat().read() & STAT_TC == 0 {}
    }

    /// Clear any sticky line-error flags (overrun / framing / parity) the family-correct way, so a
    /// polled stream RX self-recovers instead of latching dead.
    ///
    /// A receive **overrun** (ORERR) is the FIFO-less hazard: the F1/F0 USART has no RX FIFO, so a
    /// byte not taken within one character time overruns, and an uncleared ORERR then blocks all
    /// further RBNE (RX dies). The two families clear it differently, so the [`UsartModel`] owns the
    /// branch (link_bench's `clear_overrun`, generalised to framing/parity too):
    ///
    /// - **F10x** (F1-style, `model.intc == None`): a STAT-then-data-register read pair clears the
    ///   sticky flags. This reads STAT (already in `status`) then the data register, discarding the
    ///   (possibly stale/overwritten) byte; the clear is the point, not the byte.
    /// - **F1x0** (F0-style, `model.intc == Some(off)`): write the matching `*CF` bit (ORECF/FECF/
    ///   PECF) to the `INTC` register at `off`; no data-register read is involved.
    ///
    /// Only the flags present in `status` are cleared (a single masked INTC write on F1x0; the read
    /// pair on F10x clears all of them at once, which is correct because they are all recoverable).
    #[inline]
    fn clear_line_errors(&self, status: &Status) {
        match self.model.intc {
            // F1x0: write the *CF bits for the flags that are set into INTC.
            Some(intc_off) => {
                let mut clr = 0u32;
                if status.overrun {
                    clr |= INTC_ORECF;
                }
                if status.framing {
                    clr |= INTC_FECF;
                }
                if status.parity {
                    clr |= INTC_PECF;
                }
                if clr != 0 {
                    Reg32::new(self.base, intc_off).write(clr);
                }
            }
            // F10x: the STAT-then-data read pair clears the sticky flags. STAT was already read into
            // `status`; read the data register to complete the pair (the byte is discarded).
            None => {
                let _ = Reg32::new(self.base, self.model.rx_data).read();
            }
        }
    }

    /// Try to read one byte without blocking, self-recovering from a receive overrun.
    ///
    /// If a line error is present it is cleared the family-correct way (`clear_line_errors`)
    /// FIRST, so it can never latch and strand RX:
    /// - An **overrun** (ORERR) is RECOVERABLE: after clearing it, return the freshest available byte
    ///   if `RBNE` is still set, else `Ok(None)`. It is never returned as `Err` (an overrun that
    ///   stranded RX was the link_bench bug; the HAL now handles it so the application does not).
    /// - **Framing / parity**, once cleared, still surface as `Err` (the byte they describe is
    ///   suspect) but, having been cleared, they do not latch either: the next call recovers.
    ///
    /// With no line error: if `RBNE` is set, read and return the data byte; otherwise `Ok(None)`.
    /// (T7's blocking `Read`/`ReadReady` polls this; the non-blocking shape keeps that layer honest.)
    #[inline]
    pub fn try_read_byte(&self) -> Result<Option<u8>, UsartError> {
        let status = self.read_status();

        // Overrun is recoverable: clear it (family-correct) and keep going. On F10x the clear is the
        // STAT+data read pair, which also consumes a byte, so re-read STAT afterwards to report the
        // freshest RBNE state. Do NOT return Err for an overrun.
        if status.overrun {
            self.clear_line_errors(&status);
            let after = self.read_status();
            if after.rx_ready {
                let data = Reg32::new(self.base, self.model.rx_data).read();
                return Ok(Some((data & 0xFF) as u8));
            }
            return Ok(None);
        }

        // Framing / parity: clear (so they do not latch) then surface as Err for this call. The next
        // call sees a clean STAT and recovers.
        if status.framing {
            self.clear_line_errors(&status);
            return Err(UsartError::Framing);
        }
        if status.parity {
            self.clear_line_errors(&status);
            return Err(UsartError::Parity);
        }

        if !status.rx_ready {
            return Ok(None);
        }
        let data = Reg32::new(self.base, self.model.rx_data).read();
        Ok(Some((data & 0xFF) as u8))
    }
}

#[cfg(test)]
mod tests;
