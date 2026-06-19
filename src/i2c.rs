//! Shared I2C driver (T6 bring-up + T7 polled transfer / `embedded-hal` `i2c::I2c`).
//!
//! This is the single I2C bring-up + transfer path. SPEC.md: I2C is the **classic event-based**
//! block on both families (CTL0/CTL1/STAT0/STAT1/CKCFG/RT, the STM32F1-style peripheral, no
//! `TIMINGR`), so unlike [`crate::usart`] there is **one register model shared by both families**:
//! the I2C peripheral offsets and bit positions are identical on F10x and F1x0 (verified against
//! `gd32f10x_i2c.h` and `gd32f1x0_i2c.h`). The path is parameterised only by the base address
//! (data, from [`crate::addr::AddrTable`]) and by the bus clock (from the [`ClockConfig`]); there
//! is no [`crate::ClockPath`]-style selector here.
//!
//! # Register model (identical on both families)
//!
//! | reg     | offset | what                                                              |
//! |---------|--------|-------------------------------------------------------------------|
//! | `CTL0`  | `0x00` | I2CEN(0), START(8), STOP(9), ACKEN(10) (`*_i2c.h`)                 |
//! | `CTL1`  | `0x04` | `I2CCLK[6:0]` peripheral-clock-MHz field (`I2C_CTL1_I2CCLK`)       |
//! | `SADDR0`| `0x08` | own address + address format (7-bit)                              |
//! | `DATA`  | `0x10` | transfer buffer (address byte / TX / RX)                          |
//! | `STAT0` | `0x14` | SBSEND(0) ADDSEND(1) BTC(2) RBNE(6) TBE(7) BERR(8) LOSTARB(9) AERR(10) OUERR(11) |
//! | `STAT1` | `0x18` | MASTER(0) I2CBSY(1) TR(2); read after STAT0 clears ADDSEND        |
//! | `CKCFG` | `0x1C` | `CLKC[11:0]`, DTCY(14), FAST(15)                                  |
//! | `RT`    | `0x20` | `RISETIME[6:0]`                                                   |
//!
//! # Timing (CKCFG / RT) and the CKCFG formula (open item I2C-1)
//!
//! Bus speed is carried as a free `u32` Hz in the [`I2cMode`] passed to [`I2c::new`] (not a fixed
//! standard/fast enum table), so any rate the SPL accepts is expressible; the IMU runs at 100 kHz, the
//! reference firmware at 400 kHz, and both fall out of the same formula. The CKCFG/RT/I2CCLK values
//! are computed by [`timing_for`] EXACTLY as the GD SPL `i2c_clock_config` does, from the APB1 bus
//! clock (which [`ClockConfig`] + the I2C-on-APB1 fact give) and the target speed:
//!
//! - `I2CCLK = clamp(pclk1 / 1_000_000, 2, 0x48)` (the peripheral-clock MHz field, `I2CCLK_MIN`..`MAX`).
//! - **Standard mode** (speed <= 100 kHz): `RT = clamp(pclk1/1e6 + 1, 2, 0x48)`,
//!   `CLKC = max(pclk1 / (speed*2), 4)`, `CKCFG = CLKC` (FAST = 0, DTCY = 0).
//! - **Fast mode** (100 kHz < speed <= 400 kHz): `RT = (I2CCLK*300)/1000 + 1`; with DTCY_2
//!   `CLKC = pclk1/(speed*3)` (DTCY = 0), with DTCY_16/9 `CLKC = pclk1/(speed*25)` (DTCY = 1);
//!   if CLKC's low 12 bits are 0 force CLKC |= 1; `CKCFG = FAST | (DTCY?) | CLKC`.
//!
//! Because CKCFG depends on the APB1 clock, a clock-tree mistake (wrong PLL / prescaler) surfaces
//! here as a wrong CKCFG value, exactly the way M1's BAUD caught a wrong USART clock. At the proven
//! 72 MHz tree APB1 = 36 MHz, so at 100 kHz: I2CCLK = 36 (0x24), RT = 37 (0x25), CKCFG = 180 (0xB4).
//!
//! The bench probe (`i2c_probe.c`) ran at the 8 MHz reset clock (pclk1 = 8 MHz), which gives
//! I2CCLK = 8, RT = 9, CKCFG = 40; runtime-hal reproduces whatever the supplied [`ClockConfig`]
//! implies, so the same code matches the probe at 8 MHz and the 72 MHz bring-up.
//!
//! # Transfer sequence (T7, the classic event-based STAT0/STAT1 handshake)
//!
//! The polled master read/write sequence mirrors the bench-proven `i2c_probe.c` (a polled probe on
//! the same peripheral that read the IMU): START -> SBSEND -> write address to DATA -> ADDSEND ->
//! clear ADDSEND by reading STAT0 then STAT1 -> data via TBE/BTC (transmit) or RBNE (receive) ->
//! STOP, with the single-byte-receive ACK-disable-before-clearing-ADDSEND + STOP-before-RBNE
//! sequence the block requires, and the repeated-START for a register read. Every poll is bounded
//! ([`I2C_TIMEOUT`]) so a missing/stuck device cannot hang (the F130 hang-if-done-wrong class).
//!
//! The GD/ST register naming stays on this side of the trait boundary (SPEC.md): what crosses into
//! `embedded-hal` `i2c::I2c` is bytes, a 7-bit address, and an [`I2cError`].

use embedded_hal::i2c::{self, Operation, SevenBitAddress};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::clock::ClockConfig;
use crate::error::{DescriptorError, I2cError};
use crate::gpio::{configure_af, Pin, PinRole};
use crate::reg::Reg32;

// --- register offsets (identical on both families) --------------------------------------------

const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
const SADDR0: u32 = 0x08;
const DATA: u32 = 0x10;
const STAT0: u32 = 0x14;
const STAT1: u32 = 0x18;
const CKCFG: u32 = 0x1C;
const RT: u32 = 0x20;

// CTL0 bits.
const CTL0_I2CEN: u32 = 1 << 0;
const CTL0_SMBEN: u32 = 1 << 1;
const CTL0_START: u32 = 1 << 8;
const CTL0_STOP: u32 = 1 << 9;
const CTL0_ACKEN: u32 = 1 << 10;

// CTL1 / CKCFG / RT fields.
const CTL1_I2CCLK: u32 = 0x7F; // BITS(0,6)
const CKCFG_CLKC: u32 = 0xFFF; // BITS(0,11)
const CKCFG_DTCY: u32 = 1 << 14;
const CKCFG_FAST: u32 = 1 << 15;

// SADDR0 / address format. 7-bit address format is the 0 value of the ADDFORMAT field; the SPL
// masks the address with I2C_ADDRESS_MASK (BITS(1,9)) and ORs the format in. The 7-bit own address
// is held left-shifted by one in bits [7:1], the same field the master sends.
const I2C_ADDRESS_MASK: u32 = 0x03FF; // BITS(0,9): masks the address bits the SPL writes

// STAT0 flags.
const STAT0_SBSEND: u32 = 1 << 0;
const STAT0_ADDSEND: u32 = 1 << 1;
const STAT0_BTC: u32 = 1 << 2;
const STAT0_RBNE: u32 = 1 << 6;
const STAT0_TBE: u32 = 1 << 7;
const STAT0_BERR: u32 = 1 << 8;
const STAT0_LOSTARB: u32 = 1 << 9;
const STAT0_AERR: u32 = 1 << 10;
const STAT0_OUERR: u32 = 1 << 11;

/// SPL `I2CCLK_MAX` / `I2CCLK_MIN` clamp bounds for the peripheral-clock MHz field.
const I2CCLK_MAX: u32 = 0x48;
const I2CCLK_MIN: u32 = 0x02;

/// Bounded poll budget for a single status-flag wait. The bench probe used 40000 loops at 8 MHz;
/// this is the same idea: generous enough never to false-time on a working 100/400 kHz byte
/// timing, but always escaping a dead bus. It counts loop iterations, not cycles, so it is
/// clock-independent.
pub const I2C_TIMEOUT: u32 = 100_000;

/// Fast-mode duty cycle (open item I2C-1). `Two` (Tlow/Thigh = 2, DTCY = 0) is the default the IMU
/// reference and the bench probe use; `SixteenNine` (16/9, DTCY = 1) is expressible for callers
/// that need it. Ignored in standard mode (<= 100 kHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FastDuty {
    /// Tlow/Thigh = 2 (DTCY = 0), the SPL `I2C_DTCY_2` default.
    #[default]
    Two,
    /// Tlow/Thigh = 16/9 (DTCY = 1), the SPL `I2C_DTCY_16_9`.
    SixteenNine,
}

/// The I2C bus mode: the target SCL frequency plus, for fast mode, the duty cycle.
///
/// This repackages the old `I2cConfig`'s `speed_hz` + `duty` into the small mode value [`I2c::new`]
/// takes (the stm32f1xx-hal `Mode` analogue). [`I2cMode::standard`] is for <= 100 kHz (the duty is
/// ignored there, so it is not even named); [`I2cMode::fast`] names the fast-mode duty explicitly.
/// The two values flow straight into [`timing_for`] (the GD SPL `i2c_clock_config` math).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cMode {
    /// Target SCL frequency in Hz (100_000 standard, 400_000 fast; any SPL-valid rate).
    pub speed_hz: u32,
    /// Fast-mode duty cycle. Ignored by [`timing_for`] in standard mode (<= 100 kHz).
    pub duty: FastDuty,
}

impl I2cMode {
    /// Standard-mode bus at `speed_hz` (<= 100 kHz). The fast-mode duty is irrelevant here, so it
    /// defaults to `FastDuty::Two` and is never consulted by [`timing_for`].
    #[inline]
    pub const fn standard(speed_hz: u32) -> Self {
        Self {
            speed_hz,
            duty: FastDuty::Two,
        }
    }

    /// Fast-mode bus at `speed_hz` (100 kHz < speed <= 400 kHz) with the named duty cycle.
    #[inline]
    pub const fn fast(speed_hz: u32, duty: FastDuty) -> Self {
        Self { speed_hz, duty }
    }
}

/// The computed I2C timing: the three register values the SPL `i2c_clock_config` programs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cTiming {
    /// `CTL1` `I2CCLK[6:0]` field (the APB1 clock in MHz, clamped).
    pub i2cclk: u32,
    /// `CKCFG` value (CLKC plus FAST/DTCY for fast mode).
    pub ckcfg: u32,
    /// `RT` `RISETIME[6:0]` field.
    pub rt: u32,
}

/// Compute the `I2CCLK` / `CKCFG` / `RT` values for a given APB1 bus clock and target speed,
/// byte-for-byte as the GD SPL `i2c_clock_config(i2c, clkspeed, dutycyc)` does.
///
/// `pclk1_hz` is the APB1 clock (the I2C peripheral clock; I2C is on APB1 on both families).
/// `speed_hz` is the target SCL frequency. `duty` selects the fast-mode duty cycle (ignored in
/// standard mode). A speed above 400 kHz is out of the SPL's range and is clamped to fast-mode
/// behaviour at 400 kHz by the caller's contract; here it falls into the fast branch.
pub fn timing_for(pclk1_hz: u32, speed_hz: u32, duty: FastDuty) -> I2cTiming {
    // I2CCLK = pclk1 in MHz, clamped to I2CCLK_MAX (the SPL clamps the high side only here).
    let freq = (pclk1_hz / 1_000_000).min(I2CCLK_MAX);
    let i2cclk = freq;

    if speed_hz <= 100_000 {
        // Standard mode. RT = pclk1_MHz + 1, clamped to [MIN, MAX] (the SPL's if/elseif/else clamp).
        let risetime = (pclk1_hz / 1_000_000) + 1;
        let rt = risetime.clamp(I2CCLK_MIN, I2CCLK_MAX);
        // CLKC = pclk1 / (speed*2), min 4 in standard mode. CKCFG = CLKC (FAST=0, DTCY=0).
        let mut clkc = pclk1_hz / (speed_hz * 2);
        if clkc < 0x04 {
            clkc = 0x04;
        }
        I2cTiming {
            i2cclk,
            ckcfg: clkc & CKCFG_CLKC,
            rt,
        }
    } else {
        // Fast mode. RT = (I2CCLK*300)/1000 + 1.
        let rt = (freq * 300) / 1000 + 1;
        let (mut clkc, dtcy_bit) = match duty {
            FastDuty::Two => (pclk1_hz / (speed_hz * 3), 0),
            FastDuty::SixteenNine => (pclk1_hz / (speed_hz * 25), CKCFG_DTCY),
        };
        // The SPL forces the CLKC field to at least 1 in fast mode.
        if clkc & CKCFG_CLKC == 0 {
            clkc |= 0x0001;
        }
        I2cTiming {
            i2cclk,
            ckcfg: CKCFG_FAST | dtcy_bit | clkc,
            rt,
        }
    }
}

/// Derive the I2C peripheral (APB1) clock in Hz from a [`ClockConfig`]. I2C lives on APB1 on both
/// families, so this is `AHB / apb1_psc = (sysclk / ahb_psc) / apb1_psc`, the same chain the SPL
/// `rcu_clock_freq_get(CK_APB1)` walks from the prescaler bits and that [`crate::usart`] uses for
/// the USART APB1 input clock.
#[inline]
pub fn i2c_input_clock(clock: &ClockConfig) -> u32 {
    let ahb = clock.sysclk_hz / clock.ahb_psc.max(1) as u32;
    ahb / clock.apb1_psc.max(1) as u32
}

// --- the handle -------------------------------------------------------------------------------

/// A configured I2C master, resolved once at bring-up: just the base (the register model is shared,
/// so there is no per-family field). The polled transfer primitives and the `embedded-hal`
/// `i2c::I2c` impl hang off this (DECISIONS.md #4: resolve once into a concrete handle).
#[derive(Debug, Clone, Copy)]
pub struct I2c {
    base: u32,
}

/// The master own-address value programmed into `SADDR0` (the bench probe's `0x24`). As a
/// single-master bus controller the own address is rarely matched; the SPL programs it, so we keep
/// the bench-validated value rather than leave it unset (the bring-up byte-for-byte agreement with
/// the SPL only depends on writing it directly the SPL way).
const MASTER_OWN_ADDR: u8 = 0x24;

impl I2c {
    /// Bring up the I2C master, CONSUMING the SCL/SDA [`Pin`] handles from `split()`.
    ///
    /// This is the headline I2C API, the stm32f1xx-hal `I2c::new(I2C1, (scl, sda), mode, &mut rcc)`
    /// analogue: the application passes the named pins it got from `chip.gpiob().split()` (e.g.
    /// `gpiob.pb6` / `gpiob.pb7`), never a packed `(port << 4) | pin` byte, and never sees the
    /// [`crate::descriptor::GpioPath`] register model. Generic over the pins' current mode markers
    /// `S` / `D` (they come from `split()` in their reset [`crate::gpio::Input`] state); `new`
    /// reconfigures them and so takes them by value.
    ///
    /// Steps, in order:
    /// 1. Resolve the I2C `instance` to its base from the chip's address table.
    /// 2. Enable the I2C peripheral clock (`enable_i2c`). The SCL/SDA **GPIO port** clock was already
    ///    enabled when the chip handed back the [`crate::gpio::GpioPort`] the pins were split from
    ///    (the `split(&mut rcc)` clock-enable lives in `chip.gpiob()`), so it is not re-enabled here.
    /// 3. Configure both pins as AF open-drain with pull-up ([`PinRole::I2cAfOpenDrain`]) via
    ///    [`configure_af`], which owns the F10x/F1x0 register-model branch internally (PB6/PB7 at
    ///    AF1 on F1x0, AF-OD nibble on F10x: the bench-validated mux).
    /// 4. Program the timing from `clock` + `mode` and enable the peripheral + ACK (the SPL
    ///    `i2c_clock_config` -> `i2c_mode_addr_config` -> `i2c_enable` -> `i2c_ack_config` sequence).
    ///
    /// The returned [`I2c`] implements [`embedded_hal::i2c::I2c`], so an IMU driver generic over that
    /// trait drives it directly.
    pub fn new<S, D>(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        pins: (Pin<S>, Pin<D>),
        mode: I2cMode,
    ) -> Result<I2c, DescriptorError> {
        let base = chip.base(instance)?;
        let (scl, sda) = pins;

        // 2. Enable the I2C peripheral clock (the GPIO-port clock was enabled by `chip.gpiob()`).
        crate::clock::enable_i2c(chip.rcu_base()?, chip.clock(), instance)?;

        // 3. SCL/SDA as AF open-drain with pull-up. Take the resolved port base + register-model path
        //    + logical pin byte straight from each consumed Pin; the application never built them.
        configure_af(
            scl.port_base(),
            scl.path(),
            scl.pin(),
            PinRole::I2cAfOpenDrain,
        );
        configure_af(
            sda.port_base(),
            sda.path(),
            sda.pin(),
            PinRole::I2cAfOpenDrain,
        );

        // 4. Timing + mode + enable + ACK.
        let dev = I2c { base };
        let pclk1 = i2c_input_clock(clock);
        dev.configure_timing(timing_for(pclk1, mode.speed_hz, mode.duty));
        dev.configure_mode_addr(MASTER_OWN_ADDR);
        // i2c_enable: set I2CEN.
        dev.ctl0().modify(CTL0_I2CEN, CTL0_I2CEN);
        // i2c_ack_config(ENABLE): set ACKEN.
        dev.set_ack(true);
        Ok(dev)
    }

    /// Program CTL1 I2CCLK, RT, then CKCFG, exactly as `i2c_clock_config` does (CTL1 via a
    /// clear-then-set of the I2CCLK field; RT and CKCFG are written directly, CKCFG from its reset
    /// 0 so the SPL's `|=` ORs and a single set write reach the same end state).
    fn configure_timing(&self, t: I2cTiming) {
        // CTL1: clear the I2CCLK field, set the computed MHz value (SPL: temp &= ~I2CCLK; temp |= freq).
        self.ctl1().modify(CTL1_I2CCLK, t.i2cclk);
        // RT: the SPL assigns it directly (= risetime), not RMW.
        self.rt().write(t.rt);
        // CKCFG: the SPL ORs CLKC/FAST/DTCY into a reset-0 register; the net value is t.ckcfg.
        self.ckcfg().write(t.ckcfg);
    }

    /// Select I2C mode (clear SMBEN) and program the own-address register, exactly as
    /// `i2c_mode_addr_config(i2c, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, own_addr)` does.
    ///
    /// `I2C_I2CMODE_ENABLE` is 0 and `I2C_ADDFORMAT_7BITS` is 0, so this clears SMBEN in CTL0 and
    /// writes `own_addr & I2C_ADDRESS_MASK` to SADDR0. NOTE: the SPL's `addr` argument is the **raw
    /// SADDR0 value** (already positioned), NOT a 7-bit address to shift, so `own_addr` is written
    /// directly the SPL way; the bench probe passes `0x24` and it lands as `0x24`. As a
    /// single-master bus controller the own address is rarely matched, so the exact value only has
    /// to agree with the SPL byte-for-byte, which writing it directly guarantees.
    fn configure_mode_addr(&self, own_addr: u8) {
        // CTL0: clear SMBEN (select I2C, not SMBus); mode value is 0 so nothing is OR'd in.
        self.ctl0().modify(CTL0_SMBEN, 0);
        // SADDR0: 7-bit format (0) | the own-address register value, masked the SPL way.
        self.saddr0().write((own_addr as u32) & I2C_ADDRESS_MASK);
    }

    /// The underlying base address (for code that needs the register-level view).
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    // --- register accessors -------------------------------------------------------------------

    #[inline]
    fn ctl0(&self) -> Reg32 {
        Reg32::new(self.base, CTL0)
    }
    #[inline]
    fn ctl1(&self) -> Reg32 {
        Reg32::new(self.base, CTL1)
    }
    #[inline]
    fn saddr0(&self) -> Reg32 {
        Reg32::new(self.base, SADDR0)
    }
    #[inline]
    fn data(&self) -> Reg32 {
        Reg32::new(self.base, DATA)
    }
    #[inline]
    fn stat0(&self) -> Reg32 {
        Reg32::new(self.base, STAT0)
    }
    #[inline]
    fn stat1(&self) -> Reg32 {
        Reg32::new(self.base, STAT1)
    }
    #[inline]
    fn ckcfg(&self) -> Reg32 {
        Reg32::new(self.base, CKCFG)
    }
    #[inline]
    fn rt(&self) -> Reg32 {
        Reg32::new(self.base, RT)
    }

    // --- low-level event primitives (the SPL `i2c_*` calls, GD-named) -------------------------

    /// `i2c_start_on_bus`: set CTL0 START.
    #[inline]
    fn start_on_bus(&self) {
        self.ctl0().modify(CTL0_START, CTL0_START);
    }

    /// `i2c_stop_on_bus`: set CTL0 STOP.
    #[inline]
    fn stop_on_bus(&self) {
        self.ctl0().modify(CTL0_STOP, CTL0_STOP);
    }

    /// `i2c_ack_config`: set or clear CTL0 ACKEN.
    #[inline]
    fn set_ack(&self, enable: bool) {
        self.ctl0()
            .modify(CTL0_ACKEN, if enable { CTL0_ACKEN } else { 0 });
    }

    /// `i2c_master_addressing`: write the address byte to DATA. `read` selects the R/W bit:
    /// transmitter clears bit 0, receiver sets it (matching the SPL's `& I2C_TRANSMITTER` /
    /// `| I2C_RECEIVER`). `addr7` is the 7-bit address; the byte sent is `(addr7 << 1) | rw`.
    #[inline]
    fn send_address(&self, addr7: u8, read: bool) {
        let byte = ((addr7 as u32) << 1) | (read as u32);
        self.data().write(byte);
    }

    /// `i2c_data_transmit`: write a data byte to DATA.
    #[inline]
    fn transmit(&self, byte: u8) {
        self.data().write(byte as u32);
    }

    /// `i2c_data_receive`: read a data byte from DATA.
    #[inline]
    fn receive(&self) -> u8 {
        (self.data().read() & 0xFF) as u8
    }

    /// Clear ADDSEND the block's way: read STAT0 then STAT1 (the read pair clears the flag).
    #[inline]
    fn clear_addsend(&self) {
        let _ = self.stat0().read();
        let _ = self.stat1().read();
    }

    /// Poll STAT0 until `flag` is set, mapping a concurrently-set error flag (BERR/LOSTARB/AERR) to
    /// the corresponding [`I2cError`], and a budget exhaustion to [`I2cError::Timeout`]. `nack_src`
    /// names whether an AERR seen here is an address NACK or a data NACK (open item I2C-2). Returns
    /// the STAT0 snapshot that satisfied `flag` on success.
    fn wait_flag(&self, flag: u32, nack_src: NackKind) -> Result<u32, I2cError> {
        let mut budget = I2C_TIMEOUT;
        loop {
            let s = self.stat0().read();
            if s & flag != 0 {
                return Ok(s);
            }
            // Surface a bus/arbitration/NACK error if it appears while waiting (the probe checks
            // these in the same loop). BERR/LOSTARB first (bus-level), then AERR (NACK).
            if s & STAT0_BERR != 0 {
                return Err(I2cError::Bus);
            }
            if s & STAT0_LOSTARB != 0 {
                return Err(I2cError::ArbitrationLoss);
            }
            if s & STAT0_AERR != 0 {
                return Err(match nack_src {
                    NackKind::Address => I2cError::NoAcknowledgeAddress,
                    NackKind::Data => I2cError::NoAcknowledgeData,
                });
            }
            if s & STAT0_OUERR != 0 {
                return Err(I2cError::Overrun);
            }
            budget -= 1;
            if budget == 0 {
                return Err(I2cError::Timeout);
            }
        }
    }

    // --- polled master transfers (the classic event-based sequence) ---------------------------

    /// Polled master write: START -> address+W -> ADDSEND (clear) -> each byte via TBE then a final
    /// BTC -> optional STOP. If `stop` is false the transfer is left without a STOP so a repeated
    /// START can follow (the register-pointer phase of a read). Mirrors `i2c_probe.c`'s phase-1.
    ///
    /// An empty `bytes` still does the START + address handshake (an `embedded-hal` zero-length
    /// write is a bus presence check); the trailing BTC wait is skipped when no byte was sent.
    pub fn write_bytes(&self, addr7: u8, bytes: &[u8], stop: bool) -> Result<(), I2cError> {
        self.set_ack(true);
        self.start_on_bus();
        self.wait_flag(STAT0_SBSEND, NackKind::Address)?;
        self.send_address(addr7, false);
        // Wait for ADDSEND (address ACKed); an AERR here is an ADDRESS NACK.
        self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
        self.clear_addsend();

        for (i, &b) in bytes.iter().enumerate() {
            // TBE: the data register can take the next byte. An AERR while sending data is a DATA NACK.
            self.wait_flag(STAT0_TBE, NackKind::Data)?;
            self.transmit(b);
            // After the LAST byte, wait for BTC (byte transfer complete) so the shift register has
            // drained before a STOP or repeated START, exactly as the probe does.
            if i == bytes.len() - 1 {
                self.wait_flag(STAT0_BTC, NackKind::Data)?;
            }
        }

        if stop {
            self.stop_on_bus();
        }
        Ok(())
    }

    /// Polled master read: START -> address+R -> ADDSEND -> data via RBNE -> STOP, with the
    /// single-vs-multi-byte ACK/STOP sequencing the block requires (mirrors `i2c_probe.c`'s
    /// phase-2). `repeated` selects whether this is a repeated START after a write (the START is
    /// issued either way; the flag documents intent and keeps the call sites readable).
    ///
    /// Single byte: disable ACK, clear ADDSEND, program STOP, then wait RBNE and read. Multi byte:
    /// clear ADDSEND, then for each byte (NACK + STOP before the last) wait RBNE and read.
    pub fn read_bytes(&self, addr7: u8, buf: &mut [u8], repeated: bool) -> Result<(), I2cError> {
        let _ = repeated;
        if buf.is_empty() {
            return Ok(());
        }
        self.set_ack(true);
        self.start_on_bus();
        self.wait_flag(STAT0_SBSEND, NackKind::Address)?;
        self.send_address(addr7, true);

        if buf.len() == 1 {
            // Single byte: NACK must be set BEFORE clearing ADDSEND, then STOP, then read.
            self.set_ack(false);
            self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
            self.clear_addsend();
            self.stop_on_bus();
            self.wait_flag(STAT0_RBNE, NackKind::Data)?;
            buf[0] = self.receive();
        } else {
            self.wait_flag(STAT0_ADDSEND, NackKind::Address)?;
            self.clear_addsend();
            let n = buf.len();
            // Index loop (not iter): the last-byte branch needs the position, and `buf[i]` is the
            // receive target, so a plain range loop is the clear form here.
            #[allow(clippy::needless_range_loop)]
            for i in 0..n {
                if i == n - 1 {
                    // Last byte: NACK then STOP, then read it.
                    self.set_ack(false);
                    self.stop_on_bus();
                }
                self.wait_flag(STAT0_RBNE, NackKind::Data)?;
                buf[i] = self.receive();
            }
        }
        // Re-enable ACK for the next transfer (the probe restores it after the STOP).
        self.set_ack(true);
        Ok(())
    }
}

/// Whether an acknowledge failure (AERR) seen during a poll is an address-phase or a data-phase
/// NACK (open item I2C-2: address NACK -> `NoAcknowledge(Address)`, data NACK ->
/// `NoAcknowledge(Data)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NackKind {
    Address,
    Data,
}

// --- embedded-hal 1.0 i2c::I2c ----------------------------------------------------------------

impl i2c::ErrorType for I2c {
    type Error = I2cError;
}

impl i2c::I2c<SevenBitAddress> for I2c {
    /// Read `read.len()` bytes from `address` (a plain master read with its own START/STOP).
    fn read(&mut self, address: SevenBitAddress, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(address, read, false)
    }

    /// Write `write` to `address` with a terminating STOP.
    fn write(&mut self, address: SevenBitAddress, write: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(address, write, true)
    }

    /// Write `write` then, with a repeated START, read `read` (the register-pointer-then-read
    /// pattern; the IMU WHO_AM_I sequence). The write phase does NOT issue a STOP so the read's
    /// START is a repeated START.
    fn write_read(
        &mut self,
        address: SevenBitAddress,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_bytes(address, write, false)?;
        self.read_bytes(address, read, true)
    }

    /// Execute an `embedded-hal` operation list against `address` as one logical transaction.
    ///
    /// Each `Write`/`Read` runs as its own START; a write's STOP is issued only when it is the
    /// final operation, so a `[Write, Read]` list is the repeated-start register read (the IMU
    /// WHO_AM_I sequence). The classic event-based block must program STOP before the last received
    /// byte's RBNE, so a `Read` always terminates with a STOP, which means a `Read` that is not the
    /// final operation is not supported as a non-terminating phase; M2's transactions
    /// (write-then-read) always place the read last. Consecutive same-direction operations are not
    /// coalesced (the trait permits either; this is the simple form).
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let last = operations.len();
        for (i, op) in operations.iter_mut().enumerate() {
            let is_last = i + 1 == last;
            match op {
                Operation::Write(w) => self.write_bytes(address, w, is_last)?,
                Operation::Read(r) => self.read_bytes(address, r, i != 0)?,
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
