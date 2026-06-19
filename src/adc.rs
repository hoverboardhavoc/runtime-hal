//! Shared regular-conversion ADC driver (T10 bring-up + calibration, T11 single-conversion read).
//!
//! This is the single regular-ADC bring-up + read path. SPEC.md: the ADC register **core is
//! shared** (F1x0 has one ADC, F10x two); M2 brings up **ADC0, software-triggered single regular
//! conversion** on BOTH families (the `Single` arm of the `adc` selector; the F10x dual /
//! simultaneous arm is deferred to M3, open item ADC-2). The ADC peripheral register model is
//! **identical on F10x and F1x0** (verified against `gd32f10x_adc.h` and `gd32f1x0_adc.h`: STAT
//! 0x00, CTL0 0x04, CTL1 0x08, SAMPT0 0x0C, SAMPT1 0x10, RSQ0 0x2C, RSQ1 0x30, RSQ2 0x34, RDATA
//! 0x4C, and the same CTL1 bit positions), so there is **one register model shared by both
//! families**; the path is parameterised only by the base address (data, from
//! [`crate::addr::AddrTable`]). Like [`crate::i2c`] / [`crate::spi`] there is no
//! [`crate::descriptor::ClockPath`]-style selector here. The F1x0 ADC clock prescaler / source
//! select is the clock path's job (T4 [`crate::clock::enable_adc`]), not this module's.
//!
//! `embedded-hal` 1.0 has **NO ADC trait** (open item ADC-1), so the read surface is a runtime-hal
//! method ([`Adc::read_channel`]), not a trait impl. See "Read API (ADC-1)" below for the shape
//! chosen and why no `nb` 0.2 `adc::OneShot` shim is offered.
//!
//! # Register model (identical on both families)
//!
//! | reg     | offset | what                                                                    |
//! |---------|--------|-------------------------------------------------------------------------|
//! | `STAT`  | `0x00` | EOC(1) end-of-conversion flag                                           |
//! | `CTL0`  | `0x04` | SM(8) scan mode (left clear: single)                                    |
//! | `CTL1`  | `0x08` | ADCON(0) CTN(1) CLB(2) RSTCLB(3) DAL(11) `ETSRC[19:17]` ETERC(20) SWRCST(22) TSVREN(23) |
//! | `SAMPT0`| `0x0C` | sample time, channels 10..17 (3 bits each)                              |
//! | `SAMPT1`| `0x10` | sample time, channels 0..9 (3 bits each)                                |
//! | `RSQ0`  | `0x2C` | `RL[23:20]` regular sequence length; ranks 12..15                       |
//! | `RSQ1`  | `0x30` | ranks 6..11 (5 bits each)                                               |
//! | `RSQ2`  | `0x34` | ranks 0..5 (5 bits each)                                                |
//! | `RDATA` | `0x4C` | regular conversion result (right-aligned 12-bit)                        |
//!
//! # Bring-up (T10, the SPL `adc_*` single-conversion recipe)
//!
//! [`Adc::bring_up`] reproduces the SPL single-conversion configuration for ADC0, in the order a
//! polled SPL example programs it:
//!
//! - **scan OFF** (`adc_special_function_config(ADC_SCAN_MODE, DISABLE)`): clear CTL0 SM.
//! - **continuous OFF** (`adc_special_function_config(ADC_CONTINUOUS_MODE, DISABLE)`): clear CTL1 CTN.
//! - **right alignment** (`adc_data_alignment_config(ADC_DATAALIGN_RIGHT)`): clear CTL1 DAL.
//! - **regular sequence length** (`adc_channel_length_config(ADC_REGULAR_CHANNEL, len)`): RSQ0 RL =
//!   `len - 1`.
//! - **per-channel sample time + rank** (`adc_regular_channel_config(rank, channel, sample_time)`):
//!   for each entry, set the rank's 5-bit channel field in RSQ2/1/0 and the channel's 3-bit
//!   sample-time field in SAMPT1/0.
//! - **internal-channel enable** (`adc_tempsensor_vrefint_enable`): set CTL1 TSVREN when the
//!   sequence reads channel 16 (temperature) or 17 (VREFINT) (open item ADC-3). Resolution is
//!   fixed 12-bit (no resolution register on these parts; it is the only width).
//! - **software trigger source** (`adc_external_trigger_source_config(ADC_REGULAR_CHANNEL,
//!   ADC_EXTTRIG_REGULAR_NONE)`): CTL1 ETSRC = 7.
//! - **external-trigger enable** (`adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE)`): set
//!   CTL1 ETERC. (The GD block requires ETERC set even for the software trigger; the SPL examples
//!   do this and so do we.)
//! - **enable** (`adc_enable`): set CTL1 ADCON.
//! - **calibration** (`adc_calibration_enable`): set CTL1 RSTCLB and **spin until it clears**, then
//!   set CTL1 CLB and **spin until it clears** ([`Adc::calibrate`]). This is the F130
//!   hang-if-done-wrong sequence TESTING.md flags; both polls are bounded ([`ADC_TIMEOUT`]).
//!
//! # Read (T11, the polled single conversion)
//!
//! [`Adc::read_channel`] (and the trigger/read split [`Adc::software_trigger`] / [`Adc::read_data`])
//! mirrors the SPL polled example: `adc_software_trigger_enable(ADC_REGULAR_CHANNEL)` sets CTL1
//! SWRCST, then poll `adc_flag_get(ADC_FLAG_EOC)` (STAT EOC) until set, then `adc_regular_data_read`
//! (read RDATA). The EOC poll is bounded ([`ADC_TIMEOUT`]); exhaustion is [`AdcError::Timeout`].
//!
//! # Read API (ADC-1)
//!
//! `embedded-hal` 1.0 dropped the ADC trait (there is none in 1.0). The read surface is therefore a
//! plain runtime-hal method: [`Adc::read_channel`] (re-point rank 0 to `channel`, software-trigger,
//! poll EOC, read RDATA) returning `Result<u16, AdcError>`. **No `embedded-hal` `nb` 0.2
//! `adc::OneShot` shim is provided**: that trait lives in `embedded-hal` 0.2, and pulling the whole
//! 0.2 crate in (alongside the 1.0 dep this crate already carries for I2C/SPI) only to gain a
//! deprecated one-shot trait is not worth the dependency surface for M2, whose ADC validation
//! target is the internal VREFINT/temperature anchor (T13), not off-the-shelf-ADC-driver
//! compatibility (which `embedded-hal` 1.0 itself no longer expresses). If a future caller needs
//! `OneShot`, it is a thin wrapper over `read_channel` and can be added then.

use crate::error::AdcError;
use crate::reg::Reg32;

// --- register offsets (identical on both families) --------------------------------------------

const STAT: u32 = 0x00;
const CTL0: u32 = 0x04;
const CTL1: u32 = 0x08;
const SAMPT0: u32 = 0x0C;
const SAMPT1: u32 = 0x10;
const RSQ0: u32 = 0x2C;
const RSQ1: u32 = 0x30;
const RSQ2: u32 = 0x34;
const RDATA: u32 = 0x4C;

// --- injected-group registers (M3 T8; identical on both families: gd32f1x0_adc.h / gd32f10x_adc.h)
//
// ISQ 0x38 (inserted sequence: IL[21:20] length, ISQN 5-bit channel fields), IDATA0..3 0x3C/0x40/
// 0x44/0x48 (inserted data). The inserted-channel external-trigger select (ETSIC) lives in CTL1
// [14:12] and its enable (ETEIC) in CTL1 bit 15; the inserted end-of-conversion (EOIC) flag is STAT
// bit 2 and its interrupt enable (EOICIE) is CTL0 bit 7.

/// ADC inserted sequence register (`ADC_ISQ`), offset 0x38.
const ISQ: u32 = 0x38;
/// ADC inserted data registers `ADC_IDATA0..3`, offsets 0x3C/0x40/0x44/0x48.
const IDATA: [u32; 4] = [0x3C, 0x40, 0x44, 0x48];

// STAT flags.
const STAT_EOC: u32 = 1 << 1;
/// Inserted (injected) end-of-conversion flag (`ADC_STAT_EOIC`, bit 2).
const STAT_EOIC: u32 = 1 << 2;

// CTL0 bits.
/// Scan mode (`ADC_CTL0_SM`); left clear for single conversion.
const CTL0_SM: u32 = 1 << 8;

// CTL1 bits.
const CTL1_ADCON: u32 = 1 << 0;
/// Continuous conversion (`ADC_CTL1_CTN`); left clear for single conversion.
const CTL1_CTN: u32 = 1 << 1;
const CTL1_CLB: u32 = 1 << 2;
const CTL1_RSTCLB: u32 = 1 << 3;
/// Data alignment (`ADC_CTL1_DAL`); clear = right-aligned (the M2 default).
const CTL1_DAL: u32 = 1 << 11;
/// External trigger select for the regular channel (`ADC_CTL1_ETSRC`, bits[19:17]).
const CTL1_ETSRC: u32 = 0x7 << 17;
/// External trigger enable for the regular channel (`ADC_CTL1_ETERC`).
const CTL1_ETERC: u32 = 1 << 20;
/// Start-on-regular-channel software trigger (`ADC_CTL1_SWRCST`).
const CTL1_SWRCST: u32 = 1 << 22;
/// Temperature-sensor + VREFINT enable (`ADC_CTL1_TSVREN`, channels 16 and 17).
const CTL1_TSVREN: u32 = 1 << 23;
/// External trigger select for the INSERTED (injected) channel group (`ADC_CTL1_ETSIC`, bits[14:12]).
const CTL1_ETSIC: u32 = 0x7 << 12;
/// External trigger enable for the INSERTED (injected) channel group (`ADC_CTL1_ETEIC`, bit 15).
const CTL1_ETEIC: u32 = 1 << 15;

// CTL0 bits (injected-group interrupt enable).
/// Interrupt enable for the INSERTED (injected) channel group, end-of-conversion (`ADC_CTL0_EOICIE`,
/// bit 7): the EOIC interrupt that runs the control loop at the PWM rate.
const CTL0_EOICIE: u32 = 1 << 7;

/// The injected external-trigger source codes for TIMER0 (ETSIC field value), identical on both
/// families (`ADC_EXTTRIG_INSERTED_T0_TRGO` = 0, `ADC_EXTTRIG_INSERTED_T0_CH3` = 1). Stored as the
/// raw 3-bit field value; shifted into `CTL1_ETSIC` by [`Adc::configure_injected`].
pub const ETSIC_T0_TRGO: u32 = 0;
/// TIMER0 CH3 event select (`ADC_EXTTRIG_INSERTED_T0_CH3`).
pub const ETSIC_T0_CH3: u32 = 1;

/// The ISQ inserted-sequence-length field (`ADC_ISQ_IL`, bits[21:20]), stored as `length - 1`.
const ISQ_IL: u32 = 0x3 << 20;
/// The 5-bit per-rank channel field width in the ISQ register (`ADC_ISQ_ISQN`).
const ISQ_FIELD: u32 = 0x1F;

/// `ADC_EXTTRIG_REGULAR_NONE` = `CTL1_ETSRC(7)`: the software-trigger source select (both
/// families: the `0b111` ETSRC code selects no hardware trigger, so the conversion starts only on
/// the SWRCST software trigger).
const ETSRC_SOFTWARE: u32 = 0x7 << 17;

/// RSQ0 RL field (`ADC_RSQ0_RL`, bits[23:20]): regular sequence length, stored as `len - 1`.
const RSQ0_RL: u32 = 0xF << 20;

/// The 5-bit per-rank channel field width in the RSQ registers (`ADC_RSQX_RSQN`).
const RSQ_FIELD: u32 = 0x1F;
/// The 3-bit per-channel sample-time field width in the SAMPT registers (`ADC_SAMPTX_SPTN`).
const SAMPT_FIELD: u32 = 0x7;

/// Bounded poll budget for the calibration-done bits (RSTCLB / CLB) and the conversion EOC flag.
/// Counts loop iterations, not cycles, so it is clock-independent; generous enough never to
/// false-time a working calibration / conversion at any representative ADC clock, but always
/// escaping a stuck bit (the F130 hang-if-done-wrong class, which TESTING.md flags specifically for
/// the calibration sequence). Mirrors [`crate::i2c::I2C_TIMEOUT`] / [`crate::spi::SPI_TIMEOUT`].
pub const ADC_TIMEOUT: u32 = 100_000;

/// True for an internal channel (16 = temperature, 17 = VREFINT): the two channels the `TSVREN`
/// bit powers (open item ADC-3).
#[inline]
pub const fn is_internal_channel(channel: u8) -> bool {
    channel == 16 || channel == 17
}

// --- the handle -------------------------------------------------------------------------------

/// A configured ADC, resolved once at bring-up: just the base (the register model is shared, so
/// there is no per-family field). The single-conversion read primitives hang off this
/// (DECISIONS.md #4: resolve once into a concrete handle).
#[derive(Debug, Clone, Copy)]
pub struct Adc {
    base: u32,
}

impl Adc {
    /// A bare handle over the ADC at `base`, performing **no** register access. Use this to talk to
    /// an ADC that is already brought up (e.g. to re-read a channel, or to drive the calibration /
    /// read primitives directly). [`Adc::bring_up`] / [`Adc::configure_single`] are the configuring
    /// entry points.
    #[inline]
    pub const fn at(base: u32) -> Adc {
        Adc { base }
    }

    /// Bring up ADC0 at `base` for a single software-triggered regular conversion of `channel`,
    /// at `sample_time` (the `ADC_SAMPLETIME_*` field code 0..=7), reproducing the SPL
    /// single-conversion recipe (see the module docs), including the calibration step.
    ///
    /// `channel` becomes rank 0 of a length-1 regular sequence; the read API
    /// ([`Adc::read_channel`]) can re-point rank 0 to another channel later without re-running
    /// bring-up. If `channel` is an internal channel (16 = temperature, 17 = VREFINT) the `TSVREN`
    /// enable bit is set (open item ADC-3). Returns [`AdcError::Timeout`] if calibration does not
    /// complete within [`ADC_TIMEOUT`].
    pub fn bring_up(base: u32, channel: u8, sample_time: u8) -> Result<Adc, AdcError> {
        let dev = Adc::configure_single(base, channel, sample_time);
        dev.calibrate()?;
        Ok(dev)
    }

    /// Program the single-conversion configuration (everything BEFORE calibration), in the order
    /// the SPL polled example does it, and return the handle. Pure writes / RMWs; **no polling**
    /// here, so this is what the config golden traces (the calibration poll is a separate
    /// `with_polling` segment, exactly as the SPL config golden excludes `adc_calibration_enable`).
    /// [`Adc::bring_up`] is `configure_single` then [`Adc::calibrate`].
    pub fn configure_single(base: u32, channel: u8, sample_time: u8) -> Adc {
        let dev = Adc { base };
        dev.configure(channel, sample_time);
        dev
    }

    /// The CTL0/CTL1/RSQ/SAMPT field programming (everything before calibration), in SPL order.
    fn configure(&self, channel: u8, sample_time: u8) {
        // adc_special_function_config(SCAN_MODE, DISABLE): clear CTL0 SM (single, not scan).
        self.ctl0().modify(CTL0_SM, 0);
        // adc_special_function_config(CONTINUOUS_MODE, DISABLE): clear CTL1 CTN (single, not continuous).
        self.ctl1().modify(CTL1_CTN, 0);
        // adc_data_alignment_config(RIGHT): clear CTL1 DAL (right-aligned, 12-bit).
        self.ctl1().modify(CTL1_DAL, 0);
        // adc_channel_length_config(REGULAR, 1): RSQ0 RL = len-1 = 0.
        self.set_regular_length(1);
        // adc_regular_channel_config(rank 0, channel, sample_time): the rank's channel + the
        // channel's sample time.
        self.set_regular_rank(0, channel);
        self.set_sample_time(channel, sample_time);
        // adc_tempsensor_vrefint_enable: set TSVREN when reading an internal channel (16/17).
        if is_internal_channel(channel) {
            self.ctl1().modify(CTL1_TSVREN, CTL1_TSVREN);
        }
        // adc_external_trigger_source_config(REGULAR, NONE): ETSRC = software-trigger code (7).
        self.ctl1().modify(CTL1_ETSRC, ETSRC_SOFTWARE);
        // adc_external_trigger_config(REGULAR, ENABLE): set ETERC (the block needs it set even for
        // the software trigger; the SPL examples do this).
        self.ctl1().modify(CTL1_ETERC, CTL1_ETERC);
        // adc_enable: set ADCON.
        self.ctl1().modify(CTL1_ADCON, CTL1_ADCON);
    }

    /// `adc_calibration_enable`: set CTL1 RSTCLB and spin until it clears, then set CTL1 CLB and
    /// spin until it clears. Both polls are bounded ([`ADC_TIMEOUT`]); exhaustion (a calibration
    /// bit that never clears, the hang-if-done-wrong class) is [`AdcError::Timeout`] instead of an
    /// infinite spin.
    pub fn calibrate(&self) -> Result<(), AdcError> {
        // RSTCLB: reset the calibration register, then wait for the bit to self-clear.
        self.ctl1().modify(CTL1_RSTCLB, CTL1_RSTCLB);
        self.wait_clear(CTL1_RSTCLB)?;
        // CLB: run calibration, then wait for the bit to self-clear (calibration done).
        self.ctl1().modify(CTL1_CLB, CTL1_CLB);
        self.wait_clear(CTL1_CLB)
    }

    /// `adc_software_trigger_enable(ADC_REGULAR_CHANNEL)`: set CTL1 SWRCST to start a regular
    /// conversion.
    #[inline]
    pub fn software_trigger(&self) {
        self.ctl1().modify(CTL1_SWRCST, CTL1_SWRCST);
    }

    // --- injected (inserted) conversion group (M3 T8) -----------------------------------------

    /// Program the timer-triggered INJECTED (inserted) conversion group, in the order the SPL
    /// injected-trigger recipe does it, and return the handle. Pure writes / RMWs; **no polling**
    /// (the calibration poll is the separate [`Adc::calibrate`] / with_polling segment), so this is
    /// what the injected config golden traces.
    ///
    /// `channels` is the injected channel list in injected-rank order (`(channel, sample_time)`),
    /// `left_aligned` selects left-aligned data (the M3 default), and `etsic` is the injected
    /// external-trigger-source field value ([`ETSIC_T0_CH3`] / [`ETSIC_T0_TRGO`], derived from the
    /// descriptor's timer-trigger link). Reproduces the SPL:
    ///
    /// - `adc_data_alignment_config(LEFT)`: set CTL1 DAL (or clear for right-aligned).
    /// - `adc_inserted_channel_length_config(len)`: ISQ IL = `len - 1`.
    /// - per rank `adc_inserted_channel_config(rank, channel, sample_time)`: the rank's 5-bit ISQ
    ///   field (SPL's reversed packing) + the channel's 3-bit SAMPT field.
    /// - `adc_tempsensor_vrefint_enable`: set CTL1 TSVREN if an internal channel is in the list.
    /// - `adc_external_trigger_source_config(INSERTED, src)`: CTL1 ETSIC = `etsic`.
    /// - `adc_external_trigger_config(INSERTED, ENABLE)`: set CTL1 ETEIC.
    /// - `adc_interrupt_enable(ADC_INT_EOIC)`: set CTL0 EOICIE (the EOIC interrupt the control loop
    ///   runs in).
    /// - `adc_enable`: set CTL1 ADCON.
    ///
    /// The CH3/TRGO timer side is [`crate::timer`] (T6); calibration ([`Adc::calibrate`]) follows.
    pub fn configure_injected(
        base: u32,
        channels: &[(u8, u8)],
        left_aligned: bool,
        etsic: u32,
    ) -> Adc {
        let dev = Adc { base };
        // adc_data_alignment_config: DAL set = left-aligned, clear = right.
        dev.ctl1()
            .modify(CTL1_DAL, if left_aligned { CTL1_DAL } else { 0 });
        // adc_inserted_channel_length_config(len): ISQ IL = len - 1.
        let len = channels.len().max(1) as u32;
        dev.set_injected_length(len as u8);
        // adc_inserted_channel_config(rank, channel, sample_time) per rank.
        let mut internal = false;
        for (rank, &(channel, sample_time)) in channels.iter().enumerate() {
            dev.set_injected_rank(rank as u8, channel, len as u8);
            dev.set_sample_time(channel, sample_time);
            internal |= is_internal_channel(channel);
        }
        if internal {
            dev.ctl1().modify(CTL1_TSVREN, CTL1_TSVREN);
        }
        // adc_external_trigger_source_config(INSERTED, src): ETSIC = the timer trigger code.
        dev.ctl1().modify(CTL1_ETSIC, (etsic << 12) & CTL1_ETSIC);
        // adc_external_trigger_config(INSERTED, ENABLE): set ETEIC.
        dev.ctl1().modify(CTL1_ETEIC, CTL1_ETEIC);
        // adc_interrupt_enable(ADC_INT_EOIC): set CTL0 EOICIE (the PWM-rate control-loop interrupt).
        dev.ctl0().modify(CTL0_EOICIE, CTL0_EOICIE);
        // adc_enable: set ADCON.
        dev.ctl1().modify(CTL1_ADCON, CTL1_ADCON);
        dev
    }

    /// Bring up the injected group: [`Adc::configure_injected`] then the calibration step
    /// ([`Adc::calibrate`], the RSTCLB/CLB hang-if-done-wrong sequence M2 brought up). Returns
    /// [`AdcError::Timeout`] if calibration does not complete within [`ADC_TIMEOUT`].
    pub fn bring_up_injected(
        base: u32,
        channels: &[(u8, u8)],
        left_aligned: bool,
        etsic: u32,
    ) -> Result<Adc, AdcError> {
        let dev = Adc::configure_injected(base, channels, left_aligned, etsic);
        dev.calibrate()?;
        Ok(dev)
    }

    /// Poll the injected end-of-conversion flag (STAT EOIC), bounded by [`ADC_TIMEOUT`]. Exhaustion
    /// (EOIC never sets) is [`AdcError::Timeout`]. Used by a triggered read to wait for the injected
    /// group to finish before reading IDATAx.
    pub fn wait_eoic(&self) -> Result<(), AdcError> {
        let mut budget = ADC_TIMEOUT;
        while self.stat().read() & STAT_EOIC == 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }

    /// `adc_inserted_data_read(ADC_INSERTED_CHANNEL_n)`: read injected data register IDATAn (the raw,
    /// left-aligned-as-configured conversion result). `n` is the injected channel index 0..3.
    #[inline]
    pub fn read_injected_data(&self, n: u8) -> u16 {
        let off = IDATA[(n & 0x3) as usize];
        (self.reg(off).read() & 0xFFFF) as u16
    }

    /// Poll STAT EOC (the `adc_flag_get(ADC_FLAG_EOC)` the SPL does), then `adc_regular_data_read`
    /// (read RDATA). The EOC poll is bounded ([`ADC_TIMEOUT`]); exhaustion is [`AdcError::Timeout`].
    /// The result is the right-aligned 12-bit conversion value.
    pub fn read_data(&self) -> Result<u16, AdcError> {
        self.wait_eoc()?;
        Ok((self.rdata().read() & 0xFFFF) as u16)
    }

    /// Single software-triggered conversion of `channel`: re-point regular rank 0 to `channel`
    /// (so the read API can move between channels without re-running bring-up), software-trigger,
    /// poll EOC, and read RDATA. The runtime-hal read surface (open item ADC-1); returns the
    /// right-aligned 12-bit value or [`AdcError::Timeout`].
    ///
    /// NOTE: this assumes the channel's sample time was set at bring-up (or by a prior call for the
    /// same channel). The M2 anchor reads one fixed channel (VREFINT/temperature), so rank 0 +
    /// sample time set once at bring-up is the common path; re-pointing rank 0 keeps a multi-channel
    /// caller from re-running the whole bring-up per channel.
    pub fn read_channel(&self, channel: u8) -> Result<u16, AdcError> {
        self.set_regular_rank(0, channel);
        self.software_trigger();
        self.read_data()
    }

    /// The underlying base address (for code that needs the register-level view).
    #[inline]
    pub const fn base(&self) -> u32 {
        self.base
    }

    // --- sequence / sample-time field programming (mirrors adc_*_config) ----------------------

    /// `adc_channel_length_config(ADC_REGULAR_CHANNEL, len)`: RSQ0 RL = `len - 1` (clear-then-set
    /// the 4-bit RL field, the SPL's `RSQ0 &= ~RL; RSQ0 |= RSQ0_RL(len-1)`).
    fn set_regular_length(&self, len: u8) {
        let rl = ((len.max(1) as u32 - 1) << 20) & RSQ0_RL;
        self.reg(RSQ0).modify(RSQ0_RL, rl);
    }

    /// `adc_regular_channel_config` rank half: set rank `rank`'s 5-bit channel field in the right
    /// RSQ register (ranks 0..5 -> RSQ2, 6..11 -> RSQ1, 12..15 -> RSQ0), clear-then-set the field,
    /// exactly as the SPL does.
    fn set_regular_rank(&self, rank: u8, channel: u8) {
        let (off, shift) = match rank {
            0..=5 => (RSQ2, 5 * rank as u32),
            6..=11 => (RSQ1, 5 * (rank as u32 - 6)),
            12..=15 => (RSQ0, 5 * (rank as u32 - 12)),
            _ => return,
        };
        let mask = RSQ_FIELD << shift;
        let val = ((channel as u32) & RSQ_FIELD) << shift;
        self.reg(off).modify(mask, val);
    }

    /// `adc_inserted_channel_length_config(len)`: ISQ IL field = `len - 1` (clear-then-set the
    /// 2-bit IL field, the SPL's `ISQ &= ~IL; ISQ |= ISQ_IL(len-1)`).
    fn set_injected_length(&self, len: u8) {
        let il = ((len.max(1) as u32 - 1) << 20) & ISQ_IL;
        self.reg(ISQ).modify(ISQ_IL, il);
    }

    /// `adc_inserted_channel_config` rank half: set injected rank `rank`'s 5-bit channel field in
    /// ISQ, using the SPL's reversed packing. The SPL computes the field shift from the configured
    /// inserted length L (IL = L-1): `shift = 15 - ((L-1) - rank) * 5`, so a length-2 group packs
    /// rank 0 at bits[14:10] and rank 1 at bits[19:15]. `len` is the configured group length L.
    fn set_injected_rank(&self, rank: u8, channel: u8, len: u8) {
        let il = (len.max(1) as u32) - 1; // the SPL's `inserted_length` (the IL field value).
        let shift = 15u32.wrapping_sub((il - rank as u32) * 5);
        let mask = ISQ_FIELD << shift;
        let val = ((channel as u32) & ISQ_FIELD) << shift;
        self.reg(ISQ).modify(mask, val);
    }

    /// `adc_regular_channel_config` sample-time half: set channel `channel`'s 3-bit sample-time
    /// field in the right SAMPT register (channels 0..9 -> SAMPT1, 10..17 -> SAMPT0),
    /// clear-then-set the field, exactly as the SPL does.
    fn set_sample_time(&self, channel: u8, sample_time: u8) {
        let (off, shift) = if channel < 10 {
            (SAMPT1, 3 * channel as u32)
        } else if channel < 18 {
            (SAMPT0, 3 * (channel as u32 - 10))
        } else {
            return;
        };
        let mask = SAMPT_FIELD << shift;
        let val = ((sample_time as u32) & SAMPT_FIELD) << shift;
        self.reg(off).modify(mask, val);
    }

    // --- polling primitives -------------------------------------------------------------------

    /// Spin until the CTL1 bit(s) in `mask` clear (the calibration self-clearing-bit wait), bounded
    /// by [`ADC_TIMEOUT`]. Exhaustion (the bit never clears) is [`AdcError::Timeout`].
    fn wait_clear(&self, mask: u32) -> Result<(), AdcError> {
        let mut budget = ADC_TIMEOUT;
        while self.ctl1().read() & mask != 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }

    /// Spin until STAT EOC is set (the conversion-done wait), bounded by [`ADC_TIMEOUT`].
    /// Exhaustion (EOC never sets) is [`AdcError::Timeout`].
    fn wait_eoc(&self) -> Result<(), AdcError> {
        let mut budget = ADC_TIMEOUT;
        while self.stat().read() & STAT_EOC == 0 {
            budget -= 1;
            if budget == 0 {
                return Err(AdcError::Timeout);
            }
        }
        Ok(())
    }

    // --- register accessors -------------------------------------------------------------------

    #[inline]
    fn reg(&self, off: u32) -> Reg32 {
        Reg32::new(self.base, off)
    }
    #[inline]
    fn stat(&self) -> Reg32 {
        self.reg(STAT)
    }
    #[inline]
    fn ctl0(&self) -> Reg32 {
        self.reg(CTL0)
    }
    #[inline]
    fn ctl1(&self) -> Reg32 {
        self.reg(CTL1)
    }
    #[inline]
    fn rdata(&self) -> Reg32 {
        self.reg(RDATA)
    }
}

/// What ADC sampling this chip can do, as a capability "fruit": the caller matches on the silicon
/// SHAPE, never on the MCU family. Returned by [`crate::Chip::adc`].
///
/// - [`AdcCapability::Single`]: one ADC (the F1x0 baseline). Sample channels in sequence.
/// - [`AdcCapability::Dual`]: two ADCs (the F10x dual-ADC parts), handed back as `primary` +
///   `secondary`. The pair can be driven for at-the-same-instant phase-current sampling; this fruit
///   hands back BOTH handles so each is usable today. (The hardware regular-simultaneous trigger
///   coupling is a future bring-up; the two handles are independently usable now.)
///
/// Because a caller `match`es this exhaustively, firmware that uses the ADC handles BOTH shapes and so
/// runs on either family by construction, with no family test.
#[derive(Debug, Clone, Copy)]
pub enum AdcCapability {
    /// A single ADC.
    Single(Adc),
    /// Two ADCs (F10x dual-ADC parts): the primary (ADC0) and secondary (ADC1).
    Dual {
        /// The primary ADC (ADC0).
        primary: Adc,
        /// The secondary ADC (ADC1).
        secondary: Adc,
    },
}

#[cfg(test)]
mod tests;
