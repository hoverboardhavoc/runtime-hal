//! On-silicon cold-path bring-up validator for the GD32F130 bench board.
//!
//! It brings up the cold-path peripherals (the 72 MHz IRC8M->PLL clock tree, I2C0, ADC0, SPI0)
//! **entirely through runtime-hal**, from the runtime-DETECTED chip, then publishes each result into
//! a fixed RAM struct a human reads back over SWD. It does NOT depend on the `mock` feature: runtime-hal
//! is pulled in its normal real-volatile-MMIO `no_std` build (`default-features = false` in Cargo.toml).
//!
//! End-to-end path (the faithful chain, exercised on purpose):
//!   `detect_chip()` (family probe + peripheral measurement) -> `Chip` -> construct *Config ->
//!   bring-up.
//!
//! The clock tree (`ClockConfig`) and every peripheral config (`I2cConfig`, `SpiConfig`, the ADC
//! channel list as an `AdcConfig`) are constructed IN CODE here. The chip identity is DISCOVERED at
//! runtime (the same family probe the other bench firmware uses), so the whole chain (the runtime
//! detection, the synthesized descriptor, the `Chip` context, the bring-up) runs on silicon.
//!
//! Anchors (each writes its result, then the magic word is written LAST so a debugger that sees the
//! magic knows every anchor ran):
//!   - **Clock**: `clock::configure_tree` brings up the 72 MHz tree (IRC8M source, no crystal).
//!   - **I2C / IMU**: I2C0, SCL=PB6/SDA=PB7, AF1, 100 kHz; read IMU (0x68) registers 0x75
//!     (WHO_AM_I, expect 0x2E clone), 0x06 (expect 0x19), 0x07 (expect 0xF4) via `write_read`.
//!   - **ADC**: ADC0, internal VREFINT (ch 17) and temperature (ch 16) raw 12-bit reads (TSVREN is
//!     set by the driver for internal channels).
//!   - **SPI**: SPI0 loopback transfer of a 4-byte pattern; the readback + a matched flag (true only
//!     when MOSI/MISO are physically jumpered, which the human does).
//!
//! Every poll inside runtime-hal is already bounded, and the firmware's own error paths set an err
//! byte and move on, so a missing device cannot hang it (the F130 hang-if-done-wrong class). No
//! motor, no PWM, no buzzer.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal::i2c::I2c as _;
use embedded_hal::spi::SpiBus as _;

use heapless::Vec;

use runtime_hal::{
    clock,
    clock::ClockConfig,
    adc::AdcCapability,
    config::{AdcChannel, AdcClockDiv, AdcConfig, NssMode, SpiConfig},
    detect_chip,
    gpio::PinRole,
    i2c::{I2c, I2cMode},
    spi::Spi,
    Chip, PeriphLabel,
};

// --- hardware facts (the bench F130 board) ----------------------------------------------------

/// The on-board IMU 7-bit address.
const IMU_ADDR: u8 = 0x68;
/// IMU WHO_AM_I register (expect 0x2E on this clone).
const IMU_REG_WHO_AM_I: u8 = 0x75;
/// IMU register 0x06 (expect 0x19).
const IMU_REG_06: u8 = 0x06;
/// IMU register 0x07 (expect 0xF4).
const IMU_REG_07: u8 = 0x07;

/// The SPI loopback test pattern; readback matches it only when MOSI<->MISO are jumpered.
const SPI_PATTERN: [u8; 4] = [0xA5, 0x3C, 0x00, 0xFF];

// --- the SWD-readable result struct -----------------------------------------------------------

/// The fixed-layout anchor results. A debugger reads it back over SWD at the fixed [`RESULT_ADDR`]:
/// `magic` is written LAST, so a reader that sees `MAGIC` knows every anchor ran. The struct lives at
/// a reserved tail of RAM (see `memory.x`), so the reader needs no `arm-none-eabi-nm` symbol
/// resolution (pinning a section at RAM ORIGIN instead collided with cortex-m-rt's RAM allocation).
///
/// `#[repr(C)]` fixes the field order and offsets so the SWD reader can index by byte offset.
#[repr(C)]
struct M2Anchors {
    /// 0x4D32_0A2C, written LAST = the full run completed.
    magic: u32,
    /// `configure_tree` ran to completion (it returns no error; this flags that it did not hang).
    clock_ok: u8,
    /// IMU WHO_AM_I (reg 0x75); expect 0x2E.
    i2c_who_am_i: u8,
    /// IMU reg 0x06; expect 0x19.
    i2c_reg06: u8,
    /// IMU reg 0x07; expect 0xF4.
    i2c_reg07: u8,
    /// Non-zero if any I2C read errored (missing/stuck device).
    i2c_err: u8,
    /// VREFINT (channel 17) raw 12-bit reading.
    adc_vrefint: u16,
    /// Temperature sensor (channel 16) raw 12-bit reading.
    adc_temp: u16,
    /// Non-zero if any ADC bring-up/read errored.
    adc_err: u8,
    /// The 4 bytes read back during the SPI loopback transfer.
    spi_readback: [u8; 4],
    /// 1 if `spi_readback == SPI_PATTERN` (true only when jumpered).
    spi_matched: u8,
    /// Non-zero if the SPI transfer errored.
    spi_err: u8,
}

/// The magic value written last once every anchor has run.
const MAGIC: u32 = 0x4D32_0A2C;

/// Fixed RAM address of the result struct: the top of the (shrunk) RAM region, reserved by `memory.x`
/// (cortex-m-rt's RAM ends 256 bytes early so it never allocates here). The SWD reader reads this
/// CONSTANT directly, no `arm-none-eabi-nm` resolution needed (the size-optimised release ELF drops
/// the `.symtab` nm reads, so a symbol-based read is unreliable; a fixed address is not).
const RESULT_ADDR: u32 = 0x2000_1F00;

/// Initial result contents, written to [`RESULT_ADDR`] at startup. The region is OUTSIDE `.bss` (above
/// cortex-m-rt's RAM), so the C runtime does NOT zero it; `main` writes this first so a reader sees
/// `magic == 0` until the run completes and writes `magic` LAST.
const INIT_ANCHORS: M2Anchors = M2Anchors {
    magic: 0,
    clock_ok: 0,
    i2c_who_am_i: 0,
    i2c_reg06: 0,
    i2c_reg07: 0,
    i2c_err: 0,
    adc_vrefint: 0,
    adc_temp: 0,
    adc_err: 0,
    spi_readback: [0; 4],
    spi_matched: 0,
    spi_err: 0,
};

// --- entry ------------------------------------------------------------------------------------

/// The code-level 72 MHz clock tree (DR-T5): the reference IRC8M->PLL arrangement (sysclk 72 MHz,
/// AHB/APB2 /1, APB1 /2 = 36 MHz, 2 flash wait states). Constructed in code, not decoded from the
/// chip-only blob. This is the same tree the old `clock_cfg` carried.
const M2_CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;

#[entry]
fn main() -> ! {
    // Initialise the fixed-address result region (outside .bss, so not zeroed by the C runtime):
    // write the defaults first, magic = 0 until the run completes.
    // SAFETY: RESULT_ADDR is reserved RAM (see memory.x); single writer.
    unsafe { core::ptr::write_volatile(anchors_ptr(), INIT_ANCHORS) };

    // Detect the chip at runtime (family probe + peripheral measurement -> Chip), the full faithful
    // chain. A part matching neither family halts rather than misconfiguring; the human then sees
    // magic == 0 (the run never reached completion).
    let chip: Chip = match detect_chip() {
        Ok(c) => c,
        Err(_) => halt(),
    };

    run_anchors(&chip);

    // Done: everything has been written; write the magic LAST, then idle.
    write_magic();
    loop {
        cortex_m::asm::wfi();
    }
}

/// Run all four anchors against the chip context, writing each result into `M2_ANCHORS`.
fn run_anchors(chip: &Chip) {
    // 1. CLOCK: bring up the 72 MHz IRC8M->PLL tree from the code-level ClockConfig.
    //    `configure_tree` validates the config against the chip-bound ranges then polls internally;
    //    reaching the next line means the source stabilised, the PLL locked, and the system clock
    //    switched (on dead silicon it would hang there, leaving magic == 0). The RCU base + clock
    //    path come from the chip; the tree comes from code.
    let rcu = match chip.rcu_base() {
        Ok(b) => b,
        Err(_) => halt(),
    };
    if clock::configure_tree(chip, &M2_CLOCK).is_err() {
        // An invalid clock config would be a firmware bug; leave clock_ok = 0 and skip the rest.
        return;
    }
    set_clock_ok();

    // 2. I2C / IMU, 3. ADC, 4. SPI. Each is self-contained and sets its own err byte on failure.
    run_i2c(chip, rcu);
    run_adc(chip, rcu);
    run_spi(chip, rcu);
}

// --- I2C anchor -------------------------------------------------------------------------------

/// Bring up I2C0 through runtime-hal and read the three IMU registers via `embedded-hal`
/// `write_read` (write the register pointer, read one byte back). On any failure the i2c_err byte
/// is set and the routine returns (no hang: runtime-hal's polls are all bounded).
fn run_i2c(chip: &Chip, _rcu: u32) {
    // I2C0 on SCL = PB6, SDA = PB7, 100 kHz standard mode. The pins come from `split()` as type-state
    // handles; `I2c::new` CONSUMES them, configures them AF open-drain with pull-up (AF1 on F1x0 for
    // I2C0 on port B), enables the I2C peripheral clock, and brings up the timing. `gpiob()` enabled
    // the GPIOB port clock when it handed back the port. No packed `(port << 4) | pin` byte anywhere.
    let gpiob = match chip.gpiob() {
        Ok(p) => p.split(),
        Err(_) => {
            set_i2c_err(4);
            return;
        }
    };
    let mut i2c = match I2c::new(
        chip,
        &M2_CLOCK,
        PeriphLabel::I2c0,
        (gpiob.pb6, gpiob.pb7),
        I2cMode::standard(100_000),
    ) {
        Ok(dev) => dev,
        Err(_) => {
            set_i2c_err(2);
            return;
        }
    };

    // Read the three registers. The first failure sets the err byte; later reads still run so a
    // partial result is captured, but the err flag tells the human a read did not complete.
    let mut err = 0u8;
    match read_imu_reg(&mut i2c, IMU_REG_WHO_AM_I) {
        Ok(v) => set_i2c_who_am_i(v),
        Err(()) => err |= 0x10,
    }
    match read_imu_reg(&mut i2c, IMU_REG_06) {
        Ok(v) => set_i2c_reg06(v),
        Err(()) => err |= 0x20,
    }
    match read_imu_reg(&mut i2c, IMU_REG_07) {
        Ok(v) => set_i2c_reg07(v),
        Err(()) => err |= 0x40,
    }
    if err != 0 {
        set_i2c_err(err);
    }
}

/// Read one IMU register: write the register pointer, repeated-START, read one byte (the
/// `embedded-hal` `write_read` the WHO_AM_I sequence needs).
fn read_imu_reg(i2c: &mut I2c, reg: u8) -> Result<u8, ()> {
    let mut buf = [0u8; 1];
    match i2c.write_read(IMU_ADDR, &[reg], &mut buf) {
        Ok(()) => Ok(buf[0]),
        Err(_) => Err(()),
    }
}

// --- ADC anchor -------------------------------------------------------------------------------

/// Bring up ADC0 through runtime-hal and read the two internal channels (VREFINT = 17,
/// temperature = 16) raw. The ADC clock prescaler/source is set by `clock::enable_adc`; the
/// driver sets TSVREN for the internal channels. On failure the adc_err byte is set.
fn run_adc(chip: &Chip, rcu: u32) {
    // The ADC config, constructed IN CODE (DR-T5): ADC0, regular sequence reads VREFINT (ch 17)
    // then temperature (ch 16), both at the slowest sample time (code 7 = 239.5 cycles) so they
    // settle; right-aligned; APB2/6 prescaler. The driver sets TSVREN for the internal channels.
    let mut channels: Vec<AdcChannel, { runtime_hal::MAX_ADC_CHANNELS }> = Vec::new();
    let _ = channels.push(AdcChannel {
        channel: 17,
        sample_time: 7,
    });
    let _ = channels.push(AdcChannel {
        channel: 16,
        sample_time: 7,
    });
    let cfg = AdcConfig {
        adc: PeriphLabel::Adc0,
        channels,
        left_aligned: false,
        clock_div: AdcClockDiv::Div6,
    };

    // ADC peripheral clock + prescaler (chip's clock path).
    if clock::enable_adc(rcu, chip.clock(), cfg.adc).is_err() {
        set_adc_err(3);
        return;
    }

    // Resolve the ADC handle from the chip (base hidden in the descriptor). This part is single-ADC
    // on the bench; a dual part would still expose ADC0 as the primary of the pair.
    let adc = match chip.adc() {
        Ok(AdcCapability::Single(adc)) => adc,
        Ok(AdcCapability::Dual(dual)) => dual.primary(),
        Err(_) => {
            set_adc_err(2);
            return;
        }
    };

    // Bring up ADC0 on rank 0 = the first sequence channel (VREFINT, sample time from the config).
    // bring_up runs the calibration sequence (bounded polls inside). The AdcConfig is the code-level
    // source those values come from.
    let first = match cfg.channels.first() {
        Some(c) => *c,
        None => {
            set_adc_err(4);
            return;
        }
    };
    if adc.bring_up(first.channel, first.sample_time).is_err() {
        set_adc_err(5);
        return;
    }

    // Make sure each internal channel has its sample time programmed, then read it. The sequence
    // carries channel 17 (VREFINT) then 16 (temperature), both at sample-time code 7.
    let mut err = 0u8;
    for c in cfg.channels.iter() {
        // configure_single re-programs the sample-time field for the channel (no calibration), so a
        // channel other than the bring-up one still has a valid sample time before we read it.
        adc.configure_single(c.channel, c.sample_time);
        match adc.read_channel(c.channel) {
            Ok(raw) => {
                if c.channel == 17 {
                    set_adc_vrefint(raw);
                } else if c.channel == 16 {
                    set_adc_temp(raw);
                }
            }
            Err(_) => err |= 0x10,
        }
    }
    if err != 0 {
        set_adc_err(err);
    }
}

// --- SPI anchor -------------------------------------------------------------------------------

/// Bring up SPI0 through runtime-hal and do a loopback transfer of the 4-byte pattern. The
/// readback is stored regardless; the matched flag is set only if it equals the sent pattern (true
/// only when MOSI/MISO are physically jumpered). On a transfer error the spi_err byte is set.
fn run_spi(chip: &Chip, rcu: u32) {
    // The SPI config, constructed IN CODE (DR-T5): SPI0, SCK=PA5, MISO=PA6, MOSI=PA7, NSS=PA4,
    // MODE_0, 8-bit, 1 MHz target, MSB-first, software NSS. The chip resolves SPI0 to its base.
    let cfg = SpiConfig {
        spi: PeriphLabel::Spi0,
        sck: (0 << 4) | 5,  // PA5
        miso: (0 << 4) | 6, // PA6
        mosi: (0 << 4) | 7, // PA7
        nss: (0 << 4) | 4,  // PA4
        mode: 0,            // MODE_0
        data16: false,      // 8-bit
        target_hz: 1_000_000,
        lsb_first: false,
        nss_mode: NssMode::Software,
    };

    // SPI peripheral clock (chip's clock path).
    if clock::enable_spi(rcu, chip.clock(), cfg.spi).is_err() {
        set_spi_err(3);
        return;
    }

    // SCK/MOSI as AF push-pull, MISO as input, NSS as AF push-pull (software NSS, but the pin is
    // configured for completeness), all on the SPI AF (AF0 on F1x0). Enable each pin's GPIO port.
    let pins = [
        (cfg.sck, PinRole::SpiAfPushPull),
        (cfg.mosi, PinRole::SpiAfPushPull),
        (cfg.miso, PinRole::SpiInput),
        (cfg.nss, PinRole::SpiAfPushPull),
    ];
    for (pin, role) in pins {
        // route_spi_pin resolves the pin's port base, enables its GPIO clock, and writes the family
        // SPI AF mux internally (the public per-pin SPI routing path).
        if chip.route_spi_pin(pin, role).is_err() {
            set_spi_err(4);
            return;
        }
    }

    // Bring up the SPI master from the chip context + the code-level ClockConfig + SpiConfig.
    let mut spi = match Spi::bring_up(chip, &M2_CLOCK, &cfg) {
        Ok(dev) => dev,
        Err(_) => {
            set_spi_err(2);
            return;
        }
    };

    // Loopback: clock out the pattern, capture the readback. embedded-hal `transfer` is full-duplex.
    let mut readback = [0u8; 4];
    match spi.transfer(&mut readback, &SPI_PATTERN) {
        Ok(()) => {
            set_spi_readback(&readback);
            if readback == SPI_PATTERN {
                set_spi_matched(1);
            }
        }
        Err(_) => set_spi_err(0x10),
    }
}

// --- result-struct writers (volatile, through the raw pointer to the pinned static) -----------
//
// `static mut` access goes through a raw pointer + volatile writes so the optimiser cannot drop or
// reorder the stores the SWD reader depends on, and so the magic genuinely lands last.

#[inline]
fn anchors_ptr() -> *mut M2Anchors {
    RESULT_ADDR as *mut M2Anchors
}

macro_rules! store {
    ($field:ident, $val:expr) => {{
        // SAFETY: single-threaded firmware, no interrupts touch M2_ANCHORS; the only writer is this
        // code path, and reads are external (SWD). Volatile so the stores are not elided/reordered.
        unsafe {
            let p = anchors_ptr();
            core::ptr::addr_of_mut!((*p).$field).write_volatile($val);
        }
    }};
}

fn set_clock_ok() {
    store!(clock_ok, 1);
}
fn set_i2c_who_am_i(v: u8) {
    store!(i2c_who_am_i, v);
}
fn set_i2c_reg06(v: u8) {
    store!(i2c_reg06, v);
}
fn set_i2c_reg07(v: u8) {
    store!(i2c_reg07, v);
}
fn set_i2c_err(v: u8) {
    store!(i2c_err, v);
}
fn set_adc_vrefint(v: u16) {
    store!(adc_vrefint, v);
}
fn set_adc_temp(v: u16) {
    store!(adc_temp, v);
}
fn set_adc_err(v: u8) {
    store!(adc_err, v);
}
fn set_spi_readback(v: &[u8; 4]) {
    store!(spi_readback, *v);
}
fn set_spi_matched(v: u8) {
    store!(spi_matched, v);
}
fn set_spi_err(v: u8) {
    store!(spi_err, v);
}

/// Write the magic word LAST (after every anchor has run): a reader that sees it knows the run
/// completed. The compiler fence keeps it after the anchor stores.
fn write_magic() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    store!(magic, MAGIC);
}

// --- halt -------------------------------------------------------------------------------------

/// Halt forever on an unrecoverable config error (a bad blob, a missing RCU base). The reader sees
/// magic == 0 (the run never completed) rather than a half-configured peripheral.
fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
