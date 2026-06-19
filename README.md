# runtime-hal

A runtime HAL for STM32F1-compatible Cortex-M3 MCUs (the GD32 F1x0 / F10x parts and their relatives).
ONE binary boots on any supported chip and uses HEURISTICS to detect the MCU family and measure its
peripheral capabilities at runtime, then drives the correct register model. There is no compile-time
chip selection, no descriptor data file, and no peripheral-access crate per chip.

Two halves of the HAL are at different maturity. The **cold path** is everything brought up once at
boot and used at human speed: the clock tree, GPIO, USART, I2C, SPI, and the regular ADC. The **hot
path** is the real-time motor-control loop: complementary PWM with dead-time plus a timer-triggered
("injected") ADC that fire every PWM cycle from an interrupt.

Status: early. Detection and the cold-path peripherals have been validated on real silicon on three
GD32 parts; the motor hot path is reviewed against the GD32 SPL but has not yet been run on hardware
driving a motor. See "Tested hardware" below.

## What it is

Most embedded HALs bind the chip at compile time: you pick a feature or a PAC crate for exactly one
part, and the binary only runs on that part. runtime-hal does the opposite. The chip identity is
DISCOVERED, not configured: the same image, flashed to an F103 or an F130, works out what it is
running on at boot and then talks to the silicon through the matching register model.

The idea in one line: a runtime HAL that uses heuristics to work out what the MCU is and what it can
do, so one image serves many chips.

Internally there is still a small chip-capability model (a register-model selector per peripheral
group, a base-address table, and the measured timer/ADC counts). The difference from a conventional
HAL is where that model comes from: detection fills it in at runtime, rather than a build-time choice
or an on-flash data blob.

## How detection works

Three heuristics run once at boot, before any peripheral bring-up, on the reset IRC8M clock. They are
implemented in `src/detect.rs` and `src/detect/probe.rs` and reached through `detect::detect_chip()`.

1. **Family probe (the discriminator).** The two families put GPIOA at different addresses on
   different buses: the F10x model has GPIOA on APB2 at `0x4001_0800`, the F1x0 model has it on the
   AHB GPIO area at `0x4800_0000`. The probe deliberately reads the reserved-region GPIO base of one
   family; on Cortex-M3 that read raises a precise BusFault, which is an unambiguous "not that
   family". A clean read confirms the family. This single determination fixes all four register-model
   selectors (GPIO, clock, ADC, IRQ) and the base-address table. It distinguishes the F10x APB-GPIO
   model (CRL/CRH config registers, RCC-style clock) from the F1x0 AHB-GPIO model (CTL/AFSEL config,
   RCU-style clock).

2. **Bus-fault-safe recovery (HAL-owned, no application handler).** The probe is bracketed by a
   deliberately-armed fault window. It enables the dedicated BusFault handler (`SHCSR.BUSFAULTENA`) so
   the reserved-region read traps to a BusFault rather than escalating to a HardFault, emits the
   candidate access as a fixed-width 32-bit load, and the handler advances the stacked return PC past
   that load so execution resumes after the probe instead of re-faulting forever. The window is armed
   only around each candidate read; a fault outside it is treated as a genuine error. The handler state
   is restored before bring-up. The BusFault handling is owned entirely by the HAL: for the duration of
   the probe it installs its OWN vector table in RAM (it saves `VTOR`, copies the active table so every
   other handler is preserved, points the BusFault slot at an internal handler, runs the probe, then
   restores `VTOR`), so application code defines no fault handler. The swap is probe-scoped (installed
   and restored inside detection), so it never interferes with a production RAM vector table the
   application installs later.

3. **Peripheral-presence measurement (MEASURE, do not infer).** The advanced-timer and ADC INSTANCE
   counts are measured, not taken from a per-family constant. For each candidate (TIMER0 at
   `0x4001_2C00`, TIMER7 at `0x4001_3400`; ADC0 at `0x4001_2400`, ADC1 at `0x4001_2800`, ADC2 at
   `0x4001_3C00`) the probe writes a recognizable pattern to a benign, side-effect-free scratch
   register (`TIMERx_PSC` / `ADC_WDLT`, both at offset `0x28`, both reset to 0), reads it back, and
   restores the reset value. A present instance retains the pattern; an absent slot reads back as
   zero. On real silicon the family constant proved wrong **in both directions**: a GD32F103C8 has 1
   advanced timer (the constant said 2), and a GD32F103RCT6 has 3 ADCs (the constant said 2), so the
   measured count is what `detect_chip` writes into the chip model. (On these parts, clock-enable-bit
   stickiness and the base-read-fault signal do NOT discriminate present from absent; only the scratch
   write-back does.)

The probe also reads the flash density word at `0x1FFF_F7E0` (`[15:0]` = KiB of flash), which feeds
the F10x flash page-size decision (> 128 KiB => 2 KiB pages, else 1 KiB; F1x0 is always 1 KiB).

Detection is fail-loud: if neither family matches, `detect_chip()` returns `DetectError::NoFamily`
and the firmware fails safe (stays on the reset clock, outputs untouched) rather than guessing.

## Tested hardware

Detection and the register paths have been exercised on real silicon on the three GD32 parts below.
State of validation, exactly:

- **GD32F103C8T6** and **GD32F130C8T6**: family detection and peripheral-presence measurement,
  clock-tree bring-up, and the cold-path peripherals: USART (including a polled serial link between
  two boards), I2C (hardware I2C confirmed against an external device at address 0x68), SPI, and the
  regular ADC.
- **GD32F103RCT6**: family detection and peripheral-presence measurement only. It measured 2 advanced
  timers, 3 ADCs, and 256 KiB of flash. No peripheral bring-up was run on this part.

**Not yet run on hardware: the motor hot path.** The hot path (complementary PWM with dead-time and
break, timer-triggered injected ADC, RAM vector table) is implemented and checked by register-trace
comparison against the GD32 SPL, but it has NOT been run on silicon driving a motor. Treat it as
reviewed-against-reference, not silicon-proven.

## GD32F103 (F10x) vs GD32F130 (F1x0) peripheral differences

The two families share the Cortex-M3 core and much of the peripheral set, but they diverge in several
register models. These are the differences the HAL's two code paths exist to bridge, grounded in the
GD32F10x User Manual (Rev2.6) and the GD32F1x0 User Manual (Rev3.6) and reflected in the synthesized
constants in `src/detect.rs`.

- **GPIO register model + base.** F10x: APB2 GPIO at `0x4001_0800`, configured through `CRL`/`CRH`
  (4 bits per pin: a 2-bit MODE + a 2-bit CNF, with the alternate function implied by the mode/cnf
  combination). F1x0: AHB GPIO at `0x4800_0000`, configured F0-style through `CTL` (2-bit MODE per
  pin) plus `AFSEL0`/`AFSEL1` (an explicit per-pin alternate-function mux). This is the family
  discriminator: the wrong-family base is a reserved region that bus-faults.
- **Clock controller.** F10x uses the RCC-style model; F1x0 uses the RCU model. Different enable
  registers and bit positions, e.g. the GPIO port clock is on an AHB enable register for the F1x0
  (AHB-resident GPIO) versus an APB2 enable for the F10x. The two `ClockPath` selectors own the
  divergent enable/tree register layouts.
- **IRQ layout.** F10x has separate vector lines at family-specific positions; F1x0 groups several
  interrupts (e.g. the advanced-timer break/update/trigger/commutation bundle and grouped EXTI
  lines). The `IrqLayout` selector picks the RAM-vector-table layout.
- **ADC.** F10x is dual-ADC capable and carries 2 to 3 ADC instances depending on flash density;
  F1x0 has a single ADC. `AdcPath` (Single vs Dual) follows the family; the actual instance count is
  MEASURED, not assumed.
- **Advanced timers.** F10x high-density parts carry TIMER0 + TIMER7 (two complementary-PWM-capable
  advanced timers, e.g. for two 3-phase bridges); a medium-density F10x and the F1x0 carry a single
  advanced TIMER0. Again the count is measured.
- **Flash page size.** F1x0 is always 1 KiB pages. F10x is density-dependent (1 KiB up to 128 KiB,
  2 KiB above), derived from the density word.

The measured counts tie back to the silicon: the 103 RCT6 measures 2 advanced timers / 3 ADCs, the
103 C8 measures 1 / 2, and the 130 C8 measures 1 / 1. The HAL's two register code paths exist
precisely because of the differences above, and the runtime family probe selects which path to drive.

## Other chips that might be supported

Validation has only been done on the three GD32 parts above. Beyond them, in decreasing confidence:

- **High confidence (same register family, untested):** other GD32F10x densities (F103 across
  densities, F101, and the F105/F107 connectivity line) and other GD32F1x0 (F150/F170/F190). These
  share the register models the two code paths already implement; the heuristics should resolve them,
  but they have not been run.
- **Plausible but untested, heuristics need re-validation per part:** genuine STMicroelectronics
  STM32F103 / STM32F1 (the F10x model is largely register-compatible with the ST original), and close
  STM32F1 clones such as APM32F103 (Geehy), CKS32F103, and MM32F103 (MindMotion). The family probe and
  the peripheral-presence test would need to be re-confirmed on each, since reserved-region behavior
  and peripheral maps can differ in the details.
- **Out of scope without verification:** the WCH CH32F103 is a DIFFERENT core/architecture (not a
  straight STM32F1-compatible Cortex-M3 in the relevant respects) and is not in scope without
  separate verification.

Bottom line: this is a Cortex-M3 STM32F1-compatible HAL whose detection heuristics are proven only on
the three GD32 parts listed under "Tested hardware". Adding a new part means re-validating the
detection heuristics and adding the register/peripheral knowledge in code (plus a rebuild). There is
no descriptor data path to drop a new chip definition into.

## Prior art

This HAL's distinguishing mechanism is to determine the MCU family and peripheral set from inside the
firmware, at boot, in a single binary: a reserved-region read that faults selects the register model
(F10x APB CRL/CRH vs F1x0 AHB CTL/AFSEL), and per-peripheral write-back-then-readback measures how many
advanced timers / ADCs the part actually has. Each underlying piece is established prior art. Compile-time
chip-selected HALs are the dominant, opposite model: Rust
[`stm32f1xx-hal`](https://github.com/stm32-rs/stm32f1xx-hal) fixes the part (and density) via Cargo
features, and [Zephyr's devicetree is consumed at compile time](https://docs.zephyrproject.org/latest/build/dts/howtos.html),
with no single binary across board variants. Host-side target auto-detection
([probe-rs](https://github.com/probe-rs/probe-rs), [pyOCD](https://pyocd.io/docs/target_support.html))
reads IDCODE plus the CoreSight ROM table from outside over SWD, not the firmware configuring itself.
Runtime board/SoC detection exists in firmware
([U-Boot board-ID + FIT select](https://docs.u-boot.org/en/stable/develop/devicetree/control.html),
[coreboot `fw_config`](https://doc.coreboot.org/lib/fw_config.html)), but keyed off a read identifier
plus a table, not a deliberate fault. Fault-tolerant probing (BIOS memory sizing) and write-back presence
detection ([Linux Device Drivers](https://www.xml.com/ldd/chapter/book/ch15.html)) are standard
techniques. The chip ID is known-unreliable on these clones: STM32F103 and GD32F103
[report the same `DEV_ID` 0x410](https://www.blaatschaap.be/identifying-32f103-clones/), which is why
this project probes rather than reads an ID.

Bottom line: we are not the first to do firmware-side runtime hardware detection, and we invent none of
the underlying techniques. What we found no prior example of is this exact combination: a firmware HAL
that picks its register model by a boot-time reserved-region fault and counts its peripherals by register
write-back, in one binary, with no compile-time chip selection and no descriptor. Stated as "no prior
example found", not "first ever".

## Blinky demo

A minimal program that detects the chip, takes a GPIO port (which enables its clock through the
detected chip's clock path), splits it into named pins, reconfigures one as a push-pull output, and
toggles it. The API calls (`detect_chip`, `Chip::free_jtag_pins`, `Chip::gpioa`, `GpioPort::split`,
`Pin::into_push_pull_output`) are the real names. `split()` mirrors `stm32f1xx-hal`'s
`gpioa.split()`, but `into_push_pull_output()` takes NO config-register handle: the chip is detected
at runtime, so the register model (F10x CRL/CRH vs F1x0 CTL/OMODE/OSPD) is carried by the pin and the
family branch lives inside the HAL. The application never sees a `GpioPath` or a raw register. The
output `Pin` implements the standard `embedded_hal::digital::OutputPin` trait, and the blink
interval is timed by `runtime_hal::Delay`, the HAL's `embedded_hal::delay::DelayNs` implementer
(SysTick-backed), rather than a hand-rolled busy-wait.

The LED pin is board-dependent: the example picks a pin the board wires an LED to, and calls
`chip.free_jtag_pins()` first for pins that are JTAG overlays on the F10x after reset (a no-op on the
F1x0). The application defines NO fault handler: `detect_chip()` installs its own probe-scoped vector
table for the deliberate reserved-region read and restores it before returning (see "How detection
works" above).

The full program is [`examples/src/bin/blinky.rs`](examples/src/bin/blinky.rs); see the Examples
section below for the whole suite.

## Examples

The [`examples/`](examples/) crate is a standalone, board-agnostic example suite built for the target
(it carries its own linker layout valid on both boards). ONE image per example runs unmodified on both
the GD32F103 (F10x) and the GD32F130 (F1x0): the chip is detected at runtime, so there is no per-board
build. Each example is a separate bin under [`examples/src/bin/`](examples/src/bin/):

- [`blinky`](examples/src/bin/blinky.rs): detect the chip, take a GPIO port, and toggle one LED, timed
  by the SysTick-backed `Delay`.
- [`switches`](examples/src/bin/switches.rs): read pad inputs and mirror them to LEDs (`InputPin` /
  `OutputPin` on `split()` pins).
- [`usart_link`](examples/src/bin/usart_link.rs): inter-board USART link, each board shows the other's
  2-bit counter on its LEDs (`Serial::new` pin-handle bring-up, polled RX with overrun self-recovery).
- [`buzzer`](examples/src/bin/buzzer.rs): drive a buzzer from `Timebase`, the SysTick-interrupt tick source.
- [`imu_tilt`](examples/src/bin/imu_tilt.rs): I2C MPU-6050 read and tilt, using the in-repo `imu` and
  `attitude` crates over the HAL's `embedded-hal` I2C.

Build one (from `examples/`):

```sh
cargo build --release --bin blinky
```

This builds for `thumbv7m-none-eabi`; the same image runs on both parts. (The `firmware/` crate is the
other worked target build.)

## Building and testing

- Library (host unit tests, mock register backend): `cargo test --features mock`
- Library for the firmware target: `cargo build --release --target thumbv7m-none-eabi`
- Firmware images (built per crate; there is no Cargo workspace): e.g.
  `cargo build --manifest-path firmware/Cargo.toml --target thumbv7m-none-eabi --features f103`

The GD32 Standard Peripheral Library (SPL) is the sole register reference for this project.

## License

Licensed under either of Apache-2.0 or MIT at your option.
