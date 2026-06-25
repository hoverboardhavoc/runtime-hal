"""Task 4 validation: runtime-hal GPIO-AF Rust snippet builds; nm shows
regcmp_test; the extractor produces a non-empty write trace at GPIOA_BASE.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from regcmp import build_rusthal, extractor, paths, targets

# Post-refactor public-API GPIO-AF body: gpio::configure_af is pub(crate), so route a timer gate pin
# through the public chip router (which writes the GPIOA AF registers plus an RCU port-clock enable).
RUST_BODY_GPIO_AF_F1X0 = """\
use runtime_hal::{AddrTable, AdcPath, ClockPath, GpioPath, IrqLayout, McuDescriptor, PageSize, PeriphLabel};
use runtime_hal::Chip;

pub fn body() {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Rcu, 0x4002_1000);
    addrs.set(PeriphLabel::Gpioa, 0x4800_0000);
    let chip = Chip::from_descriptor(McuDescriptor {
        gpio: GpioPath::AhbCtlAfsel, clock: ClockPath::F1x0Rcu, adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped, addrs, flash_page: PageSize::K1, flash_kib: 64, adv_timers: 1, adc_count: 1,
    });
    let _ = chip.route_advanced_pwm_pin(0x08);
}
"""

pytestmark = pytest.mark.skipif(
    shutil.which("cargo") is None or shutil.which(f"{paths.toolchain_prefix()}gcc") is None,
    reason="cargo or arm-none-eabi toolchain not on PATH",
)


def test_rusthal_gpio_af_builds_and_traces(tmp_path):
    out = tmp_path / "rh_gpio_af.elf"
    res = build_rusthal.build("gd32f1x0", RUST_BODY_GPIO_AF_F1X0, out)
    assert res.elf_path.exists()

    nm = subprocess.check_output([f"{paths.toolchain_prefix()}nm", str(out)], text=True)
    assert "regcmp_test" in nm

    target = targets.load("gd32f1x0")
    trace = extractor.extract(out, target)
    writes = [e for e in trace.events if e.op == "W"]
    assert writes, f"no writes captured; status={trace.status}"
    # The router writes the GPIOA AF registers (plus an RCU port-clock enable); assert the GPIOA
    # window writes are present (the RCU enable is the only non-GPIOA write).
    gpioa = [w for w in writes if 0x48000000 <= w.address < 0x48000400]
    assert gpioa, f"no GPIOA writes captured; writes={writes}"
    assert "clean-exit" in trace.status, trace.status


def test_working_tree_restored_after_build(tmp_path):
    # The build must restore the checked-in default snippet_body.rs.
    body_file = paths.snippet_crate_dir() / "src" / "snippet_body.rs"
    before = body_file.read_text()
    out = tmp_path / "rh_gpio_af2.elf"
    build_rusthal.build("gd32f1x0", RUST_BODY_GPIO_AF_F1X0, out)
    assert body_file.read_text() == before
