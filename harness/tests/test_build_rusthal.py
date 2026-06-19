"""Task 4 validation: runtime-hal GPIO-AF Rust snippet builds; nm shows
regcmp_test; the extractor produces a non-empty write trace at GPIOA_BASE.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from regcmp import build_rusthal, extractor, paths, targets

RUST_BODY_GPIO_AF_F1X0 = """\
use runtime_hal::descriptor::GpioPath;
use runtime_hal::gpio::{configure_af, PinRole};

const GPIOA_BASE: u32 = 0x4800_0000;

pub fn body() {
    configure_af(GPIOA_BASE, GpioPath::AhbCtlAfsel, 2, PinRole::Tx);
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
    # All writes land in the GPIOA window.
    assert all(0x48000000 <= w.address < 0x48000400 for w in writes), writes
    assert "clean-exit" in trace.status, trace.status


def test_working_tree_restored_after_build(tmp_path):
    # The build must restore the checked-in default snippet_body.rs.
    body_file = paths.snippet_crate_dir() / "src" / "snippet_body.rs"
    before = body_file.read_text()
    out = tmp_path / "rh_gpio_af2.elf"
    build_rusthal.build("gd32f1x0", RUST_BODY_GPIO_AF_F1X0, out)
    assert body_file.read_text() == before
