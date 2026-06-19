"""Task 3 validation: GD SPL GPIO-AF snippet builds to an ELF with regcmp_test.

Builds the F1x0 GPIO-AF body (gpio_mode_set / gpio_af_set /
gpio_output_options_set for PA2 = USART1 TX at AF1) against the local GD SPL
source, asserts nm shows regcmp_test, and objdump shows the body is non-empty
and calls into gpio_*.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from regcmp import build_spl, paths

GPIO_AF_BODY_F1X0 = """\
gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);
gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
"""

_GCC = f"{paths.toolchain_prefix()}gcc"

pytestmark = pytest.mark.skipif(
    shutil.which(_GCC) is None or not paths.bench_config_present(),
    reason="arm-none-eabi toolchain or local GD SPL bench config not available",
)


def test_spl_gpio_af_builds(tmp_path):
    out = tmp_path / "spl_gpio_af.elf"
    res = build_spl.build(
        family="gd32f1x0",
        body=GPIO_AF_BODY_F1X0,
        includes=["gd32f1x0.h", "gd32f1x0_gpio.h"],
        spl_sources=build_spl.SPL_SOURCES_F1X0["gpio"],
        out_elf=out,
    )
    assert res.elf_path.exists()
    nm = subprocess.check_output([f"{paths.toolchain_prefix()}nm", str(out)], text=True)
    assert "regcmp_test" in nm
    # The gpio_* SPL functions linked in.
    assert "gpio_mode_set" in nm
    assert "gpio_af_set" in nm
    assert "gpio_output_options_set" in nm

    dis = subprocess.check_output(
        [f"{paths.toolchain_prefix()}objdump", "-d", str(out)], text=True
    )
    # regcmp_test disassembly present and calls gpio_* (bl targets).
    assert "<regcmp_test>:" in dis
    seg = dis.split("<regcmp_test>:", 1)[1]
    # body must be non-empty (more than just a bx lr).
    body_lines = [l for l in seg.splitlines()[:40] if "\t" in l]
    assert len(body_lines) > 3, "regcmp_test body looks empty"
    assert "bl" in seg.split("\n\n", 1)[0]
