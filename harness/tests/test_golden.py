"""Task 7 validation: a fresh runtime-hal trace matches the committed golden via
--against-trace; a deliberate edit to the golden makes it fail.
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import engine, paths, runner, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")

GOLDEN = (paths.golden_dir() / "gd-spl" / "local" / "gd32f1x0"
          / "gpio_af_usart1_tx_pa2_f1x0.trace")


def test_committed_golden_exists():
    assert GOLDEN.exists(), f"golden missing: {GOLDEN}"
    text = GOLDEN.read_text()
    assert "# vector:        gpio_af_usart1_tx_pa2_f1x0" in text
    # Post-refactor: Usart::new routes BOTH PA2 (TX) and PA3 (RX), so CTL carries both pins' AF mode
    # (2<<4 | 2<<6 = 0xA0), not just PA2's 0x20.
    assert "W4 <GPIOA_BASE>+0x00 0x000000A0" in text


@needs_tools
def test_against_committed_golden_passes():
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    cr, _live = runner.compare_against_trace(vec, slug, GOLDEN, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
def test_edited_golden_fails(tmp_path):
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    edited = tmp_path / "edited.trace"
    # Perturb the CTL write value 0xA0 -> 0xA1.
    edited.write_text(GOLDEN.read_text().replace("0x000000A0", "0x000000A1"))
    cr, _live = runner.compare_against_trace(vec, slug, edited, out_dir)
    assert not cr.matched
    assert any("+0x00" in d for d in cr.diff)
