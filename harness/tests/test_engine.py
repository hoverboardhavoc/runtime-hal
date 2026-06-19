"""Task 6 validation: engine modes, width-strictness, filters, end-to-end compare.

The end-to-end test is the thin-slice acceptance gate: runtime-hal vs the GD SPL
golden for the F1x0 GPIO-AF vector must match in register_writes mode.
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import engine, paths, runner, vectors
from regcmp.engine import TraceLine as TL

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
# The end-to-end engine tests build the GD SPL oracle, so they also need the
# local SPL bench config (absent in CI, which compares against committed goldens).
needs_tools = pytest.mark.skipif(
    not _TOOLS or not paths.bench_config_present(),
    reason="cargo/arm-none-eabi or local GD SPL bench config not available",
)


def test_width_strict_key():
    a = [TL("W", 4, "<GPIOA_BASE>+0x00", 0x20)]
    b = [TL("W", 2, "<GPIOA_BASE>+0x00", 0x20), TL("W", 2, "<GPIOA_BASE>+0x02", 0)]
    cr = engine.compare("register_writes", a, "a", b, "b")
    assert not cr.matched  # W4 != two W2 over the same bytes


def test_register_writes_ignores_reads():
    a = [TL("R", 4, "<GPIOA_BASE>+0x00", 0), TL("W", 4, "<GPIOA_BASE>+0x00", 0x20)]
    b = [TL("W", 4, "<GPIOA_BASE>+0x00", 0x20)]
    cr = engine.compare("register_writes", a, "a", b, "b")
    assert cr.matched


def test_ignore_filter():
    a = [TL("W", 4, "<GPIOA_BASE>+0x20", 0x100), TL("W", 4, "<GPIOA_BASE>+0x24", 0)]
    b = [TL("W", 4, "<GPIOA_BASE>+0x20", 0x100)]
    # Without the filter: divergent (a has the extra AFSEL1 write).
    assert not engine.compare("register_writes", a, "a", b, "b").matched
    fa = engine.apply_filters(a, ignore=("<GPIOA_BASE>+0x24",))
    fb = engine.apply_filters(b, ignore=("<GPIOA_BASE>+0x24",))
    assert engine.compare("register_writes", fa, "a", fb, "b").matched


@needs_tools
def test_thin_slice_compare_matches():
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    a_slug, b_slug = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, a_slug, b_slug, out_dir)
    assert cr.matched, "\n".join(cr.diff)
    assert cr.mode == "register_writes"


@needs_tools
def test_only_difference_is_afsel1():
    # The SOLE substantive difference between the two traces is the SPL's
    # redundant AFSEL1 (=GPIOA_BASE+0x24) no-op rewrite. Concretely: the set of
    # runtime-hal writes equals the set of SPL writes minus the AFSEL1 write,
    # and dropping that one SPL entry makes the ordered diff match exactly.
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    a_slug, b_slug = runner.canonical_pair(vec)
    a = runner.build_and_extract(vec, a_slug, out_dir)
    b = runner.build_and_extract(vec, b_slug, out_dir)
    rh_w = [l for l in engine.lines_from_extracted(a.trace) if l.op == "W"]
    spl_w = [l for l in engine.lines_from_extracted(b.trace) if l.op == "W"]

    # The SPL has exactly one more write than runtime-hal, and it is AFSEL1=0.
    assert len(spl_w) == len(rh_w) + 1, (rh_w, spl_w)
    extra = [l for l in spl_w if l.address_str == "<GPIOA_BASE>+0x24"]
    assert len(extra) == 1 and extra[0].value == 0, spl_w

    # Drop the AFSEL1 write from the SPL side; the ordered writes then match
    # runtime-hal exactly, byte-for-byte and in order.
    spl_no_afsel1 = [l for l in spl_w if l.address_str != "<GPIOA_BASE>+0x24"]
    cr = engine.compare("register_writes", rh_w, a_slug, spl_no_afsel1, b_slug)
    assert cr.matched, "\n".join(cr.diff)
