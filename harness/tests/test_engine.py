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
    # Post-refactor the USART pin-AF slice is final_state, scoped to the GPIOA window with assert_only
    # (the only public path, Usart::new, routes both pins and also touches USART/RCU). It still
    # exercises the full pipeline end-to-end and must match the live SPL.
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    a_slug, b_slug = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, a_slug, b_slug, out_dir)
    assert cr.matched, "\n".join(cr.diff)
    assert cr.mode == "final_state"


@needs_tools
def test_only_difference_is_afsel1():
    # The runtime-hal AFSEL handling (write only the half holding the pin) vs the SPL gpio_af_set
    # (rewrites BOTH AFSEL halves, a no-op AFSEL1=0) still holds post-refactor: on the GPIOA window,
    # the SPL trace contains an AFSEL1 (=GPIOA_BASE+0x24) write of 0 that the runtime-hal trace does
    # NOT. The vector excludes AFSEL1 via assert_only; here we confirm the underlying delta directly.
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    a_slug, b_slug = runner.canonical_pair(vec)
    a = runner.build_and_extract(vec, a_slug, out_dir)
    b = runner.build_and_extract(vec, b_slug, out_dir)

    def gpioa_writes(tr):
        return [l for l in engine.lines_from_extracted(tr)
                if l.op == "W" and l.address_str.startswith("<GPIOA_BASE>")]

    rh_afsel1 = [l for l in gpioa_writes(a.trace) if l.address_str == "<GPIOA_BASE>+0x24"]
    spl_afsel1 = [l for l in gpioa_writes(b.trace) if l.address_str == "<GPIOA_BASE>+0x24"]
    # The SPL rewrites AFSEL1 = 0 (redundant); runtime-hal does not touch it.
    assert spl_afsel1 and all(l.value == 0 for l in spl_afsel1), spl_afsel1
    assert not rh_afsel1, rh_afsel1
