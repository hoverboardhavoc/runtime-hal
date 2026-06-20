"""DR-T4 validation: the advanced-timer clock-enable + timer-AF gate pins, the two
HAL capabilities the M3 bench firmware had to bypass with raw register writes.

Two new HAL primitives, each proven by its own symbol-keyed, width-strict vector
diffed against the GD SPL on both families:

  * clock::enable_timer(Timer0) vs rcu_periph_clock_enable(RCU_TIMER0)  (APB2EN bit 11)
  * gpio::configure_af(.., TimerAfPushPull) for the six gate pins (PA8/9/10 +
    PB13/14/15) vs gpio_af_set(AF_2) (F1x0) / gpio_init(AF_PP) (F10x)

Each vector is checked three ways: the committed golden exists and carries the
expected write, a fresh runtime-hal trace matches that committed golden, and a
live two-oracle build (runtime-hal vs GD SPL) agrees. The pattern mirrors the M1
matrix / M2 SPI-clock goldens.
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import paths, runner, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")
# The two-oracle test rebuilds the GD SPL; it needs the local SPL tree + bench/harness.toml, absent
# on CI (which runs the committed-golden compare instead), so gate it to skip cleanly there.
needs_spl = pytest.mark.skipif(
    not _TOOLS or not paths.bench_config_present(),
    reason="two-oracle compare needs the local GD SPL tree + bench/harness.toml",
)

# (vector_id, family). The two new capabilities, each on both families.
DR_T4 = [
    ("clock_enable_timer0_f1x0", "gd32f1x0"),
    ("clock_enable_timer0_f10x", "gd32f10x"),
    ("gpio_af_timer0_gates_f1x0", "gd32f1x0"),
    ("gpio_af_timer0_gates_f10x", "gd32f10x"),
]


def _golden_for(vector_id: str, family: str):
    return paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"


@pytest.mark.parametrize("vector_id,family", DR_T4)
def test_golden_committed(vector_id, family):
    g = _golden_for(vector_id, family)
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


def test_timer0_clock_enable_is_apb2en_bit11_both_families():
    """The TIMER0 enable bit is APB2EN bit 11 (0x800) on BOTH families
    (rcu_periph_clock_enable(RCU_TIMER0))."""
    for vector_id, family in [
        ("clock_enable_timer0_f1x0", "gd32f1x0"),
        ("clock_enable_timer0_f10x", "gd32f10x"),
    ]:
        text = _golden_for(vector_id, family).read_text()
        assert "W4 <RCU_BASE>+0x18 0x00000800" in text, text


def test_timer_af2_nibbles_present_on_f1x0():
    """The F1x0 timer-gate golden carries the AF2 mux number in AFSEL1 (the SPI
    vector wrote AF0 = 0; AF2 is non-zero, so this pins the AF number)."""
    text = _golden_for("gpio_af_timer0_gates_f1x0", "gd32f1x0").read_text()
    # GPIOA AFSEL1: AF2 at pins 8,9,10 -> nibbles [3:0],[7:4],[11:8] = 0x222.
    assert "W4 <GPIOA_BASE>+0x24 0x00000222" in text, text
    # GPIOB AFSEL1: AF2 at pins 13,14,15 -> 0x22200000.
    assert "W4 <GPIOB_BASE>+0x24 0x22200000" in text, text


@needs_tools
@pytest.mark.parametrize("vector_id,family", DR_T4)
def test_against_committed_golden(vector_id, family):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    cr, _live = runner.compare_against_trace(vec, slug, _golden_for(vector_id, family), out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_spl
@pytest.mark.parametrize("vector_id,family", DR_T4)
def test_two_oracle_runtime_hal_matches_spl(vector_id, family):
    """runtime-hal's enable_timer / timer-AF writes reach the same state as the GD
    SPL, both families (the oracle diff, no committed golden involved)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    rh, spl = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, rh, spl, out_dir)
    assert cr.matched, "\n".join(cr.diff)
