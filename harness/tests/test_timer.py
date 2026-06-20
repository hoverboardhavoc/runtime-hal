"""M3 T3/T4/T5 validation: the advanced-timer complementary-PWM config goldens.

Each runtime-hal timer-pwm trace compares clean both against the GD SPL (two-oracle) and against
its committed GD SPL golden (--against-trace), on both families. The headline assertions pin the
center-aligned CTL0 word and the dead-time CCHP word, and enforce the SAFETY invariant that MOE
(CCHP POEN, bit 15) is NEVER set by the config path (the bridge is left disarmed). The set_duties
vector additionally pins the three CHxCV compare writes the resolve-once handle emits (T5).
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import engine, paths, runner, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")
# Two-oracle tests rebuild the GD SPL; they need the local SPL tree + bench/harness.toml, absent on
# CI (which runs the committed-golden compares instead), so gate them to skip cleanly there.
needs_spl = pytest.mark.skipif(
    not _TOOLS or not paths.bench_config_present(),
    reason="two-oracle compare needs the local GD SPL tree + bench/harness.toml",
)

# T3/T4 full complementary-PWM bring-up config goldens (both families).
PWM_CONFIG = [
    ("timer_pwm_bringup_timer0_f1x0", "gd32f1x0"),
    ("timer_pwm_bringup_timer0_f10x", "gd32f10x"),
]

# T5 configure-via-trait + set_duties goldens (both families): the config writes plus three duties.
PWM_SETDUTIES = [
    ("timer_pwm_setduties_timer0_f1x0", "gd32f1x0"),
    ("timer_pwm_setduties_timer0_f10x", "gd32f10x"),
]

ALL_PWM = PWM_CONFIG + PWM_SETDUTIES


def _golden_for(vector_id: str, family: str):
    return paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"


@needs_spl
@pytest.mark.parametrize("vector_id,family", ALL_PWM)
def test_pwm_config_two_oracle(vector_id, family):
    """runtime-hal's TIMER0 complementary-PWM bring-up reaches the same end state as the GD SPL
    timer_init / timer_channel_output_* / timer_break_config recipe, both families."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    rh, spl = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, rh, spl, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
@pytest.mark.parametrize("vector_id,family", ALL_PWM)
def test_pwm_config_against_golden(vector_id, family):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = _golden_for(vector_id, family)
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id,family", ALL_PWM)
def test_pwm_golden_committed(vector_id, family):
    g = _golden_for(vector_id, family)
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


@needs_tools
@pytest.mark.parametrize("vector_id,family", PWM_CONFIG)
def test_pwm_ctl0_is_center_aligned_and_moe_off(vector_id, family):
    """Headline timer-PWM numbers: CTL0 = 0x1C0 (center-up | CKDIV/2 | ARSE), and CCHP NEVER has
    MOE (bit 15) set by the config path (the bridge is left disarmed: the SAFETY invariant at the
    golden boundary)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.lines_from_extracted(bt.trace)

    ctl0 = [l for l in lines if l.op == "W" and l.address_str == "<TIMER0_BASE>+0x00"]
    assert ctl0, "no CTL0 write in the runtime-hal trace"
    assert ctl0[-1].value == 0x1C0, [l.render() for l in ctl0]

    # The final CCHP word holds the dead-time + off-states; MOE (bit 15) is clear in EVERY write.
    cchp = [l for l in lines if l.op == "W" and l.address_str == "<TIMER0_BASE>+0x44"]
    assert cchp, "no CCHP write in the runtime-hal trace"
    assert cchp[-1].value == 0xC1C, [l.render() for l in cchp]
    for l in cchp:
        assert l.value & (1 << 15) == 0, f"config path must not set MOE: {l.render()}"

    # And the committed SPL golden agrees on CTL0 and the MOE-off CCHP.
    golden_text = _golden_for(vector_id, family).read_text()
    assert "W4 <TIMER0_BASE>+0x00 0x000001C0" in golden_text, golden_text
    assert "W4 <TIMER0_BASE>+0x44 0x00000C1C" in golden_text, golden_text


@needs_tools
@pytest.mark.parametrize("vector_id,family", PWM_SETDUTIES)
def test_set_duties_writes_three_compares(vector_id, family):
    """The resolve-once handle's set_duties([750,1125,1500]) writes the three phase compares
    CH0CV/CH1CV/CH2CV (and never CCHP/MOE), matching the SPL compare-value writes."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.lines_from_extracted(bt.trace)

    def last_write(off):
        ws = [l for l in lines if l.op == "W" and l.address_str == f"<TIMER0_BASE>+0x{off:02X}"]
        assert ws, f"no write at +0x{off:02X}"
        return ws[-1].value

    assert last_write(0x34) == 750
    assert last_write(0x38) == 1125
    assert last_write(0x3C) == 1500
    # MOE still off: the per-cycle handle cannot arm the bridge.
    cchp = [l for l in lines if l.op == "W" and l.address_str == "<TIMER0_BASE>+0x44"]
    for l in cchp:
        assert l.value & (1 << 15) == 0, f"set_duties path must not set MOE: {l.render()}"


# --- M3 T6/T8/T9: the timer TRGO trigger config, the injected ADC, and the trigger-matrix invariant

# T6 timer trigger-output config goldens (both families): CH3 compare + the CR2/CTL1 MMC TRGO select.
TRGO_CONFIG = [
    ("timer_trgo_trigger_timer0_f1x0", "gd32f1x0"),
    ("timer_trgo_trigger_timer0_f10x", "gd32f10x"),
]

# T8 timer-triggered injected-ADC config goldens (both families): the injected group + trigger source.
INJECT_CONFIG = [
    ("adc_inject_bringup_adc0_f1x0", "gd32f1x0"),
    ("adc_inject_bringup_adc0_f10x", "gd32f10x"),
]

# T9 injected "trigger then read" with_polling goldens (both families): wait EOIC then read IDATA.
INJECT_READ = [
    ("adc_inject_read_adc0_f1x0", "gd32f1x0"),
    ("adc_inject_read_adc0_f10x", "gd32f10x"),
]


@needs_spl
@pytest.mark.parametrize("vector_id,family", TRGO_CONFIG + INJECT_CONFIG)
def test_hotpath_config_two_oracle(vector_id, family):
    """runtime-hal's T6 timer-trigger config and T8 injected-ADC config each reach the same end
    state as the GD SPL, both families."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    rh, spl = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, rh, spl, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
@pytest.mark.parametrize("vector_id,family", TRGO_CONFIG + INJECT_CONFIG)
def test_hotpath_config_against_golden(vector_id, family):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = _golden_for(vector_id, family)
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id,family", TRGO_CONFIG + INJECT_CONFIG)
def test_hotpath_config_golden_committed(vector_id, family):
    g = _golden_for(vector_id, family)
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


@needs_tools
@pytest.mark.parametrize("vector_id,family", INJECT_READ)
def test_inject_read_against_golden(vector_id, family):
    """The T9 trigger-then-read with_polling golden: wait EOIC busy->done, THEN read IDATA, vs its
    own committed runtime-hal golden (the poll-sequence self-test)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = paths.golden_dir() / "runtime-hal" / "local" / family / f"{vector_id}.trace"
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id,family", INJECT_READ)
def test_inject_read_golden_committed(vector_id, family):
    g = paths.golden_dir() / "runtime-hal" / "local" / family / f"{vector_id}.trace"
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


# --- The cross-peripheral trigger-matrix invariant (TESTING.md "Hot path: the timer-to-ADC trigger
# coupling"): the single check spanning two peripherals that proves the timer->ADC coupling. ---------

@needs_tools
@pytest.mark.parametrize("family", ["gd32f1x0", "gd32f10x"])
def test_trigger_matrix_invariant(family):
    """The timer CH3 compare sits at ~CAR-1 (2249) AND the ADC injected external-trigger-source
    (ETSIC) field selects exactly TIMER0 CH3 (code 1). Asserted ACROSS the two peripherals' traces
    (the T6 timer trigger vector + the T8 injected-ADC vector), not as a single-register diff: this
    is the one check that proves the timer->ADC coupling (a wrong ETSIC or a misplaced compare
    breaks it). The PWM period is CAR/ARR = 2250, so the trigger compare 2249 is the up-count top."""
    timer_vec = "timer_trgo_trigger_timer0_" + family[-4:]
    adc_vec = "adc_inject_bringup_adc0_" + family[-4:]

    # Timer side: the CH3CV compare write is the trigger compare at ~CAR-1 (2249 = CAR 2250 - 1).
    tvec = vectors.find(timer_vec)
    tslug = tvec.impl_for("runtime-hal").slug
    tlines = engine.lines_from_extracted(
        runner.build_and_extract(tvec, tslug, paths.build_dir() / tvec.vector_id).trace
    )
    ch3cv = [l for l in tlines if l.op == "W" and l.address_str == "<TIMER0_BASE>+0x40"]
    assert ch3cv, "no CH3CV (trigger compare) write in the timer trace"
    PERIOD = 2250
    assert ch3cv[-1].value == PERIOD - 1, f"trigger compare must sit at ~CAR-1: {ch3cv[-1].render()}"
    # Timer side: the CTL1 MMC field (master-mode TRGO select) is set (UPDATE = 2<<4 = 0x20).
    ctl1_timer = [l for l in tlines if l.op == "W" and l.address_str == "<TIMER0_BASE>+0x04"]
    assert ctl1_timer, "no CTL1 (MMC/TRGO select) write in the timer trace"
    assert ctl1_timer[-1].value & (0x7 << 4) == (2 << 4), "TRGO master mode = UPDATE"

    # ADC side: the CTL1 ETSIC field ([14:12]) selects TIMER0 CH3 (code 1 = 1<<12), and ETEIC (bit
    # 15, injected ext-trigger enable) is set. This is the source the timer CH3 compare feeds.
    avec = vectors.find(adc_vec)
    aslug = avec.impl_for("runtime-hal").slug
    alines = engine.lines_from_extracted(
        runner.build_and_extract(avec, aslug, paths.build_dir() / avec.vector_id).trace
    )
    ctl1_adc = [l for l in alines if l.op == "W" and l.address_str == "<ADC0_BASE>+0x08"]
    assert ctl1_adc, "no ADC CTL1 (ETSIC) write in the injected trace"
    final = ctl1_adc[-1].value
    etsic = (final >> 12) & 0x7
    assert etsic == 1, f"ADC injected ETSIC must select TIMER0 CH3 (code 1), got {etsic}"
    assert final & (1 << 15), "ETEIC (injected ext-trigger enable) must be set"
