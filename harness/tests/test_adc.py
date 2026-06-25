"""M2 T10/T11 validation: the regular-ADC config golden + the calibration / read with_polling
goldens, both families.

T10 config goldens: runtime-hal's ADC single-conversion configuration writes vs the GD SPL adc_*
recipe (final_state, both families). The ADC register core is identical on both families, so the
runtime-hal body is shared; the goldens differ only in which SPL family build produced them.

T10 calibration with_polling: Adc::calibrate spins on the CTL1 RSTCLB then CLB self-clearing bits
(busy -> done). With the bits never clearing the bounded poll must still hit the step-cap (proving
it is a real poll); with the CTL1 stub scripting busy -> done it terminates and traces in order.

T11 read with_polling: Adc::read_channel software-triggers (SWRCST), polls STAT EOC, reads RDATA.
Same step-cap-unstubbed / traces-stubbed gate. These are the gates that make the ADC
transfer-sequencing goldens trustworthy (the dropped-poll class is in test_selftest.py).
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import build_rusthal, engine, extractor, paths, runner, targets, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")
# Two-oracle tests rebuild the GD SPL; they need the local SPL tree + bench/harness.toml, absent on
# CI (which runs the committed-golden compares instead), so gate them to skip cleanly there.
needs_spl = pytest.mark.skipif(
    not _TOOLS or not paths.spl_present(),
    reason="two-oracle compare needs the local GD SPL tree + bench/harness.toml",
)


# --- T10 config goldens (gd-spl oracle, final_state) ------------------------------------------

ADC_CONFIG = [
    ("adc_bringup_adc0_vrefint_f1x0", "gd32f1x0"),
    ("adc_bringup_adc0_vrefint_f10x", "gd32f10x"),
]

# T10 calibration + T11 read with_polling goldens (runtime-hal self-test oracle).
ADC_POLLING = [
    ("adc_calibrate_adc0_f1x0", "gd32f1x0"),
    ("adc_calibrate_adc0_f10x", "gd32f10x"),
    ("adc_transfer_vrefint_f1x0", "gd32f1x0"),
    ("adc_transfer_vrefint_f10x", "gd32f10x"),
]


@needs_spl
@pytest.mark.parametrize("vector_id,family", ADC_CONFIG)
def test_adc_config_two_oracle(vector_id, family):
    """runtime-hal's ADC bring-up reaches the same end state as the GD SPL adc_* recipe, both
    families (scan/continuous off, right-aligned 12-bit, software trigger, TSVREN for the internal
    channel, RSQ0 length-1, rank 0 = channel 17, the channel sample time)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    rh, spl = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, rh, spl, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
@pytest.mark.parametrize("vector_id,family", ADC_CONFIG)
def test_adc_config_against_golden(vector_id, family):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
@pytest.mark.parametrize("vector_id,family", ADC_CONFIG)
def test_adc_config_ctl1_is_0x9e0001(vector_id, family):
    """Headline ADC bring-up number: CTL1 = 0x9E0001 (ADCON | ETSRC software-code(7<<17) |
    ETERC(20) | TSVREN(23)) for a single software-triggered VREFINT read, both families."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.lines_from_extracted(bt.trace)
    ctl1 = [l for l in lines if l.op == "W" and l.address_str == "<ADC0_BASE>+0x08"]
    assert ctl1, "no CTL1 write in the runtime-hal trace"
    assert ctl1[-1].value == 0x9E0001, [l.render() for l in ctl1]
    golden_text = (paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace").read_text()
    assert "W4 <ADC0_BASE>+0x08 0x009E0001" in golden_text, golden_text


@pytest.mark.parametrize("vector_id,family", ADC_CONFIG)
def test_adc_config_golden_committed(vector_id, family):
    g = paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


@needs_tools
@pytest.mark.parametrize("vector_id,family", ADC_POLLING)
def test_adc_polling_against_golden(vector_id, family):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = paths.golden_dir() / "runtime-hal" / "local" / family / f"{vector_id}.trace"
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id,family", ADC_POLLING)
def test_adc_polling_golden_committed(vector_id, family):
    g = paths.golden_dir() / "runtime-hal" / "local" / family / f"{vector_id}.trace"
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()


# --- with_polling extractor gate: unstubbed -> step-cap, stubbed -> ordered trace -------------

def _build_polling_elf(vector_id, slug):
    vec = vectors.find(vector_id)
    impl = vec.implementations[slug]
    tgt = targets.load(impl.target)
    out_dir = paths.build_dir() / vec.vector_id
    out_dir.mkdir(parents=True, exist_ok=True)
    elf = out_dir / f"{vec.vector_id}.runtime-hal.{impl.target}.elf"
    build_rusthal.build(family=impl.target, rust_body=impl.body, out_elf=elf)
    return vec, tgt, elf


@needs_tools
@pytest.mark.parametrize("vector_id,slug", [
    ("adc_calibrate_adc0_f1x0", "runtime-hal/gd32f1x0"),
    ("adc_transfer_vrefint_f1x0", "runtime-hal/gd32f1x0"),
])
def test_adc_unstubbed_poll_hits_step_cap(vector_id, slug):
    """No flag stub -> the calibration / EOC poll spins -> step-cap with a clear message."""
    _vec, tgt, elf = _build_polling_elf(vector_id, slug)
    tr = extractor.extract(elf, tgt, read_responses={}, step_cap=20_000)
    assert "step-cap-hit" in tr.status
    assert "likely polling target" in tr.status


@needs_tools
def test_adc_calibrate_traces_rstclb_then_clb():
    """With the CTL1 stub, calibrate sets RSTCLB then polls it clear, sets CLB then polls it clear,
    in that order (the SPL adc_calibration_enable sequence; busy -> done)."""
    vec, tgt, elf = _build_polling_elf("adc_calibrate_adc0_f1x0", "runtime-hal/gd32f1x0")
    tr = extractor.extract(elf, tgt, read_responses=vec.read_responses)
    assert "clean-exit" in tr.status, tr.status

    def sym(ev):
        return (ev.op, tgt.symbolise(ev.address), ev.value)

    seq = [sym(ev) for ev in tr.events]
    # RSTCLB (bit3 = 0x08) set, then polled busy (read 0x08) then done (read 0x00); then CLB (bit2 =
    # 0x04) set, polled busy (read 0x04) then done.
    w_rstclb = seq.index(("W", "<ADC0_BASE>+0x08", 0x08))
    r_busy_rstclb = next(i for i, e in enumerate(seq)
                         if i > w_rstclb and e == ("R", "<ADC0_BASE>+0x08", 0x08))
    w_clb = seq.index(("W", "<ADC0_BASE>+0x08", 0x04))
    r_busy_clb = next(i for i, e in enumerate(seq)
                      if i > w_clb and e == ("R", "<ADC0_BASE>+0x08", 0x04))
    assert w_rstclb < r_busy_rstclb < w_clb < r_busy_clb, seq


@needs_tools
def test_adc_read_traces_trigger_poll_then_data():
    """With the STAT/RDATA stub, read_channel re-points rank 0, software-triggers (SWRCST), polls
    STAT EOC (busy -> done), then reads RDATA, in that order."""
    vec, tgt, elf = _build_polling_elf("adc_transfer_vrefint_f1x0", "runtime-hal/gd32f1x0")
    tr = extractor.extract(elf, tgt, read_responses=vec.read_responses)
    assert "clean-exit" in tr.status, tr.status

    def sym(ev):
        return (ev.op, tgt.symbolise(ev.address), ev.value)

    seq = [sym(ev) for ev in tr.events]
    # rank 0 re-pointed to channel 17 in RSQ2; SWRCST (bit22 = 0x400000) in CTL1; STAT EOC poll
    # busy (0x00) then done (0x02); then RDATA read.
    rsq = seq.index(("W", "<ADC0_BASE>+0x34", 0x11))
    trig = seq.index(("W", "<ADC0_BASE>+0x08", 0x400000))
    eoc_done = seq.index(("R", "<ADC0_BASE>+0x00", 0x02))
    rdata = next(i for i, e in enumerate(seq)
                 if i > eoc_done and e[0] == "R" and e[1] == "<ADC0_BASE>+0x4C")
    assert rsq < trig < eoc_done < rdata, seq


# --- dual-ADC (F10x only): regular-simultaneous config two-oracle + SYNCM headline -------------

DUAL_ADC = ("adc_dual_adc_simultaneous_f10x", "gd32f10x")


@needs_spl
def test_dual_adc_config_two_oracle():
    """The F10x Dual arm: chip.adc() -> Dual; configure_simultaneous(ch4, ch5, st) reaches the same
    end state as the GD SPL per-ADC single config + adc_mode_config(ADC_DAUL_REGULAL_PARALLEL)."""
    vector_id, _family = DUAL_ADC
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    rh, spl = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, rh, spl, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
def test_dual_adc_config_against_golden():
    vector_id, family = DUAL_ADC
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    golden = paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"
    assert golden.exists(), f"golden missing: {golden}"
    cr, _live = runner.compare_against_trace(vec, slug, golden, out_dir)
    assert cr.matched, "\n".join(cr.diff)


@needs_tools
def test_dual_adc_sync_mode_is_regular_parallel():
    """Headline dual-ADC number: ADC0 CTL0 SYNCM field ([19:16]) == 6 (ADC_DAUL_REGULAL_PARALLEL),
    the regular-parallel sync mode that couples the two ADCs."""
    vector_id, _family = DUAL_ADC
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.lines_from_extracted(bt.trace)
    ctl0 = [l for l in lines if l.op == "W" and l.address_str == "<ADC0_BASE>+0x04"]
    assert ctl0, "no ADC0 CTL0 write in the runtime-hal trace"
    syncm = (ctl0[-1].value >> 16) & 0xF
    assert syncm == 6, f"ADC0 SYNCM must be 6 (regular-parallel), got {syncm}: {[l.render() for l in ctl0]}"


@pytest.mark.parametrize("vector_id,family", [DUAL_ADC])
def test_dual_adc_golden_committed(vector_id, family):
    g = paths.golden_dir() / "gd-spl" / "local" / family / f"{vector_id}.trace"
    assert g.exists(), f"golden missing: {g}"
    assert f"# vector:        {vector_id}" in g.read_text()
