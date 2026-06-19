"""Task 8 validation: planted-bug self-test.

Captures the runtime-hal write trace, perturbs exactly one access (wrong value,
wrong offset, missing write), and asserts the engine flags exactly that mutation
and only that; the unmutated control run reports a clean match. This is the gate
that makes every later green diff trustworthy. Covers wrong-value and
wrong-offset (the missing-write class is covered too; dropped-poll waits for
with_polling, out of thin-slice scope).
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import engine, paths, runner, selftest, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
pytestmark = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")


def _runtime_hal_writes():
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return selftest.writes_only(lines), vec.mode


def test_control_run_matches():
    writes, mode = _runtime_hal_writes()
    cr = engine.compare(mode, writes, "live", writes, "golden")
    assert cr.matched, "\n".join(cr.diff)


def test_planted_wrong_value_flagged():
    golden, mode = _runtime_hal_writes()
    # Perturb the CTL write (index 0) to a wrong value.
    target_idx = 0
    assert golden[target_idx].address_str == "<GPIOA_BASE>+0x00"
    bad = selftest.mutate_wrong_value(golden, target_idx, golden[target_idx].value ^ 0x1)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched
    # Exactly one diff line, at the perturbed index/address.
    assert len(cr.diff) == 1, cr.diff
    assert "+0x00" in cr.diff[0] and "mismatch" in cr.diff[0]


def test_planted_wrong_offset_flagged():
    golden, mode = _runtime_hal_writes()
    # Retarget the AFSEL0 write (find it) to a wrong offset (+0x10, unused here).
    idx = next(i for i, l in enumerate(golden) if l.address_str == "<GPIOA_BASE>+0x20")
    bad = selftest.mutate_wrong_offset(golden, idx, "<GPIOA_BASE>+0x10")
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched
    assert len(cr.diff) == 1, cr.diff
    assert "+0x10" in cr.diff[0] or "+0x20" in cr.diff[0]


def test_planted_missing_write_flagged():
    golden, mode = _runtime_hal_writes()
    # Drop the last write (OSPD). A missing write at the tail is flagged as a
    # golden-only entry at that index.
    idx = len(golden) - 1
    bad = selftest.mutate_missing_write(golden, idx)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched
    assert len(cr.diff) == 1, cr.diff
    assert "golden-only" in cr.diff[0]


# --- M2: dropped-poll self-test on the clock with_polling vector -----------------------------

def _clock_polling_trace(vector_id: str):
    """Extract the runtime-hal with_polling trace (ordered reads + writes) for a clock vector."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return lines, vec.mode


@pytest.mark.parametrize("vector_id", [
    "clock_tree_polling_72m_f1x0",
    "clock_tree_polling_72m_f10x",
])
def test_clock_polling_control_run_matches(vector_id):
    lines, mode = _clock_polling_trace(vector_id)
    assert mode == "with_polling"
    cr = engine.compare(mode, lines, "live", lines, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "clock_tree_polling_72m_f1x0",
    "clock_tree_polling_72m_f10x",
])
def test_clock_dropped_poll_flagged(vector_id):
    """Dropping a status-read (a poll) from the with_polling trace must be caught."""
    golden, mode = _clock_polling_trace(vector_id)
    # Find a status-register READ (a poll): the second RCU CTL read is the source-stable poll exit.
    reads = [i for i, l in enumerate(golden)
             if l.op == "R" and l.address_str.startswith("<RCU_BASE>")]
    assert reads, "expected RCU status reads (polls) in the with_polling trace"
    drop_idx = reads[1] if len(reads) > 1 else reads[0]
    bad = selftest.mutate_dropped_poll(golden, drop_idx)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping a poll must fail the with_polling diff"


# --- M2 T7: dropped-poll self-test on the I2C transfer with_polling vector --------------------

def _i2c_transfer_trace(vector_id: str):
    """Extract the runtime-hal with_polling trace for an I2C transfer vector (ordered reads+writes)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return lines, vec.mode


@pytest.mark.parametrize("vector_id", [
    "i2c_transfer_whoami_f1x0",
    "i2c_transfer_whoami_f10x",
])
def test_i2c_transfer_control_run_matches(vector_id):
    lines, mode = _i2c_transfer_trace(vector_id)
    assert mode == "with_polling"
    cr = engine.compare(mode, lines, "live", lines, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "i2c_transfer_whoami_f1x0",
    "i2c_transfer_whoami_f10x",
])
def test_i2c_transfer_dropped_poll_flagged(vector_id):
    """Dropping a STAT0 poll (e.g. not waiting for RBNE before reading the byte) must be caught.

    The with_polling diff is over the full ordered read+write list, so removing a STAT0 status read
    shifts the tail and the ordered diff flags it: a transfer that drops the RBNE wait would read a
    stale/garbage byte on silicon (the dropped-poll failure class), and the golden must catch it.
    """
    golden, mode = _i2c_transfer_trace(vector_id)
    # The STAT0 (0x14) reads are the I2C status polls (SBSEND/ADDSEND/TBE/BTC/RBNE). Drop the last
    # one (the RBNE wait), the dropped-poll the plan names explicitly.
    polls = [i for i, l in enumerate(golden)
             if l.op == "R" and l.address_str == "<I2C0_BASE>+0x14"]
    assert polls, "expected STAT0 reads (polls) in the I2C with_polling trace"
    bad = selftest.mutate_dropped_poll(golden, polls[-1])
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping the RBNE poll must fail the with_polling diff"


# --- DR-T8: planted-bug self-test on a *Config FIELD-driven write (drop + swap) ---------------
#
# After the descriptor-rework the bring-up behaviour comes from code-level *Config values, not a
# parsed wiring record (DECISIONS.md #10). The two-value golden pairs prove a knob is not inert;
# these self-tests prove the diff engine catches a *Config field's write being DROPPED or its VALUE
# SWAPPED, so the matrix is trustworthy on the config (final_state) goldens, not only the polls. The
# clean field-to-register example is I2cConfig.own_addr -> SADDR0 (+0x08), a newly-explicit knob
# whose value lands in exactly one register; the I2C bring-up config golden carries it on both
# families (the I2C block is family-identical).

@pytest.mark.parametrize("vector_id", [
    "i2c_bringup_i2c0_100k_f1x0",
    "i2c_bringup_i2c0_100k_f10x",
])
def test_i2c_config_control_run_matches(vector_id):
    """The unperturbed I2C bring-up config trace matches itself (the trustworthy-diff baseline)."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    cr = engine.compare(mode, golden, "live", golden, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "i2c_bringup_i2c0_100k_f1x0",
    "i2c_bringup_i2c0_100k_f10x",
])
def test_i2c_own_addr_field_dropped_write_flagged(vector_id):
    """Dropping the SADDR0 (+0x08) write models the HAL not writing the `I2cConfig.own_addr` field
    at all (the present-but-inert / dropped-field class the rework's two-value goldens guard). The
    own-address register would keep its reset value and the bus would respond on the wrong address;
    the diff must flag exactly that one missing write."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    idxs = [i for i, l in enumerate(golden) if l.address_str == "<I2C0_BASE>+0x08"]
    assert idxs, "expected a SADDR0 (own_addr) write in the I2C bring-up config trace"
    bad = selftest.mutate_missing_write(golden, idxs[-1])
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping the own_addr (SADDR0) write must be flagged"
    assert len(cr.diff) == 1, cr.diff
    assert "+0x08" in cr.diff[0]


@pytest.mark.parametrize("vector_id", [
    "i2c_bringup_i2c0_100k_f1x0",
    "i2c_bringup_i2c0_100k_f10x",
])
def test_i2c_own_addr_field_swapped_value_flagged(vector_id):
    """Swapping the SADDR0 value models the HAL programming the wrong `I2cConfig.own_addr` (a field
    written but not from the config value). The reference own_addr is 0x24; swap it to 0x48 and the
    diff must flag exactly that one write at +0x08."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    idxs = [i for i, l in enumerate(golden) if l.address_str == "<I2C0_BASE>+0x08"]
    assert idxs, "expected a SADDR0 (own_addr) write in the I2C bring-up config trace"
    last = idxs[-1]
    assert golden[last].value == 0x24, golden[last].render()
    bad = selftest.mutate_wrong_value(golden, last, 0x48)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "a wrong own_addr (SADDR0) value must be flagged"
    assert len(cr.diff) == 1, cr.diff
    assert "+0x08" in cr.diff[0] and "mismatch" in cr.diff[0]


# --- M2 T9: dropped-poll self-test on the SPI transfer with_polling vector ---------------------

def _spi_transfer_trace(vector_id: str):
    """Extract the runtime-hal with_polling trace for an SPI transfer vector (ordered reads+writes)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return lines, vec.mode


@pytest.mark.parametrize("vector_id", [
    "spi_transfer_loopback_f1x0",
    "spi_transfer_loopback_f10x",
])
def test_spi_transfer_control_run_matches(vector_id):
    lines, mode = _spi_transfer_trace(vector_id)
    assert mode == "with_polling"
    cr = engine.compare(mode, lines, "live", lines, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "spi_transfer_loopback_f1x0",
    "spi_transfer_loopback_f10x",
])
def test_spi_transfer_dropped_poll_flagged(vector_id):
    """Dropping a STAT poll (e.g. not waiting for RBNE before reading the received byte) must be
    caught.

    The with_polling diff is over the full ordered read+write list, so removing a STAT status read
    shifts the tail and the ordered diff flags it: a transfer that drops the RBNE wait would read a
    stale/garbage byte on silicon (the dropped-poll failure class), and the golden must catch it.
    """
    golden, mode = _spi_transfer_trace(vector_id)
    # The STAT (0x08) reads are the SPI status polls (TBE/RBNE). Drop the last one (the second
    # byte's RBNE wait), the dropped-poll the plan names explicitly.
    polls = [i for i, l in enumerate(golden)
             if l.op == "R" and l.address_str == "<SPI0_BASE>+0x08"]
    assert polls, "expected STAT reads (polls) in the SPI with_polling trace"
    bad = selftest.mutate_dropped_poll(golden, polls[-1])
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping the RBNE poll must fail the with_polling diff"


# --- M2 T10/T11: dropped-poll self-test on the ADC calibration + read with_polling vectors -----

def _adc_polling_trace(vector_id: str):
    """Extract the runtime-hal with_polling trace for an ADC vector (ordered reads + writes)."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return lines, vec.mode


@pytest.mark.parametrize("vector_id", [
    "adc_calibrate_adc0_f1x0",
    "adc_calibrate_adc0_f10x",
    "adc_transfer_vrefint_f1x0",
    "adc_transfer_vrefint_f10x",
])
def test_adc_polling_control_run_matches(vector_id):
    lines, mode = _adc_polling_trace(vector_id)
    assert mode == "with_polling"
    cr = engine.compare(mode, lines, "live", lines, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id,poll_addr", [
    # Calibration polls the CTL1 (0x08) self-clearing bits (RSTCLB then CLB); drop the last poll.
    ("adc_calibrate_adc0_f1x0", "<ADC0_BASE>+0x08"),
    ("adc_calibrate_adc0_f10x", "<ADC0_BASE>+0x08"),
    # The read polls STAT (0x00) for EOC; drop the EOC wait (would read a stale/garbage sample).
    ("adc_transfer_vrefint_f1x0", "<ADC0_BASE>+0x00"),
    ("adc_transfer_vrefint_f10x", "<ADC0_BASE>+0x00"),
])
def test_adc_dropped_poll_flagged(vector_id, poll_addr):
    """Dropping a calibration-done poll (RSTCLB/CLB) or the EOC poll must be caught.

    The with_polling diff is over the full ordered read+write list, so removing a status read shifts
    the tail and the ordered diff flags it: a calibration that proceeds before the bit clears, or a
    read that proceeds before EOC, is the F130 hang-if-done-wrong / stale-sample class, and the
    golden must catch it.
    """
    golden, mode = _adc_polling_trace(vector_id)
    polls = [i for i, l in enumerate(golden)
             if l.op == "R" and l.address_str == poll_addr]
    assert polls, f"expected status reads (polls) at {poll_addr} in the ADC with_polling trace"
    bad = selftest.mutate_dropped_poll(golden, polls[-1])
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping the poll must fail the with_polling diff"


# --- M3 T4: planted-bug self-test on the timer-PWM dead-time (the wrong-dead-time class) ----------

@pytest.mark.parametrize("vector_id", [
    "timer_pwm_bringup_timer0_f1x0",
    "timer_pwm_bringup_timer0_f10x",
])
def test_pwm_config_control_run_matches(vector_id):
    """The unperturbed timer-PWM config trace matches itself (the trustworthy-diff baseline)."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    cr = engine.compare(mode, golden, "live", golden, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "timer_pwm_bringup_timer0_f1x0",
    "timer_pwm_bringup_timer0_f10x",
])
def test_pwm_wrong_dead_time_flagged(vector_id):
    """Mutating the CCHP dead-time field (DTCFG, bits [7:0], 0x1C in the reference) is the
    wrong-dead-time class the plan flags as combination-sensitive: a too-short dead-time can
    shoot-through a half-bridge. The CCHP word is 0xC1C; flip the dead-time to a wrong value and the
    engine must flag exactly that one write (MOE / off-state bits untouched)."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    idxs = [i for i, l in enumerate(golden) if l.address_str == "<TIMER0_BASE>+0x44"]
    assert idxs, "expected a CCHP (dead-time) write in the timer-PWM config trace"
    last = idxs[-1]
    assert golden[last].value & 0xFF == 0x1C, golden[last].render()
    # Keep the high bits (off-states, MOE-off), corrupt only the dead-time [7:0]: 0x1C -> 0x10.
    wrong = (golden[last].value & ~0xFF) | 0x10
    bad = selftest.mutate_wrong_value(golden, last, wrong)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "a wrong dead-time encoding must fail (shoot-through hazard)"
    assert len(cr.diff) == 1, cr.diff
    assert "+0x44" in cr.diff[0] and "mismatch" in cr.diff[0]


# --- M3 T8/T9: planted-bug self-test on the injected-ADC config + the injected read with_polling --

def _runtime_hal_filtered_writes(vector_id: str):
    """Extract the runtime-hal write trace (filtered) for a config (final_state) vector."""
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return selftest.writes_only(lines), vec.mode


@pytest.mark.parametrize("vector_id", [
    "adc_inject_bringup_adc0_f1x0",
    "adc_inject_bringup_adc0_f10x",
])
def test_inject_wrong_trigger_source_flagged(vector_id):
    """Mutating the ADC CTL1 ETSIC field (the injected external-trigger source) BREAKS the timer->ADC
    coupling: the injected group would no longer be triggered by TIMER0 CH3. The engine must flag
    exactly that mutation against the unperturbed golden (the wrong-trigger-source class the
    trigger-matrix invariant guards)."""
    golden, mode = _runtime_hal_filtered_writes(vector_id)
    # The final CTL1 (0x08) write carries ETSIC (code 1 = TIMER0 CH3) in bits [14:12]. Flip it to a
    # different source (code 4 = TIMER2 CH3): 1<<12 -> 4<<12, a wrong but plausible trigger source.
    idxs = [i for i, l in enumerate(golden) if l.address_str == "<ADC0_BASE>+0x08"]
    assert idxs, "expected a CTL1 (ETSIC) write in the injected config trace"
    last = idxs[-1]
    wrong = (golden[last].value & ~(0x7 << 12)) | (4 << 12)
    bad = selftest.mutate_wrong_value(golden, last, wrong)
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "a wrong injected trigger source must fail (breaks the timer->ADC coupling)"


def _inject_read_trace(vector_id: str):
    vec = vectors.find(vector_id)
    out_dir = paths.build_dir() / vec.vector_id
    slug = vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    lines = engine.apply_filters(
        engine.lines_from_extracted(bt.trace), vec.assert_only, vec.ignore
    )
    return lines, vec.mode


@pytest.mark.parametrize("vector_id", [
    "adc_inject_read_adc0_f1x0",
    "adc_inject_read_adc0_f10x",
])
def test_inject_read_control_run_matches(vector_id):
    lines, mode = _inject_read_trace(vector_id)
    assert mode == "with_polling"
    cr = engine.compare(mode, lines, "live", lines, "golden")
    assert cr.matched, "\n".join(cr.diff)


@pytest.mark.parametrize("vector_id", [
    "adc_inject_read_adc0_f1x0",
    "adc_inject_read_adc0_f10x",
])
def test_inject_read_dropped_eoic_poll_flagged(vector_id):
    """Dropping the STAT EOIC poll (reading the injected IDATA before the conversion completes) is
    the stale-sample / hang-if-done-wrong class. The with_polling diff is over the full ordered
    read+write list, so removing a STAT read shifts the tail and the diff flags it."""
    golden, mode = _inject_read_trace(vector_id)
    polls = [i for i, l in enumerate(golden)
             if l.op == "R" and l.address_str == "<ADC0_BASE>+0x00"]
    assert polls, "expected STAT EOIC reads (polls) in the injected-read with_polling trace"
    bad = selftest.mutate_dropped_poll(golden, polls[-1])
    cr = engine.compare(mode, bad, "live", golden, "golden")
    assert not cr.matched, "dropping the EOIC poll must fail the with_polling diff"
