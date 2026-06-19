"""Task 2 validation: the Unicorn extractor against a hand-written tiny ELF.

Assembles a thumbv7m snippet that writes two known words to GPIOA_BASE and
returns (bx lr), with a sentinel/_start wrapper, then asserts the extractor
captures both writes (address/value/size) and reports a clean sentinel hit (not
a step-cap).
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from regcmp import extractor, targets
from regcmp.paths import toolchain_prefix

AS = None  # resolved per-session


def _tool(name: str) -> str:
    return f"{toolchain_prefix()}{name}"


pytestmark = pytest.mark.skipif(
    shutil.which(_tool("gcc")) is None, reason="arm-none-eabi toolchain not on PATH"
)


# A tiny snippet: regcmp_test writes 0x0000000B to GPIOA_BASE+0x00 (CTL on F1x0)
# and 0x00000001 to GPIOA_BASE+0x20 (AFSEL0), then returns. Word-sized (str).
ASM = """
    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .text
    .thumb_func
    .global regcmp_test
regcmp_test:
    ldr  r0, =0x48000000      @ GPIOA_BASE
    movs r1, #0x0B
    str  r1, [r0, #0x00]      @ W4 GPIOA+0x00 = 0x0B
    movs r1, #0x01
    str  r1, [r0, #0x20]      @ W4 GPIOA+0x20 = 0x01
    bx   lr
"""


def _build_tiny_elf(tmp: Path) -> Path:
    src = tmp / "tiny.s"
    src.write_text(ASM)
    obj = tmp / "tiny.o"
    elf = tmp / "tiny.elf"
    subprocess.check_call([_tool("as"), "-mcpu=cortex-m3", "-mthumb", str(src), "-o", str(obj)])
    # Link at flash origin; keep regcmp_test. -e regcmp_test sets the ELF entry
    # but the extractor uses the symbol address directly.
    subprocess.check_call([
        _tool("ld"), "-Ttext=0x08000000", "-e", "regcmp_test",
        str(obj), "-o", str(elf),
    ])
    return elf


def test_extractor_captures_known_writes():
    target = targets.load("gd32f1x0")
    with tempfile.TemporaryDirectory(prefix="regcmp-tiny-") as d:
        elf = _build_tiny_elf(Path(d))
        # nm shows regcmp_test.
        nm = subprocess.check_output([_tool("nm"), str(elf)], text=True)
        assert "regcmp_test" in nm
        trace = extractor.extract(elf, target)

    writes = [e for e in trace.events if e.op == "W"]
    assert len(writes) == 2, f"expected 2 writes, got {writes}"
    assert (writes[0].address, writes[0].value, writes[0].size) == (0x48000000, 0x0B, 4)
    assert (writes[1].address, writes[1].value, writes[1].size) == (0x48000020, 0x01, 4)
    # Sentinel hit, not a step-cap.
    assert "clean-exit" in trace.status, trace.status
    assert "step-cap" not in trace.status


def test_symbolised_render():
    target = targets.load("gd32f1x0")
    with tempfile.TemporaryDirectory(prefix="regcmp-tiny-") as d:
        elf = _build_tiny_elf(Path(d))
        trace = extractor.extract(elf, target)
    text = trace.render()
    assert "W4 <GPIOA_BASE>+0x00 0x0000000B" in text
    assert "W4 <GPIOA_BASE>+0x20 0x00000001" in text


# --- M2: OR-MASK read responses + per-vector reset overrides ----------------------------------

# regcmp_test: RMW GPIOA+0x00 (read, set bit0, write back), then poll GPIOA+0x04
# until bit3 is set (read in a loop), then return. The poll spins unless the read
# response sets bit3.
ASM_RMW_POLL = """
    .syntax unified
    .cpu cortex-m3
    .thumb
    .section .text
    .thumb_func
    .global regcmp_test
regcmp_test:
    ldr  r0, =0x48000000      @ GPIOA_BASE
    ldr  r1, [r0, #0x00]      @ R4 read CTL (accumulated)
    orrs r1, r1, #0x01        @ set bit0
    str  r1, [r0, #0x00]      @ W4 write back
1:
    ldr  r2, [r0, #0x04]      @ R4 poll STATUS
    tst  r2, #0x08            @ bit3 set?
    beq  1b                   @ spin until set
    bx   lr
"""


def _build_rmw_poll_elf(tmp: Path) -> Path:
    src = tmp / "rmw.s"
    src.write_text(ASM_RMW_POLL)
    obj = tmp / "rmw.o"
    elf = tmp / "rmw.elf"
    subprocess.check_call([_tool("as"), "-mcpu=cortex-m3", "-mthumb", str(src), "-o", str(obj)])
    subprocess.check_call([
        _tool("ld"), "-Ttext=0x08000000", "-e", "regcmp_test", str(obj), "-o", str(elf),
    ])
    return elf


def test_or_mask_preserves_accumulated_value():
    """An {"or": mask} read returns memory|mask and does NOT clobber accumulated config bits."""
    target = targets.load("gd32f1x0")
    with tempfile.TemporaryDirectory(prefix="regcmp-rmw-") as d:
        elf = _build_rmw_poll_elf(Path(d))
        trace = extractor.extract(
            elf, target,
            # CTL (0x00) starts at a non-zero config (reset override); the OR-MASK status read on
            # STATUS (0x04) sets bit3 so the poll exits without zeroing memory.
            reset_overrides={"<GPIOA_BASE>+0x00": 0x000000A0},
            read_responses={"<GPIOA_BASE>+0x04": {"or": 0x08}},
        )
    assert "clean-exit" in trace.status, trace.status
    writes = [e for e in trace.events if e.op == "W"]
    # The RMW read saw 0xA0 (the override), set bit0 -> wrote 0xA1: the accumulated bits survived.
    assert (writes[0].address, writes[0].value) == (0x48000000, 0xA1), writes
    # The poll read returned bit3 set.
    polls = [e for e in trace.events if e.op == "R" and e.address == 0x48000004]
    assert polls and (polls[-1].value & 0x08), polls


def test_reset_override_sets_initial_memory():
    target = targets.load("gd32f1x0")
    with tempfile.TemporaryDirectory(prefix="regcmp-rmw-") as d:
        elf = _build_rmw_poll_elf(Path(d))
        trace = extractor.extract(
            elf, target,
            reset_overrides={"<GPIOA_BASE>+0x00": 0x12340000},
            read_responses={"<GPIOA_BASE>+0x04": {"or": 0x08}},
        )
    first_read = next(e for e in trace.events if e.op == "R" and e.address == 0x48000000)
    assert first_read.value == 0x12340000, first_read
