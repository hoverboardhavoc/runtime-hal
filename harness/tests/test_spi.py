"""M2 T9 validation: the SPI transfer with_polling extractor gate.

The polled SPI transfer (transfer_byte) spins on STAT flags (TBE then RBNE, per byte). With STAT
left at its seeded reset value the loops never exit, so the extractor must hit the step-cap and
report a "likely polling target". With read_responses scripting the STAT progression, the loops
terminate and the transfer traces in order. This is the gate that makes the SPI transfer-sequencing
golden trustworthy (the dropped-poll class is in test_selftest.py).
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import build_rusthal, extractor, paths, targets, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")

VECTOR = "spi_transfer_loopback_f1x0"
SLUG = "runtime-hal/gd32f1x0"


def _build_elf():
    vec = vectors.find(VECTOR)
    impl = vec.implementations[SLUG]
    tgt = targets.load(impl.target)
    out_dir = paths.build_dir() / vec.vector_id
    out_dir.mkdir(parents=True, exist_ok=True)
    elf = out_dir / f"{vec.vector_id}.runtime-hal.{impl.target}.elf"
    build_rusthal.build(family=impl.target, rust_body=impl.body, out_elf=elf)
    return vec, tgt, elf


def test_vector_declares_with_polling_and_seed():
    vec = vectors.find(VECTOR)
    assert vec.mode == "with_polling"
    # STAT (0x08) scripted TBE then RBNE, per byte (2 bytes).
    assert vec.read_responses["<SPI0_BASE>+0x08"] == [0x02, 0x01, 0x02, 0x01]


@needs_tools
def test_unstubbed_poll_hits_step_cap():
    """No STAT stub -> the TBE poll spins -> step-cap with a clear message."""
    _vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses={}, step_cap=20_000)
    assert "step-cap-hit" in tr.status
    assert "likely polling target" in tr.status


@needs_tools
def test_stubbed_poll_completes_and_traces_the_transfer_sequence():
    """With the STAT stub the transfer terminates and the full-duplex handshake traces in order."""
    vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses=vec.read_responses)
    assert "clean-exit" in tr.status, tr.status

    def sym(ev):
        return (ev.op, tgt.symbolise(ev.address), ev.value)

    seq = [sym(ev) for ev in tr.events]
    # Byte 0: TBE poll (STAT read 0x02) -> write DATA 0xA5 -> RBNE poll (STAT read 0x01) -> read DATA.
    tbe0 = seq.index(("R", "<SPI0_BASE>+0x08", 0x02))
    w0 = seq.index(("W", "<SPI0_BASE>+0x0C", 0xA5))
    rbne0 = seq.index(("R", "<SPI0_BASE>+0x08", 0x01))
    rd0 = next(i for i, e in enumerate(seq)
               if i > rbne0 and e[0] == "R" and e[1] == "<SPI0_BASE>+0x0C")
    # Byte 1: the second TBE poll, the 0x5A write, the second RBNE poll.
    w1 = seq.index(("W", "<SPI0_BASE>+0x0C", 0x5A))
    assert tbe0 < w0 < rbne0 < rd0 < w1, seq
