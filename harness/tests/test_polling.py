"""Task 9 validation: with_polling mode + read-response stubbing + step-cap.

A polled transfer (Usart::write_byte) spins on STAT.TBE then STAT.TC. With STAT
left at its seeded reset value the loops never exit, so the extractor must hit
the step-cap and report a "likely polling target". With read_responses scripting
STAT (TBE set, then TC set), the loops terminate and the transfer traces: a STAT
read, the TDATA write, a STAT read. This is the gate that makes USART transfer
vectors trustworthy.
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import (
    build_rusthal,
    extractor,
    paths,
    targets,
    vectors,
)

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")

VECTOR = "usart_write_byte_usart1_f1x0"
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


def test_vector_declares_with_polling_and_read_responses():
    vec = vectors.find(VECTOR)
    assert vec.mode == "with_polling"
    # STAT (F1x0 0x1C) scripted: TBE set, then TC set.
    assert vec.read_responses == {"<USART1_BASE>+0x1C": [0x80, 0xC0]}


@needs_tools
def test_unstubbed_poll_hits_step_cap():
    """No STAT stub -> the TBE/TC poll spins -> step-cap with a clear message."""
    _vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses={}, step_cap=20_000)
    assert "step-cap-hit" in tr.status
    assert "likely polling target" in tr.status


@needs_tools
def test_stubbed_poll_completes_and_traces():
    """With the STAT stub the polled send terminates and the transfer traces."""
    vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses=vec.read_responses)
    assert "clean-exit" in tr.status, tr.status

    # The transfer tail: read STAT (TBE=0x80), write TDATA (F1x0 0x28) = 0x41,
    # read STAT (TC=0xC0). Assert those three events appear in order.
    def sym(ev):
        return (ev.op, tgt.symbolise(ev.address), ev.value)

    seq = [sym(ev) for ev in tr.events]
    tbe = seq.index(("R", "<USART1_BASE>+0x1C", 0x80))
    tdata = seq.index(("W", "<USART1_BASE>+0x28", 0x41))
    tc = seq.index(("R", "<USART1_BASE>+0x1C", 0xC0))
    assert tbe < tdata < tc, seq
