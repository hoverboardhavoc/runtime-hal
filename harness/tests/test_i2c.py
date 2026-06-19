"""M2 T7 validation: the I2C transfer with_polling extractor gate.

The polled I2C transfer (write_read = write_bytes + read_bytes) spins on STAT0 flags (SBSEND,
ADDSEND, TBE, BTC, RBNE). With STAT0 left at its seeded reset value the loops never exit, so the
extractor must hit the step-cap and report a "likely polling target". With read_responses scripting
the STAT0/STAT1 progression, the loops terminate and the transfer traces in order. This is the gate
that makes the I2C transfer-sequencing golden trustworthy (the dropped-poll class is in
test_selftest.py).
"""

from __future__ import annotations

import shutil

import pytest

from regcmp import build_rusthal, extractor, paths, targets, vectors

_TOOLS = shutil.which("cargo") and shutil.which(f"{paths.toolchain_prefix()}gcc")
needs_tools = pytest.mark.skipif(not _TOOLS, reason="cargo or arm-none-eabi not on PATH")

VECTOR = "i2c_transfer_whoami_f1x0"
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
    # STAT0 (0x14) scripted SBSEND..RBNE; STAT1 (0x18) read twice (clear-ADDSEND).
    assert vec.read_responses["<I2C0_BASE>+0x14"] == [0x01, 0x02, 0x02, 0x80, 0x04, 0x01, 0x02, 0x02, 0x40]
    assert vec.read_responses["<I2C0_BASE>+0x18"] == [0x00, 0x00]


@needs_tools
def test_unstubbed_poll_hits_step_cap():
    """No STAT0 stub -> the SBSEND poll spins -> step-cap with a clear message."""
    _vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses={}, step_cap=20_000)
    assert "step-cap-hit" in tr.status
    assert "likely polling target" in tr.status


@needs_tools
def test_stubbed_poll_completes_and_traces_the_whoami_sequence():
    """With the STAT0/STAT1 stub the write_read terminates and the handshake traces in order."""
    vec, tgt, elf = _build_elf()
    tr = extractor.extract(elf, tgt, read_responses=vec.read_responses)
    assert "clean-exit" in tr.status, tr.status

    def sym(ev):
        return (ev.op, tgt.symbolise(ev.address), ev.value)

    seq = [sym(ev) for ev in tr.events]
    # Phase 1: write the address byte (0x68<<1 = 0xD0, write bit clear) then the register 0x75.
    addr_w = seq.index(("W", "<I2C0_BASE>+0x10", 0x68 << 1))
    reg = seq.index(("W", "<I2C0_BASE>+0x10", 0x75))
    # Phase 2: the repeated-start address byte with the READ bit set (0xD1), then the RBNE read.
    addr_r = seq.index(("W", "<I2C0_BASE>+0x10", (0x68 << 1) | 1))
    rbne = seq.index(("R", "<I2C0_BASE>+0x14", 0x40))
    data_read = next(i for i, e in enumerate(seq)
                     if e[0] == "R" and e[1] == "<I2C0_BASE>+0x10")
    assert addr_w < reg < addr_r < rbne < data_read, seq
