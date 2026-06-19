"""Planted-bug self-test helpers (harness self-validation).

A comparison harness that never fails is worthless. These helpers take an
extracted runtime-hal access list and perturb exactly one access, so a test can
assert the engine flags that mutation (and only that) against the unperturbed
trace. Covers the wrong-value, wrong-offset, missing-write, and (M2) dropped-poll
classes (the dropped-poll class needs a with_polling trace, where reads are part
of the ordered diff).
"""

from __future__ import annotations

import copy

from .engine import TraceLine


def writes_only(lines: list[TraceLine]) -> list[TraceLine]:
    return [l for l in lines if l.op == "W"]


def mutate_wrong_value(lines: list[TraceLine], index: int, new_value: int) -> list[TraceLine]:
    """Return a copy with the write at `index` given a wrong value."""
    out = copy.deepcopy(lines)
    l = out[index]
    out[index] = TraceLine(l.op, l.size, l.address_str, new_value)
    return out


def mutate_wrong_offset(lines: list[TraceLine], index: int, new_addr_str: str) -> list[TraceLine]:
    """Return a copy with the write at `index` retargeted to a wrong offset."""
    out = copy.deepcopy(lines)
    l = out[index]
    out[index] = TraceLine(l.op, l.size, new_addr_str, l.value)
    return out


def mutate_missing_write(lines: list[TraceLine], index: int) -> list[TraceLine]:
    """Return a copy with the write at `index` dropped entirely."""
    out = copy.deepcopy(lines)
    del out[index]
    return out


def reads_only(lines: list[TraceLine]) -> list[TraceLine]:
    return [l for l in lines if l.op == "R"]


def mutate_dropped_poll(lines: list[TraceLine], index: int) -> list[TraceLine]:
    """Return a copy with the access at `index` dropped, modelling a DROPPED POLL.

    The with_polling diff is over the full ordered read+write list, so removing a status read (a
    poll) shifts the tail and the ordered diff flags it. This is the M2 dropped-poll class: a
    bring-up that proceeds before a flag is set (the F130 hang-if-done-wrong failure) drops a poll
    read from the trace, and the golden must catch it.
    """
    out = copy.deepcopy(lines)
    del out[index]
    return out
