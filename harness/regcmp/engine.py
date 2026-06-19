"""Comparison engine: three modes, width-strict, with filters.

Modes (per-vector metadata):
  register_writes : ordered list of writes; reads ignored. Order is correctness.
  final_state     : unordered set of (address, final_value); reads ignored.
  with_polling    : ordered writes + reads; a missing poll is a failure.

All modes are width-strict: the key is ``(op, size, addr_str, value)``, so a
32-bit write never compares equal to two 16-bit writes over the same bytes.
``assert_only`` (whitelist) / ``ignore`` (blacklist) filters scope a vector's
assertion; they are mutually exclusive (the loader enforces that).
"""

from __future__ import annotations

from dataclasses import dataclass

from .extractor import ExtractedTrace
from . import targets as targets_mod


@dataclass(frozen=True)
class TraceLine:
    op: str          # "W" | "R"
    size: int
    address_str: str  # symbolic, e.g. "<GPIOA_BASE>+0x00"
    value: int

    @property
    def key(self) -> tuple:
        return (self.op, self.size, self.address_str, self.value)

    def render(self) -> str:
        return f"{self.op}{self.size} {self.address_str} 0x{self.value:0{self.size * 2}X}"


@dataclass
class CompareResult:
    matched: bool
    mode: str
    summary: str
    diff: list[str]


def lines_from_extracted(t: ExtractedTrace) -> list[TraceLine]:
    out: list[TraceLine] = []
    for ev in t.events:
        sym = t.target.symbolise(ev.address)
        out.append(TraceLine(op=ev.op, size=ev.size, address_str=sym, value=ev.value))
    return out


def parse_trace_text(text: str) -> tuple[list[TraceLine], dict[str, str]]:
    """Parse a committed .trace file into trace lines + header key/values."""
    headers: dict[str, str] = {}
    lines: list[TraceLine] = []
    for raw in text.splitlines():
        s = raw.strip()
        if not s:
            continue
        if s.startswith("#"):
            body = s[1:].strip()
            if ":" in body:
                k, v = body.split(":", 1)
                headers[k.strip()] = v.strip()
            continue
        if "#" in s:
            s = s[: s.index("#")].rstrip()
        parts = s.split()
        if len(parts) < 3:
            continue
        op_token = parts[0]
        op, size = op_token[0], int(op_token[1:])
        addr = parts[1]
        value = int(parts[2], 0)
        lines.append(TraceLine(op=op, size=size, address_str=addr, value=value))
    return lines, headers


# --- filters -----------------------------------------------------------------

def _parse_range(expr: str) -> tuple[str, int, int]:
    """Parse ``<SYMBOL>+0xLO..0xHI`` (or ``<SYMBOL>+0xN`` single, or ``<SYMBOL>``)."""
    if not expr.startswith("<"):
        raise ValueError(f"filter expr must start with <SYMBOL>: {expr!r}")
    end = expr.index(">")
    sym = expr[1:end]
    rest = expr[end + 1:]
    if not rest:
        return (sym, 0, 0xFFFFFFFF)
    if not rest.startswith("+"):
        raise ValueError(f"malformed filter expr: {expr!r}")
    rest = rest[1:]
    if ".." in rest:
        lo_s, hi_s = rest.split("..", 1)
        return (sym, int(lo_s, 0), int(hi_s, 0))
    off = int(rest, 0)
    return (sym, off, off)


def _line_in_range(line: TraceLine, sym: str, lo: int, hi: int) -> bool:
    s = line.address_str
    if not s.startswith(f"<{sym}>"):
        return False
    rest = s[len(sym) + 2:]
    if not rest:
        offset = 0
    elif rest.startswith("+"):
        offset = int(rest[1:], 0)
    else:
        return False
    return lo <= offset <= hi


def apply_filters(lines: list[TraceLine],
                  assert_only: tuple[str, ...] = (),
                  ignore: tuple[str, ...] = ()) -> list[TraceLine]:
    if assert_only:
        ranges = [_parse_range(e) for e in assert_only]
        return [l for l in lines if any(_line_in_range(l, *r) for r in ranges)]
    if ignore:
        ranges = [_parse_range(e) for e in ignore]
        return [l for l in lines if not any(_line_in_range(l, *r) for r in ranges)]
    return lines


# --- comparison --------------------------------------------------------------

def compare(mode: str, a: list[TraceLine], a_label: str,
            b: list[TraceLine], b_label: str) -> CompareResult:
    if mode == "register_writes":
        return _diff_ordered(mode,
                             [x for x in a if x.op == "W"], a_label,
                             [x for x in b if x.op == "W"], b_label)
    if mode == "with_polling":
        return _diff_ordered(mode, a, a_label, b, b_label)
    if mode == "final_state":
        a_w = {x.address_str: x for x in a if x.op == "W"}
        b_w = {x.address_str: x for x in b if x.op == "W"}
        diff: list[str] = []
        for k in sorted(set(a_w) | set(b_w)):
            la, lb = a_w.get(k), b_w.get(k)
            if la is None:
                diff.append(f"  - {b_label}-only: {lb.render()}")
            elif lb is None:
                diff.append(f"  - {a_label}-only: {la.render()}")
            elif la.value != lb.value or la.size != lb.size:
                diff.append(f"  - mismatch at {k}: {a_label}={la.render()} vs {b_label}={lb.render()}")
        return CompareResult(
            matched=not diff, mode=mode,
            summary=("match" if not diff else f"divergent ({len(diff)} differences)"),
            diff=diff,
        )
    raise ValueError(f"unknown mode {mode!r}")


def _diff_ordered(mode: str, a: list[TraceLine], a_label: str,
                  b: list[TraceLine], b_label: str) -> CompareResult:
    diff: list[str] = []
    n = max(len(a), len(b))
    for i in range(n):
        la = a[i] if i < len(a) else None
        lb = b[i] if i < len(b) else None
        if la is None:
            diff.append(f"  [{i:3d}] {b_label}-only: {lb.render()}")
        elif lb is None:
            diff.append(f"  [{i:3d}] {a_label}-only: {la.render()}")
        elif la.key != lb.key:
            diff.append(f"  [{i:3d}] mismatch: {a_label}={la.render()} vs {b_label}={lb.render()}")
    matched = not diff
    summary = (
        f"match: {min(len(a), len(b))}/{max(len(a), len(b))} entries identical"
        if matched else f"divergent ({len(diff)} differences)"
    )
    return CompareResult(matched=matched, mode=mode, summary=summary, diff=diff)
