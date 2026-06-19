"""Target description loader: per-family peripheral memory layout.

A ``target/<family>.toml`` declares, in GD vocabulary (the runtime-hal source
vocabulary: RCU_BASE, GPIOA_BASE, USART0/1/2_BASE), the peripheral bases, the
peripheral address ranges (so the extractor can tell "is this a peripheral
access?"), the per-base register names (for readable trace comments and as a
register-model conformance cross-check against the runtime-hal source), and the
reset values RMW reads observe before writing.

Both the runtime-hal snippet and the GD SPL snippet are GD-native and target the
same bases, so a captured access keys to ``<LABEL_BASE>+0xNN`` against the
runtime-hal source directly. There is no STM32 oracle in scope, hence GD-native
labels rather than an STM32-keyed target.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

from .paths import targets_dir


@dataclass(frozen=True)
class AddressRange:
    start: int
    end: int  # inclusive

    def contains(self, addr: int) -> bool:
        return self.start <= addr <= self.end


@dataclass(frozen=True)
class Target:
    name: str
    arch: str
    unicorn_arch: str
    unicorn_mode: str
    peripheral_ranges: tuple[AddressRange, ...]
    peripheral_bases: dict[str, int] = field(default_factory=dict)
    register_names: dict[str, dict[int, str]] = field(default_factory=dict)
    reset_values: dict[str, dict[int, int]] = field(default_factory=dict)

    def is_peripheral(self, addr: int) -> bool:
        return any(r.contains(addr) for r in self.peripheral_ranges)

    def symbolise(self, addr: int) -> str:
        """Return ``<SYMBOL>+0xNN`` for a peripheral address, longest-base match.

        Falls back to a raw hex address if no declared base sits at or below it.
        """
        best_name: str | None = None
        best_base: int = -1
        for name, base in self.peripheral_bases.items():
            if base <= addr and base > best_base:
                best_name = name
                best_base = base
        if best_name is None:
            return f"0x{addr:08X}"
        return f"<{best_name}>+0x{addr - best_base:02X}"

    def resolve(self, expr: str) -> int:
        """Parse ``<SYMBOL>+0xOFFSET`` (or ``<SYMBOL>``, or a bare number) to an address."""
        expr = expr.strip()
        if not expr.startswith("<"):
            return int(expr, 0)
        end = expr.index(">")
        sym = expr[1:end]
        rest = expr[end + 1:].strip()
        if sym not in self.peripheral_bases:
            raise KeyError(f"unknown symbol {sym!r} in {expr!r}")
        base = self.peripheral_bases[sym]
        if not rest:
            return base
        if not rest.startswith("+"):
            raise ValueError(f"malformed symbolic address {expr!r}")
        return base + int(rest[1:].strip(), 0)

    def register_name(self, addr: int) -> str | None:
        """The register name for an address, or None if not declared."""
        best_name: str | None = None
        best_base: int = -1
        for name, base in self.peripheral_bases.items():
            if base <= addr and base > best_base:
                best_name = name
                best_base = base
        if best_name is None or best_name not in self.register_names:
            return None
        return self.register_names[best_name].get(addr - best_base)


def _to_int(v) -> int:
    return int(v, 0) if isinstance(v, str) else int(v)


def load(name: str) -> Target:
    """Load ``target/<name>.toml``."""
    path = targets_dir() / f"{name}.toml"
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    ranges = tuple(
        AddressRange(start=_to_int(r["start"]), end=_to_int(r["end"]))
        for r in raw["peripheral_ranges"]
    )
    bases = {k: _to_int(v) for k, v in raw.get("peripheral_bases", {}).items()}
    reg_names = {
        k: {_to_int(off): nm for off, nm in v.items()}
        for k, v in raw.get("register_names", {}).items()
    }
    reset = {
        k: {_to_int(off): _to_int(val) for off, val in v.items()}
        for k, v in raw.get("reset_values", {}).items()
    }
    return Target(
        name=name,
        arch=raw["arch"],
        unicorn_arch=raw["unicorn_arch"],
        unicorn_mode=raw["unicorn_mode"],
        peripheral_ranges=ranges,
        peripheral_bases=bases,
        register_names=reg_names,
        reset_values=reset,
    )
