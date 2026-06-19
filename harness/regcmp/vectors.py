"""Vector YAML loader.

A vector declares one logical configuration and one or more implementation
snippets keyed ``<library>/<target>`` (e.g. ``gd-spl/gd32f1x0``,
``runtime-hal/gd32f1x0``). Each implementation carries either an inline C body +
includes (the GD SPL) or a Rust body (runtime-hal). The vector picks a
comparison ``mode`` (``final_state`` / ``register_writes`` / ``with_polling``),
optional ``read_responses`` (symbolic addr -> scalar or list, for polled reads),
and optional ``assert_only`` / ``ignore`` address filters (mutually exclusive).

Schema (YAML):
    name: <str>
    description: <str>
    mode: register_writes | final_state | with_polling
    read_responses: { "<SYM>+0xNN": <int|list> }      # optional; {"or": mask} = OR-mask form
    reset_overrides: { "<SYM>+0xNN": <int> }           # optional; per-vector reset-value override
    assert_only: [ "<SYM>+0xLO..0xHI", ... ]           # optional
    ignore:      [ "<SYM>+0xNN", ... ]                 # optional
    implementations:
      gd-spl/gd32f1x0:
        includes: [gd32f1x0.h, gd32f1x0_gpio.h]
        body: |  <C statements>
      runtime-hal/gd32f1x0:
        body: |  <Rust module body defining `pub fn body()`>
The vector id is ``<dir>_<stem>`` (e.g. ``gpio_af_usart1_tx_pa2_f1x0``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .paths import vectors_dir

VALID_MODES = ("final_state", "register_writes", "with_polling")


@dataclass(frozen=True)
class Implementation:
    slug: str          # "<library>/<target>"
    library: str       # "gd-spl" | "runtime-hal"
    target: str        # "gd32f1x0" | "gd32f10x"
    body: str
    includes: tuple[str, ...] = ()

    @property
    def is_spl(self) -> bool:
        return self.library == "gd-spl"

    @property
    def is_runtime_hal(self) -> bool:
        return self.library == "runtime-hal"


@dataclass(frozen=True)
class Vector:
    vector_id: str
    name: str
    description: str
    mode: str
    implementations: dict[str, Implementation]
    read_responses: dict = field(default_factory=dict)
    reset_overrides: dict = field(default_factory=dict)
    assert_only: tuple[str, ...] = ()
    ignore: tuple[str, ...] = ()
    source_path: Path | None = None

    def impl_for(self, library: str, target: str | None = None) -> Implementation:
        matches = [
            i for i in self.implementations.values()
            if i.library == library and (target is None or i.target == target)
        ]
        if not matches:
            raise KeyError(f"no implementation for library={library!r} target={target!r}")
        if len(matches) > 1:
            raise KeyError(f"ambiguous: multiple {library!r} implementations: "
                           f"{[m.slug for m in matches]}")
        return matches[0]


def _vector_id(path: Path) -> str:
    return f"{path.parent.name}_{path.stem}"


def load(path: Path) -> Vector:
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: vector must be a YAML mapping")
    mode = raw.get("mode", "final_state")
    if mode not in VALID_MODES:
        raise ValueError(f"{path}: invalid mode {mode!r}; expected one of {VALID_MODES}")
    assert_only = tuple(raw.get("assert_only", ()) or ())
    ignore = tuple(raw.get("ignore", ()) or ())
    if assert_only and ignore:
        raise ValueError(f"{path}: assert_only and ignore are mutually exclusive")

    impls: dict[str, Implementation] = {}
    raw_impls = raw.get("implementations", {})
    if not raw_impls:
        raise ValueError(f"{path}: vector has no implementations")
    for slug, spec in raw_impls.items():
        if "/" not in slug:
            raise ValueError(f"{path}: implementation key {slug!r} must be <library>/<target>")
        library, target = slug.split("/", 1)
        body = spec.get("body")
        if not body:
            raise ValueError(f"{path}: implementation {slug!r} has no body")
        includes = tuple(spec.get("includes", ()) or ())
        impls[slug] = Implementation(
            slug=slug, library=library, target=target, body=body, includes=includes
        )

    return Vector(
        vector_id=_vector_id(path),
        name=raw.get("name", path.stem),
        description=raw.get("description", ""),
        mode=mode,
        implementations=impls,
        read_responses=raw.get("read_responses", {}) or {},
        reset_overrides=raw.get("reset_overrides", {}) or {},
        assert_only=assert_only,
        ignore=ignore,
        source_path=path,
    )


def find(vector_id: str) -> Vector:
    """Locate a vector by its ``<dir>_<stem>`` id under the vectors dir."""
    for yml in vectors_dir().rglob("*.yaml"):
        if _vector_id(yml) == vector_id:
            return load(yml)
    raise FileNotFoundError(f"no vector matches id {vector_id!r}")
