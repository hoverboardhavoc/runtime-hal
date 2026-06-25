"""Repo-relative path discovery and bench-config loading.

The committed harness hardcodes no absolute paths. Repo-relative directories are
resolved from this file's location; local-only facts (the GD SPL source tree,
the toolchain prefix) come from a gitignored bench config discovered at
``$REGCMP_BENCH_CONFIG`` or ``<repo>/bench/harness.toml``.
"""

from __future__ import annotations

import os
import sys
from functools import lru_cache
from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def harness_root() -> Path:
    """The ``harness/`` directory (parent of the ``regcmp`` package)."""
    return Path(__file__).resolve().parent.parent


def repo_root() -> Path:
    """The runtime-hal repo root (parent of ``harness/``)."""
    return harness_root().parent


def targets_dir() -> Path:
    return harness_root() / "target"


def vectors_dir() -> Path:
    return harness_root() / "vectors"


def build_assets_dir(library: str, target: str) -> Path:
    return harness_root() / "build_assets" / library / target


def build_dir() -> Path:
    """Ephemeral build output (gitignored)."""
    return harness_root() / "build"


def golden_dir() -> Path:
    return harness_root() / "golden"


def snippet_crate_dir() -> Path:
    return harness_root() / "snippet-crate"


def runtime_hal_crate_dir() -> Path:
    """Path the snippet crate's ``runtime-hal`` path-dependency points at."""
    return repo_root()


def bench_config_path() -> Path:
    """Locate the gitignored bench config."""
    env = os.environ.get("REGCMP_BENCH_CONFIG")
    if env:
        return Path(env)
    return repo_root() / "bench" / "harness.toml"


def bench_config_present() -> bool:
    """True if a bench config is discoverable (the local GD SPL build is wired).

    CI has no local GD SPL tree, so the SPL-build path (and tests that exercise
    it) gate on this; the runtime-hal build + --against-trace compare against the
    committed goldens needs no bench config.
    """
    return bench_config_path().exists()


@lru_cache(maxsize=1)
def bench_config() -> dict:
    """Load and cache the bench config TOML."""
    path = bench_config_path()
    if not path.exists():
        raise FileNotFoundError(
            f"bench config not found at {path}. Create bench/harness.toml (see "
            f"bench/README-harness.md) or set $REGCMP_BENCH_CONFIG. It carries "
            f"the local GD SPL source paths and the toolchain prefix."
        )
    with open(path, "rb") as f:
        return tomllib.load(f)


def toolchain_prefix() -> str:
    # The toolchain prefix has a sane default and is read at import time by some
    # tests' skip guards, so do not require the bench config just to read it
    # (CI has no bench config but does have arm-none-eabi- on PATH). Only the SPL
    # build path (spl_layout) genuinely needs the config.
    if not bench_config_present():
        return "arm-none-eabi-"
    return bench_config().get("toolchain", {}).get("arm_prefix", "arm-none-eabi-")


# The GD SPL source ships as the `third_party/GD32Firmware` submodule: a shallow pin of
# github.com/CommunityGD32Cores/GD32Firmware (the public GD SPL mirror). Local dev and CI build
# the SPL oracle from it, so no machine-local paths are needed. Per-family layout RELATIVE to the
# submodule root: the SPL `Source` dir, the include dirs the compile needs (SPL Include, CMSIS GD
# Include, CMSIS core), and the family-category define the SPL headers require.
VENDORED_SPL_SUBMODULE = "third_party/GD32Firmware"
_VENDORED_SPL_LAYOUT = {
    "gd32f10x": {
        "src_dir": "GD32F10x/GD32F10x_standard_peripheral/Source",
        "include_dirs": [
            "GD32F10x/GD32F10x_standard_peripheral/Include",
            "GD32F10x/CMSIS/GD/GD32F10x/Include",
            "GD32F10x/CMSIS",
        ],
        "chip_define": "GD32F10X_HD",
    },
    "gd32f1x0": {
        "src_dir": "GD32F1x0/GD32F1x0_standard_peripheral/Source",
        "include_dirs": [
            "GD32F1x0/GD32F1x0_standard_peripheral/Include",
            "GD32F1x0/CMSIS/GD/GD32F1x0/Include",
            "GD32F1x0/CMSIS",
        ],
        "chip_define": "GD32F130_150",
    },
}


def vendored_spl_root() -> Path:
    """Root of the GD SPL submodule (`third_party/GD32Firmware`)."""
    return repo_root() / VENDORED_SPL_SUBMODULE


def spl_present() -> bool:
    """True if the GD SPL submodule is checked out (the source the SPL oracle builds from).

    Replaces the old "bench config present?" availability signal: the SPL source now ships as a
    submodule, so the SPL-build path is available on CI too once
    ``git submodule update --init --depth 1`` has run.
    """
    return (vendored_spl_root() / "GD32F10x").is_dir()


def spl_layout(family: str) -> dict:
    """Per-family GD SPL layout (src_dir, include_dirs, chip_define).

    Defaults to the vendored ``third_party/GD32Firmware`` submodule, resolved repo-relative so no
    machine-local paths are needed (local dev and CI share this). A bench config ``[spl.<family>]``
    section, if present, overrides it (e.g. to point at a full local SPL tree).
    """
    if bench_config_present():
        spl = bench_config().get("spl", {})
        if family in spl:
            return spl[family]
    if family not in _VENDORED_SPL_LAYOUT:
        raise KeyError(f"no vendored SPL layout for family {family!r}")
    layout = _VENDORED_SPL_LAYOUT[family]
    root = vendored_spl_root()
    return {
        "src_dir": str(root / layout["src_dir"]),
        "include_dirs": [str(root / d) for d in layout["include_dirs"]],
        "chip_define": layout["chip_define"],
    }
