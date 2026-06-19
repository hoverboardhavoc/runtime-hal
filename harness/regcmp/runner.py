"""Build -> extract -> compare orchestration for a vector.

Wires the SPL builder, the runtime-hal builder, the extractor, and the engine.
Also renders a golden ``.trace`` with a provenance header and runs the
``--against-trace`` mode (a fresh runtime-hal trace vs a committed golden).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from . import build_rusthal, build_spl, engine, extractor, paths, targets, vectors
from . import __version__


# Map a target family + body to the SPL sources its peripherals need. The .c
# file names are family-prefixed (gd32f10x_*.c / gd32f1x0_*.c), so this works
# for both families. A usart body links the rcu driver too, because
# usart_baudrate_set calls rcu_clock_freq_get.
def _spl_sources_for(family: str, body: str) -> list[str]:
    srcs: list[str] = []
    if "gpio_" in body:
        srcs.append(build_spl.spl_source(family, "gpio"))
    if "rcu_" in body or "usart_" in body:
        srcs.append(build_spl.spl_source(family, "rcu"))
    if "usart_" in body:
        srcs.append(build_spl.spl_source(family, "usart"))
    # The clock-tree golden calls fmc_wscnt_set for the flash wait states (M2 T2).
    if "fmc_" in body:
        srcs.append(build_spl.spl_source(family, "fmc"))
    if "i2c_" in body:
        srcs.append(build_spl.spl_source(family, "i2c"))
        srcs.append(build_spl.spl_source(family, "rcu"))
    if "spi_" in body:
        srcs.append(build_spl.spl_source(family, "spi"))
    if "adc_" in body:
        srcs.append(build_spl.spl_source(family, "adc"))
    # The advanced-timer complementary-PWM golden (M3 T3/T4) calls timer_init / timer_channel_* /
    # timer_break_config / timer_auto_reload_shadow_enable from the timer SPL source.
    if "timer_" in body:
        srcs.append(build_spl.spl_source(family, "timer"))
    # De-dup, preserve order.
    seen: list[str] = []
    for s in srcs:
        if s not in seen:
            seen.append(s)
    return seen or [build_spl.spl_source(family, "gpio")]


@dataclass
class BuiltTrace:
    slug: str
    family: str
    trace: extractor.ExtractedTrace
    elf_path: Path
    provenance: dict


def build_and_extract(vec: vectors.Vector, slug: str, out_dir: Path) -> BuiltTrace:
    impl = vec.implementations[slug]
    family = impl.target
    tgt = targets.load(family)
    out_dir.mkdir(parents=True, exist_ok=True)
    elf = out_dir / f"{vec.vector_id}.{impl.library}.{family}.elf"

    if impl.is_spl:
        res = build_spl.build(
            family=family,
            body=impl.body,
            includes=list(impl.includes),
            spl_sources=_spl_sources_for(family, impl.body),
            out_elf=elf,
        )
        provenance = {
            "library": "gd-spl",
            "compiler": res.gcc_version,
            "chip_define": res.chip_define,
            "compile_flags": " ".join(build_spl.SPL_CFLAGS),
        }
    elif impl.is_runtime_hal:
        res = build_rusthal.build(family=family, rust_body=impl.body, out_elf=elf)
        provenance = {
            "library": "runtime-hal",
            "compiler": res.rustc_version,
            "compile_flags": "cargo build --release --target thumbv7m-none-eabi",
        }
    else:
        raise ValueError(f"unsupported library {impl.library!r}")

    tr = extractor.extract(
        elf, tgt, read_responses=vec.read_responses, reset_overrides=vec.reset_overrides
    )
    return BuiltTrace(slug=slug, family=family, trace=tr, elf_path=elf, provenance=provenance)


def golden_header(vec: vectors.Vector, bt: BuiltTrace) -> list[str]:
    p = bt.provenance
    return [
        f"# regcmp v{__version__} captured {extractor.now_iso()}",
        f"# vector:        {vec.vector_id}",
        f"# library:       {p.get('library', '')}",
        f"# target:        {bt.family}",
        f"# compiler:      {p.get('compiler', '')}",
        f"# compile_flags: {p.get('compile_flags', '')}",
        f"# emulator:      {bt.trace.emulator}",
        f"# emulation:     {bt.trace.status}",
        f"# mode:          {vec.mode}",
    ]


def compare_implementations(vec: vectors.Vector, a_slug: str, b_slug: str,
                            out_dir: Path) -> tuple[engine.CompareResult, BuiltTrace, BuiltTrace]:
    a = build_and_extract(vec, a_slug, out_dir)
    b = build_and_extract(vec, b_slug, out_dir)
    a_lines = engine.apply_filters(engine.lines_from_extracted(a.trace), vec.assert_only, vec.ignore)
    b_lines = engine.apply_filters(engine.lines_from_extracted(b.trace), vec.assert_only, vec.ignore)
    cr = engine.compare(vec.mode, a_lines, a_slug, b_lines, b_slug)
    return cr, a, b


def compare_against_trace(vec: vectors.Vector, slug: str, golden: Path,
                          out_dir: Path) -> tuple[engine.CompareResult, BuiltTrace]:
    live = build_and_extract(vec, slug, out_dir)
    live_lines = engine.apply_filters(
        engine.lines_from_extracted(live.trace), vec.assert_only, vec.ignore
    )
    golden_lines, _headers = engine.parse_trace_text(golden.read_text())
    golden_lines = engine.apply_filters(golden_lines, vec.assert_only, vec.ignore)
    cr = engine.compare(vec.mode, live_lines, slug, golden_lines, f"golden:{golden.name}")
    return cr, live


def canonical_pair(vec: vectors.Vector) -> tuple[str, str]:
    """The runtime-hal vs gd-spl pair for a vector (runtime-hal first)."""
    rh = vec.impl_for("runtime-hal")
    spl = vec.impl_for("gd-spl")
    return (rh.slug, spl.slug)
