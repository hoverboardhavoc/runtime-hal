"""regcmp CLI: build / extract / compare / capture.

  regcmp compare <vector_id>                  build both impls, extract, diff
  regcmp compare <vector_id> --against-trace G diff a fresh runtime-hal trace
                                              against a committed golden G
  regcmp capture <vector_id>                  write the GD SPL golden .trace +
                                              BUILD.txt provenance
  regcmp build <vector_id> [--slug S]         build the impl ELF(s)
  regcmp extract <vector_id> --slug S         print a trace for one impl
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import paths, runner, vectors


def _print_result(cr) -> int:
    print(f"  mode: {cr.mode}")
    print(f"  {cr.summary}")
    for line in cr.diff:
        print(line)
    return 0 if cr.matched else 1


def cmd_compare(args) -> int:
    vec = vectors.find(args.vector)
    out_dir = paths.build_dir() / vec.vector_id
    if args.against_trace:
        slug = args.slug or vec.impl_for("runtime-hal").slug
        cr, _live = runner.compare_against_trace(vec, slug, Path(args.against_trace), out_dir)
        print(f"vector: {vec.vector_id}  {slug}  vs  golden:{Path(args.against_trace).name}")
        return _print_result(cr)
    a_slug, b_slug = runner.canonical_pair(vec)
    cr, _a, _b = runner.compare_implementations(vec, a_slug, b_slug, out_dir)
    print(f"vector: {vec.vector_id}  {a_slug}  vs  {b_slug}")
    return _print_result(cr)


def cmd_capture(args) -> int:
    vec = vectors.find(args.vector)
    # Default to the gd-spl oracle (the M1 / config goldens). A vector with no SPL impl (a
    # runtime-hal-only self-test, e.g. the clock with_polling vector) captures the runtime-hal
    # trace instead; pass --slug to force one.
    if args.slug:
        cap_slug = args.slug
    else:
        try:
            cap_slug = vec.impl_for("gd-spl").slug
        except KeyError:
            cap_slug = vec.impl_for("runtime-hal").slug
    impl = vec.implementations[cap_slug]
    library = impl.library
    family = impl.target
    out_dir = paths.build_dir() / vec.vector_id
    bt = runner.build_and_extract(vec, cap_slug, out_dir)

    spl_pin = args.spl_pin or "local"
    golden_root = paths.golden_dir() / library / spl_pin / family
    golden_root.mkdir(parents=True, exist_ok=True)
    trace_path = golden_root / f"{vec.vector_id}.trace"
    header = runner.golden_header(vec, bt)
    trace_path.write_text(bt.trace.render(header_lines=header))

    # Per-vector provenance: a single shared BUILD.txt per family dir would be
    # overwritten by whichever vector was captured last (ambiguous once a dir
    # holds several vectors, as the M1 matrix does), so name it per vector.
    build_txt = golden_root / f"{vec.vector_id}.BUILD.txt"
    p = bt.provenance
    build_txt.write_text(
        f"vector:        {vec.vector_id}\n"
        f"library:       {p.get('library')}\n"
        f"target:        {family}\n"
        f"spl_pin:       {spl_pin}\n"
        f"compiler:      {p.get('compiler')}\n"
        f"chip_define:   {p.get('chip_define', '')}\n"
        f"compile_flags: {p.get('compile_flags')}\n"
        f"emulator:      {bt.trace.emulator}\n"
        f"emulation:     {bt.trace.status}\n"
        f"mode:          {vec.mode}\n"
    )
    print(f"[capture] {trace_path}")
    print(f"[capture] {build_txt}")
    return 0


def cmd_build(args) -> int:
    vec = vectors.find(args.vector)
    out_dir = paths.build_dir() / vec.vector_id
    slugs = [args.slug] if args.slug else list(vec.implementations)
    rc = 0
    for slug in slugs:
        try:
            bt = runner.build_and_extract(vec, slug, out_dir)
            print(f"[ok] {slug} -> {bt.elf_path} ({bt.elf_path.stat().st_size}B)  {bt.trace.status}")
        except Exception as e:  # noqa: BLE001
            print(f"[fail] {slug}: {e}")
            rc = 1
    return rc


def cmd_extract(args) -> int:
    vec = vectors.find(args.vector)
    out_dir = paths.build_dir() / vec.vector_id
    slug = args.slug or vec.impl_for("runtime-hal").slug
    bt = runner.build_and_extract(vec, slug, out_dir)
    header = runner.golden_header(vec, bt)
    sys.stdout.write(bt.trace.render(header_lines=header))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="regcmp", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    pc = sub.add_parser("compare", help="build + extract + diff a vector")
    pc.add_argument("vector")
    pc.add_argument("--against-trace", help="diff a fresh runtime-hal trace against this golden .trace")
    pc.add_argument("--slug", help="implementation slug (for --against-trace)")
    pc.set_defaults(func=cmd_compare)

    pcap = sub.add_parser("capture", help="write the golden .trace + BUILD.txt")
    pcap.add_argument("vector")
    pcap.add_argument("--spl-pin", help="golden directory tag (default: local)")
    pcap.add_argument("--slug", help="implementation slug to capture (default: gd-spl, else runtime-hal)")
    pcap.set_defaults(func=cmd_capture)

    pb = sub.add_parser("build", help="build impl ELF(s) for a vector")
    pb.add_argument("vector")
    pb.add_argument("--slug")
    pb.set_defaults(func=cmd_build)

    pe = sub.add_parser("extract", help="print a trace for one impl")
    pe.add_argument("vector")
    pe.add_argument("--slug")
    pe.set_defaults(func=cmd_extract)

    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
