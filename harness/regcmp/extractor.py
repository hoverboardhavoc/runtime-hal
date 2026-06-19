"""Unicorn MMIO trace extractor (writes + reads, width-tagged).

Loads a thumbv7m snippet ELF, maps its LOAD segments + RAM/stack + a sentinel
page holding ``BKPT 0x55``, sets SP / LR(sentinel|1) / PC(entry|1), seeds the
target's reset values, hooks peripheral reads/writes filtered by
``Target.is_peripheral``, and runs from ``regcmp_test`` to the sentinel under a
step cap. Returns width-tagged ``TraceEvent``s. No SystemInit/main runs, so only
the snippet body's MMIO is captured (write isolation).

The entry symbol convention is fixed here: ``regcmp_test``. ``read_responses``
(symbolic addr -> scalar or sequence) let a polled status read return scripted
values so a poll loop terminates (used by later with_polling vectors; the M1
GPIO path needs none). No bit-band aliasing is needed for the M1 paths, but the
hook structure can host it later.
"""

from __future__ import annotations

import datetime as dt
from dataclasses import dataclass, field
from pathlib import Path

import unicorn
from unicorn import arm_const
from elftools.elf.elffile import ELFFile

from . import __version__
from . import targets as targets_mod

# Address-space layout in the emulator.
STACK_BASE = 0x20000000
STACK_SIZE = 16 * 1024
STACK_TOP = STACK_BASE + STACK_SIZE
SENTINEL_PC = 0x10000000
SENTINEL_SIZE = 0x1000
PAGE = 0x1000
PERIPHERAL_PAGE_SIZE = 0x10000

DEFAULT_STEP_CAP = 200_000

ENTRY_SYMBOL = "regcmp_test"


@dataclass
class TraceEvent:
    op: str  # "W" or "R"
    address: int
    value: int
    size: int


@dataclass
class ExtractedTrace:
    events: list[TraceEvent]
    target: targets_mod.Target
    status: str
    instr_count: int
    emulator: str

    def render(self, header_lines: list[str] | None = None) -> str:
        lines: list[str] = []
        if header_lines:
            lines.extend(header_lines)
        for ev in self.events:
            sym = self.target.symbolise(ev.address)
            name = self.target.register_name(ev.address)
            comment = f"   # {name}" if name else ""
            lines.append(f"{ev.op}{ev.size} {sym} 0x{ev.value:0{ev.size * 2}X}{comment}")
        return "\n".join(lines) + "\n"


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


@dataclass
class ELFInfo:
    entry_addr: int
    segments: list[tuple[int, bytes]]
    symbols: dict[str, int]


def load_elf(elf_path: Path) -> ELFInfo:
    with open(elf_path, "rb") as f:
        ef = ELFFile(f)
        segments: list[tuple[int, bytes]] = []
        for seg in ef.iter_segments():
            if seg["p_type"] != "PT_LOAD":
                continue
            vaddr = seg["p_vaddr"]
            data = seg.data()
            if seg["p_filesz"] < seg["p_memsz"]:
                data = data + b"\x00" * (seg["p_memsz"] - seg["p_filesz"])
            segments.append((vaddr, data))
        symbols: dict[str, int] = {}
        symtab = ef.get_section_by_name(".symtab")
        if symtab is None:
            raise RuntimeError(f"{elf_path} has no .symtab (stripped?)")
        for sym in symtab.iter_symbols():
            if sym.name:
                symbols[sym.name] = sym["st_value"]
    if ENTRY_SYMBOL not in symbols:
        raise RuntimeError(f"{elf_path}: {ENTRY_SYMBOL!r} symbol not found")
    return ELFInfo(entry_addr=symbols[ENTRY_SYMBOL], segments=segments, symbols=symbols)


def _align_down(v: int, page: int) -> int:
    return v & ~(page - 1)


def _align_up(v: int, page: int) -> int:
    return (v + page - 1) & ~(page - 1)


def _ensure_mapped(uc: unicorn.Uc, mapped: dict[int, int], addr: int, size: int, page: int = PAGE) -> None:
    base = _align_down(addr, page)
    end = _align_up(addr + size, page)
    cur = base
    while cur < end:
        if cur not in mapped:
            try:
                uc.mem_map(cur, page)
            except unicorn.UcError:
                pass
            mapped[cur] = page
        cur += page


@dataclass
class _ReadSeq:
    """A scripted read response for one peripheral address.

    Two flavours:
      * REPLACE (``or_mask is None``): each read returns the next scalar in ``values`` (the last
        sticks) and that value is also written into emulator memory, clobbering any accumulated
        state at the address. Used for status registers that software never RMW-writes (e.g. a
        USART STAT, or a clock CTL whose only writes are to disjoint bits).
      * OR-MASK (``or_mask`` set): each read returns ``(current_memory | or_mask)`` and does NOT
        clobber memory. Used for a status flag that lives in the SAME register software RMW-writes
        (e.g. the clock CFG0's read-only SCSS bits, which mirror the SCS field after the switch):
        the accumulated config bits are preserved while the poll still sees the flag set. This is
        the faithful way to make a poll exit on a register that also accumulates configuration.
    """
    values: list[int]
    index: int = 0
    or_mask: int | None = None

    def next_value(self) -> int:
        if self.index < len(self.values):
            v = self.values[self.index]
            self.index += 1
        else:
            v = self.values[-1]  # last sticks
        return v


def _resolve_read_responses(target, raw: dict) -> dict[int, _ReadSeq]:
    out: dict[int, _ReadSeq] = {}
    for sym_addr, value in (raw or {}).items():
        addr = target.resolve(sym_addr)
        if isinstance(value, dict) and "or" in value:
            # OR-MASK form: {"or": 0xNN} preserves accumulated memory, ORs the mask on each read.
            out[addr] = _ReadSeq(values=[0], or_mask=int(value["or"]))
        else:
            out[addr] = _ReadSeq(values=list(value) if isinstance(value, list) else [value])
    return out


def extract(
    elf_path: Path,
    target: targets_mod.Target,
    read_responses: dict | None = None,
    step_cap: int = DEFAULT_STEP_CAP,
    reset_overrides: dict | None = None,
) -> ExtractedTrace:
    info = load_elf(elf_path)
    if target.unicorn_arch != "arm":
        raise NotImplementedError(f"unicorn_arch {target.unicorn_arch!r} not supported")
    uc = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB)
    mapped: dict[int, int] = {}

    for vaddr, data in info.segments:
        _ensure_mapped(uc, mapped, vaddr, len(data))
        uc.mem_write(vaddr, data)

    _ensure_mapped(uc, mapped, STACK_BASE, STACK_SIZE)
    _ensure_mapped(uc, mapped, SENTINEL_PC, SENTINEL_SIZE)
    # Thumb BKPT #0x55 = 0xBE55 (little-endian).
    uc.mem_write(SENTINEL_PC, b"\x55\xBE")

    # Seed reset values: map the peripheral pages and write the seeded words.
    for sym, regs in target.reset_values.items():
        if sym not in target.peripheral_bases:
            continue
        base = target.peripheral_bases[sym]
        _ensure_mapped(uc, mapped, base, max(regs.keys()) + 4, page=PERIPHERAL_PAGE_SIZE)
        for offset, val in regs.items():
            uc.mem_write(base + offset, val.to_bytes(4, "little"))

    # Per-vector reset overrides: a vector can override a target's shared reset value for one run
    # (e.g. the clock-tree vector starts RCU_CFG0 from 0, while the usart vector seeds it to a
    # pre-configured 72 MHz tree). Keyed by the same "<SYM>+0xNN" symbolic addresses.
    for sym_addr, val in (reset_overrides or {}).items():
        addr = target.resolve(sym_addr)
        _ensure_mapped(uc, mapped, addr, 4, page=PERIPHERAL_PAGE_SIZE)
        uc.mem_write(addr, int(val).to_bytes(4, "little"))

    read_state = _resolve_read_responses(target, read_responses)

    events: list[TraceEvent] = []

    def hook_mem_write(uc, access, address, size, value, user_data):
        if target.is_peripheral(address):
            events.append(TraceEvent("W", address, value & ((1 << (size * 8)) - 1), size))

    def hook_mem_read(uc, access, address, size, value, user_data):
        if target.is_peripheral(address):
            if address in read_state:
                seq = read_state[address]
                if seq.or_mask is not None:
                    # OR-MASK: set the status mask bits ON TOP of the accumulated memory value, so
                    # the CPU read sees both the config bits already written and the status flag.
                    # Unlike REPLACE this never zeroes the config bits; the mask bits then ride
                    # along in subsequent RMWs identically for both oracles (they model read-only
                    # status bits that HW sets, harmlessly written back). The write before the read
                    # is how the value reaches the CPU on this hook.
                    cur = int.from_bytes(uc.mem_read(address, 4), "little")
                    v = cur | seq.or_mask
                    uc.mem_write(address, v.to_bytes(4, "little"))
                else:
                    # REPLACE: return the scripted scalar and write it into memory.
                    v = seq.next_value()
                    uc.mem_write(address, v.to_bytes(4, "little"))
                events.append(TraceEvent("R", address, v & ((1 << (size * 8)) - 1), size))
            else:
                cur = int.from_bytes(uc.mem_read(address, size), "little")
                events.append(TraceEvent("R", address, cur, size))

    def hook_mem_invalid(uc, access, address, size, value, user_data):
        # Lazily map any peripheral page the snippet touches that was not seeded.
        if target.is_peripheral(address):
            _ensure_mapped(uc, mapped, address, size, page=PERIPHERAL_PAGE_SIZE)
            return True
        return False

    uc.hook_add(unicorn.UC_HOOK_MEM_WRITE, hook_mem_write)
    uc.hook_add(unicorn.UC_HOOK_MEM_READ, hook_mem_read)
    uc.hook_add(
        unicorn.UC_HOOK_MEM_READ_UNMAPPED
        | unicorn.UC_HOOK_MEM_WRITE_UNMAPPED
        | unicorn.UC_HOOK_MEM_FETCH_UNMAPPED,
        hook_mem_invalid,
    )

    uc.reg_write(arm_const.UC_ARM_REG_SP, STACK_TOP)
    uc.reg_write(arm_const.UC_ARM_REG_LR, SENTINEL_PC | 1)  # Thumb sentinel return.
    pc = info.entry_addr | 1  # Thumb.

    last_pc = pc
    instr_count = 0

    def hook_code(uc, addr, size, ud):
        nonlocal last_pc, instr_count
        last_pc = addr
        instr_count += 1

    uc.hook_add(unicorn.UC_HOOK_CODE, hook_code)

    status = "clean-exit"
    try:
        # Run from entry to the sentinel PC. On unicorn 2.x the BKPT at the
        # `until` address stops cleanly; older builds raise UC_ERR_EXCEPTION.
        uc.emu_start(pc, SENTINEL_PC, count=step_cap)
        status = f"clean-exit (sentinel reached at instr {instr_count})"
    except unicorn.UcError as e:
        if e.errno == unicorn.UC_ERR_EXCEPTION and (last_pc & ~1) == SENTINEL_PC:
            status = f"clean-exit (BKPT sentinel at instr {instr_count})"
        else:
            status = f"emulation-error: {e} (last pc=0x{last_pc:08X}, instr {instr_count})"
    if instr_count >= step_cap:
        status = f"step-cap-hit ({step_cap} instrs); likely polling target near pc=0x{last_pc:08X}"

    emulator = f"unicorn {unicorn.__version__} (UC_ARCH_ARM, UC_MODE_THUMB)"
    return ExtractedTrace(
        events=events,
        target=target,
        status=status,
        instr_count=instr_count,
        emulator=emulator,
    )
