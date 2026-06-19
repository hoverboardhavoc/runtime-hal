"""Task 1 validation: target description round-trip + register-model conformance.

Round-trips symbolise(resolve(...)), asserts is_peripheral over the declared
ranges, and cross-checks that every register offset named in the runtime-hal
source (src/{gpio,clock,usart}.rs) has a register_names entry in the target.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from regcmp import targets
from regcmp.paths import repo_root


def test_round_trip_symbolise_resolve():
    t = targets.load("gd32f1x0")
    addr = t.resolve("<GPIOA_BASE>+0x20")
    assert addr == 0x48000000 + 0x20
    assert t.symbolise(addr) == "<GPIOA_BASE>+0x20"
    # Bare base resolves and symbolises to +0x00.
    base = t.resolve("<RCU_BASE>")
    assert base == 0x40021000
    assert t.symbolise(base) == "<RCU_BASE>+0x00"


def test_is_peripheral_over_declared_ranges():
    for fam in ("gd32f1x0", "gd32f10x"):
        t = targets.load(fam)
        for name, base in t.peripheral_bases.items():
            assert t.is_peripheral(base), f"{fam}: {name}@{base:#x} not in a range"
        # A clearly-non-peripheral address (flash, SRAM) is not a peripheral.
        assert not t.is_peripheral(0x08000000)
        assert not t.is_peripheral(0x20000000)


def test_longest_match_symbolise():
    # Two GPIO bases 0x400 apart: an address in GPIOB must not resolve to GPIOA.
    t = targets.load("gd32f1x0")
    a = t.resolve("<GPIOB_BASE>+0x00")
    assert t.symbolise(a) == "<GPIOB_BASE>+0x00"


# --- register-model conformance: source offsets must be in register_names ----

def _source_offsets(text: str, const_names: list[str]) -> set[int]:
    """Extract `const NAME: u32 = 0xNN;` values for the named constants."""
    out = set()
    for name in const_names:
        m = re.search(rf"const {name}:\s*u32\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*;", text)
        if m:
            out.add(int(m.group(1), 0))
    return out


def test_gpio_offsets_have_register_names():
    gpio_src = (repo_root() / "src" / "gpio.rs").read_text()
    f1x0 = _source_offsets(gpio_src, ["F1X0_CTL", "F1X0_OMODE", "F1X0_OSPD",
                                      "F1X0_PUD", "F1X0_AFSEL0", "F1X0_AFSEL1"])
    t = targets.load("gd32f1x0")
    named = set(t.register_names["GPIOA_BASE"].keys())
    assert f1x0 <= named, f"gpio.rs F1x0 offsets missing from target: {f1x0 - named}"

    f10x = _source_offsets(gpio_src, ["F10X_CTL0", "F10X_CTL1"])
    t10 = targets.load("gd32f10x")
    named10 = set(t10.register_names["GPIOA_BASE"].keys())
    assert f10x <= named10, f"gpio.rs F10x offsets missing: {f10x - named10}"


def test_clock_offsets_have_register_names():
    clk = (repo_root() / "src" / "clock.rs").read_text()
    # AHBEN/APB2EN/APB1EN are module consts.
    offs = _source_offsets(clk, ["AHBEN", "APB2EN", "APB1EN"])
    assert offs == {0x14, 0x18, 0x1C}
    t1 = targets.load("gd32f1x0")
    named1 = set(t1.register_names["RCU_BASE"].keys())
    assert offs <= named1, f"clock.rs offsets missing from f1x0 RCU: {offs - named1}"
    # F10x has no AHBEN GPIO enable; APB2EN/APB1EN must be named.
    t0 = targets.load("gd32f10x")
    named0 = set(t0.register_names["RCU_BASE"].keys())
    assert {0x18, 0x1C} <= named0


def test_usart_offsets_have_register_names():
    usart_src = (repo_root() / "src" / "usart.rs").read_text()
    # The F1X0 / F10X UsartModel literals carry the offsets; extract them.
    def model_offsets(family: str) -> set[int]:
        m = re.search(rf"pub const {family}: UsartModel = UsartModel \{{(.*?)\}};",
                      usart_src, re.S)
        assert m, f"{family} model not found"
        body = m.group(1)
        offs = set()
        for field in ("stat", "tx_data", "rx_data", "baud", "ctl0", "ctl1", "ctl2"):
            fm = re.search(rf"{field}:\s*(0x[0-9A-Fa-f]+)", body)
            assert fm, f"{family}.{field} not found"
            offs.add(int(fm.group(1), 0))
        return offs

    f1x0 = model_offsets("F1X0")
    t1 = targets.load("gd32f1x0")
    named1 = set(t1.register_names["USART1_BASE"].keys())
    assert f1x0 <= named1, f"usart.rs F1X0 offsets missing from f1x0: {f1x0 - named1}"

    f10x = model_offsets("F10X")
    t0 = targets.load("gd32f10x")
    named0 = set(t0.register_names["USART1_BASE"].keys())
    assert f10x <= named0, f"usart.rs F10X offsets missing from f10x: {f10x - named0}"


def test_i2c_offsets_have_register_names():
    """M2 T6/T7: the I2C register offsets the source names must be in both tomls' I2C0_BASE table.

    The classic event-based I2C block is shared across families, so i2c.rs uses bare module consts
    (CTL0/CTL1/SADDR0/DATA/STAT0/STAT1/CKCFG/RT); both family targets must name them.
    """
    i2c_src = (repo_root() / "src" / "i2c.rs").read_text()
    offs = _source_offsets(
        i2c_src, ["CTL0", "CTL1", "SADDR0", "DATA", "STAT0", "STAT1", "CKCFG", "RT"]
    )
    assert offs == {0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x1C, 0x20}, offs
    for fam in ("gd32f1x0", "gd32f10x"):
        t = targets.load(fam)
        named = set(t.register_names["I2C0_BASE"].keys())
        assert offs <= named, f"{fam}: i2c.rs offsets missing from I2C0_BASE: {offs - named}"


def test_spi_offsets_have_register_names():
    """M2 T8/T9: the SPI register offsets the source names must be in both tomls' SPI0_BASE table.

    The SPI block is shared across families, so spi.rs uses bare module consts (CTL0/CTL1/STAT/
    DATA/I2SCTL); both family targets must name them.
    """
    spi_src = (repo_root() / "src" / "spi.rs").read_text()
    offs = _source_offsets(spi_src, ["CTL0", "CTL1", "STAT", "DATA", "I2SCTL"])
    assert offs == {0x00, 0x04, 0x08, 0x0C, 0x1C}, offs
    for fam in ("gd32f1x0", "gd32f10x"):
        t = targets.load(fam)
        named = set(t.register_names["SPI0_BASE"].keys())
        assert offs <= named, f"{fam}: spi.rs offsets missing from SPI0_BASE: {offs - named}"
