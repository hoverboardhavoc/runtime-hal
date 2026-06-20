"""Task 5 validation: the vector loader parses the GPIO vector and exposes both
implementations with the right target/mode.
"""

from __future__ import annotations

from regcmp import vectors


def test_load_gpio_af_vector():
    vec = vectors.find("gpio_af_usart1_tx_pa2_f1x0")
    assert vec.vector_id == "gpio_af_usart1_tx_pa2_f1x0"
    # Post-refactor: the only public USART pin-AF path is Usart::new (routes both pins), so the
    # vector is final_state scoped to the GPIOA window with assert_only (AFSEL1 +0x24 excluded).
    assert vec.mode == "final_state"
    assert vec.assert_only
    assert not vec.ignore

    spl = vec.impl_for("gd-spl", "gd32f1x0")
    assert spl.is_spl and spl.target == "gd32f1x0"
    assert "gd32f1x0_gpio.h" in spl.includes
    assert "gpio_mode_set" in spl.body
    assert "gpio_af_set" in spl.body

    rh = vec.impl_for("runtime-hal", "gd32f1x0")
    assert rh.is_runtime_hal and rh.target == "gd32f1x0"
    assert "Usart::new" in rh.body
    assert "pub fn body()" in rh.body


def test_mutually_exclusive_filters(tmp_path):
    import pytest
    bad = tmp_path / "gpio" / "bad.yaml"
    bad.parent.mkdir(parents=True)
    bad.write_text(
        "name: bad\nmode: register_writes\n"
        "assert_only: ['<GPIOA_BASE>']\nignore: ['<GPIOA_BASE>']\n"
        "implementations:\n  gd-spl/gd32f1x0:\n    body: 'x;'\n"
    )
    with pytest.raises(ValueError):
        vectors.load(bad)
