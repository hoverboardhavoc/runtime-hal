"""regcmp: in-repo golden-comparison harness for runtime-hal.

Traces a thumbv7m snippet's MMIO accesses under Unicorn and diffs runtime-hal
against the GigaDevice SPL golden for the same logical configuration. The GD SPL
is the golden oracle. No dependency on any external trace tool; the design is a
fresh implementation of the register-trace comparison technique.
"""

__version__ = "0.1.0"
