//! no_std thumbv7m snippet staticlib for the regcmp harness.
//!
//! Exposes `regcmp_test`, a single runtime-hal path-function call generated per
//! vector into `snippet_body.rs`. Built with the real MMIO backend so the Reg
//! writes hit Unicorn-backed memory. The harness links this staticlib with the
//! vendored startup.S (entry at regcmp_test, LR = sentinel) + link.ld
//! (KEEP(regcmp_test)); SystemInit/main never run.

#![no_std]

use core::panic::PanicInfo;

// The generated body. build_rusthal.py writes src/snippet_body.rs with a
// function `pub fn body()` containing the path-fn call. Kept in a separate file
// so the template lib.rs is stable and only the body is swapped per vector.
mod snippet_body;

/// The harness entry. `#[no_mangle]` gives it external linkage in the staticlib
/// archive, and the link script `KEEP`s `.text.regcmp_test` so --gc-sections
/// cannot drop it. `extern "C"` gives the plain symbol the extractor enters at.
#[no_mangle]
pub extern "C" fn regcmp_test() {
    snippet_body::body();
}

/// `#[used]` static referencing the entry, so even aggressive dead-code passes
/// keep `regcmp_test` reachable from the staticlib's retained roots.
#[used]
static KEEP_REGCMP_TEST: extern "C" fn() = regcmp_test;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
