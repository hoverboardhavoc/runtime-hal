//! Put the per-board `memory.x` on the linker search path.
//!
//! ONE source runs on either board (the chip is detected at runtime, not selected at compile time).
//! The `f103` / `f130` cargo feature now selects ONLY the linker MEMORY layout (RAM size: the F130C8
//! has 8 KiB, the F103C8 20 KiB), since that is a property of the flashed image, not of the code. The
//! matching `memory-fXXX.x` is copied to `OUT_DIR/memory.x`, which `cortex-m-rt`'s `link.x` includes.
//! Exactly one feature must be set. (A single image linked for the smallest RAM, 8 KiB, would run on
//! both; the two layouts are kept so each board can use its full RAM.)

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    let f103 = env::var_os("CARGO_FEATURE_F103").is_some();
    let f130 = env::var_os("CARGO_FEATURE_F130").is_some();

    let src = match (f103, f130) {
        (true, false) => "memory-f103.x",
        (false, true) => "memory-f130.x",
        (true, true) => panic!("enable exactly one of the `f103` / `f130` features, not both"),
        (false, false) => panic!("enable exactly one of the `f103` / `f130` features"),
    };

    fs::copy(src, out.join("memory.x")).unwrap_or_else(|e| panic!("copy {src}: {e}"));

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory-f103.x");
    println!("cargo:rerun-if-changed=memory-f130.x");
}
