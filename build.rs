//! One job: tell the crate whether it is being compiled by a nightly `rustc`.
//!
//! The crate's boot-time functions carry `#[cfg_attr(runtime_hal_nightly, optimize(size))]` (see
//! `lib.rs`), which is the unstable `optimize_attribute` feature: the consumer that ships this HAL
//! (hoverboard-firmware) pins its whole `runtime-hal` package to `opt-level = 3` for the I2C burst
//! and the ISRs, and pays for it on every once-per-boot bring-up function compiled at that level,
//! about 4.5 KB of cold code in its image. The attribute puts those functions back on the size
//! pipeline inside the O3 crate. Its feature gate only exists on nightly, and this crate's own
//! checks stay on stable (`rust-toolchain.toml`), so the gate is a `cfg` this script derives from
//! the compiler's channel rather than a cargo feature a consumer would have to know to enable, and
//! a stable build compiles the crate exactly as before.
use std::process::Command;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(runtime_hal_nightly)");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=RUSTC");
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".into());
    let Ok(out) = Command::new(rustc).arg("-vV").output() else {
        return;
    };
    let text = String::from_utf8_lossy(&out.stdout);
    // `release: 1.99.0-nightly` on a nightly, `release: 1.96.0` on stable; a dev build of rustc
    // reports `-dev` and has the feature gates open too.
    let nightly = text
        .lines()
        .find_map(|l| l.strip_prefix("release: "))
        .is_some_and(|r| r.contains("nightly") || r.contains("dev"));
    if nightly {
        println!("cargo:rustc-cfg=runtime_hal_nightly");
    }
}
