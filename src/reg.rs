//! Typed-width register accessor.
//!
//! DECISIONS.md #2: internal access is a typed-width accessor over a runtime base address, with
//! the access width encoded in the type so a 32-bit write can never be confused with two 16-bit
//! writes (the testing spec is width-strict). Volatile read / modify / write, monomorphic, no
//! `dyn`. `Reg16` and `Reg32` are distinct types, not one generic, so a 16-bit and a 32-bit
//! access are not interchangeable at a call site.
//!
//! Under the normal build the accessor is real volatile MMIO. Under the `mock` (host-test)
//! feature it is backed by a shared backing array so host tests can read/write the "register
//! space" without hardware, while preserving width semantics byte-for-byte (a 32-bit write lands
//! as four little-endian bytes; reading the same span back as two `Reg16`s sees those bytes).

/// A 16-bit register at `base + offset`.
#[derive(Debug, Clone, Copy)]
pub struct Reg16 {
    addr: u32,
}

/// A 32-bit register at `base + offset`.
#[derive(Debug, Clone, Copy)]
pub struct Reg32 {
    addr: u32,
}

impl Reg16 {
    /// Construct an accessor for the 16-bit register at `base + offset`.
    #[inline]
    pub const fn new(base: u32, offset: u32) -> Self {
        Self {
            addr: base.wrapping_add(offset),
        }
    }

    /// The resolved absolute address.
    #[inline]
    pub const fn addr(self) -> u32 {
        self.addr
    }

    /// Volatile read.
    #[inline]
    pub fn read(self) -> u16 {
        backend::read16(self.addr)
    }

    /// Volatile write.
    #[inline]
    pub fn write(self, value: u16) {
        backend::write16(self.addr, value);
    }

    /// Read-modify-write: clear the bits in `mask`, then set the bits of `value` within `mask`.
    #[inline]
    pub fn modify(self, mask: u16, value: u16) {
        let cur = self.read();
        self.write((cur & !mask) | (value & mask));
    }
}

impl Reg32 {
    /// Construct an accessor for the 32-bit register at `base + offset`.
    #[inline]
    pub const fn new(base: u32, offset: u32) -> Self {
        Self {
            addr: base.wrapping_add(offset),
        }
    }

    /// The resolved absolute address.
    #[inline]
    pub const fn addr(self) -> u32 {
        self.addr
    }

    /// Volatile read.
    #[inline]
    pub fn read(self) -> u32 {
        backend::read32(self.addr)
    }

    /// Volatile write.
    #[inline]
    pub fn write(self, value: u32) {
        backend::write32(self.addr, value);
    }

    /// Read-modify-write: clear the bits in `mask`, then set the bits of `value` within `mask`.
    #[inline]
    pub fn modify(self, mask: u32, value: u32) {
        let cur = self.read();
        self.write((cur & !mask) | (value & mask));
    }
}

// --- Backends -----------------------------------------------------------------------------

/// Real volatile MMIO backend (normal build).
#[cfg(not(feature = "mock"))]
mod backend {
    use core::ptr::{read_volatile, write_volatile};

    #[inline]
    pub fn read16(addr: u32) -> u16 {
        // SAFETY: `addr` is a peripheral register address supplied by the validated descriptor.
        unsafe { read_volatile(addr as *const u16) }
    }
    #[inline]
    pub fn write16(addr: u32, value: u16) {
        // SAFETY: as above.
        unsafe { write_volatile(addr as *mut u16, value) }
    }
    #[inline]
    pub fn read32(addr: u32) -> u32 {
        // SAFETY: as above.
        unsafe { read_volatile(addr as *const u32) }
    }
    #[inline]
    pub fn write32(addr: u32, value: u32) {
        // SAFETY: as above.
        unsafe { write_volatile(addr as *mut u32, value) }
    }
}

/// Host-test backing-array backend (`mock` feature).
///
/// A single flat byte array stands in for the register space; addresses index into it modulo the
/// span. Accesses are little-endian (the GD32 is little-endian and the frame is LE per
/// DECISIONS.md #3), so width strictness is exact at the byte level: a `Reg32::write` lays down
/// four bytes that two `Reg16` reads then observe, and vice-versa. Each access takes a short mutex
/// so the array is internally consistent; a case that seeds the space and later asserts it holds
/// [`mock::lock`] for its whole duration so the cargo test runner's threads do not interleave a
/// [`mock::reset`] into the middle of it.
#[cfg(feature = "mock")]
pub mod backend {
    use std::sync::Mutex;

    /// Size of the simulated register window, in bytes. Generous for M1's handful of peripherals.
    pub const SPACE_BYTES: usize = 1 << 16;

    static SPACE: Mutex<[u8; SPACE_BYTES]> = Mutex::new([0u8; SPACE_BYTES]);

    /// Serializes whole test cases that seed-then-assert against the shared `SPACE`. The
    /// per-access `SPACE` lock is dropped between each `read`/`write`, so a case that seeds a
    /// register and later asserts it would otherwise race another case's [`mock::reset`]. A case
    /// holds [`mock::lock`] for its duration to make the seed/run/assert sequence atomic across
    /// the (multi-threaded by default) cargo test runner.
    static TEST_SERIAL: Mutex<()> = Mutex::new(());

    #[inline]
    fn idx(addr: u32) -> usize {
        (addr as usize) & (SPACE_BYTES - 1)
    }

    /// Read a 16-bit little-endian value from the mock register backing store.
    #[inline]
    pub fn read16(addr: u32) -> u16 {
        let s = SPACE.lock().unwrap();
        let i = idx(addr);
        u16::from_le_bytes([s[i], s[i + 1]])
    }
    /// Write a 16-bit little-endian value to the mock register backing store.
    #[inline]
    pub fn write16(addr: u32, value: u16) {
        let mut s = SPACE.lock().unwrap();
        let i = idx(addr);
        let b = value.to_le_bytes();
        s[i] = b[0];
        s[i + 1] = b[1];
    }
    /// Read a 32-bit little-endian value from the mock register backing store.
    #[inline]
    pub fn read32(addr: u32) -> u32 {
        let s = SPACE.lock().unwrap();
        let i = idx(addr);
        u32::from_le_bytes([s[i], s[i + 1], s[i + 2], s[i + 3]])
    }
    /// Write a 32-bit little-endian value to the mock register backing store.
    #[inline]
    pub fn write32(addr: u32, value: u32) {
        let mut s = SPACE.lock().unwrap();
        let i = idx(addr);
        let b = value.to_le_bytes();
        s[i] = b[0];
        s[i + 1] = b[1];
        s[i + 2] = b[2];
        s[i + 3] = b[3];
    }

    /// Test helpers for the backing array.
    pub mod mock {
        use super::{idx, SPACE, SPACE_BYTES, TEST_SERIAL};
        use std::sync::MutexGuard;

        /// Acquire the whole-case serialization lock. Hold the returned guard for the duration of
        /// a test that seeds the register space and later asserts it, so concurrent cases do not
        /// reset the space out from under it. Poison is ignored (a panicking case still releases
        /// the lock cleanly for the next).
        pub fn lock() -> MutexGuard<'static, ()> {
            TEST_SERIAL.lock().unwrap_or_else(|e| e.into_inner())
        }

        /// Zero the whole simulated register space (call at the start of a test case).
        pub fn reset() {
            let mut s = SPACE.lock().unwrap();
            *s = [0u8; SPACE_BYTES];
        }

        /// Peek a single byte (for assertions about width/endianness).
        pub fn peek_byte(addr: u32) -> u8 {
            let s = SPACE.lock().unwrap();
            s[idx(addr)]
        }
    }
}

/// Re-export of the mock test helpers when the backing-array backend is active.
#[cfg(feature = "mock")]
pub use backend::mock;
