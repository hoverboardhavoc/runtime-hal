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

/// Host-test backing-map backend (`mock` feature).
///
/// A sparse byte map stands in for the FULL 32-bit register space, keyed by the exact address, so
/// distinct peripherals can NEVER alias (the old fixed 64 KiB array indexed by `addr & 0xFFFF`
/// collapsed e.g. F1x0 GPIOA `0x4800_0000`, the DMA `INTF` `0x4002_0000`, and flash `0x0800_0000`
/// onto one index; debt-paydown slice 10). Unwritten addresses read 0, matching the old zeroed
/// array. Accesses are little-endian (the GD32 is little-endian), so width strictness is exact at
/// the byte level: a `Reg32::write` lays down four bytes that two `Reg16` reads then observe, and
/// vice-versa. Each access takes a short mutex so the map is internally consistent; a case that
/// seeds the space and later asserts it holds [`mock::lock`] for its whole duration so the cargo
/// test runner's threads do not interleave a [`mock::reset`] into the middle of it.
///
/// The backend models NO silicon behavior of its own: device side effects (a read-clears-flag
/// register, a write-1-to-clear pair) exist only as RULES a test harness registers
/// ([`mock::read_clears`] / [`mock::w1c_pair`]), so the code under test never manufactures the
/// behavior its tests observe (the old in-driver `cfg(mock)` side effects are gone).
#[cfg(feature = "mock")]
pub mod backend {
    use std::collections::BTreeMap;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Mutex;

    static SPACE: Mutex<BTreeMap<u32, u8>> = Mutex::new(BTreeMap::new());

    /// A single "non-responsive" address whose writes are dropped, modelling a register that does
    /// not stick (a disabled-clock / wrong-base channel). `u64::MAX` = none. Used by the DMA-ring
    /// write-back self-check test (host case B11); reset clears it.
    static FROZEN: AtomicU64 = AtomicU64::new(u64::MAX);

    /// Test-registered read side effects: `(read_addr, clear_addr, mask)` - a read at `read_addr`
    /// clears `mask` at `clear_addr` (e.g. the GD32 USART's data-register read clearing
    /// `STAT.RBNE`). Registered by the test harness that stages the device state; cleared by
    /// [`mock::reset`].
    static READ_CLEARS: Mutex<Vec<(u32, u32, u32)>> = Mutex::new(Vec::new());

    /// Test-registered write-1-to-clear pairs: `(w1c_addr, target_addr)` - writing `v` to
    /// `w1c_addr` clears bits `v` at `target_addr` (e.g. the GD32 DMA `INTC` clearing `INTF`).
    static W1C_PAIRS: Mutex<Vec<(u32, u32)>> = Mutex::new(Vec::new());

    #[inline]
    fn get(map: &BTreeMap<u32, u8>, addr: u32) -> u8 {
        *map.get(&addr).unwrap_or(&0)
    }
    #[inline]
    fn put(map: &mut BTreeMap<u32, u8>, addr: u32, b: u8) {
        map.insert(addr, b);
    }
    /// Apply the registered read side effects for `addr` (rules lock taken BEFORE the space lock,
    /// the fixed order every path uses).
    fn apply_read_rules(map: &mut BTreeMap<u32, u8>, rules: &[(u32, u32, u32)], addr: u32) {
        for &(read_addr, clear_addr, mask) in rules {
            if read_addr == addr {
                for i in 0..4u32 {
                    let m = (mask >> (8 * i)) as u8;
                    if m != 0 {
                        let cur = get(map, clear_addr + i);
                        put(map, clear_addr + i, cur & !m);
                    }
                }
            }
        }
    }
    /// Apply the registered W1C pairs for a write of `value` at `addr`.
    fn apply_w1c_rules(map: &mut BTreeMap<u32, u8>, pairs: &[(u32, u32)], addr: u32, value: u32) {
        for &(w1c_addr, target) in pairs {
            if w1c_addr == addr {
                for i in 0..4u32 {
                    let m = (value >> (8 * i)) as u8;
                    if m != 0 {
                        let cur = get(map, target + i);
                        put(map, target + i, cur & !m);
                    }
                }
            }
        }
    }

    /// Serializes whole test cases that seed-then-assert against the shared `SPACE`. The
    /// per-access `SPACE` lock is dropped between each `read`/`write`, so a case that seeds a
    /// register and later asserts it would otherwise race another case's [`mock::reset`]. A case
    /// holds [`mock::lock`] for its duration to make the seed/run/assert sequence atomic across
    /// the (multi-threaded by default) cargo test runner.
    static TEST_SERIAL: Mutex<()> = Mutex::new(());

    /// Read a 16-bit little-endian value from the mock register backing store.
    #[inline]
    pub fn read16(addr: u32) -> u16 {
        let rules = READ_CLEARS.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
        let v = u16::from_le_bytes([get(&s, addr), get(&s, addr + 1)]);
        apply_read_rules(&mut s, &rules, addr);
        v
    }
    /// Write a 16-bit little-endian value to the mock register backing store.
    #[inline]
    pub fn write16(addr: u32, value: u16) {
        if addr as u64 == FROZEN.load(Ordering::Relaxed) {
            return; // non-responsive register: the write does not stick
        }
        let pairs = W1C_PAIRS.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
        let b = value.to_le_bytes();
        put(&mut s, addr, b[0]);
        put(&mut s, addr + 1, b[1]);
        apply_w1c_rules(&mut s, &pairs, addr, value as u32);
    }
    /// Read a 32-bit little-endian value from the mock register backing store.
    #[inline]
    pub fn read32(addr: u32) -> u32 {
        let rules = READ_CLEARS.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
        let v = u32::from_le_bytes([
            get(&s, addr),
            get(&s, addr + 1),
            get(&s, addr + 2),
            get(&s, addr + 3),
        ]);
        apply_read_rules(&mut s, &rules, addr);
        v
    }
    /// Write a 32-bit little-endian value to the mock register backing store.
    #[inline]
    pub fn write32(addr: u32, value: u32) {
        if addr as u64 == FROZEN.load(Ordering::Relaxed) {
            return; // non-responsive register: the write does not stick
        }
        let pairs = W1C_PAIRS.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
        let b = value.to_le_bytes();
        put(&mut s, addr, b[0]);
        put(&mut s, addr + 1, b[1]);
        put(&mut s, addr + 2, b[2]);
        put(&mut s, addr + 3, b[3]);
        apply_w1c_rules(&mut s, &pairs, addr, value);
    }

    /// Test helpers for the backing map.
    pub mod mock {
        use super::{FROZEN, READ_CLEARS, SPACE, TEST_SERIAL, W1C_PAIRS};
        use std::sync::atomic::Ordering;
        use std::sync::MutexGuard;

        /// Mark the word at `addr` as non-responsive: subsequent writes to it are dropped (reads see
        /// the prior value, default 0), modelling a register that does not stick. Cleared by
        /// [`reset`]. One frozen word at a time (the DMA self-check test needs exactly one).
        pub fn freeze(addr: u32) {
            FROZEN.store(addr as u64, Ordering::Relaxed);
        }

        /// Register a device READ side effect: a read at `read_addr` clears `mask` at `clear_addr`
        /// (e.g. the GD32 USART data-register read clearing `STAT.RBNE`). The test harness that
        /// stages device state declares the device behavior; the drivers under test never
        /// manufacture it. Cleared by [`reset`].
        pub fn read_clears(read_addr: u32, clear_addr: u32, mask: u32) {
            let mut r = READ_CLEARS.lock().unwrap_or_else(|e| e.into_inner());
            if !r.contains(&(read_addr, clear_addr, mask)) {
                r.push((read_addr, clear_addr, mask));
            }
        }

        /// Register a write-1-to-clear pair: writing `v` to `w1c_addr` clears bits `v` at
        /// `target_addr` (e.g. the GD32 DMA `INTC` clearing `INTF`). Cleared by [`reset`].
        pub fn w1c_pair(w1c_addr: u32, target_addr: u32) {
            let mut p = W1C_PAIRS.lock().unwrap_or_else(|e| e.into_inner());
            if !p.contains(&(w1c_addr, target_addr)) {
                p.push((w1c_addr, target_addr));
            }
        }

        /// Acquire the whole-case serialization lock. Hold the returned guard for the duration of
        /// a test that seeds the register space and later asserts it, so concurrent cases do not
        /// reset the space out from under it. Poison is ignored (a panicking case still releases
        /// the lock cleanly for the next).
        pub fn lock() -> MutexGuard<'static, ()> {
            TEST_SERIAL.lock().unwrap_or_else(|e| e.into_inner())
        }

        /// Zero the whole simulated register space (call at the start of a test case). Also clears
        /// any frozen (non-responsive) word and every registered device rule.
        pub fn reset() {
            FROZEN.store(u64::MAX, Ordering::Relaxed);
            READ_CLEARS
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clear();
            W1C_PAIRS.lock().unwrap_or_else(|e| e.into_inner()).clear();
            let mut s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
            s.clear();
        }

        /// Peek a single byte (for assertions about width/endianness).
        pub fn peek_byte(addr: u32) -> u8 {
            let s = SPACE.lock().unwrap_or_else(|e| e.into_inner());
            *s.get(&addr).unwrap_or(&0)
        }
    }
}

/// Re-export of the mock test helpers when the backing-array backend is active.
#[cfg(feature = "mock")]
pub use backend::mock;
