//! Pinned DMA-target/source buffers in PSRAM.
//!
//! A `&DmaBuf` may be held while DMA is in flight: contents sit behind
//! `UnsafeCell` and all CPU access is volatile. The dcache is *not* assumed
//! coherent with DMA: [`DmaBuf::invalidate`] after a DMA write,
//! [`DmaBuf::clean`] before a DMA read of CPU-written contents.

#![cfg_attr(not(test), no_std)]

use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering};

/// HACK: this should be customizable, even though it's true for pretty
/// much every realistic platform
pub const CACHE_LINE_BYTES: usize = 64;

/// HACK: we want to use the rv32im target on stable rust, where
/// #[target_feature] is not supported for zicbom, even though our
/// custom Vexii core has it. So we sneakily pretend it exists :)
#[cfg(target_arch = "riscv32")]
macro_rules! cbo_lines {
    ($op:literal, $base:expr, $len:expr) => {
        for i in 0..$len.div_ceil(CACHE_LINE_BYTES) {
            let line = unsafe { $base.add(i * CACHE_LINE_BYTES) };
            unsafe {
                core::arch::asm!(
                    ".option push",
                    ".option arch, +zicbom",
                    concat!($op, " ({line})"),
                    ".option pop",
                    line = in(reg) line,
                    options(nostack, preserves_flags),
                );
            }
        }
    };
}

#[repr(transparent)]
pub struct DmaBuf([UnsafeCell<u8>]);

// SAFETY: single-core; all CPU access is volatile and DMA completion is
// observed via the engine's sequence counters before written data is read.
unsafe impl Sync for DmaBuf {}

impl DmaBuf {
    /// # Safety
    /// - `ptr..ptr+len` must be valid for reads/writes for `'a` and unaliased
    ///   by any other reference.
    /// - `ptr` must be [`CACHE_LINE_BYTES`]-aligned.
    pub unsafe fn from_raw_parts<'a>(ptr: *mut u8, len: usize) -> &'a DmaBuf {
        debug_assert_eq!(ptr as usize % CACHE_LINE_BYTES, 0);
        unsafe {
            &*(core::ptr::slice_from_raw_parts(ptr as *const UnsafeCell<u8>, len)
                as *const DmaBuf)
        }
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn as_ptr(&self) -> *mut u8 {
        self.0.as_ptr() as *mut u8
    }

    pub fn psram_offset(&self, psram_base: u32) -> u32 {
        (self.as_ptr() as u32).wrapping_sub(psram_base)
    }

    pub fn slice(&self, offset: usize, len: usize) -> &DmaBuf {
        assert!(offset.checked_add(len).is_some_and(|end| end <= self.len()));
        unsafe {
            &*(core::ptr::slice_from_raw_parts(
                self.0.as_ptr().add(offset), len) as *const DmaBuf)
        }
    }

    pub fn read_byte(&self, offset: usize) -> u8 {
        unsafe { self.0[offset].get().read_volatile() }
    }

    pub fn read_into(&self, offset: usize, dst: &mut [u8]) {
        assert!(offset.checked_add(dst.len()).is_some_and(|end| end <= self.len()));
        for (i, d) in dst.iter_mut().enumerate() {
            *d = unsafe { self.0[offset + i].get().read_volatile() };
        }
    }

    pub fn write_byte(&self, offset: usize, value: u8) {
        unsafe { self.0[offset].get().write_volatile(value) }
    }

    pub fn write_from(&self, offset: usize, src: &[u8]) {
        assert!(offset.checked_add(src.len()).is_some_and(|end| end <= self.len()));
        for (i, s) in src.iter().enumerate() {
            unsafe { self.0[offset + i].get().write_volatile(*s) };
        }
    }

    /// # Safety
    /// No DMA may target this buffer while the slice lives, and
    /// [`Self::invalidate`] must have run since the last DMA write.
    pub unsafe fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.as_ptr() as *const u8, self.len()) }
    }

    /// Drop stale dcache lines covering this buffer after a DMA write, so
    /// subsequent reads see DMA-written memory (the `Acquire` fence stops
    /// later reads being hoisted above observing the write done).
    pub fn invalidate(&self) {
        debug_assert_eq!(self.as_ptr() as usize % CACHE_LINE_BYTES, 0);
        #[cfg(target_arch = "riscv32")]
        cbo_lines!("cbo.inval", self.as_ptr(), self.len());
        compiler_fence(Ordering::Acquire);
    }

    /// Write back dirty dcache lines covering this buffer, so a DMA engine
    /// reading PSRAM afterwards observes CPU-written contents. Call after
    /// filling the buffer, before submitting a DMA read of it.
    pub fn clean(&self) {
        debug_assert_eq!(self.as_ptr() as usize % CACHE_LINE_BYTES, 0);
        compiler_fence(Ordering::Release);
        #[cfg(target_arch = "riscv32")]
        cbo_lines!("cbo.clean", self.as_ptr(), self.len());
    }
}
