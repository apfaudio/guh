//! Blocking rust-fatfs implementation (see also: `rs/README.md`).

use fatfs::{IoBase, IoError, Read as FatfsRead, Write as FatfsWrite, Seek as FatfsSeek, SeekFrom};
use log::warn;

use guh_dma::DmaBuf;
use crate::usb_msc::UsbMsc;

#[derive(Debug, Clone, Copy)]
pub struct BlockDeviceError;

impl core::fmt::Display for BlockDeviceError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "BlockDeviceError")
    }
}

impl IoError for BlockDeviceError {
    fn is_interrupted(&self) -> bool { false }
    fn new_unexpected_eof_error() -> Self { BlockDeviceError }
    fn new_write_zero_error() -> Self { BlockDeviceError }
}

struct Window {
    buf: &'static DmaBuf,
    /// Device LBA the window starts at, or `None` when it holds nothing valid.
    start_lba: Option<u32>,
    dirty: bool,
}

/// Ways in the window cache, i.e. how many disjoint regions stay resident.
///
/// Two, not one: FAT mirroring alternates between the FAT copies on every
/// entry write, and a single window would pay a flush *and* a reload per
/// 4-byte FAT entry.
const CACHE_WAYS: usize = 2;

pub struct UsbMscBlockDevice<'a, M: UsbMsc> {
    usb_msc: &'a mut M,
    windows: [Window; CACHE_WAYS],
    /// Most recently used window; the other one is the eviction victim.
    recent: usize,
    blocks_per_read: u32,
    position: u64,
    size: u64,
    block_size: u32,
}

impl<'a, M: UsbMsc> UsbMscBlockDevice<'a, M> {
    /// `usb_msc` is read from its own LBA 0; pass a
    /// [`crate::usb_msc::PartitionView`] to mount a partition instead.
    ///
    /// Writes are write-back cached in the same windows as reads: a dirty
    /// window goes to the device on eviction or `flush()`. One buffer per
    /// window, all the same length; fails unless every window is a whole
    /// number of blocks.
    pub fn new(
        usb_msc: &'a mut M,
        cache: [&'static DmaBuf; CACHE_WAYS],
    ) -> Result<Self, BlockDeviceError> {
        let block_size = usb_msc.block_size();
        let capacity = usb_msc.capacity();
        let win_bytes = cache[0].len();
        if block_size == 0
            || win_bytes < block_size as usize
            || win_bytes % block_size as usize != 0
            || cache.iter().any(|b| b.len() != win_bytes)
        {
            warn!("block device: {}-byte windows do not hold whole {}-byte blocks",
                  win_bytes, block_size);
            return Err(BlockDeviceError);
        }
        Ok(Self {
            usb_msc,
            windows: cache.map(|buf| Window { buf, start_lba: None, dirty: false }),
            recent: 0,
            blocks_per_read: win_bytes as u32 / block_size,
            position: 0,
            size: capacity as u64 * block_size as u64,
            block_size,
        })
    }

    /// Index of the window holding `lba`, loading it over the least recently
    /// used one if neither has it.
    fn ensure_buffer_contains(&mut self, lba: u32) -> Result<usize, BlockDeviceError> {
        let bpr = self.blocks_per_read;
        let aligned_lba = (lba / bpr) * bpr;
        for i in 0..CACHE_WAYS {
            if self.windows[i].start_lba == Some(aligned_lba) {
                self.recent = i;
                return Ok(i);
            }
        }

        let victim = (self.recent + 1) % CACHE_WAYS;
        self.flush_window(victim)?;
        let buf = self.windows[victim].buf;
        if self.usb_msc.read_blocks_blocking(aligned_lba, buf).is_err() {
            // A failed transfer still clobbers part of the buffer, so the
            // window it used to hold is gone too - drop it, or later access
            // would be served from a cache that no longer contains it.
            self.windows[victim].start_lba = None;
            return Err(BlockDeviceError);
        }
        self.windows[victim].start_lba = Some(aligned_lba);
        self.recent = victim;
        Ok(victim)
    }

    /// Resolve the current position against the cache: which window holds it,
    /// the byte offset within that window, and how much of `remaining` can be
    /// transferred before running off its end.
    fn window_at_position(
        &mut self, remaining: usize,
    ) -> Result<(usize, usize, usize), BlockDeviceError> {
        let block_size = self.block_size as usize;
        let current_lba = (self.position / self.block_size as u64) as u32;
        let offset_in_block = (self.position % self.block_size as u64) as usize;

        let w = self.ensure_buffer_contains(current_lba)?;

        let bpr = self.blocks_per_read;
        let aligned_lba = (current_lba / bpr) * bpr;
        let offset_in_buffer =
            (current_lba - aligned_lba) as usize * block_size + offset_in_block;
        let to_copy = remaining.min(self.windows[w].buf.len() - offset_in_buffer);
        Ok((w, offset_in_buffer, to_copy))
    }

    /// Write window `i` back to the device if it has pending writes.
    fn flush_window(&mut self, i: usize) -> Result<(), BlockDeviceError> {
        let Some(lba) = self.windows[i].start_lba.filter(|_| self.windows[i].dirty) else {
            return Ok(());
        };
        let buf = self.windows[i].buf;
        self.usb_msc.write_blocks_blocking(lba, buf)
            .map_err(|_| BlockDeviceError)?;
        self.windows[i].dirty = false;
        Ok(())
    }

    fn flush_all(&mut self) -> Result<(), BlockDeviceError> {
        let mut result = Ok(());
        for i in 0..CACHE_WAYS {
            // Keep going after a failure so one bad window cannot strand the
            // other's writes; the failed one stays dirty for a retry.
            if self.flush_window(i).is_err() {
                result = Err(BlockDeviceError);
            }
        }
        result
    }
}

/// Best-effort write-back on drop: fatfs's own `FileSystem::drop` writes
/// FSInfo / dirty-flag sectors *after* the last explicit `flush()`, and those
/// land in our cached windows just before the device is dropped.
impl<M: UsbMsc> Drop for UsbMscBlockDevice<'_, M> {
    fn drop(&mut self) {
        let _ = self.flush_all();
    }
}

impl<M: UsbMsc> IoBase for UsbMscBlockDevice<'_, M> {
    type Error = BlockDeviceError;
}

impl<M: UsbMsc> FatfsRead for UsbMscBlockDevice<'_, M> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() || self.position >= self.size {
            return Ok(0);
        }

        let mut bytes_read = 0usize;
        let mut remaining = buf.len().min((self.size - self.position) as usize);

        while remaining > 0 {
            let (w, offset_in_buffer, to_copy) = self.window_at_position(remaining)?;

            self.windows[w].buf.read_into(offset_in_buffer,
                                          &mut buf[bytes_read..bytes_read + to_copy]);

            bytes_read += to_copy;
            remaining -= to_copy;
            self.position += to_copy as u64;
        }

        Ok(bytes_read)
    }
}

impl<M: UsbMsc> FatfsWrite for UsbMscBlockDevice<'_, M> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() || self.position >= self.size {
            return Ok(0);
        }

        let mut bytes_written = 0usize;
        let mut remaining = buf.len().min((self.size - self.position) as usize);

        while remaining > 0 {
            // Read-modify-write: the window is loaded first, so untouched
            // bytes survive the write-back.
            // TODO: skip the load when the write covers the whole window.
            let (w, offset_in_buffer, to_copy) = self.window_at_position(remaining)?;

            self.windows[w].buf.write_from(offset_in_buffer,
                                           &buf[bytes_written..bytes_written + to_copy]);
            self.windows[w].dirty = true;

            bytes_written += to_copy;
            remaining -= to_copy;
            self.position += to_copy as u64;
        }

        Ok(bytes_written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_all()
    }
}

impl<M: UsbMsc> FatfsSeek for UsbMscBlockDevice<'_, M> {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        let new_pos = match pos {
            SeekFrom::Start(offset)   => offset as i64,
            SeekFrom::End(offset)     => self.size as i64 + offset,
            SeekFrom::Current(offset) => self.position as i64 + offset,
        };
        if new_pos < 0 {
            return Err(BlockDeviceError);
        }
        self.position = new_pos as u64;
        Ok(self.position)
    }
}
