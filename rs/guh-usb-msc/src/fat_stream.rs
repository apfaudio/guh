//! Non-blocking file streaming utilities (see also: `rs/README.md`).

use fatfs::{ChainMap, FatType, FsGeometry, MapNext};
use log::{info, warn};

use guh_dma::DmaBuf;
use crate::usb_msc::{UsbMsc, XferError, BLOCK_BYTES};

#[derive(Debug, Clone, Copy)]
pub enum OpenError {
    UnsupportedVolume,
    BadAlignment,
    StartCluster,
}

#[derive(Debug, Clone, Copy)]
pub enum StreamError {
    UsbError,
    EndOfChain,
    Overrun,
}

pub struct StreamConfig {
    pub max_chunk_blocks: u32,
    pub ring: &'static DmaBuf,
    pub fat_cache: &'static DmaBuf,
}

enum Lookup {
    Lba(u32),
    Pending,
    OutOfChain,
}

struct StreamGeometry {
    chunk_blocks: u32,
    n_blocks: u32,
    ring: &'static DmaBuf,
}

impl StreamGeometry {
    fn ring_len(&self) -> u64 {
        self.ring.len() as u64
    }

    fn chunk_bytes(&self) -> u32 {
        self.chunk_blocks * BLOCK_BYTES as u32
    }

    fn total_bytes(&self) -> u64 {
        self.n_blocks as u64 * BLOCK_BYTES as u64
    }

    fn advance_wrapped(&self, logical: u64, cur: u32) -> u64 {
        let ring = self.ring_len();
        let last = logical % ring;
        logical + (cur as u64 + ring - last) % ring
    }
}

enum FatWindow {
    Empty,
    Filling { first_sector: u32, seq: u32 },
    Valid { first_sector: u32 },
}

struct ClusterMap {
    fat_cache: &'static DmaBuf,

    map: ChainMap,
    fat_window: FatWindow,
}

impl ClusterMap {
    fn new(fat_cache: &'static DmaBuf, map: ChainMap) -> Self {
        Self {
            fat_cache,
            map,
            fat_window: FatWindow::Empty,
        }
    }

    fn geometry(&self) -> FsGeometry {
        self.map.geometry()
    }

    fn gulp_blocks(&self) -> u32 {
        (self.fat_cache.len() / BLOCK_BYTES) as u32
    }

    fn poll_window<M: UsbMsc>(&mut self, msc: &mut M) -> bool {
        if let FatWindow::Filling { first_sector, seq } = self.fat_window {
            if !msc.seq_done(seq) {
                return false;
            }
            self.fat_cache.invalidate();
            self.fat_window = FatWindow::Valid { first_sector };
        }
        true
    }

    fn wait_progress<M: UsbMsc>(&self, msc: &M) -> Result<(), StreamError> {
        match self.fat_window {
            FatWindow::Filling { seq, .. } => msc.wait_seq(seq)?,
            _ => msc.wait_idle()?,
        }
        Ok(())
    }

    fn lba_of<M: UsbMsc>(&mut self, msc: &mut M, block: u32) -> Lookup {
        let (window_first, window): (u32, &[u8]) = match self.fat_window {
            // SAFETY: `Valid` is only entered after the gulp's DMA completed
            // and `invalidate()` ran; the engine only writes this while
            // `Filling`.
            FatWindow::Valid { first_sector } =>
                (first_sector, unsafe { self.fat_cache.as_slice() }),
            _ => (0, &[]),
        };
        match self.map.sector_of(block, window_first, window) {
            MapNext::Sector(lba) => Lookup::Lba(lba),
            MapNext::NeedSector(sector) => {
                let gulp_sector = self.geometry().fat_window_start(sector, self.gulp_blocks());
                if let Some(seq) = msc.read_blocks(gulp_sector, self.fat_cache) {
                    self.fat_window = FatWindow::Filling {
                        first_sector: gulp_sector, seq };
                }
                Lookup::Pending
            }
            MapNext::OutOfChain => {
                warn!("fat stream: chain ended before range end (block {})", block);
                Lookup::OutOfChain
            }
        }
    }
}

impl From<XferError> for StreamError {
    fn from(_: XferError) -> Self { StreamError::UsbError }
}

struct ErrorLatch {
    baseline: Option<u32>,
    plugs: Option<u32>,
    error: Option<StreamError>,
}

impl ErrorLatch {
    fn new() -> Self {
        Self { baseline: None, plugs: None, error: None }
    }

    fn rebaseline<M: UsbMsc>(&mut self, msc: &M) {
        self.baseline = Some(msc.error_count());
        self.plugs = Some(msc.plug_events());
    }

    fn check<M: UsbMsc>(&mut self, msc: &M) -> Result<(), StreamError> {
        if let Some(e) = self.error {
            return Err(e);
        }
        let baseline = *self.baseline.get_or_insert_with(|| msc.error_count());
        if msc.error_count() != baseline {
            warn!("fat stream: USB MSC transfer error");
            return Err(self.fail(StreamError::UsbError));
        }
        // Catches a disconnect that re-enumerated between polls, which the
        // spin-side liveness checks can miss entirely.
        let plugs = *self.plugs.get_or_insert_with(|| msc.plug_events());
        if msc.plug_events() != plugs {
            warn!("fat stream: USB MSC device went away");
            return Err(self.fail(StreamError::UsbError));
        }
        Ok(())
    }

    fn fail(&mut self, e: StreamError) -> StreamError {
        self.error = Some(e);
        e
    }
}

fn resolve<IO, TP, OCC>(
    file: &fatfs::File<'_, IO, TP, OCC>,
    cfg: &StreamConfig,
) -> Result<(StreamGeometry, ClusterMap), OpenError>
where
    IO: fatfs::ReadWriteSeek,
    TP: fatfs::TimeProvider,
{
    let geo = file.geometry();
    if geo.fat_type != FatType::Fat32 {
        warn!("FatStream: only FAT32 volumes are supported");
        return Err(OpenError::UnsupportedVolume);
    }
    if geo.bytes_per_sector != BLOCK_BYTES as u32 {
        warn!("FatStream: sector size {} unsupported (need {})",
              geo.bytes_per_sector, BLOCK_BYTES);
        return Err(OpenError::UnsupportedVolume);
    }
    let cluster_blocks = geo.sectors_per_cluster;
    let chunk = cluster_blocks.min(cfg.max_chunk_blocks);
    if chunk == 0 || cluster_blocks % chunk != 0 {
        warn!("FatStream: chunk_blocks={} does not divide cluster_blocks={}",
              chunk, cluster_blocks);
        return Err(OpenError::BadAlignment);
    }
    let chunk_bytes = chunk as usize * BLOCK_BYTES;
    if cfg.fat_cache.is_empty() || cfg.fat_cache.len() % BLOCK_BYTES != 0 {
        warn!("FatStream: FAT cache size {} not a non-zero multiple of {}",
              cfg.fat_cache.len(), BLOCK_BYTES);
        return Err(OpenError::BadAlignment);
    }
    if cfg.ring.is_empty() || cfg.ring.len() % chunk_bytes != 0 {
        warn!("FatStream: ring size {} not a non-zero multiple of chunk bytes {}",
              cfg.ring.len(), chunk_bytes);
        return Err(OpenError::BadAlignment);
    }

    // Rounded *up* to a whole number of chunks - `chunk` divides
    // `cluster_blocks`, so this never leaves the allocated chain. Streaming
    // the slack past EOF avoids a short final command.
    let file_size = file.size().ok_or(OpenError::StartCluster)? as u64;
    let file_blocks = file_size.div_ceil(BLOCK_BYTES as u64) as u32;
    let n_blocks = file_blocks.div_ceil(chunk) * chunk;

    let map = file.chain_map(0)
        .map_err(|_| OpenError::StartCluster)?
        .ok_or(OpenError::StartCluster)?;

    info!("stream open: file_size={} cluster_blocks={} chunk={} n_blocks={}",
          file_size, cluster_blocks, chunk, n_blocks);

    Ok((
        StreamGeometry { chunk_blocks: chunk, n_blocks, ring: cfg.ring },
        ClusterMap::new(cfg.fat_cache, map),
    ))
}

pub struct FatStream {
    geo: StreamGeometry,
    map: ClusterMap,

    logical_write: u64,
    logical_read: u64,
    underruns: u32,

    errors: ErrorLatch,
}

impl FatStream {
    pub fn open<IO, TP, OCC>(
        file: &fatfs::File<'_, IO, TP, OCC>,
        cfg: &StreamConfig,
    ) -> Result<Self, OpenError>
    where
        IO: fatfs::ReadWriteSeek,
        TP: fatfs::TimeProvider,
    {
        let (geo, map) = resolve(file, cfg)?;
        Ok(Self {
            geo,
            map,
            logical_write: 0,
            logical_read: 0,
            underruns: 0,
            errors: ErrorLatch::new(),
        })
    }

    pub fn prefill<M: UsbMsc>(&mut self, msc: &mut M) -> Result<(), StreamError> {
        msc.wait_idle()?;
        self.errors.rebaseline(msc);

        while self.logical_write < self.geo.ring_len() {
            let before = self.logical_write;
            self.tick(msc, 0)?;
            if self.logical_write == before {
                self.map.wait_progress(msc)?;
            }
        }
        msc.wait_idle()?;
        self.errors.check(msc)?;
        Ok(())
    }

    pub fn bytes_submitted(&self) -> u64 {
        self.logical_write
    }

    pub fn bytes_total(&self) -> u64 {
        self.geo.total_bytes()
    }

    pub fn underruns(&self) -> u32 {
        self.underruns
    }

    fn update_read_position(&mut self, cur_read: u32) {
        let ring = self.geo.ring_len();
        self.logical_read = self.geo.advance_wrapped(self.logical_read, cur_read);
        if self.logical_read > self.logical_write {
            self.underruns = self.underruns.saturating_add(1);
        }
        debug_assert!(self.logical_write <= self.logical_read + ring);
    }

    pub fn tick<M: UsbMsc>(&mut self, msc: &mut M, read_pos: u32) -> Result<(), StreamError> {
        self.errors.check(msc)?;
        self.update_read_position(read_pos);

        if !self.map.poll_window(msc) {
            return Ok(());
        }

        loop {
            // Saturating: an underrun must read as an empty ring, not a u64
            // underflow that wedges this check.
            let fill = self.logical_write.saturating_sub(self.logical_read);
            if fill + self.geo.chunk_bytes() as u64 > self.geo.ring_len() {
                break;
            }

            // n_blocks is a multiple of chunk_blocks, so the wrap lands on a
            // chunk boundary.
            let block = ((self.logical_write / BLOCK_BYTES as u64)
                         % self.geo.n_blocks as u64) as u32;
            match self.map.lba_of(msc, block) {
                Lookup::Lba(lba) => {
                    let ring_off = (self.logical_write % self.geo.ring_len()) as usize;
                    let dst = self.geo.ring.slice(
                        ring_off, (self.geo.chunk_blocks as usize) * BLOCK_BYTES);
                    if msc.read_blocks(lba, dst).is_none() {
                        break;
                    }
                    self.logical_write += self.geo.chunk_bytes() as u64;
                }
                Lookup::Pending => break,
                Lookup::OutOfChain =>
                    return Err(self.errors.fail(StreamError::EndOfChain)),
            }
        }
        Ok(())
    }
}

/// Write-direction dual of [`FatStream`]: drains producer-filled chunks from
/// the ring into the file's *pre-existing* cluster chain. Never allocates -
/// open a file that is already large enough; completes at [`Self::is_full`],
/// no looping.
///
/// Producer contract: fill the ring at [`Self::bytes_produced`] (wrapping)
/// and report progress through `tick`'s `produced_pos`; only whole chunks are
/// submitted. Stay within half the ring of [`Self::bytes_submitted`] -
/// submitted chunks may still be read live by the engine. `tick` fails the
/// stream with [`StreamError::Overrun`] if breached (best-effort).
pub struct FatStreamWriter {
    geo: StreamGeometry,
    map: ClusterMap,

    logical_producer: u64,
    logical_submitted: u64,

    errors: ErrorLatch,
}

impl FatStreamWriter {
    pub fn open<IO, TP, OCC>(
        file: &fatfs::File<'_, IO, TP, OCC>,
        cfg: &StreamConfig,
    ) -> Result<Self, OpenError>
    where
        IO: fatfs::ReadWriteSeek,
        TP: fatfs::TimeProvider,
    {
        let (geo, map) = resolve(file, cfg)?;
        Ok(Self {
            geo,
            map,
            logical_producer: 0,
            logical_submitted: 0,
            errors: ErrorLatch::new(),
        })
    }

    pub fn bytes_total(&self) -> u64 {
        self.geo.total_bytes()
    }

    pub fn is_full(&self) -> bool {
        self.logical_submitted == self.bytes_total()
    }

    pub fn bytes_submitted(&self) -> u64 {
        self.logical_submitted
    }

    pub fn bytes_produced(&self) -> u64 {
        self.logical_producer
    }

    pub fn tick<M: UsbMsc>(&mut self, msc: &mut M, produced_pos: u32) -> Result<(), StreamError> {
        self.errors.check(msc)?;
        self.logical_producer =
            self.geo.advance_wrapped(self.logical_producer, produced_pos);

        if !self.is_full()
            && self.logical_producer - self.logical_submitted > self.geo.ring_len() / 2
        {
            warn!("fat stream: producer overran the ring (produced {}, submitted {})",
                  self.logical_producer, self.logical_submitted);
            return Err(self.errors.fail(StreamError::Overrun));
        }

        if !self.map.poll_window(msc) {
            return Ok(());
        }

        loop {
            if self.is_full() {
                break;
            }
            let pending = self.logical_producer - self.logical_submitted;
            if pending < self.geo.chunk_bytes() as u64 {
                break;
            }

            let block = (self.logical_submitted / BLOCK_BYTES as u64) as u32;
            match self.map.lba_of(msc, block) {
                Lookup::Lba(lba) => {
                    let ring_off = (self.logical_submitted % self.geo.ring_len()) as usize;
                    let src = self.geo.ring.slice(
                        ring_off, (self.geo.chunk_blocks as usize) * BLOCK_BYTES);
                    if msc.write_blocks(lba, src).is_none() {
                        break;
                    }
                    self.logical_submitted += self.geo.chunk_bytes() as u64;
                }
                Lookup::Pending => break,
                Lookup::OutOfChain =>
                    return Err(self.errors.fail(StreamError::EndOfChain)),
            }
        }
        Ok(())
    }

    pub fn drain<M: UsbMsc>(&mut self, msc: &mut M, produced_pos: u32) -> Result<u64, StreamError> {
        loop {
            self.tick(msc, produced_pos)?;
            let pending = self.logical_producer - self.logical_submitted;
            if self.is_full() || pending < self.geo.chunk_bytes() as u64 {
                break;
            }
            self.map.wait_progress(msc)?;
        }
        msc.wait_idle()?;
        self.errors.check(msc)?;
        Ok(self.logical_submitted)
    }
}
