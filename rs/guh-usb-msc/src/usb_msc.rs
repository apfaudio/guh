use guh_dma::DmaBuf;

/// Logical block size assumed throughout the stack (gateware DMA word counts,
/// FAT math, ring chunking, buffer sizing). Devices reporting anything else
/// are rejected at mount rather than accommodated.
pub const BLOCK_BYTES: usize = 512;

/// A transfer failed: the device reported an error mid-transfer, or it went
/// away (or was swapped) while we were waiting on it.
#[derive(Debug, Clone, Copy)]
pub struct XferError;

#[derive(Debug, Clone, Copy)]
pub struct UsbMscStatus {
    pub connected: bool,
    pub ready: bool,
    pub busy: bool,
    pub fifo_full: bool,
    pub fifo_empty: bool,
    pub error: bool,
}

pub trait UsbMsc {
    fn status(&self) -> UsbMscStatus;

    fn capacity(&self) -> u32;

    fn block_size(&self) -> u32;

    fn is_ready(&self) -> bool {
        self.status().ready
    }

    fn is_idle(&self) -> bool {
        let s = self.status();
        !s.busy && s.fifo_empty
    }

    /// Times the engine re-enumerated (wrapping). Any change invalidates
    /// capacity, cached blocks, and commands queued across it.
    fn plug_events(&self) -> u32;

    /// Whether waiting any longer is futile: the device was swapped since
    /// `plugs`, or there is nothing attached to make progress.
    fn link_lost(&self, plugs: u32) -> bool {
        self.plug_events() != plugs || !self.status().connected
    }

    /// Read `buf.len() / 512` contiguous blocks from `start_lba` into `buf`
    /// (len must be a non-zero multiple of 512). `'static` because the DMA
    /// outlives the borrow. Returns the command's seq number, `None` if full.
    fn read_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32>;

    /// Submit a `read_blocks` and block until it completes, invalidating the
    /// CPU's cached view of `buf` on success. Spins while the FIFO is full,
    /// giving up if the device disappears.
    fn read_blocks_blocking(
        &mut self, start_lba: u32, buf: &'static DmaBuf,
    ) -> Result<(), XferError> {
        let errors_before = self.error_count();
        let plugs = self.plug_events();
        let seq = loop {
            if let Some(seq) = self.read_blocks(start_lba, buf) { break seq; }
            if self.link_lost(plugs) { return Err(XferError); }
            core::hint::spin_loop();
        };
        self.wait_seq(seq)?;
        if self.error_count() != errors_before {
            return Err(XferError);
        }
        buf.invalidate();
        Ok(())
    }

    /// Write `buf.len() / 512` contiguous blocks from `buf` to `start_lba`.
    /// Same contract as [`Self::read_blocks`], reversed; `buf` must not be
    /// modified until the command completes (the engine reads it live).
    fn write_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32>;

    /// Submit a `write_blocks` and block until it completes, giving up if the
    /// device disappears. Cache maintenance happens at submit: `write_blocks`
    /// cleans the dcache over `buf`.
    fn write_blocks_blocking(
        &mut self, start_lba: u32, buf: &'static DmaBuf,
    ) -> Result<(), XferError> {
        let errors_before = self.error_count();
        let plugs = self.plug_events();
        let seq = loop {
            if let Some(seq) = self.write_blocks(start_lba, buf) { break seq; }
            if self.link_lost(plugs) { return Err(XferError); }
            core::hint::spin_loop();
        };
        self.wait_seq(seq)?;
        if self.error_count() != errors_before {
            return Err(XferError);
        }
        Ok(())
    }

    fn psram_base(&self) -> u32;

    /// Commands completed (hardware counter, wrapping); a command completes
    /// only once its data is fully in PSRAM (or it failed).
    fn completed_count(&self) -> u32;

    fn error_count(&self) -> u32;

    fn seq_done(&self, seq: u32) -> bool {
        self.completed_count().wrapping_sub(seq) < 0x8000_0000
    }

    fn wait_seq(&self, seq: u32) -> Result<(), XferError> {
        let plugs = self.plug_events();
        while !self.seq_done(seq) {
            if self.link_lost(plugs) { return Err(XferError); }
            core::hint::spin_loop();
        }
        Ok(())
    }

    fn wait_idle(&self) -> Result<(), XferError> {
        let plugs = self.plug_events();
        while !self.is_idle() {
            if self.link_lost(plugs) { return Err(XferError); }
            core::hint::spin_loop();
        }
        Ok(())
    }
}

/// A partition-relative view of an underlying [`UsbMsc`] device.
///
/// Adds `lba_offset` to every transfer so LBA 0 of the view is the first
/// sector of the partition; everything else delegates verbatim (counters
/// are device-global, DMA addressing unchanged).
pub struct PartitionView<M: UsbMsc> {
    inner: M,
    lba_offset: u32,
}

impl<M: UsbMsc> PartitionView<M> {
    pub fn new(inner: M, lba_offset: u32) -> Self {
        Self { inner, lba_offset }
    }
}

impl<M: UsbMsc> UsbMsc for PartitionView<M> {
    fn read_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32> {
        self.inner.read_blocks(start_lba + self.lba_offset, buf)
    }

    fn write_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32> {
        self.inner.write_blocks(start_lba + self.lba_offset, buf)
    }

    fn capacity(&self) -> u32 {
        self.inner.capacity().saturating_sub(self.lba_offset)
    }

    fn status(&self) -> UsbMscStatus { self.inner.status() }
    fn block_size(&self) -> u32 { self.inner.block_size() }
    fn psram_base(&self) -> u32 { self.inner.psram_base() }
    fn completed_count(&self) -> u32 { self.inner.completed_count() }
    fn error_count(&self) -> u32 { self.inner.error_count() }
    fn plug_events(&self) -> u32 { self.inner.plug_events() }
}

#[macro_export]
macro_rules! impl_usb_msc {
    ($(
        $USBMSCX:ident: $PACUSBMSCX:ty,
    )+) => {
        $(
            use $crate::usb_msc::{UsbMsc, UsbMscStatus};
            use $crate::DmaBuf;

            #[derive(Debug)]
            pub struct $USBMSCX {
                registers: $PACUSBMSCX,
                psram_base: u32,
            }

            impl $USBMSCX {
                pub fn new(registers: $PACUSBMSCX, psram_base: u32) -> Self {
                    Self { registers, psram_base }
                }

                pub fn free(self) -> $PACUSBMSCX {
                    self.registers
                }

                fn submit_cmd(&mut self, start_lba: u32, buf: &'static DmaBuf,
                              write: bool) -> Option<u32> {
                    let n_blocks = (buf.len() / $crate::usb_msc::BLOCK_BYTES) as u32;
                    debug_assert!(buf.len() % $crate::usb_msc::BLOCK_BYTES == 0);
                    debug_assert!(n_blocks >= 1 && n_blocks <= 64);

                    if write {
                        // Outside the critical section: hundreds of cache
                        // ops for a large buffer, touches no CSR.
                        buf.clean();
                    }

                    let physical_offset = buf.psram_offset(self.psram_base);

                    // One set of staging registers: a concurrent submitter
                    // would interleave commands. The full-check is inside too,
                    // so the slot we found free is still free when we fill it.
                    $crate::critical_section::with(|_| {
                        if self.registers.status().read().fifo_full().bit() {
                            return None;
                        }
                        // The FIFO has room and we are the only submitter, so
                        // this enqueue is the next one the engine counts.
                        let seq = self.registers.cmds_submitted().read()
                            .count().bits().wrapping_add(1);

                        self.registers.cmd_lba().write(|w| unsafe {
                            w.lba().bits(start_lba)
                        });
                        // Physical byte address; gateware converts to word address.
                        self.registers.cmd_addr().write(|w| unsafe {
                            w.addr().bits(physical_offset)
                        });
                        self.registers.cmd_blocks().write(|w| unsafe {
                            w.blocks().bits(n_blocks as u8)
                        });
                        self.registers.cmd_dir().write(|w| {
                            w.write().bit(write)
                        });

                        // Release: no prior memory op (including buffer fills
                        // for writes) may sink past handing it to the engine.
                        core::sync::atomic::compiler_fence(
                            core::sync::atomic::Ordering::Release);

                        self.registers.cmd_start().write(|w| {
                            w.start().bit(true)
                        });

                        Some(seq)
                    })
                }
            }

            impl UsbMsc for $USBMSCX {
                fn status(&self) -> UsbMscStatus {
                    let s = self.registers.status().read();
                    UsbMscStatus {
                        connected: s.connected().bit(),
                        ready: s.ready().bit(),
                        busy: s.busy().bit(),
                        fifo_full: s.fifo_full().bit(),
                        fifo_empty: s.fifo_empty().bit(),
                        error: s.error().bit(),
                    }
                }

                fn capacity(&self) -> u32 {
                    self.registers.capacity().read().block_count().bits()
                }

                fn block_size(&self) -> u32 {
                    self.registers.block_size().read().block_size().bits()
                }

                fn read_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32> {
                    self.submit_cmd(start_lba, buf, false)
                }

                fn write_blocks(&mut self, start_lba: u32, buf: &'static DmaBuf) -> Option<u32> {
                    self.submit_cmd(start_lba, buf, true)
                }

                fn psram_base(&self) -> u32 {
                    self.psram_base
                }

                fn completed_count(&self) -> u32 {
                    self.registers.cmds_done().read().count().bits()
                }

                fn error_count(&self) -> u32 {
                    self.registers.errors().read().count().bits()
                }

                fn plug_events(&self) -> u32 {
                    self.registers.plug_events().read().count().bits()
                }
            }
        )+
    };
}
