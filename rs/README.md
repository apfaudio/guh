`guh` rust crates
=================

Firmware crates for the USB MSC host and DMA engine in [guh](https://github.com/apfaudio/guh):

- `guh-dma`: `DmaBuf`, a pinned buffer in PSRAM that DMA may be touching.
- `guh-usb-msc`: HAL for the MSC DMA peripheral (`guh/periph/msc.py`), `rust-fatfs` support, and read/write dma streaming.

`guh-usb-msc` is a bit of a strange hybrid between `rust-fatfs`, which is generally blocking, and an async DMA submission engine, which doesn't really fit into rust's async ecosystem, despite me having thought quite hard about it...

The idea is that you use the `rust-fatfs` API for normal filesystem operations, such as listing directories, creating / allocating files, deleting things, and so on. Then, once the filesystem is in the state you want and you only want to read/write from files (i.e not move them around any more), you can instantiate N `FatStream` objects - any number of readers and writers, from file handles, and this lets you read/write to any number of files simultaneously. This is safe, as reading/writing from fixed-sized files doesn't require any modifications to the FAT any more (which should be atomic).

The API is flexible enough (or ... dangerous enough) to let you instantiate multiple streams even on the same file - so as long as you (manually) ensure they don't step on each other, it's even possible to read/write from the same file in different positions concurrently, to execute long copies.

Below is a bit of an ASCII picture of how things fit together

Where is what
-------------

```
   your firmware
        |
        |  sync dma path (mount, ls, create,  |  async dma path (e.g. per audio block,
        |  preallocate - blocking)            |  from an ISR)
        v                                     v
   fatfs::FileSystem                     FatStream / FatStreamWriter
   fatfs::File, resize_uninit()          (non-blocking read/write streaming)
        |                                     |
        v                                     |
   UsbMscBlockDevice  <- FAT cache            |
        |                                     |
        +------------------+------------------+
                           v
                  UsbMsc trait  (and PartitionView)    cpu/rust land
                  impl_usb_msc!  <- PAC
- - - - - - - - - - - - - -|- - - - - - - - - - - - - - - - - - - - - - -
                    CSR + DMA registers                gateware land
                           v
                 guh/periph/msc.py  ->  PSRAM <-> DmaBuf
                           v
                 USBMSCHost / SCSIBulkHost  (SCSI over BOT over USB)
                           v
                       USB Stick
```

All paths to access the filesystem use DMA to copy raw blocks from the attached USB stick directly into PSRAM. The difference is that the streamers generally issue a DMA request and don't block, but the normal filesystem ops issue a DMA request and then block for it to be serviced.

The general usage pattern is you'll set up the filesystem how you want, turn specific files into FatStreamers and then drop the fatfs::Filesystem, proceeding to do all streaming operations with async dma. The `guh` gateware has a command FIFO of USB/DMA requests, so when streaming files, it can continue to read/write blocks in the background while the CPU is doing something else.

Example
-----------------

Streaming a file off a thumbdrive into a PSRAM ring buffer, which some other
piece of gateware (an audio player, a video scanout, whatever) is reading from
concurrently, will look something like this --

```rust
// Allocate some buffers in PSRAM (assuming you created your own alloc method to create DmaBufs)
// These don't really need to be this big, but they should be smaller than MAX_BLOCKS_PER_XFER
let stage: [&'static DmaBuf; 2] = [alloc_dma_buf(64*1024), alloc_dma_buf(64*1024)];
let fat_cache: &'static DmaBuf  = alloc_dma_buf(64*1024);
let ring: &'static DmaBuf       = alloc_dma_buf(256*1024);

// Wait for the device to enumerate
let mut usb_msc = unsafe { UsbMsc0::new(...) };
while !usb_msc.is_ready() {}

// Find the FAT32 partition and create a view over it
let part_lba = find_first_fat32_lba(&mut usb_msc, stage[0]);
let mut usb_msc = PartitionView::new(usb_msc, part_lba);

let mut stream = {
    // mount the filesystem, fetch a file handle (blocking methods!)
    let bd = UsbMscBlockDevice::new(&mut usb_msc, stage).unwrap();
    let fs = fatfs::FileSystem::new(bd, fatfs::FsOptions::new()).unwrap();
    let file = fs.root_dir().open_file("KICK.WAV").unwrap();

    // turn the File into a nonblocking, dma-driven stream
    FatStream::open(&file, &StreamConfig {
        max_chunk_blocks: USB_MSC_MAX_BLOCKS_PER_XFER,
        ring,
        fat_cache,
    }).unwrap()

    // dropped the filesystem mount here
};

// Fill up the PSRAM ring to its max capacity (blocking)
stream.prefill(&mut usb_msc).unwrap();

// Imagine we have another piece of gateware playing audio from the PSRAM ring.
// From an ISR context, we can now asynchronously ensure the PSRAM ring is
// always containing new data from the remaining contents of the file, by
// reading where the hardware peripheral is, and enqueuing new DMAs into the
// ring ahead of where the hardware peripheral is:

let read_pos = player.position().read().bytes().bits();
stream.tick(&mut usb_msc, read_pos);

// Because `FatStream/FatStreamWriter` are not modifying the FAT, you can have
// many of these open simultaneously and `tick`ing after each-other inside an ISR.
```

The write direction is the opposite: preallocate a file with `file.resize_uninit(size)`,
open a `FatStreamWriter` on it, have your producer fill the ring and report its position
through `tick(&mut usb_msc, produced_pos)`, and `FatStreamWriter` will issue DMA requests
from PSRAM to the USB device up to wherever your producer has already placed valid blocks.

Performance
-----------

On a small 60MHz VexiiRiscv used in Tiliqua, this library can easily saturate a 480Mbps USB2 thumbdrive and achieve read/write speeds in excess of 30MiB/sec on hardware, or 45MiB/sec in simulation (when I emulate a perfect thumbdrive which never stalls!).

`rust-fatfs` fork
-----------------

`guh-usb-msc` depends on a few modifications to upstream `rust-fatfs`, which you can find here, with their own [README from me describing the changes](https://github.com/vk2seb/rust-fatfs/tree/seb/dma-geo).

Examples
--------

For examples on actually using this gateware and rust library, see [tiliqua](https://github.com/apfaudio/tiliqua) - specifically TODO (add once done).
