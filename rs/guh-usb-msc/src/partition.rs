//! Best-effort partition detection for the first sector of a USB MSC device.
//!
//! rust-fatfs has no partition support, so callers feed the resolved start
//! LBA to the block device's `lba_offset`. Classification structure adapted
//! from the `embedded-partitions` crate's MBR module
//! (<https://github.com/mabezdev/embedded-fatfs>, MIT).

use guh_dma::DmaBuf;
use log::{info, warn};

use crate::usb_msc::{UsbMsc, BLOCK_BYTES};

/// Offset of the `0x55AA` boot signature (little-endian) in the sector.
const BOOT_SIGNATURE_OFFSET: usize = 510;
const BOOT_SIGNATURE: u16 = 0xAA55;

/// MBR partition table: four 16-byte entries starting at byte 446.
const PARTITION_TABLE_OFFSET: usize = 446;
const PARTITION_ENTRY_SIZE: usize = 16;
const PARTITION_COUNT: usize = 4;
/// Byte offsets within a partition entry.
const ENTRY_STATUS: usize = 0;
const ENTRY_TYPE: usize = 4;
const ENTRY_START_LBA: usize = 8;
/// Partition status byte: only "inactive" or "bootable" are valid.
const STATUS_INACTIVE: u8 = 0x00;
const STATUS_BOOTABLE: u8 = 0x80;

/// FAT BPB field offsets used to sniff a bare boot sector (superfloppy).
const BPB_JUMP: usize = 0;
const BPB_BYTES_PER_SECTOR: usize = 11;
const BPB_NUM_FATS: usize = 16;

/// MBR partition type IDs for FAT32: 0x0B (CHS) and 0x0C (LBA). FAT12/16
/// types are intentionally excluded — the streamer only supports FAT32.
const PTYPE_FAT32_CHS: u8 = 0x0B;
const PTYPE_FAT32_LBA: u8 = 0x0C;

/// Classification of a device's first sector.
enum Layout {
    /// A plausible MBR: boot signature present and all four partition status
    /// bytes are `0x00`/`0x80`. Mount the first FAT partition it lists.
    Mbr,
    /// A FAT boot sector sitting directly at LBA 0 (whole-device /
    /// "superfloppy" format, no partition table). Mount at offset 0.
    Superfloppy,
    /// No recognised layout. Fall back to mounting at offset 0.
    Unknown,
}

fn le16(sector: &DmaBuf, off: usize) -> u16 {
    (sector.read_byte(off) as u16) | ((sector.read_byte(off + 1) as u16) << 8)
}

fn le32(sector: &DmaBuf, off: usize) -> u32 {
    (sector.read_byte(off) as u32)
        | ((sector.read_byte(off + 1) as u32) << 8)
        | ((sector.read_byte(off + 2) as u32) << 16)
        | ((sector.read_byte(off + 3) as u32) << 24)
}

/// Classify the first sector. The MBR status-byte test runs *before* the BPB
/// sniff, so an MBR whose first byte happens to look like a jump instruction
/// still classifies as `Mbr`.
fn classify(sector: &DmaBuf) -> Layout {
    if le16(sector, BOOT_SIGNATURE_OFFSET) != BOOT_SIGNATURE {
        return Layout::Unknown;
    }
    let status_ok = (0..PARTITION_COUNT).all(|i| {
        let s = sector.read_byte(PARTITION_TABLE_OFFSET + i * PARTITION_ENTRY_SIZE + ENTRY_STATUS);
        s == STATUS_INACTIVE || s == STATUS_BOOTABLE
    });
    if status_ok {
        Layout::Mbr
    } else if looks_like_fat_bpb(sector) {
        Layout::Superfloppy
    } else {
        Layout::Unknown
    }
}

/// Best-effort FAT BPB sniff: jump instruction at offset 0, a sane
/// `bytes_per_sector`, and `num_fats` of 1 or 2.
fn looks_like_fat_bpb(sector: &DmaBuf) -> bool {
    let jmp = matches!(sector.read_byte(BPB_JUMP), 0xEB | 0xE9);
    let bps_ok = matches!(le16(sector, BPB_BYTES_PER_SECTOR), 512 | 1024 | 2048 | 4096);
    let nfats_ok = matches!(sector.read_byte(BPB_NUM_FATS), 1 | 2);
    jmp && bps_ok && nfats_ok
}

/// Start LBA of the first FAT32 partition in the MBR at LBA 0 (`scratch` must
/// hold at least one block). 0 means "mount at the device start": superfloppy
/// format, or nothing usable found. The FAT type itself is not checked here.
pub fn find_first_fat32_lba<M: UsbMsc>(usb_msc: &mut M, scratch: &'static DmaBuf) -> u32 {
    let sector = scratch.slice(0, BLOCK_BYTES);
    if usb_msc.read_blocks_blocking(0, sector).is_err() {
        warn!("partition probe: read of LBA 0 failed; assuming no partition table");
        return 0;
    }

    match classify(sector) {
        Layout::Superfloppy | Layout::Unknown => 0,
        Layout::Mbr => {
            // First entry naming a non-empty FAT32 partition wins.
            for i in 0..PARTITION_COUNT {
                let e = PARTITION_TABLE_OFFSET + i * PARTITION_ENTRY_SIZE;
                let ptype = sector.read_byte(e + ENTRY_TYPE);
                let start = le32(sector, e + ENTRY_START_LBA);
                let is_fat32 = matches!(ptype, PTYPE_FAT32_CHS | PTYPE_FAT32_LBA);
                if is_fat32 && start != 0 {
                    info!("MBR FAT32 partition {} (type 0x{:02x}) at LBA {}", i, ptype, start);
                    return start;
                }
            }
            warn!("LBA 0 is an MBR but names no FAT32 partition; mounting at 0");
            0
        }
    }
}
