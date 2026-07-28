//! Rust HAL for the guh USB Mass Storage Class DMA peripheral
//! (`guh/periph/msc.py`). The [`impl_usb_msc!`] macro encodes that
//! peripheral's register map; the [`usb_msc::UsbMsc`] trait is PAC-independent.

#![cfg_attr(not(test), no_std)]

pub mod usb_msc;
pub mod partition;
#[cfg(feature = "fatfs")]
pub mod usb_msc_fatfs;
#[cfg(feature = "fatfs")]
pub mod fat_stream;

pub use guh_dma::DmaBuf;

/// Re-exported so [`impl_usb_msc!`] can name it: the macro expands in the
/// caller's crate, which need not depend on `critical-section` itself.
pub use critical_section;
