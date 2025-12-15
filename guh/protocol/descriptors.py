# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB descriptor type definitions. Mostly used by the descriptor parser for endpoint extraction.
"""

from amaranth import *
from amaranth.lib import data, enum


class DescriptorType(enum.Enum, shape=unsigned(8)):
    UNKNOWN              = 0x0 # XXX: Not part of standard, but makes parser logic simpler
    DEVICE               = 0x1
    CONFIG               = 0x2
    STRING               = 0x3
    INTERFACE            = 0x4
    ENDPOINT             = 0x5
    DEVICE_QUALIFIER     = 0x6
    OTHER_SPEED_CONFIG   = 0x7
    INTERFACE_POWER      = 0x8


class InterfaceClass(enum.Enum, shape=unsigned(8)):
    UNKNOWN              = 0x00  # XXX: Not part of standard, makes parser logic simpler
    # DEVICE             = 0x00 # Only appears in device (not interface) descriptors
    AUDIO                = 0x01
    COMMUNICATIONS       = 0x02
    HID                  = 0x03
    PHYSICAL             = 0x05
    IMAGE                = 0x06
    PRINTER              = 0x07
    MASS_STORAGE         = 0x08
    # HUB                = 0x09 # Only appears in device (not interface) descriptors
    CDC_DATA             = 0x0a
    SMART_CARD           = 0x0b
    CONTENT_SECURITY     = 0x0d
    VIDEO                = 0x0e
    PERSONAL_HEALTHCARE  = 0x0f
    AUDIO_VIDEO          = 0x10
    BILLBOARD            = 0x11
    USB_C_BRIDGE         = 0x12
    BULK_DISPLAY_PROTO   = 0x13
    MCTP_USB_EP          = 0x14
    I3C                  = 0x3c
    DIAGNOSTIC_DEVICE    = 0xdc
    WIRELESS_CONTROLLER  = 0xe0
    MISCELLANEOUS        = 0xef
    APPLICATION_SPECIFIC = 0xfe
    VENDOR_SPECIFIC      = 0xff


class AudioSubClass(enum.Enum, shape=unsigned(8)):
    UNDEFINED            = 0x0
    AUDIOCONTROL         = 0x1
    AUDIOSTREAMING       = 0x2
    MIDISTREAMING        = 0x3


class AudioProtocol(enum.Enum, shape=unsigned(8)):
    AUDIO_1_0 = 0x00  # Audio Class 1.0 (or undefined)
    AUDIO_2_0 = 0x20  # Audio Class 2.0 (IP_VERSION_02_00)


class MSCSubClass(enum.Enum, shape=unsigned(8)):
    SCSI_NOT_REPORTED = 0x00  # SCSI command set not reported
    RBC               = 0x01  # Reduced Block Commands
    MMC5              = 0x02  # ATAPI (CD/DVD)
    QIC157            = 0x03  # Tape
    UFI               = 0x04  # Floppy (USB)
    SFF8070I          = 0x05  # Floppy (ATAPI)
    SCSI_TRANSPARENT  = 0x06  # SCSI transparent command set (thumbdrives)


class MSCProtocol(enum.Enum, shape=unsigned(8)):
    CBI_WITH_INTERRUPT    = 0x00  # Control/Bulk/Interrupt with command completion
    CBI_WITHOUT_INTERRUPT = 0x01  # Control/Bulk/Interrupt without command completion
    BULK_ONLY             = 0x50  # Bulk-Only Transport (BBB) - most common
    UAS                   = 0x62  # USB Attached SCSI (faster, newer)


class HIDSubClass(enum.Enum, shape=unsigned(8)):
    NONE           = 0x00
    BOOT_INTERFACE = 0x01


class HIDProtocol(enum.Enum, shape=unsigned(8)):
    NONE     = 0x00
    KEYBOARD = 0x01
    MOUSE    = 0x02


class EndpointTransferType(enum.Enum, shape=unsigned(2)):
    CONTROL     = 0b00
    ISOCHRONOUS = 0b01
    BULK        = 0b10
    INTERRUPT   = 0b11


class EndpointDirection(enum.Enum, shape=unsigned(1)):
    OUT = 0  # Host to device
    IN  = 1  # Device to host


class EndpointAddress(data.Struct):
    number:    unsigned(4)            # Endpoint number (bits 3:0)
    _reserved: unsigned(3)            # Reserved (bits 6:4)
    direction: EndpointDirection      # Direction (bit 7)


class EndpointAttributes(data.Struct):
    transfer_type: EndpointTransferType  # Transfer type (bits 1:0)
    _reserved:     unsigned(6)           # Reserved (bits 7:2)
