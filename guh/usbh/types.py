# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Shared type definitions for USB Host components.
"""

from amaranth import *
from amaranth.lib import enum

from luna.gateware.interface.utmi import UTMIOperatingMode, UTMITerminationSelect
from luna.gateware.usb.usb2 import USBSpeed as LUNAUSBSpeed


class UTMIOperatingModeEnum(enum.Enum, shape=unsigned(2)):
    """native 'Enum' version of LUNA's UTMIOperatingMode constants."""
    NORMAL                    = UTMIOperatingMode.NORMAL
    NON_DRIVING               = UTMIOperatingMode.NON_DRIVING
    RAW_DRIVE                 = UTMIOperatingMode.RAW_DRIVE
    DISABLE_BITSTUFF_AND_NRZI = UTMIOperatingMode.DISABLE_BITSTUFF_AND_NRZI
    CHIRP                     = UTMIOperatingMode.CHIRP
    NO_SYNC_OR_EOP            = UTMIOperatingMode.NO_SYNC_OR_EOP


class UTMITerminationSelectEnum(enum.Enum, shape=unsigned(1)):
    """native 'Enum' version of LUNA's UTMITerminationSelect constants."""
    HS_NORMAL    = UTMITerminationSelect.HS_NORMAL
    HS_CHIRP     = UTMITerminationSelect.HS_CHIRP
    LS_FS_NORMAL = UTMITerminationSelect.LS_FS_NORMAL


class USBHostSpeed(enum.Enum, shape=unsigned(2)):
    """native 'Enum' version of LUNA's USBSpeed constants."""
    HIGH     = LUNAUSBSpeed.HIGH  # High-speed (480 Mbps)
    FULL     = LUNAUSBSpeed.FULL  # Full-speed (12 Mbps)
    LOW      = LUNAUSBSpeed.LOW   # Low-speed (1.5 Mbps)
    UNKNOWN  = 0b11               # XXX: has no meaning, but used by state machines


class UTMILineState(enum.Enum, shape=unsigned(2)):
    """UTMI D+/D- line state."""
    SE0 = 0b00
    J   = 0b01
    K   = 0b10
    SE1 = 0b11
