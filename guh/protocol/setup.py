# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB Setup Packet structure. Mostly used by the Enumerator component state machine.
"""

from amaranth import *
from amaranth.lib import data, enum

from usb_protocol.types import (
    USBStandardRequests, USBRequestType, USBDirection,
    USBRequestRecipient
)


class SetupPayload(data.Struct):
    """
    USB Control Transfer Setup Packet structure.
    """

    # Reuse python-usb-protocol types to avoid 2 sources of truth

    class Recipient(enum.Enum, shape=unsigned(5)):
        DEVICE    = int(USBRequestRecipient.DEVICE)
        INTERFACE = int(USBRequestRecipient.INTERFACE)
        ENDPOINT  = int(USBRequestRecipient.ENDPOINT)
        OTHER     = int(USBRequestRecipient.OTHER)

    class Type(enum.Enum, shape=unsigned(2)):
        STANDARD  = int(USBRequestType.STANDARD)
        CLASS     = int(USBRequestType.CLASS)
        VENDOR    = int(USBRequestType.VENDOR)
        RESERVED  = int(USBRequestType.RESERVED)

    class Direction(enum.Enum, shape=unsigned(1)):
        HOST_TO_DEVICE = int(USBDirection.OUT)
        DEVICE_TO_HOST = int(USBDirection.IN)

    class Request(enum.Enum, shape=unsigned(8)):
        SET_ADDRESS       = int(USBStandardRequests.SET_ADDRESS)
        GET_DESCRIPTOR    = int(USBStandardRequests.GET_DESCRIPTOR)
        SET_CONFIGURATION = int(USBStandardRequests.SET_CONFIGURATION)

    bmRequestType: data.StructLayout({
        'bmRecipient': Recipient,
        'bmType':      Type,
        'bmDirection': Direction,
    })
    bRequest:      Request
    wValue:        unsigned(16)
    wIndex:        unsigned(16)
    wLength:       unsigned(16)

    #
    # Helper methods to create standard request payloads.
    # These return byte lists that can be used directly or passed to Memory init.
    #

    def _dict_to_bytes(payload_dict):
        """Convert a SetupPayload init dict to a list of 8 bytes."""
        v = Signal(SetupPayload, init=payload_dict).as_value().init
        return [(v >> (n*8)) & 0xFF for n in range(8)]

    def get_descriptor(descriptor_type, descriptor_index=0, language_id=0, length=8):
        """Create GET_DESCRIPTOR setup packet as bytes."""
        return SetupPayload._dict_to_bytes({
            'bmRequestType': {
                'bmRecipient': SetupPayload.Recipient.DEVICE,
                'bmType':      SetupPayload.Type.STANDARD,
                'bmDirection': SetupPayload.Direction.DEVICE_TO_HOST,
            },
            'bRequest': SetupPayload.Request.GET_DESCRIPTOR,
            'wValue':   (descriptor_type << 8) | descriptor_index,
            'wIndex':   language_id,
            'wLength':  length,
        })

    def set_address(address):
        """Create SET_ADDRESS setup packet as bytes."""
        return SetupPayload._dict_to_bytes({
            'bmRequestType': {
                'bmRecipient': SetupPayload.Recipient.DEVICE,
                'bmType':      SetupPayload.Type.STANDARD,
                'bmDirection': SetupPayload.Direction.HOST_TO_DEVICE,
            },
            'bRequest': SetupPayload.Request.SET_ADDRESS,
            'wValue':   address,
            'wIndex':   0x0000,
            'wLength':  0x0000,
        })

    def set_configuration(configuration):
        """Create SET_CONFIGURATION setup packet as bytes."""
        return SetupPayload._dict_to_bytes({
            'bmRequestType': {
                'bmRecipient': SetupPayload.Recipient.DEVICE,
                'bmType':      SetupPayload.Type.STANDARD,
                'bmDirection': SetupPayload.Direction.HOST_TO_DEVICE,
            },
            'bRequest': SetupPayload.Request.SET_CONFIGURATION,
            'wValue':   configuration,
            'wIndex':   0x0000,
            'wLength':  0x0000,
        })
