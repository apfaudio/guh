# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB descriptor parsing and endpoint extraction.
"""

from enum import auto, Enum

from amaranth import *
from amaranth.lib import data, stream, wiring
from amaranth.lib.wiring import In, Out

from guh.protocol.descriptors import *


class EndpointFilter(Enum):
    """Which endpoint directions to extract from the descriptor."""
    IN = auto()
    OUT = auto()
    IN_AND_OUT = auto()


class USBDescriptorParser(wiring.Component):

    """
    Takes a stream of bytes from a USB configuration descriptor,
    walks each descriptor and extracts endpoints matching the specified
    interface class, transfer type, and optional subclass.

    Simulated against real descriptors in `tests/test_descriptor.py`

    Usage:
    - Hook up ``i`` to the USB Rx stream, and assert ``enable`` just
      before the configuration descriptor arrives.
    - After the descriptor is transferred, check ``o.valid`` and read endpoint(s) found.

    TODO: this is kind of a big ugly streaming state machine, but it
    seems quite reliable across all devices I have tested. Maybe it would be
    cleaner to build some more Amaranth types to represent the different overall
    descriptor structures, instead of this messy switch/case work...
    """

    def __init__(self, *, endpoint_filter, transfer_type, interface_class,
                 interface_subclass=None, interface_protocol=None):
        self._endpoint_filter = endpoint_filter
        self._transfer_type = transfer_type
        self._interface_class = interface_class
        self._interface_subclass = interface_subclass
        self._interface_protocol = interface_protocol

        # Build output signature based on endpoint_filter
        if endpoint_filter == EndpointFilter.IN:
            o_layout = {"i_endp": EndpointAddress, "valid": unsigned(1)}
        elif endpoint_filter == EndpointFilter.OUT:
            o_layout = {"o_endp": EndpointAddress, "valid": unsigned(1)}
        else:  # IN_AND_OUT
            o_layout = {"i_endp": EndpointAddress, "o_endp": EndpointAddress, "valid": unsigned(1)}

        self._o_signature = Out(data.StructLayout(o_layout))
        super().__init__({"enable": In(unsigned(1)),
                          "i": In(stream.Signature(unsigned(8))),
                          "o": self._o_signature})

    def elaborate(self, platform):

        m = Module()

        bLength = Signal(unsigned(8))
        offset = Signal.like(bLength)

        desc_type = Signal(DescriptorType, init=0)
        iface_class = Signal(InterfaceClass, init=0)
        if self._interface_subclass is not None:
            iface_subclass = Signal(type(self._interface_subclass), init=0)
        if self._interface_protocol is not None:
            iface_protocol = Signal(type(self._interface_protocol), init=0)
        in_matching_interface = Signal()

        # Endpoint descriptor fields (temporary during parsing)
        endp_addr = Signal(EndpointAddress)
        endp_attr = Signal(EndpointAttributes)

        # Tracking which endpoints have been found
        want_in = self._endpoint_filter in (EndpointFilter.IN, EndpointFilter.IN_AND_OUT)
        want_out = self._endpoint_filter in (EndpointFilter.OUT, EndpointFilter.IN_AND_OUT)
        found_in = Signal() if want_in else None
        found_out = Signal() if want_out else None

        m.d.comb += self.i.ready.eq(1)

        with m.FSM(domain="usb"):
            with m.State("INIT"):
                m.d.comb += self.i.ready.eq(0)
                with m.If(self.enable):
                    m.next = "GET-LEN"
            with m.State("GET-LEN"):
                with m.If(self.i.valid):
                    m.d.usb += offset.eq(0)
                    m.d.usb += bLength.eq(self.i.payload)
                    m.next = "IN-DESCRIPTOR"
            with m.State("IN-DESCRIPTOR"):
                with m.If(self.i.valid):
                    with m.Switch(offset):
                        # Byte 1: descriptor type
                        with m.Case(0):
                            m.d.usb += desc_type.eq(self.i.payload)
                        # Endpoint descriptor: byte 2 = bEndpointAddress
                        with m.Case(1):
                            with m.If(desc_type == DescriptorType.ENDPOINT):
                                m.d.usb += endp_addr.eq(self.i.payload)
                        # Endpoint descriptor: byte 3 = bmAttributes
                        with m.Case(2):
                            with m.If(desc_type == DescriptorType.ENDPOINT):
                                m.d.usb += endp_attr.eq(self.i.payload)
                        # Interface descriptor: byte 5 = bInterfaceClass
                        with m.Case(4):
                            with m.If(desc_type == DescriptorType.INTERFACE):
                                m.d.usb += iface_class.eq(self.i.payload)
                        # Interface descriptor: byte 6 = bInterfaceSubClass
                        if self._interface_subclass is not None:
                            with m.Case(5):
                                with m.If(desc_type == DescriptorType.INTERFACE):
                                    m.d.usb += iface_subclass.eq(self.i.payload)
                        # Interface descriptor: byte 7 = bInterfaceProtocol
                        if self._interface_protocol is not None:
                            with m.Case(6):
                                with m.If(desc_type == DescriptorType.INTERFACE):
                                    m.d.usb += iface_protocol.eq(self.i.payload)

                    m.d.usb += offset.eq(offset+1)

                    # At the end of each descriptor
                    with m.If(offset == (bLength-2)):
                        m.d.usb += Print(desc_type, 'len =', bLength)
                        # Interface descriptor: update in_matching_interface flag
                        with m.If(desc_type == DescriptorType.INTERFACE):
                            m.d.usb += Print('\t bInterfaceClass =', iface_class)
                            if self._interface_subclass is not None:
                                m.d.usb += Print('\t bInterfaceSubClass =', iface_subclass)
                            if self._interface_protocol is not None:
                                m.d.usb += Print('\t bInterfaceProtocol =', iface_protocol)

                            # Check class match (and subclass/protocol if specified)
                            interface_match = (iface_class == self._interface_class)
                            if self._interface_subclass is not None:
                                interface_match = interface_match & (iface_subclass == self._interface_subclass)
                            if self._interface_protocol is not None:
                                interface_match = interface_match & (iface_protocol == self._interface_protocol)

                            with m.If(interface_match):
                                m.d.usb += in_matching_interface.eq(1)
                            with m.Else():
                                m.d.usb += in_matching_interface.eq(0)

                        # Endpoint descriptor: capture first matching endpoints
                        capturing_in = Signal()
                        capturing_out = Signal()

                        with m.Elif((desc_type == DescriptorType.ENDPOINT)):
                            m.d.usb += Print('\t bEndpointAddress = ', endp_addr)
                            m.d.usb += Print('\t bmAttributes = ', endp_attr)
                            with m.If(in_matching_interface):
                                type_match = endp_attr.transfer_type == self._transfer_type
                                is_in = endp_addr.direction == EndpointDirection.IN

                                # Capture IN endpoint if wanted and not yet found
                                if want_in:
                                    with m.If(type_match & is_in & ~found_in):
                                        m.d.comb += capturing_in.eq(1)
                                        m.d.usb += [
                                            self.o.i_endp.eq(endp_addr),
                                            found_in.eq(1),
                                            Print('\t **** EXTRACTED IN ****')
                                        ]

                                # Capture OUT endpoint if wanted and not yet found
                                if want_out:
                                    with m.If(type_match & ~is_in & ~found_out):
                                        m.d.comb += capturing_out.eq(1)
                                        m.d.usb += [
                                            self.o.o_endp.eq(endp_addr),
                                            found_out.eq(1),
                                            Print('\t **** EXTRACTED OUT ****')
                                        ]

                        # Check if we have all required endpoints
                        all_found = Const(1)
                        if want_in:
                            all_found = all_found & (found_in | capturing_in)
                        if want_out:
                            all_found = all_found & (found_out | capturing_out)

                        with m.If(all_found):
                            m.d.usb += self.o.valid.eq(1)
                            m.next = "DONE"
                        with m.Else():
                            m.next = "GET-LEN"

            with m.State("DONE"):
                pass

        return m
