# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Fake USB devices for integration testing.
"""

from amaranth import *

from luna.gateware.interface.utmi import UTMIInterface
from luna.gateware.usb.usb2.control import USBControlEndpoint
from luna.usb2 import USBDevice, USBStreamInEndpoint, USBStreamOutEndpoint
from usb_protocol.emitters import DeviceDescriptorCollection

from guh.protocol.descriptors import *
from guh.engines.msc import *


def USBHSDevice(full_speed_only=False, **kwargs):
    usb = USBDevice(**kwargs)
    # workaround https://github.com/greatscottgadgets/luna/issues/276
    usb.always_fs = full_speed_only
    usb.data_clock = 60e6
    return usb


class FakeUSBMIDIDevice(Elaboratable):

    """
    Simple USB device only used for integration tests.

    Exposes a MIDI bulk OUT endpoint that emits upcounting data.
    """

    def __init__(self, max_packet_size=64, full_speed_only=False):
        self.max_packet_size = max_packet_size
        self.full_speed_only = full_speed_only
        self.utmi = UTMIInterface()
        super().__init__()

    def create_descriptors(self):
        descriptors = DeviceDescriptorCollection()
        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = 0x16d0
            d.idProduct          = 0xf3b
            d.iManufacturer      = "LUNA"
            d.iProduct           = "Test Device"
            d.iSerialNumber      = "1234"
            d.bNumConfigurations = 1
            d.bMaxPacketSize0    = self.max_packet_size

        with descriptors.ConfigurationDescriptor() as c:
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber   = 0
                i.bInterfaceClass    = InterfaceClass.AUDIO.value
                i.bInterfaceSubclass = AudioSubClass.MIDISTREAMING.value
                i.bInterfaceProtocol = AudioProtocol.AUDIO_1_0.value
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x01
                    e.wMaxPacketSize   = self.max_packet_size
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x84
                    e.wMaxPacketSize   = self.max_packet_size
        return descriptors

    def elaborate(self, platform):

        m = Module()
        m.submodules.usb = usb = USBHSDevice(full_speed_only=self.full_speed_only, bus=self.utmi)
        descriptors = self.create_descriptors()
        control_endpoint = USBControlEndpoint(utmi=self.utmi, max_packet_size=self.max_packet_size)
        control_endpoint.add_standard_request_handlers(descriptors)
        usb.add_endpoint(control_endpoint)

        # Counting endpoint OUT
        stream_ep = USBStreamInEndpoint(
            endpoint_number=4,
            max_packet_size=self.max_packet_size
        )
        usb.add_endpoint(stream_ep)
        counter = Signal(8)
        with m.If(stream_ep.stream.ready):
            m.d.usb += counter.eq(counter + 1)
        m.d.comb += [
            stream_ep.stream.valid    .eq(1),
            stream_ep.stream.payload  .eq(counter)
        ]

        m.d.comb += [
            usb.connect          .eq(1),
            usb.full_speed_only  .eq(self.full_speed_only),
        ]
        return m


class FakeUSBMSCDevice(Elaboratable):
    """
    Simple USB Mass Storage device for integration testing.
    Exposes a bulk IN/OUT endpoint pair that responds to SCSI commands.

    Simulates failing TEST_UNIT_READY first 2 attempts as thumbdrives tend to
    do something like this. Rejects commands until we returned success on the
    READY polling.

    Emits data which is the requested block address XOR'd with the byte
    index within the block.

    TODO: worth splitting out CBW wrapping into a subcomponent so we could
    use this in a real MSC device? I don't think LUNA has an example of
    this so maybe an MSC device could be worth upstreaming...
    """

    BLOCK_SIZE = 512
    BLOCK_COUNT = 1024  # 512KB total

    def __init__(self, max_packet_size=64, full_speed_only=False):
        self.ep0_max_packet_size = min(max_packet_size, 64)  # EP0 caps at 64
        self.bulk_max_packet_size = max_packet_size
        self.full_speed_only = full_speed_only
        self.utmi = UTMIInterface()
        super().__init__()

    def create_descriptors(self):
        descriptors = DeviceDescriptorCollection()
        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = 0x16d0
            d.idProduct          = 0xf3c
            d.iManufacturer      = "Test"
            d.iProduct           = "MSC Device"
            d.iSerialNumber      = "1234"
            d.bNumConfigurations = 1
            d.bMaxPacketSize0    = self.ep0_max_packet_size

        with descriptors.ConfigurationDescriptor() as c:
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber   = 0
                i.bInterfaceClass    = InterfaceClass.MASS_STORAGE.value
                i.bInterfaceSubclass = MSCSubClass.SCSI_TRANSPARENT.value
                i.bInterfaceProtocol = MSCProtocol.BULK_ONLY.value
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x02  # OUT EP2
                    e.bmAttributes     = 0x02  # Bulk
                    e.wMaxPacketSize   = self.bulk_max_packet_size
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x81  # IN EP1
                    e.bmAttributes     = 0x02  # Bulk
                    e.wMaxPacketSize   = self.bulk_max_packet_size
        return descriptors

    def elaborate(self, platform):
        m = Module()

        m.submodules.usb = usb = USBHSDevice(full_speed_only=self.full_speed_only, bus=self.utmi)
        descriptors = self.create_descriptors()
        control_endpoint = USBControlEndpoint(utmi=self.utmi, max_packet_size=self.ep0_max_packet_size)
        control_endpoint.add_standard_request_handlers(descriptors)
        usb.add_endpoint(control_endpoint)

        # Bulk OUT endpoint for receiving CBW
        stream_out = USBStreamOutEndpoint(
            endpoint_number=2,
            max_packet_size=self.bulk_max_packet_size
        )
        usb.add_endpoint(stream_out)

        # Bulk IN endpoint for sending data and CSW
        stream_in = USBStreamInEndpoint(
            endpoint_number=1,
            max_packet_size=self.bulk_max_packet_size
        )
        usb.add_endpoint(stream_in)

        # ================================================================
        # Overall USB state
        # ================================================================

        m.d.comb += [
            usb.connect          .eq(1),
            usb.full_speed_only  .eq(self.full_speed_only),
        ]

        # ================================================================
        # Capacity response (big-endian on wire)
        # ================================================================

        last_lba = self.BLOCK_COUNT - 1
        cap_response = Signal(ReadCapacity10Response)
        m.d.comb += [
            cap_response.last_lba_be.eq(byteswap(Const(last_lba, 32))),
            cap_response.block_size_be.eq(byteswap(Const(self.BLOCK_SIZE, 32))),
        ]
        cap_flat = cap_response.as_value()

        # ================================================================
        # CBW parsing
        # ================================================================

        cbw = Signal(CBW)
        cbw_flat = cbw.as_value()
        cbw_byte_idx = Signal(6)

        # Extract LBA and transfer length (block count) from CDB10
        cbw_lba = byteswap(cbw.CBWCB.cdb10.lba_be)
        cbw_xfer_len = byteswap(cbw.CBWCB.cdb10.xfer_len_be)

        # Total bytes to transfer = xfer_len * BLOCK_SIZE
        max_xfer_bytes = MAX_BLOCKS_PER_READ * self.BLOCK_SIZE
        xfer_total_bytes = Signal(range(max_xfer_bytes + 1))
        m.d.comb += xfer_total_bytes.eq(cbw_xfer_len * self.BLOCK_SIZE)

        # ================================================================
        # TEST_UNIT_READY simulation (fail first 2 attempts)
        # ================================================================

        test_ready_count = Signal(range(3))
        device_ready = Signal() # Set when TEST_UNIT_READY succeeds
        csw_fail = Signal()

        # ================================================================
        # CSW response (dynamic based on CBW tag)
        # ================================================================

        csw = Signal(CSW)
        m.d.comb += [
            csw.dCSWSignature.eq(CSW_SIGNATURE),
            csw.dCSWTag.eq(cbw.dCBWTag),
            csw.dCSWDataResidue.eq(0),
            csw.bCSWStatus.eq(Mux(csw_fail, CSWStatus.FAILED, CSWStatus.PASSED)),
        ]
        csw_flat = csw.as_value()

        # ================================================================
        # Outgoing data generation
        # ================================================================

        tx_byte_idx = Signal(range(max_xfer_bytes))
        # Block data pattern: byte index XOR'd with LBA
        block_data_byte = Signal(8)
        m.d.comb += block_data_byte.eq(tx_byte_idx[0:8] ^ cbw_lba[0:8])

        # ================================================================
        # Response state machine
        # ================================================================

        with m.FSM(domain="usb"):

            with m.State("RECV-CBW"):
                m.d.comb += stream_out.stream.ready.eq(1)
                with m.If(stream_out.stream.valid):
                    # Capture CBW bytes
                    m.d.usb += [
                        cbw_flat.word_select(cbw_byte_idx, 8).eq(stream_out.stream.payload),
                        cbw_byte_idx.eq(cbw_byte_idx + 1),
                    ]
                    with m.If(cbw_byte_idx == CBW_SIZE_BYTES - 1):
                        m.d.usb += [
                            cbw_byte_idx.eq(0),
                            tx_byte_idx.eq(0),
                            Print("MSC-DEV: recv CBW opcode =", cbw.CBWCB.cdb10.opcode),
                        ]
                        m.next = "PROCESS-CBW"

            with m.State("PROCESS-CBW"):
                with m.If(cbw.dCBWSignature != CBW_SIGNATURE):
                    # Invalid CBW, ignore and wait for next
                    m.next = "RECV-CBW"
                with m.Else():
                    with m.Switch(cbw.CBWCB.cdb10.opcode):
                        with m.Case(SCSIOpCode.TEST_UNIT_READY):
                            # Fail the first 2 attempts, succeed on 3rd
                            with m.If(test_ready_count < 2):
                                m.d.usb += csw_fail.eq(1)
                                m.d.usb += test_ready_count.eq(test_ready_count + 1)
                            with m.Else():
                                m.d.usb += device_ready.eq(1)
                            m.next = "SEND-CSW"
                        with m.Case(SCSIOpCode.READ_CAPACITY_10):
                            with m.If(device_ready):
                                m.next = "SEND-CAPACITY"
                            with m.Else():
                                m.d.usb += csw_fail.eq(1)
                                m.next = "SEND-CSW"
                        with m.Case(SCSIOpCode.READ_10):
                            with m.If(device_ready):
                                m.next = "SEND-DATA"
                            with m.Else():
                                m.d.usb += csw_fail.eq(1)
                                m.next = "SEND-CSW"
                        with m.Default():
                            m.next = "SEND-CSW"

            with m.State("SEND-CAPACITY"):
                m.d.comb += [
                    stream_in.stream.valid.eq(1),
                    stream_in.stream.payload.eq((cap_flat >> (tx_byte_idx[:3] * 8)) & 0xFF),
                    stream_in.stream.last.eq(tx_byte_idx == READ_CAPACITY_SIZE_BYTES - 1),
                ]
                with m.If(stream_in.stream.ready):
                    m.d.usb += tx_byte_idx.eq(tx_byte_idx + 1)
                    with m.If(tx_byte_idx == READ_CAPACITY_SIZE_BYTES - 1):
                        m.d.usb += tx_byte_idx.eq(0)
                        m.next = "SEND-CSW"

            with m.State("SEND-DATA"):
                is_last_byte = (tx_byte_idx == (xfer_total_bytes - 1))
                m.d.comb += [
                    stream_in.stream.valid.eq(1),
                    stream_in.stream.payload.eq(block_data_byte),
                    stream_in.stream.last.eq(is_last_byte),
                ]
                with m.If(stream_in.stream.ready):
                    m.d.usb += tx_byte_idx.eq(tx_byte_idx + 1)
                    with m.If(is_last_byte):
                        m.d.usb += tx_byte_idx.eq(0)
                        m.next = "SEND-CSW"

            with m.State("SEND-CSW"):
                m.d.comb += [
                    stream_in.stream.valid.eq(1),
                    stream_in.stream.payload.eq((csw_flat >> (tx_byte_idx[:4] * 8)) & 0xFF),
                    stream_in.stream.last.eq(tx_byte_idx == CSW_SIZE_BYTES - 1),
                ]
                with m.If(stream_in.stream.ready):
                    m.d.usb += tx_byte_idx.eq(tx_byte_idx + 1)
                    with m.If(tx_byte_idx == CSW_SIZE_BYTES - 1):
                        m.d.usb += [
                            tx_byte_idx.eq(0),
                            csw_fail.eq(0),
                            Print("MSC-DEV: sent CSW status =", csw.bCSWStatus),
                        ]
                        m.next = "RECV-CBW"

        return m
