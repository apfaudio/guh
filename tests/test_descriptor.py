# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Tests for USB descriptor endpoint extraction.
"""

import unittest

from amaranth import *
from amaranth.sim import *
from parameterized import parameterized

from guh.util import test_util
from guh.usbh.descriptor import *

# Some aliases for common device classes
# Usually the host engine itself defines its own parser for the
# types of endpoints it is looking for.

MIDIDescriptorParser = lambda: USBDescriptorParser(
    endpoint_filter=EndpointFilter.IN_AND_OUT,
    transfer_type=EndpointTransferType.BULK,
    interface_class=InterfaceClass.AUDIO,
    interface_subclass=AudioSubClass.MIDISTREAMING,
    interface_protocol=AudioProtocol.AUDIO_1_0,
)

MSCDescriptorParser = lambda: USBDescriptorParser(
    endpoint_filter=EndpointFilter.IN_AND_OUT,
    transfer_type=EndpointTransferType.BULK,
    interface_class=InterfaceClass.MASS_STORAGE,
    interface_subclass=MSCSubClass.SCSI_TRANSPARENT,
    interface_protocol=MSCProtocol.BULK_ONLY,
)

HIDKeyboardDescriptorParser = lambda: USBDescriptorParser(
    endpoint_filter=EndpointFilter.IN,
    transfer_type=EndpointTransferType.INTERRUPT,
    interface_class=InterfaceClass.HID,
    interface_subclass=HIDSubClass.BOOT_INTERFACE,
    interface_protocol=HIDProtocol.KEYBOARD,
)

HIDMouseDescriptorParser = lambda: USBDescriptorParser(
    endpoint_filter=EndpointFilter.IN,
    transfer_type=EndpointTransferType.INTERRUPT,
    interface_class=InterfaceClass.HID,
    interface_subclass=HIDSubClass.BOOT_INTERFACE,
    interface_protocol=HIDProtocol.MOUSE,
)

class DescriptorTests(unittest.TestCase):

    @parameterized.expand([
        ["arturia_keylabmkii", MIDIDescriptorParser, 1, 2],
        ["oxi_one",            MIDIDescriptorParser, 1, 1],
        ["yamaha_cp73",        MIDIDescriptorParser, 2, 3],
        ["yamaha_pssa50",      MIDIDescriptorParser, 2, 1],
        ["android_uac_midi",   MIDIDescriptorParser, 1, 1],
        ["korg_microkey2",     MIDIDescriptorParser, 2, 1],
        ["sandisk_32gen1",     MSCDescriptorParser, 1, 2],
        ["samsung_ssd_t5",     MSCDescriptorParser, 1, 2],
        ["anker_cardreader",   MSCDescriptorParser, 2, 1],
        ["logi_g502",          HIDMouseDescriptorParser, 1, None],
        # dual-function wireless receivers (keyboard and mouse, we selecting one function)
        ["logi_rec1",          HIDMouseDescriptorParser, 2, None],
        ["logi_rec2",          HIDKeyboardDescriptorParser, 1, None],
    ])
    def test_descriptor_parser(self, name, parser_cls, expected_endp_in, expected_endp_out):

        dut = DomainRenamer({"usb": "sync"})(parser_cls())

        async def testbench(ctx):
            ctx.set(dut.enable, 1)
            with open(f'tests/data/usbdesc_config/{name}.bin', 'rb') as f:
                for byte in f.read():
                    await test_util.put(ctx, dut.i, byte)
            ctx.tick()
            self.assertEqual(ctx.get(dut.o.valid), 1)
            if expected_endp_in is not None:
                self.assertEqual(ctx.get(dut.o.i_endp.number), expected_endp_in)
            if expected_endp_out is not None:
                self.assertEqual(ctx.get(dut.o.o_endp.number), expected_endp_out)

        sim = Simulator(dut)
        sim.add_clock(1e-6)
        sim.add_testbench(testbench)
        with sim.write_vcd(vcd_file=open(f"test_endpoint_extractor_{name}.vcd", "w")):
            sim.run()
