# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Integration tests for USB host stack.
"""

import unittest

from amaranth import *
from amaranth.sim import *
from parameterized import parameterized

from guh.usbh.types import USBHostSpeed
from guh.engines.midi import USBMIDIHost
from guh.engines.msc import USBMSCHost
from guh.util.test_devices import FakeUSBMIDIDevice, FakeUSBMSCDevice
from guh.util import test_util

class IntegrationTests(unittest.TestCase):

    @parameterized.expand([
        ["full_speed_mps8", True, 8],
        ["full_speed_mps64", True, 64],
        ["high_speed_mps64", False, 64],
    ])
    def test_usb_midi_host_integration(self, name, full_speed_only, max_packet_size):
        """
        Tests the USBMIDIHost engine against a fake MIDI device.
        Simulates the entire speed negotiation, enumeration and polling sequence.
        Run with `-srv` for a nice packet dump while this is running.
        """

        m = Module()

        test_util.patch_usb_timing_for_simulation()

        host = USBMIDIHost(device_address=0x12)
        m.submodules.hst = hst = DomainRenamer({"usb": "sync"})(host)
        m.submodules.dev = dev = DomainRenamer({"usb": "sync"})(
            FakeUSBMIDIDevice(full_speed_only=full_speed_only, max_packet_size=max_packet_size))

        bus_event = test_util.connect_utmi(m, hst.sie.utmi, dev.utmi)

        expected_speed = USBHostSpeed.FULL if full_speed_only else USBHostSpeed.HIGH
        midi_bytes_received = []

        async def testbench(ctx):
            ctx.set(hst.o_midi.ready, 1)
            for _ in range(80000):
                await ctx.tick()
                if ctx.get(hst.o_midi.valid):
                    midi_bytes_received.append(ctx.get(hst.o_midi.payload.data))
            self.assertGreater(len(midi_bytes_received), 0,
                "Expected MIDI output bytes but none were received")
            self.assertTrue(ctx.get(hst.sie.ctrl.status.detected_speed == expected_speed),
                f"Expected detected speed to be {expected_speed.name}")

        sim = Simulator(m)
        sim.add_clock(1/60e6)
        sim.add_testbench(testbench)
        sim.add_process(test_util.make_packet_capture_process(
            hst.sie.utmi, dev.utmi, bus_event, f"test_usb_midi_host_integration_{name}.pcap"))
        with sim.write_vcd(vcd_file=open(f"test_usb_midi_host_integration_{name}.vcd", "w")):
            sim.run()

    def test_usb_msc_host_integration(self):
        """
        Tests the USBMSCHost engine against a fake MSC device at high speed.
        Goes through enumeration, MSC initialization (TEST UNIT READY, READ CAPACITY),
        and block read operations.
        """

        m = Module()

        test_util.patch_usb_timing_for_simulation()

        host = USBMSCHost(device_address=0x12)
        m.submodules.hst = hst = DomainRenamer({"usb": "sync"})(host)
        m.submodules.dev = dev = DomainRenamer({"usb": "sync"})(
            FakeUSBMSCDevice(full_speed_only=False, max_packet_size=64))

        bus_event = test_util.connect_utmi(m, hst.sie.utmi, dev.utmi)

        block_data_received = []

        async def testbench(ctx):
            ctx.set(hst.rx_data.ready, 1)

            # Wait for enumeration and MSC setup to complete.
            for _ in range(100000):
                await ctx.tick()
                if ctx.get(hst.status.ready):
                    break
            else:
                self.fail("MSC host did not become ready")

            # Should enumerate at high-speed.
            self.assertTrue(ctx.get(hst.sie.ctrl.status.detected_speed == USBHostSpeed.HIGH),
                "Expected high speed")

            # Verify reported block count / size is what we expect
            block_count = ctx.get(hst.status.block_count)
            block_size = ctx.get(hst.status.block_size)
            print(f"Device capacity: {block_count} blocks x {block_size} bytes")
            self.assertEqual(block_count, FakeUSBMSCDevice.BLOCK_COUNT)
            self.assertEqual(block_size, FakeUSBMSCDevice.BLOCK_SIZE)

            # Issue a read request for block 42 and start
            ctx.set(hst.cmd.lba, 42)
            ctx.set(hst.cmd.start, 1)
            await ctx.tick()
            ctx.set(hst.cmd.start, 0)

            # Wait for block contents to arrive
            for _ in range(50000):
                await ctx.tick()
                if ctx.get(hst.rx_data.valid):
                    block_data_received.append(ctx.get(hst.rx_data.payload.data))
                if ctx.get(hst.resp.done):
                    break

            # Verify block data matches what the test device should emit
            self.assertEqual(len(block_data_received), 512,
                f"Expected 512 bytes, got {len(block_data_received)}")
            for i, byte in enumerate(block_data_received):
                expected = (i ^ 42) & 0xFF
                self.assertEqual(byte, expected,
                    f"Byte {i}: expected {expected}, got {byte}")

            # Verify no unexpected errors occurred
            self.assertFalse(ctx.get(hst.resp.error), "Block read reported error")

        sim = Simulator(m)
        sim.add_clock(1/60e6)
        sim.add_testbench(testbench)
        sim.add_process(test_util.make_packet_capture_process(
            hst.sie.utmi, dev.utmi, bus_event, "test_usb_msc_host_integration.pcap"))
        with sim.write_vcd(vcd_file=open("test_usb_msc_host_integration.vcd", "w")):
            sim.run()
