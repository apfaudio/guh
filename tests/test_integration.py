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

from conftest import (
    FakeUSBMIDIDevice,
    connect_utmi,
    make_packet_capture_process,
    patch_usb_timing_for_simulation,
)

class IntegrationTests(unittest.TestCase):

    @parameterized.expand([
        ["full_speed_mps8", True, 8],
        ["full_speed_mps64", True, 64],
        ["high_speed_mps64", False, 64],
    ])
    def test_usb_host_integration(self, name, full_speed_only, max_packet_size):
        """
        Tests the USBMIDIHost engine against a fake MIDI device.
        Simulates the entire speed negotiation, enumeration and polling sequence.
        Run with `-srv` for a nice packet dump while this is running.
        """

        m = Module()

        patch_usb_timing_for_simulation()

        host = USBMIDIHost(device_address=0x12)
        m.submodules.hst = hst = DomainRenamer({"usb": "sync"})(host)
        m.submodules.dev = dev = DomainRenamer({"usb": "sync"})(
            FakeUSBMIDIDevice(full_speed_only=full_speed_only, max_packet_size=max_packet_size))

        bus_event = connect_utmi(m, hst.sie.utmi, dev.utmi)

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
        sim.add_process(make_packet_capture_process(
            hst.sie.utmi, dev.utmi, bus_event, f"test_usb_host_integration_{name}.pcap"))
        with sim.write_vcd(vcd_file=open(f"test_usb_host_integration_{name}.vcd", "w")):
            sim.run()
