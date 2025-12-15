# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Tests for USB protocol components (tokens, setup payloads).
"""

import unittest

from amaranth import *
from amaranth.sim import *
from parameterized import parameterized
from usb_protocol.types import DescriptorTypes

from luna.gateware.test.contrib import usb_packet as testp

from guh.usbh.sie import (
    TokenPID,
    TokenPayload,
    USBTokenPacketGenerator,
)
from guh.protocol.setup import SetupPayload


class TokenTests(unittest.TestCase):

    def _setup_token(pid, addr, endp):
        def _token(ctx, payload):
            ctx.set(payload.pid, pid)
            ctx.set(payload.data.addr, addr)
            ctx.set(payload.data.endp, endp)
        return _token

    def _setup_sof_token(frame_no):
        def _sof(ctx, payload):
            ctx.set(payload.pid, TokenPID.SOF)
            ctx.set(payload.data.as_value(), frame_no)
        return _sof

    @parameterized.expand([
        ["setup00", _setup_token(TokenPID.SETUP, 0, 0),   testp.token_packet(testp.PID.SETUP, 0, 0)],
        ["out00",   _setup_token(TokenPID.OUT, 0, 0),     testp.token_packet(testp.PID.OUT, 0, 0)],
        ["in00",    _setup_token(TokenPID.IN, 0, 0),      testp.token_packet(testp.PID.IN, 0, 0)],
        ["in01",    _setup_token(TokenPID.IN, 0, 1),      testp.token_packet(testp.PID.IN, 0, 1)],
        ["in10",    _setup_token(TokenPID.IN, 1, 0),      testp.token_packet(testp.PID.IN, 1, 0)],
        ["in7a",    _setup_token(TokenPID.IN, 0x70, 0xa), testp.token_packet(testp.PID.IN, 0x70, 0xa)],
        ["sof_min", _setup_sof_token(1),                  testp.sof_packet(1)],
        ["sof_max", _setup_sof_token(2**11-1),            testp.sof_packet(2**11-1)],
    ])
    def test_usb_tokens(self, name, test_payload, test_ref):
        """
        Verify our USBTokenPacketGenerator emits exactly the same bits
        as LUNA's test packet reference library.
        """

        dut = DomainRenamer({"usb": "sync"})(
            USBTokenPacketGenerator())

        async def testbench(ctx):
            data = []
            ctx.set(dut.tx.ready, 1)
            test_payload(ctx, dut.i.payload)
            ctx.set(dut.i.valid, 1)
            await ctx.tick()
            while ctx.get(dut.tx.valid):
                data.append(int(ctx.get(dut.tx.data)))
                await ctx.tick()
            print("[packet]", [hex(d) for d in data])
            bs = ("{0:08b}".format(data[0])[::-1] +
                  "{0:08b}".format(data[1])[::-1] +
                  "{0:08b}".format(data[2])[::-1])
            print("[ref]", test_ref)
            print("[got]", bs)
            self.assertEqual(bs, test_ref)

        sim = Simulator(dut)
        sim.add_clock(1e-6)
        sim.add_testbench(testbench)
        with sim.write_vcd(vcd_file=open(f"test_usb_token_{name}.vcd", "w")):
            sim.run()


class SetupPayloadTests(unittest.TestCase):

    @parameterized.expand([
        ["get_descriptor",    SetupPayload.get_descriptor(int(DescriptorTypes.DEVICE), 0, 0, 0x40),
                              [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00]],
        ["set_address",       SetupPayload.set_address(0x12),
                              [0x00, 0x05, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00]],
        ["set_configuration", SetupPayload.set_configuration(1),
                              [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]],
    ])
    def test_setup_payload(self, name, payload, ref):
        """
        Verify SetupPayload produces the same bits measured using Cynthion on the wire.
        """
        self.assertEqual(payload, ref)
