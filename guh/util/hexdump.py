# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Hex dump raw bytes to UART.
"""

from amaranth import *
from amaranth.lib import stream, wiring
from amaranth.lib.wiring import In, Out

from luna.gateware.interface.uart import UARTTransmitter
from luna.gateware.stream.future import Packet

class HexDump(wiring.Component):

    i: In(stream.Signature(Packet(unsigned(8))))
    tx: Out(unsigned(1))

    def __init__(self, *, divisor, bytes_per_line=16):
        self._divisor = divisor
        self._bytes_per_line = bytes_per_line
        super().__init__()

    def elaborate(self, platform):
        m = Module()

        m.submodules.uart = uart = UARTTransmitter(divisor=self._divisor)
        m.d.comb += self.tx.eq(uart.tx)

        hex_chars = Array([ord(c) for c in "0123456789ABCDEF"])

        byte_latch = Signal(8)
        byte_count = Signal(range(self._bytes_per_line + 1))

        with m.FSM():

            with m.State("IDLE"):
                m.d.comb += self.i.ready.eq(1)
                with m.If(self.i.valid):
                    m.d.sync += byte_latch.eq(self.i.payload.data)
                    m.next = "SEND-HIGH"

            with m.State("SEND-HIGH"):
                m.d.comb += [
                    uart.stream.valid.eq(1),
                    uart.stream.payload.eq(hex_chars[byte_latch[4:8]]),
                ]
                with m.If(uart.stream.ready):
                    m.next = "SEND-LOW"

            with m.State("SEND-LOW"):
                m.d.comb += [
                    uart.stream.valid.eq(1),
                    uart.stream.payload.eq(hex_chars[byte_latch[0:4]]),
                ]
                with m.If(uart.stream.ready):
                    m.d.sync += byte_count.eq(byte_count + 1)
                    with m.If(byte_count == (self._bytes_per_line - 1)):
                        m.d.sync += byte_count.eq(0)
                        m.next = "SEND-CR"
                    with m.Else():
                        m.next = "SEND-SPACE"

            with m.State("SEND-SPACE"):
                m.d.comb += [
                    uart.stream.valid.eq(1),
                    uart.stream.payload.eq(ord(' ')),
                ]
                with m.If(uart.stream.ready):
                    m.next = "IDLE"

            with m.State("SEND-CR"):
                m.d.comb += [
                    uart.stream.valid.eq(1),
                    uart.stream.payload.eq(ord('\r')),
                ]
                with m.If(uart.stream.ready):
                    m.next = "SEND-LF"

            with m.State("SEND-LF"):
                m.d.comb += [
                    uart.stream.valid.eq(1),
                    uart.stream.payload.eq(ord('\n')),
                ]
                with m.If(uart.stream.ready):
                    m.next = "IDLE"

        return m
