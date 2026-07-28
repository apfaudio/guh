# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: CERN-OHL-S-2.0

"""
Gearbox utils: streaming byte <-> word conversion, packet unframing.
"""

from amaranth import *
from amaranth.lib import stream, wiring
from amaranth.lib.wiring import In, Out

from luna.gateware.stream.future import Packet


class Pack(wiring.Component):

    """Pack a byte stream into ``word_width``-bit words, LSB first."""

    def __init__(self, word_width=32):
        assert word_width % 8 == 0
        self._n = word_width // 8
        super().__init__({
            "i": In(stream.Signature(unsigned(8))),
            "o": Out(stream.Signature(unsigned(word_width))),
        })

    def elaborate(self, platform):
        m = Module()
        n     = self._n
        lane  = Signal(range(n))
        accum = Signal(8 * (n - 1))     # lower n-1 bytes; top byte forwarded live
        emit  = lane == (n - 1)
        byte  = self.i.payload
        m.d.comb += [
            self.o.payload.eq(Cat(accum, byte)),
            self.o.valid.eq(self.i.valid & emit),
            self.i.ready.eq(Mux(emit, self.o.ready, 1)),
        ]
        with m.If(self.i.valid & self.i.ready):
            m.d.sync += lane.eq(Mux(emit, 0, lane + 1))
            with m.If(~emit):
                m.d.sync += accum.word_select(lane, 8).eq(byte)
        return m


class Unpack(wiring.Component):

    """Unpack a ``word_width``-bit word stream into bytes, LSB first."""

    def __init__(self, word_width=32):
        assert word_width % 8 == 0
        self._n = word_width // 8
        super().__init__({
            "i": In(stream.Signature(unsigned(word_width))),
            "o": Out(stream.Signature(unsigned(8))),
        })

    def elaborate(self, platform):
        m = Module()
        n    = self._n
        lane = Signal(range(n))
        last = lane == (n - 1)
        m.d.comb += [
            self.o.payload.eq(self.i.payload.word_select(lane, 8)),
            self.o.valid.eq(self.i.valid),
            self.i.ready.eq(self.o.ready & last),
        ]
        with m.If(self.o.valid & self.o.ready):
            m.d.sync += lane.eq(Mux(last, 0, lane + 1))
        return m


class Unframe(wiring.Component):

    """Adapt a ``Packet`` stream to a plain one, framing re-exposed as strobes."""

    def __init__(self, shape=unsigned(8)):
        super().__init__({
            "i":     In(stream.Signature(Packet(shape))),
            "o":     Out(stream.Signature(shape)),
            # High for the one cycle the packet's first / last token transfers,
            # so they may be used directly as strobes.
            "first": Out(1),
            "last":  Out(1),
        })

    def elaborate(self, platform):
        m = Module()
        xfer = self.i.valid & self.i.ready
        m.d.comb += [
            self.o.payload.eq(self.i.payload.data),
            self.o.valid.eq(self.i.valid),
            self.i.ready.eq(self.o.ready),
            self.first.eq(xfer & self.i.payload.first),
            self.last.eq(xfer & self.i.payload.last),
        ]
        return m
