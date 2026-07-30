# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Wishbone DMA engine (initiator).
"""

from amaranth import *
from amaranth.lib import data, fifo, stream, wiring
from amaranth.lib.wiring import In, Out
from amaranth_soc import wishbone


class DMAEngine(wiring.Component):

    """
    Wishbone DMA engine. FIXME: hard-codes some 32-bit assumptions.

    Takes a ``self.cmd`` stream of one ``Descriptor`` per transfer. When a
    ``Descriptor`` is ingested, the core uses word-wide 'tx' and 'rx' streams
    (depending on the direction: write or read) to send/fetch data externally
    from this core and always buffer enough data to issue wishbone transactions
    in bursts which are ``burst_len`` long. Note that ``words`` of each command
    must therefore be a multiple of ``burst_len``.
    """

    class Descriptor(data.StructLayout):

        """Move ``words`` 32-bit words to/from a PSRAM ``addr``.
        ``words`` MUST be a multiple of the engine's ``burst_len``."""

        def __init__(self, *, addr_width, max_words):
            super().__init__({
                "write": unsigned(1),           # 1 = PSRAM -> device
                "addr":  unsigned(addr_width),  # PSRAM word address base
                "words": range(max_words + 1),  # 32-bit words to transfer
            })

    def __init__(self, *, addr_width, burst_len, max_words, stage_words=None,
                 data_width=32, granularity=8):
        # Default to a double-buffer (two bursts), so one burst can fill while
        # the other drains and bursts pipeline back-to-back.
        if stage_words is None:
            stage_words = burst_len * 2
        assert stage_words >= burst_len, "staging FIFO must hold at least one burst"
        assert max_words % burst_len == 0, "max_words must be a whole number of bursts"
        self._burst_len   = burst_len
        self._stage_words = stage_words
        self._data_width  = data_width
        self._max_words   = max_words
        self.descriptor   = self.Descriptor(addr_width=addr_width, max_words=max_words)
        super().__init__({
            "cmd":     In(stream.Signature(self.descriptor)),
            "abort":   In(1),
            "done":    Out(1),
            "rx":      In(stream.Signature(unsigned(data_width))),
            "tx":      Out(stream.Signature(unsigned(data_width))),
            "bus":     Out(wishbone.Signature(addr_width=addr_width, data_width=data_width,
                                              granularity=granularity, features={"cti", "bte"})),
        })

    def elaborate(self, platform) -> Module:
        m = Module()

        # Empties the staging FIFO on every new command (and on abort), so the
        # direction mux is exact and residue can't skew the next transfer.
        flush = Signal()
        m.submodules.stage = stage = ResetInserter(flush)(
            fifo.SyncFIFOBuffered(width=self._data_width, depth=self._stage_words))

        cur          = Signal(self.descriptor)
        dma_word_idx = Signal(range(self._max_words + 1))  # DMA cursor
        burst_idx    = Signal(range(self._burst_len))
        m.d.comb += [
            self.bus.sel.eq(2**len(self.bus.sel) - 1),  # all lanes
            self.done.eq(dma_word_idx == cur.words),
        ]

        # Direction mux: the burst FSM drives one FIFO port, the corresponding
        # word stream is wired to the other.
        fifo_r_en = Signal()   # drain a word to PSRAM   (block read)
        fifo_w_en = Signal()   # fill a word from PSRAM  (block write)
        with m.If(cur.write):
            m.d.comb += [
                stage.w_data.eq(self.bus.dat_r),
                stage.w_en.eq(fifo_w_en),
            ]
            wiring.connect(m, stage.r_stream, wiring.flipped(self.tx))
        with m.Else():
            m.d.comb += stage.r_en.eq(fifo_r_en)
            wiring.connect(m, wiring.flipped(self.rx), stage.w_stream)

        # A command is in flight from accept until it fully completes / aborts.
        busy      = Signal()
        # An abort may land mid-burst; latch it and honour it at the next burst
        # boundary (IDLE), so we never tear down cyc/stb inside a burst.
        abort_req = Signal()
        with m.If(self.abort & busy):
            m.d.sync += abort_req.eq(1)

        # For a block read this holds at `done`; for a block write it lags
        # until `tx` drains.
        fifo_empty = ~stage.r_rdy

        with m.FSM(name="dma"):

            with m.State("IDLE"):
                with m.If(~busy):
                    m.d.comb += self.cmd.ready.eq(1)
                    with m.If(self.cmd.valid):
                        m.d.comb += flush.eq(1)
                        m.d.sync += [
                            cur.eq(self.cmd.payload),
                            dma_word_idx.eq(0),
                            busy.eq(1),
                        ]
                with m.Elif(abort_req | (self.done & fifo_empty)):
                    # Retire the in-flight command: cancelled, or fully done.
                    # A clean completion leaves the FIFO empty already.
                    with m.If(abort_req):
                        m.d.comb += flush.eq(1)
                    m.d.sync += [busy.eq(0), abort_req.eq(0)]
                with m.Elif(~self.done):
                    with m.If(~cur.write &
                              (stage.level >= self._burst_len)):
                        m.d.sync += burst_idx.eq(0)
                        m.next = "BURST-WR"
                    with m.Elif(cur.write &
                                ((stage.depth - stage.level)
                                 >= self._burst_len)):
                        m.d.sync += burst_idx.eq(0)
                        m.next = "BURST-RD"

            # Block read: burst staged words from the FIFO into PSRAM.
            with m.State("BURST-WR"):
                last_beat = burst_idx == (self._burst_len - 1)
                m.d.comb += [
                    self.bus.cyc.eq(1),
                    self.bus.stb.eq(1),
                    self.bus.we.eq(1),
                    self.bus.adr.eq(cur.addr + dma_word_idx),
                    self.bus.dat_w.eq(stage.r_data),
                    self.bus.cti.eq(Mux(last_beat, wishbone.CycleType.END_OF_BURST,
                                                       wishbone.CycleType.INCR_BURST)),
                    fifo_r_en.eq(self.bus.ack),
                ]
                with m.If(self.bus.ack):
                    m.d.sync += [
                        dma_word_idx.eq(dma_word_idx + 1),
                        burst_idx.eq(burst_idx + 1),
                    ]
                    with m.If(last_beat):
                        m.next = "IDLE"

            # Block write: burst-prefetch PSRAM contents into the staging FIFO.
            with m.State("BURST-RD"):
                last_beat = burst_idx == (self._burst_len - 1)
                m.d.comb += [
                    self.bus.cyc.eq(1),
                    self.bus.stb.eq(1),
                    self.bus.adr.eq(cur.addr + dma_word_idx),
                    self.bus.cti.eq(Mux(last_beat, wishbone.CycleType.END_OF_BURST,
                                                       wishbone.CycleType.INCR_BURST)),
                    fifo_w_en.eq(self.bus.ack),
                ]
                with m.If(self.bus.ack):
                    m.d.sync += [
                        dma_word_idx.eq(dma_word_idx + 1),
                        burst_idx.eq(burst_idx + 1),
                    ]
                    with m.If(last_beat):
                        m.next = "IDLE"

        return m
