# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: CERN-OHL-S-2.0

"""
USB Mass Storage Class / DMA engine peripheral.

Provides a CSR interface for a SoC to enumerate
a block device, enqueue block read/write requests, and autonomously
execute them as burst transfers on a provided wishbone bus
(bus writes for block reads, bus reads for block writes).
"""

from amaranth import *
from amaranth.lib import data, fifo, stream, wiring
from amaranth.lib.wiring import In, Out
from amaranth_soc import csr, wishbone

from guh.engines.msc import USBMSCHost, MAX_BLOCKS_PER_XFER
from guh.periph.dma import DMAEngine
from guh.util.gearbox import Pack, Unpack, Unframe


# Block size is fixed rather than read from host.status.block_size: making it
# dynamic costs a fair bit of logic and every thumbdrive in existence uses
# 512-byte blocks, so this is 'dirty but works'.
_WORDS_PER_BLOCK   = USBMSCHost._DEFAULT_BLOCK_SIZE_BYTES // 4
MAX_TRANSFER_WORDS = _WORDS_PER_BLOCK * MAX_BLOCKS_PER_XFER   # per-command word cap


class Peripheral(wiring.Component):
    """
    USB MSC peripheral.

    The CPU can enqueue `fifo_depth` block transfer requests. Each request
    specifies a starting LBA, a PSRAM address, a block count
    (1..MAX_BLOCKS_PER_XFER) and a direction; the peripheral transfers N
    contiguous blocks between the device and PSRAM. Block reads DMA device
    data into PSRAM; block writes (cmd_dir.write=1) fetch data from PSRAM
    and send it to the device.

    The default settings - 512-byte blocks, max 64-block transfers and 8-deep
    command FIFO permits the CPU to enqueue up to 256KiB of transfers at
    a time. This is enough to saturate USB2 HS (~40MiB/sec) if an ISR
    services this peripheral every 5ms or so.

    Usage:
    1. Poll `status` register until `ready` is set (MSC device enumerated)
    2. Optionally check `capacity` and `block_size` for device info
    3. Write `cmd_lba`, `cmd_addr`, `cmd_blocks` and `cmd_dir` to set up
       a request
    4. Write 1 to `cmd_start` to enqueue the request
    5. For cmd completion checking, check the upcounters (warn: they wrap)
       `cmds_done` and `errors` as well as `status.fifo_empty` and
       `status.fifo_full`. Depending on whether your driver can handle
       multiple in-flight commands, you may only need to check a subset
       of these completion registers.

    WARN: for writes, the PSRAM source contents must not be modified until
    the command completes (the DMA engine reads them live).

    WARN: this currently assumes 'sync' and 'usb' clock domains are the same!
    """

    #
    # Write side: Control / Command registers
    #

    class CmdLbaReg(csr.Register, access="rw"):
        """starting LBA on the device."""
        lba: csr.Field(csr.action.RW, unsigned(32))

    class CmdAddrReg(csr.Register, access="rw"):
        """PSRAM address (byte address, low 2 bits ignored)."""
        addr: csr.Field(csr.action.RW, unsigned(32))

    class CmdBlocksReg(csr.Register, access="rw"):
        """N contiguous blocks to transfer, 1..MAX_BLOCKS_PER_XFER."""
        blocks: csr.Field(csr.action.RW, range(MAX_BLOCKS_PER_XFER+1), init=1)

    class CmdDirReg(csr.Register, access="rw"):
        """Direction: 0 = read (device to PSRAM), 1 = write (PSRAM to device)."""
        write: csr.Field(csr.action.RW, unsigned(1))

    class CmdStartReg(csr.Register, access="w"):
        """Write 1 to enqueue command with last LBA/addr/blocks/dir."""
        start: csr.Field(csr.action.W, unsigned(1))

    #
    # Read side: Status / command response registers
    #

    class StatusReg(csr.Register, access="r"):
        connected:  csr.Field(csr.action.R, unsigned(1))  # USB device enumerated
        ready:      csr.Field(csr.action.R, unsigned(1))  # MSC initialized, ready for commands
        busy:       csr.Field(csr.action.R, unsigned(1))  # Transfer in progress
        fifo_full:  csr.Field(csr.action.R, unsigned(1))  # Command FIFO full
        fifo_empty: csr.Field(csr.action.R, unsigned(1))  # Command FIFO empty (all done)
        error:      csr.Field(csr.action.R, unsigned(1))  # Most recent completed command errored

    class CapacityReg(csr.Register, access="r"):
        """if status.ready: device capacity in blocks."""
        block_count: csr.Field(csr.action.R, unsigned(32))

    class BlockSizeReg(csr.Register, access="r"):
        """if status.ready: block size in bytes (typically 512)."""
        block_size: csr.Field(csr.action.R, unsigned(32))

    class CmdsDoneReg(csr.Register, access="r"):
        """Total commands completed, successfully or not (wrapping)."""
        count: csr.Field(csr.action.R, unsigned(32))

    class ErrorsReg(csr.Register, access="r"):
        """Total commands that completed with an error (wrapping)."""
        count: csr.Field(csr.action.R, unsigned(32))

    class CmdsSubmittedReg(csr.Register, access="r"):
        """Total commands enqueued (wrapping); the Nth completes at cmds_done == N."""
        count: csr.Field(csr.action.R, unsigned(32))

    class PlugEventsReg(csr.Register, access="r"):
        """Times the engine re-enumerated (wrapping); a change invalidates
        capacity, cached blocks and any queued commands."""
        count: csr.Field(csr.action.R, unsigned(32))

    #
    # Structure of internal command FIFO
    #

    class DMACommand(data.Struct):
        lba:       unsigned(32)               # msc lba
        addr_w:    unsigned(30)               # psram word address (cmd_addr >> 2)
        blocks_m1: range(MAX_BLOCKS_PER_XFER) # blocks to transfer - 1
        write:     unsigned(1)                # 1 = PSRAM -> device

    #
    # Constants
    #

    # PSRAM/DMA data path width. The rest of this core assumes 32-bit words;
    # the DMAEngine and the (un)packers are width-generic, so they take this
    # explicitly.
    _DMA_DATA_WIDTH   = 32

    # Length of each DMA burst. 8 words = 32 bytes per burst. Must divide
    # _WORDS_PER_BLOCK so a per-command word count is always a whole number of
    # bursts (the DMAEngine only issues whole bursts).
    _DMA_BURST_LEN    = 8

    # 8KiB = size of FIFO between USB and PSRAM DMA engines.
    # Empirically determined to be just enough to not cause backpressure on an SoC
    # design which is hammering PSRAM at the same time as this core.
    _DATA_FIFO_WORDS  = 8192 // 4

    def __init__(self, *, fifo_depth=8, addr_width=22, device_address=0x12,
                 msc_host=None):
        self.fifo_depth = fifo_depth
        self.addr_width = addr_width
        self.device_address = device_address
        # Test seam: inject a pre-built USBMSCHost (e.g. bus=None for sim, so
        # the testbench can reach its UTMI interface).
        self._msc_host = msc_host

        regs = csr.Builder(addr_width=6, data_width=8)
        self._status     = regs.add("status",     self.StatusReg(),    offset=0x00)
        self._capacity   = regs.add("capacity",   self.CapacityReg(),  offset=0x04)
        self._block_size = regs.add("block_size", self.BlockSizeReg(), offset=0x08)
        self._cmd_lba    = regs.add("cmd_lba",    self.CmdLbaReg(),    offset=0x0C)
        self._cmd_addr   = regs.add("cmd_addr",   self.CmdAddrReg(),   offset=0x10)
        self._cmd_start  = regs.add("cmd_start",  self.CmdStartReg(),  offset=0x14)
        self._cmds_done  = regs.add("cmds_done",  self.CmdsDoneReg(),  offset=0x18)
        self._errors     = regs.add("errors",     self.ErrorsReg(),    offset=0x1C)
        self._cmd_blocks = regs.add("cmd_blocks", self.CmdBlocksReg(), offset=0x20)
        self._cmd_dir    = regs.add("cmd_dir",    self.CmdDirReg(),    offset=0x24)
        self._plug_events = regs.add("plug_events", self.PlugEventsReg(), offset=0x28)
        self._cmds_submitted = regs.add("cmds_submitted", self.CmdsSubmittedReg(),
                                        offset=0x2C)

        self._bridge = csr.Bridge(regs.as_memory_map())

        super().__init__({
            "csr_bus": In(csr.Signature(addr_width=regs.addr_width, data_width=regs.data_width)),
            "dma_bus": Out(wishbone.Signature(addr_width=addr_width, data_width=32,
                                              granularity=8, features={"cti", "bte"})),
        })

        self.csr_bus.memory_map = self._bridge.bus.memory_map

    def elaborate(self, platform) -> Module:
        m = Module()

        if platform is not None and hasattr(platform, 'default_usb_connection'):
            ulpi_bus = platform.request(platform.default_usb_connection)
        else:
            ulpi_bus = None

        #
        # Submodules
        #

        if self._msc_host is not None:
            msc_host = self._msc_host
        else:
            msc_host = USBMSCHost(
                bus=ulpi_bus,
                handle_clocking=True,
                device_address=self.device_address,
            )
        m.submodules.msc_host = msc_host
        m.submodules.bridge = self._bridge
        m.submodules.cmd_fifo = cmd_fifo = fifo.SyncFIFOBuffered(
            width=self.DMACommand.as_shape().size, depth=self.fifo_depth)

        wiring.connect(m, wiring.flipped(self.csr_bus), self._bridge.bus)

        #
        # Command enqueue
        #

        cmd_payload = self.DMACommand(cmd_fifo.w_data)
        m.d.comb += [
            cmd_payload.lba.eq(self._cmd_lba.f.lba.data),
            cmd_payload.addr_w.eq(self._cmd_addr.f.addr.data[2:]),
            cmd_payload.blocks_m1.eq(self._cmd_blocks.f.blocks.data - 1),
            cmd_payload.write.eq(self._cmd_dir.f.write.data),
            # Enqueue while full is silently dropped - TODO increment errors?
            cmd_fifo.w_en.eq(self._cmd_start.f.start.w_stb & self._cmd_start.f.start.w_data),
        ]

        #
        # Status registers
        #

        error_flag     = Signal()
        cmds_done      = Signal(32)
        errors         = Signal(32)
        plug_events    = Signal(32)
        cmds_submitted = Signal(32)
        with m.If(msc_host.recovered):
            m.d.sync += plug_events.eq(plug_events + 1)
        with m.If(cmd_fifo.w_en & cmd_fifo.w_rdy):
            m.d.sync += cmds_submitted.eq(cmds_submitted + 1)
        m.d.comb += [
            self._status.f.connected.r_data.eq(msc_host.status.connected),
            self._status.f.ready.r_data.eq(msc_host.status.ready),
            self._status.f.busy.r_data.eq(msc_host.status.busy),
            self._status.f.fifo_full.r_data.eq(~cmd_fifo.w_rdy),
            self._status.f.fifo_empty.r_data.eq(~cmd_fifo.r_rdy),
            self._status.f.error.r_data.eq(error_flag),
            self._capacity.f.block_count.r_data.eq(msc_host.status.block_count),
            self._block_size.f.block_size.r_data.eq(msc_host.status.block_size),
            self._cmds_done.f.count.r_data.eq(cmds_done),
            self._errors.f.count.r_data.eq(errors),
            self._plug_events.f.count.r_data.eq(plug_events),
            self._cmds_submitted.f.count.r_data.eq(cmds_submitted),
        ]

        #
        # Request / DMA: a SCSI/command FSM here, plus the DMAEngine submodule
        # (transfer FIFO + Wishbone master). Only the byte<->word packing lives
        # up here, on the host side of the engine's word streams.
        #

        # The engine's transfer FIFO is sized here to double as the large
        # USB<->PSRAM slack buffer.
        m.submodules.dma = dma = DMAEngine(
            addr_width=self.addr_width, burst_len=self._DMA_BURST_LEN,
            max_words=MAX_TRANSFER_WORDS,
            stage_words=self._DATA_FIFO_WORDS, data_width=self._DMA_DATA_WIDTH)
        wiring.connect(m, wiring.flipped(self.dma_bus), dma.bus)

        # `flush` restarts the (un)packer byte lanes on hand-off/abort and on
        # each rx packet's last byte (a no-op for unpack, only one direction is
        # ever in flight), so a mis-sized packet can't misalign the rest.
        data_flush = Signal()   # command hand-off / abort
        flush      = Signal()   # packer byte-lane reset
        m.submodules.unframe = unframe = Unframe(unsigned(8))
        m.submodules.pack    = pack    = ResetInserter(flush)(
            Pack(self._DMA_DATA_WIDTH))
        m.submodules.unpack  = unpack  = ResetInserter(flush)(
            Unpack(self._DMA_DATA_WIDTH))
        wiring.connect(m, msc_host.rx_data, unframe.i)
        wiring.connect(m, unframe.o, pack.i)
        m.d.comb += flush.eq(data_flush | unframe.last)
        wiring.connect(m, pack.o, dma.rx)
        wiring.connect(m, dma.tx, unpack.i)
        wiring.connect(m, unpack.o, msc_host.tx_data)

        #
        # SCSI/CMD FSM: RUN holds until both the bulk transfer and DMA drain
        # are done, then dequeues a new command.
        #

        # resp.done is a strobe; capture it.
        transfer_done  = Signal()
        transfer_error = Signal()
        with m.If(msc_host.resp.done):
            m.d.sync += [
                transfer_done.eq(1),
                transfer_error.eq(msc_host.resp.error),
            ]

        # view of the command being dequeued.
        fifo_cmd = self.DMACommand(cmd_fifo.r_data)

        with m.FSM(name="cmd"):

            with m.State("IDLE"):
                # `dma.cmd.ready` is high only when the engine has no command
                # in flight, so we only dequeue once the previous transfer has
                # fully completed or been aborted below.
                with m.If(cmd_fifo.r_rdy & msc_host.status.ready & dma.cmd.ready):
                    m.d.comb += [
                        cmd_fifo.r_en.eq(1),
                        data_flush.eq(1),
                        msc_host.cmd.lba.eq(fifo_cmd.lba),
                        msc_host.cmd.block_count.eq(fifo_cmd.blocks_m1),
                        msc_host.cmd.write.eq(fifo_cmd.write),
                        msc_host.cmd.start.eq(1),
                        dma.cmd.valid.eq(1),
                        dma.cmd.payload.write.eq(fifo_cmd.write),
                        dma.cmd.payload.addr.eq(fifo_cmd.addr_w),
                        # block -> word translation lives here, not in the engine.
                        dma.cmd.payload.words.eq(
                            (fifo_cmd.blocks_m1 + 1) * _WORDS_PER_BLOCK),
                    ]
                    m.d.sync += [
                        transfer_done.eq(0),
                        transfer_error.eq(0),
                    ]
                    m.next = "RUN"

            with m.State("RUN"):
                with m.If(transfer_done & (transfer_error | dma.done)):
                    # On error the DMA may still be in flight; abort it so the
                    # engine flushes and frees up for the next command. A clean
                    # completion self-retires.
                    with m.If(transfer_error):
                        m.d.comb += [dma.abort.eq(1), data_flush.eq(1)]
                    m.d.sync += [
                        cmds_done.eq(cmds_done + 1),
                        errors.eq(errors + transfer_error),
                        error_flag.eq(transfer_error),
                    ]
                    m.next = "IDLE"

        return m
