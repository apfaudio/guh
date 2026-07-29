# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB Mass Storage Class engine.

Enumerates USB mass storage devices and provides a simple streaming
read interface for user gateware, to fetch desired raw blocks.

A lot of this comes from the various sources linked at:

    https://www.downtowndougbrown.com/2018/12/usb-mass-storage-with-embedded-devices-tips-and-quirks/

"""

from amaranth import *
from amaranth.lib import data, enum, fifo, stream, wiring
from amaranth.lib.cdc import ResetInserter
from amaranth.lib import memory
from amaranth.lib.wiring import In, Out

from luna.gateware.stream.future import Packet

from guh.usbh.enumerator import USBHostEnumerator
from guh.usbh.sie import TransferType, TransferResponse, DataPID
from guh.usbh.descriptor import USBDescriptorParser, EndpointFilter
from guh.protocol.descriptors import *

# ============================================================
# SCSI command / wrapper data structures
# ============================================================

CBW_SIGNATURE = 0x43425355
CSW_SIGNATURE = 0x53425355

class SCSIOpCode(enum.Enum, shape=unsigned(8)):
    TEST_UNIT_READY  = 0x00
    REQUEST_SENSE    = 0x03
    READ_CAPACITY_10 = 0x25
    READ_10          = 0x28
    WRITE_10         = 0x2A


class CBWFlags(enum.Enum, shape=unsigned(8)):
    DATA_OUT = 0x00  # Host to device
    DATA_IN  = 0x80  # Device to host


class CSWStatus(enum.Enum, shape=unsigned(8)):
    PASSED      = 0x00
    FAILED      = 0x01
    PHASE_ERROR = 0x02

class CDB6(data.Struct):
    opcode:   unsigned(8)
    _misc:    unsigned(32)
    control:  unsigned(8)
    _padding: unsigned(80)


class CDB10(data.Struct):
    opcode:      unsigned(8)
    flags:       unsigned(8)
    lba_be:      unsigned(32)  # big-endian
    group:       unsigned(8)
    xfer_len_be: unsigned(16)  # big-endian
    control:     unsigned(8)
    _padding:    unsigned(48)


class CBW(data.Struct):
    dCBWSignature:          unsigned(32)
    dCBWTag:                unsigned(32)
    dCBWDataTransferLength: unsigned(32)
    bmCBWFlags:             unsigned(8)
    bCBWLUN:                unsigned(4)
    _reserved1:             unsigned(4)
    bCBWCBLength:           unsigned(5)
    _reserved2:             unsigned(3)
    CBWCB:                  data.UnionLayout({
        "cdb6":  CDB6,
        "cdb10": CDB10,
    })


class CSW(data.Struct):
    dCSWSignature:   unsigned(32)
    dCSWTag:         unsigned(32)
    dCSWDataResidue: unsigned(32)
    bCSWStatus:      unsigned(8)


class ReadCapacity10Response(data.Struct):
    last_lba_be:   unsigned(32)  # big-endian
    block_size_be: unsigned(32)  # big-endian

CBW_SIZE_BYTES = CBW.as_shape().size // 8
CSW_SIZE_BYTES = CSW.as_shape().size // 8
READ_CAPACITY_SIZE_BYTES = ReadCapacity10Response.as_shape().size // 8


def byteswap(value):
    value = Value.cast(value)
    assert len(value) % 8 == 0
    return Cat(value[i:i+8] for i in reversed(range(0, len(value), 8)))

# ============================================================
# USB MSC / SCSI Command Wrapper Engine
# ============================================================

class SCSIBulkHost(wiring.Component):

    """
    SCSI command wrapper transport engine (bulk-only BBB).
    Issues CBWs, parses CSWs (protocol which encapsulates the actual commands)
    On read ops, data is streamed out rx_data.
    On write ops (cmd.dir_out), data is streamed in from tx_data; exactly
    cmd.data_len bytes are consumed on success. On failure, some prefix of
    the data may have been consumed - the caller must flush its data source.

    TODO: drop non-streaming mode and punt this to higher layers.
    TODO: handle more error conditions.
    """

    # HS bulk packets are up to 512 bytes. Both the internal rx FIFO and the
    # SIE tx FIFO must be able to hold a whole packet.
    MAX_BULK_PACKET_BYTES = 512

    class Command(data.Struct):
        start:       unsigned(1)
        data_len:    unsigned(32)
        dir_out:     unsigned(1) # 1=data phase is host-to-device (from tx_data)
        stream_data: unsigned(1) # 0=capture to self.captured, 1=stream to rx_data
                                 # TODO: probably cleaner to drop self.captured...
        cdb:         data.UnionLayout({
            "cdb6":  CDB6,
            "cdb10": CDB10,
        })

    class Status(data.Struct):
        idle:     unsigned(1)
        done:     unsigned(1)
        error:    unsigned(1)
        rejected: unsigned(1)
        aborted:  unsigned(1)  # transport fault, device phase state unknown
        timeout:  unsigned(1)  # rejection was a timeout, i.e. no response at all

    cmd:      In(Command)
    status:   Out(Status)
    rx_data:  Out(stream.Signature(Packet(unsigned(8))))
    tx_data:  In(stream.Signature(unsigned(8)))
    captured: Out(ReadCapacity10Response)

    def __init__(self, **kwargs):
        self.enumerator = USBHostEnumerator(
            **kwargs,
            config_number=1,
            # The SIE tx FIFO must hold a whole preloaded bulk OUT packet.
            fifo_depth=self.MAX_BULK_PACKET_BYTES,
            parser=USBDescriptorParser(
                endpoint_filter=EndpointFilter.IN_AND_OUT,
                transfer_type=EndpointTransferType.BULK,
                interface_class=InterfaceClass.MASS_STORAGE,
                interface_subclass=MSCSubClass.SCSI_TRANSPARENT,
                interface_protocol=MSCProtocol.BULK_ONLY,
            ),
        )
        super().__init__()

    def elaborate(self, platform):
        m = Module()

        m.submodules.enumerator = enum = self.enumerator
        packet_layout = Packet(unsigned(8))

        # The rx FIFO must always be able to absorb a whole packet
        # (see DATA state).
        RX_FIFO_DEPTH = 600
        assert RX_FIFO_DEPTH >= self.MAX_BULK_PACKET_BYTES
        m.submodules.rx_fifo = rx_fifo = DomainRenamer("usb")(fifo.SyncFIFOBuffered(
            width=packet_layout.size, depth=RX_FIFO_DEPTH))
        wiring.connect(m, rx_fifo.r_stream, wiring.flipped(self.rx_data))

        cbw_tag = Signal(32, init=1)
        tx_byte_idx = Signal(range(CBW_SIZE_BYTES))
        rx_byte_idx = Signal(10)
        rx_data_count = Signal(16)
        data_len = Signal(32)

        csw_sig = Signal(CSW)
        csw_flat = csw_sig.as_value()
        captured_sig = Signal(ReadCapacity10Response)
        captured_flat = captured_sig.as_value()
        m.d.comb += self.captured.eq(captured_sig)

        endp_in = enum.parser.o.i_endp.number
        endp_out = enum.parser.o.o_endp.number
        pid_in = Signal(DataPID, init=DataPID.DATA0)
        pid_out = Signal(DataPID, init=DataPID.DATA0)

        rx_packet = packet_layout(rx_fifo.w_stream.payload)
        stream_mode = Signal()
        dir_out = Signal()

        # OUT packets are chunked by the OUT endpoint's wMaxPacketSize and
        # mirrored into a replay buffer, since the SIE drains its tx FIFO
        # after every transaction, successful or not.
        mps_out = enum.parser.o.o_endp_mps.size
        tx_sent = Signal(32)
        pkt_len = Signal(range(self.MAX_BULK_PACKET_BYTES + 1))
        out_idx = Signal(range(self.MAX_BULK_PACKET_BYTES + 1))
        m.submodules.replay_mem = replay_mem = memory.Memory(
            shape=unsigned(8), depth=self.MAX_BULK_PACKET_BYTES, init=[])
        replay_wr = replay_mem.write_port(domain="usb")
        replay_rd = replay_mem.read_port(domain="usb")

        # Build CBW from command
        cbw_sig = Signal(CBW)
        cdb_opcode = CDB6(self.cmd.cdb.cdb6).opcode
        cdb_len = Signal(5)
        m.d.comb += cdb_len.eq(Mux(
            (cdb_opcode == SCSIOpCode.TEST_UNIT_READY) | (cdb_opcode == SCSIOpCode.REQUEST_SENSE),
            6, 10))

        m.d.comb += [
            cbw_sig.dCBWSignature.eq(CBW_SIGNATURE),
            cbw_sig.dCBWTag.eq(cbw_tag),
            cbw_sig.dCBWDataTransferLength.eq(self.cmd.data_len),
            cbw_sig.bmCBWFlags.eq(Mux((self.cmd.data_len > 0) & ~self.cmd.dir_out,
                                      CBWFlags.DATA_IN, CBWFlags.DATA_OUT)),
            cbw_sig.bCBWCBLength.eq(cdb_len),
            cbw_sig.CBWCB.eq(self.cmd.cdb),
        ]

        cbw_flat = cbw_sig.as_value()
        cbw_byte_out = Signal(8)
        m.d.comb += cbw_byte_out.eq((cbw_flat >> (tx_byte_idx * 8)) & 0xFF)

        def start_bulk_out(endp):
            return [
                enum.ctrl.xfer.start.eq(1),
                enum.ctrl.xfer.type.eq(TransferType.OUT),
                enum.ctrl.xfer.data_pid.eq(pid_out),
                enum.ctrl.xfer.dev_addr.eq(enum.status.dev_addr),
                enum.ctrl.xfer.ep_addr.eq(endp),
            ]

        def start_bulk_in(endp):
            return [
                enum.ctrl.xfer.start.eq(1),
                enum.ctrl.xfer.type.eq(TransferType.IN),
                enum.ctrl.xfer.data_pid.eq(pid_in),
                enum.ctrl.xfer.dev_addr.eq(enum.status.dev_addr),
                enum.ctrl.xfer.ep_addr.eq(endp),
            ]

        m.d.comb += self.status.idle.eq(0)

        with m.FSM(domain="usb"):

            with m.State("WAIT-ENUMERATION"):
                with m.If(enum.status.enumerated & enum.parser.o.valid):
                    m.next = "IDLE"

            with m.State("IDLE"):
                m.d.comb += self.status.idle.eq(1)
                with m.If(self.cmd.start):
                    m.d.usb += [
                        tx_byte_idx.eq(0),
                        data_len.eq(self.cmd.data_len),
                        stream_mode.eq(self.cmd.stream_data),
                        dir_out.eq(self.cmd.dir_out),
                    ]
                    m.next = "CBW-LOAD"

            with m.State("CBW-LOAD"):
                m.d.comb += [
                    enum.ctrl.txs.valid.eq(1),
                    enum.ctrl.txs.payload.eq(cbw_byte_out),
                ]
                with m.If(enum.ctrl.txs.ready):
                    m.d.usb += tx_byte_idx.eq(tx_byte_idx + 1)
                    with m.If(tx_byte_idx == CBW_SIZE_BYTES - 1):
                        m.d.usb += tx_byte_idx.eq(0)
                        m.next = "CBW-XFER"

            with m.State("CBW-XFER"):
                with m.If(enum.ctrl.status.idle):
                    m.d.comb += start_bulk_out(endp_out)
                    m.next = "CBW-WAIT"

            with m.State("CBW-WAIT"):
                with m.If(enum.ctrl.status.idle):
                    with m.Switch(enum.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.d.usb += [
                                pid_out.eq(Mux(pid_out, DataPID.DATA0, DataPID.DATA1)),
                                rx_byte_idx.eq(0),
                                rx_data_count.eq(0),
                            ]
                            with m.If(data_len == 0):
                                m.next = "CSW"
                            with m.Elif(dir_out):
                                m.d.usb += [
                                    tx_sent.eq(0),
                                    out_idx.eq(0),
                                    pkt_len.eq(Mux(data_len > mps_out,
                                                   mps_out, data_len)),
                                ]
                                m.next = "DATA-OUT-LOAD"
                            with m.Else():
                                m.next = "DATA"
                        with m.Case(TransferResponse.NAK):
                            # Pure flow control: the device never accepted the
                            # command, so resend the same CBW with the same
                            # data toggle.
                            m.next = "CBW-LOAD"
                        with m.Default():
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.rejected.eq(1),
                                self.status.timeout.eq(
                                    enum.ctrl.status.response == TransferResponse.TIMEOUT),
                            ]
                            m.next = "IDLE"

            with m.State("DATA"):
                # Only issue the next bulk-IN token once rx_fifo has room for
                # a full packet: the receive path can't pause mid-packet, so
                # less headroom would silently drop bytes under downstream
                # backpressure. (Captured non-stream reads bypass the FIFO.)
                rx_has_room = Signal()
                m.d.comb += rx_has_room.eq(
                    (rx_fifo.depth - rx_fifo.level) >= self.MAX_BULK_PACKET_BYTES)
                with m.If(enum.ctrl.status.idle & (rx_has_room | ~stream_mode)):
                    m.d.comb += start_bulk_in(endp_in)
                    m.next = "DATA-RX"

            with m.State("DATA-RX"):
                with m.If(stream_mode):
                    m.d.comb += [
                        enum.ctrl.rxs.ready.eq(rx_fifo.w_stream.ready),
                        rx_fifo.w_stream.valid.eq(enum.ctrl.rxs.valid),
                        rx_packet.data.eq(enum.ctrl.rxs.payload),
                        rx_packet.first.eq(rx_data_count == 0),
                        rx_packet.last.eq(rx_data_count == (data_len - 1)),
                    ]
                with m.Else():
                    m.d.comb += enum.ctrl.rxs.ready.eq(1)
                    with m.If(enum.ctrl.rxs.valid):
                        m.d.usb += captured_flat.word_select(rx_byte_idx, 8).eq(enum.ctrl.rxs.payload)

                with m.If(enum.ctrl.rxs.valid & enum.ctrl.rxs.ready):
                    m.d.usb += [
                        rx_byte_idx.eq(rx_byte_idx + 1),
                        rx_data_count.eq(rx_data_count + 1),
                    ]

                with m.If(enum.ctrl.status.idle):
                    with m.Switch(enum.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.d.usb += pid_in.eq(Mux(pid_in, DataPID.DATA0, DataPID.DATA1))
                            with m.If(rx_data_count >= data_len):
                                m.d.usb += rx_byte_idx.eq(0)
                                m.next = "CSW"
                            with m.Else():
                                m.next = "DATA"
                        with m.Case(TransferResponse.NAK):
                            m.next = "DATA"
                        with m.Default():
                            # STALL / CRC_ERROR / TIMEOUT / RX_OVERFLOW. Some
                            # data phase bytes may already be downstream, so
                            # this is an error, not a retryable rejection.
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.error.eq(1),
                                self.status.aborted.eq(1),
                            ]
                            m.next = "IDLE"

            with m.State("DATA-OUT-LOAD"):
                m.d.comb += [
                    enum.ctrl.txs.valid.eq(self.tx_data.valid),
                    enum.ctrl.txs.payload.eq(self.tx_data.payload),
                    self.tx_data.ready.eq(enum.ctrl.txs.ready),
                    replay_wr.addr.eq(out_idx),
                    replay_wr.data.eq(self.tx_data.payload),
                    replay_wr.en.eq(self.tx_data.valid & enum.ctrl.txs.ready),
                ]
                with m.If(self.tx_data.valid & enum.ctrl.txs.ready):
                    m.d.usb += out_idx.eq(out_idx + 1)
                    with m.If(out_idx == pkt_len - 1):
                        m.d.usb += out_idx.eq(0)
                        m.next = "DATA-OUT-XFER"

            with m.State("DATA-OUT-XFER"):
                with m.If(enum.ctrl.status.idle):
                    m.d.comb += start_bulk_out(endp_out)
                    m.next = "DATA-OUT-WAIT"

            with m.State("DATA-OUT-WAIT"):
                sent_next = tx_sent + pkt_len
                remaining_next = data_len - sent_next
                with m.If(enum.ctrl.status.idle):
                    with m.Switch(enum.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.d.usb += [
                                pid_out.eq(Mux(pid_out, DataPID.DATA0, DataPID.DATA1)),
                                tx_sent.eq(sent_next),
                            ]
                            with m.If(sent_next >= data_len):
                                m.d.usb += rx_byte_idx.eq(0)
                                m.next = "CSW"
                            with m.Else():
                                m.d.usb += pkt_len.eq(Mux(remaining_next > mps_out,
                                                          mps_out, remaining_next))
                                m.next = "DATA-OUT-LOAD"
                        with m.Case(TransferResponse.STALL):
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.rejected.eq(1),
                            ]
                            m.next = "IDLE"
                        with m.Default():
                            # NAK/TIMEOUT/etc: retransmit same packet, same PID.
                            # Unbounded - devices NAK for ages while committing
                            # to flash; the engine watchdog is the backstop.
                            m.next = "DATA-OUT-REFILL"

            with m.State("DATA-OUT-REFILL"):
                # The SIE drained its tx FIFO before reporting idle and its
                # depth >= max packet, so txs.ready is guaranteed high here.
                # Sync read port: data lags addr by one cycle.
                m.d.comb += replay_rd.addr.eq(out_idx)
                m.d.usb += out_idx.eq(out_idx + 1)
                with m.If(out_idx != 0):
                    m.d.comb += [
                        enum.ctrl.txs.valid.eq(1),
                        enum.ctrl.txs.payload.eq(replay_rd.data),
                    ]
                with m.If(out_idx == pkt_len):
                    m.d.usb += out_idx.eq(0)
                    m.next = "DATA-OUT-XFER"

            with m.State("CSW"):
                with m.If(enum.ctrl.status.idle):
                    m.d.comb += start_bulk_in(endp_in)
                    m.d.usb += rx_byte_idx.eq(0)
                    m.next = "CSW-RX"

            with m.State("CSW-RX"):
                m.d.comb += enum.ctrl.rxs.ready.eq(1)
                with m.If(enum.ctrl.rxs.valid):
                    m.d.usb += [
                        csw_flat.word_select(rx_byte_idx, 8).eq(enum.ctrl.rxs.payload),
                        rx_byte_idx.eq(rx_byte_idx + 1),
                    ]

                with m.If(enum.ctrl.status.idle):
                    with m.Switch(enum.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.d.usb += pid_in.eq(Mux(pid_in, DataPID.DATA0, DataPID.DATA1))
                            with m.If(rx_byte_idx == 0):
                                # Zero-length packet, not a CSW: toggle and
                                # poll again.
                                m.next = "CSW"
                            with m.Else():
                                m.d.usb += cbw_tag.eq(cbw_tag + 1)
                                m.d.comb += [
                                    self.status.done.eq(1),
                                    self.status.error.eq(csw_sig.bCSWStatus != CSWStatus.PASSED),
                                ]
                                m.next = "IDLE"
                        with m.Case(TransferResponse.NAK):
                            m.next = "CSW"
                        with m.Default():
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.error.eq(1),
                                self.status.aborted.eq(1),
                            ]
                            m.next = "IDLE"

        return m


# ============================================================
# (the actual high level) USB MSC Engine
# ============================================================

# Max blocks per READ_10 / WRITE_10 transfer.
MAX_BLOCKS_PER_XFER = 64
assert MAX_BLOCKS_PER_XFER <= 255  # must fit the low byte of xfer_len_be


class USBMSCHost(wiring.Component):
    """
    USB Mass Storage Class Host - block device interface.

    Performs MSC-specific SCSI initialization (TEST UNIT READY, READ CAPACITY)
    before accepting block read/write commands.

    Usage:
    1. Wait for status.ready == 1
    2. Check status.block_count and status.block_size for device capacity
    3. Set cmd.lba to desired block address (and cmd.write for writes) and
       strobe cmd.start
    4. Reads: if the read succeeds, up to status.block_size*(cmd.block_count+1)
       bytes are streamed out on rx_data. Block fetches the device rejects
       outright are retried up to 5x before we bail (some msc devices will
       reject sequential block fetches 1-2x depending on where you are fetching
       from, I haven't read the spec thoroughly, just found this empirically).
       Failures partway through a read are not retried - some bytes are already
       on rx_data, so the caller must flush its own sink.
       Writes: the same byte count is consumed from tx_data. Failed writes are
       NOT retried (the data stream was already consumed); on resp.error the
       caller must flush its own data source before the next command.
    5. Check resp.done and resp.error. resp.error is not very helpful, but
       at least you know if something failed. For debugging resp.error
       your next step is usually a USB analyzer :)

    TODO: exponential backoff instead of dumb retries?
    """

    class Status(data.Struct):
        connected:   unsigned(1)     # device enumerated
        ready:       unsigned(1)     # device ready for block ops
        busy:        unsigned(1)     # scsi op in progress
        block_size:  unsigned(16)    # block size in bytes (typically 512)
        block_count: unsigned(32)    # total number of blocks

    class Command(data.Struct):
        start:       unsigned(1)     # Strobe to begin transfer
        write:       unsigned(1)     # 0 = READ_10, 1 = WRITE_10
        lba:         unsigned(32)    # starting block address
        block_count: range(MAX_BLOCKS_PER_XFER)  # blocks per transfer, off-by-one: 0 = 1 block

    class Response(data.Struct):
        done:  unsigned(1)           # Transfer complete (strobed for 1 cycle)
        error: unsigned(1)           # CSW indicated failure

    _WATCHDOG_CYCLES = 10 * 60000000  # ~10 seconds at 60MHz
                                      # Some things (like SSDs) can take >5sec to start emitting blocks.
    _INIT_RETRY_MAX = 10
    _READ_RETRY_MAX = 5               # READ_10 retries before reporting failure
    _RETRY_DELAY_CYCLES = 2048        # ~34 µs at 60 MHz, post-CSW settling time
    _DEFAULT_BLOCK_SIZE_BYTES = 512   # vast majority of block devices use 512-byte blocks

    status:  Out(Status)
    cmd:     In(Command)
    resp:    Out(Response)
    recovered: Out(1)  # strobed whenever the engine resets itself to recover
    rx_data: Out(stream.Signature(Packet(unsigned(8))))
    tx_data: In(stream.Signature(unsigned(8)))

    def __init__(self, *, bus=None, handle_clocking=True, device_address=0x12):
        self.scsi = SCSIBulkHost(
            bus=bus,
            handle_clocking=handle_clocking,
            device_address=device_address,
        )
        super().__init__()

    @property
    def sie(self):
        """Expose internal SIE for bus forwarding / testing."""
        return self.scsi.enumerator.sie

    def elaborate(self, platform):
        m = Module()

        m.submodules.scsi = scsi = self.scsi
        enum = scsi.enumerator

        wiring.connect(m, scsi.rx_data, wiring.flipped(self.rx_data))
        wiring.connect(m, wiring.flipped(self.tx_data), scsi.tx_data)

        block_size = Signal(16, init=self._DEFAULT_BLOCK_SIZE_BYTES)
        block_count = Signal(32)
        current_lba = Signal(32)
        current_write = Signal()
        current_block_count = Signal.like(self.cmd.block_count)  # off-by-one encoded
        init_retry  = Signal(range(self._INIT_RETRY_MAX + 1))
        read_retry  = Signal(range(self._READ_RETRY_MAX + 1))
        retry_timer = Signal(range(self._RETRY_DELAY_CYCLES + 1))

        xfer_blocks = Signal(range(MAX_BLOCKS_PER_XFER + 1))  # decoded: 1..MAX_BLOCKS_PER_XFER
        m.d.comb += xfer_blocks.eq(current_block_count + 1)

        watchdog = Signal(32)
        watchdog_expired = Signal()
        m.d.usb += watchdog.eq(watchdog + 1)
        m.d.comb += watchdog_expired.eq(watchdog >= (self._WATCHDOG_CYCLES - 1))

        # An aborted transfer leaves the device mid-phase, and we implement
        # neither BBB reset nor CLEAR_FEATURE, so re-enumerate (same for a
        # departed device: it comes back unaddressed). `enumerated` gates
        # the disconnect so we don't re-trigger on the way up.
        recover_req = Signal()
        with m.If((scsi.status.done & scsi.status.aborted) |
                  (enum.status.enumerated & enum.ctrl.status.disconnected)):
            m.d.usb += recover_req.eq(1)

        m.d.comb += [
            self.status.connected.eq(enum.status.enumerated),
            self.status.ready.eq(~self.status.busy),
            self.status.busy.eq(1),
            self.status.block_size.eq(block_size),
            self.status.block_count.eq(block_count),
        ]

        # Command setup helpers
        scsi_cmd = SCSIBulkHost.Command(scsi.cmd)
        cdb6 = CDB6(scsi_cmd.cdb.cdb6)
        cdb10 = CDB10(scsi_cmd.cdb.cdb10)

        # Shared by the issue and -WAIT states of each command below.
        read_capacity_setup = [
            cdb10.opcode.eq(SCSIOpCode.READ_CAPACITY_10),
            scsi_cmd.data_len.eq(READ_CAPACITY_SIZE_BYTES),
        ]
        xfer10_setup = [
            cdb10.opcode.eq(Mux(current_write, SCSIOpCode.WRITE_10,
                                               SCSIOpCode.READ_10)),
            cdb10.lba_be.eq(byteswap(current_lba)),
            # xfer_blocks fits in low byte; high byte is always zero.
            cdb10.xfer_len_be.eq(Cat(Const(0, 8), xfer_blocks)),
            scsi_cmd.data_len.eq(block_size * xfer_blocks),
            scsi_cmd.stream_data.eq(1),
            scsi_cmd.dir_out.eq(current_write),
        ]

        with m.FSM(domain="usb"):

            with m.State("WAIT-ENUMERATION"):
                with m.If(scsi.status.idle):
                    m.d.usb += [watchdog.eq(0), init_retry.eq(0)]
                    m.next = "TEST-UNIT-READY"

            with m.State("TEST-UNIT-READY"):
                m.d.comb += [
                    cdb6.opcode.eq(SCSIOpCode.TEST_UNIT_READY),
                    scsi_cmd.data_len.eq(0),
                    scsi_cmd.stream_data.eq(0),
                    scsi_cmd.start.eq(1),
                ]
                m.next = "TEST-UNIT-READY-WAIT"

            with m.State("TEST-UNIT-READY-WAIT"):
                m.d.comb += cdb6.opcode.eq(SCSIOpCode.TEST_UNIT_READY)
                with m.If(scsi.status.done):
                    with m.If(~scsi.status.error & ~scsi.status.rejected):
                        m.d.usb += [watchdog.eq(0), init_retry.eq(0)]
                        m.next = "READ-CAPACITY"
                    with m.Else():
                        m.d.usb += init_retry.eq(init_retry + 1)
                        with m.If(init_retry >= self._INIT_RETRY_MAX):
                            m.next = "WAIT-ENUMERATION"
                        with m.Else():
                            m.next = "TEST-UNIT-READY"

            with m.State("READ-CAPACITY"):
                m.d.comb += read_capacity_setup + [scsi_cmd.start.eq(1)]
                m.next = "READ-CAPACITY-WAIT"

            with m.State("READ-CAPACITY-WAIT"):
                m.d.comb += read_capacity_setup
                with m.If(scsi.status.done):
                    with m.If(~scsi.status.error & ~scsi.status.rejected):
                        m.d.usb += watchdog.eq(0)
                        last_lba_le = byteswap(scsi.captured.last_lba_be)
                        blk_size_le = byteswap(scsi.captured.block_size_be)
                        m.d.usb += [
                            block_count.eq(last_lba_le + 1),
                            block_size.eq(blk_size_le[0:16]),
                        ]
                        m.next = "READY"
                    with m.Else():
                        m.next = "READ-CAPACITY"

            with m.State("READY"):
                m.d.comb += self.status.busy.eq(0)
                m.d.usb += watchdog.eq(0)
                with m.If(self.cmd.start):
                    m.d.usb += [
                        current_lba.eq(self.cmd.lba),
                        current_write.eq(self.cmd.write),
                        current_block_count.eq(self.cmd.block_count),
                        read_retry.eq(0),
                    ]
                    m.next = "XFER"

            with m.State("XFER"):
                m.d.comb += xfer10_setup + [scsi_cmd.start.eq(1)]
                m.next = "XFER-WAIT"

            with m.State("XFER-WAIT"):
                m.d.comb += xfer10_setup
                with m.If(scsi.status.done):
                    failed = scsi.status.error | scsi.status.rejected
                    # `rejected` without `error` is the only failure where no
                    # data phase ran; replaying one that did would double-stream
                    # into a consumer that already has its bytes. Writes are
                    # never retried, their tx_data source is already consumed.
                    retryable = scsi.status.rejected & ~scsi.status.error
                    with m.If(retryable & ~current_write &
                              (read_retry < self._READ_RETRY_MAX)):
                        m.d.usb += [
                            read_retry.eq(read_retry + 1),
                            retry_timer.eq(self._RETRY_DELAY_CYCLES),
                        ]
                        m.next = "XFER-RETRY-DELAY"
                    with m.Else():
                        # Success, or out of retry budget. Having burnt the
                        # whole budget on timeouts means nothing is answering
                        # at the address we enumerated, so assume it left.
                        with m.If(scsi.status.timeout):
                            m.d.usb += recover_req.eq(1)
                        m.d.usb += [watchdog.eq(0), read_retry.eq(0)]
                        m.d.comb += [
                            self.resp.done.eq(1),
                            self.resp.error.eq(failed),
                        ]
                        m.next = "READY"

            with m.State("XFER-RETRY-DELAY"):
                m.d.usb += retry_timer.eq(retry_timer - 1)
                with m.If(retry_timer == 0):
                    m.next = "XFER"

        # The reset below tears down any in-flight transfer without the FSM
        # ever completing, so report one here. Normal completions just
        # re-report; the peripheral retires on the state transition.
        with m.If(watchdog_expired | recover_req):
            m.d.comb += [
                self.resp.done.eq(1),
                self.resp.error.eq(1),
            ]
        m.d.comb += self.recovered.eq(watchdog_expired | recover_req)

        return ResetInserter({"usb": watchdog_expired | recover_req})(m)
