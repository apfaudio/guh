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

# ============================================================
# USB MSC / SCSI Command Wrapper Engine
# ============================================================

class SCSIBulkHost(wiring.Component):

    """
    SCSI command wrapper transport engine (bulk-only BBB).
    Issues CBWs, parses CSWs (protocol which encapsulates the actual commands)
    On read ops, data is streamed out rx_data.

    TODO: drop non-streaming mode and punt this to higher layers.
    TODO: handle more error conditions.
    """

    class Command(data.Struct):
        start:       unsigned(1)
        data_len:    unsigned(32)
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

    cmd:      In(Command)
    status:   Out(Status)
    rx_data:  Out(stream.Signature(Packet(unsigned(8))))
    captured: Out(ReadCapacity10Response)

    def __init__(self, **kwargs):
        self.enumerator = USBHostEnumerator(
            **kwargs,
            config_number=1,
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

        # RX FIFO with Packet framing (512 bytes + some margin)
        m.submodules.rx_fifo = rx_fifo = DomainRenamer("usb")(fifo.SyncFIFOBuffered(
            width=packet_layout.size, depth=600))
        wiring.connect(m, rx_fifo.r_stream, wiring.flipped(self.rx_data))

        cbw_tag = Signal(32, init=1)
        tx_byte_idx = Signal(6)
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
            cbw_sig.bmCBWFlags.eq(Mux(self.cmd.data_len > 0, CBWFlags.DATA_IN, CBWFlags.DATA_OUT)),
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
                            with m.If(data_len > 0):
                                m.next = "DATA"
                            with m.Else():
                                m.next = "CSW"
                        with m.Default():
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.rejected.eq(1),
                            ]
                            m.next = "IDLE"

            with m.State("DATA"):
                with m.If(enum.ctrl.status.idle):
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
                            m.d.usb += [
                                pid_in.eq(Mux(pid_in, DataPID.DATA0, DataPID.DATA1)),
                                cbw_tag.eq(cbw_tag + 1),
                            ]
                            m.d.comb += [
                                self.status.done.eq(1),
                                self.status.error.eq(csw_sig.bCSWStatus != CSWStatus.PASSED),
                            ]
                            m.next = "IDLE"
                        with m.Case(TransferResponse.NAK):
                            m.next = "CSW"

        return m


# ============================================================
# (the actual high level) USB MSC Engine
# ============================================================

class USBMSCHost(wiring.Component):
    """
    USB Mass Storage Class Host - read-only block device interface.

    Performs MSC-specific SCSI initialization (TEST UNIT READY, READ CAPACITY)
    before accepting block read commands.

    Usage:
    1. Wait for status.ready == 1
    2. Check status.block_count and status.block_size for device capacity
    3. Set cmd.lba to desired block address and strobe cmd.start
    4. If the read succeeds, status.block_size bytes are streamed out on rx_data
    5. Check resp.done and resp.error

    Eventually, this engine could be used to feed a pure-gateware DMA engine.

    TODO: exponential backoff instead of dumb retries?
    TODO: write support?
    TODO: cleaner way to do be/le conversion?
    """

    class Status(data.Struct):
        connected:   unsigned(1)     # device enumerated
        ready:       unsigned(1)     # device ready for block ops
        busy:        unsigned(1)     # scsi op in progress
        block_size:  unsigned(16)    # block size in bytes (typically 512)
        block_count: unsigned(32)    # total number of blocks

    class Command(data.Struct):
        start: unsigned(1)           # Strobe to begin read transfer
        lba:   unsigned(32)          # block address to read

    class Response(data.Struct):
        done:  unsigned(1)           # Transfer complete (strobed for 1 cycle)
        error: unsigned(1)           # CSW indicated failure

    _WATCHDOG_CYCLES = 10 * 60000000  # ~10 seconds at 60MHz
                                      # Some things (like SSDs) can take >5sec to start emitting blocks.
    _INIT_RETRY_MAX = 10
    _BLOCKS_PER_READ = 1

    status:  Out(Status)
    cmd:     In(Command)
    resp:    Out(Response)
    rx_data: Out(stream.Signature(Packet(unsigned(8))))

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

        block_size = Signal(16, init=512)
        block_count = Signal(32)
        current_lba = Signal(32)
        init_retry = Signal(range(self._INIT_RETRY_MAX + 1))

        watchdog = Signal(32)
        watchdog_expired = Signal()
        m.d.usb += watchdog.eq(watchdog + 1)
        m.d.comb += watchdog_expired.eq(watchdog >= (self._WATCHDOG_CYCLES - 1))

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
                m.d.comb += [
                    cdb10.opcode.eq(SCSIOpCode.READ_CAPACITY_10),
                    scsi_cmd.data_len.eq(READ_CAPACITY_SIZE_BYTES),
                    scsi_cmd.stream_data.eq(0),
                    scsi_cmd.start.eq(1),
                ]
                m.next = "READ-CAPACITY-WAIT"

            with m.State("READ-CAPACITY-WAIT"):
                m.d.comb += [
                    cdb10.opcode.eq(SCSIOpCode.READ_CAPACITY_10),
                    scsi_cmd.data_len.eq(READ_CAPACITY_SIZE_BYTES),
                ]
                with m.If(scsi.status.done):
                    with m.If(~scsi.status.error & ~scsi.status.rejected):
                        m.d.usb += watchdog.eq(0)
                        # big-endian (scsi) to little-endian (amaranth)
                        last_lba_le = Cat(
                            scsi.captured.last_lba_be[24:32], scsi.captured.last_lba_be[16:24],
                            scsi.captured.last_lba_be[8:16], scsi.captured.last_lba_be[0:8])
                        blk_size_le = Cat(
                            scsi.captured.block_size_be[24:32], scsi.captured.block_size_be[16:24],
                            scsi.captured.block_size_be[8:16], scsi.captured.block_size_be[0:8])
                        m.d.usb += [
                            block_count.eq(last_lba_le + 1),
                            block_size.eq(blk_size_le[0:16]),
                        ]
                        m.next = "READY"
                    with m.Else():
                        m.next = "READ-CAPACITY"

            with m.State("READY"):
                m.d.comb += self.status.busy.eq(0)
                with m.If(self.cmd.start):
                    m.d.usb += current_lba.eq(self.cmd.lba)
                    m.next = "READ"

            with m.State("READ"):
                m.d.comb += [
                    cdb10.opcode.eq(SCSIOpCode.READ_10),
                    # little-endian (amaranth) to big-endian (scsi)
                    cdb10.lba_be.eq(Cat(
                        current_lba[24:32], current_lba[16:24],
                        current_lba[8:16], current_lba[0:8])),
                    cdb10.xfer_len_be.eq(Cat(
                        Const(self._BLOCKS_PER_READ >> 8, 8),
                        Const(self._BLOCKS_PER_READ & 0xFF, 8))),
                    scsi_cmd.data_len.eq(block_size * self._BLOCKS_PER_READ),
                    scsi_cmd.stream_data.eq(1),
                    scsi_cmd.start.eq(1),
                ]
                m.next = "READ-WAIT"

            with m.State("READ-WAIT"):
                m.d.comb += [
                    cdb10.opcode.eq(SCSIOpCode.READ_10),
                    cdb10.lba_be.eq(Cat(
                        current_lba[24:32], current_lba[16:24],
                        current_lba[8:16], current_lba[0:8])),
                    cdb10.xfer_len_be.eq(Cat(
                        Const(self._BLOCKS_PER_READ >> 8, 8),
                        Const(self._BLOCKS_PER_READ & 0xFF, 8))),
                    scsi_cmd.data_len.eq(block_size * self._BLOCKS_PER_READ),
                    scsi_cmd.stream_data.eq(1),
                ]
                with m.If(scsi.status.done):
                    with m.If(~scsi.status.error & ~scsi.status.rejected):
                        m.d.usb += watchdog.eq(0)
                    m.d.comb += [
                        self.resp.done.eq(1),
                        self.resp.error.eq(scsi.status.error | scsi.status.rejected),
                    ]
                    m.next = "READY"

        return ResetInserter({"usb": watchdog_expired})(m)
