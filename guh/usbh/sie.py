# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB Host Serial Interface Engine (SIE).
Token packet generation, SOF controller, and the main transfer engine.
"""

from amaranth import *
from amaranth.lib import data, enum, fifo, stream, wiring
from amaranth.lib.cdc import ResetInserter
from amaranth.lib.wiring import In, Out

from luna.gateware.interface.ulpi import UTMITranslator
from luna.gateware.interface.utmi import *
from luna.gateware.usb.usb2.packet import *

from .types import *
from .reset import *

# ============================================================
# SIE Interface (used by Enumerator and class-specific engines)
# Check the USBSIE class itself for usage information.
# ============================================================

class TransferType(enum.Enum, shape=unsigned(2)):
    SETUP = 0
    IN    = 1
    OUT   = 2


class DataPID(enum.Enum, shape=unsigned(1)):
    DATA0 = 0
    DATA1 = 1


class TransferResponse(enum.Enum, shape=unsigned(3)):
    NONE        = 0
    ACK         = 1
    NAK         = 2
    STALL       = 3
    TIMEOUT     = 4
    CRC_ERROR   = 5
    RX_OVERFLOW = 6


class USBSIEInterface(wiring.Signature):

    class Transfer(data.Struct):
        start:    unsigned(1)
        type:     TransferType
        dev_addr: unsigned(7)
        ep_addr:  unsigned(4)
        data_pid: DataPID

    class Status(data.Struct):
        idle:           unsigned(1)
        response:       TransferResponse
        rx_len:         unsigned(8)
        sof_frame:      unsigned(11)
        reset_active:   unsigned(1)
        detected_speed: USBHostSpeed

    def __init__(self):
        super().__init__({
            "bus_reset": In(1),
            "xfer":      In(USBSIEInterface.Transfer),
            "status":    Out(USBSIEInterface.Status),
            "txs":       In(stream.Signature(unsigned(8))),
            "rxs":       Out(stream.Signature(unsigned(8))),
        })


# ============================================================
# Token Packet Generator
# ============================================================

class TokenPID(enum.Enum, shape=unsigned(4)):
    OUT   = int(USBPacketID.OUT)
    IN    = int(USBPacketID.IN)
    SOF   = int(USBPacketID.SOF)
    SETUP = int(USBPacketID.SETUP)


class TokenPayload(data.Struct):
    # Lightweight storage for token contents,
    # excluding crc5 and pid nibble that are
    # added before this is sent on the wire.
    pid:  TokenPID
    data: data.StructLayout({
        "addr": unsigned(7),
        "endp": unsigned(4),
    })

class USBTokenPacketGenerator(wiring.Component):

    """
    Send a stream of TokenPayloads over UTMI.

    A TokenPayload requires a second PID nibble and crc5 for it to
    be ready for the wire (UTMI). This is calculated here.
    """

    i: In(stream.Signature(TokenPayload))
    speed: In(USBHostSpeed)
    txa: Out(1)

    #
    # IN tokens use InterPacketTimer to determine when `txa`
    # (Tx Allowed) is permitted, other tokens need more time.
    # This is that time in cycles.
    #

    _LONG_TXA_POST_TRANSMIT_FS = 200
    _LONG_TXA_POST_TRANSMIT_HS = 20 # TODO: check correctness...

    def __init__(self):
        self.tx = UTMITransmitInterface()
        self.timer = InterpacketTimerInterface()
        super().__init__()

    def elaborate(self, platform):
        m = Module()

        pkt = Signal(shape=TokenPayload)

        long_txa_post_transmit_cnt = Signal(range(self._LONG_TXA_POST_TRANSMIT_FS+1))

        with m.FSM(domain="usb"):

            with m.State('IDLE'):
                m.d.comb += self.i.ready.eq(1)
                m.d.usb += long_txa_post_transmit_cnt.eq(
                    Mux(self.speed==USBHostSpeed.HIGH,
                            self._LONG_TXA_POST_TRANSMIT_HS,
                            self._LONG_TXA_POST_TRANSMIT_FS
                        ) - 1)
                with m.If(self.i.valid):
                    m.d.usb += pkt.eq(self.i.payload)
                    m.next = "SEND_PID"

            with m.State('SEND_PID'):

                with m.Switch(pkt.pid):
                    with m.Case(TokenPID.OUT):
                        m.d.comb += self.tx.data.eq(USBPacketID.OUT.byte()),
                    with m.Case(TokenPID.IN):
                        m.d.comb += self.tx.data.eq(USBPacketID.IN.byte()),
                    with m.Case(TokenPID.SOF):
                        m.d.comb += self.tx.data.eq(USBPacketID.SOF.byte()),
                    with m.Case(TokenPID.SETUP):
                        m.d.comb += self.tx.data.eq(USBPacketID.SETUP.byte()),

                m.d.comb += self.tx.valid.eq(1),

                with m.If(self.tx.ready):
                    m.next = 'SEND_PAYLOAD0'

            with m.State('SEND_PAYLOAD0'):
                m.d.comb += [
                    self.tx.data .eq(pkt.data.as_value()[0:8]),
                    self.tx.valid.eq(1),
                ]
                with m.If(self.tx.ready):
                    m.next = 'SEND_PAYLOAD1'

            with m.State('SEND_PAYLOAD1'):
                crc5 = Signal(5)
                m.d.comb += [
                    crc5.eq(USBTokenDetector._generate_crc_for_token(pkt.data.as_value())),
                    self.tx.data .eq(Cat(pkt.data.as_value()[8:11], crc5)),
                    self.tx.valid.eq(1),
                ]
                with m.If(self.tx.ready):
                    m.d.comb += self.timer.start.eq(1)
                    with m.If(pkt.pid == TokenPID.IN):
                        m.d.comb += self.txa.eq(1)
                        m.next = 'WAIT-SHORT-TXA'
                    with m.Else():
                        m.next = 'WAIT-LONG-TXA'

            with m.State('WAIT-SHORT-TXA'):
                with m.If(self.timer.tx_allowed):
                    m.next = 'IDLE'

            with m.State('WAIT-LONG-TXA'):
                cnt = Signal.like(long_txa_post_transmit_cnt)
                m.d.usb += cnt.eq(cnt+1)
                with m.If(cnt == long_txa_post_transmit_cnt):
                    m.d.comb += self.txa.eq(1)
                    m.d.usb += cnt.eq(0)
                    m.next = 'IDLE'

        return m


# ============================================================
# SOF Controller
# ============================================================

class USBSOFController(wiring.Component):

    """
    Emit SOF TokenPayloads at intervals based on speed.
    - Full-Speed: 1ms frames
    - High-Speed: 125us microframes (8 per 1ms frame)

    :py:`txa` is a window signal HIGH between _SOF_TX_TO_TX_MIN and _SOF_TX_TO_TX_MAX
    after each SOF transmission, indicating when other transfers are allowed.

    Use ResetInserter to hold this component in reset until enumeration is complete.

    TODO: `txa` window is overly conservative for now. We could make it much wider
    for more bandwidth, at the moment we're limited to ~half the link.

    TODO: `TX_TO_RX` is kind of meaningless WRT the USB standard, and it's kind of
    an implementation detail as to how this USB Host implementation works. Perhaps
    it's best to remove this altogether and keep the Rx timeout tracking in USBSIE.
    """

    speed:  In(USBHostSpeed)
    txa:    Out(1)  # Transmission window (HIGH = allowed to transmit)
    rxa:    Out(1)  # Reception window (HIGH = device allowed to send)
    o: Out(stream.Signature(TokenPayload))

    # FS: emit a SOF packet every 1ms
    _SOF_CYCLES_FS = 60000
    _SOF_TX_TO_TX_MIN_FS = 2*6000   # 0.2ms - start of TX window
    _SOF_TX_TO_TX_MAX_FS = 7*6000   # 0.7ms - end of TX window
    _SOF_TX_TO_RX_MAX_FS = 9*6000   # 0.9ms - end of RX window

    # HS: emit a SOF packet every 125us (microframe)
    _SOF_CYCLES_HS = 7500
    _SOF_TX_TO_TX_MIN_HS = 2*750    # ~25us - start of TX window
    _SOF_TX_TO_TX_MAX_HS = 7*750    # ~87us - end of TX window
    _SOF_TX_TO_RX_MAX_HS = 9*750    # ~112us - end of RX window

    def elaborate(self, platform):
        m = Module()

        sof_timer = Signal(16)
        frame_number = Signal(11)
        microframe_number = Signal(3)

        # Dynamic timing constants based on speed
        sof_cycles = Signal(16)
        tx_to_tx_min = Signal(16)
        tx_to_tx_max = Signal(16)
        tx_to_rx_max = Signal(16)

        with m.If(self.speed == USBHostSpeed.HIGH):
            m.d.comb += [
                sof_cycles.eq(self._SOF_CYCLES_HS),
                tx_to_tx_min.eq(self._SOF_TX_TO_TX_MIN_HS),
                tx_to_tx_max.eq(self._SOF_TX_TO_TX_MAX_HS),
                tx_to_rx_max.eq(self._SOF_TX_TO_RX_MAX_HS),
            ]
        with m.Else():
            m.d.comb += [
                sof_cycles.eq(self._SOF_CYCLES_FS),
                tx_to_tx_min.eq(self._SOF_TX_TO_TX_MIN_FS),
                tx_to_tx_max.eq(self._SOF_TX_TO_TX_MAX_FS),
                tx_to_rx_max.eq(self._SOF_TX_TO_RX_MAX_FS),
            ]

        m.d.usb += sof_timer.eq(sof_timer + 1)

        m.d.comb += [
            self.o.payload.pid.eq(TokenPID.SOF),
            self.o.payload.data.eq(frame_number),
        ]

        m.d.comb += self.txa.eq(
            (sof_timer >= tx_to_tx_min) &
            (sof_timer < tx_to_tx_max)
        )

        m.d.comb += self.rxa.eq(
            (sof_timer >= tx_to_tx_min) &
            (sof_timer < tx_to_rx_max)
        )

        with m.FSM(domain="usb"):

            with m.State('IDLE'):
                with m.If(sof_timer == (sof_cycles - 1)):
                    m.d.usb += sof_timer.eq(0)

                    with m.If(self.speed == USBHostSpeed.HIGH):
                        m.d.usb += microframe_number.eq(microframe_number + 1)
                        with m.If(microframe_number == 7):
                            m.d.usb += [
                                microframe_number.eq(0),
                                frame_number.eq(frame_number + 1),
                            ]
                    with m.Else():
                        m.d.usb += frame_number.eq(frame_number + 1)

                    m.next = 'SEND'

            with m.State('SEND'):
                m.d.comb += self.o.valid.eq(1)
                with m.If(self.o.ready):
                    m.next = 'IDLE'

        return m


# ============================================================
# USB Serial Interface Engine (SIE)
# ============================================================

class USBSIE(wiring.Component):
    """
    LUNA-based USB Host core.

    This is a simple USB Host transfer engine, controlled via 'USBSIEInterface'.

    For transmission:
    - Clock data payload into ctrl.txs
    - Set transfer properties in ctrl.xfer
    - Strobe ctrl.xfer.start
    - Wait for ctrl.status to report completion.

    For reception:
    - Set transfer properties in ctrl.xfer
    - Strobe ctrl.xfer.start
    - Hook up ctrl.rxs so you can accept bytes from the link.
    - Wait for ctrl.status to report completion.
    """

    ctrl: Out(USBSIEInterface())

    # TODO: check these against standard, I think they're overly conservative.
    _XFER_IPD_FS = 1000  # Inter-packet delay for Full-Speed
    _XFER_IPD_HS = 100   # Inter-packet delay for High-Speed

    def __init__(self, *, bus=None, handle_clocking=True, fullspeed_only=False):
        self.fifo_depth = 64  # Max USB packet size

        # UTMI interface (TODO: move to component signature once Records removed from LUNA)
        # TODO: support also non-ULPI interfaces? should be pretty easy...
        if bus is None:
            self.utmi = UTMIInterface()
        else:
            self.utmi = UTMITranslator(ulpi=bus, handle_clocking=handle_clocking)
            self.translator = self.utmi

        # nb. internals timings may be tweaked by simulation harness to reduce delays
        self.reset_ctrl = USBResetController(
            fullspeed_only=fullspeed_only
        )

        super().__init__()

    def elaborate(self, platform):
        m = Module()

        # Submodules
        if hasattr(self, 'translator'):
            m.submodules.translator = self.translator

        # LUNA USB components - NOTE/HACK: some are held in reset during bus reset
        transmitter         = USBDataPacketGenerator()
        receiver            = USBDataPacketReceiver(utmi=self.utmi)
        data_crc            = USBDataPacketCRC()
        handshake_generator = USBHandshakeGenerator()
        handshake_detector  = USBHandshakeDetector(utmi=self.utmi)
        token_generator     = USBTokenPacketGenerator()
        sof_controller      = USBSOFController()
        timer               = USBInterpacketTimer(fs_only=False)
        tx_multiplexer      = UTMIInterfaceMultiplexer()

        # Data CRC interfaces
        data_crc.add_interface(transmitter.crc)
        data_crc.add_interface(receiver.data_crc)

        # Inter-packet timer interfaces
        timer.add_interface(receiver.timer)
        timer.add_interface(token_generator.timer)

        # UTMI transmission interfaces
        tx_multiplexer.add_input(token_generator.tx)
        tx_multiplexer.add_input(transmitter.tx)
        tx_multiplexer.add_input(handshake_generator.tx)

        # Tx/Rx FIFOs
        m.submodules.tx_fifo = tx_fifo = fifo.SyncFIFO(width=8, depth=self.fifo_depth)
        m.submodules.rx_fifo = rx_fifo = fifo.SyncFIFO(width=8, depth=self.fifo_depth)

        # ============================================================
        # RESET CONTROLLER - Bus Reset and Speed Detection
        # ============================================================

        m.submodules.reset_ctrl = reset_ctrl = self.reset_ctrl
        m.d.comb += [
            reset_ctrl.bus_reset.eq(self.ctrl.bus_reset),
            reset_ctrl.phy.line_state.eq(self.utmi.line_state),
        ]
        m.d.comb += [
            self.ctrl.status.reset_active.eq(reset_ctrl.reset_active),
            self.ctrl.status.detected_speed.eq(reset_ctrl.detected_speed),
        ]
        detected_speed = reset_ctrl.detected_speed
        # Add reset controller TX to the multiplexer (with highest priority)
        tx_multiplexer.add_input(reset_ctrl.tx)

        # Add LUNA components as submodules, with ResetInserter for speed-dependent ones
        # this is needed for HS/FS switching to work reliably for now.
        m.submodules.transmitter         = ResetInserter({"usb": reset_ctrl.reset_active})(transmitter)
        m.submodules.receiver            = ResetInserter({"usb": reset_ctrl.reset_active})(receiver)
        m.submodules.data_crc            = ResetInserter({"usb": reset_ctrl.reset_active})(data_crc)
        m.submodules.handshake_generator = ResetInserter({"usb": reset_ctrl.reset_active})(handshake_generator)
        m.submodules.handshake_detector  = ResetInserter({"usb": reset_ctrl.reset_active})(handshake_detector)
        m.submodules.token_generator     = ResetInserter({"usb": reset_ctrl.reset_active})(token_generator)
        m.submodules.sof_controller      = ResetInserter({"usb": reset_ctrl.reset_active})(sof_controller)
        m.submodules.timer               = ResetInserter({"usb": reset_ctrl.reset_active})(timer)
        m.submodules.tx_multiplexer      = tx_multiplexer

        # Hosts must always assert pull-downs on both D+ and D- to
        # detect device connection. These are driven at the top level
        # and not included in the PHY control signature shared with the
        # reset controller because it doesn't need to modify this.
        m.d.comb += [
            self.utmi.dm_pulldown.eq(1),
            self.utmi.dp_pulldown.eq(1),
        ]

        # UTMI TX multiplexer output to UTMI PHY
        # TODO: migrate to streams once LUNA does...
        m.d.comb += [
            self.utmi.tx_data.eq(tx_multiplexer.output.data),
            self.utmi.tx_valid.eq(tx_multiplexer.output.valid),
            tx_multiplexer.output.ready.eq(self.utmi.tx_ready),
        ]

        # Data CRC for normal packet operation
        m.d.comb += [
            data_crc.tx_valid.eq(self.utmi.tx_valid & self.utmi.tx_ready),
            data_crc.tx_data.eq(self.utmi.tx_data),
        ]

        m.d.comb += [
            sof_controller.speed.eq(detected_speed),
            token_generator.speed.eq(detected_speed),
            timer.speed.eq(detected_speed),
        ]

        # Wire up remaining LUNA components
        m.d.comb += [
            data_crc.rx_data.eq(self.utmi.rx_data),
            data_crc.rx_valid.eq(self.utmi.rx_valid),
        ]

        # Stream interface connections to FIFOs
        wiring.connect(m, wiring.flipped(self.ctrl.txs), tx_fifo.w_stream)  # Controller -> Tx FIFO
        wiring.connect(m, rx_fifo.r_stream, wiring.flipped(self.ctrl.rxs))  # Rx FIFO -> Controller

        # SIE State Machine
        # Transfer parameters from ctrl interface
        xfer = Signal(USBSIEInterface.Transfer)

        # Transfer status
        send_sofs = Signal()
        rx_len = Signal(8)
        response = Signal(TransferResponse)
        tx_len = Signal(8)  # Captured from TX FIFO level at transfer start
        tx_byte_count = Signal(16)

        # SIE token stream for multiplexing with SOF. SOF has priority when SIE is idle
        sie_token = stream.Signature(TokenPayload).create()
        m.d.comb += [
            token_generator.i.valid.eq(Mux(send_sofs,
                                           sof_controller.o.valid,
                                           sie_token.valid)),
            token_generator.i.payload.eq(Mux(send_sofs,
                                            sof_controller.o.payload,
                                            sie_token.payload)),
            sof_controller.o.ready.eq(send_sofs & token_generator.i.ready),
            sie_token.ready.eq(~send_sofs & token_generator.i.ready),
        ]

        m.d.comb += [
            self.ctrl.status.rx_len.eq(rx_len),
            self.ctrl.status.response.eq(response),
            self.ctrl.status.sof_frame.eq(sof_controller.o.payload.data),
        ]

        m.d.comb += [
            self.utmi.op_mode.eq(UTMIOperatingModeEnum.NORMAL),
            self.utmi.xcvr_select.eq(Mux(detected_speed == USBHostSpeed.HIGH,
                                         USBHostSpeed.HIGH,
                                         USBHostSpeed.FULL)),
            self.utmi.term_select.eq(Mux(detected_speed == USBHostSpeed.HIGH,
                                         UTMITerminationSelectEnum.HS_NORMAL,
                                         UTMITerminationSelectEnum.LS_FS_NORMAL)),
        ]

        with m.FSM(domain="usb") as fsm:

            with m.State("RESET"):
                # Reset controller overrides default PHY configuration
                m.d.comb += [
                    self.utmi.op_mode.eq(reset_ctrl.phy.op_mode),
                    self.utmi.xcvr_select.eq(reset_ctrl.phy.xcvr_select),
                    self.utmi.term_select.eq(reset_ctrl.phy.term_select),
                ]

                # Transition to IDLE when reset completes
                with m.If(~reset_ctrl.reset_active):
                    m.next = "IDLE"

            with m.State("IDLE"):
                m.d.comb += send_sofs.eq(1)
                m.d.comb += self.ctrl.status.idle.eq(1)
                # Capture transfer parameters on xfer.start
                with m.If(self.ctrl.xfer.start):
                    m.d.usb += [
                        xfer.eq(self.ctrl.xfer),
                        tx_byte_count.eq(0),
                        rx_len.eq(0),
                        response.eq(TransferResponse.NONE),
                        tx_len.eq(tx_fifo.w_level),  # Capture TX FIFO level
                    ]
                    m.next = "DRAIN_RX"

            with m.State("DRAIN_RX"):
                m.d.comb += send_sofs.eq(1)
                m.d.comb += rx_fifo.r_en.eq(rx_fifo.r_rdy)
                with m.If(~rx_fifo.r_rdy):
                    m.next = "WAIT_TXA"

            with m.State("WAIT_TXA"):
                m.d.comb += send_sofs.eq(1)
                # Only start transfers when txa (transmission window) opens up
                with m.If(sof_controller.txa):
                    m.next = "SEND_TOKEN"

            with m.State("SEND_TOKEN"):
                # Derive token PID from transfer type
                # This is possible as long as we don't support iso
                token_pid = Signal(TokenPID, init=TokenPID.SETUP)
                with m.Switch(xfer.type):
                    with m.Case(TransferType.SETUP):
                        m.d.comb += token_pid.eq(TokenPID.SETUP)
                    with m.Case(TransferType.IN):
                        m.d.comb += token_pid.eq(TokenPID.IN)
                    with m.Case(TransferType.OUT):
                        m.d.comb += token_pid.eq(TokenPID.OUT)

                m.d.comb += [
                    sie_token.valid.eq(1),
                    sie_token.payload.pid.eq(token_pid),
                    sie_token.payload.data.addr.eq(xfer.dev_addr),
                    sie_token.payload.data.endp.eq(xfer.ep_addr),
                ]
                with m.If(sie_token.ready):
                    m.next = "WAIT_TOKEN_COMPLETE"

            with m.State("WAIT_TOKEN_COMPLETE"):
                # Wait for token transmission to complete before sending data
                with m.If(token_generator.txa):
                    with m.If(xfer.type == TransferType.IN):
                        m.next = "WAIT_IN_DATA"
                    with m.Else():
                        with m.If(tx_len):
                            m.next = "SEND_OUT_DATA"
                        with m.Else():
                            m.next = "SEND_OUT_ZLP"

            with m.State("SEND_OUT_DATA"):
                # Send data phase for OUT transfer
                m.d.comb += [
                    transmitter.data_pid.eq(xfer.data_pid),
                    transmitter.stream.valid.eq(tx_fifo.r_rdy),
                    transmitter.stream.payload.eq(tx_fifo.r_data),
                    tx_fifo.r_en.eq(transmitter.stream.ready & tx_fifo.r_rdy),
                ]

                with m.If(tx_byte_count == 0):
                    m.d.comb += transmitter.stream.first.eq(1)
                with m.If(tx_byte_count == (tx_len - 1)):
                    m.d.comb += transmitter.stream.last.eq(1)

                with m.If(transmitter.stream.ready & tx_fifo.r_rdy):
                    m.d.usb += tx_byte_count.eq(tx_byte_count + 1)
                    with m.If(tx_byte_count == (tx_len - 1)):
                        m.d.usb += tx_byte_count.eq(0)
                        m.next = "WAIT_HANDSHAKE"

            with m.State("SEND_OUT_ZLP"):
                m.d.comb += [
                    transmitter.data_pid.eq(xfer.data_pid),
                    transmitter.stream.valid.eq(1),
                    transmitter.stream.payload.eq(0),
                    transmitter.stream.last.eq(1),
                ]
                m.next = "WAIT_HANDSHAKE"

            with m.State("WAIT_IN_DATA"):
                with m.If(receiver.stream.next):
                    m.d.comb += rx_fifo.w_data.eq(receiver.stream.payload)
                    with m.If(rx_fifo.w_rdy):
                        m.d.comb += rx_fifo.w_en.eq(1)
                    with m.Else():
                        m.d.usb += response.eq(TransferResponse.RX_OVERFLOW)
                    m.d.usb += rx_len.eq(rx_len + 1)
                with m.If(receiver.ready_for_response):
                    with m.If(response == TransferResponse.RX_OVERFLOW):
                        m.next = "IPD_DRAIN_TX"
                    with m.Else():
                        # TODO: not correct for iso transfers
                        m.next = "SEND_ACK"
                with m.If(receiver.crc_mismatch):
                    m.d.usb += response.eq(TransferResponse.CRC_ERROR)
                    m.next = "IPD_DRAIN_TX"
                with m.If(handshake_detector.detected.nak):
                    m.d.usb += response.eq(TransferResponse.NAK)
                    m.next = "IPD_DRAIN_TX"
                with m.If(handshake_detector.detected.stall):
                    m.d.usb += response.eq(TransferResponse.STALL)
                    m.next = "IPD_DRAIN_TX"
                with m.If(~sof_controller.rxa):
                    m.d.usb += response.eq(TransferResponse.TIMEOUT)
                    m.next = "IPD_DRAIN_TX"

            with m.State("WAIT_HANDSHAKE"):
                with m.If(handshake_detector.detected.ack):
                    m.d.usb += response.eq(TransferResponse.ACK)
                    m.next = "IPD_DRAIN_TX"
                with m.If(handshake_detector.detected.nak):
                    m.d.usb += response.eq(TransferResponse.NAK)
                    m.next = "IPD_DRAIN_TX"
                with m.If(handshake_detector.detected.stall):
                    m.d.usb += response.eq(TransferResponse.STALL)
                    m.next = "IPD_DRAIN_TX"
                with m.If(~sof_controller.rxa):
                    m.d.usb += response.eq(TransferResponse.TIMEOUT)
                    m.next = "IPD_DRAIN_TX"

            with m.State("SEND_ACK"):
                # Send ACK handshake for IN transfer
                m.d.comb += handshake_generator.issue_ack.eq(1)
                m.d.usb += response.eq(TransferResponse.ACK)
                m.next = "IPD_DRAIN_TX"

            with m.State("IPD_DRAIN_TX"):
                # Post-transaction interpacket delay while draining TX FIFO
                ipd_max = Mux(detected_speed == USBHostSpeed.HIGH,
                              self._XFER_IPD_HS, self._XFER_IPD_FS)
                ipd = Signal(range(self._XFER_IPD_FS), init=0)
                m.d.usb += ipd.eq(ipd+1)
                m.d.comb += tx_fifo.r_en.eq(tx_fifo.r_rdy)
                with m.If((ipd == (ipd_max-1)) & ~tx_fifo.r_rdy):
                    m.d.usb += ipd.eq(0)
                    m.next = "IDLE"

        return m
