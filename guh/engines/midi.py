# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB MIDI Host engine.
"""

from amaranth import *
from amaranth.lib import fifo, stream, wiring
from amaranth.lib.cdc import ResetInserter
from amaranth.lib.wiring import In, Out

from luna.gateware.stream.future import Packet

from guh.usbh.enumerator import USBHostEnumerator
from guh.usbh.sie import TransferType, TransferResponse, DataPID
from guh.usbh.descriptor import USBDescriptorParser, EndpointFilter
from guh.protocol.descriptors import *

# USB-MIDI event size (Cable+CIN byte + 3 MIDI bytes)
USB_MIDI_EVENT_SIZE = 4

class USBMIDIHost(wiring.Component):
    """
    USB MIDI Host - receives MIDI data from USB MIDI devices.

    Output stream uses Packet wrapper with first/last markers for USB-MIDI event boundaries.
    USB-MIDI events are 4 bytes: Cable Number + CIN (1 byte) + MIDI data (3 bytes).
    """

    # Watchdog timeout: reset enumeration if no response for this many cycles
    # At 60MHz, 3*60000000 cycles = ~3 seconds
    _WATCHDOG_CYCLES = 3 * 60000000

    # After enumeration, transmits MIDI data received from device
    # (4-byte packets) with first/last framing
    o_midi: Out(stream.Signature(Packet(unsigned(8))))

    def __init__(self, *, bus=None, handle_clocking=True, device_address=0x12):
        self.enumerator = USBHostEnumerator(
            bus=bus,
            handle_clocking=handle_clocking,
            device_address=device_address,
            config_number=1,
            parser=USBDescriptorParser(
                endpoint_filter=EndpointFilter.IN,
                transfer_type=EndpointTransferType.BULK,
                interface_class=InterfaceClass.AUDIO,
                interface_subclass=AudioSubClass.MIDISTREAMING,
                interface_protocol=AudioProtocol.AUDIO_1_0,
            ),
        )
        super().__init__()

    @property
    def sie(self):
        """Expose internal SIE for bus forwarding / testing."""
        return self.enumerator.sie

    def elaborate(self, platform):
        m = Module()

        m.submodules.enumerator = enum = self.enumerator

        # Watchdog: kicked on successful device responses, triggers reset on timeout
        watchdog = Signal(32)
        watchdog_expired = Signal()
        m.d.usb += watchdog.eq(watchdog + 1)
        m.d.comb += watchdog_expired.eq(watchdog == (self._WATCHDOG_CYCLES - 1))

        # MIDI RX FIFO with Packet framing
        packet_layout = Packet(unsigned(8))
        m.submodules.midi_fifo = midi_fifo = fifo.SyncFIFOBuffered(
            width=packet_layout.size, depth=64)
        wiring.connect(m, midi_fifo.r_stream, wiring.flipped(self.o_midi))

        pid = Signal(DataPID, init=DataPID.DATA0)
        l_sof_frame = Signal(11)
        rx_byte_count = Signal(2)

        # RX path: add first/last markers based on 4-byte USB-MIDI event boundaries
        rx_packet = packet_layout(midi_fifo.w_stream.payload)
        m.d.comb += [
            midi_fifo.w_stream.valid.eq(enum.ctrl.rxs.valid),
            enum.ctrl.rxs.ready.eq(midi_fifo.w_stream.ready),
            rx_packet.data.eq(enum.ctrl.rxs.payload),
            rx_packet.first.eq(rx_byte_count == 0),
            rx_packet.last.eq(rx_byte_count == USB_MIDI_EVENT_SIZE - 1),
        ]

        # Track incoming midi packets, wrapping at 4
        with m.If(enum.ctrl.rxs.valid & enum.ctrl.rxs.ready):
            m.d.usb += rx_byte_count.eq(rx_byte_count + 1)

        with m.FSM(domain="usb"):

            with m.State("WAIT-ENUMERATION"):

                with m.If(enum.status.enumerated & enum.parser.o.valid):
                    m.d.usb += watchdog.eq(0)  # Kick watchdog on successful enumeration
                    m.next = "MIDI-POLL"

            with m.State("MIDI-POLL"):

                # Issue an IN transaction (poll) every SOF (1ms, not every microframe)
                with m.If(enum.ctrl.status.idle & (enum.ctrl.status.sof_frame != l_sof_frame)):
                    m.d.comb += [
                        enum.ctrl.xfer.start.eq(1),
                        enum.ctrl.xfer.type.eq(TransferType.IN),
                        enum.ctrl.xfer.data_pid.eq(pid),
                        enum.ctrl.xfer.dev_addr.eq(enum.status.dev_addr),
                        enum.ctrl.xfer.ep_addr.eq(enum.parser.o.i_endp.number),
                    ]
                    # Reset byte counter at start of each transfer
                    m.d.usb += [
                        l_sof_frame.eq(enum.ctrl.status.sof_frame),
                        rx_byte_count.eq(0),
                    ]
                with m.Else():
                    with m.Switch(enum.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            # Success: toggle data PID, kick watchdog
                            m.d.usb += [
                                pid.eq(Mux(pid, DataPID.DATA0, DataPID.DATA1)),
                                watchdog.eq(0),
                            ]
                        with m.Case(TransferResponse.NAK):
                            # Device has no data but responded: kick watchdog
                            m.d.usb += watchdog.eq(0)
                        with m.Case(TransferResponse.STALL):
                            # STALL: let watchdog handle recovery
                            pass

        # Watchdog triggers reset of both this module and enumerator
        return ResetInserter({"usb": watchdog_expired})(m)
