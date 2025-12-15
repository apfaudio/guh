# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB Host Enumerator - performs USB FS/HS enumeration sequence.
"""

from amaranth import *
from amaranth.lib import data, stream, wiring
from amaranth.lib.wiring import In, Out
from amaranth.lib.memory import Memory

from usb_protocol.types import DescriptorTypes

from .types import USBHostSpeed
from .sie import USBSIE, USBSIEInterface, TransferType, TransferResponse, DataPID
from guh.protocol.setup import SetupPayload


# ============================================================
# Enumerator Status
# ============================================================

class USBHostEnumeratorStatus(data.Struct):
    """Status outputs from USBHostEnumerator to driver."""
    enumerated:      unsigned(1)    # Enumeration complete, driver can use ctrl
    dev_addr:        unsigned(7)    # Assigned device address
    max_packet_size: unsigned(8)    # From device descriptor (bMaxPacketSize0)
    speed:           USBHostSpeed   # Detected speed (FULL or HIGH)


# ============================================================
# USB Host Enumerator
# ============================================================

class USBHostEnumerator(wiring.Component):
    """
    USB Host Enumerator - performs a USB FS/HS enumeration sequence.

    After enumeration completes (status.enumerated=1), the driver can use
    the `ctrl` interface for class-specific transfers.

    Enumeration Sequence:
    1. Reset Bus
    2. Get Device Descriptor - 8 bytes (to learn bMaxPacketSize)
    3. Set Address (to configured device_address)
    4. Get Device Descriptor - 18 bytes (full descriptor, may be multi-packet)
    5. Get Configuration Descriptor (streams to descriptor parser, may be multi-packet)
    6. Set Configuration (hard coded to configuration 1 for now)
    7. Assert 'enumerated', forward 'self.ctrl' for class-specific requests.
    """

    # SOFs to wait until first enumeration packets are sent.
    # TODO: enforce pow2 for this
    _SOF_DELAY_RDY = 0x3f

    # Status outputs to driver
    status: Out(USBHostEnumeratorStatus)

    # Pass-through transfer engine control interface after enumeration (use ONLY after enumerated=1)
    ctrl: Out(USBSIEInterface())

    def __init__(self, *, bus=None, handle_clocking=True,
                 device_address=0x12, config_number=1, parser):
        self._device_address = device_address
        self._config_number = config_number

        # Descriptor parser (streamed config descriptor stream at correct stage internally)
        self.parser = parser

        # Create USB transfer engine
        self.sie = USBSIE(bus=bus, handle_clocking=handle_clocking)

        super().__init__()

    @property
    def utmi(self):
        """Expose UTMI interface for testing."""
        return self.sie.utmi

    def elaborate(self, platform):
        m = Module()

        # Instantiate SIE
        m.submodules.sie = sie = self.sie

        # Setup Packet ROM
        m.submodules.setup_packets = setup_packets = Memory(
            shape=unsigned(8),
            depth=40,
            init=(
                SetupPayload.get_descriptor(int(DescriptorTypes.DEVICE), 0, 0, 8) +
                SetupPayload.get_descriptor(int(DescriptorTypes.DEVICE), 0, 0, 18) +
                SetupPayload.set_address(self._device_address) +
                SetupPayload.get_descriptor(int(DescriptorTypes.CONFIGURATION), 0, 0, 512) +
                SetupPayload.set_configuration(self._config_number)
            )
        )
        setup_mem = setup_packets.read_port(domain='comb')

        # Bits of state discovered by state machine
        current_dev_addr = Signal(7, init=0)
        enum_retry       = Signal(range(4))
        reset_triggered  = Signal(1, init=0)
        setup_byte_ix    = Signal(range(8))
        max_packet_size  = Signal(unsigned(8), init=64)
        last_packet_byte = Signal(unsigned(8), init=0)
        enumerated       = Signal(1, init=0)

        # Speed discovered by reset sequencer (HS/FS)
        detected_speed = sie.ctrl.status.detected_speed

        # Status outputs
        m.d.comb += [
            self.status.enumerated.eq(enumerated),
            self.status.dev_addr.eq(current_dev_addr),
            self.status.max_packet_size.eq(max_packet_size),
            self.status.speed.eq(detected_speed),
        ]

        # Descriptor parser: wire to rxs during GET_CONFIGURATION IN phase
        desc_stream_active = Signal()
        m.submodules.parser = parser = self.parser
        m.d.comb += [
            parser.enable.eq(1),
            parser.i.valid.eq(desc_stream_active & sie.ctrl.rxs.valid),
            parser.i.payload.eq(sie.ctrl.rxs.payload),
        ]

        # ctrl pass-through: after enumeration, forward signals between driver and USBSIE
        # During enumeration, enumerator owns the interface
        with m.If(enumerated):
            m.d.comb += [
                sie.ctrl.xfer.eq(self.ctrl.xfer),
                sie.ctrl.bus_reset.eq(self.ctrl.bus_reset),
            ]
            m.d.comb += self.ctrl.status.eq(sie.ctrl.status)
            wiring.connect(m, wiring.flipped(self.ctrl.txs), sie.ctrl.txs)
            wiring.connect(m, sie.ctrl.rxs, wiring.flipped(self.ctrl.rxs))

        # ============================================================
        # ENUMERATION HELPER FUNCTIONS
        # ============================================================

        def make_load_setup_state(state_name, next_state, setup_offset):
            """Generate state that loads 8 bytes from setup ROM to Tx FIFO."""
            with m.State(state_name):
                m.d.comb += [
                    setup_mem.addr.eq(setup_offset + setup_byte_ix),
                    sie.ctrl.txs.payload.eq(setup_mem.data),
                    sie.ctrl.txs.valid.eq(1),
                ]
                with m.If(sie.ctrl.txs.ready):
                    m.d.usb += setup_byte_ix.eq(setup_byte_ix + 1)
                    with m.If(setup_byte_ix == 7):
                        m.d.usb += setup_byte_ix.eq(0)
                        m.next = next_state

        def make_setup_xfer_state(state_name, next_state, dev_addr):
            """Generate state that sends SETUP token."""
            with m.State(state_name):
                with m.If(sie.ctrl.status.idle):
                    m.d.comb += [
                        sie.ctrl.xfer.start.eq(1),
                        sie.ctrl.xfer.type.eq(TransferType.SETUP),
                        sie.ctrl.xfer.data_pid.eq(DataPID.DATA0),
                        sie.ctrl.xfer.dev_addr.eq(dev_addr),
                        sie.ctrl.xfer.ep_addr.eq(0),
                    ]
                    m.next = next_state

        def make_wait_ack_state(state_name, on_ack, on_error, retry_sig=None, retry_from=None):
            """Generate state waiting for SETUP ACK with optional retry logic."""
            with m.State(state_name):
                with m.If(sie.ctrl.status.idle):
                    with m.Switch(sie.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            if retry_sig is not None:
                                m.d.usb += retry_sig.eq(0)
                            m.next = on_ack
                        with m.Case(TransferResponse.NAK, TransferResponse.TIMEOUT):
                            if retry_sig is not None:
                                m.d.usb += retry_sig.eq(retry_sig + 1)
                                with m.If(retry_sig >= 3):
                                    m.d.usb += retry_sig.eq(0)
                                    m.next = on_error
                                with m.Else():
                                    m.next = retry_from
                            else:
                                m.next = on_error

        def make_in_data_states(load_state, wait_state, next_state, dev_addr):
            """Generate IN data phase (2 states: send IN token, wait for data)."""
            with m.State(load_state):
                with m.If(sie.ctrl.status.idle):
                    m.d.comb += [
                        sie.ctrl.xfer.start.eq(1),
                        sie.ctrl.xfer.type.eq(TransferType.IN),
                        sie.ctrl.xfer.data_pid.eq(DataPID.DATA1),
                        sie.ctrl.xfer.dev_addr.eq(dev_addr),
                        sie.ctrl.xfer.ep_addr.eq(0),
                    ]
                    m.next = wait_state

            with m.State(wait_state):
                m.d.comb += sie.ctrl.rxs.ready.eq(1)
                with m.If(sie.ctrl.rxs.valid):
                    m.d.usb += last_packet_byte.eq(sie.ctrl.rxs.payload)
                with m.If(sie.ctrl.status.idle):
                    with m.Switch(sie.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.next = next_state
                        with m.Case(TransferResponse.NAK):
                            m.next = load_state
                        with m.Default():
                            m.next = next_state

        def make_multi_packet_in_states(in_state, wait_state, next_state, dev_addr,
                                        emit_desc_stream=False):
            """Generate multi-packet IN data phase (loops until short packet)."""
            with m.State(in_state):
                if emit_desc_stream:
                    m.d.comb += desc_stream_active.eq(1)
                with m.If(sie.ctrl.status.idle):
                    m.d.comb += [
                        sie.ctrl.xfer.start.eq(1),
                        sie.ctrl.xfer.type.eq(TransferType.IN),
                        sie.ctrl.xfer.data_pid.eq(DataPID.DATA1),
                        sie.ctrl.xfer.dev_addr.eq(dev_addr),
                        sie.ctrl.xfer.ep_addr.eq(0),
                    ]
                    m.next = wait_state

            with m.State(wait_state):
                if emit_desc_stream:
                    m.d.comb += desc_stream_active.eq(1)
                m.d.comb += sie.ctrl.rxs.ready.eq(1)
                with m.If(sie.ctrl.status.idle):
                    with m.Switch(sie.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            with m.If(sie.ctrl.status.rx_len == max_packet_size):
                                m.next = in_state
                            with m.Else():
                                m.next = next_state
                        with m.Case(TransferResponse.NAK):
                            m.next = in_state
                        with m.Default():
                            m.next = next_state

        def make_status_phase_states(status_state, wait_state, next_state, dev_addr,
                                     direction_in, on_timeout):
            """Generate status phase (2 states: send ZLP, wait for completion)."""
            with m.State(status_state):
                with m.If(sie.ctrl.status.idle):
                    m.d.comb += [
                        sie.ctrl.xfer.start.eq(1),
                        sie.ctrl.xfer.type.eq(TransferType.IN if direction_in else TransferType.OUT),
                        sie.ctrl.xfer.data_pid.eq(DataPID.DATA1),
                        sie.ctrl.xfer.dev_addr.eq(dev_addr),
                        sie.ctrl.xfer.ep_addr.eq(0),
                    ]
                    m.next = wait_state

            with m.State(wait_state):
                with m.If(sie.ctrl.status.idle):
                    with m.Switch(sie.ctrl.status.response):
                        with m.Case(TransferResponse.ACK):
                            m.next = next_state
                        with m.Case(TransferResponse.NAK):
                            m.next = status_state
                        with m.Case(TransferResponse.TIMEOUT):
                            m.next = on_timeout
                        with m.Default():
                            m.next = wait_state

        # ============================================================
        # FSM - USB HOST ENUMERATION
        # ============================================================
        with m.FSM(domain="usb"):

            with m.State("INIT-RESET"):
                with m.If(~reset_triggered):
                    m.d.comb += sie.ctrl.bus_reset.eq(1)
                    m.d.usb += reset_triggered.eq(1)

                with m.If(detected_speed != USBHostSpeed.UNKNOWN):
                    m.d.usb += [
                        enum_retry.eq(0),
                        reset_triggered.eq(0),
                    ]
                    m.next = "WAIT-SIE-READY"

            with m.State("WAIT-SIE-READY"):
                with m.If(sie.ctrl.status.idle &
                         ((sie.ctrl.status.sof_frame & self._SOF_DELAY_RDY) == self._SOF_DELAY_RDY)):
                    m.next = "ENUM-GET-DESC-DEVICE-LOAD"

            # -----------------------------------------------------------------
            # ENUMERATION STAGE 1: GET DEVICE DESCRIPTOR (8 bytes)
            # -----------------------------------------------------------------

            make_load_setup_state("ENUM-GET-DESC-DEVICE-LOAD",
                                  "ENUM-GET-DESC-DEVICE-XFER",
                                  setup_offset=0)

            make_setup_xfer_state("ENUM-GET-DESC-DEVICE-XFER",
                                  "ENUM-GET-DESC-DEVICE-WAIT-SETUP",
                                  dev_addr=0)

            make_wait_ack_state("ENUM-GET-DESC-DEVICE-WAIT-SETUP",
                                on_ack="ENUM-GET-DESC-DEVICE-IN",
                                on_error="INIT-RESET",
                                retry_sig=enum_retry,
                                retry_from="ENUM-GET-DESC-DEVICE-LOAD")

            make_in_data_states("ENUM-GET-DESC-DEVICE-IN",
                                "ENUM-GET-DESC-DEVICE-WAIT-IN",
                                "ENUM-GET-DESC-DEVICE-STATUS",
                                dev_addr=0)

            make_status_phase_states("ENUM-GET-DESC-DEVICE-STATUS",
                                     "ENUM-GET-DESC-DEVICE-WAIT-STATUS",
                                     "ENUM-GET-DESC-DEVICE-SET-MPS",
                                     dev_addr=0,
                                     direction_in=0,
                                     on_timeout="INIT-RESET")

            with m.State("ENUM-GET-DESC-DEVICE-SET-MPS"):
                m.d.usb += max_packet_size.eq(last_packet_byte)
                m.next = "ENUM-GET-DESC-DEVICE-COMPLETE"

            with m.State("ENUM-GET-DESC-DEVICE-COMPLETE"):
                m.d.usb += enum_retry.eq(0)
                m.next = "ENUM-SET-ADDRESS-LOAD"

            # -----------------------------------------------------------------
            # ENUMERATION STAGE 2: SET ADDRESS
            # -----------------------------------------------------------------

            make_load_setup_state("ENUM-SET-ADDRESS-LOAD",
                                  "ENUM-SET-ADDRESS-XFER",
                                  setup_offset=16)

            make_setup_xfer_state("ENUM-SET-ADDRESS-XFER",
                                  "ENUM-SET-ADDRESS-WAIT-SETUP",
                                  dev_addr=0)

            make_wait_ack_state("ENUM-SET-ADDRESS-WAIT-SETUP",
                                on_ack="ENUM-SET-ADDRESS-STATUS",
                                on_error="INIT-RESET")

            make_status_phase_states("ENUM-SET-ADDRESS-STATUS",
                                     "ENUM-SET-ADDRESS-WAIT-STATUS",
                                     "ENUM-SET-ADDRESS-UPDATE-ADDR",
                                     dev_addr=0,
                                     direction_in=1,
                                     on_timeout="INIT-RESET")

            with m.State("ENUM-SET-ADDRESS-UPDATE-ADDR"):
                m.d.usb += [
                    current_dev_addr.eq(self._device_address),
                    enum_retry.eq(0)
                ]
                m.next = "ENUM-GET-DESC-DEVICE-FULL-LOAD"

            # -----------------------------------------------------------------
            # ENUMERATION STAGE 3: GET FULL DEVICE DESCRIPTOR (18 bytes)
            # -----------------------------------------------------------------

            make_load_setup_state("ENUM-GET-DESC-DEVICE-FULL-LOAD",
                                  "ENUM-GET-DESC-DEVICE-FULL-XFER",
                                  setup_offset=8)

            make_setup_xfer_state("ENUM-GET-DESC-DEVICE-FULL-XFER",
                                  "ENUM-GET-DESC-DEVICE-FULL-WAIT-SETUP",
                                  dev_addr=current_dev_addr)

            make_wait_ack_state("ENUM-GET-DESC-DEVICE-FULL-WAIT-SETUP",
                                on_ack="ENUM-GET-DESC-DEVICE-FULL-IN",
                                on_error="INIT-RESET",
                                retry_sig=enum_retry,
                                retry_from="ENUM-GET-DESC-DEVICE-FULL-LOAD")

            make_multi_packet_in_states("ENUM-GET-DESC-DEVICE-FULL-IN",
                                        "ENUM-GET-DESC-DEVICE-FULL-WAIT-IN",
                                        "ENUM-GET-DESC-DEVICE-FULL-STATUS",
                                        dev_addr=current_dev_addr)

            make_status_phase_states("ENUM-GET-DESC-DEVICE-FULL-STATUS",
                                     "ENUM-GET-DESC-DEVICE-FULL-WAIT-STATUS",
                                     "ENUM-GET-DESC-DEVICE-FULL-COMPLETE",
                                     dev_addr=current_dev_addr,
                                     direction_in=0,
                                     on_timeout="INIT-RESET")

            with m.State("ENUM-GET-DESC-DEVICE-FULL-COMPLETE"):
                m.d.usb += enum_retry.eq(0)
                m.next = "ENUM-GET-DESC-CONFIG-LOAD"

            # -----------------------------------------------------------------
            # ENUMERATION STAGE 4: GET CONFIGURATION DESCRIPTOR
            # -----------------------------------------------------------------

            make_load_setup_state("ENUM-GET-DESC-CONFIG-LOAD",
                                  "ENUM-GET-DESC-CONFIG-XFER",
                                  setup_offset=24)

            make_setup_xfer_state("ENUM-GET-DESC-CONFIG-XFER",
                                  "ENUM-GET-DESC-CONFIG-WAIT-SETUP",
                                  dev_addr=current_dev_addr)

            make_wait_ack_state("ENUM-GET-DESC-CONFIG-WAIT-SETUP",
                                on_ack="ENUM-GET-DESC-CONFIG-IN",
                                on_error="INIT-RESET",
                                retry_sig=enum_retry,
                                retry_from="ENUM-GET-DESC-CONFIG-LOAD")

            # This is the key stage: emit desc_stream during IN phase
            make_multi_packet_in_states("ENUM-GET-DESC-CONFIG-IN",
                                        "ENUM-GET-DESC-CONFIG-WAIT-IN",
                                        "ENUM-GET-DESC-CONFIG-STATUS",
                                        dev_addr=current_dev_addr,
                                        emit_desc_stream=True)

            make_status_phase_states("ENUM-GET-DESC-CONFIG-STATUS",
                                     "ENUM-GET-DESC-CONFIG-WAIT-STATUS",
                                     "ENUM-GET-DESC-CONFIG-COMPLETE",
                                     dev_addr=current_dev_addr,
                                     direction_in=0,
                                     on_timeout="INIT-RESET")

            with m.State("ENUM-GET-DESC-CONFIG-COMPLETE"):
                m.d.usb += enum_retry.eq(0)
                m.next = "ENUM-SET-CONFIG-LOAD"

            # -----------------------------------------------------------------
            # ENUMERATION STAGE 5: SET CONFIGURATION (configuration 1)
            # -----------------------------------------------------------------

            # TODO: this should probably not happen automatically and instead
            # be exchanged between the 'Enumerator' <-> 'Driver' interface
            # after it has had a chance to look at the descriptors.

            # So far though, configuration 1 is correct for every single one
            # of the devices I have tested.

            make_load_setup_state("ENUM-SET-CONFIG-LOAD",
                                  "ENUM-SET-CONFIG-XFER",
                                  setup_offset=32)

            make_setup_xfer_state("ENUM-SET-CONFIG-XFER",
                                  "ENUM-SET-CONFIG-WAIT-SETUP",
                                  dev_addr=current_dev_addr)

            make_wait_ack_state("ENUM-SET-CONFIG-WAIT-SETUP",
                                on_ack="ENUM-SET-CONFIG-STATUS",
                                on_error="INIT-RESET")

            make_status_phase_states("ENUM-SET-CONFIG-STATUS",
                                     "ENUM-SET-CONFIG-WAIT-STATUS",
                                     "ENUMERATION-COMPLETE",
                                     dev_addr=current_dev_addr,
                                     direction_in=1,
                                     on_timeout="INIT-RESET")

            # -----------------------------------------------------------------
            # ENUMERATION COMPLETE - hand off to higher-level engine
            # -----------------------------------------------------------------

            with m.State("ENUMERATION-COMPLETE"):
                m.d.usb += enumerated.eq(1)
                m.next = "IDLE"

            with m.State("IDLE"):
                # Enumeration complete, driver has control of SIE via ctrl pass-through
                pass

        return m
