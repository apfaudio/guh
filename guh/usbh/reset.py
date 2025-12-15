# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB Host reset and speed detection controller.
"""

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out

from luna.gateware.interface.utmi import UTMITransmitInterface

from .types import *

class UTMIPhyControlSignature(wiring.Signature):
    def __init__(self):
        super().__init__({
            # PHY Configuration Outputs (to PHY)
            "op_mode":      Out(UTMIOperatingModeEnum),      # Operating mode
            "xcvr_select":  Out(USBHostSpeed),               # Speed selection
            "term_select":  Out(UTMITerminationSelectEnum),  # Termination mode

            # PHY Status Inputs (from PHY)
            "line_state":   In(UTMILineState),           # D+/D- line state
        })


class USBResetController(wiring.Component):
    """
    USB Host reset and speed detection controller.

    Handles USB bus reset sequence, high-speed chirp negotiation, and
    speed detection.

    TODO: currently, we don't gracefully handle disconnects.
    That's probably the next big low-hanging fruit, even though it
    is indirectly handled by our watchdog in higher-level components.
    """

    # TODO: some of these are kind of magic discovered during testing.
    # and from reading LUNA. double-check against the standard...
    # -> all of these are in 60MHz domain clock cycles
    _SETTLE_TIME = 6000                 # Device connection settle time (~100us)
    _MAX_RESET_TIME = 3000000           # Maximum reset duration (~50ms)
    _MIN_RESET_BEFORE_CHIRP = 3000      # Minimum reset before looking for chirp (~50us)
    _CHIRP_FILTER_CYCLES = 30000        # Filter time for device chirp K detection (~500us)
    _CHIRP_DURATION = 3000              # Host chirp K/J duration (~50us)

    def __init__(self, *, fullspeed_only=False):
        self.fullspeed_only = fullspeed_only
        # TODO: this should be below once Records are deprecated in LUNA.
        self.tx = UTMITransmitInterface()
        super().__init__({
            "bus_reset":      In(1),                          # Strobe to trigger reset
            "reset_active":   Out(1),                         # Reset FSM is active
            "detected_speed": Out(USBHostSpeed),              # UNKNOWN/FULL/HIGH
            "phy":            Out(UTMIPhyControlSignature()), # PHY control signals (chirps on self.tx though)
        })

    def elaborate(self, platform):
        m = Module()

        # Internal FSM state signals
        reset_counter = Signal(range(4000000))
        chirp_timer = Signal(16)
        in_idle_state = Signal()

        m.d.comb += [
            self.tx.valid.eq(0),
            self.tx.data.eq(0),

            self.phy.op_mode.eq(UTMIOperatingModeEnum.NORMAL),
            self.phy.xcvr_select.eq(USBHostSpeed.FULL),
            self.phy.term_select.eq(UTMITerminationSelectEnum.LS_FS_NORMAL),

            # Default: reset active, speed unknown
            self.reset_active.eq(1),
            self.detected_speed.eq(USBHostSpeed.UNKNOWN),
        ]

        with m.If(in_idle_state):
            m.d.usb += reset_counter.eq(0)
        with m.Else():
            m.d.usb += reset_counter.eq(reset_counter + 1)

        with m.FSM(domain="usb"):

            with m.State("DISCONNECTED"):
                m.d.comb += in_idle_state.eq(1)
                with m.If(self.phy.line_state == UTMILineState.J):
                    m.next = "WAIT-CONNECT"

            with m.State("WAIT-CONNECT"):
                with m.If(self.phy.line_state == UTMILineState.J):
                    with m.If(reset_counter >= self._SETTLE_TIME):
                        m.next = "BUS-RESET"
                with m.Else():
                    m.next = "DISCONNECTED"

            with m.State("BUS-RESET"):
                m.d.comb += [
                    self.phy.op_mode.eq(UTMIOperatingModeEnum.RAW_DRIVE),
                    self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                    self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                ]

                if not self.fullspeed_only:
                    with m.If(reset_counter >= self._MIN_RESET_BEFORE_CHIRP):
                        with m.If(self.phy.line_state == UTMILineState.K):
                            m.d.usb += chirp_timer.eq(chirp_timer + 1)
                            with m.If(chirp_timer >= self._CHIRP_FILTER_CYCLES):
                                m.d.usb += chirp_timer.eq(0)
                                m.next = "WAIT-DEVICE-CHIRP-END"
                        with m.Else():
                            m.d.usb += chirp_timer.eq(0)

                with m.If(reset_counter >= self._MAX_RESET_TIME):
                    m.d.usb += chirp_timer.eq(0)
                    m.next = "IDLE-FS"

            if not self.fullspeed_only:
                with m.State("WAIT-DEVICE-CHIRP-END"):
                    m.d.comb += [
                        self.phy.op_mode.eq(UTMIOperatingModeEnum.RAW_DRIVE),
                        self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                        self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                    ]

                    with m.If(self.phy.line_state != UTMILineState.K):
                        m.d.usb += chirp_timer.eq(0)
                        m.next = "WAIT-DEVICE-CHIRP-END-SE0"

                with m.State("WAIT-DEVICE-CHIRP-END-SE0"):
                    m.d.comb += [
                        self.phy.op_mode.eq(UTMIOperatingModeEnum.RAW_DRIVE),
                        self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                        self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                    ]

                    m.d.usb += chirp_timer.eq(chirp_timer + 1)
                    with m.If(chirp_timer == self._CHIRP_DURATION):
                        m.d.usb += chirp_timer.eq(0)
                        m.next = "SEND-HOST-CHIRP-K"

                with m.State("SEND-HOST-CHIRP-K"):
                    m.d.comb += [
                        self.phy.op_mode.eq(UTMIOperatingModeEnum.CHIRP),
                        self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                        self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                        self.tx.valid.eq(1),
                        self.tx.data.eq(0x00),
                    ]

                    m.d.usb += chirp_timer.eq(chirp_timer + 1)
                    with m.If(chirp_timer >= self._CHIRP_DURATION):
                        m.d.usb += chirp_timer.eq(0)
                        m.next = "SEND-HOST-CHIRP-J"

                with m.State("SEND-HOST-CHIRP-J"):
                    m.d.comb += [
                        self.phy.op_mode.eq(UTMIOperatingModeEnum.CHIRP),
                        self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                        self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                        self.tx.valid.eq(1),
                        self.tx.data.eq(0xff),
                    ]

                    m.d.usb += chirp_timer.eq(chirp_timer + 1)

                    with m.If(chirp_timer >= self._CHIRP_DURATION):
                        m.d.usb += chirp_timer.eq(0)
                        with m.If(reset_counter >= self._MAX_RESET_TIME):
                            m.next = "IDLE-HS"
                        with m.Else():
                            m.next = "SEND-HOST-CHIRP-K"

            with m.State("IDLE-FS"):
                m.d.comb += [
                    in_idle_state.eq(1),
                    self.reset_active.eq(0),
                    self.detected_speed.eq(USBHostSpeed.FULL),
                ]

                with m.If(self.bus_reset):
                    m.d.usb += chirp_timer.eq(0)
                    m.next = "DISCONNECTED"

            if not self.fullspeed_only:
                with m.State("IDLE-HS"):
                    m.d.comb += [
                        in_idle_state.eq(1),
                        self.reset_active.eq(0),
                        self.detected_speed.eq(USBHostSpeed.HIGH),
                        self.phy.xcvr_select.eq(USBHostSpeed.HIGH),
                        self.phy.term_select.eq(UTMITerminationSelectEnum.HS_NORMAL),
                    ]

                    with m.If(self.bus_reset):
                        m.d.usb += chirp_timer.eq(0)
                        m.next = "DISCONNECTED"

        return m
