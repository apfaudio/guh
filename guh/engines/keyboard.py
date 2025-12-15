# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
USB HID Keyboard Host engine.
"""

from amaranth import *
from amaranth.lib import data, stream, wiring
from amaranth.lib.cdc import ResetInserter
from amaranth.lib.wiring import In, Out

from guh.usbh.enumerator import USBHostEnumerator
from guh.usbh.sie import TransferType, TransferResponse, DataPID
from guh.usbh.descriptor import USBDescriptorParser, EndpointFilter
from guh.protocol.descriptors import *


class KeyboardModifiers(data.Struct):
    left_ctrl:   unsigned(1)
    left_shift:  unsigned(1)
    left_alt:    unsigned(1)
    left_gui:    unsigned(1)
    right_ctrl:  unsigned(1)
    right_shift: unsigned(1)
    right_alt:   unsigned(1)
    right_gui:   unsigned(1)


class KeyboardReport(data.Struct):
    modifiers: KeyboardModifiers
    reserved:  unsigned(8)
    key0:      unsigned(8)
    key1:      unsigned(8)
    key2:      unsigned(8)
    key3:      unsigned(8)
    key4:      unsigned(8)
    key5:      unsigned(8)

KEYBOARD_REPORT_SIZE = KeyboardReport.as_shape().size // 8

class USBKeyboardHost(wiring.Component):

    """
    USB HID Keyboard Host.

    Output is a stream of KeyboardReport structs, emitted whenever the
    keyboard sends a new report (typically on key press/release).
    Polling pauses until the consumer accepts the report.
    """

    # Watchdog timeout: reset enumeration if no response for this many cycles
    # At 60MHz, 3*60000000 cycles = ~3 seconds
    _WATCHDOG_CYCLES = 3 * 60000000

    # Output stream of keyboard reports
    o_report: Out(stream.Signature(KeyboardReport))

    def __init__(self, *, bus=None, handle_clocking=True, device_address=0x12):
        self.enumerator = USBHostEnumerator(
            bus=bus,
            handle_clocking=handle_clocking,
            device_address=device_address,
            config_number=1,
            parser=USBDescriptorParser(
                endpoint_filter=EndpointFilter.IN,
                transfer_type=EndpointTransferType.INTERRUPT,
                interface_class=InterfaceClass.HID,
                interface_subclass=HIDSubClass.BOOT_INTERFACE,
                interface_protocol=HIDProtocol.KEYBOARD,
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

        pid = Signal(DataPID, init=DataPID.DATA0)
        l_sof_frame = Signal(11)

        rx_byte_count = Signal(4)
        report_bytes = Array([Signal(8, name=f"report_byte{i}") for i in range(KEYBOARD_REPORT_SIZE)])
        report = KeyboardReport(self.o_report.payload)

        # RX path: collect bytes into report_bytes
        with m.If(enum.ctrl.rxs.valid & enum.ctrl.rxs.ready):
            with m.If(rx_byte_count < KEYBOARD_REPORT_SIZE):
                m.d.usb += report_bytes[rx_byte_count].eq(enum.ctrl.rxs.payload)
            m.d.usb += rx_byte_count.eq(rx_byte_count + 1)

        with m.FSM(domain="usb"):

            with m.State("WAIT-ENUMERATION"):
                with m.If(enum.status.enumerated & enum.parser.o.valid):
                    m.d.usb += watchdog.eq(0)  # Kick watchdog on successful enumeration
                    m.next = "KBD-POLL"

            with m.State("KBD-POLL"):
                m.d.comb += enum.ctrl.rxs.ready.eq(1)

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
                            # If we received a full report, emit it
                            with m.If(rx_byte_count >= KEYBOARD_REPORT_SIZE):
                                m.next = "EMIT-REPORT"
                        with m.Case(TransferResponse.NAK):
                            # Device has no data but responded: kick watchdog
                            m.d.usb += watchdog.eq(0)
                        with m.Case(TransferResponse.STALL):
                            # STALL: let watchdog handle recovery
                            pass

            with m.State("EMIT-REPORT"):
                # Output the assembled report, wait for consumer to accept
                m.d.comb += [
                    report.modifiers.eq(report_bytes[0]),
                    report.reserved.eq(report_bytes[1]),
                    report.key0.eq(report_bytes[2]),
                    report.key1.eq(report_bytes[3]),
                    report.key2.eq(report_bytes[4]),
                    report.key3.eq(report_bytes[5]),
                    report.key4.eq(report_bytes[6]),
                    report.key5.eq(report_bytes[7]),
                    self.o_report.valid.eq(1),
                ]
                with m.If(self.o_report.ready):
                    m.next = "KBD-POLL"

        # Watchdog triggers reset of both this module and enumerator
        return ResetInserter({"usb": watchdog_expired})(m)
