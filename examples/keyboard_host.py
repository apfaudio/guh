#!/usr/bin/env python3
#
# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

"""
USB Keyboard Host example.

Enumerates a USB HID keyboard and prints typed characters over UART
at 115200 baud. LED0 lights when any key is pressed, LED1 on shift.
"""

from amaranth import *
from amaranth.lib.memory import Memory

from luna.gateware.interface.uart import UARTTransmitter

from guh.engines.keyboard import USBKeyboardHost, KeyboardReport
from guh.util.clocks import CLOCK_FREQUENCIES_60MHZ

# TODO: shrink or remove these lookup tables. We should be able to
# reduce area usage quite a bit by dropping them.

# HID keycode to ASCII lookup (unshifted)
# Keycodes 0x04-0x1D = a-z, 0x1E-0x27 = 1-0, 0x2C = space, 0x28 = enter
HID_TO_ASCII = [0] * 256
for i, c in enumerate("abcdefghijklmnopqrstuvwxyz"):
    HID_TO_ASCII[0x04 + i] = ord(c)
for i, c in enumerate("1234567890"):
    HID_TO_ASCII[0x1E + i] = ord(c)
HID_TO_ASCII[0x2C] = ord(' ')   # Space
HID_TO_ASCII[0x28] = ord('\r')  # Enter

# Shifted versions (uppercase letters, symbols on number keys)
HID_TO_ASCII_SHIFT = [0] * 256
for i, c in enumerate("ABCDEFGHIJKLMNOPQRSTUVWXYZ"):
    HID_TO_ASCII_SHIFT[0x04 + i] = ord(c)
for i, c in enumerate("!@#$%^&*()"):
    HID_TO_ASCII_SHIFT[0x1E + i] = ord(c)
HID_TO_ASCII_SHIFT[0x2C] = ord(' ')
HID_TO_ASCII_SHIFT[0x28] = ord('\r')


class USBKeyboardHostExample(Elaboratable):

    def elaborate(self, platform):
        m = Module()

        m.submodules.car = platform.clock_domain_generator(
            clock_frequencies=CLOCK_FREQUENCIES_60MHZ)

        # Hardwire VBUS power to the TARGET USB A port
        vbus_en = platform.request("control_vbus_en")
        m.d.comb += vbus_en.o.eq(1)

        ulpi = platform.request("target_phy")
        m.submodules.kbd_host = kbd_host = USBKeyboardHost(bus=ulpi)

        # UART transmitter (115200 baud at 60MHz)
        uart_pins = platform.request("uart")
        m.submodules.uart = uart = UARTTransmitter(divisor=520)
        m.d.comb += uart_pins.tx.o.eq(uart.tx)
        if hasattr(uart_pins.tx, 'oe'):
            m.d.comb += uart_pins.tx.oe.eq(1)  # Cynthion has tristate UART TX

        # HID to ASCII lookup tables
        m.submodules.ascii_mem = ascii_mem = Memory(shape=8, depth=256, init=HID_TO_ASCII)
        m.submodules.ascii_shift_mem = ascii_shift_mem = Memory(shape=8, depth=256, init=HID_TO_ASCII_SHIFT)
        ascii_rd = ascii_mem.read_port(domain="usb")
        ascii_shift_rd = ascii_shift_mem.read_port(domain="usb")

        # Track previous key to detect new presses
        prev_key0 = Signal(8)
        current_report = Signal(KeyboardReport)
        shift_held = Signal()

        # Connect lookup address
        m.d.comb += [
            ascii_rd.addr.eq(current_report.key0),
            ascii_shift_rd.addr.eq(current_report.key0),
            # Shift held if either left or right shift is pressed
            shift_held.eq(current_report.modifiers.left_shift | current_report.modifiers.right_shift),
        ]

        # Select ASCII based on shift state
        ascii_char = Signal(8)
        m.d.comb += ascii_char.eq(Mux(shift_held, ascii_shift_rd.data, ascii_rd.data))

        with m.FSM(domain="usb"):
            with m.State("IDLE"):
                m.d.comb += kbd_host.o_report.ready.eq(1)
                with m.If(kbd_host.o_report.valid):
                    m.d.usb += current_report.eq(kbd_host.o_report.payload)
                    m.next = "CHECK"

            with m.State("CHECK"):
                with m.If((current_report.key0 != prev_key0) & (current_report.key0 != 0)):
                    m.d.usb += prev_key0.eq(current_report.key0)
                    m.next = "SEND"
                with m.Else():
                    m.d.usb += prev_key0.eq(current_report.key0)
                    m.next = "IDLE"

            with m.State("SEND"):
                with m.If(ascii_char != 0):
                    m.d.comb += [
                        uart.stream.valid.eq(1),
                        uart.stream.payload.eq(ascii_char),
                    ]
                    with m.If(uart.stream.ready):
                        m.next = "IDLE"
                with m.Else():
                    m.next = "IDLE"

        # LED0=any key pressed, LED1=shift held
        m.d.comb += [
            platform.request("led", 0).o.eq(current_report.key0 != 0),
            platform.request("led", 1).o.eq(shift_held),
        ]

        return m


if __name__ == "__main__":
    from luna import top_level_cli
    top_level_cli(USBKeyboardHostExample)
