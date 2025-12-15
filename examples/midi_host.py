#!/usr/bin/env python3
#
# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

"""
USB MIDI Host example.

Enumerates a USB MIDI device, outputs received data as hex over UART
at 115200 baud, and displays a packet counter on onboard LEDs.
"""

from amaranth import *
from amaranth.lib import wiring

from guh.engines.midi import USBMIDIHost
from guh.util.clocks import CLOCK_FREQUENCIES_60MHZ
from guh.util.hexdump import HexDump


class USBMIDIHostExample(Elaboratable):

    def elaborate(self, platform):
        m = Module()

        m.submodules.car = platform.clock_domain_generator(clock_frequencies=CLOCK_FREQUENCIES_60MHZ)

        # Hardwire VBUS power to the TARGET USB A port
        vbus_en = platform.request("control_vbus_en")
        m.d.comb += vbus_en.o.eq(1)

        ulpi = platform.request("target_phy")
        m.submodules.midi_host = midi_host = USBMIDIHost(bus=ulpi)

        # UART Hex dump (115200 baud at 60MHz)
        uart_pins = platform.request("uart")
        m.submodules.hexdump = hexdump = HexDump(divisor=520)
        m.d.comb += uart_pins.tx.o.eq(hexdump.tx)
        if hasattr(uart_pins.tx, 'oe'):
            m.d.comb += uart_pins.tx.oe.eq(1)  # Cynthion has tristate UART TX

        wiring.connect(m, midi_host.o_midi, hexdump.i)

        # Count complete USB-MIDI events (4 bytes each, marked by 'last')
        packet_count = Signal(32)
        with m.If(midi_host.o_midi.valid & midi_host.o_midi.ready & midi_host.o_midi.payload.last):
            m.d.usb += packet_count.eq(packet_count + 1)

        # Display lower bits of packet count on LEDs
        leds = Cat(platform.request("led", n).o for n in range(2))
        m.d.comb += leds.eq(packet_count[:2])

        return m


if __name__ == "__main__":
    from luna import top_level_cli
    top_level_cli(USBMIDIHostExample)
