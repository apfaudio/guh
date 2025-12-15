#!/usr/bin/env python3
#
# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

"""
USB Mass Storage Host example.

Enumerates a USB mass storage device, reads block 0, and outputs the
contents as hex over UART at 115200 baud repeatedly.
"""

from amaranth import *
from amaranth.lib import wiring

from guh.engines.msc import USBMSCHost
from guh.util.clocks import CLOCK_FREQUENCIES_60MHZ
from guh.util.hexdump import HexDump


class USBMSCHostExample(Elaboratable):

    def elaborate(self, platform):
        m = Module()

        m.submodules.car = platform.clock_domain_generator(clock_frequencies=CLOCK_FREQUENCIES_60MHZ)

        # Hardwire VBUS on
        vbus_en = platform.request("control_vbus_en")
        m.d.comb += vbus_en.o.eq(1)

        ulpi = platform.request("target_phy")
        m.submodules.msc_host = msc_host = USBMSCHost(bus=ulpi)

        # UART Hex dump (115200 baud at 60MHz)
        uart_pins = platform.request("uart")
        m.submodules.hexdump = hexdump = HexDump(divisor=520)
        m.d.comb += uart_pins.tx.o.eq(hexdump.tx)
        if hasattr(uart_pins.tx, 'oe'):
            m.d.comb += uart_pins.tx.oe.eq(1)  # Cynthion has tristate UART TX

        wiring.connect(m, msc_host.rx_data, hexdump.i)

        # LED feedback: LED0=connected/ready, LED1=busy
        led0 = platform.request("led", 0)
        led1 = platform.request("led", 1)

        m.d.comb += led0.o.eq(msc_host.status.ready)
        m.d.comb += led1.o.eq(msc_host.status.busy)

        # trigger block read once per second (60MHz USB clock)
        timer = Signal(32)
        read_pending = Signal()

        # Count up to 1 second, then set read_pending
        with m.If(timer >= 60_000_000 - 1):
            m.d.usb += [
                timer.eq(0),
                read_pending.eq(1),
            ]
        with m.Else():
            m.d.usb += timer.eq(timer + 1)

        with m.If(msc_host.status.ready & read_pending & ~msc_host.status.busy):
            m.d.comb += [
                msc_host.cmd.start.eq(1),
                msc_host.cmd.lba.eq(0), # Block we want to read (first one over and over...)
            ]
            m.d.usb += read_pending.eq(0)

        return m


if __name__ == "__main__":
    from luna import top_level_cli
    top_level_cli(USBMSCHostExample)
