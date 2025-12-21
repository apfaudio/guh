# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Shared test fixtures and utilities for USB Host tests.
"""

import struct

from amaranth import *
from amaranth.lib import enum, stream
from amaranth.sim import SimulatorContext

from colorama import Fore, Style

from luna.gateware.interface.utmi import UTMIOperatingMode
from luna.gateware.usb.usb2.packet import USBPacketID

class USBPcapWriter:
    """
    Utility class for writing raw USB packets to a .pcap file readable
    in Packtree or Wireshark.
    """

    LINKTYPE_USB_2_0 = 288
    PCAP_NSEC_MAGIC = 0xa1b23c4d

    def __init__(self, filename):
        self.filename = filename

    def __enter__(self):
        self.f = open(self.filename, 'wb')
        self._write_header()
        return self

    def _write_header(self):
        header = struct.pack('<IHHIIII',
            self.PCAP_NSEC_MAGIC,  # magic (nanosecond timestamps)
            2, 4,                  # version major, minor
            0, 0,                  # thiszone, sigfigs
            65535,                 # snaplen
            self.LINKTYPE_USB_2_0) # network (link type)
        self.f.write(header)

    def write_packet(self, timestamp_ns: int, data: bytes):
        packet_data = bytes(data)
        ts_sec = timestamp_ns // int(1e9)
        ts_nsec = timestamp_ns % int(1e9)
        pcap_header = struct.pack('<IIII',
            ts_sec,               # timestamp seconds
            ts_nsec,              # timestamp nanoseconds
            len(packet_data),     # number of octets saved
            len(packet_data))     # actual length
        self.f.write(pcap_header)
        self.f.write(packet_data)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.f.close()


class BusEvent(enum.Enum):
    """USB bus events for simulation monitoring."""
    IDLE         = 0   # Bus idle (J state) or packet TX (printed separately)
    HOST_RESET   = 1   # Host driving SE0 (bus reset)
    DEV_CHIRP_K  = 2   # Device chirping K
    DEV_CHIRP_J  = 3   # Device chirping J
    HOST_CHIRP_K = 4   # Host chirping K
    HOST_CHIRP_J = 5   # Host chirping J


def connect_utmi(m, hst_utmi, dev_utmi):
    """
    Simulate a bidirectional USB connection between host and device UTMI PHY
    interfaces by forwarding packets in both directions and bridging line_state
    for proper HS chirp negotiation.
    """
    def _bridge_utmi_tx_rx(src_utmi, snk_utmi):
        """Helper to bridge one direction of UTMI TX->RX"""
        m.d.comb += [
            snk_utmi.rx_active.eq(0),
            snk_utmi.rx_valid.eq(src_utmi.tx_valid & src_utmi.tx_ready),
            snk_utmi.rx_data.eq(src_utmi.tx_data),
        ]
        # Simulate `rx_active` being asserted for a few cycles before `tx_ready`
        # is asserted. This is kind of simulating the packet sync preamble arriving
        # before the real data bytes arrive, which the LUNA depacketizer expects.
        preamble_cnt = Signal(2, init=0)
        with m.If(src_utmi.tx_valid):
            m.d.comb += snk_utmi.rx_active.eq(1)
            with m.If(preamble_cnt == 0x3):
                m.d.comb += src_utmi.tx_ready.eq(1)
            with m.Else():
                m.d.sync += preamble_cnt.eq(preamble_cnt + 1)
        with m.Else():
            m.d.sync += preamble_cnt.eq(0)

    _bridge_utmi_tx_rx(hst_utmi, dev_utmi)  # Host TX -> Device RX
    _bridge_utmi_tx_rx(dev_utmi, hst_utmi)  # Device TX -> Host RX

    # Simulate VBUS connected (session_end = 0 means VBUS present)
    m.d.comb += dev_utmi.session_end.eq(0)

    # Determine bus line state based on who's driving:
    #
    # - RAW_DRIVE mode (host reset): SE0, BUT device can override with chirp
    # - CHIRP mode (device/host chirp): K or J state from tx_data
    # - TX active (packet transmission): SE0 during sync, then data
    # - Idle: J state (0b01)
    #
    # whilst being careful with the priority of each.

    bus_line_state = Signal(2)
    bus_event = Signal(BusEvent, init=BusEvent.IDLE)

    with m.If((dev_utmi.op_mode == UTMIOperatingMode.CHIRP) & dev_utmi.tx_valid):
        # Device driving chirp: tx_data directly maps to line state:
        with m.If(dev_utmi.tx_data == 0x00):
            m.d.comb += bus_line_state.eq(0b10)  # K
            m.d.comb += bus_event.eq(BusEvent.DEV_CHIRP_K)
        with m.Else():
            m.d.comb += bus_line_state.eq(0b01)  # J
            m.d.comb += bus_event.eq(BusEvent.DEV_CHIRP_J)
    with m.Elif((hst_utmi.op_mode == UTMIOperatingMode.CHIRP) & hst_utmi.tx_valid):
        # Host driving chirp
        with m.If(hst_utmi.tx_data == 0x00):
            m.d.comb += bus_line_state.eq(0b10)  # K
            m.d.comb += bus_event.eq(BusEvent.HOST_CHIRP_K)
        with m.Else():
            m.d.comb += bus_line_state.eq(0b01)  # J
            m.d.comb += bus_event.eq(BusEvent.HOST_CHIRP_J)
    with m.Elif(hst_utmi.op_mode == UTMIOperatingMode.RAW_DRIVE):
        # Host driving bus reset (SE0)
        m.d.comb += bus_line_state.eq(0b00)  # SE0
        m.d.comb += bus_event.eq(BusEvent.HOST_RESET)
    with m.Elif(dev_utmi.tx_valid | hst_utmi.tx_valid):
        # Normal packet transmission between dev/host (hardwire to SE0
        # as the cores are no longer checking line_state!)
        m.d.comb += bus_line_state.eq(0b00)
        m.d.comb += bus_event.eq(BusEvent.IDLE)
    with m.Else():
        # Idle + connected - J state
        m.d.comb += bus_line_state.eq(0b01)
        m.d.comb += bus_event.eq(BusEvent.IDLE)

    # Both sides see the same bus line state
    m.d.comb += [
        hst_utmi.line_state.eq(bus_line_state),
        dev_utmi.line_state.eq(bus_line_state),
    ]

    return bus_event


def prettyprint_packet(prefix, timestamp_ns, packet):
    packet_id = USBPacketID.from_int(packet[0])
    print(f'[{prefix} t={timestamp_ns/1e9:.6f} {packet_id.name}]', end=' ')
    print(':'.join(f"{byte:02x}" for byte in packet))


def make_packet_capture_process(hst_utmi, dev_utmi, bus_event, pcap_filename):
    """
    Create a packet capture process that monitors UTMI traffic and writes to pcap.
    Returns an async coroutine suitable for sim.add_process().
    """
    async def process(ctx):
        packet_hst = []
        packet_dev = []
        cycle_count = 0
        last_bus_event = None

        pcap = USBPcapWriter(pcap_filename)
        try:
            pcap.__enter__()
            while True:
                timestamp_ns = int(1e9*(cycle_count/60e6))
                _, _, dev_rxv, dev_rxa, dev_rxd, hst_rxv, hst_rxa, hst_rxd, evt = await ctx.tick().sample(
                        dev_utmi.rx_valid, dev_utmi.rx_active, dev_utmi.rx_data,
                        hst_utmi.rx_valid, hst_utmi.rx_active, hst_utmi.rx_data,
                        bus_event)
                # Monitor bus events (clear packets on any transition)
                if evt != last_bus_event:
                    packet_hst = []
                    packet_dev = []
                    evt_name = BusEvent(evt).name
                    print(f'[{Fore.BLUE}EVT{Style.RESET_ALL} t={timestamp_ns/1e9:.6f}] {evt_name}')
                    last_bus_event = evt
                # Monitor Host->Device traffic
                if dev_rxv:
                    packet_hst.append(int(dev_rxd))
                if packet_hst and not dev_rxa:
                    prettyprint_packet(f"{Fore.GREEN}HST{Style.RESET_ALL}", timestamp_ns, packet_hst)
                    pcap.write_packet(timestamp_ns, packet_hst)
                    packet_hst = []
                # Monitor Device->Host traffic
                if hst_rxv:
                    packet_dev.append(int(hst_rxd))
                if packet_dev and not hst_rxa:
                    prettyprint_packet(f"{Fore.RED}DEV{Style.RESET_ALL}", timestamp_ns, packet_dev)
                    pcap.write_packet(timestamp_ns, packet_dev)
                    packet_dev = []
                cycle_count += 1
        finally:
            pcap.__exit__(None, None, None)
    return process


_usb_timing_patched = False


def patch_usb_timing_for_simulation():
    """Patch USB timing constants for faster simulation."""

    # This should only run once per context, otherwise timing gets super broken
    global _usb_timing_patched
    if _usb_timing_patched:
        return
    _usb_timing_patched = True

    from luna.gateware.usb.usb2.reset import USBResetSequencer
    from guh.usbh.reset import USBResetController
    from guh.usbh.sie import USBSOFController
    from guh.usbh.enumerator import USBHostEnumerator
    # Arbitrarily reduced such that we still enumerate at the correct speed in FS and HS
    USBResetSequencer._CYCLES_2_MILLISECONDS //= 20
    USBResetSequencer._CYCLES_2P5_MILLISECONDS //= 20
    USBResetController._SETTLE_TIME //= 10
    USBResetController._MAX_RESET_TIME //= 200
    USBResetController._MIN_RESET_BEFORE_CHIRP //= 10
    USBResetController._CHIRP_FILTER_CYCLES //= 10
    USBResetController._CHIRP_DURATION //= 10
    USBSOFController._SOF_CYCLES_FS //= 10
    USBSOFController._SOF_TX_TO_TX_MIN_FS //= 10
    USBSOFController._SOF_TX_TO_TX_MAX_FS //= 10
    USBSOFController._SOF_TX_TO_RX_MAX_FS //= 10
    USBSOFController._SOF_CYCLES_HS //= 10
    USBSOFController._SOF_TX_TO_TX_MIN_HS //= 10
    USBSOFController._SOF_TX_TO_TX_MAX_HS //= 10
    USBSOFController._SOF_TX_TO_RX_MAX_HS //= 10
    USBHostEnumerator._SOF_DELAY_RDY = 1 # only wait for one SOF on startup


# Stream test helpers lifted from:
# URL: https://github.com/zyp/katsuo-stream
# License: MIT
# Author: Vegard Storheil Eriksen <zyp@jvnv.net>

async def put(ctx: SimulatorContext, strm: stream.Interface, payload):
    ctx.set(strm.valid, 1)
    ctx.set(strm.payload, payload)
    await ctx.tick().until(strm.ready == 1)
    ctx.set(strm.valid, 0)

async def get(ctx: SimulatorContext, strm: stream.Interface):
    ctx.set(strm.ready, 1)
    payload, = await ctx.tick().sample(strm.payload).until(strm.valid == 1)
    ctx.set(strm.ready, 0)
    return payload
