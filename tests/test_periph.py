# Copyright (c) 2024 Seb Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

import types
import unittest

from amaranth import *
from amaranth.sim import *

from guh.engines.msc import USBMSCHost
from guh.periph.msc import Peripheral
from guh.util.test_devices import FakeUSBMSCDevice
from guh.util import test_util

def csr_regs(periph):
    # introspect csr offset from amaranth-soc
    return {name[-1]: (start, end - start)
            for _, name, (start, end) in periph.csr_bus.memory_map.resources()}


class PeripheralTests(unittest.TestCase):

    """
    Enqueue mixed read/write psram-dma-through-usb requests against
    a fake USB device, and make sure the peripheral does what it should.
    """

    def run_periph_testbench(self, name, body):

        m = Module()
        test_util.patch_usb_timing_for_simulation()

        msc_host = USBMSCHost(device_address=0x12)
        periph = Peripheral(msc_host=msc_host)
        m.submodules.periph = DomainRenamer({"usb": "sync"})(periph)
        m.submodules.dev = dev = DomainRenamer({"usb": "sync"})(
            FakeUSBMSCDevice(full_speed_only=False, max_packet_size=64))

        bus_event = test_util.connect_utmi(m, msc_host.sie.utmi, dev.utmi)

        # PSRAM simulation behind the wishbone bus
        psram = {}

        async def wb_ram(ctx):
            bus = periph.dma_bus
            while True:
                await ctx.tick()
                if ctx.get(bus.cyc) and ctx.get(bus.stb) and not ctx.get(bus.ack):
                    adr = ctx.get(bus.adr)
                    if ctx.get(bus.we):
                        psram[adr] = ctx.get(bus.dat_w)
                    else:
                        ctx.set(bus.dat_r, psram.get(adr, 0xDEADBEEF))
                    ctx.set(bus.ack, 1)
                else:
                    ctx.set(bus.ack, 0)

        regs = csr_regs(periph)

        async def csr_write(ctx, name, value):
            bus = periph.csr_bus
            addr, size = regs[name]
            for i in range(size):
                ctx.set(bus.addr, addr + i)
                ctx.set(bus.w_stb, 1)
                ctx.set(bus.w_data, (value >> (8 * i)) & 0xFF)
                await ctx.tick()
            ctx.set(bus.w_stb, 0)

        async def csr_read(ctx, name):
            bus = periph.csr_bus
            addr, size = regs[name]
            value = 0
            for i in range(size):
                ctx.set(bus.addr, addr + i)
                ctx.set(bus.r_stb, 1)
                await ctx.tick()
                value |= ctx.get(bus.r_data) << (8 * i)
            ctx.set(bus.r_stb, 0)
            return value

        async def start_cmd(ctx, *, lba, addr, blocks=1, write=0):
            await csr_write(ctx, "cmd_lba", lba)
            await csr_write(ctx, "cmd_addr", addr)
            await csr_write(ctx, "cmd_blocks", blocks)
            await csr_write(ctx, "cmd_dir", write)
            await csr_write(ctx, "cmd_start", 1)

        async def wait_cmds_done(ctx, count):
            for _ in range(500):
                for _ in range(500):
                    await ctx.tick()
                if await csr_read(ctx, "cmds_done") == count:
                    self.assertEqual(await csr_read(ctx, "errors"), 0)
                    return
            self.fail(f"Timed out waiting for cmds_done == {count}")

        def psram_write(byte_addr, data):
            for w in range(len(data) // 4):
                psram[(byte_addr >> 2) + w] = (
                    data[4 * w]
                    | (data[4 * w + 1] << 8)
                    | (data[4 * w + 2] << 16)
                    | (data[4 * w + 3] << 24))

        def psram_bytes(byte_addr, n):
            out = []
            for i in range(n):
                w = psram.get((byte_addr >> 2) + i // 4, 0)
                out.append((w >> (8 * (i % 4))) & 0xFF)
            return out

        h = types.SimpleNamespace(
            csr_read=csr_read, csr_write=csr_write, start_cmd=start_cmd,
            wait_cmds_done=wait_cmds_done,
            psram_write=psram_write, psram_bytes=psram_bytes)

        async def testbench(ctx):
            # Wait for device enumeration / msc init.
            for _ in range(300):
                status = await csr_read(ctx, "status")
                if status & 0x2:  # ready
                    break
                for _ in range(500):
                    await ctx.tick()
            else:
                self.fail("Peripheral never became ready")

            capacity = await csr_read(ctx, "capacity")
            self.assertEqual(capacity, FakeUSBMSCDevice.BLOCK_COUNT)

            await body(ctx, h)

        sim = Simulator(m)
        sim.add_clock(1/60e6)
        sim.add_testbench(testbench)
        sim.add_testbench(wb_ram, background=True)
        sim.add_process(test_util.make_packet_capture_process(
            msc_host.sie.utmi, dev.utmi, bus_event, f"{name}.pcap"))
        with sim.write_vcd(vcd_file=open(f"{name}.vcd", "w")):
            sim.run()

    def test_usb_msc_periph_read(self):
        async def body(ctx, h):
            await h.start_cmd(ctx, lba=42, addr=0x1000)
            await h.wait_cmds_done(ctx, 1)
            self.assertEqual(h.psram_bytes(0x1000, 512),
                             [(i ^ 42) & 0xFF for i in range(512)])

        self.run_periph_testbench("test_usb_msc_periph_read", body)

    def test_usb_msc_periph_write(self):
        async def body(ctx, h):
            # Block write: PSRAM byte address 0x2000 -> LBA 7
            write_data = [(3 * i + 1) & 0xFF for i in range(512)]
            h.psram_write(0x2000, write_data)
            await h.start_cmd(ctx, lba=7, addr=0x2000, write=1)
            await h.wait_cmds_done(ctx, 1)
            # Read back LBA 7 -> PSRAM byte address 0x3000
            await h.start_cmd(ctx, lba=7, addr=0x3000)
            await h.wait_cmds_done(ctx, 2)
            self.assertEqual(h.psram_bytes(0x3000, 512), write_data)

        self.run_periph_testbench("test_usb_msc_periph_write", body)

    def test_usb_msc_periph_back_to_back(self):
        async def body(ctx, h):
            # (lba, block count, source PSRAM address)
            spans = [(64, 2, 0x8000), (80, 8, 0x10000), (96, 16, 0x18000)]

            # fill up psram with some test data
            wr = [[(7 * i + 1 + s) & 0xFF for i in range(blocks * 512)]
                  for s, (_, blocks, _) in enumerate(spans)]
            for s, (_, _, addr) in enumerate(spans):
                h.psram_write(addr, wr[s])

            # back to back writes and reads
            for s, (lba, blocks, addr) in enumerate(spans):
                await h.start_cmd(ctx, lba=lba, addr=addr, blocks=blocks, write=1)
                if s == 1:  # interleave a read
                    await h.start_cmd(ctx, lba=300, addr=0x20000, blocks=1)
            await h.wait_cmds_done(ctx, 4)

            # check the read was good
            self.assertEqual(h.psram_bytes(0x20000, 512),
                             [(i ^ 300) & 0xFF for i in range(512)])

            # Read all three written spans back (somewhere else) and check they are good
            for s, (lba, blocks, _) in enumerate(spans):
                await h.start_cmd(ctx, lba=lba, addr=0x30000 + s * 0x2000,
                                  blocks=blocks)
            await h.wait_cmds_done(ctx, 7)
            for s, (_, blocks, _) in enumerate(spans):
                self.assertEqual(h.psram_bytes(0x30000 + s * 0x2000, blocks * 512),
                                 wr[s], f"write span {s} corrupted")

        self.run_periph_testbench("test_usb_msc_periph_back_to_back", body)


if __name__ == "__main__":
    unittest.main()
