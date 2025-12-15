# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Minimal Tiliqua R4/R5 platform definition for use with guh examples.

Extracted from the full Tiliqua gateware and contains just enough to
run USB host examples.

Resource names changed to match Cynthion examples:
    - target_phy: ULPI PHY
    - control_vbus_en: VBUS enable for host mode
    - led 0/1: Status LEDs on SoM (active low)

The USB C negotiation chip (TUSB322I) is not supported, so you have
to use a USB C to A recepticale for this to work. On the full
Tiliqua gateware, UFP/DFP negotiation is implemented properly.
"""

from amaranth import *
from amaranth.build import *
from amaranth.vendor import LatticeECP5Platform
from amaranth_boards.resources import *
from luna.gateware.architecture.car import LunaECP5DomainGenerator
from luna.gateware.platform.core import LUNAPlatform

from guh.util.clocks import CLOCK_FREQUENCIES_60MHZ

# -----------------------------------------------------------------------------
# SoldierCrab R3 base platform (ECP5 SoM)
# -----------------------------------------------------------------------------

class _SoldierCrabR3Base(LatticeECP5Platform):
    device       = "LFE5U-25F"
    package      = "BG256"
    speed        = "6"
    default_clk  = "clk48"

    # Bank 6/7 voltage for ULPI and PSRAM (1V8 parts on R3)
    @staticmethod
    def bank_6_7_iotype():
        return "LVCMOS18"

    resources = [
        # 48MHz master oscillator
        Resource("clk48", 0, Pins("A8", dir="i"), Clock(48e6), Attrs(IO_TYPE="LVCMOS33")),

        # PROGRAMN, triggers warm self-reconfiguration
        Resource("self_program", 0, PinsN("T13", dir="o"),
                 Attrs(IO_TYPE="LVCMOS33", PULLMODE="UP")),

        # Indicator LEDs on SoM (active low)
        Resource("led", 0, PinsN("T14", dir="o"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("led", 1, PinsN("T15", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        # USB2 ULPI PHY - named "target_phy" for Cynthion example compatibility
        ULPIResource("target_phy", 0,
            data="N1 M2 M1 L2 L1 K2 K1 K3",
            clk="T3", clk_dir="o", dir="P2", nxt="P1",
            stp="R2", rst="T2", rst_invert=True,
            attrs=Attrs(IO_TYPE="LVCMOS18")
        ),

        # oSPIRAM / HyperRAM
        Resource("ram", 0,
            Subsignal("clk",   DiffPairs("C3", "D3", dir="o"),
                      Attrs(IO_TYPE="LVCMOS18")),
            Subsignal("dq",    Pins("F2 B1 C2 E1 E3 E2 F3 G4", dir="io")),
            Subsignal("rwds",  Pins("D1", dir="io")),
            Subsignal("cs",    PinsN("B2", dir="o")),
            Subsignal("reset", PinsN("C1", dir="o")),
            Attrs(IO_TYPE="LVCMOS18")
        ),

        # Configuration SPI flash
        Resource("spi_flash", 0,
            Subsignal("sdi", Pins("T8",  dir="o")),
            Subsignal("sdo", Pins("T7",  dir="i")),
            Subsignal("cs",  PinsN("N8", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # Pseudo-supply pins
        Resource("pseudo_vccio", 0,
                 Pins("E6 E7 D10 E10 E11 F12 J12 K12 L12 N13 P13 M11 P11 P12 R6", dir="o"),
                 Attrs(IO_TYPE="LVCMOS33")),
        Resource("pseudo_gnd", 0,
                 Pins("E5 E8 E9 E12 F13 M13 M12 N12 N11", dir="o"),
                 Attrs(IO_TYPE="LVCMOS33")),
        Resource("pseudo_vccram", 0,
                 Pins("L4 M4 R5 N5 P4 M6 F5 G5 H5 H4 J4 J5 J3 J1 J2", dir="o"),
                 Attrs(IO_TYPE="LVCMOS18")),
        Resource("pseudo_gndram", 0,
                 Pins("L5 L3 M3 N6 P5 P6 F4 G2 G3 H3 H2", dir="o"),
                 Attrs(IO_TYPE="LVCMOS18")),
    ]

    connectors = [
        Connector("m2", 0,
            # 'E' side of slot
            "-     -   -   -   -  C4  -  D5   - T10 A3 D11  A2 D13  B3   -  B4 R11  A4  E4 "
            "T11  D4 M10   -   -   -  -   -   -   - "
            # Other side of slot
            "-    D6   -  C5 B5   - A5  C6   -  C7  B6  D7  A6  D8   -  C9  B7 C10   -  D9 "
            "A7  B11  B8 C11 A9 C12 B9 C13 A10 B13 B10 A13 B15 A14 A15 B14 C15 C14 B16 D14 "
            "C16   - D16   -  -"),
    ]


# -----------------------------------------------------------------------------
# Tiliqua R4/R5 motherboard resources (minimal: VBUS and UART only)
# -----------------------------------------------------------------------------

class _TiliquaMoboMinimal:
    """Minimal motherboard resources for USB host operation."""

    resources = [
        # USB: 5V supply OUT enable - named "control_vbus_en" for Cynthion compatibility
        Resource("control_vbus_en", 0, PinsN("32", dir="o", conn=("m2", 0)),
                 Attrs(IO_TYPE="LVCMOS33")),

        # RP2040 UART bridge
        UARTResource(0,
            rx="19", tx="17", conn=("m2", 0),
            attrs=Attrs(IO_TYPE="LVCMOS33", PULLMODE="UP")
        ),
    ]


# -----------------------------------------------------------------------------
# Combined Tiliqua platform
# -----------------------------------------------------------------------------

class TiliquaR4R5Platform(_SoldierCrabR3Base, LUNAPlatform):

    name = "Tiliqua R4/R5 (minimal)"
    clock_domain_generator = LunaECP5DomainGenerator
    default_usb_connection = "target_phy"

    DEFAULT_CLOCK_FREQUENCIES_MHZ = CLOCK_FREQUENCIES_60MHZ

    resources = [
        *_SoldierCrabR3Base.resources,
        *_TiliquaMoboMinimal.resources,
    ]

    connectors = [
        *_SoldierCrabR3Base.connectors,
    ]

    def toolchain_program(self, products, name):
        """Program bitstream to Soldiercrab ECP5 SRAM using openFPGALoader."""
        import subprocess
        import tempfile
        import os
        bitstream = products.get(f"{name}.bit")
        with tempfile.NamedTemporaryFile(suffix=".bit", delete=False) as f:
            f.write(bitstream)
            f.flush()
            try:
                subprocess.run(["openFPGALoader", "-c", "dirtyJtag", f.name], check=True)
            finally:
                os.unlink(f.name)
