# `guh` - Gateware USB Host

`guh` is an experimental gateware library (written in [Amaranth HDL](https://amaranth-lang.org/docs/amaranth/v0.5.8/intro.html)), for building **custom USB2 high-speed and full-speed host engines<sup>*</sup>** for FPGAs. It builds heavily on [LUNA](https://github.com/greatscottgadgets/luna), which, whilst being extremely useful for implementing USB devices, does not implement USB Host. Eventually (perhaps after a lot of cleanup!) `guh` hopes to become part of LUNA in some form.

<sup>* `guh` uses the term **host engine** rather than **host controller**, as modern USB host controllers like xHCI are complex beasts with sophisticated control and DMA interfaces. `guh` does not implement an entire host controller, although it provides primitives which could be used to create one.</sup>

At the moment, `guh` provides building blocks for creating small, single-purpose host engines, using very little LUTs/area. Examples include:

- **A USB Mass Storage host**: for reading blocks from an attached thumbdrive
- **A USB HID host**: for reading scancodes from an attached keyboard
- **A USB MIDI host**: for recieving notes from attached musical instruments

As of now, this library is predominantly built to serve the needs of [Tiliqua](https://github.com/apfaudio/tiliqua), which uses this library for its USB Host functionality. That being said, a goal is to make the various components generic enough to be useful in other domains. It should be considered experimental and all interfaces are subject to change.

# Status

As of now, `guh` can enumerate USB2 high-speed (480Mbit) and full-speed (12Mbit) devices in pure gateware. A simple state machine can parse incoming descriptors to determine what kind of device is attached, and which endpoints to use. Then, a custom 'plug-in' state machine for the desired device class can directly perform transfers on the device endpoints. I have tested this on USB thumbdrives at 480Mbit HS, and MIDI/HID devices at 12Mbit FS. More device classes will obviously require more work.

The enumeration speed is dynamic: the same gateware can support both HS and LS devices. As of now, USB hubs can be enumerated *but not the devices behind them*. In the future, I hope to remove this limitation.

As a highly experimental library, proper handling of all error conditions is incomplete, although enough is implemented to enumerate most devices I have tested without stalling. For this reason, it is common for host engines to include a watchdog which resets the enumeration state machine if nothing happens for too long, as a fallback to ensure we reattach to a device if something hangs.

# Tour

The interesting bits of `guh` are:
```
guh/
├── usbh/
│   ├── reset.py        # Bus reset controller and HS/FS speed detection
│   ├── sie.py          # USB transaction engine ('SIE'): token packets, SOF generation, SETUP/IN/OUT transactions
│   ├── descriptor.py   # Descriptor parsing logic, endpoint extraction
│   └── enumerator.py   # Host enumeration state machine. Issue reset, assign device address, fetch descriptors
│                       # and so on. Once enumeration succeeds, hand off to one of the 'engines' below...
└── engines/
    ├─── midi.py        # MIDI Host engine: poll for a bytestream from an attached MIDI device.
    ├─── keyboard.py    # HID Keyboard engine: poll for all pressed keycodes on an attached keyboard.
    ├─── msc.py         # Mass Storage engine (read-only for now) - stream desired blocks from an attached USB drive or SSD.
   ...
 # more engines wanted!
```

## Simulation / Testing

In `tests/` you will find:

- `test_integration.py`: which simulates an entire host engine against a 'fake' USB device by forwarding traffic between them. It also simulates HS/FS negotiation by emulating the PHY line states. The USB packets are logged in realtime as the simulation is run (if run with `-v`), and all USB transactions also saved to a `.pcap` file for inspection in Packetry or Wireshark.
- `test_descriptor.py`: which simulates the descriptor parser against a set of real USB descriptors. Feel free to add more.

For debugging on real hardware, I suggest purchasing a USB analyzer, like [Cynthion](https://github.com/greatscottgadgets/cynthion).

# Examples

The `examples/` folder contains simple top-level bitstreams that work on both [Cynthion](https://github.com/greatscottgadgets/cynthion) and [Tiliqua](https://github.com/apfaudio/tiliqua):

| Example | Description |
|---------|-------------|
| `midi_host.py` | Enumerate a USB MIDI device, display packet count on LEDs, hex dump to UART |
| `keyboard_host.py` | Enumerate a USB HID keyboard, display key activity on LEDs, pressed ASCII keys to UART |
| `msc_host.py` | Enumerate a USB mass storage device and hexdump block 0 over UART repeatedly |

## Running Examples

Examples require the `LUNA_PLATFORM` environment variable to select your hardware:

**Cynthion:**
```bash
LUNA_PLATFORM=cynthion.gateware.platform:CynthionPlatformRev1D4 pdm run python3 examples/midi_host.py --upload
```

**Tiliqua R4/R5:**
```bash
LUNA_PLATFORM=guh.platform.tiliqua:TiliquaR4R5Platform pdm run python3 examples/midi_host.py --upload
```

Replace `midi_host.py` with any other example.

## Important Notes

**VBUS Power:** These examples hard-wire the VBUS output to ON, because this repository does not include drivers for the I2C Type-C CC controller (TUSB322I). The full [Tiliqua](https://github.com/apfaudio/tiliqua) repository includes these drivers and handles VBUS properly.

**Tiliqua USB-C Adapter:** Tiliqua has a USB Type-C receptacle instead of Type-A like Cynthion. Since this repository does not include TUSB322I drivers, you must use a **USB-C to USB-A adapter** to connect USB devices. The full Tiliqua repository does not require an adapter.

For more advanced usage, see how this library is used in the [Tiliqua](https://github.com/apfaudio/tiliqua) project.

# License

BSD 3-Clause, same as LUNA. See `LICENSE` text in this repository.
