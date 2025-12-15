# Copyright (c) 2024 S. Holzapfel <me@sebholzapfel.com>
#
# SPDX-License-Identifier: BSD-3-Clause
"""
Common clock frequency configurations.
"""

# All domains at 60MHz - suitable for USB host examples on both Tiliqua and Cynthion
# TODO: test with different clock frequencies? I had some issues with sync=120 so 
# there might be some CDC dragons to be squashed.

CLOCK_FREQUENCIES_60MHZ = {
    "fast": 60,
    "sync": 60,
    "usb":  60,
}
