# What is this?

These are binary USB device descriptors used for testing the gateware descriptor parser. On Linux these are easy to fetch from sysfs with something like:

```
    $ lsusb
1d6b:0003 (bus 8, device 1)
1d6b:0002 (bus 7, device 1)
1d6b:0003 (bus 6, device 1)
1d6b:0002 (bus 5, device 1)
1d6b:0003 (bus 4, device 1)
0489:e0d8 (bus 3, device 2) path: 5
046d:c08b (bus 3, device 7) path: 1 # was plugged: bus 3 path 1
1d6b:0002 (bus 3, device 1)
1d6b:0003 (bus 2, device 1)
30c9:00c2 (bus 1, device 2) path: 1
1d6b:0002 (bus 1, device 1)

    $ cd /sys/bus/usb/devices/

    $ xxd 3-1/product # check it's the device you want
00000000: 4735 3032 2048 4552 4f20 4761 6d69 6e67  G502 HERO Gaming
00000010: 204d 6f75 7365 0a                         Mouse.

    $ xxd 3-1/descriptors
00000000: 1201 0002 0000 0040 6d04 8bc0 0327 0102  .......@m....'..
00000010: 0301 0902 3b00 0201 04a0 9609 0400 0001  ....;...........
00000020: 0301 0200 0921 1101 0001 2243 0007 0581  .....!...."C....
00000030: 0308 0001 0904 0100 0103 0000 0009 2111  ..............!.
00000040: 0100 0122 9700 0705 8203 1400 01         ...".........

    $ cat 3-1/descriptors > my_descriptor.bin
    # this file can be used in gateware tests
```

Now you can use your binary descriptor to debug any issues with the descriptor parser / endpoint extractor in simulation.
