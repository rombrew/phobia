## Overview

This page gives you an overview of VESC hardware supported by PMC.

## Hardware

| Item                                 | HWREV           | Notes                          |
|:-------------------------------------|:----------------|:-------------------------------|
| FLIPSKY FSESC 6.7                    | FSESC_67        | Distorted current measurement  |
| FLIPSKY MINI FSESC 6.7 PRO           | FSESC_67_mini   | Low side shunts                |
| FLIPSKY 75100 V202 ESC               | FSESC_75100_r2  | Low side shunts                |
| Holybro Mini FOC ESC Based on VESC6  | HBRO_foc_mini   |                                |
| Makerbase VESC 60100 V2              | MKESC_60100_r2  | DRV OCP malfunction            |
| Makerbase VESC 84200                 | MKESC_84200     |                                |
| VESC 6 MkVI                          | VESC_60_mk6     |                                |
| VESC 75/300 R3                       | VESC_75_300_r3  |                                |

Note that some of VESC clones have bad PCB design that causes distorted
measurement and total malfunction at high load. Also prefer to use PCB with
inline current measurement instead of low side shunts.

You are also welcome to look into `src/hal/hw/...` directory to get a full
actual information about specific hardware port.

You can build the firmware binary yourself from sources or get it from the
bundle [resleases](https://github.com/rombrew/phobia/releases).

	$ cd phobia/src
	$ make HWREV=MKESC_60100_r2

To load the firmware into the MCU first time you will need SWD probe. Next time
to upgrade the firmware you can use USB DFU or serial bootloader.

	$ make HWREV=MKESC_60100_r2 stlink

Also check [GettingStarted](GettingStarted.md) page howto build and load the
firmware to MCU.

## Analog interfaces

VESC does not have voltage dividers on ADC pins. You should use external
dividers or specify 3.3v levels in analog knob range configuration.

## References

Get more info about original VESC motor controllers from official
[VESC Project](https://vesc-project.com/) website.

