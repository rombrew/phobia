## Overview

This page gives you an overview of VESC hardware supported by PMC.

## Hardware

| Item                                 | HWREV           | Notes           |
|:-------------------------------------|:----------------|:----------------|
| FLIPSKY FSESC 6.7                    | FSESC_67        | Bad PCB design  |
| FLIPSKY MINI FSESC 6.7 PRO           | FSESC_67_mini   | Low side shunts |
| FLIPSKY 75100 V202 ESC               | FSESC_75100_v2  | Low side shunts |
| Holybro Mini FOC ESC Based on VESC6  | HBRO_foc_mini   |                 |
| Makerbase VESC 60100 V2              | MKESC_60100_v2  |                 |
| Makerbase VESC 84200                 | MKESC_84200     |                 |
| VESC 6 MkVI                          | VESC_60_mk6     |                 |
| VESC 75/300 R3                       | VESC_75_300_r3  |                 |

Note that some of VESC clone have bad PCB design that causes distorted current
measurement and total malfunction at high load. Also prefer to use PCB with
inline current measurement instead of low side shunts.

You are also welcome to look into `src/hal/hw/...` directory to get a full
actual information about specific hardware port.

You can build the firmware binary yourself from sources or get it from the
bundle release.

	$ cd phobia/src
	$ make HWREV=MKESC_60100_v2

To load the firmware into the MCU first time you will need SWD probe. Next time
to upgrade the firmware you can use USB DFU or serial bootloader.

	$ make HWREV=MKESC_60100_v2 stlink

Also check [GettingStarted](GettingStarted.md) page howto build and load the
firmware to MCU.

## Analog interfaces

VESC does not have voltage dividers on ADC pins. You should use external
dividers or specify 3.3v levels on analog knob range configuration.

## Terminal voltages

VESC does not have required RC filters on the terminal voltage measurement
circuits. So you are not able to use dead-time distortion compensation.

