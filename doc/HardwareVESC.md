## Overview

This page gives you an overview of VESC hardware supported by PMC.

## Hardware

| Item                                  | HWREV       | Notes             |
|:--------------------------------------|:-----------:|:------------------|
| FLIPSKY FSESC 6.7                     | FSESC67     | Bad PCB design    |
| FLIPSKY MINI FSESC 6.7 PRO            | FSESC67MINI | Low side shunts   |
| FLIPSKY 75100 V202 ESC                | FSESC75100  | Low side shunts   |
| Holybro Mini FOC ESC Based on VESC6   | HBESC6FOC   |                   |
| Makerbase VESC 60100 V2               | MKESC60100  |                   |
| Makerbase VESC 84200                  | MKESC84200  |                   |
| VESC 6 MkVI                           | VESC6MK5    |                   |
| VESC 75/300 R3                        | VESC75300   |                   |

Note that some of VESC clone have bad PCB design that causes distorted current
measurement and total malfunction at high load. Also prefer to use PCB with
inline current measurement instead of low side shunts.

You are also welcome to look into `src/hal/hw/...` directory to get a full
actual information about specific hardware port.

You can build the firmware binary yourself from sources or get it from the
bundle release.

	$ cd phobia/src
	$ make HWREV=MKESC60100

To load the firmware into the MCU first time you will need SWD probe. Next time
to upgrade the firmware you can use USB DFU or serial bootloader.

	$ make HWREV=MKESC60100 stlink

Also check [GettingStarted](GettingStarted.md) page howto build and load the
firmware to MCU.

## Analog interfaces

Note that VESC does not have voltage dividers on ADC pins. You should use
external dividers or specify 3.3v levels on analog knob range configuration.

## Terminal voltages

Note that VESC does not have required RC filters on the terminal voltage
measurement circuits. So you are not able to use dead-time distortion
compensation.

