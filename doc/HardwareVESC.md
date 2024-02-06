## Overview

This page gives you an overview of VESC hardware supported by PMC.

## Hardware

| Item                                  | HWREV     | Notes             |
|:--------------------------------------|:---------:|:------------------|
| FLIPSKY FSESC 6.7                     | VESC6A    | Bad PCB design    |
| Makerbase VESC 60100 V2               | VESC6A    |                   |
| FLIPSKY MINI FSESC 6.7 PRO            | VESC6B    | Low side shunts   |
| VESC 6 MkVI                           | VESC6C    | Original          |
| Holybro Mini FOC ESC Based on VESC6   | VESC6H    |                   |
| FLIPSKY 75100 V202 ESC                | VESC75A   | Low side shunts   |
| Makerbase VESC 75200 V2 84V 200A      | VESC75A   |                   |
| VESC 75/300 R3                        | VESC75B   | Original          |

Note that some of VESC clone have bad PCB design that causes distorted current
measurement and total malfunction at high load. Prefer to use PCB with inline
current measurement instead of low side shunts.

You are also welcome to look into `src/hal/hw/...` directory to get a full
actual information about specific hardware port.

You can build the firmware binary yourself from sources or get it from the
bundle release.

	$ cd phobia/src
	$ make HWREV=VESC6A

To load the firmware into the MCU first time you will need SWD probe. Next time
to upgrade the firmware you can use USB DFU or serial bootloader.

	$ make HWREV=VESC6A stlink

Also check [GettingStarted](GettingStarted.md) page howto build and load the
firmware to MCU.

## Interfaces

Note that VESC does not have voltage dividers on ADC pins. You should use
external dividers or specify 3.3v levels on analog knob range configuration.

