## Overview

This manual gives a basic info about Phobia Motor Controller (PMC). See other
documents for specific topics.

## Hardware

We do not assemble hardware for sales. You can get fabrication files from our
[releases](https://sourceforge.net/projects/phobia/files/) or look into the PCB
[repo](https://github.com/rombrew/phobia-pcb). You will have to order the
fabrication and assembly yourself.

	$ git clone https://github.com/rombrew/phobia-pcb.git phobia-pcb

The aim of our PCB design is to optimize electrical and thermal performance.
We are not trying to cram all the components into a small volume. However, we
sometimes cross the border of quality in favor of PCB size.

Our recent HW revision is `REV5` designed in 8-layer PCB with 35um copper foil
thickness. To improve heat dissipation it is necessary to mount an aluminium
heatsink at bottom side through isolated thermal interface.

You can also try to use third-party hardware like VESC or its clones. Look into
[Hardware VESC](HardwareVESC.md) document to get an overview of supported
third-party hardware.

## Basic wiring

Plug PMC according to the diagram. If you need to run a bootloader (in case of
erased MCU) then short BOOT pin to +3.3v before the power up.

```
	 +-----------+
	 |           |               +---------------+
	 |  Host PC  |-------//------| USART adapter |
	 |           |               |               |
	 +-----------+               |  GND  RX  TX  |
	                             +---+---+---+---+
	                                 |   |   |
	                                 |   |   |
	                 +---------------+---+---+------+           Motor
	 +---------+     |              GND  TX  RX     |           -----
	 |         |-----|                              |----------/     \
	 | Battery |     |    Phobia Motor Controller   |---------|   o   |
	 |         |-----|                              |----------\     /
	 +---------+     |       BOOT  +3.3v            |           -----
	                 +--------+------+--------------+
	                          |      |
	                          +--/ --+
```

**WARNING**: You also can use an USB connection if your hardware has one.

## Software

PMC software consist of several main parts.

1. Workbench includes numerical model of VSI with PMSM connected (`bench/...`).
   The numerical model enables us to develop control code in fast cycle without
   hardware tests. It is complete enough to take into account all of machine
   parameters. We also provide some set of automated tests which uses a
   numerical model. But keep in mind that only abstract control code from
   `src/phobia/...` directory is covered by these tests.

2. Phobia Graphical User Interface (`pgui/...`). It is a user tool to configure
   and diagnose PMC in visual way. This frontend communicates with PMC using
   the CLI via serial port or USB.

3. Firmware code for onboard MCU (`src/...`). All of PMSM control algorithms
   and related functions are implemented here.

You can build the software from sources yourself or get binaries from bundle
[resleases](https://github.com/rombrew/phobia/releases).

## Build

The firmware can be compiled with appropriate [GCC](https://gcc.gnu.org/) or
[Clang](https://clang.llvm.org/) toolchain. Setting up the build toolchain is
beyond the scope of this document.

For example let us build the firmware for our `REV5` hardware.

	$ git clone https://github.com/rombrew/phobia.git phobia
	$ cd phobia/src
	$ make HWREV=PHOBIA_rev5

You can specify cross-compiler prefix `CROSS` and serial device name `TTY`.

	$ make HWREV=PHOBIA_rev5 CROSS=armv7m-none-eabi TTY=/dev/rfcomm0

If you modified source code files it may be necessary to regenerate `defs.h`
files by `mkconfig` python script before build.

	$ ./mkconfig
	$ make HWREV=PHOBIA_rev5

## Load

There are several ways to load the firmware into the MCU.

SWD interface with [GDB](https://www.gnu.org/software/gdb/). We use
[Black Magic Probe](https://1bitsquared.com/products/black-magic-probe). Be
careful when using hardware debugging while the machine is running. The sudden
stop of feedback loop can cause overcurrent accident.

	$ make HWREV=PHOBIA_rev5 gdb
	(gdb) load

SWD interface with [STLINK](https://github.com/stlink-org/stlink). You can use
GDB as in previous case but we are only concerned with using `st-flash` util.

	$ make HWREV=PHOBIA_rev5 stlink

USART interface with ST embedded bootloader using
[stm32flash](https://sourceforge.net/projects/stm32flash/). You should have a
serial port connected to the board TX and RX pins and BOOT pin shorted to the +3.3v.

	$ make HWREV=PHOBIA_rev5 flash

USB interface with ST embedded bootloader using
[DFU](http://dfu-util.sourceforge.net/). You should have an USB port connected
to the board and BOOT pin shorted to the +3.3v.

	$ make HWREV=PHOBIA_rev5 dfu

After MCU is flashed the Command Line Interface (CLI) will be presented on
USART, USB and CAN interfaces. We use
[picocom](https://github.com/npat-efault/picocom) terminal program. Default
baudrate is 57600 with 8-bits data 1-bit even parity and 1-bit stop.

	$ make TTY=/dev/ttyUSB0 connect

If MCU was already flashed with PMC firmware you are able to activate ST
embedded bootloader without BOOT pin. Just run the command in the CLI.

	(pmc) ap_bootload

## Basic configuration

We recommend you to do built-in [integrity check](IntegritySelfTest.md) and
self-adjustment the first time you power PMC up. Then read all of documentation
for setting PMC up.

- [Command Line Interface](CommandLineInterface.md)
- [Graphical User Interface](GraphicalUserInterface.md)
- [Hardware Design](HardwareDesign.md)
- [Hardware VESC](HardwareVESC.md)
- [Integrity Self Test](IntegritySelfTest.md)
- [Machine Probe](MachineProbe.md)
- [Machine Tuning](MachineTuning.md)
- [High Frequency Injection](HighFrequencyInjection.md)
- [Input Analog Knob](InputAnalogKnob.md)
- [Input Pulse Width](InputPulseWidth.md)
- [Input STEP/DIR Interface](InputStepDirection.md)
- [Network CAN](NetworkCAN.md)
- [Trouble Shooting](TroubleShooting.md)

## Feedback and support

Yuo can contact me on [sourceforge](https://sourceforge.net/projects/phobia/)
or [github](https://github.com/rombrew/phobia) as well as by email.

Roman Belov <romblv@gmail.com>

