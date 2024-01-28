## Overview

This manual gives a basic info about Phobia Motor Controller (PMC). Look into
other documents for specific questions.

## Hardware

We do not assemble hardware for sales. You can get fabrication files from our
[releases](https://sourceforge.net/projects/phobia/files/) or look into the PCB
repos. You will have to order the fabrication and assembly yourself.

	$ git clone https://github.com/rombrew/phobia-pcb.git phobia-pcb

The aim of our PCB design is to optimize electrical and thermal performance.
We are not trying to cram all the components into a small volume. However, we
sometimes cross the border of quality in favor of PCB size.

Our recent HW revision is `REV5A` designed in 8-layer PCB with 35um or 70um
copper foil thickness. To improve heat dissipation it is necessary to mount an
aluminium heatsink at bottom side through thermal interface.

You can also try to use third-party hardware like VESC or its clones. Look into
[Hardware VESC](HardwareVESC.md) document to get an overview of supported
hardware.

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

## Software

There are a few parts of software:

1. Workbench includes numerical model of VSI with PMSM connected (`bench/...`).
   The numerical model enables us to develop control code in fast cycle without
   hardware tests. It is complete enough to take into account all of machine
   parameters. We also provide some set of automated tests which uses a
   numerical model. But keep in mind that only abstract control code from
   `src/phobia/...` directory is covered by these tests.

2. Phobia graphical frontend application (`phobia/...`). It is a user tool to
   configure and diagnose PMC in visual way. This frontend communicates with
   PMC using the CLI via serial port or USB.

3. Firmware code for onboard MCU (`src/...`). All of control algorithms are
   implemented here.

The firmware can be compiled with appropriate [GCC](https://gcc.gnu.org/) or
[Clang](https://clang.llvm.org/) toolchain. For example, let us build the
firmware for the `REV5A` hardware.

	$ git clone https://github.com/rombrew/phobia.git phobia
	$ cd phobia/src
	$ make HWREV=REV5A

So using the above commands we have built the firmware. Next there are a few
ways to load the firmware into the MCU:

SWD interface with [GDB](https://www.gnu.org/software/gdb/). We use
[Black Magic Probe](https://1bitsquared.com/products/black-magic-probe). Be
careful when using hardware debugging while the machine is running. The sudden
stop of feedback loop can cause overcurrent accident.

	$ make HWREV=REV5A gdb
	(gdb) load

SWD interface with [STLINK](https://github.com/stlink-org/stlink). You can use
GDB as in previous case but we are only concerned with using `st-flash` util.

	$ make HWREV=REV5A stlink

USART interface with ST embedded bootloader using
[stm32flash](https://sourceforge.net/projects/stm32flash/). You should have a
serial port connected to the board TX and RX pins and BOOT pin shorted to the
+3.3v.

	$ make HWREV=REV5A flash

USB interface with ST embedded bootloader using
[DFU](http://dfu-util.sourceforge.net/). You should have an USB port connected
to the board and BOOT pin shorted to the +3.3v.

	$ make HWREV=REV5A dfu

After MCU is flashed the Command Line Interface (CLI) will be presented on
USART, USB and CAN interfaces. We use
[picocom](https://github.com/npat-efault/picocom) terminal program. Default
baudrate is 57600 with 8-bits data 1-bit even parity and 1-bit stop.

	$ make TTY=/dev/ttyUSB0 connect

If MCU was already flashed with PMC firmware you are able to activate ST
embedded bootloader without BOOT pin. Just run the command in the CLI.

	(pmc) ap_bootload

Read the following documentation for setting PMC up.

* [Command Line Interface](CommandLineInterface.md)
* [Hardware Design](HardwareDesign.md)
* [Hardware VESC](HardwareVESC.md)
* [Integrity Self Test](IntegritySelfTest.md)
* [Machine Probe](MachineProbe.md)
* [Machine Tuning](MachineTuning.md)
* [High Frequency Injection](HighFrequencyInjection.md)
* [Input Analog Knob](InputAnalogKnob.md)
* [Input Pulse Width](InputPulseWidth.md)
* [Input STEP/DIR Interface](InputStepDirection.md)
* [Network CAN](NetworkCAN.md)
* [Trouble Shooting](TroubleShooting.md)

## Feedback and support

Yuo can contact me on [sourceforge](https://sourceforge.net/projects/phobia/)
[github](https://github.com/rombrew/phobia) or by email.

Roman Belov <romblv@gmail.com>

