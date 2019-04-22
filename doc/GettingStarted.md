## Overview

This manual gives a basic info about Phobia Motor Controller (PMC). Look into
other docs for specific issues.

## Hardware

We do not assemble hardware for sales. You can get appropriate revision of PCB
from repo and order fabrication and assembly somewhere.

	# hg clone https://bitbucket.org/amaora/phobia-pcb

The aim of our PCB design is to optimize electrical and thermal performance.
We are not trying to cram all the components into a small volume. However, we
sometimes cross the border of quality in favor of PCB size.

## Software

There is two parts of software:

1. Numerical BLDC model of BLDC. The model enables us to develop control code
   in fast cycle without hardware tests. It is complete enough to take into
   account all of motor parameters.
2. Firmware for MCU.

The firmware can be compiled with appropriate [GCC](https://gcc.gnu.org/)
toolchain for Cortex-M4F target.

	# hg clone https://bitbucket.org/amaora/phobia
	# cd phobia/src
	# make flash

We use [stmflasher](https://bitbucket.org/amaora/stmflasher) to upload the
firmware into MCU. You should have a serial port connected to the board USART
pins and BOOT pin shorted to the +3.3v. Alternatively, you can use SWD.

After the firmware is loaded the command line interface (CLI) will be available
via the serial port. We use [picocom](https://github.com/npat-efault/picocom)
terminal program.

	# make connect

## Further reading

[Command Line Interface](CLI.md)\
[Motor Identification](MotorIdentification.md)\
[Motor Tuning](MotorTuning.md)\
[Input Pulse Width](InputPulseWidth.md)\
[Trouble Shooting](TroubleShooting.md)\

[State Observer](StateObserver.md)\
[High Frequency Injection](HFI.md)\

