## Overview

This manual gives a basic info about Phobia Motor Controller (PMC). Look into
other docs for specific questions.

## Hardware

We do not assemble hardware for sales. You can get appropriate revision of PCB
from repo and order fabrication and assembly somewhere.

	$ hg clone https://bitbucket.org/amaora/phobia-pcb

## Software

You can upload software through USART or SWD. To configure PMC you need a
terminal program. We suggest you satisfy the following requirements.

* arm-none-eabi toolchain
* [stmflasher](https://bitbucket.org/amaora/stmflasher)
* [picocom](https://github.com/npat-efault/picocom)

Adapt our Makefile if you need any other toolchain or flasher or terminal.

	$ hg clone https://bitbucket.org/amaora/phobia
	$ cd phobia/src
	$ make flash

Once software is uploaded you should configure PMC through USART or CAN. There
is command line interface for this.

## Basic configuration

Connect the motor to the terminals. Remove any load from motor to allow it
rotate freely.

You can reset PM if you are not sure that current configuration is correct.

	# pm_default

Provide the number of the rotor pole pairs if you need a correct printouts of
rpm/v constant and mechanical speed.

	# pm_const_Zp <n>

Try automated motor probe.

	# ap_probe_base

All identified parameters will be printed out. Verify they are within the
permissible range. During the probe motor will rotate on final stages.

## Usage

Once configuration is done you can run the motor with controlled torque or
speed or absolute position. Without HFI you will need a blind spinup to go into
the high speed region.

	# ap_blind_spinup

Set Q axis current in torque control mode.

	# pm_i_set_point_Q <A>

Set RPM or absolute position in servo mode.

	# pm_p_set_point_w_rpm <rpm>
	# pm_p_track_point_x_g <x>

By default PMC is in servo mode after a blind spinup. It can be disabled as
follows.

	# pm_m_bitmask_servo_control_loop 0

