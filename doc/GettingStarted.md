## Overview

This manual gives a basic info about Phobia Motor Controller (PMC). Look into
other docs for specific questions.

## Hardware

We do not assemble hardware for sales. You can get appropriate revision of PCB
from repo and order the fabrication and assembly somewhere.

	$ hg clone https://bitbucket.org/amaora/phobia-pcb

## Software

You can upload software through USART or SWD. To configure PMC you need a
terminal program. We suggest you satisfy the following requirements.

* arm-non-eabi toolchain
* [stmflasher](https://bitbucket.org/amaora/stmflasher)
* [picocom](https://github.com/npat-efault/picocom)

Adapt our Makefile if you need any other toolchain or flasher or terminal.

	$ hg clone https://bitbucket.org/amaora/phobia
	$ make flash

Once software is uploaded you should configure PMC through USART or CAN. There
is command line interface.

## Basic configuration

Connect the motor to the terminals. First do a base parameters identification.

	# ap_identify_base

Verify that obtained parameters are within the permissible range. Set the
number of the rotor pole pairs if you know.

	# pm_const_Zp <n>

Set BEMF constant also known as Kv constant. If you do not know it exactly you
could try to guess a close value. It is important to set Zp correctly before.

	# pm_const_E_kv <value>

Then you can try a first start. Remove any load from the motor, allow it rotate
freely. You can disable HFI and position control loop for the first time.

	# pm_m_bitmask_high_frequency_injection 0
	# pm_m_bitmask_position_control_loop 0

Set a low current limit.

	# pm_i_high_maximal 2
	# pm_i_low_maximal 2

Also set a voltage utilisation to limit a maximal achievable speed.

	# pm_vsi_u_maximal .2

Align the rotor.

	# pm_request_wave_hold

Try to run.

	# pm_request_start
	# pm_i_set_point_Q 1

Next stage is to estimate BEMF constant more accurately. Do this when motor is
in run. Choice of different speed may give different results.

	# ap_identify_const_E

There can be a many alternatives if some step fails. Look into command
description for details. This completes the basic configuration.

## Usage

Once configuration is done you can run the motor with controlled torque or
speed or absolute position.

