# Getting Started

In this doc we briefly describe how to use PMC. We will assume that you use
GNU/Linux.

## Prepare

First you need to have the appropriate hardware. Currently only prototype is
available. It is based on stm32f4discovery board and custom power board.

To upload software you need to connect to the board USART. Then install our
fork of [stmflasher](https://bitbucket.org/amaora/stmflasher) or any
replacement software. Also you will need
[picocom](https://github.com/npat-efault/picocom) or another terminal
program. Get the PMC source.

	$ hg clone https://bitbucket.org/amaora/phobia

Adapt src/Makefile to you needs. Probably will need to change compiler prefix,
tty name, ld script. Then build it and flash the binary.

	$ make flash

Once binary is uploaded a terminal program is connected to the tty. If all goes
well you will see a shell prompt.

## Configure a new motor

Connect the motor to the power terminals. Remove any load from it, allow it
rotate freely. Then do a base parameters identification.

	# ap_identify_base

Verify that obtained parameters are within the permissible range. Set the
number of the rotor pole pairs if you know.

	# pm_const_Zp <n>

Set BEMF constant also known as Kv constant. If you do not know it exactly you
could try to guess a close value. It is important to set Zp correctly before.

	# pm_const_E_kv <value>

Then you can try a first start. Disable HFI and position control loop.

	# pm_m_bitmask_high_frequency_injection 0
	# pm_m_bitmask_position_control_loop 0

Set a low current limit.

	# pm_i_maximal 2

Align the rotor.

	# pm_request_wave_hold

Now try to run.

	# pm_request_start
	# pm_i_set_point_Q 1

The motor will run without speed limit. Keep in mind it may be dangerous.
Alternatively you could use position control loop at the first start.

Next stage is to estimate BEMF constant more accurately. Do this when motor is
in run. Choice of the speed may affect the result.

	# ap_update_const_E

This completed the basic configuration.

