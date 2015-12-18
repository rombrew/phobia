# Getting Started

In this doc we briefly describe how to use PMC. We will assume that you use
GNU/Linux.

## Prepare

First you need to have the appropriate hardware. Currently only prototype is
available. It is based on stm32f4discovery board and custom power board.

To upload the software you need to connect to the board USART. Install our fork
of [stmflasher](https://bitbucket.org/amaora/stmflasher) or any replacement
software. Also you will need [picocom](https://github.com/npat-efault/picocom)
or another terminal program. Get the PMC source.

	$ hg clone https://bitbucket.org/amaora/phobia

Adapt src/Makefile to your needs. Probably will need to change compiler prefix,
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

Then you can try a first start. Make sure that HFI and position control loop
are disabled.

	# pm_m_bitmask_high_frequency_injection 0
	# pm_m_bitmask_position_control_loop 0

Set a low current limit.

	# pm_i_high_maximal 2
	# pm_i_low_maximal 2

Also set a voltage utilisation in irder to limit a maximal achievable speed.

	# pm_vsi_u_maximal .2

Align the rotor.

	# pm_request_wave_hold

Now try to run.

	# pm_request_start
	# pm_i_set_point_Q 1

Next stage is to estimate BEMF constant more accurately. Do this when motor is
in run. Choice of different speed may give different results.

	# ap_identify_const_E

This completed the basic configuration.

