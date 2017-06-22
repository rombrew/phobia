## Overview

This page introduces you to the Command Line Interface (CLI).

## Basics

The CLI is similar to the others. You could type command name manually or use
autocomplete. To call one of previous command there is a history. Most often
the command prints out the value of parameter if it is called without any
argument. If argument is specified then value of parameter will be modified.
More complex commands can block the shell until the job is done.

## HAL

Basic informational commands.

	# hal_uptime
	# hal_cpu_usage

Configuration of the Pulse Width Modulation (PWM).

	# hal_pwm_freq_hz <hz>
	# hal_pwm_dead_time_ns <ns>

Set the duty cycle of PWM. Values from 0 to the PWM resolution. For diagnostic
purposes.

	# hal_pwm_DC <a> <b> <c>

Enable Z state of the bridge. Bit field value from 0 to 7. For diagnostic
purposes.

	# hal_pwm_Z <x>

Show thermal information from NTC and internal MCU sensor.

	# hal_thermal



