## Overview

This page describes the configuration of RC servo pulse width input interface.

## Hardware

The pulse signal is fed to PPM pin that is 5v-tolerant.

	        +-----------------< +5v
	        |   +-------------< pulse signal
	        |   |       +-----< gnd
	        |   |       |
	+-------|---|---|---|--------------------------+
	|      +5v PPM DIR GND                         |

## Configuration

First you need to enable the appropriate mode of the PPM interface.

	# reg hal.PPM_mode 1

Now you can see how the controller receives the control signal. If variable
**hal.PPM_signal_caught** is 1 then pulse is caught. Use HAL commands to view
pulse parameters.

	# hal_PPM_get_PERIOD
	# hal_PPM_get_PULSE

Select the pulse width range in which you want to work. Write the range to a
PPM configuration. We use three point conversion from pulse width to the
control value.

	# reg ap.ppm_pulse_range_0 <us>
	# reg ap.ppm_pulse_range_1 <us>
	# reg ap.ppm_pulse_range_2 <us>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the speed control is selected as a percentage
of maximal no load speed. There is a variable **pm.s_setpoint_pc**.

	# reg ap.ppm_reg_ID <reg>

Select the control variable range. So the input pulse width range will be
converted to this control range.

	# reg ap.ppm_control_range_0 <x>
	# reg ap.ppm_control_range_1 <x>
	# reg ap.ppm_control_range_2 <x>

Now enable motor startup/shutdown control (based on PPM signal caught status).

	# reg ap.ppm_STARTUP 1

When you change the configuration we recommend to disconnect pulse signal for
safety.

## Precision

You can change the resolution of the timer that is used to measure the pulse
width. By increasing the resolution you increase a minimum pulse rate that
can be captured.

	# reg hal.PPM_timebase <hz>

The minimum frequency is determined from the expression. Default timebase
allows you to capture pulses from 31 Hz.

	        timebase
	Fmin = ----------
	         65536

Maximal timebase allowed is 84000000 Hz that gives a resolution about 12 ns
with 1282 Hz minimal pulse frequency.

