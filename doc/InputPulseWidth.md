## Overview

This page describes the configuration of RC servo pulse width input interface.

## Wiring

The pulse signal is fed to PPM pin that is 5v-tolerant.

	        +-----------------> +5v
	        |   +-------------< (pulse)
	        |   |       +-----> GND
	        |   |       |
	+-------+---+---+---+--------------------------+
	|      +5v PPM DIR GND                         |

## Configuration

First you need to enable the appropriate mode of the PPM interface.

	(pmc) reg hal.PPM_mode 1

Now you can see how the controller receives the control signal. If variable
`hal.PPM_caught` is nonzero then pulse is caught. Use these registers to view
pulse parameters.

	(pmc) reg ap.ppm_in_pulse
	(pmc) reg ap.ppm_in_freq

Select the pulse width range in which you want to operate. Write the range to a
PPM configuration or leave defaults. We use three point conversion from pulse
width to the control value.

	(pmc) reg ap.ppm_range_pulse0 <ms>
	(pmc) reg ap.ppm_range_pulse1 <ms>
	(pmc) reg ap.ppm_range_pulse2 <ms>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the `pm.s_setpoint_speed_knob` register is
selected that mapped on current or speed percentage depending on control loop
enabled.

	(pmc) reg ap.ppm_reg_ID <reg>

Note that setting the control variable does not enable appropriate control loop
automatically. You can select control loop if you need.

	(pmc) reg pm.config_LU_DRIVE 1

Select the control variable range. So the input pulse width range will be
converted to this control range.

	(pmc) reg ap.ppm_range_control0 <x>
	(pmc) reg ap.ppm_range_control1 <x>
	(pmc) reg ap.ppm_range_control2 <x>

Now enable the machine startup control. The condition to start is the transition
of pulse signal to the operating range from the low side.

	(pmc) reg ap.ppm_STARTUP 1

When you change the configuration we recommend to disconnect pulse signal for
safety.

## Precision

You can change the timebase frequency of the timer that is used to measure the
pulse width. By increasing the resolution you increase a minimum pulse rate
that can be captured.

	(pmc) reg hal.PPM_frequency <Hz>

The minimum pulse frequency is determined from the expression. Default timebase
allows you to capture pulses from 31 Hz.

	       frequency
	min = -----------
	         65536

Maximal timebase allowed is 84000000 Hz that gives a resolution about 12 ns
with 1282 Hz minimal pulse frequency.

# Disarm reset

To ensure a safe startup it is required to hold low pulse signal for
`ap.disarm_timeout` seconds until disarmed state was reset.

	(pmc) reg ap.disarm_timeout <s>

