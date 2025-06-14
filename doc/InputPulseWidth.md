## Overview

This page describes the configuration of RC servo pulse width input interface.

## Wiring

The pulse signal is fed to PPM pin that must be 5v-tolerant.

```
	        +-----------------> +5v
	        |   +-------------< (pulse)
	        |   |   +---------> GND
	        |   |   |
	+-------+---+---+-----------------------------+
	|      +5v PPM GND                            |
```

**WARNING**: Refer to your hardware manual or look into `src/hal/hw/...`
directory to find out actual pin mapping and voltage levels of your port.

## Configuration

First you need to enable the appropriate mode of the PPM interface in HAL.

	(pmc) reg hal.PPM_mode 1

Now you can see how the controller receives the control signal. If variable
`ap.ppm_FREQ` is nonzero then pulse is caught. Use these registers to view
pulse parameters.

	(pmc) reg ap.ppm_PULSE
	(pmc) reg ap.ppm_FREQ

Select the pulse width range in which you want to operate. Write the range to a
PPM configuration or leave defaults. We use three point conversion from pulse
width to the control value.

	(pmc) reg ap.ppm_range0 <ms>
	(pmc) reg ap.ppm_range1 <ms>
	(pmc) reg ap.ppm_range2 <ms>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the `pm.s_setpoint_speed_knob` register is
selected that mapped on current or speed percentage depending on control loop
enabled.

	(pmc) reg ap.ppm_reg_ID <reg>

Note that setting the control variable does not enable appropriate control loop
automatically. You can select control loop if you need.

	(pmc) reg pm.config_LU_DRIVE 2

Select the control variable range. So the input pulse width range will be
converted to this control range.

	(pmc) reg ap.ppm_control0 <x>
	(pmc) reg ap.ppm_control1 <x>
	(pmc) reg ap.ppm_control2 <x>

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

# Disarm timeout

To ensure a safe startup it is required to hold low pulse signal for
`ap.timeout_DISARM` milliseconds until disarmed state was reset.

	(pmc) reg ap.timeout_DISARM <ms>

