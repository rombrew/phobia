## Overview

This page describes the configuration of RC servo pulse width input
interface.

## Hardware

The signal is fed to PPM pin. It is 5v-tolerant. You can use +5v output and GND.

## Configuration

First you need to enable the appropriate mode of the PPM interface.

	# reg hal.PPM_mode 1

Now you can see how the controller receives the control signal. If variable
**hal.PPM_signal_caught** is 1 then pulse is captured. Use HAL commands to view
pulse parameters.

	# hal_PPM_get_PERIOD
	# hal_PPM_get_PULSE

Select the pulse width range in which you want to work. Put this range to a PPM
configuration.

	# reg ap.ppm_pulse_range[0] <minimal>
	# reg ap.ppm_pulse_range[1] <maximal>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the speed control is selected as a percentage
of maximal speed. There is a variable **pm.s_setpoint_pc**.

	# reg ap.ppm_reg_ID <reg>

Select the control variable range. So the input pulse width range will be
converted to this control range.

	# reg ap.ppm_control_range[0] <minimal>
	# reg ap.ppm_control_range[1] <maximal>

Also select the safe range which means an area in which the motor can start.
This is necessary to avoid an unexpected start. So to start the motor, you will
need to apply a control that motor is not dangerous (low speed), and only then
increase control signal.

	# reg ap.ppm_safe_range[0] <minimal>
	# reg ap.ppm_safe_range[1] <maximal>

When you change the configuration we recommend disconnect pulse signal for
safety.

## Precision

You can change the resolution of the timer that is used to measure the pulse
width. By increasing the resolution you increase a minimum pulse rate that
can be captured.

	# reg hal.PPM_timebase <hz>

The minimum frequency is determined from the expression.

	        timebase
	Fmin = ----------
	         65536

