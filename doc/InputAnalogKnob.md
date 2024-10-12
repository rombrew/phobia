## Overview

This page describes the configuration of analog knob interface.

## Wiring

The analog signal (from 0 to 5v) is fed to `ANG` pin. The brake signal (from 0
to 5v) is fed to `BRK` pin. Unconnected inputs are pulled to `GND` inside PMC.

```
	    +-------------------+
	    |                   |
	   | |   (analog)       |
	   | |<-------------+   |
	   |_|              |   |
	    |           +---|---|-------~ (brake)
	    +-------+   |   |   |
	            |   |   |   |
	            |   |   |   |
	+-----------+---+---+---+--------------------+
	|          GND BRK ANG +5v                   |
```

**WARNING**: Refer to your hardware manual or look into `src/hal/hw/...`
directory to find out actual pin mapping and voltage levels of your port.

## Configuration

First you need to figure out how the controller receives analog signals. Use
these registers to view analog voltages on interface inputs.

	(pmc) reg ap.knob_in_ANG
	(pmc) reg ap.knob_in_BRK

If signals do not change when you turn the knobs then check the wiring.

Select the `ANG` signal range in which you want to work. We use three point
conversion from input voltage to the control value.

	(pmc) reg ap.knob_range_ANG0 <V>
	(pmc) reg ap.knob_range_ANG1 <V>
	(pmc) reg ap.knob_range_ANG2 <V>

The same for `BRK` signal but we use two point conversion here.

	(pmc) reg ap.knob_range_BRK0 <V>
	(pmc) reg ap.knob_range_BRK1 <V>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the `pm.s_setpoint_speed_knob` register is
selected that mapped on current or speed percentage depending on control loop
enabled.

	(pmc) reg ap.knob_reg_ID <reg>

Note that setting the control variable does not enable appropriate control loop
automatically. You can select control loop if you need.

	(pmc) reg pm.config_LU_DRIVE 0

Select the control variable range. So the input voltage range will be converted
to this control range.

	(pmc) reg ap.knob_control_ANG0 <x>
	(pmc) reg ap.knob_control_ANG1 <x>
	(pmc) reg ap.knob_control_ANG2 <x>

Enable machine startup control. Each time when `ANG` signal is in range the
startup is requested.

	(pmc) reg ap.knob_STARTUP 1

Enable brake signal usage if you need it.

	(pmc) reg ap.knob_BRAKE 1

Specify control variable value in case of full brake. As the `BRK` signal rises
the control variable will be interpolated to this value.

	(pmc) reg ap.knob_control_BRK <x>

If you need you can change input lost range. If analog signal goes beyond this
range it is considered lost and error reason `PM_ERROR_KNOB_SIGNAL_FAULT` is reported.

	(pmc) reg ap.knob_range_LOS0 <V>
	(pmc) reg ap.knob_range_LOS1 <V>

Now you are ready to enable the analog knob interface.

	(pmc) reg ap.knob_ENABLED 1

# Shutdown timeout

To stop the control we check if machine is still run or setpoint is high. If
setpoint is out of input range and machine does not make full turns for
`ap.timeout_IDLE` milliseconds the shutdown is requested.

	(pmc) reg ap.timeout_IDLE <ms>

# Disarm timeout

To ensure a safe startup it is required to hold low `ANG` signal for
`ap.timeout_DISARM` milliseconds until disarmed state was reset.

	(pmc) reg ap.timeout_DISARM <ms>

