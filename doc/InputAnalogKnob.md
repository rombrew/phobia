## Overview

This page describes the configuration of analog knob interface.

## Wiring

The analog signal (from 0 to 5v) is fed to `ANG` pin. The brake signal (from 0
to 5v) is fed to `BRK` pin. Unconnected inputs are pulled to `GND` inside PMC.

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


## Configuration

First you need to figure out how the controller receives analog signals. Use
these registers to view analog values from knob inputs.

	(pmc) reg ap.knob_in_ANG
	(pmc) reg ap.knob_in_BRK

If signals do not change when you turn the knobs then check the wiring.

	               ^           // mapping //
	               |
	 control_ANG2  |<---------------------------------o
	               |                                / |
	               |                              /   |
	               |                            /     |
	               |                          /       |
	               |                        /         |
	               |                      /           |
	               |                    /             |
	               |                  /               |
	               |                /                 |
	 control_ANG1  |<--------------o                  |
	               |              /|                  |
	               |             / |                  |
	               |            /  |                  |
	               |           /   |                  |
	               |          /    |                  |
	               |         /     |                  |
	               |        /      |                  |
	               |       /       |                  |
	               |      /        |                  |
	 control_ANG0  |<----o         |                  |
	               |     |         |                  |      (V)
	               +-----+---------+------------------+--------------->
	                range_ANG0  range_ANG1        range_ANG2

Select the `ANG` signal range in which you want to work. We use three point
conversion from input voltage to the control value. Look at the diagram above.

	(pmc) reg ap.knob_range_ANG0 <V>
	(pmc) reg ap.knob_range_ANG1 <V>
	(pmc) reg ap.knob_range_ANG2 <V>

The same for `BRK` signal but we use two point conversion here.

	(pmc) reg ap.knob_range_BRK0 <V>
	(pmc) reg ap.knob_range_BRK1 <V>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the current control is selected as a
percentage of maximal current. There is a variable `pm.i_setpoint_current_pc`.

	(pmc) reg ap.knob_reg_ID <reg>

Note that setting the control variable does not enable appropriate control loop
automatically. You may need to enable appropriate control mode explicitly.

	(pmc) reg pm.config_LU_DRIVE 0

* 0 - Current loop.
* 1 - Speed loop.

Select the control variable range. So the input voltage range will be converted
to this control range.

	(pmc) reg ap.knob_control_ANG0 <x>
	(pmc) reg ap.knob_control_ANG1 <x>
	(pmc) reg ap.knob_control_ANG2 <x>

Specify control variable value in case of full brake. As the `BRK` signal rises
the control variable will be interpolated to this value.

	(pmc) reg ap.knob_control_BRK <x>

If you need you can change input lost range. If signal goes beyond this range
it is considered lost and output forced to `ap.knob_control_ANG0`.

	(pmc) reg ap.knob_range_LST0 <V>
	(pmc) reg ap.knob_range_LST1 <V>

Now enable machine startup control. Each time when `ANG` signal is in range the
startup is requested.

	(pmc) reg ap.knob_STARTUP 1

Now you are ready to enable the analog knob interface.

	(pmc) reg ap.knob_ENABLED 1

# Timeout shutdown

To stop the control we check if machine is run or setpoint is high. If setpoint
is out of input range and machine does not make full turns for
`ap.idle_timeout` seconds the shutdown is requested.

	(pmc) reg ap.idle_timeout <s>

# Disarm reset

To ensure a safe startup it is required to hold low `ANG` signal for
`ap.disarm_timeout` seconds until disarmed state was reset.

	(pmc) reg ap.disarm_timeout <s>

