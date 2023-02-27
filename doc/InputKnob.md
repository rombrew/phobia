## Overview

This page describes the configuration of analog knob interface.

## Wiring

The analog signal (from 0 to 5v) is fed to ANG pin. The brake signal (from 0 to
5v) is fed to BRK pin. Unconnected inputs are pulled to GND internally.

	    +-------------------+
	    |                   |
	   | |   (analog)       |
	   | |<-------------+   |
	   |_|              |   |
	    |           +---|---|-------~
	    +-------+   |   |   |
	            |   |   |   |
	            |   |   |   |
	+-----------+---+---+---+--------------------+
	|          GND BRK ANG +5v                   |


## Configuration

First you need to figure out how the controller receives analog signals. Use
HAL registers to view analog values.

	# reg hal.ADC_get_knob_ANG
	# reg hal.ADC_get_knob_BRK

If signals do not change when you turn the knobs then check the wiring.

	               ^    // mapping of input voltage to the control variable //
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
	               |     |         |                  |      (volt)
	               +-----+---------+------------------+--------------->
	                  in_ANG0   in_ANG1            in_ANG2

Select the ANG signal range in which you want to work. We use three point
conversion from input voltage to the control value. Look at the diagram above.

	# reg ap.knob_in_ANG0 <volt>
	# reg ap.knob_in_ANG1 <volt>
	# reg ap.knob_in_ANG2 <volt>

The same for BRK signal but we use two point conversion here.

	# reg ap.knob_in_BRK0 <volt>
	# reg ap.knob_in_BRK1 <volt>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the current control is selected as a
percentage of maximal current. There is a variable **pm.i_setpoint_torque_pc**.

	# reg ap.knob_reg_ID <reg>

Note that setting the control variable does not enable appropriate control loop
automatically. You may need to enable appropriate control mode explicitly.

	# reg pm.config_LU_DRIVE <x>

Select the control variable range. So the input voltage range will be converted
to this control range.

	# reg ap.knob_control_ANG0 <x>
	# reg ap.knob_control_ANG1 <x>
	# reg ap.knob_control_ANG2 <x>

Specify control variable value in case of full brake. As the BRK signal rises
the control variable will be interpolated to this value.

	# reg ap.knob_control_BRK <x>

If you need you can change input lost range. If signal goes beyond this range
it is considered lost and output forced to **ap.knob_control_ANG_0**.

	# reg ap.knob_in_lost0 <volt>
	# reg ap.knob_in_lost1 <volt>

Now enable motor startup/shutdown control. Each time when ANG signal is in
range the startup is requested.

	# reg ap.knob_STARTUP 1

Now you are ready to enable the analog input interface.

	# reg ap.knob_ENABLED 1

# Timeout shutdown

To stop the control we check if motor is run or setpoint is high. If setpoint
is out of input range and motor does not make full turns for **ap.idle_TIME_s**
seconds the shutdown is requested.

	# reg ap.idle_TIME_s <s>

