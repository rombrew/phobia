## Overview

This page describes the configuration of analog input interface.

## Hardware

The analog signal (from 0 to 5v) is fed to ANG pin. The brake signal (from 0 to
5v) is fed to BRK pin. Unconnected inputs are pulled to GND internally.

	    +-------------------+
	    |                   |
	   | |   (analog)       |
	   | |<-------------+   |
	   |_|              |   |
	    |               |   |
	    +-------+       |   |
	            |       |   |
	            |       |   |
	+-----------|---|---|---|--------------------+
	|          GND BRK ANG +5v                   |


## Configuration

First you need to figure out how the controller receives analog signals. Use
HAL commands to view analog values.

	# hal_ADC_get_analog_ANG
	# hal_ADC_get_analog_BRK

If signals do not change when you turn the knobs then check the wiring.

	               ^    // mapping of input voltage to control variable //
	               |
	 control_ANG_2 |<---------------------------------o
	               |                                / |
	               |                              /   |
	               |                            /     |
	               |                          /       |
	               |                        /         |
	               |                      /           |
	               |                    /             |
	               |                  /               |
	               |                /                 |
	 control_ANG_1 |<--------------o                  |
	               |              /|                  |
	               |             / |                  |
	               |            /  |                  |
	               |           /   |                  |
	               |          /    |                  |
	               |         /     |                  |
	               |        /      |                  |
	               |       /       |                  |
	               |      /        |                  |
	 control_ANG_0 |<----o         |                  |
	               |     |         |                  |      (volt)
	               +-----+---------+------------------+--------------->
	                  in_ANG_0  in_ANG_1           in_ANG_2

Select the ANG signal range in which you want to work. We use three point
conversion from input voltage to the control value. Look at the diagram above.

	# reg ap.analog_in_ANG_0 <volt>
	# reg ap.analog_in_ANG_1 <volt>
	# reg ap.analog_in_ANG_2 <volt>

The same for BRK signal but we use two point conversion here.

	# reg ap.analog_in_BRK_0 <volt>
	# reg ap.analog_in_BRK_1 <volt>

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the current control is selected as a
percentage of maximal current. There is a variable **pm.i_setpoint_torque_pc**.

	# reg ap.analog_reg_ID <reg>

Select the control variable range. So the input voltage range will be converted
to this control range.

	# reg ap.analog_control_ANG_0 <x>
	# reg ap.analog_control_ANG_1 <x>
	# reg ap.analog_control_ANG_2 <x>

Specify control variable value in case of full brake. As the BRK signal rises
the control variable will be interpolated to this value.

	# reg ap.analog_control_BRK <x>

If you need you can change input lost range. If signal goes beyond this range
it is considered lost and output equals to **ap.analog_control_ANG_0**.

	# reg ap.analog_in_lost_0 <volt>
	# reg ap.analog_in_lost_1 <volt>

Now enable motor startup/shutdown control. Each time when ANG signal is in
range the startup is requested.

	# reg ap.analog_STARTUP 1

Now you are ready to enable the analog input interface.

	# reg ap.analog_ENABLED 1

# Timeout configuration

To stop the control we check if motor is run or phase current is high. If phase
current is less than **ap.timeout_current_tol** and motor does not make full
turns for **ap.timeout_TIME** seconds the shutdown is requested.

	# reg ap.timeout_TIME <s>

