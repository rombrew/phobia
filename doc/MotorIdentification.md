## Overview

This page describes how to identify motor parameters by means of PMC. The
knowledge of parameters is necessary to control. Any new motor connected to the
PMC should be identified before run in closed control loop.

## Preparation

You may need to adjust some parameters in accordance with the capabilities of
the motor to not burn it. Default is acceptable for most RC motors but be
aware. These parameters are used to probe the motor.

	# reg pm.probe

There are a lot of parameters that can affect the motor identification. But we
believe that they will need a change only in a very complicated case.

## Sensors adjustment

For a good result adjust the voltage and current sensors. To adjust the
current sensors you need to connect at least some motor.

	# pm_self_adjust

This is enough to do it once and save the values in the flash.

## Number of the rotor pole pairs

This number connects the electrical speed with the mechanical. We believe you
are able to identify Zp simply by counting the number of magnets on the rotor
then divide it by 2. This is the most famous method.

	# reg pm.const_Zp <n>

If access to the motor is restricted and to count the magnets is impossible
then just leave a value 1. Instead of mechanical speed you will see electrical
speed. By measuring the mechanical speed directly you can estimate Zp and set
it later.

## Impedance of stator windings

Resistance is measured by holding a constant current vector for some time.
Assuming that the rotor was fixed we can determine R as U/I. For greater
accuracy you need to increase the probe current and reduce the supply
voltage.

We use a high frequency sinusoidal signal to measure the full impedance then we
calculate DQ inductance and rotation angle.

	# pm_probe_base

Normally it will print the values of the identified parameters. In case of
failure the error will be printed out.

## Rotor flux linkage

This parameter also known as Kv rating. Internal representation is E that
linked with Kv by following equation.

	                 60
	E = -----------------------------
	     2 * PI * sqrt(3.) * Kv * Zp

To identify E you have to run in closed loop. Also the rotor should rotate at
significant speed. We do a forced spinup to reach this condition.

	# pm_probe_spinup

To get more accuracy increase the speed and request E probe again. Do not load
the motor.

	# reg pm.s_setpoint_pc <\%>
	# reg pm.fsm_state 10

## Moment of inertia

**TODO**

