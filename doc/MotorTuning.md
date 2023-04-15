## Overview

The PMC has many different settings to adjust the operation in different
conditions. We will consider the most important of them.

## Forced control

If the motor is at low speed then FLUX observer is unable to provide reliable
estimates. You need to use some sort of position sensor. If this is not
possible the forced control is used. We apply a current vector without feedback
to force rotor turn. You can adapt the current value and acceleration to your
needs.

	(pmc) reg pm.forced_hold_D <A>
	(pmc) reg pm.forced_accel <rad/s2>

For more reliable start increase hold current and decrease acceleration. Keep
in mind that hold current is applied constantly (like in stepper motor) so it
causes significant heating.

You have an option to disable forced control. The motor will be freewheeling at
low speed range.

	(pmc) reg pm.config_LU_FORCED 0

## Current loop

You can limit phase current. It is the main tool not to burn the motor. This is
global constraint applicable to all closed loop modes of operation. You also
can set reverse limit of negative Q current.

	(pmc) reg pm.i_maximal <A>
	(pmc) reg pm.i_reverse <A>

Derated current in case of PCB overheat (look into other **ap.tpro** regs).
Applicable to both D and Q axes.

	(pmc) reg ap.tpro_derated_PCB <A>

If you are interested in transient performance try to change slew rate. But
remember that low slew rate is necessary for safe operation of wattage
constraints.

	(pmc) reg pm.i_slew_rate <A/s>

Also you can tune PI regulator gains. But usually this is not required as
default tune is good enough.

	(pmc) reg pm.i_gain_P <x>
	(pmc) reg pm.i_gain_I <x>

PI regulator gains can be reverted to default by setting P to 0.

	# reg pm.i_gain_P 0

## Wattage

You can limit consumption and regeneration battery current. Set the limits
according to the power supply capabilities.

	(pmc) reg pm.watt_wA_maximal <A>
	(pmc) reg pm.watt_wA_reverse <A>

Alternatively you can specify power limits. Note that the lowest of all
constraints will be used.

	(pmc) reg pm.watt_wP_maximal <W>
	(pmc) reg pm.watt_wP_reverse <W>

You can limit DC link voltage at regenerative operation. This will derate
regeneration power in case of overvoltage.

	(pmc) reg pm.watt_uDC_maximal <V>

You can specify low limit of DC link voltage. This will prevent from battery
overload.

	(pmc) reg pm.watt_uDC_minimal <V>

## FLUX estimate (ORTEGA)

We use ORTEGA flux observer that almost does not need to be configured
manually. You can carefully change speed loop gain to find tradeoff between
transient rate and noise level.

	(pmc) reg pm.flux_gain_SF <x>

You have an option to disable sensorless estimation if you use position
sensors.

	(pmc) reg pm.config_ESTIMATE 0

## FLUX estimate (KALMAN)

We use KALMAN flux observer that has conventional Q and R diagonal covariance
matrices. You can carefully change Q process noise to find tradeoff between
transient rate and noise level.

    (pmc) reg pm.kalman_gain_Q3 <x>

**TODO**

## Speed loop

You can limit absolute value of speed in forward and reverse direction. Also
remember about alternative units of measure by using specific register name.

	(pmc) reg pm.s_maximal <rad/s>
	(pmc) reg pm.s_reverse <rad/s>

You can limit the acceleration. We recommend to increase the default value.

	(pmc) reg pm.s_accel <rad/s/s>

It should be noted that above constraints are used differently depending on
selected control loop. In case of speed control above constraints are applied
to speed setpoint to get trackpoint **pm.s_track**. In other words we do not
limits actual parameters but limit input setpoint to comply it with
constraints.

Quite different in the case of current control. You should enable

    (pmc) reg pm.config_SPEED_LIMITED 1

to apply above constraints to actual speed and acceleration if you need it of
course. Here trackppoint is driven by actual speed estimate with acceleration
constraint. For system stability we have introduced a linear control area
**pm.s_linspan**. So there may be some backlash in case of direction change.

Also you can tune PI regulator gains.

    (pmc) reg pm.lu_gain_mq_LP <x>
	(pmc) reg pm.s_gain_P <x>
	(pmc) reg pm.s_gain_I <x>

PI regulator gains can be reverted to default by setting P to 0.

	(pmc) reg pm.s_gain_P 0

## Brake function

If you need a holding brake function in combination with current control then
enable this. It is activated when current setpoint is negative. Brake current
is limited by absolute value of setpoint so brake is proportional.

	(pmc) reg pm.config_HOLDING_BRAKE 1

Note that speed control loop should be fine tuned to use this feature.

## Flux weakening

TODO

## MTPA control

TODO
