## Overview

The PMC has many different settings to adjust the operation in different
conditions. We will consider the most important of them.

## Forced control

If the machine is at low speed then sensorless flux observer is unable to
provide reliable estimates. You need to use some sort of position sensor. If
this is not possible the forced control is used. We apply a current vector
without feedback to force rotor turn. You can adapt the current value and
acceleration to your needs.

	(pmc) reg pm.forced_hold_D <A>
	(pmc) reg pm.forced_accel <rad/s2>

To get more reliable start increase hold current and decrease acceleration.
Keep in mind that hold current is applied constantly (like in stepper motor) so
it causes significant heating.

You have an option to disable forced control. The machine will be freewheeling
at low speed range.

	(pmc) reg pm.config_LU_FORCED 0

## Current loop

You can automatically tune current loop PI regulator gains based on damping
percentage. Reasonable values are from 20 to 200.

	(pmc) reg pm.i_damping <pc>

Phase current constraint is the main tool not to burn the machine. This is
global constraint applicable to all closed loop modes of operation. You also
can set reverse limit of negative Q current.

	(pmc) reg pm.i_maximal <A>
	(pmc) reg pm.i_reverse <A>

Derated current constraint in case of PCB overheat (also look into other
`ap.heat` regs). Applicable to both D and Q axes.

	(pmc) reg ap.heat_derated_PCB <A>

## Wattage

You can limit consumption and regeneration DC link current. Set the limits
according to the power supply capabilities.

	(pmc) reg pm.watt_wA_maximal <A>
	(pmc) reg pm.watt_wA_reverse <A>

Alternatively you can specify wattage limits. Note that the lowest of all
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

Also you have an option to completely disable sensorless estimation if you use
position sensors.

	(pmc) reg pm.config_ESTIMATE 0

## FLUX estimate (KALMAN)

We use KALMAN flux observer that has conventional Q and R diagonal covariance
matrices. You can carefully change Q process noise to find tradeoff between
transient rate and noise level.

    (pmc) reg pm.kalman_gain_Q3 <x>

## Speed loop

You can automatically tune speed loop PID regulator gains based on damping
percentage. Reasonable values are from 40 to 400.

	(pmc) reg pm.s_damping <pc>

You can limit absolute value of speed in forward and reverse direction. Also
remember about alternative units of measure by using specific register name.

	(pmc) reg pm.s_maximal <rad/s>
	(pmc) reg pm.s_reverse <rad/s>

You can limit the acceleration in forward and reverse direction. We suggest you
to increase the default acceleration value.

	(pmc) reg pm.s_accel_forward <rad/s2>
	(pmc) reg pm.s_accel_reverse <rad/s2>

It should be noted that above constraints are used differently depending on
selected control loop. In case of speed control above constraints are applied
to speed setpoint to get trackpoint `pm.s_track`. In other words we do not
limits actual parameters but limit the input setpoint to comply it with
constraints.

Quite different in the case of current control. You should enable
`CC_SPEED_TRACK` feature to apply above speed loop constraints to actual speed
and acceleration.

    (pmc) reg pm.config_CC_SPEED_TRACK 1

Here trackpoint is driven by actual speed estimate with acceleration
constraint. For system stability we have introduced a linear regulation region
`pm.l_track_tol` and blending gain `pm.l_gain_LP`. So there may be some
backlash in case of direction change.

Also you can tune PID regulator and load torque gains manually.

    (pmc) reg pm.lu_gain_mq_LP <x>
	(pmc) reg pm.s_gain_P <x>
	(pmc) reg pm.s_gain_I <x>
	(pmc) reg pm.s_gain_D <x>

## Brake function

If you need a brake function without a reverse in combination with current
control then enable `CC_BRAKE` feature. It is activated when current setpoint
is negative. Brake current is limited by absolute value of setpoint so brake is
proportional.

	(pmc) reg pm.config_CC_BRAKE 1

Note that speed control loop should be fine tuned to use this feature.

## Flux weakening

TODO

## MTPA control

TODO
