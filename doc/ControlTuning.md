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
it may causes significant heating.

You have an option to disable forced control completely. The machine will be
freewheeling at low speed range.

	(pmc) reg pm.config_LU_FORCED 0

Also you can allow zero freewheeling. This will mean that motor is only
freewheeling if current (or speed) setpoint is zero.

	(pmc) reg pmc.config_LU_FREEWHEEL 1

## Current control

We can automatically tune current loop PI regulator gains based on damping
percentage. Reasonable values are from 20 to 200 percent. You can change the
damping to find tradeoff between low noise and fast transient performance.

	(pmc) reg pm.i_damping <pc>

Phase current constraint is the main means not to burn the machine out. This is
global constraint applicable to all closed loop modes of operation. You also
can set reverse limit of negative Q current.

	(pmc) reg pm.i_maximal <A>
	(pmc) reg pm.i_reverse <A>

HFI current constraint is applicable in HFI mode of operation.

	(pmc) reg pm.hfi_maximal <A>

Weakening current constraint is applicable if `WEAKENING` is enabled.

	(pmc) reg pm.weak_maximal <A>

## VSI control

We have several types of zero sequence modulation in `VSI_ZERO` option.

`PM_VSI_GND` - Clamp to GND. Provides switching loss minimisation as one
terminal in always clamped to GND.

`PM_VSI_CENTER` - Clamp to middle. Provides minimal noise in current loop by
operating mostly in linear region.

`PM_VSI_EXTREME` - Clamp to GND or VCC depending on phase currents. Provides
even more switching loss minimisation as one terminal that conduct the largest
absolute current in always clamped to GND or VCC.

	(pmc) reg pm.config_VSI_ZERO 0

If you need to clamp output voltage along an inscribed circle and thus
eliminate overmodulation enable `VSI_CLAMP` option.

	(pmc) reg pm.config_VSI_CLAMP 1

## Wattage constraints

We can limit consumption and regeneration DC link current. Set the limits
according to your power supply capabilities.

	(pmc) reg pm.watt_wA_maximal <A>
	(pmc) reg pm.watt_wA_reverse <A>

Alternatively you can specify wattage limits. Note that the lowest of all
constraints will be used.

	(pmc) reg pm.watt_wP_maximal <W>
	(pmc) reg pm.watt_wP_reverse <W>

You can limit DC link voltage at regenerative operation. This will derate
regeneration power in case of overvoltage. This can be useful if you are using
machine as a generator.

	(pmc) reg pm.watt_uDC_maximal <V>

You can specify low limit of DC link voltage. This will prevent from supply
overload.

	(pmc) reg pm.watt_uDC_minimal <V>

## Sensorless estimate

We have two sensorless estimation algorithms in `ESTIMATE` option.

`PM_FLUX_NONE`   - No sensorless estimation.
`PM_FLUX_ORTEGA` - Robust `ORTEGA` observer with gain scheduling against speed.
`PM_FLUX_KALMAN` - Accurate `KALMAN` observer having convergence at HF injection.

`ORTEGA` nonlinear observer almost does not need to be configured manually. You
can carefully change speed loop gain to find tradeoff between transient rate
and noise level.

	(pmc) reg pm.flux_gain_SF <x>

`KALMAN` observer is conventional EKF with Q and R diagonal covariance
matrices. You can carefully change Q process noise to find tradeoff between
transient rate and noise level.

	(pmc) reg pm.kalman_gain_Q3 <x>

Also you have an option to completely disable sensorless estimation if you use
position sensors or forced control.

	(pmc) reg pm.config_ESTIMATE 0

## Speed control

We can automatically tune speed loop PID regulator gains based on damping
percentage. Reasonable values are from 40 to 400 percent. You can change the
damping to find tradeoff between low noise and fast transient performance.

	(pmc) reg pm.s_damping <pc>

You can limit absolute value of speed in forward and reverse direction. Also
remember about alternative units of measure by using specific register name.

	(pmc) reg pm.s_maximal <rad/s>
	(pmc) reg pm.s_reverse <rad/s>

You can limit the acceleration in forward and reverse direction. We suggest you
to increase the default acceleration values.

	(pmc) reg pm.s_accel_forward <rad/s2>
	(pmc) reg pm.s_accel_reverse <rad/s2>

Note that above constraints are used differently depending on selected control
loop. In case of speed control above constraints are applied to speed setpoint
to get trackpoint `pm.s_track`. In other words we do not limits actual speed
but limit the input setpoint to comply it with constraints.

Quite different in the case of current control. You should enable
`CC_SPEED_TRACK` feature to apply the above speed loop constraints to actual
speed and acceleration.

	(pmc) reg pm.config_CC_SPEED_TRACK 1

Here trackpoint is driven by actual speed estimate with acceleration
constraint. For system stability we have introduced a linear regulation region
`pm.l_track_tol` and blending gain `pm.l_gain_LP`. So there may be some
backlash in case of direction change.

## Brake function

If you need a brake function without a reverse in combination with current
control then enable `CC_BRAKE_STOP` feature. It is activated when current
setpoint is negative. Brake current is limited by absolute value of setpoint so
brake is proportional.

	(pmc) reg pm.config_CC_BRAKE_STOP 1

Note that speed control loop should be fine tuned to use this feature.

## Flux weakening

TODO

## MTPA control

TODO

