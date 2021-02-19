## Overview

The PMC has many different settings to adjust the operation in different
conditions. We will consider the most important of them.

## Forced control

If the motor is at low speed then FLUX observer is unable to provide reliable
estimates. You need to use some sort of position sensor. If this is not
possible the forced control is used. We apply a current vector without feedback
to force rotor turn. You can adapt the current value and acceleration to your
needs.

	# reg pm.forced_hold_D <amp>
	# reg pm.forced_accel <rad/s/s>

For more reliable start increase hold current and decrease acceleration. Keep
in mind that hold current is applied constantly (like in stepper motor) so it
causes significant heating.

You have an option to disable forced control. The motor will be freewheeling at
low speed range.

	# reg pm.config_FORCED 0

## Current loop

You can limit phase current. It is the main tool not to burn the motor. This is
global restriction applicable to all closed loop modes of operation. You also
should set reverse limit for negative Q-axis current.

	# reg pm.i_maximal <amp>
	# reg pm.i_reverse <amp>

Thus, D-axis current will be limited to [**-pm.i_maximal** **+pm.i_maximal**]
but Q-axis will be limited to [**-pm.i_reverse** **+pm.i_maximal**].

Derated current in case of PCB overheat (look into **ap.heat_PCB**). Applicable
to both D and Q axes.

	# reg ap.heat_PCB_derated <amp>

Derated current in case of HFI operation. Applicable to both D and Q axes.

	# reg pm.i_derated_HFI <amp>

If you are interested in transient performance try to change slew rate. But
remember that low slew rate is useful for safe operation of the entire set of
constraints.

	# reg pm.i_slew_rate <amp/s>

You can limit consumption or regeneration battery current. Set the limit
according to the power supply capabilities.

	# reg pm.watt_iDC_maximal <amp>
	# reg pm.watt_iDC_reverse <amp>

Alternatively you can specify power limits. Note that the lowest of all
constraints is used.

	# reg pm.watt_wP_maximal <watt>
	# reg pm.watt_wP_reverse <watt>

You can limit DC link voltage at regenerative operation. This will derate
regeneration power in case of overvoltage.

	# reg pm.watt_dclink_HI <volt>

You can specify low limit of DC link voltage. This will prevent from source
overload.

	# reg pm.watt_dclink_LO <volt>

Also you can tune PI regulator gains. But usually this is not required as
default tune is good enough.

	# reg pm.i_gain_P <x>
	# reg pm.i_gain_I <x>

Default gains tune can be done by setting to zero.

	# reg pm.i_gain_P 0

## FLUX observer

We use simple FLUX observer that almost does not need to be configured
manually. However if it happened then say oh. You will probably need to change
the range of low speed region. There are BEMF values at which switching occurs
expressed relative to MPPE.

	# reg pm.flux_gain_TAKE_E <volt>
	# reg pm.flux_gain_GIVE_E <volt>

Also inspect the MPPE value itself. This is approximate peak-to-peak noise of
speed estimate.

	# reg pm.flux_MPPE

Carefully try to enlarge low pass filter gain to get more fast but noisy speed
estimate.

	# reg pm.flux_gain_SF <x>

Specify **IF** gain (from 0 to 1) as much as you want to add apriori speed
prediction that takes into account moment of inertia and current applied.

	# reg pm.flux_gain_IF <x>

You have an option to disable sensorless estimation (**EXPERIMENTAL**).

	# reg pm.config_ESTIMATE 0

## Speed loop

You can limit absolute value of speed in forward and reverse direction. Also
remember about alternative units of measure by using specific register name.

	# reg pm.s_maximal <rad/s>
	# reg pm.s_reverse <rad/s>

You can limit acceleration.

	# reg pm.s_accel <rad/s/s>

It should be noted that above restrictions are used differently depending on
**pm.config_DRIVE**. In case of speed control above restrictions are applied to
speed setpoint to get trackpoint **pm.s_track**. In other words we do not
limits actual parameters but limit input setpoint to comply it with
restrictions.

Quite different in the case of current control. You should specify
**pm.config_LIMITED** to apply above restrictions to actual speed and
acceleration if you need it of course. Here trackppoint is driven by actual
speed estimate with acceleration restriction. For system stability we have
introduced a linear control area **pm.s_linspan**. So there may be some
backlash e.g. in case of direction change.

Also you can tune P+FF regulator gains.

	# reg pm.s_gain_P <x>
	# reg pm.lu_gain_TF <x>

Default gains tune can be done by setting to zero.

	# reg pm.s_gain_P 0

## Brake function

If you need holding brake function in combination of current control then
enable this. It is activated when current setpoint is negative. Brake current
is limited by absolute value of setpoint, so brake is proportional.

	# reg pm.config_BRAKE 1

Note that speed control loop should be fine tuned to use this feature.

## Flux weakening

**TODO**

