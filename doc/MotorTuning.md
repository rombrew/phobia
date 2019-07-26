## Overview

The PMC has many different settings to adjust the operation in different
conditions. We will consider the most important of them.

## Forced control

If the motor is at low speed then FLUX observer is unable to provide reliable
estimates. You need to use HFI or some sort of position sensor. If this is not
possible the forced control is used. We apply a current vector without feedback
to force rotor turn. You can adapt the current value and acceleration to your
needs.

	# reg pm.forced_hold_D <amp>
	# reg pm.forced_accel_rpm <rpm/s>

For more reliable start increase hold current and decrease acceleration. Keep
in mind that hold current is applied constantly (as in stepper motor control)
so it causes significant heating.

You have an option to set hold current to 0. The motor will be freewheeling at
low speed range.

## FLUX observer

We try to make the observer does not need to be configured. However, sometimes
it is necessary to configure some parameters. You will probably need to change
the range of low speed region. This determines at what BEMF level the forced
control ends.

	# reg pm.lu_lock_S <volt>
	# reg pm.lu_unlock_S <volt>

Enlarge speed PLL gain to to get more fast and noisy speed estimate.

	# reg pm.flux_gain_SF <x>

Adjust stator resistance drift range if you know temperature range of the
motor.

	# reg pm.flux_lower_R <x>
	# reg pm.flux_upper_R <x>

Also if you do not need resistance adaptation you can decrease number of
hypotesis to 1. This will save computing resources.

	# reg pm.flux_N <N>

To get stable estimates at low speed we use D-axis current injection in
proportion from Q-axis current.

	# reg pm.inject_ratio_D <x>

## HF injection

The main parameters of the HFI is a frequency and swing. It is usually
sufficient that swing is greater than the noise of the current sensors. Large
injection swing will probably need to estimate flux polarity.

	# reg pm.hfi_freq_hz <hz>
	# reg pm.hfi_swing_D <amp>

In complicated cases you will need to tune observer gains.

	# reg pm.hfi_gain_EP <x>
	# reg pm.hfi_gain_SF <x>
	# reg pm.hfi_gain_FP <x>

## Current loop

You can limit DQ current value. It is the main tool not to burn the motor.

	# reg pm.i_maximal <amp>

Individual limit of brake current. This value should be lower than
**pm.i_maximal**.

	# reg pm.i_brake <amp>

You can limit consumption or regeneration DC-link current. Set the limit
according to the power supply capabilities.

	# reg pm.watt_ib_maximal <amp>
	# reg pm.watt_ib_reverse <amp>

Alternatively you can specify power limits. Note that the lowest of all
constraints is used.

	# reg pm.watt_wp_maximal <watt>
	# reg pm.watt_wp_reverse <watt>

You can limit DC link voltage at regenerative operation. This will derate
regeneration power in case of overvoltage.

	# reg pm.watt_derate_HI_U <volt>

You can specify low limit of DC link voltage. This will prevent source
overload.

	# reg pm.watt_derate_LO_U <volt>

Also you can tune PI regulator gains. But usually this is not required as
default tune is good enough.

	# reg pm.i_gain_P <x>
	# reg pm.i_gain_I <x>

## Speed loop

You can limit absolute value of speed.

	# reg pm.s_maximal_rpm <rpm>

You can limit acceleration.

	# reg pm.s_accel_rpm <rpm/s>

Also you can tune P+LP regulator gains.

	# reg pm.s_gain_P <x>
	# reg pm.s_gain_LP_I <x>

## Derating

There is a derate mechanism in case of overheating. You can control the
temperature of the PCB or the value from external sensor.

	# reg ap.temp_PCB
	# reg ap.temp_EXT

You can specify the maximal temperature above which derate occurs.

	# reg ap.heat_PCB <C>

When derate occurs the maximal DQ current is limited to the specified value.

	# reg ap.heat_PCB_derated <amp>

