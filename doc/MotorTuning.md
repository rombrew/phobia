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

## FLUX observer

We try to make the observer does not need to be configured. However, sometimes
it is necessary to configure some parameters. You will probably need to extend
the range of low speed region. This determines at what BEMF level the forced
control ends.

	# reg pm.flux_bemf_unlock <volt>
	# reg pm.flux_bemf_lock <volt>

You can change the observer gains but it is a complicated procedure.

	# reg pm.flux_gain

## Resistance adaptation

**TODO**

## HF injection

The main parameters of the HFI is a frequency and swing. It is usually
sufficient that swing is greater than the noise of the current sensors. Large
injection swing will probably need to estimate flux polarity.

	# reg pm.hfi_freq_hz <hz>
	# reg pm.hfi_swing_D <amp>

In complicated cases you will need to tune observer gains.

	# reg pm.hfi_gain

## Current loop

You can limit DQ current value. It is the main tool not to burn the motor.

	# reg pm.i_maximal <amp>

You can limit consumption or regeneration power. Set the limit according to the
power supply capabilities.

	# reg pm.watt_maximal <watt>
	# reg pm.watt_reverse <watt>

You can limit DC link voltage at regenerative operation. This will derate
regeneration power in case of overvoltage.

	# reg pm.lpfu_maximal <volt>

Also you can tune PI regulator gains. But usually this is not required as
default tune is good enough.

	# reg pm.i_gain

## Speed loop

You can limit absolute value of speed.

	# reg pm.s_maximal_rpm <rpm>

You can limit acceleration.

	# reg pm.s_accel_rpm <rpm/s>

Also you can tune PI regulator gains.

	# reg pm.s_gain

## Derating

There is a derate mechanism in case of overheating or battery overdischarge.
You can control the temperature of the PCB or the value from external sensor
(e.g. motor temperature).

	# reg ap.temp_PCB
	# reg ap.temp_EXT

You can specify the maximal temperature above which derate occurs.

	# reg ap.heat_PCB <C>

When derate occurs the maximal DQ current is limited to lower value than
**pm.i_maximal**.

	# reg ap.heat_PCB_derated <amp>

Similarly for battery overdischarge we set the low voltage threshold and power
consumption limit.

	# reg ap.batt_voltage_low <volt>
	# reg ap.batt_derated <watt>

