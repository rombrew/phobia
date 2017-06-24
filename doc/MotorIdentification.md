## Overview

This page describes how to identify motor parameters by means of PMC. The
knowledge of parameters is necessary to control. Any new motor connected to the
PMC should be identified before run in closed control loop.

## Preparation

First of all reset the configuration. However, you may be not need this to do
if you connect a new motor with similar parameters.

	# pm_default

You may need to adjust some parameters in accordance with the capabilities of
the motor. Default is acceptable for most RC motors but be aware. These
currents are used to probe the motor.

	# pm_pb_i_hold <amp>
	# pm_pb_i_sine <amp>

Maximal current in closed loop for low and high speed regions. Keep it low in
first run, avoid iron saturation.

	# pm_i_low_maximal <amp>
	# pm_i_high_maximal <amp>

There are a lot of parameters that can affect the motor identification. But we
believe that they will need a change only in a very complicated case.

Then it would be nice to calibrate the current sensors. This can be done with
an external standard or to align the sensors among themselves.

	# ap_calibrate

If everything is ok then you can connect the motor to the terminals. Remove any
load from motor to allow it rotate freely.

## Number of the rotor pole pairs

This number connects the electrical speed with the mechanical. We believe you
are able to identify Zp simply by counting the number of magnets on the rotor
then divide it by 2. This is the most famous method.

If access to the motor is restricted and to count the magnets is impossible
there is another way. We can turn the rotor in open loop by applying a slowly
rotating current vector. Trying a different number of electrical turns that
gives a single mechanical turn we can guess Zp value. The following command
does the rotation for specified number of electrical turns.

	# ap_blind_turn <n> <hz>

Where <n> means the number of electrical turns, optional <hz> specifies the
frequency in Hz. After that you should manually set Zp value.

	# pm_const_Zp <n>

## Impedance of stator windings

Resistance is measured by holding a constant current vector for some time.
Assuming that the rotor was fixed we can determine R as U/I. For extra accuracy
we do this procedure twice.

We use a high frequency signal to measure the full impedance then we calculate
the inductance. For extra accuracy we also do this procedure twice.

This command will estimate zero drift of current sensors then measure R, Ld,
Lq. At the end it will update PI gains and slew rate of current loop.

	# ap_identify_base

Normally it will print the values of the identified parameters. In case of
failure the error number will be printed out.

## Resistance imbalance

We can estimate resistance of individual phase windings by sequentially
applying three different current vectors.

	# ap_identify_const_R_abc

For extra accuracy you may need to increase the current of probing. But beware
that high current will heat the winding that will lead to resistance thermal
drift.

	# pm_pb_i_hold <amp>

Currently these parameters are not saved or used anywhere. You only can take a
look at them and make any conclusions.

## Rotor flux linkage

This parameter also known as Kv rating. Internal representation is E that
linked with Kv by following equation.

	                 60
	E = -----------------------------
	     2 * PI * sqrt(3.) * Kv * Zp

First we recommend you to set an initial guess of E to facilitate the job of
state observer.

	# pm_const_E_kv <rpm/v>

To identify E you have to run in closed loop. Also the rotor should rotate at
significant speed. We do a blind spinup to reach this condition.

	# pm_pb_speed_low_rpm <rpm>
	# ap_blind_spinup

Then you need to make the identification. It measures the voltage discrepancy
in Q axis divided by the speed. The resulting value is used to update E.

	# ap_identify_const_E

To get more accuracy repeat the procedure at higher speed but try to keep the
current low, do not load the motor.

## Moment of inertia

This is mechanical parameter depends on the motor and connected load. It could
be estimated but we do not use it in control. It affects the choice of speed
loop gains if you want to improve control performance.

You may need to adjust two speed values we going to step between which. They
should be much different.

	# pm_pb_speed_low_rpm <rpm>
	# pm_pb_speed_high_rpm <rpm>

To begin you have to run in closed loop with enabled only speed control.

	# ap_blind_spinup
	# pm_m_bitmask_forced_control 0

Do the job.

	# ap_identify_const_J

## Automated probe

All of main parameters could be measures by single call that sequentially
executes the previously discussed commands.

	# ap_probe_base

This will estimate the following.

* Zero drift & impedance (R, Ld, Lq)
* Rotor flux linkage (E)
* Moment of inertia (J)

You still need to specify Zp and initial guess of E before to do this.

## Thermal drift

## BEMF waveform

