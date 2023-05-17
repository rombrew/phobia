## Overview

This page describes how to identify the motor parameters by means of PMC. The
knowledge of parameters is necessary to control. Any new motor connected to the
PMC should be identified before run in closed control loop.

## Preparation

You may need to adjust some parameters in accordance with the capabilities of
the motor to not burn it. Default is acceptable for most RC motors but be
aware. These parameters are used to probe the motor.

	(pmc) reg pm.probe

There are a lot of parameters that can affect the motor identification. But we
believe that they will need a change only in a very complicated case.

Most likely you will need to change `pm.probe_speed_hold` parameter. The probe
speed should provide enough BEMF but not to exceed `pm.forced_maximal` speed.

Also pay attention to the forced control parameters which are used to achieve
initial spinup.

	(pmc) reg pm.forced

Most likely you will change `pm.forced_accel` and `pm.forced_hold_D` parameters
to get reliable startup.

Also do not forget to reset the motor parameters if you have previously run
with another motor.

    (pmc) pm_default_probe

## Sensors adjustment

To achieve a good result PMC has the ability to adjust voltage and current
sensors. The automatic self-adjustment is necessary to match the voltage
measurement channels. Also current sensors will be self-adjusted if motor is
connected.

	(pmc) pm_self_adjust

This is enough to do it once and save the values in the flash. But we recommend
to do it again when you significantly change DC link voltage.

## Number of the rotor pole pairs

This number relates the electrical speed with the mechanical. We believe you
are able to identify `Zp` simply by counting the number of magnets on the rotor
then divide it by 2. This is the most famous method.

	(pmc) reg pm.const_Zp <N>

If access to the motor is restricted and to count the magnets is impossible
then just leave a value 1. Instead of mechanical speed you will see electrical
speed. By measuring the mechanical speed directly you can estimate `Zp` and set
it later.

## Impedance of stator windings

We measure the resistance `pm.const_Rs` by difference of voltage drop on two
values of holding current. For more accuracy you need to increase the probe
current or reduce DC link voltage.

Then we use a high frequency sinusoidal signal to measure the full impedance
and calculate DQ inductances `L1` and `L2`.

	(pmc) pm_probe_base

## Rotor flux linkage

This parameter also known as `Kv` rating. Internal representation is `lambda`
that linked with `Kv` by following equation.

	                      60
	lambda = ----------------------------
	          2 * PI * sqrt(3) * Kv * Zp

To identify `lambda` you have to run the motor. Also the rotor should rotate at
significant speed. We do a forced initial spinup to reach this condition.

	(pmc) pm_probe_spinup

To get more accurate estimate you can run the motor at high speed and request
lambda probe manually. Do not load the motor.

	(pmc) pm_fsm_startup
	(pmc) reg pm.s_setpoint_rpm <rpm>
	(pmc) pm_probe_const_flux_linkage

## No forced spinup

If you failed to start the motor with `pm_probe_spinup` you have an option to
identify flux linkage in detached mode. You will have to rotate the motor
manually in this case.

	(pmc) pm_probe_detached

PMC will wait for the motor to reach at least `pm.probe_speed_detached`
speed.

## Speed estimation noise

After flux linkage we estimate speed noise to know the lower bound of flux
observer operation. As a result these threshold values are calculated.

	# reg pm.zone_speed_noise
	# reg pm.zone_speed_threshold

## Moment of inertia

Final estimate is moment of inertia `pm.const_Ja`. To do this possible a speed
maneuver is performed. Note that this may result energy regeneration so your
power supply must tolerate this. Either limit maximal regeneration.

This constant is used to tune speed control loop. Also it is used in operation
to predict the speed changes from an applied current.

