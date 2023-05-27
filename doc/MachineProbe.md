## Overview

This page describes how to identify the machine parameters by means of PMC. The
knowledge of parameters is necessary to control. Any new machine connected to
the PMC should be identified before run in closed control loop.

## Preparation

You may need to adjust some parameters in accordance with the capabilities of
the machine to not burn it. Default is acceptable for most RC motors but be
aware. These parameters are used to probe the machine.

	(pmc) reg pm.probe

There are a lot of parameters that can affect the machine identification. But
we believe that they will need a change only in a complicated case. Most likely
you will need to decrease probe currents for a small machine.

* `pm.probe_current_hold`, `pm.probe_current_weak` - Two current setpoints that
  is used to estimate stator resistance. They must be significantly different.
* `pm.probe_freq_sine`, `pm.probe_current_sine` - Sine wave frequency and
  amplitude that is used to estimate stator impedance.
* `pm.probe_speed_hold` - Speed setpoint for the initial spinup. At this speed
  flux linkage and noise threshold will be estimated.

Also pay attention to the forced control parameters which are used to achieve
initial spinup.

	(pmc) reg pm.forced

* `pm.forced_hold_D` - Current setpoint which should be enough to hold rotor in
  aligned position and force it turn.
* `pm.forced_accel` - Allowed acceleration of the forced control.

If you use power supply that not tolerate reverse current then consider the
wattage limit settings.

    (pmc) reg pm.watt

* `pm.watt_wA_reverse` - Maximal reverse current on DC link.
* `pm.watt_uDC_maximal` - Maximal overvoltage on DC link.

Also do not forget to reset the machine parameters if you have previously run
with another machine.

    (pmc) pm_default_probe

## Sensors adjustment

To achieve the best accuracy PMC has the ability of self-adjust voltage and
current onboard sensors. The automatic self-adjustment is necessary to match
the voltage measurement channels. Also current sensors will be self-adjusted if
machine is connected.

	(pmc) pm_self_adjust

This is enough to do it once and save the values in the flash. But we recommend
to do it again after you radically change DC link voltage.

## Number of the rotor pole pairs

This number relates the electrical speed with the mechanical. We believe you
are able to identify `Zp` simply by counting the number of magnets on the rotor
then divide it by 2. This is the most famous method.

	(pmc) reg pm.const_Zp <n>

If access to the machine is restricted and to count the magnets is impossible
then just leave a value 1. Instead of mechanical speed you will see electrical
speed. By measuring the mechanical speed directly you can estimate `Zp` and set
it later.

## Impedance of stator windings

We measure the resistance `pm.const_Rs` by difference of voltage drop on two
values of holding current. For more accuracy you need to increase the probe
current or reduce DC link voltage.

Then we use a high frequency sinusoidal signal to measure the full impedance
and calculate DQ inductances `pm.const_im_L1` and `pm.const_im_L2`.

	(pmc) pm_probe_base

If the procedure fails with an error sure that probe currents are suitable for
you machine.

## Rotor flux linkage

This parameter also known as `Kv` rating. Internal representation is
`pm.const_lambda` that linked with `Kv` by following equation.

	                      60
	lambda = ----------------------------
	          2 * PI * sqrt(3) * Kv * Zp

To identify `lambda` you have to run the machine. Also the rotor should rotate
at significant speed. We do a forced initial spinup to reach this condition.

	(pmc) pm_probe_spinup

If the procedure fails to spinup the machine try to adjust forced control
parameters.

To get more accurate flux linkage estimate you can run the machine at high
speed and request lambda probe manually. Do not load the machine.

	(pmc) pm_fsm_startup
	(pmc) reg pm.s_setpoint_rpm <rpm>
	(pmc) pm_probe_const_flux_linkage

## No forced spinup

If you failed to start the machine with `pm_probe_spinup` you have an option to
identify flux linkage in detached mode. You will have to rotate the machine
manually in this case.

	(pmc) pm_probe_detached

PMC will wait for the machine to reach at least `pm.zone_speed_threshold`
speed.

## Speed noise threshold

After flux linkage we estimate speed noise to know the lower bound of flux
observer operation. As a result these threshold values are calculated.

	# reg pm.zone_speed_noise
	# reg pm.zone_speed_threshold

## Moment of inertia

Final estimate is a moment of inertia `pm.const_Ja`. To do this possible a
speed maneuver will be performed. Note that this may result energy regeneration
so your power supply must tolerate this. Either limit maximal regeneration.

This constant is used to tune speed control loop. Also it is used in operation
to predict the speed changes from an applied current.

