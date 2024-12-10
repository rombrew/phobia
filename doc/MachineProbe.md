## Overview

This page describes how to identify the machine parameters by means of PMC. The
knowledge of parameters is necessary to control.

**WARNING**: Any new machine connected to the PMC should be identified before
run in closed control loop.

## Preparation

You may need to adjust some parameters in accordance with the capabilities of
the machine to not burn it. Default is acceptable for most large RC motors but
be aware. These parameters are used to probe the machine.

	(pmc) reg pm.probe

There are a lot of parameters that can affect the machine identification. But
we believe that they will need a change only in a complicated case. Most likely
you will need to decrease probing currents for a small machine.

- `pm.probe_current_hold` - Machine probing current that is used to estimate
  stator resistance. Note it must be large enough to get perceptible voltage drop.
- `pm.probe_current_sine` - Sine wave probing signal amplitude that is used to
  estimate stator winding impedance.
- `pm.probe_speed_hold` - Speed setpoint for the initial spinup. At this speed
  flux linkage and noise threshold will be estimated.
- `pm.probe_loss_maximal` - Maximal heating losses on stator winding. This
  allows us to assume maximal machine current.
- `pm.forced_hold_D` - Forced current setpoint which should be enough to hold
  rotor in aligned position and force it turn.

If you use power supply that not tolerate reverse current then pay attention to
the wattage limit settings.

- `pm.watt_wA_reverse` - Maximal reverse current on DC link.
- `pm.watt_uDC_maximal` - Maximal overvoltage on DC link.

Also do not forget to reset the machine parameters if you have previously run
another machine.

    (pmc) pm_default_machine

## Sensors adjustment

To achieve the best accuracy PMC has the ability of self-adjust voltage and
current onboard sensors. The self-adjustment procedure allows you to match
measurement channels between phases.

	(pmc) pm_self_adjust

This is enough to do it once and store the values in the flash. But we recommend
to do it again if you change DC link voltage.

Also look into [Integrity Self Test](IntegritySelfTest.md) page to get more
info about self-adjustment functions.

## Number of the rotor pole pairs

This number relates the electrical speed with the mechanical. We believe you
are able to identify `Zp` simply by counting the number of magnets on the rotor
then divide it by 2. This is the most famous method.

	(pmc) reg pm.const_Zp <n>

If access to the machine is restricted and to count the magnets is impossible
then you can leave a value 1. Instead of mechanical speed you will see
electrical speed. By independent measuring the mechanical speed you can
estimate `Zp` and set it later.

## Impedance of stator windings

We measure the resistance `pm.const_Rs` by difference of voltage drop on two
values of holding current. For more accuracy you need to increase the probing
current or reduce DC link voltage.

Then we inject a high frequency sinusoidal signal to measure the full impedance
tensor and calculate DQ inductances `pm.const_im_Ld` and `pm.const_im_Lq`.

	(pmc) pm_probe_impedance

If the procedure fails with an error code sure that probing currents are
suitable for your machine.

## Rotor flux linkage

This parameter also known as `Kv` rating. Internal representation is
`pm.const_lambda` that linked with `Kv` by following equation.

```
	                      60
	lambda = ----------------------------
	          2 * PI * sqrt(3) * Kv * Zp
```

To identify `lambda` you have to run the machine. Also the rotor should rotate
at significant speed to get enough BEMF voltage. We do a forced initial spinup
to reach this condition.

	(pmc) pm_probe_spinup

If the procedure fails to spinup the machine try to adjust forced control
parameters. Also you can specify Kv manually if you know it exactly and Zp
number is already configured correcly.

	(pmc) reg pm.const_lambda_kv <rpm/V>

To get a more accurate flux linkage estimate you can run the machine at high
speed and request lambda probing manually. Do not load the machine at this.

	(pmc) pm_fsm_startup
	(pmc) reg pm.s_setpoint_rpm <rpm>
	...
	(pmc) pm_probe_const_flux_linkage

## No forced spinup

If you failed to start the machine with `pm_probe_spinup` you have an option to
identify the flux linkage in detached mode. You will have to rotate the machine
manually in this case.

	(pmc) pm_probe_detached

PMC will wait for the machine to reach at least `pm.zone_threshold` speed.

## Speed noise threshold

In case of you use `pm_probe_spinup` after a flux linkage we estimate speed
noise level to know the lower bound of flux observer operation. As a result
these threshold values are calculated.

	(pmc) reg pm.zone_noise
	(pmc) reg pm.zone_threshold

If you have estimated flux linkage in detached mode or specified Kv manually
you should probe the noise threshold manually when the machine is in run.

	(pmc) pm_fsm_startup
	(pmc) reg pm.s_setpoint_rpm <rpm>
	...
	(pmc) pm_probe_noise_threshold

## Moment of inertia

In case of you use `pm_probe_spinup` the final estimate is a moment of inertia
`pm.const_Ja`. To do this possible a speed maneuver will be performed. Note
that this may result energy regeneration so your power supply must tolerate
this. Either you should limit maximal DC link current reverse as stated above.

This constant is used to tune speed control loop. Also it is used in operation
to predict the speed changes from an applied current and so on.

If you have estimated flux linkage in detached mode or specified Kv manually
you should probe the moment of inertia manually.

	(pmc) pm_fsm_startup
	(pmc) reg pm.s_setpoint_rpm <rpm>
	...
	(pmc) pm_probe_const_inertia

## See also

Also look into [Trouble Shooting](TroubleShooting.md) page in case of you
getting any error codes in `pm.fsm_errno`.

