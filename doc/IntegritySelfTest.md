## Overview

This page describes how to verify the electrical integrity of PMC hardware.

**WARNING**: The self-test procedures listed here are MUST DO in case of new
hardware first power up.

## Automated Self Test

There is basic built-in integrity check. It is useful to do this the first time
you power PMC up to make sure that hardware is OK.

	(pmc) pm_self_test

This test does the following steps in this order.

- Get current sensors zero drift and check it is within the acceptable range.
- Check the power stages respond to the control.
- Check actual bootstrap retention time and compare with configuration.
- Check measurement accuracy against the distortion introduced by PWM.

You can do it with or without machine connected. Ignore the appropriate error
notification when do it without machine connected.

## Automated Self Adjust

The result of self-adjustment is also useful in diagnosis since it allows you
to identify abnormal deviations in measuring circuits.

	(pmc) pm_self_adjust

This adjustment does the following steps in this order.

- Get current sensors zero drift and check it is within the acceptable range.
- Adjust the voltage measurement channels relatively to DC link voltage.
- Adjust the current measurement channels pairwise.
- Adjust dead-time distortion compensation (DCU).

Note that in last two steps you need to connect the machine or test coil to let
the current flow in phase circuits.

## DCU adjustment

We estimate deadband time exactly the same as when measuring resistance by
difference of voltage drop on two values of holding current. To get accurate
deadband estimate you need to have a low machine resistance and minimal thermal
drift in time window of probing. We recommend you to use `Rs < 50 mOhm` machine
or test coil.

When probing machine DC resistance we use this deadband estimate as
compensation of constant voltage drop.

Note that estimated deadband may be different from actual deadtime in PWM
settings and depends on transient precess in power stages.

## PWM control

It may be useful to enable PWM with constant DC when testing and debugging a
new hardware.

	(pmc) hal_PWM_set_Z 0
	(pmc) hal_PWM_set_DC <xDC>

## See also

Also look into [Trouble Shooting](TroubleShooting.md) page in case of you
getting any error codes in `pm.fsm_errno`.

