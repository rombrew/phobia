## Overview

This page describes how to verify the electrical integrity of PMC hardware.

**WARNING**: The procedures listed here are MUST DO in case of new hardware
first powerup.

## Automated Self Test

There is basic built-in integrity check. It is useful to do this the first time
you power PMC up to make sure that hardware is ok.

	(pmc) pm_self_test

This test does the following steps in this order:

- Get current sensors zero drift and check it is within the acceptable range.
- Check actual bootstrap retention time and compare with configuration.
- Check the power stage responds to the control.
- Check measurement accuracy against the distortion introduced by PWM.

You can do it with or without machine connected. Ignore the appropriate error
notification when do it without machine connected.

## Automated Self Adjust

The result of self-adjustment is also useful in diagnosis since it allows you
to identify abnormal deviations in measuring circuits.

	(pmc) pm_self_adjust

This adjustment does the following steps in this order:

- Get current sensors zero drift and check it is within the acceptable range.
- Adjust the voltage measurement channels relatively to DC link voltage.
- Adjust the current measurement channels pairwise.
- Adjust dead-time distortion compensation (DTC).

Note that in last two steps you need to connect the machine (or test coil) to
let the current flow in phase circuits.

## See also

Also look into [Trouble Shooting](TroubleShooting.md) page in case of you
getting any error codes in `pm.fsm_errno`.

