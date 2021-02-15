## Overview

This page describes how to verify the integrity of PMC.

## Automated Self Test

Basic built-in integrity check. It is useful to do this the first time you
power PMC on to make sure that hardware is OK.

	# pm_self_test

The results of self-adjustment are also useful.

	# pm_self_adjust

You can do it with or without motor connected. Ignore appropriate error reason
when do it without motor connected. In case of motor connected you also check
the current sensing misalignment. Above two procedures are must do in case of
new hardware first powerup.

