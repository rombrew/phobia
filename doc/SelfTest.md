## Overview

This page describes how to verify the integrity of PMC.

## Automated Self Test

There is basic built-in integrity check. It is useful to do this the first time
you power PMC up to make sure that hardware is ok.

	(pmc) pm_self_test

You can do it with or without motor connected. Ignore the appropriate error
notification when do it without motor connected.

## Automated Self Adjust

The result of self-adjustment is also useful since it allows you to identify
abnormal deviations in measuring circuits.

	(pmc) pm_self_adjust

In case of motor connected you also check the current sensing circuit. Above
two procedures are must do in case of new hardware first powerup.

