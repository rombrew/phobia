## Overview

This page explains how to overcome the difficulties that can arise when deal
with PMC.

## Error condition

If something goes wrong we stop and give the error in **pm.fail_reason**.

	# reg pm.fail_reason

It may return the following values.

* **PM_ERROR_ZERO_DRIFT_FAULT** if zero drift of current sensor is above the
  fault threshold. This may happen if current sensing path is damaged.
* **PM_ERROR_NO_MOTOR_CONNECTED** if no motor detected at the output
  terminals.
* **PM_ERORR_POWER_STAGE_FAULT** if no voltage response is detected at the
  output terminals. Power stages or terminal voltage sensors may be damaged.
* **PM_ERROR_ACCURACY_FAULT** if the adjustment has occurred with a error above
  the fault threshold.
* **PM_ERROR_CURRENT_LOOP_FAULT** means that we apply the full voltage but
  current is still zero. If current sensors is ok this may be caused by open
  circuit in motor phases or its connection.
* **PM_ERROR_OVER_CURRENT** this cannot be, we think that current is always
  under control.
* **PM_ERROR_RESIDUAL_UNSTABLE** means that state observer is no longer
  understand what is happening. It may be caused by inconsistency of model with
  real motor. Usually appears in intense transient conditions.
* **PM_ERROR_INVALID_OPERATION** if invalid conditions is detected like zero
  divizion or appearance of NaN.

