## Overview

This page explains how to understand the cause of problems that may occur when
deal with PMC.

## Error condition

If something goes wrong PMC stops and gives the error in **pm.fail_reason**.

	# reg pm.fail_reason

It may return the following values:

* **PM_ERROR_ZERO_DRIFT_FAULT** - Zero drift of current sensor is above the
  fault threshold. This may happen if current sensing path is damaged.
* **PM_ERROR_NO_MOTOR_CONNECTED** - Power stages are OK but no motor detected
  at the output terminals.
* **PM_ERROR_BOOTSTRAP_TIME** - Actual bootstrap retention time is lower that
  configured.
* **PM_ERROR_POWER_STAGE_DAMAGED** - No appropriate voltage response is
  detected at the output terminals. Power stages or terminal voltage sensors
  may be damaged.
* **PM_ERROR_LOW_ACCURACY** - Result of adjustment shows the deviation is above
  the fault threshold.
* **PM_ERROR_CURRENT_LOOP_IS_OPEN** - Means that we apply the full voltage but
  current setpoint is still not reached. If current sensing path is OK this may
  be caused by open circuit in motor phases or in its connection.
* **PM_ERROR_INLINE_OVERCURRENT** - This cannot be as we think that current is
  always under control. It may be current sensing is occasionally noisy. Also
  check for current regulation overshoot.
* **PM_ERROR_INVALID_OPERATION** - Means result of operation is NaN or other
  obviously unfit number.
* **PM_ERROR_SENSOR_HALL_FAULT** - Invalid value (0 or 7) of Discrete Hall
  sensors code was detected or values are inadequate (e.g. constant) during
  adjustment procedure.
* **PM_ERROR_SENSOR_ABI_FAULT** - **TODO**

These reasons come from high-level:

* **PM_ERROR_TIMEOUT** - Timeout has occurred because of no event occurred we
  waiting for. Typically it cannot reach speed setpoint.
* **PM_ERROR_NO_FLUX_CAUGHT** - FLUX observer has not caught flux for some
  reason. Maybe speed is too low or spinup was failed at all.
* **PM_ERROR_LOSS_OF_SYNC** - Position estimate is too poor (BLM model only).

