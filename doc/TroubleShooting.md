## Overview

This page explains how to overcome the difficulties that may arise when deal
with PMC.

## Error condition

If something goes wrong we stop and give the error number. Some of commands
print it out or you could view manually.

	# reg pm.fail_reason

It may return the following values.

* **PM_ERROR_ZERO_DRIFT_FAULT** if zero drift of current sensor A is above the
  fault threshold. This may happen if analog path is dead, it is dangerous to
  run without feedback.
* **PM_ERROR_NO_MOTOR_CONNECTED** 
* **PM_ERORR_POWER_STAGE_FAULT**
* **PM_ERROR_ACCURACY_FAULT**
* **PM_ERROR_CURRENT_LOOP_FAULT** means that we apply the full voltage but
  current is still zero. If sensors is ok this may be caused by open circuit in
  motor phases or its connection. Also it may happen if power stage is dead.
* **PM_ERROR_OVER_CURRENT** this cannot be, we think that current is always
  under control.
* **PM_ERROR_LU_RESIDUAL_UNSTABLE** means that state observer is no longer
  understand what is happening. It may be caused by inconsistency of model with
  real motor.
* **PM_ERROR_LU_INVALID_OPERATION**

