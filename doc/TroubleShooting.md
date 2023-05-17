## Overview

This page explains how to understand the cause of problems that may occur when
deal with PMC.

## Error condition

If something goes wrong PMC stops and gives the error code in `pm.fsm_errno`.

	(pmc) reg pm.fsm_errno

There are internal PMC codes:

#### PM_ERROR_ZERO_DRIFT_FAULT

Zero drift of current sensor is above the fault threshold. This may happen if
current sensing circuit is damaged.

#### PM_ERROR_NO_MOTOR_CONNECTED

Power stages are ok but no motor detected at the output terminals.

#### PM_ERROR_BOOTSTRAP_FAULT

Actual bootstrap retention time is lower that configured.

#### PM_ERROR_POWER_STAGE_DAMAGED

No appropriate voltage response detected at the output terminals. Power stages
or terminal voltage sensors may be damaged.

#### PM_ERROR_INSUFFICIENT_ACCURACY

Result of adjustment shows the parameter deviation is above the fault
threshold. Check the voltage and current measurement accuracy.

#### PM_ERROR_CURRENT_LOOP_FAULT

This means that we apply the full voltage but actual current is still near
zero. If current sensing circuit is ok this may be caused by open circuit in
motor phases or in its connection. Also a common cause of this error is a low
level of DC link voltage in combine with large stator resistance.

#### PM_ERROR_INSTANT_OVERCURRENT

Overcurrent accident detected. Check for current regulation transient and noise
level. The most likely reason is an unstable sensorless estimate causes a
current burst.

#### PM_ERROR_DC_LINK_OVERVOLTAGE

DC link overvoltage accident detected. Check for the motor speed does not rises
uncontrolled.

#### PM_ERROR_UNCERTAIN_RESULT 

Result of adjustment is uncertain or ill-conditioned. The observability
conditions may not have been met. For example there was no speed maneuver
during the moment of inertia probing.

#### PM_ERROR_INVALID_OPERATION

Numerical instability inside PMC control code or deliberately invalid operation
was requested.

#### PM_ERROR_SENSOR_HALL_FAULT

Forbidden value of Hall code was detected. Or result of Hall adjustment shows
an inadequacy of sensor measurements.

These codes come from application level:

#### PM_ERROR_TIMEOUT

Timeout has occured because of no event happen we waiting for. This typically
happens when PMC cannot reach the speed setpoint in motor probing procedures.

#### PM_ERROR_NO_FLUX_CAUGHT

FLUX observer has not caught the rotor position for some unknown reason. Maybe
speed is too low or spinup was failed at all.

#### PM_ERROR_NO_MOTOR_SYNC

Position estimate discrepancy is out of range. This erorr code can only come
from BLM model.

These codes arise by hardware:

#### PM_ERROR_HW_UNMANAGED_IRQ

PMC control code has not completed its execution when the next ADC IRQ
occurred. Try to decrease PWM frequency or disable some computationally
expensive features.

#### PM_ERROR_HW_OVERCURRENT

Overcurrent accident detected by hardware.

#### PM_ERROR_HW_OVERTEMPERATURE

PCB overheat condition has been reached. Temperature regulation by current
derate is not functional.

#### PM_ERROR_HW_EMERGENCY_STOP

Emergency situation detected by emergency endstop signal.


