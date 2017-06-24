## Overview

This page explains how to overcome the difficulties that may arise when working
with PMC.

## Error condition

If something goes wrong we stop and give the error number. Some of commands
print it out or you could view manually.

	# pm_m_errno

It may return the following values.

* "No Error" in case of everything is ok.
* "Current Sensor A" if zero drift of current sensor A is above the fault
  threshold. This may happen if analog path is dead, it is dangerous to run
  without feedback.
* "Current Sensor B" the same for current sensor B.
* "Open Circuit" means that we apply the full voltage but current is still
  zero. If sensors is ok this may be caused by open circuit in motor phases or
  its connection. Also it may happen if power stage is dead.
* "Over Current" this cannot be, we think that current is always under control.
* "Low Voltage" supply voltage is below the fault threshold.
* "High Voltage" supply voltage is above the fault threshold.
* "Stability Loss" means that state observer is no longer understand what is
  happening. It may be caused by inconsistency of model with real motor. We
  cannot predict everything, if the motor has been damaged it definitely will
  cause this error.

Reset the error to be able to try again.

	# pm_m_errno 0

## Asserts

In the code of commands we check some important conditions. If it fails you
will see that condition. To understand what this means it is necessary to look
into the code. We put such terms in the code to prevent dangerous accidents.

