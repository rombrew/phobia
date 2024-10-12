## Overview

This page describes the configuration of STEP/DIR input interface.

## Wiring

The step and direction signals are fed to STEP and DIR pins.

```
	           +---------------< STEP (or CW)
	           |    +----------< DIR (or CCW)
	           |    |    +-----> GND
	           |    |    |
	+----------+----+----+-------------------------+
	|         STEP DIR GND                         |
```

**WARNING**: Refer to your hardware manual or look into `src/hal/hw/...`
directory to find out actual pin mapping and voltage levels of your port.

## Configuration

Enable the appropriate mode of the STEP interface in HAL configuration.

    (pmc) reg hal.STEP_mode 1

Now you can see how the controller receives the control signal. If variable
`ap.step_POS` is changing then the STEP pulses are counted.

    (pmc) reg ap.step_POS

Choose what parameter you want to control. You can choose any of the registers
available for writing. By default the `pm.x_setpoint_location` register is
selected that mapped on absolute location in electrical radians.

	(pmc) reg ap.step_reg_ID <reg>

Define the step length constant to convert steps from `ap.step_POS` register to
a setpoint value in `ap.step_reg_DATA` register.

	(pmc) reg ap.step_const_Sm <rad/step>

Your probably should select location control loop. Note that machine must
already be configured to operate in speed control loop.

	(pmc) reg pm.config_LU_DRIVE 3

Now enable the machine startup control. The condition to start is no error code
in `pm.fsm_errno` register.

	(pmc) reg ap.step_STARTUP 1

## Precision

You can change the timebase frequency of the timer that is used to sample STEP
and DIR signals. By increasing the frequency you decrease the minimum pulse
width that can be accurately detected. But we use DMA based software GPIO
polling so a high frequency will cause a high computational load.

	(pmc) reg hal.STEP_frequency <Hz>

Maximal reasonable frequency is 1000000 Hz that gives a resolution about 1 us.
Thus the frequency of the STEP signal should not exceed 500 kHz.

