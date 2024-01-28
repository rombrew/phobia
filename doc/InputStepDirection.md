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

## Configuration

First you need to enable the appropriate mode of the STEP interface.

    (pmc) reg hal.STEP_mode 1

Now you can see how the controller receives the control signal. If variable
`ap.step_POS` is changing then the STEP pulses are counted.

    (pmc) reg ap.step_POS

TODO

