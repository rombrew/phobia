## Overview

If you are going to port PMC to your own hardware you should be aware of some
PMC internals. Also this page will be useful for understanding.

## Hardware

We have a three-phase Voltage Source Inverter (VSI) which typically consists of
at least six Metal–Oxide–Semiconductor Field-Effect Transistors (MOSFET). The
voltage at each of the output terminals is measured. Phase current A and B
(optionally C) is measured. The output terminals are connected to the machine.

```
	  (VCC) >---+--------+---------+---------+
	            |        |         |         |
	            |     --FET     --FET     --FET
	          -----      |         |         |
	          -----      +--< A    +--< B    +--< C
	            |        |         |         |
	            |     --FET     --FET     --FET
	            |        |         |         |
	  (GND) >---+--------+---------+---------+

	                // Three-phase VSI //
```

The three-phase bridge operates as three separate half-bridges. Each of which
can be abstractly considered as controlled voltage source. We use a symmetric
PWM scheme as shown in the diagram.

```
	  ---+    |    +-----------------------------+    |    +---
	     |         |                             |         |
	     |    |    |          A = 75\%           |    |    |
	     +---------+                             +---------+
	          |                                       |
	                    +-------------------+
	          |         |                   |         |
	                    |     B = 50\%      |
	  --------|---------+                   +---------|--------

	  ------+ | +-----------------------------------+ | +------
	        |   |                                   |   |
	        | | |             C = 90\%              | | |
	        +---+                                   +---+
	          |                                       |
	           <---------------- dT ----------------->
	          |                                       |
	                // Output voltage waveform //
```

Each half-bridge consists of two MOSFETs controlled by a gate drivers with a
specified dead-time `DT`. Depending on the direction of the current flow during
the dead-time the actual voltage on half-bridge may be different. The amount of
uncertainty in the output voltage `DTu` expressed as follows.

```
	        DT * DC_link_voltage
	 DTu = ----------------------
	                dT

	           |
	           |
	           +------+
	           |      |              +-----------------+
	        |--+      |              |                 |
	       ||        _|_             |                 |
	 GH ---||<-+     / \    ---------+                 +---------
	       ||  |     ---             |                 |
	        |--+      |           |                       |
	           |      |              |                 |
	           +------+       --->|   <--- DT             |
	           |                     |                 |
	           +---< Terminal     |                       |
	           |                     |                 |
	           +------+           |                       |
	           |      |     ------+                       +------
	        |--+      |           |                       |
	       ||        _|_          |                       |
	 GL ---||<-+     / \          +-----------------------+
	       ||  |     ---
	        |--+      |
	           |      |
	           +------+       // Half-bridge gate waveforms //
	           |
	           |
	          ---
```

The voltage divider (R1, R2) and filter capacitor (C1) are used to measure the
terminal voltage (uA, uB, uC) and supply voltage (uS). This RC scheme forms an
exponential integrator that allows us to restore the pulse width by measured
voltage. Additional resistor R3 can be used to bias the zero voltage into the
linear region. You can skip the terminal voltage sensing if you do not need
related features but supply voltage measuring is mandatory.


```
	                         +------< REF
	                         |                 // Voltage measurement //
	                         |
	                        | |
	                        | | R3 470K
	                        |_|
	               R1 470K   |               +---+
	                 ____    |              /    |
	  Terminal >----|____|---+--------+----- ADC |
	                         |        |     \    |
	                         |        |      +---+
	                        | |       |
	                 R2 27K | |     -----
	                        |_|     -----
	                         |        |    C1 1nF
	                         |        |
	                         +--------+
	                         |
	                        ---
	                        \ /  AGND
```

The current (iA, iB, iC) is measured in phases A and B (optionally C) using a
shunt and amplifier. Typically the measurement is distorted for some time after
a switching of the MOSFETs.

Note that we prefer to use in-line current measurement. To use low-side
measurement you will need to configure the software appropriately.

```
	                   // Current measurement //
	  >-----+
	        |  (+) |\   R1 = 0.5 mOhm
	        +------| \
	        |      |  \          +---+
	        \      |   \        /    |
	    R1  /      |Amp +------- ADC |
	        \      |   /        \    |
	        |  (-) |  /          +---+
	        +------| /
	        |      |/
	        |
	        +---< Terminal
```

## Sampling scheme

Currents are sampled strictly in the middle of PWM period simultaneously using
three ADCs. Then the voltages and other signals are sampled depending on
particular sampling scheme selected.

The values obtained are passed to the main IRQ handler to process. The MCU
software calculates a new value of duty cycle and load it to TIM. This value
will be used at next PWM period.

```
	                // Control loop diagram //

	  -->|<--------------------- dT ---------------------->|<-------------
	     |                                                 |
	     |  TIM update     +----+---+----+        PWM      |
	     | /               |    |   |    | <--  waveforms  |    next PWM period
	     |/                |    |   |    |                 |
	  ---*--*--*-----------------------------+-------------*--*--*--------
	     |  |  |                             |             |  |  |
	     iA uS uC         preload DC         |             iA uS uC
	     iB uA u?          to HW timer -->  TIM            iB uA u?
         iC uB u?                                          iC uB u?
	             \                          /                      \ ...
	              pm_feedback()            /
	                proc_set_DC(xA, xB, xC)
```

## Clean zones

We typically need about 5 us before current samples to be clean. If MOSFETs
switching occur at this time then ADC result is discarded.

```
	                // ADC sample clean zones //

	  -->|<--------------------- dT ---------------------->|<-------------
	     |                                                 |
	     |    PWM      +--------+---+--------+             |
	     |  waveforms  |        |   |        |             |    next PWM period
	     |             |        |   |        |             |
	  ---*--*--*-------------------------------------------*--*--*--------
	                                         |             |
	                                      -->|  clearence  |<--
	                                                       |      |
	                                                    -->| skip |<--
```

The diagram above shows `clearance` and `skip` parameters that is used in
software to decide ADC samples further usage. Based on transient performance of
current measurements circuit you should specify clerance thresholds in board
configuration. To get the best result you should have a current sensor with a
fast transient that allows you to specify narrow clearance zone.

```
	               1 - sqrt(3) / 2
	  clearance < -----------------
	                  PWM_freq
```

## TODO

