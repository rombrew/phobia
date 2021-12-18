## Overview

If you are going to port PMC to your own hardware you should be aware of some
PMC internals. Also this page will be useful for understanding.

## Hardware

We have a three-phase voltage source inverter (VSI) which consists of six
field-effect transistors (FET). The voltage at each of the output terminals is
measured. Phase current A and B is measured. The output terminals are connected
to the motor.

	  (+)   >---+--------+---------+---------+
	            |        |         |         |
	            |     --FET     --FET     --FET
	          -----      |         |         |
	          -----      +--< A    +--< B    +--< C
	            |        |         |         |
	            |     --FET     --FET     --FET
	            |        |         |         |
	  (GND) >---+--------+---------+---------+
	
	  // Three-phase voltage source inverter //

The three-phase bridge operates as three separate half-bridges. Each of which
can be abstractly considered as controlled voltage source. We use a symmetric
PWM scheme as shown in the diagram.

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

Each half-bridge consists of two MOSFETs controlled by a gate drivers with a
specified Dead-Time (TDT). Depending on the direction of the current flow
during the Dead-Time the actual voltage may be different. The amount of
uncertainty in output voltage expressed as follows:

	         2 * TDT * DC_link_voltage
	 dUDT = ---------------------------
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
	           +------+       --->|   <--- TDT            |
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

The voltage divider (R1, R2) and filter capacitor (C1) are used to measure the
terminal voltage (uA, uB, uC). This RC scheme forms an exponential integrator
that allows us to restore the pulse width by measured voltage. Additional
resistor R3 is used to bias the operating point into the linear region. You can
skip voltage sensing if you do not need related features.

To get acceptable accuracy you need to make sure that the RC scheme time
constant is comparable to dT.

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
	                         |        |      (C0G)
	                         +--------+
	                         |
	                        ---
	                        \ /  AGND

The current (iA, iB) is measured in phases A and B using a shunt and amplifier.
The measurement is distorted for some time after switching the half-bridge.

Note that we use in-line current measurement. To use low-side measurement you
will need to adapt the software.

	                   // Current measurement //
	  >-----+
	        |  (+) |\       R1 = 0.5 mOhm
	        +------| \
	        |      |  \      +---+
	        \      |   \    /    |
	    R1  /      |Amp +--- ADC |
	        \      |   /    \    |
	        |  (-) |  /      +---+
	        +------| /
	        |      |/
	        |
	        +---< Terminal

Also supply voltage (uS) is measured using a voltage divider.

## Control loop

ADC is sampled in the middle of PWM period. The values obtained are processed
by software. The new value of duty cycle is loaded to the hardware timer. This
value is used at next PWM period. ADC samples are made using two ADCs in order
shown in the diagram.

	                // Control loop diagram //
	
	     |<--------------------- dT ---------------------->|
	     |                                                 |
	     |  TIM update     +----+---+----+        PWM      |
	     | /               |    |   |    | <--  waveforms  |
	     |/                |    |   |    |                 |
	  ---*--*--*-----------------------------+-------------*---
	     |  |  |                             |             |
	     iA uS uB         preload DC         |
	     iB uA uC          to hw timer -->  TIM
	             \                          /
	              pm_feedback()            /
	                    p_set_DC(xA, xB, xC)

We need about 3 us before ADC samples to be clean. If MOSFET switching occurs
at this time then the result is discarded.

## Current zones

