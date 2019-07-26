## Overview

If you are going to port PMC to your own hardware you should be aware of
requirements and restrictions. Also this page will be useful for understanding
PMC internals.

## Hardware

We have a three-phase voltage source inverter (VSI) which consists of six
transistors. The voltage at each of the output terminals is measured. Phase
current A and B is measured. The output terminals are connected to the motor.

	  (+)   >---+------+--------+--------+
	            |      |        |        |
	            |   --FET    --FET    --FET
	           ---     |        |        |
	           ---     +--< A   +--< B   +--< C
	            |      |        |        |
	            |   --FET    --FET    --FET
	            |      |        |        |
	  (GND) >---+------+--------+--------+
	
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

	         2 * TDT * uS
	 dUDT = --------------
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
terminal voltage (uA, uB, uC). This scheme forms an exponential integrator that
allows us to restore the pulse width by measured voltage. Additional resistor
R3 is used to bias the operating point into the linear region. You can skip
voltage sensing if you do not need related features.

	          
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
need to adapt the software.

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

## Current zones

