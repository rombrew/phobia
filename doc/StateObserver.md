## Overview

This page describes some details of sensorless method used in PMC.

## Hardware overview

We have a three-phase voltage source inverter (VSI) which consists of six
transistors. The voltage at each of the output terminals is measured. Phase
current A and B is measured.  The output terminals are connected to the motor.

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
specified Dead-Time (TDT). Depending on the direction of the current during the
Dead-Time the actual voltage may be different. The amount of uncertainty in
output voltage:

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
	           +---< Output       |                       |
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
allows us to restore the pulse width by measured voltage. Optional resistor R3
is used to bias the operating point.

	           |
	           |
	           +------+     // Voltage measurement //
	           |      |
	        |--+      |               +------< REF
	       ||        _|_              |
	 GH ---||<-+     / \              |
	       ||  |     ---             | |
	        |--+      |              | | R3 (optional)
	           |      |              |_|
	           +------+     R1 470K   |               +---+
	           |              ____    |              /    |
	           +-------------|____|---+--------+----- ADC |
	           |                      |        |     \    |
	           +------+               |        |      +---+
	           |      |              | |       |
	        |--+      |       R2 27K | |     -----
	       ||        _|_             |_|     -----
	 GL ---||<-+     / \              |        |    C1 1nF
	       ||  |     ---              |        |      (C0G)
	        |--+      |               +--------+
	           |      |               |
	           +------+              ---
	           |                     \ /  AGND
	           |
	          ---

The current (iA, iB) is measured in phases A and B using a shunt and amplifier.
The measurement is distorted for some time after switching the half-bridge.

	           |
	           |
	           +------+        // Current measurement //
	           |      |
	        |--+      |
	       ||        _|_
	 GH ---||<-+     / \
	       ||  |     ---
	        |--+      |
	           |      |
	           +------+       R1 = 0.25 mOhm
	           |
	           +-----------------+
	           |                 |  (+) |\
	           +------+          +------| \
	           |      |          |      |  \      +---+
	        |--+      |          \      |   \    /    |
	       ||        _|_     R1  /      |Amp +--- ADC |
	 GL ---||<-+     / \         \      |   /    \    |
	       ||  |     ---         |  (-) |  /      +---+
	        |--+      |          +------| /
	           |      |          |      |/
	           +------+          |
	           |                 +---< Output
	           |
	          ---

Also supply voltage (uS) is measured using a voltage divider.

## Control loop

ADC is sampled at the begin of PWM period. The values obtained are processed by
software. The new value of duty cycle is loaded to the hardware timer. This
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

We need about 5 us before ADC samples to be clean. If MOSFET switching occurs
at this time then the result is not used.

Now consider the internals of the **pm_feedback** procedure. At simplest view
it is state observer. We use the following state vector X[5]:

	X[0] (iD) current on D axis [A]
	X[1] (iQ) current on Q axis [A]
	X[2] DQ frame cosine
	X[3] DQ frame sine (atan(X[3] / X[2]) gives a rotor angle)
	X[4] (wS) electrical speed [radian/s]

Also there is **drift_Q** variable that in fact is a part of state vector. The
observer procedure consists of the following steps:

1. Get corrected terminal voltages (uXY) from the measurements (uA, uB, uC). It
   corresponds to the previous PWM period.
2. Update X using the corrected uXY. Thus we have done the propagation step on
   previous PWM period.
3. Update X using current measurement (iA, iB).
4. Propagate X to the next PMW period with supposed terminal voltages (uXY).

Note that 1,2 are performed if voltage correction conditions are met, 3 is
performed if current measurement is recognized as reliable, 4 is performed
always.

After that we have state vector estimation to the moment of next PWM period
begins. This state vector goes into the current control loop and then into
voltage control procedure. As a result we get the duty cycle (xA, xB, xC) to
the next PWM period. Then it transferred to the hardware timer.

## Block diagram

	                    +----+       +-------+      +-----+    +-----+
	       iD ---X----->| PI |-+---->| DQ   /|      |     |--->|     |
	             |      +----+ |  -->|   /   |-+--->| PWM |--->| VSI |
	    +----+   |      +----+ | /   |/   XY |---+->|     |--->|     |
	    | PI |---|--X-->| PI |---    +-------+ | |  +-----+    +-----+
	    +----+   |  |   +----+ |         ^     | |              | | |
	      ^      |  |          |         |     | |  +-----------+ | | uA
	      |      +--|------+   |         |     | |  | +---------|-+ | uB
	wS -->X      |  |      |   |         |     | |  | | +-------|-|-+ uC
	      |      |  |   +-----------+    |     | |  | | |       | | |
	      |      |  |   |    HF     |----/     v v  v v v       | | |
	      \------|--|---| injection |    |   +------------+     | | |
	      |      |  |   +-----------+    +---|    FLUX    |<----+ | | iA
	      |      |  |      |                 |  observer  |<----|-+ | iB
	      |      |  |      +--< residual     +------------+     | | |
	      |      |  |                          |  |  |          | | |
	      |      +-----------------------------+  |  | ~iD      | | |
	      |         +-----------------------------+  | ~iQ      -----
	      +------------------------------------------+ ~wS     /     \
	                                                            Motor
	                                                           \     /
	            // Block diagram //                             -----

## FLUX observer

The main sensorless method is a flux linkage observer. It is based on BEMF
induced in stator windings when rotor flux is moving. This method works well
when the value of BEMF is greater that voltage distortion (dUDT). Look at the
synchronous machine equations:

	iD
	-- * Ld = uD - iD * R + fluxQ * wS
	dT

	iQ
	-- * Lq = uQ - iQ * R - fluxD * wS + drift_Q
	dT

Where uD uQ is voltages applied from VSI, fluxD fluxQ is a flux linkage.

	uD = X[2] * uX + X[3] * uY
	uQ = X[2] * uY - X[3] * uX
	
	fluxD = Ld * iD + E
	fluxQ = Lq * iQ

To implement an observer we need to know Ld Lq R E parameters.

