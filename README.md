# Phobia Motor Controller

PMC is an open project that aims to build the quality permanent magnet
synchronous motor (PMSM) controller for use in a variety of scopes like RC or
electrotransport.

## Hardware specification (rev5a)

* Dimension: 82mm x 55mm x 35mm.
* Weight: 40g (PCB) or about 400g (with wires and heatsink).
* Wires: 10 AWG.
* Connector: XT90-S and bullet 5.5mm.
* Battery voltage from 5v to 50v.
* Phase current up to 120A (with IPT007N06N, 60v, 0.75 mOhm).
* Light capacitor bank (4 x 4.7uF + 2 x 330uF).
* PWM frequency from 20 to 60 kHz.
* STM32F405RG microcontroller (Cortex-M4F at 168 MHz).

* Onboard sensors:
	* Two current shunts (0.5 mOhm) with amplifiers (AD8418) give a measuring range of 165A.
	* Battery voltage from 0 to 60v.
	* Three terminal voltages from 0 to 60v.
	* Temperature of PCB with NTC resistor.

* Motor interfaces:
	* Discrete Hall sensors or ABI incremental encoder (5v pull-up).
	* External NTC resistor (e.g. motor temperature sensing).

* Control interfaces:
	* CAN transceiver with optional termination resistor on PCB (5v).
	* USART to bootload and configure (3.3v).
	* Pulse input: RC servo, STEP/DIR, backup ABI (5v pull-up).
	* Two analog input channels (from 0 to 6v).

* Auxiliary interfaces:
	* Combined port: SPI, ADC, DAC, GPIO (3.3v).
	* BOOT pin combined with SWDIO to use embedded bootloader.
	* SWD to get hardware debug.
	* External FAN control (5v, 0.5A).

* Power conversion:
	* Battery voltage to 5v buck (up to 1A).
	* 5v to 12v boost (up to 100 mA).
	* 5v to 3.3v linear (up to 400 mA).

Look into [phobia-pcb](https://sourceforge.net/p/phobia/pcb/) repository for
PCB design source files. You can also view photos of the assembled PCBs in
[doc/imgs/](doc/imgs/).

## Software features

* Sensorless vector control of PMSM based on two inline current measurements.
* Fast and robust FLUX observer with gain scheduling.
* Self adjust of onboard measurements along symmetrical channels.
* Flux weakening control (**EXPERIMENTAL**).
* Two phase machine support (e.g. bipolar stepper) (**EXPERIMENTAL**).
* Advanced command line interface (CLI) with autocompletion and history.
* Non time-critical tasks are managed by [FreeRTOS](http://www.freertos.org/).
* Flash storage for all of configurable parameters.

* Advanced PWM scheme provides:
	* Reduced switching losses (clamp to GND) and fully utilised DC link voltage.
	* Hopping to get accurate ADC measurements near PWM-edges.
	* Prevents bootstrap circuit undervoltage condition.
	* Optional reduced ripple mode (clamp to middle) for precise control.

* Terminal voltage measurements (TVM):
	* In operation is used to reduce the effect of Dead-Time distortion.
	* BEMF tracking to get smooth start when motor is already running.
	* Self test of power stages integrity.
	* Self test of bootstrap retention time.

* Automated motor parameters identification with no additional tools:
	* Stator DC resistance (R).
	* Stator AC impedance in DQ frame (L1, L2, R).
	* Motor back EMF constant (E).
	* Moment of inertia (Ja).
	* Sensorless speed estimate peak-to-peak noise (MPPE).

* Advanced CAN networking:
	* Up to 30 nodes in peer network.
	* Network survey on request (no heartbeat messages).
	* Automated node address assignment.
	* Flash update across network.
	* IO forwarding to get CLI of remote node.
	* Flexible configurable data pipes.

* Operation at low or zero speed:
	* Forced control that applies a current vector without feedback to force rotor turn.
	* Freewheling.
	* High frequency injection (HFI) based on magnetic saliency (**EXPERIMENTAL**).
	* Discrete Hall sensors or ABI incremental encoder.

* Control loops:
	* Current control is always enabled.
	* Speed control loop.
	* Servo operation (**EXPERIMENTAL**).
	* Boost loop (battery charger) (**TODO**).

* Adjustable limits:
	* Phase current with adjustable derate on PCB overheat.
	* Motor voltage applied from VSI.
	* Battery current (power) consumption and regeneration.
	* DC link overvoltage and undervoltage.
	* Maximal speed and acceleration.

* Control inputs:
	* CAN bus flexible configurable data pipes.
	* RC servo pulse width.
	* STEP/DIR interface (**EXPERIMENTAL**).
	* Analog input with brake signal.
	* Manual control through CLI.
	* Custom embedded application can implement any control strategy.

* Available information:
	* Total distance traveled.
	* Battery energy (Wh) and charge (Ah) consumed (reverted).
	* Fuel gauge percentage.

## TODO

* Make a detailed documentation.
* Consider to add flux observer saliency tracking (for SRM).
* Add pulse output signal.

## Current Status

Now we can declare that PMC is ready to use in most applications. But there is
still a lot of unresolved issues. It may be difficult to configure the PMC for
a specific motor.

There are a few videos about PMC on
[youtube](https://www.youtube.com/channel/UCuSexDRnJVpbnZxfqPS3Eew).

Read further in [doc/GettingStarted](doc/GettingStarted.md).

