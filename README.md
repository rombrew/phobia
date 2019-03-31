# Phobia Motor Controller

PMC is an open project that aims to build the quality three-phase BLDC motor
controller for RC and robotics.

![PMC](doc/phobia_rev4b.jpg)

## Hardware specification (rev4c)

* Dimension: 70mm x 50mm x 15mm.
* Weight: 40g.
* Single supply from 6v to 50v.
* Phase current up to 80A (IPT007N06N, 60v, 0.75 mOhm).
* Lightweight capacitor bank (3 x 2.2uF + 3 x 220uF).
* PWM frequency from 20 to 80 kHz.
* Onboard sensors:
	* Two current shunts (0.25 mOhm) with amplifiers (AD8417) give a measuring range of 100A.
	* Supply voltage from 0 to 60v.
	* Three terminal voltages from 0 to 60v.
	* Temperature of PCB with NTC resistor.
* Motor interfaces:
	* Hall Sensors or Quadrature Encoder (5v pull-up).
	* External NTC resistor (e.g. motor temperature sensing).
* Control interfaces:
	* CAN transceiver with optional termination resistor on PCB (5v).
	* USART to bootload and configure (3.3v).
	* Pulse input control: RC servo pulse width, STEP/DIR, QEP (5v tolerant).
	* Analog input control (from 0 to 5v).
* Auxiliary interfaces:
	* Two combined ports with: SPI, I2C, USART, ADC, DAC, GPIO (3.3v).
	* BOOT and RESET pins to use embedded bootloader.
	* SWD to hardware debug.
	* External FAN control (5v).
* Power conversion:
	* Supply to 5v buck (up to 1A).
	* 5v to 12v boost (up to 100 mA).
	* 5v to 3.3v linear (up to 400 mA).
	* 5v to 3.3vREF optional reference voltage (accuracy 0.2%, 25 mA).
* STM32F405RG microcontroller (50% typical computational load).
* Anti-spark circuit: No.
* Reverse polarity protection: No.
* Overcurrent protection: Implemented in software.

Look into [phobia-pcb](https://bitbucket.org/amaora/phobia-pcb) repository for
PCB design source files.

## Software features

* Sensorless vector control. All the code of motor control was written from
  scratch, no external libs are used, portable as it is plain C code.
* Advanced PWM scheme to reduce switching losses and fully utilise DC bus voltage.
* Fast and robust flux linkage estimation algorithm based on Luenberger
  observer with gain scheduling.
* Terminal voltage sensing to reduce the effect of Dead-Time (**EXPERIMENTAL**).
* Operation at low or zero speed:
	* Forced control that applies a current vector without feedback to force rotor turn.
	* High frequency injection (HFI) based on magnetic saliency.
	* Hall Sensors or Quadrature Encoder (**TODO**).
* Control loops:
	* Current (torque) control is always enabled.
	* Speed control loop.
	* Servo operation (**TODO**).
	* Alternator voltage rectifier (**TODO**).
* Adjustable limits:
	* Phase current (with adjustable derate from overheat).
	* Power consumption and regeneration.
 	* Maximal speed and acceleration.
* Control inputs:
	* CAN bus (**TODO**).
	* RC servo pulse width.
	* Analog (**TODO**).
	* Manual control through CLI.
	* Custom embedded application can implement any control strategy.
* Automated motor parameters identification with no additional tools.
* Self test of hardware integrity to diagnose troubles.
* Terminal voltage tracking to get smooth start when motor is already running (**EXPERIMENTAL**).
* Advanced command line interface (CLI) with autocompletion and history.
* Operation at current values outside the sensor range (**EXPERIMENTAL**).
* Two phase machine support (e.g. bipolar stepper) (**EXPERIMENTAL**).
* Non critical tasks are managed by [FreeRTOS](http://www.freertos.org/).
* Flash storage for all of configurable parameters.

## TODO

* Analyse of rapid transient modes. Introduce an iron saturation model if needed.
* Analyse HFI operation on large current values.
* Make a detailed documentation.

## Current Status

Now we can declare that PMC is ready to use in most applications. But there is
still a lot of unresolved issues. It may be difficult to configure the PMC for
a specific motor.

There are a few videos that show the operation of the prototypes (may be outdated).

[![PMC](https://i.ytimg.com/vi/IM0k0_boXc4/default.jpg)](https://www.youtube.com/watch?v=IM0k0_boXc4)
[![PMC](https://i.ytimg.com/vi/n_E2ThFQvD4/default.jpg)](https://www.youtube.com/watch?v=n_E2ThFQvD4)
[![PMC](https://i.ytimg.com/vi/rfigI6fnWxI/default.jpg)](https://www.youtube.com/watch?v=rfigI6fnWxI)

Read more in [Getting Started](doc/GettingStarted.md).

