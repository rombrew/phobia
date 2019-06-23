# Phobia Motor Controller

PMC is an open project that aims to build the quality permanent magnet
synchronous motor (PMSM) controller for use in a variety of scopes like RC or
electrotransport.

![PMC](doc/phobia_rev4b.jpg)

## Hardware specification (rev4c)

* Dimension: 70mm x 50mm x 15mm.
* Weight: 40g.
* Single supply from 5v to 50v.
* Phase current up to 80A (IPT007N06N, 60v, 0.75 mOhm).
* Lightweight capacitor bank (3 x 2.2uF + 3 x 220uF).
* PWM frequency from 20 to 80 kHz.
* Onboard sensors:
	* Two current shunts (0.5 mOhm) with amplifiers (AD8418) give a measuring range of 150A.
	* Supply voltage from 0 to 60v.
	* Three terminal voltages from 0 to 60v.
	* Temperature of PCB with NTC resistor.
* Motor interfaces:
	* Hall Sensors or Quadrature Encoder (5v pull-up).
	* External NTC resistor (e.g. motor temperature sensing).
* Control interfaces:
	* CAN transceiver with optional termination resistor on PCB (5v).
	* USART to bootload and configure (3.3v).
	* Pulse input control: RC servo pulse width, STEP/DIR, QEP (5v-tolerant).
	* Two analog input channels (from 0 to 5v).
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
* STM32F405RG microcontroller (70% typical computational load).
* Anti-spark circuit: No.
* Reverse polarity protection: No.
* Overcurrent protection: Implemented in software.

Look into [phobia-pcb](https://bitbucket.org/amaora/phobia-pcb) repository for
PCB design source files.

## Software features

* Sensorless vector control of PMSM based on two inline current measurement.
* Advanced PWM scheme to reduce switching losses and fully utilise DC link voltage.
* Fast and robust multi-hypothesis flux observer with gain scheduling.
* Terminal voltage sensing to reduce the effect of Dead-Time.
* Automated motor parameters identification with no additional tools.
* Operation at low or zero speed:
	* Forced control that applies a current vector without feedback to force rotor turn.
	* High frequency injection (HFI) based on magnetic saliency (**EXPERIMENTAL**).
	* Hall Sensors or Quadrature Encoder (**TODO**).
* Control loops:
	* Current control is always enabled.
	* Brake function when current control is in use.
	* Speed control loop.
	* Servo operation (**EXPERIMENTAL**).
	* Alternator voltage rectifier (**TODO**).
* Adjustable limits:
	* Phase current (with adjustable derate from overheat).
	* Phase brake current.
	* Source power consumption and regeneration.
	* Source current consumption and regeneration.
	* DC link overvoltage and undervoltage.
	* Maximal speed and acceleration (as part of speed control loop).
* Control inputs:
	* CAN bus (**TODO**).
	* RC servo pulse width.
	* STEP/DIR (**TODO**).
	* Analog input.
	* Manual control through CLI.
	* Custom embedded application can implement any control strategy.
* Self test of hardware integrity to diagnose troubles.
* Terminal voltage tracking to get smooth start when motor is already running (**EXPERIMENTAL**).
* Advanced command line interface (CLI) with autocompletion and history.
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

[![PMC](https://i.ytimg.com/vi/ANp_5zZkh48/default.jpg)](https://www.youtube.com/watch?v=ANp_5zZkh48)
[![PMC](https://i.ytimg.com/vi/IM0k0_boXc4/default.jpg)](https://www.youtube.com/watch?v=IM0k0_boXc4)
[![PMC](https://i.ytimg.com/vi/rfigI6fnWxI/default.jpg)](https://www.youtube.com/watch?v=rfigI6fnWxI)

Read more in [Getting Started](doc/GettingStarted.md).

