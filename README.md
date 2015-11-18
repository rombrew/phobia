# Phobia Motor Controller

PMC is an open project that aims to build a quality three-phase BLDC motor
controller for RC.

## Features

* Only vector control is supported. Features such as field weakening or
  regenerative breaking are an integral part of the vector control.
* Advanced PWM scheme to reduce switching losses and fully utilise DC bus.
* Adjustable power limits for consumption and regeneration.
* Only sensor-less operation. Two phase current sensors are used to obtain all
  the info need to control the motor.
* Fast and robust rotor position estimation algorithm based on Luenberger
  observer with a bit of gain scheduling.
* Operation at low or zero speed using HFI. But it requires a motor with
  magnetic saliency.
* Speed and absolute position control. So it can operate like a servo.
* Some simple motor failures as open circuit or winding short can be detected.
* All functions are available from command line interface through USART.
* Adjustable PWM frequency (currently 60kHz).
* Semi-automated motor parameters identification. No additional tools are needed.

## TODO

* Analyse accuracy of the position estimation.
* Analyse stability at fast transients.
* Analyse robustness to the motor parameters uncertainty.
* Add dead time compensation.
* Add BEMF waveform compensation.
* Add saliency distortion compensation.
* Add cogging torque compensation.
* Add CAN interface.
* Design the final hardware.
* Make a detailed documentation.

## Current Status

A significant part of the functions are already implemented in software.
Hardware is currently on prototyping stage. Schematic of the prototype power
board is
[here](https://bitbucket.org/amaora/phobia/downloads/phobia-f4d.pdf).

There are a few videos showing the operation of the prototype. It has been
tested mostly on AX-4005D 650KV motor.

[![PMC](https://i.ytimg.com/vi/1u1OoLLYefY/1.jpg)](https://www.youtube.com/watch?v=1u1OoLLYefY)
[![PMC](https://i.ytimg.com/vi/zmCW5BRNJgU/2.jpg)](https://www.youtube.com/watch?v=zmCW5BRNJgU)


