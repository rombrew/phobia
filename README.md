# Phobia Motor Controller

PMC is an open project that aims to build a quality three-phase BLDC motor
controller for RC.

## Features

* Only vector control is supported. There is advanced PWM scheme to reduce
  switching losses and fully utilise DC bus. Current control loop can limit
  consumption or regeneration power. Features such as field weakening or
  regenerative breaking are an integral part of vector control.
* Only sensor-less operation. Two current sensors are used to obtain all the
  info need to control the motor.
* Fast and robust rotor position estimation algorithm (based on Luenberger
  observer with a bit of gain scheduling).
* Operation at low or zero speed using HFI. But it requires a motor with
  magnetic saliency.
* Speed and absolute position control. So it can operate like servo.
* Some simple motor faults as open circuit or winding short can be detected.
* All functions are available from command line interface through USART.
* Only hardware limited PWM frequency (currently 60kHz).
* Semi-automated motor parameters identification. No additional tools are needed.

## TODO

* Accuracy of the position estimation should be carefully analysed.
* Add CAN interface.
* Design the final hardware.
* Make a detailed documentation.

## Current Status

A significant part of the functions are already implemented in software.
Hardware is currently on prototyping stage. Schematic of the prototype power
board is
[here](https://bitbucket.org/amaora/phobia/downloads/phobia-f4d.pdf).

There are a few videos showing the operation of the prototype. It has been
tested mostly on AX 4005D motor.

[![PMC](https://i.ytimg.com/vi/1u1OoLLYefY/1.jpg)](https://www.youtube.com/watch?v=1u1OoLLYefY)
[![PMC](https://i.ytimg.com/vi/zmCW5BRNJgU/2.jpg)](https://www.youtube.com/watch?v=zmCW5BRNJgU)


