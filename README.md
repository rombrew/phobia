Phobia Motor Controller
=======================

PMC is an open project that aims to build a quality three-phase BLDC motor controller for RC.

## Features

* Only vector control is supported. There is advanced PWM scheme to reduce switching losses. Current control loop can limit consumption or regeneration power.
* Only sensor-less operation. Two current sensors are used to obtain all the info need to control the motor.
* Simple and robust rotor position estimation algorithm (Luenberger observer with a bit of gain scheduling).
* Operation at low speed using HFI. But it requires a motor with magnetic saliency.
* Speed and absolute position control.
* Some simple motor faults like open circuit or winding short can be detected.
* All functions are available from command line interface through USART.
* Only hardware limited PWM frequency (currently 60kHz).
* Semi-automatic motor parameters identification. No additional tools are needed.

## TODO

* CAN interface
* The final hardware

## Current Status

A significant part of the functions already implemented in software. Hardware are currently on prototyping stage. Schematic of the prototype power stages is [here](https://bitbucket.org/amaora/phobia/downloads/phobia-f4d.pdf).

![PMC](https://bitbucket.org/amaora/phobia/downloads/pmc1.jpg "PMC prototype")
![PMC](https://bitbucket.org/amaora/phobia/downloads/pmc2.jpg "PMC prototype")

[![PMC](https://i.ytimg.com/vi/1u1OoLLYefY/1.jpg)](https://www.youtube.com/watch?v=1u1OoLLYefY)
[![PMC](https://i.ytimg.com/vi/zmCW5BRNJgU/2.jpg)](https://www.youtube.com/watch?v=zmCW5BRNJgU)