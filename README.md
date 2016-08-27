# Phobia Motor Controller

PMC is an open project that aims to build the quality three-phase BLDC motor
controller for RC.

![PMC rev1](https://bitbucket.org/amaora/phobia/downloads/pmcr1.jpg)

## Hardware features (rev1)

* Separate power and control PCB.
* Dimension: 40mm x 70mm x 20mm.
* Single supply from 5v to 42v.
* Phase current up to 40A.
* Controller: STM32F4XX (typical computation load is about 60%).
* MOSFETs: CSD18532Q5B (switching time less than 50ns).
* Sensors: Current sensors of phase A and B (ACS7XX), supply voltage sensing,
  NTC resistor.
* Interfaces: USART, CAN, SWD, LED.
* Anti-spark circuit: No.
* Reverse polarity protection: No.
* Overcurrent protection: No (implemented in software).

Look into [phobia-pcb](https://bitbucket.org/amaora/phobia-pcb) repository for
PCB design.

## Software features

* Sensor-less vector control. Features such as field weakening or regenerative
  breaking are an integral part.
* Advanced PWM scheme to reduce switching losses and fully utilise DC bus.
* Adjustable power limits for consumption and regeneration.
* Fast and robust rotor position estimation algorithm based on Luenberger
  observer with a bit of gain scheduling.
* Operation at low or zero speed using HFI. But it requires a motor with
  magnetic saliency.
* Speed and absolute position control. So it can operate like a servo.
* Some simple motor failures as open circuit or winding short can be detected.
* All functions are available from command line interface through USART.
* Adjustable PWM frequency (currently 60kHz).
* Automated motor parameters identification. No additional tools are needed.
* BEMF waveform online estimation.
* Operation at current values outside the sensor range (overcurrent).

## TODO

* Analyse accuracy of the position estimation.
* Analyse stability at fast transients.
* Analyse robustness to the motor parameters uncertainty.
* Add dead time compensation.
* Add saliency distortion compensation.
* Add cogging torque compensation.
* Add CAN interface.
* Design the final hardware.
* Make a detailed documentation.

## Current Status

A significant part of the functions are already implemented in software. New
revision of hardware is in development.

There are a few videos show the operation of the prototypes.

[![PMC](https://i.ytimg.com/vi/7XdBx24nlt0/default.jpg)](https://www.youtube.com/watch?v=7XdBx24nlt0)
[![PMC](https://i.ytimg.com/vi/rfigI6fnWxI/default.jpg)](https://www.youtube.com/watch?v=rfigI6fnWxI)
[![PMC](https://i.ytimg.com/vi/1u1OoLLYefY/default.jpg)](https://www.youtube.com/watch?v=1u1OoLLYefY)
[![PMC](https://i.ytimg.com/vi/zmCW5BRNJgU/default.jpg)](https://www.youtube.com/watch?v=zmCW5BRNJgU)


