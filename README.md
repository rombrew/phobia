# Phobia Motor Controller

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build](https://github.com/rombrew/phobia/actions/workflows/makefile.yml/badge.svg)](https://github.com/rombrew/phobia/actions/workflows/makefile.yml)

PMC is an open project that aims to build the quality permanent magnet
synchronous machine (PMSM) controller for use in a variety of scopes like RC or
electric transport.

## Brief description

PMC is ready to use in most intended applications. You can flash any supported
third-party hardware to work with PMC or use our original hardware.

Read further in [doc/GettingStarted](doc/GettingStarted.md).

There are a few videos about PMC on [YouTube](https://www.youtube.com/@romblv).

## Software features

- Sensorless vector control of PMSM by measurement of currents and voltages.
- Robust ORTEGA observer with gain scheduling against speed.
- Accurate KALMAN observer having convergence at HF injection.
- Flux weakening and MTPA control.
- Three and two phases machine connection.
- Hardware abstraction layer (HAL) over STM32F4 and STM32F7.
- Various controller hardware are supported (including VESC clones).
- Command Line Interface (CLI) with autocompletion and history.
- Graphical User Interface (PGUI) based on
  [Nuklear](https://github.com/Immediate-Mode-UI/Nuklear) and
  [SDL2](https://www.libsdl.org/).
- Non time-critical tasks are managed by
  [FreeRTOS](http://www.freertos.org/).
- USB protocol stack from
  [CherryUSB](https://github.com/sakumisu/CherryUSB).
- Least Squares estimate library
  [LSE](https://github.com/rombrew/lse).

- Phase current sampling scheme includes two or three sensors configuration
  with inline or low-side placement.
- Self-adjustment of all onboard measurements (current and voltage) along
  symmetrical channels.
- Dead-Time Compensation (DCU) based on currents polatity.

- Advanced SVPWM scheme provides:
	- Reduced switching losses and fully utilised DC link voltage.
	- Reduced voltage distortion for precise control.
	- Voltage hopping to get accurate ADC measurements with inline current sensors.
	- Prevent bootstrap circuit undervoltage condition.

- Terminal voltage measurements (TVM):
	- Back EMF voltage tracking to catch an already running machine.
	- Self-test of the power stages integrity and machine wiring.
	- Self-test of bootstrap retention time.

- Automated machine parameters identification:
	- Stator DC resistance (Rs).
	- Stator AC impedance in DQ frame (Ld, Lq, Rz).
	- Rotor flux linkage constant (lambda).
	- Mechanical moment of inertia (Ja).

- Automated configuration of external measurements:
	- Discrete Hall sensors installation angles recognition.
	- EABI resolution and direction adjustment.
	- Analog Hall sensors signal distortion adjustment.

- Operation at low or zero speed:
	- Forced control that applies a current vector without feedback to
	  force rotor hold or spinup.
	- Freewheeling.
	- High Frequency Injection (HFI) based on magnetic saliency.
	- Discrete Hall sensors.
	- AB quadrature incremental encoder (EABI).
	- Absolute encoder on SPI interface (AS5047).
	- Analog Hall sensors and resolver decoder (**EXPERIMENTAL**).

- Nested control loops:
	- Detached voltage monitoring.
	- Current control PI regulator with feedforward compensation.
	- Speed control PID regulator with load torque compensation.
	- Absolute location control with constant acceleration regulator.

- Adjustable constraints:
	- Phase current (forward and reverse, on HFI current, weakening current).
	- Overtemperature protection (derate phase current or halt).
	- Machine voltage applied from VSI.
	- DC link current consumption and regeneration.
	- DC link overvoltage and undervoltage.
	- Maximal speed and acceleration (forward and reverse).
	- Absolute location limits in case of servo operation.

- Input control interfaces:
	- Analog input knob and brake signal.
	- RC servo pulse width modulation.
	- CAN bus flexible configurable data transfers.
	- STEP/DIR (or CW/CCW) interface (**EXPERIMENTAL**).
	- Manual control through CLI or PGUI.
	- Custom embedded application can implement any control strategy.

- Advanced CAN networking:
	- Up to 30 nodes in peer network.
	- Network survey on request (no heartbeat messages).
	- Automated node address assignment.
	- IO forwarding for remote node CLI login.
	- Flexible configurable data transfers.

- Available information:
	- Machine state (angle, speed, torque, current, etc.)
	- DC link voltage and current consumption.
	- Information from temperature sensors.
	- Total distance traveled.
	- Battery energy (Wh) and charge (Ah) consumed.
	- Fuel gauge percentage.

## Hardware specification (`REV5`)

- Dimension: 82mm x 55mm x 35mm.
- Weight: 40g (PCB) or about 400g (with wires and heatsink).
- Wires: 10 AWG.
- Connector: XT90-S and bullet 5.5mm.
- Battery voltage from 5v to 52v.
- Phase current up to 120A (with IPT007N06N, 60v, 0.75 mOhm).
- Light capacitor bank (4 x 4.7uF + 2 x 330uF).
- PWM frequency from 20 to 60 kHz.
- STM32F405RG microcontroller (Cortex-M4F at 168 MHz).

- Onboard sensors:
	- Two current shunts 0.5 mOhm with amplifiers AD8418 give a
	  measuring range of 165A.
	- Battery voltage from 0 to 60v.
	- Three terminal voltages from 0 to 60v.
	- Temperature of PCB with NTC resistor.

- Machine interfaces:
	- Discrete Hall sensors or EABI encoder (5v pull-up).
	- External NTC resistor (e.g. machine temperature sensing).

- Control interfaces:
	- CAN transceiver with optional termination resistor on PCB (5v).
	- USART to bootload and configure (3.3v).
	- RC servo PWM or STEP/DIR (5v pull-up).
	- Two analog input channels (from 0 to 6v).

- Auxiliary interfaces:
	- SPI port with alternative functions: ADC, DAC, GPIO (3.3v).
	- BOOT pin combined with SWDIO to use embedded bootloader.
	- External FAN control (5v, ~0.5A).

- Power conversion:
	- Battery voltage to 5v buck (~1A).
	- 5v to 12v boost (~100 mA).
	- 5v to 3.3v linear (~400 mA).

## TODO

- Investigate iron saturation effect on sensorless operation.
- Make a detailed documentation.
- Improve PGUI software.
- Add pulse output signal.
- Make a drawing of the heatsink case for `REV5`.
- Design the new hardware for 120v battery voltage.

