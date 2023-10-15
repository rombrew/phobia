# Phobia Motor Controller

PMC is an open project that aims to build the quality permanent magnet
synchronous machine (PMSM) controller for use in a variety of scopes like RC or
electric transport.

## Software features

* Sensorless vector control of PMSM by measurement of currents and voltages.
* Robust ORTEGA observer with gain scheduling against speed.
* Accurate KALMAN observer having convergence at HF injection.
* Flux weakening and MTPA control (**EXPERIMENTAL**).
* Two and three phase machine support.
* Hardware abstraction layer (HAL) over STM32F4 and STM32F7.
* Various controller boards are supported (including VESC clones).
* Regular Command Line Interface (CLI) with autocompletion and history.
* Graphical front-end software based on
  [Nuklear](https://github.com/Immediate-Mode-UI/Nuklear) and
  [SDL2](https://www.libsdl.org/).
* Non time-critical tasks are managed by
  [FreeRTOS](http://www.freertos.org/).
* USB protocol stack from
  [CherryUSB](https://github.com/sakumisu/CherryUSB).
* Least Squares estimate library
  [LSE](https://github.com/rombrew/lse).

* Phase current sampling schemes includes two or three sensors configuration
  with inline or low-side placement.
* Self-adjustment of all onboard measurements (current and voltage) along
  symmetrical channels.

* Advanced SVPWM scheme provides:
	* Reduced switching losses and fully utilised DC link voltage.
	* Reduced voltage distortion for precise control.
	* Voltage hopping to get accurate ADC measurements with inline current sensors.
	* Prevent bootstrap circuit undervoltage condition.

* Terminal voltage measurements (TVM):
	* Compensation of the voltage distortion caused by Dead-Time insertion.
	* Back EMF voltage tracking to catch an already running machine.
	* Self-test of the power stages integrity and machine connection.
	* Self-test of bootstrap retention time.

* Automated machine parameters identification with no additional tools:
	* Stator DC resistance (Rs).
	* Stator AC impedance in DQ frame (L1, L2, R).
	* Rotor flux linkage constant (lambda).
	* Mechanical moment of inertia (Ja).
	* Discrete Hall signals recognition.

* Operation at low or zero speed:
	* Forced control that applies a current vector without feedback to
	  force rotor hold or spinup.
	* Freewheeling.
	* High Frequency Injection (HFI) based on magnetic saliency.
	* Discrete Hall sensors.
	* AB incremental encoder (EABI).
	* Absolute position sensor with SPI interface (**TODO**).
	* Analog Hall sensors and resolver decoder (**TODO**).

* Control loops:
	* Current control is always enabled.
	* Speed control loop.
	* Location control loop.

* Adjustable constraints:
	* Phase current with derate on PCB overheat.
	* Motor voltage applied from VSI.
	* DC link current consumption and regeneration.
	* DC link overvoltage and undervoltage.
	* Maximal speed and acceleration.

* Input interfaces:
	* Analog input knob with brake signal.
	* RC servo pulse width.
	* CAN bus flexible configurable data pipes.
	* STEP/DIR interface (**EXPERIMENTAL**).
	* Manual control through CLI or graphical front-end.
	* Custom embedded application can implement any control strategy.

* Advanced CAN networking:
	* Up to 30 nodes in peer network.
	* Network survey on request (no heartbeat messages).
	* Automated node address assignment.
	* IO forwarding to log in to the remote node CLI.
	* Flexible configurable data pipes.

* Available information:
	* Total distance traveled.
	* Battery energy (Wh) and charge (Ah) consumed.
	* Fuel gauge percentage.

## Hardware specification (**REV5A**)

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
	* Two current shunts (0.5 mOhm) with amplifiers (AD8418) give a
	  measuring range of 165A.
	* Battery voltage from 0 to 60v.
	* Three terminal voltages from 0 to 60v.
	* Temperature of PCB with NTC resistor.

* Motor interfaces:
	* Discrete Hall sensors or EABI encoder (5v pull-up).
	* External NTC resistor (e.g. machine temperature sensing).

* Control interfaces:
	* CAN transceiver with optional termination resistor on PCB (5v).
	* USART to bootload and configure (3.3v).
	* Pulse input: RC servo, STEP/DIR, backup EABI (5v pull-up).
	* Two analog input channels (from 0 to 6v).

* Auxiliary interfaces:
	* Combined port: SPI, ADC, DAC, GPIO (3.3v).
	* BOOT pin combined with SWDIO to use embedded bootloader.
	* SWD to get hardware debug.
	* External FAN control (5v, ~0.5A).

* Power conversion:
	* Battery voltage to 5v buck (~1A).
	* 5v to 12v boost (~100 mA).
	* 5v to 3.3v linear (~400 mA).

## Current Status

Now we can declare that PMC is ready to use in most applications. But it may be
difficult to configure the PMC for some types of machine.

There are a few videos about PMC on [youtube](https://www.youtube.com/@romblv).

Read further in [doc/GettingStarted](doc/GettingStarted.md).

## TODO

* Make a detailed documentation.
* Add pulse output signal.
* Make a drawing of the heatsink case for REV5A.
* Design the new hardware for 120v battery voltage.

