## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate at low speed.

## Basics

HFI operation is based on inductance variance depending on the rotor position.
Inductance variance can be produced by salient pole machine design (IPMSM,
SynRM) or by magnetic saturation in the stator iron (BLDC outrunner). You can
check the inductance difference along the DQ axes on the motor probe
procedures. Inductances L1 and L2 should differ by more than 10% for reliable
HFI operation.

To setup HFI configuration you should select KALMAN sensorless estimation.

    (pmc) reg pm.config_LU_ESTIMATE 2

Select injection waveform type. The usual type is SINE wave.

    (pmc) reg pm.config_HFI_WAVE 1

The main parameters of injection is a frequency and amplitude. It is usually
large frequency and amplitude makes HFI operation is more stable. Beware of
sine degeneracy at frequencies close to the PWM frequency.

    (pmc) reg pm.hfi_freq <Hz>
	(pmc) reg pm.hfi_sine <A>

If the machine loses its magnetic saliency at high current you can limit that
current on HFI operation mode.

	(pmc) reg pm.i_derate_on_HFI <A>

## Flux polarity detection

In HFI operation it is possible to detect only direction of the rotor flux axis
but not its orientation or flux polarity. But we can use iron saturation effect
to estimate flux polarity.

    (pmc) reg pm.config_HFI_POLARITY 1

TODO
