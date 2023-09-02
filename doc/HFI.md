## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate at low speed.

## Basics

HFI operation is based on inductance variance depending on the rotor position.
Inductance variance can be produced by salient pole machine design (IPM, SynRM)
or be saturation induced (BLDC). The first step is to specify `SALIENCY`
depending on machine type.

	(pmc) reg pm.config_SALIENCY 1

Note that non-salient machine implies `Ld = Lq`, negative saliency (BLDC)
implies `Ld < Lq`, positive saliency (IPM, SynRM) implies `Ld > Lq`.

You should check the inductance difference along the DQ axes after the machine
probing procedures. Inductances L1 and L2 should differ by more than ~10% to
get reliable HFI operation.

	(pmc) pm_probe_impedance

The second step is to select KALMAN sensorless estimation. This is the only
observer that can appreciate machine saliency.

	(pmc) reg pm.config_LU_ESTIMATE 2

Select injection waveform type. The usual type is SINE wave.

	(pmc) reg pm.config_HFI_WAVE 1

The main parameters of injection is a frequency and amplitude. It is usually
large frequency and amplitude makes HFI operation is more stable. Beware of
sine degeneracy at frequencies close to the PWM frequency.

	(pmc) reg pm.hfi_freq <Hz>
	(pmc) reg pm.hfi_sine <A>

If the machine loses its magnetic anisotropy at high current you can limit that
current on HFI operation mode.

	(pmc) reg pm.i_derate_on_HFI <A>

## Flux polarity detection

In HFI operation it is possible to detect only direction of the rotor flux axis
but not its orientation or flux polarity. But we can use iron saturation effect
to estimate flux polarity.

	(pmc) reg pm.config_HFI_POLARITY 1

TODO
