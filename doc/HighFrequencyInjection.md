## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate at low speed.

## Basics

HFI operation is based on inductance variance depending on the rotor position.
Inductance variance can be produced by salient pole machine design (IPM, SynRM)
or be saturation induced (BLDC). The first step is to specify the `SALIENCY`
depending on machine type.

	(pmc) reg pm.config_SALIENCY 1

Note that non-salient machine implies `Ld = Lq`, negative saliency (BLDC)
implies `Ld < Lq`, positive saliency (IPM, SynRM) implies `Ld > Lq`.

You should check the inductance difference along the DQ axes after the machine
probing procedures. Inductances Ld and Lq should differ by more than ~10% to
get reliable HFI operation.

	(pmc) pm_probe_impedance

The second step is to select KALMAN sensorless estimation. This is the only
observer that can appreciate machine saliency.

	(pmc) reg pm.config_LU_ESTIMATE 2

Select injection waveform type. The usual type is SINE waveform.

	(pmc) reg pm.config_HFI_WAVETYPE 1

The main parameters of injection is a frequency and amplitude. It is usually a
high frequency and amplitude makes HFI operation is more stable. Beware of sine
degeneracy at frequencies close to the PWM frequency.

	(pmc) reg pm.hfi_freq <Hz>
	(pmc) reg pm.hfi_sine <A>

If the machine loses its magnetic anisotropy at high current you can limit
machine current in case of HFI operation mode. This gives reliable operation at
the cost of reduced torque production.

	(pmc) reg pm.i_maximal_on_HFI <A>

If you are concerned about increased levels of acoustic noise and vibration you
can select RANDOM waveform and adjust an amplitude.

	(pmc) reg pm.config_HFI_WAVETYPE 2

Note that in HFI operation it is possible to detect only direction of the rotor
flux axis but not its orientation or flux polarity.

## Permanent injection

In case of salient pole machine (IPM, SynRM) with MTPA control could be useful
to enable permanent HF injection.

	(pmc) reg pm.config_HFI_PERMANENT 1

