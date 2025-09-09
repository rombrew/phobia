## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate at low speed.

## Basics

HFI operation is based on inductance variance depending on the rotor position.
Inductance variance can be produced by salient pole machine design (IPM, SynRM)
or be saturation induced (BLDC). The first step is to specify the `SALIENCY`
depending on machine type.

	(pmc) reg pm.config_SALIENCY 1

`PM_SALIENCY_NONE`      - Non-salient machine implies               `Ld = Lq`.
`PM_SALIENCY_NEGATIVE`  - Negative saliency (BLDC) implies          `Ld < Lq`.
`PM_SALIENCY_POSITIVE`  - Positive saliency (IPM, SynRM) implies    `Ld > Lq`.

You should check the inductance difference along the DQ axes after the machine
probing procedures. Inductances Ld and Lq should differ by more than ~10% to
get reliable HFI operation.

	(pmc) pm_probe_impedance

The second step is to select `KALMAN` sensorless estimation. This is the only
observer that can appreciate machine saliency.

	(pmc) reg pm.config_LU_ESTIMATE 2

Select injection waveform type. The usual type is `SINE` waveform.

	(pmc) reg pm.config_HFI_WAVETYPE 1

The main parameters of injection is a frequency and amplitude. Usually a high
frequency and large amplitude makes HFI operation is more stable. Beware of
sine degeneracy at frequencies close to half of PWM frequency.

	(pmc) reg pm.hfi_freq <Hz>
	(pmc) reg pm.hfi_amplitude <A>

If the machine loses its magnetic anisotropy at high current you can limit
machine current in case of HFI operation mode. This gives reliable operation at
the cost of reduced torque production.

	(pmc) reg pm.hfi_maximal <A>

Note on HFI operation it is possible to detect only direction of the rotor flux
axis but not its orientation or flux polarity. So we bootstrap from forced
alignment or flux observer estimate.

## Advanced waveforms

Consider using one of the advanced waveform types instead of `SINE`.

`PM_HFI_NONE`   - No HF injection.
`PM_HFI_SINE`   - Sinusoidal wavefore with configurable frequency.
`PM_HFI_SILENT` - Non audible waveform at maximum frequency.
`PM_HFI_RANDOM` - Random sequence with configurable frequency.

If you are concerned about acoustic noise and vibration you can select `SILENT`
waveform and adjust an amplitude. In this case injection frequency is not
configurable and always at maximum (half of PWM frequency).

	(pmc) reg pm.config_HFI_WAVETYPE 2

Also try `RANDOM` waveform that can provide more reliable estimate at high
machine current. In this case better to use extremely high injection frequency.

	(pmc) reg pm.config_HFI_WAVETYPE 3
	(pmc) reg pm.hfi_freq 15000

## Permanent injection

In case of salient pole machine (IPM, SynRM) with MTPA control could be useful
to enable permanent HF injection.

	(pmc) reg pm.config_HFI_PERMANENT 1

