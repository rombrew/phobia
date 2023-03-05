## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate even if speed is zero.

## Basics

The main parameters of injection is a frequency and amplitude. It is usually
important that amplitude is much higher than the noise of the current sensors.
Also a large injection amplitude will probably be need to estimate flux
polarity.

	(pmc) reg pm.hfi_sine <amp>

The HFI frequency is specified by divider of the main PWM frequency. Reasonable
values are from 6 to 16.

	(pmc) reg pm.hfi_INJS <x>

These numbers are specified how much injection cycles will be skipped and
grabbed to calculate position estimate.

	(pmc) reg pm.hfi_SKIP <x>
	(pmc) reg pm.hfi_ESTI <x>

## Flux polarity detection

