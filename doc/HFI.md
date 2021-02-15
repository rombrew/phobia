## Overview

This page describes the High Frequency Injection (HFI) method that provides a
position estimate even if speed is zero.

## Basics

The main parameters of the HFI is a frequency and swing. It is usually
sufficient that swing is greater than the noise of the current sensors. Large
injection swing will probably need to estimate flux polarity.

	# reg pm.hfi_freq_hz <hz>
	# reg pm.hfi_swing_D <amp>

In complicated cases you will need to tune observer gains.

	# reg pm.hfi_gain_EP <x>
	# reg pm.hfi_gain_SF <x>
	# reg pm.hfi_gain_FP <x>

## Flux polarity detection

