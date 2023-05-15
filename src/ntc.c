#include <stddef.h>

#include "hal/hal.h"

#include "ntc.h"
#include "phobia/libm.h"

float ntc_read_temperature(ntc_t *ntc)
{
	float			um, ohm, log;
	float			temp = 0.f;

	switch (ntc->type) {

		case NTC_GND:

			um = ADC_get_sample(ntc->gpio);
			ohm = um * ntc->balance / (1.f - um);
			log = m_logf(ohm / ntc->ntc0);
			temp = 1.f / (1.f / (ntc->ta0 + 273.f)
					+ log / ntc->betta) - 273.f;
			break;

		case NTC_VCC:

			um = ADC_get_sample(ntc->gpio);
			ohm = (1.f - um) * ntc->balance / um;
			log = m_logf(ohm / ntc->ntc0);
			temp = 1.f / (1.f / (ntc->ta0 + 273.f)
					+ log / ntc->betta) - 273.f;
			break;

		case NTC_CMOS:

			um = ADC_get_sample(ntc->gpio);
			temp = um * ntc->betta + ntc->ta0;
			break;

		default:
			break;
	}

	return temp;
}

