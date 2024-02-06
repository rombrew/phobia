#include <stddef.h>

#include "hal/hal.h"

#include "ntc.h"
#include "phobia/libm.h"

float ntc_read_temperature(ntc_t *ntc)
{
	float			um, ohm, log, temp;

	um = ADC_get_sample(ntc->gpio);

	switch (ntc->type) {

		case NTC_GND:

			ohm = um * ntc->balance / (1.f - um);
			log = m_logf(ohm / ntc->ntc0);
			temp = 1.f / (1.f / (ntc->ta0 + 273.f)
					+ log / ntc->betta) - 273.f;
			break;

		case NTC_VCC:

			ohm = (1.f - um) * ntc->balance / um;
			log = m_logf(ohm / ntc->ntc0);
			temp = 1.f / (1.f / (ntc->ta0 + 273.f)
					+ log / ntc->betta) - 273.f;
			break;

		case NTC_LMT87:

			temp = 194.1f - um * 242.7f;
			break;

		case NTC_KTY84:

			temp = 0.f;	/* TODO */
			break;

		default:
			temp = 0.f;
			break;
	}

	return temp;
}

