#include <stddef.h>

#include "hal/hal.h"

#include "ntc.h"
#include "phobia/libm.h"

float ntc_read_temperature(ntc_t *ntc)
{
	float			um, ohm, temp;

	um = ADC_get_sample(ntc->gpio) / hal.ADC_reference_voltage;

	switch (ntc->type) {

		case NTC_GND:

			ohm = um * ntc->balance / (1.f - um);
			temp = ntc->betta / (ntc->betta / (ntc->ta0 + 273.f)
					+ m_logf(ohm / ntc->ntc0)) - 273.f;
			break;

		case NTC_VCC:

			ohm = (1.f - um) * ntc->balance / um;
			temp = ntc->betta / (ntc->betta / (ntc->ta0 + 273.f)
					+ m_logf(ohm / ntc->ntc0)) - 273.f;
			break;

		case NTC_LMT87:

			ohm = um * hal.ADC_reference_voltage;

			temp = -1.7805429f;
			temp = -65.912105f + temp * ohm;
			temp =  185.81927f + temp * ohm;
			break;

		case NTC_KTY83:

			ohm = um * ntc->balance / (1.f - um);

			temp =  1.0572638E-8f;
			temp = -7.1592662E-5f + temp * ohm;
			temp =  2.4608573E-1f + temp * ohm;
			temp = -1.6020638E+2f + temp * ohm;
			break;

		case NTC_KTY84:

			ohm = um * ntc->balance / (1.f - um);

			temp =  1.7536720E-8f;
			temp = -1.0947810E-4f + temp * ohm;
			temp =  3.3583154E-1f + temp * ohm;
			temp = -1.4331780E+2f + temp * ohm;
			break;

		default:
			temp = 0.f;
			break;
	}

	return temp;
}

