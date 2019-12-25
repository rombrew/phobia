#include "ntc.h"
#include "phobia/libm.h"

float ntc_temperature(ntc_t *ntc, float u)
{
	float			r_ntc, log, temp;

	r_ntc = (u != 1.f) ? ntc->r_balance * u / (1.f - u) : 0.f;

	log = m_logf(r_ntc / ntc->r_ntc_0);
	temp = 1.f / (1.f / (ntc->ta_0 + 273.f) + log / ntc->betta) - 273.f;

	return temp;
}

