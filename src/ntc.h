#ifndef _H_NTC_
#define _H_NTC_

typedef struct {

	float		r_balance;
	float		r_ntc_0;
	float		ta_0;
	float		betta;
}
ntc_t;

float ntc_temperature(ntc_t *ntc, float u);

#endif /* _H_NTC_ */

