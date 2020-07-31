#ifndef _H_NTC_
#define _H_NTC_

/*
	            +------< vREF
	            |                 // Temperature measurement schematic //
	            |
	           | |
	           | | R_balance 10K (1%)
	           |_|
	            |               +---+
	            |              /    |
	            +--------+----- ADC |  U output (from 0 to 1)
	            |        |     \    |
	            |        |      +---+
	           | |       |
	 R_ntc 10K |/|     -----
	           |_|     -----
	            |        |    C1 10nF
	            |        |
	            +--------+
	            |
	           ---
	           \ /  AGND

*/

typedef struct {

	float		r_balance;
	float		r_ntc_0;
	float		ta_0;
	float		betta;
}
ntc_t;

float ntc_temperature(ntc_t *ntc, float u);

#endif /* _H_NTC_ */

