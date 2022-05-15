#ifndef _H_NTC_
#define _H_NTC_

/* NTC measurement schematic
 *
	            +------< vREF
	            |
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

enum {
	NTC_NONE		= 0,
	NTC_GND,
	NTC_VCC,
	NTC_LINEAR,
};

typedef struct {

	int		type;
	int		gpio;

	float		balance;
	float		ntc_0;
	float		ta_0;
	float		betta;
}
ntc_t;

float ntc_read_temperature(ntc_t *ntc);

#endif /* _H_NTC_ */

