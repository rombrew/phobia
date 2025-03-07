#ifndef _H_NTC_
#define _H_NTC_

/* Temperature measurement schematic (placement NTC on GND)
 *
	            +------< VCC
	            |
	            |
	           | |
	           | | R_balance
	           |_|
	            |               +---+
	            |              /    |
	            +--------+----- ADC |  U output
	            |        |     \    |
	            |        |      +---+
	           | |       |
	    R_ntc  |/|     -----
	           |_|     -----
	            |        |    C1
	            |        |
	            +--------+
	            |
	           ---
	           \ /  GND

*/

enum {
	NTC_NONE		= 0,
	NTC_ON_GND,
	NTC_ON_VCC,

	NTC_LMT87,
	NTC_KTY83_GND,
	NTC_KTY84_GND
};

typedef struct {

	int		type;
	int		gpio;

	float		balance;
	float		ntc0;
	float		ta0;
	float		betta;
}
ntc_t;

float ntc_read_temperature(ntc_t *ntc);

#endif /* _H_NTC_ */

