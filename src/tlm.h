#ifndef _H_TLM_
#define _H_TLM_

#include <stdint.h>

#define TLM_DATA_MAX		5000
#define TLM_INPUT_MAX		10

enum {
	TLM_MODE_DISABLED	= 0,
	TLM_MODE_GRAB,
	TLM_MODE_WATCH,
	TLM_MODE_LIVE
};

typedef struct {

	int		grabfreq;
	int		livefreq;

	int		mode;
	int		reg_ID[TLM_INPUT_MAX];

	int		span;
	int		clock;

	int		skip;
	int		line;

	/* The memory to keep telemetry.
	 * */
	uint16_t	vm[TLM_DATA_MAX][TLM_INPUT_MAX];
}
tlm_t;

void tlm_reg_default(tlm_t *tlm);
void tlm_reg_grab(tlm_t *tlm);
void tlm_startup(tlm_t *tlm, int freq, int mode);
void tlm_halt(tlm_t *tlm);

#endif /* _H_TLM_ */

