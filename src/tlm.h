#ifndef _H_TLM_
#define _H_TLM_

#include <stdint.h>

#include "regfile.h"

#define TLM_DATA_MAX		22500
#define TLM_INPUT_MAX		20

enum {
	TLM_MODE_DISABLED	= 0,
	TLM_MODE_GRAB,
	TLM_MODE_WATCH,
	TLM_MODE_LIVE
};

typedef struct {

	int		rate_grab;
	int		rate_watch;
	int		rate_live;

	int		mode;
	int		reg_ID[TLM_INPUT_MAX];

	const reg_t	*layout_reg[TLM_INPUT_MAX];

	int		layout_N;
	int		length_MAX;

	int		clock;
	int		skip;

	int		rate;
	int		line;

	rval_t		rdata[TLM_DATA_MAX];	/* memory to keep telemetry data */
}
tlm_t;

void tlm_reg_default(tlm_t *tlm);
void tlm_reg_grab(tlm_t *tlm);
void tlm_startup(tlm_t *tlm, int rate, int mode);
void tlm_halt(tlm_t *tlm);

#endif /* _H_TLM_ */

