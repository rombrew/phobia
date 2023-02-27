#ifndef _H_TLM_
#define _H_TLM_

#include "regfile.h"

#define TLM_DATA_MAX		1000
#define TLM_INPUT_MAX		10

enum {
	TLM_MODE_DISABLED	= 0,
	TLM_MODE_SINGLE_GRAB,
	TLM_MODE_LIVE
};

typedef struct {

	int		freq_grab_hz;
	int		freq_live_hz;

	int		mode;
	int		reg_ID[TLM_INPUT_MAX];

	reg_val_t	data[TLM_DATA_MAX][TLM_INPUT_MAX];

	int		span;
	int		clock;

	int		S, N;
}
tlm_t;

void tlm_reg_default(tlm_t *tlm);
void tlm_reg_grab(tlm_t *tlm);
void tlm_startup(tlm_t *tlm, int freq, int mode);
void tlm_halt(tlm_t *tlm);

#endif /* _H_TLM_ */

