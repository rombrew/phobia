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

	int		mode;
	int		reg_ID[TLM_INPUT_MAX];

	reg_val_t	data[TLM_DATA_MAX][TLM_INPUT_MAX];

	int		tim, i, n;
}
TLM_t;

void TLM_reg_default(TLM_t *tlm);
void TLM_reg_grab(TLM_t *tlm);
void TLM_startup(TLM_t *tlm, int freq, int mode);
void TLM_halt(TLM_t *tlm);

#endif /* _H_TLM_ */

