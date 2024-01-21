#ifndef _H_TLM_
#define _H_TLM_

#include <stdint.h>

#include "regfile.h"

#define TLM_DATA_MAX		4000
#define TLM_INPUT_MAX		10

enum {
	TLM_MODE_DISABLED	= 0,
	TLM_MODE_GRAB,
	TLM_MODE_WATCH,
	TLM_MODE_LIVE
};

enum {
	TLM_TYPE_NONE		= 0,
	TLM_TYPE_FLOAT,
	TLM_TYPE_INT
};

typedef struct {

	float		grabfreq;
	float		livefreq;

	int		mode;
	int		reg_ID[TLM_INPUT_MAX];

	struct {

		int		type;
		const reg_t	*reg;
	}
	layout[TLM_INPUT_MAX];

	int		clock;
	int		count;

	int		span;
	int		line;

	/* The memory to keep telemetry data.
	 * */
	uint16_t	vm[TLM_DATA_MAX][TLM_INPUT_MAX];
}
tlm_t;

uint16_t tlm_fp_half(float x);
float tlm_fp_float(uint16_t x);

void tlm_reg_default(tlm_t *tlm);
void tlm_reg_grab(tlm_t *tlm);
void tlm_startup(tlm_t *tlm, float freq, int mode);
void tlm_halt(tlm_t *tlm);

#endif /* _H_TLM_ */

