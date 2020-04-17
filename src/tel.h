#ifndef _H_TEL_
#define _H_TEL_

#include "regfile.h"

#define TEL_DATA_MAX		1000
#define TEL_INPUT_MAX		10

enum {
	TEL_MODE_DISABLED	= 0,
	TEL_MODE_SINGLE_GRAB,
	TEL_MODE_LIVE
};

typedef struct {

	int		mode;
	int		reg_ID[TEL_INPUT_MAX];

	reg_val_t	data[TEL_DATA_MAX][TEL_INPUT_MAX];

	int		tim, i, n;
}
tel_t;

void tel_reg_default(tel_t *ti);
void tel_reg_grab(tel_t *ti);
void tel_startup(tel_t *ti, int freq, int mode);
void tel_halt(tel_t *ti);

#endif /* _H_TEL_ */

