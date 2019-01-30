#ifndef _H_TELINFO_
#define _H_TELINFO_

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

	int		d;
	int		i;

	reg_val_t	data[TEL_DATA_MAX][TEL_INPUT_MAX];

	int		n;
}
teli_t;

void teli_reg_default(teli_t *ti);
void teli_reg_grab(teli_t *ti);
void teli_startup(teli_t *ti, int freq, int mode);
void teli_halt(teli_t *ti);

#endif /* _H_TELINFO_ */

