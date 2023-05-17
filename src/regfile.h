#ifndef _H_REGFILE_
#define _H_REGFILE_

enum {
	REG_CONFIG		= 1U,
	REG_READ_ONLY		= 2U,
	REG_LINKED		= 4U
};

enum {
#include "regdefs.h"
};

typedef union {

	float		f;
	int		i;
}
reg_value_t;

typedef struct {

	const char		*sym;
	const char		fmt[4];

	int			mode;
	reg_value_t		*link;

	void	(* proc) (const void *reg, reg_value_t *lval, const reg_value_t *rval);
	void	(* format) (const void *reg);
}
reg_t;

extern const reg_t	regfile[];

void reg_format_rval(const reg_t *reg, const reg_value_t *rval);
void reg_format(const reg_t *reg);

const reg_t *reg_search(const char *sym);
const reg_t *reg_search_fuzzy(const char *sym);

void reg_GET(int reg_ID, reg_value_t *lval);
void reg_SET(int reg_ID, const reg_value_t *rval);

int reg_GET_I(int reg_ID);
float reg_GET_F(int reg_ID);

void reg_SET_I(int reg_ID, int x);
void reg_SET_F(int reg_ID, float x);

void reg_TOUCH_I(int reg_ID);

#endif /* _H_REGFILE_ */

