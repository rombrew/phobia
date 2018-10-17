/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_REGFILE_
#define _H_REGFILE_

#define REG_CONFIG_VERSION		18

enum {
	REG_CONFIG		= 1,
	REG_READ_ONLY		= 2,
	REG_LINKED		= 4
};

enum {
#include "regdefs.h"
};

typedef union {

	float		f;
	int		i;
}
reg_val_t;

typedef struct {

	const char		*sym;

	const char		*fmt;
	int			mode;

	reg_val_t		*link;

	void			(* proc) (const void *reg, void *lval, const void *rval);
}
reg_t;

extern const reg_t	regfile[];

void reg_getval(const reg_t *reg, void *lval);
void reg_setval(const reg_t *reg, const void *rval);
void reg_format_rval(const reg_t *reg, const void *rval);
void reg_format(const reg_t *reg);
const reg_t *reg_search(const char *sym);

void reg_GET(int n, void *lval);
void reg_SET(int n, const void *rval);

void reg_SET_F(int n, float rval);

#endif /* _H_REGFILE_ */

