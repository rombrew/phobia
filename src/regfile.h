/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#define REG_CONFIG_VERSION		7

enum {
	REG_INT			= 0x0001,
	REG_F_EXP		= 0x0002,
	REG_F_SHORT		= 0x0004,
	REG_I_HEX		= 0x0008,
	REG_CONFIG		= 0x0010,
	REG_READ_ONLY		= 0x0020,
};

typedef struct {

	const char		*sym;
	int			bits;

	union {

		float		*f;
		int		*i;
	}
	link;

	void			(* proc) (const void *reg, void *lval, const void *rval);
}
reg_t;

extern const reg_t	regfile[];

void reg_GET(int n, void *lval);
void reg_SET(int n, const void *rval);

void reg_print_fmt(const reg_t *reg);

#endif /* _H_REGFILE_ */

