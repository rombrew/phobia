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

	const reg_t	*in[TEL_INPUT_MAX];

	int		d;
	int		i;

	union {
		float		f;
		int		i;
	}
	data[TEL_DATA_MAX][TEL_INPUT_MAX], live[TEL_INPUT_MAX];

	int		n;
	int		l;
}
teli_t;

void teli_reg_default(teli_t *ti);
void teli_reg_grab(teli_t *ti);
void teli_startup(teli_t *ti, int freq, int mode);
void teli_halt(teli_t *ti);

#endif /* _H_TELINFO_ */

