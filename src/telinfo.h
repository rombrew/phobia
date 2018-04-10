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

typedef struct {

	int		en;

	const reg_t	*in[TEL_INPUT_MAX];

	int		d;
	int		i;

	unsigned long	data[TEL_DATA_MAX][TEL_INPUT_MAX];
	int		n;
}	
telinfo_t;

void telinfo_default(telinfo_t *ti);
void telinfo_capture(telinfo_t *ti);

#endif /* _H_TELINFO_ */

