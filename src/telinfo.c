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

#include <stddef.h>

#include "freertos/FreeRTOS.h"

#include "lib.h"
#include "main.h"
#include "shell.h"
#include "telinfo.h"

void telinfo_default(telinfo_t *ti)
{
	ti->in[0] = &regfile[5];
	ti->in[1] = &regfile[5];
}

void telinfo_capture(telinfo_t *ti)
{
	const reg_t		*reg;
	int			j;

	ti->i = (ti->en != 0) ? ti->i + 1 : 0;

	if (ti->i >= ti->d) {

		ti->i = 0;

		for (j = 0; j < TEL_INPUT_MAX; ++j) {

			reg = ti->in[j];
			ti->data[ti->n][j] = (reg != NULL) ?
				* (unsigned long *) reg->link.i : 0;
		}

		ti->n = (ti->n < (TEL_DATA_MAX - 1)) ? ti->n + 1 : 0;
	}
}

SH_DEF(ti_flush)
{
}

