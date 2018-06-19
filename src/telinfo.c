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
#include "hal/hal.h"

#include "telinfo.h"
#include "lib.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void telinfo_default(telinfo_t *ti)
{
	ti->in[0] = reg_search("pm.fb_iA");
	ti->in[1] = reg_search("pm.fb_iA");
	ti->in[2] = &regfile[5];
	ti->in[3] = &regfile[5];
}

void telinfo_capture(telinfo_t *ti)
{
	const reg_t		*reg;
	int			j;

	if (ti->mode != 0) {

		ti->i++;

		if (ti->i >= ti->d) {

			ti->i = 0;

			for (j = 0; j < TEL_INPUT_MAX; ++j) {

				reg = ti->in[j];

				//reg_getval(reg, );

				ti->data[ti->n][j] = (reg != NULL) ?
					* (unsigned long *) reg->link.i : 0;
			}

			ti->n++;

			if (ti->n >= TEL_DATA_MAX) {

				ti->mode = 0;
			}
		}
	}
}

void telinfo_enable(telinfo_t *ti, int freq)
{
	if (freq > 0) {

		ti->d = (hal.PWM_freq_hz + freq / 2) / freq;
	}
	else {
		ti->d = 1;
	}

	ti->i = 0;
	ti->n = 0;

	hal_fence();
	ti->mode = 1;
}

void telinfo_disable(telinfo_t *ti)
{
	ti->mode = 0;
}

void telinfo_flush(telinfo_t *ti)
{
	const reg_t		*reg;
	int			n, j;

	for (j = 0; j < TEL_INPUT_MAX; ++j) {

		reg = ti->in[j];

		if (reg != NULL) {

			printf("%s ", reg->sym);
		}
	}

	puts(EOL);

	for (n = 0; n < TEL_DATA_MAX; ++n) {

		for (j = 0; j < TEL_INPUT_MAX; ++j) {

			reg = ti->in[j];

			if (reg != NULL) {

				printf(reg->fmt, (float *) &ti->data[n][j]);
				puts(" ");
			}
		}

		puts(EOL);
	}
}

SH_DEF(ti_flush)
{
	telinfo_flush(&ti);
}

