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

#include "teli.h"
#include "lib.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void teli_default(teli_t *ti)
{
	ti->in[0] = &regfile[ID_PM_FB_CURRENT_A];
	ti->in[1] = &regfile[ID_PM_FB_CURRENT_B];
	ti->in[2] = &regfile[ID_PM_LU_X_0_];
	ti->in[3] = &regfile[ID_PM_LU_X_1_];
	ti->in[4] = &regfile[ID_PM_LU_X_2_];
	ti->in[5] = &regfile[ID_PM_LU_X_3_];
	ti->in[6] = &regfile[ID_PM_LU_X_4__RPM];
	ti->in[7] = &regfile[ID_PM_FLUX_DRIFT_Q];
	ti->in[8] = &regfile[ID_PM_FLUX_RESIDUAL_LPF];
	ti->in[9] = &regfile[ID_PM_VSI_LPF_WATT];
}

void teli_capture(teli_t *ti)
{
	const reg_t		*reg;
	int			j;

	if (ti->mode != 0) {

		ti->i++;

		if (ti->i >= ti->d) {

			ti->i = 0;

			for (j = 0; j < TEL_INPUT_MAX; ++j) {

				reg = ti->in[j];

				if (reg != NULL) {

					reg_getval(reg, (void *) &ti->data[ti->n][j]);
				}
				else {
					ti->data[ti->n][j] = 0;
				}
			}

			ti->n++;

			if (ti->n >= TEL_DATA_MAX) {

				ti->mode = 0;
			}
		}
	}
}

void teli_enable(teli_t *ti, int freq)
{
	if (freq > 0 && freq <= hal.PWM_freq_hz) {

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

void teli_disable(teli_t *ti)
{
	ti->mode = 0;
}

void teli_flush(teli_t *ti)
{
	const reg_t		*reg;
	const char		*su;
	int			n, j;

	for (j = 0; j < TEL_INPUT_MAX; ++j) {

		reg = ti->in[j];

		if (reg != NULL) {

			puts(reg->sym);

			su = reg->sym + strlen(reg->sym) + 1;

			if (*su != 0) {

				printf("_[%s]", su);
			}

			puts(";");
		}
	}

	puts(EOL);

	for (n = 0; n < TEL_DATA_MAX; ++n) {

		for (j = 0; j < TEL_INPUT_MAX; ++j) {

			reg = ti->in[j];

			if (reg != NULL) {

				if (reg->fmt[1] == 'i') {

					printf(reg->fmt, (int) ti->data[n][j]);
				}
				else {
					printf(reg->fmt, (float *) &ti->data[n][j]);
				}

				puts(";");
			}
		}

		puts(EOL);
	}
}

SH_DEF(tel_default)
{
	teli_default(&ti);
}

SH_DEF(tel_reg_assign)
{
}

SH_DEF(tel_flush)
{
	teli_flush(&ti);
}

SH_DEF(tel_live)
{
	/*int		freq = 0;

	stoi(&freq, s);

	teli_enable(&ti, freq);
	teli_flush(&ti);*/
}

