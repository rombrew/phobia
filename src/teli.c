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
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void teli_reg_default(teli_t *ti)
{
	ti->in[0] = &regfile[ID_PM_FB_CURRENT_A];
	ti->in[1] = &regfile[ID_PM_FB_CURRENT_B];
	ti->in[2] = &regfile[ID_PM_LU_X_0];
	ti->in[3] = &regfile[ID_PM_LU_X_1];
	ti->in[4] = &regfile[ID_PM_LU_X_2];
	ti->in[5] = &regfile[ID_PM_LU_X_3];
	ti->in[6] = &regfile[ID_PM_LU_X_4_RPM];
	ti->in[7] = &regfile[ID_PM_FLUX_DRIFT_Q];
	ti->in[8] = &regfile[ID_PM_FLUX_RESIDUAL_LPF];
	ti->in[9] = &regfile[ID_PM_VSI_LPF_WATT];
}

void teli_reg_grab(teli_t *ti)
{
	union {
		float		f;
		int		i;
	} u;

	const reg_t		*reg;
	int			j;

	if (ti->mode != 0) {

		ti->i++;

		for (j = 0; j < TEL_INPUT_MAX; ++j) {

			reg = ti->in[j];

			if (reg != NULL) {

				if (ti->i == 1) {

					reg_getval(reg, (void *) &ti->data[ti->n][j]);
				}
				else {
					if (reg->fmt[1] != 'i') {

						/* Get average value if register is float.
						 * */
						reg_getval(reg, &u);
						ti->data[ti->n][j].f += u.f;
					}
				}
			}
		}

		if (ti->i >= ti->d) {

			ti->i = 0;

			for (j = 0; j < TEL_INPUT_MAX; ++j) {

				reg = ti->in[j];

				if (reg != NULL) {

					if (reg->fmt[1] != 'i') {

						ti->data[ti->n][j].f /= (float) ti->d;
					}

					if (ti->mode == TEL_MODE_LIVE) {

						ti->live[j] = ti->data[ti->n][j];
					}
				}
			}

			if (ti->mode == TEL_MODE_SINGLE_GRAB) {

				ti->n++;

				if (ti->n >= TEL_DATA_MAX) {

					ti->mode = 0;
				}
			}
			else if (ti->mode == TEL_MODE_LIVE) {

				ti->l = 1;
			}
		}
	}
}

void teli_startup(teli_t *ti, int freq, int mode)
{
	if (freq > 0 && freq <= hal.PWM_freq_hz) {

		ti->d = (hal.PWM_freq_hz + freq / 2) / freq;
	}
	else {
		ti->d = 1;
	}

	ti->i = 0;
	ti->n = 0;
	ti->l = 0;

	hal_fence();
	ti->mode = mode;
}

void teli_halt(teli_t *ti)
{
	ti->mode = 0;
}

SH_DEF(tel_reg_default)
{
	teli_reg_default(&ti);
}

static void
teli_reg_print(int n, const reg_t *reg)
{
	printf("[%i] = %s" EOL, n, (reg != NULL) ? reg->sym : "(null)");
}

SH_DEF(tel_reg_assign)
{
	const reg_t	*reg;
	int		n;

	if (stoi(&n, s) != NULL) {

		if (n >= 0 && n < TEL_INPUT_MAX) {

			reg = reg_search(sh_next_arg(s));
			ti.in[n] = reg;

			teli_reg_print(n, reg);
		}
	}
	else {
		for (n = 0; n < TEL_INPUT_MAX; ++n) {

			teli_reg_print(n, ti.in[n]);
		}
	}
}

SH_DEF(tel_grab)
{
	int		freq = 0;

	stoi(&freq, s);
	teli_startup(&ti, freq, TEL_MODE_SINGLE_GRAB);
}

SH_DEF(tel_halt)
{
	teli_halt(&ti);
}

void teli_reg_labels(teli_t *ti)
{
	const reg_t		*reg;
	const char		*su;
	int			j;

	for (j = 0; j < TEL_INPUT_MAX; ++j) {

		reg = ti->in[j];

		if (reg != NULL) {

			puts(reg->sym);

			su = reg->sym + strlen(reg->sym) + 1;

			if (*su != 0) {

				printf("@%s", su);
			}

			puts(";");
		}
	}

	puts(EOL);
}

void teli_reg_flush(teli_t *ti)
{
	const reg_t		*reg;
	int			n, D_MAX, j;

	D_MAX = (ti->mode == TEL_MODE_LIVE) ? 1 : TEL_DATA_MAX;

	for (n = 0; n < D_MAX; ++n) {

		for (j = 0; j < TEL_INPUT_MAX; ++j) {

			reg = ti->in[j];

			if (reg != NULL) {

				if (reg->fmt[1] == 'i') {

					if (ti->mode == TEL_MODE_LIVE) {

						printf(reg->fmt, ti->live[j].i);
					}
					else {
						printf(reg->fmt, ti->data[n][j].i);
					}
				}
				else {
					if (ti->mode == TEL_MODE_LIVE) {

						printf(reg->fmt, &ti->live[j].f);
					}
					else {
						printf(reg->fmt, &ti->data[n][j].f);
					}
				}

				puts(";");
			}
		}

		puts(EOL);
	}
}

SH_DEF(tel_flush)
{
	if (ti.mode == TEL_MODE_DISABLED) {

		teli_reg_labels(&ti);
		teli_reg_flush(&ti);
	}
}

void taskTELIVE(void *pData)
{
	teli_reg_labels(&ti);

	do {
		vTaskDelay((TickType_t) 5);

		if (ti.l != 0) {

			ti.l = 0;
			teli_reg_flush(&ti);
		}
	}
	while (1);
}

SH_DEF(tel_live)
{
	TaskHandle_t		xHandle;
	int			freq = 20;

	xTaskCreate(taskTELIVE, "tTELIVE", configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);

	if (stoi(&freq, s) != NULL) {

		freq = (freq < 1) ? 1 : (freq > 50) ? 50 : freq;
	}

	teli_startup(&ti, freq, TEL_MODE_LIVE);

	/* Block until we get any character.
	 * */
	iodef->getc();

	teli_halt(&ti);
	vTaskDelete(xHandle);
}

