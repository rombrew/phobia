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

#include <stddef.h>

#include "hal/hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal_task.h"
#include "lib.h"
#include "pm_control.h"
#include "pm_math.h"
#include "shell.h"

#define	TEL_SIZE		1024
#define TEL_MAX			16

typedef struct {

	int			enabled;

	short int		fifo[TEL_SIZE][TEL_MAX];
	int			fifo_N;

	int			decimal;
	int			tick;
}
telemetry_t;

static telemetry_t		tel;

void tel_bare_enabled(int en)
{
	tel.enabled = en;
}

static void
tel_fifo_put()
{
	short int		*m;

	m = tel.fifo[tel.fifo_N];
	tel.fifo_N = (tel.fifo_N < (TEL_SIZE - 1)) ? tel.fifo_N + 1 : 0;

	m[0] = (short int) (pm.lu_X[0] * 1000.f);
	m[1] = (short int) (pm.lu_X[1] * 1000.f);
	m[2] = (short int) (pm.lu_X[2] * 4096.f);
	m[3] = (short int) (pm.lu_X[3] * 4096.f);
	m[4] = (short int) (pm.lu_X[4] * 30.f / M_PI_F / pm.const_Zp);
	m[5] = (short int) (pm.drift_Q * 1000.f);
	m[6] = (short int) (pm.lu_residual_D * 1000.f);
	m[7] = (short int) (pm.lu_residual_Q * 1000.f);
	m[8] = (short int) 0;
}

void tel_collect()
{
	if (pm.lu_region == PMC_LU_DISABLED) {

		tel.enabled = 0;
	}

	if (tel.enabled != 0) {

		tel.tick++;

		if (tel.tick < tel.decimal) ;
		else {

			tel.tick = 0;
			tel_fifo_put();
		}
	}
}

SH_DEF(tel_enabled)
{
	stoi(&tel.enabled, s);
	printf("%i" EOL, tel.enabled);
}

SH_DEF(tel_decimal)
{
	stoi(&tel.decimal, s);
	printf("%i" EOL, tel.decimal);
}

static void
taskLIVE(void *pData)
{
	short int 	*m;
	int		j;

	do {
		if (tel.fifo_N != 0) {

			tel.fifo_N = 0;
			m = tel.fifo[tel.fifo_N];

			for (j = 0; j < TEL_MAX; ++j)
				printf("%i ", m[j]);

			puts(EOL);
		}

		vTaskDelay(10);
	}
	while (1);
}

SH_DEF(tel_live)
{
	TaskHandle_t	xLIVE;
	int		enval, decval, min;
	int		xC;

	enval = tel.enabled;
	decval = tel.decimal;

	min = (int) (pm.freq_hz / 50.f + .5f);
	tel.decimal = (tel.decimal < min) ? min : tel.decimal;
	tel.fifo_N = 0;

	halFence();

	tel.enabled = 1;

	xTaskCreate(taskLIVE, "tLIVE", configMINIMAL_STACK_SIZE, NULL, 1, &xLIVE);

	do {
		xC = iodef->getc();

		if (xC == 3 || xC == 4)
			break;
	}
	while (1);

	vTaskDelete(xLIVE);

	tel.decimal = decval;
	tel.enabled = enval;
}

SH_DEF(tel_flush)
{
	short int		*m;
	int			j, fifo_j, enval;

	enval = tel.enabled;
	tel.enabled = 0;

	halFence();

	fifo_j = (tel.fifo_N < (TEL_SIZE - 1)) ? tel.fifo_N + 1 : 0;

	while (fifo_j != tel.fifo_N) {

		m = tel.fifo[fifo_j];
		fifo_j = (fifo_j < (TEL_SIZE - 1)) ? fifo_j + 1 : 0;

		for (j = 0; j < TEL_MAX; ++j)
			printf("%i ", m[j]);

		puts(EOL);
	}

	tel.enabled = enval;
}

