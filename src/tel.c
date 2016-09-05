/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#include "lib.h"
#include "m.h"
#include "main.h"
#include "pmc.h"
#include "sh.h"

#define	TELSZ			20000

typedef struct {

	int			iEN;

	short int		pIN[8];
	int			pSZ;

	int			sAVG[8];
	int			sCNT, sDEC;

	short int		pD[TELSZ];
	short int		*pZ;
}
tel_t;

static tel_t			tel;

static void
telCapture()
{
	int			j, SZ;

	for (j = 0; j < tel.pSZ; ++j)
		tel.sAVG[j] += tel.pIN[j];

	tel.sCNT++;

	if (tel.sCNT >= tel.sDEC) {

		for (j = 0; j < tel.pSZ; ++j) {

			*tel.pZ++ = tel.sAVG[j] / tel.sCNT;
			tel.sAVG[j] = 0;
		}

		SZ = tel.pZ - tel.pD + tel.pSZ;
		tel.iEN = (SZ <= TELSZ) ? tel.iEN : 0;

		tel.sCNT = 0;
	}
}

static void
telFlush()
{
	short int		*pZ, *pEnd;
	int			j;

	pZ = tel.pD;
	pEnd = tel.pZ;

	while (pZ < pEnd) {

		for (j = 0; j < tel.pSZ; ++j)
			printf("%i ", *pZ++);

		puts(EOL);
	}
}

static void
evTEL_0()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.lu_X[0] * 1000.f);
		tel.pIN[1] = (short int) (pm.lu_X[1] * 1000.f);
		tel.pIN[2] = (short int) (pm.lu_X[2] * 4096.f);
		tel.pIN[3] = (short int) (pm.lu_X[3] * 4096.f);
		tel.pIN[4] = (short int) (pm.lu_X[4] * 30.f / MPIF / pm.const_Zp);
		tel.pIN[5] = (short int) (pm.drift_Q * 1000.f);
		tel.pIN[6] = (short int) (pm.lu_residual_D * 1000.f);
		tel.pIN[7] = (short int) (pm.lu_residual_Q * 1000.f);
		tel.pSZ = 8;

		telCapture();
	}
	else
		ma.pEX = NULL;
}

static void
evTEL_1()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) (pm.fb_iA * 1000.f);
		tel.pIN[1] = (short int) (pm.fb_iB * 1000.f);
		tel.pSZ = 2;

		telCapture();
	}
	else
		ma.pEX = NULL;
}

static void
evTEL_2()
{
	if (tel.iEN) {

		tel.pIN[0] = (short int) 0;
		tel.pSZ = 1;

		telCapture();
	}
	else
		ma.pEX = NULL;
}

static void (* const evTEL_list[]) () = {

	&evTEL_0,
	&evTEL_1,
	&evTEL_2,
};

SH_DEF(tel_decimal)
{
	stoi(&tel.sDEC, s);
	printf("%i" EOL, tel.sDEC);
}

SH_DEF(tel_capture)
{
	const int	nMAX = sizeof(evTEL_list) / sizeof(evTEL_list[0]);
	void 		(* evTEL) ();
	int		nTEL = 0;

	if (ma.pEX == NULL) {

		tel.iEN = 1;
		tel.sCNT = 0;
		tel.pZ = tel.pD;

		stoi(&nTEL, s);
		nTEL = (nTEL < 0) ? 0 : (nTEL > nMAX) ? nMAX : nTEL;
		evTEL = evTEL_list[nTEL];

		halFence();
		ma.pEX = evTEL;
	}
}

SH_DEF(tel_disable)
{
	tel.iEN = 0;
	tel.sCNT = 0;
	tel.pZ = tel.pD;
}

void taskLIVE(void *pvParameters)
{
	do {
		if (tel.pZ != tel.pD) {

			telFlush();
			tel.pZ = tel.pD;
		}

		vTaskDelay(10);
	}
	while (1);
}

SH_DEF(tel_live)
{
	const int	nMAX = sizeof(evTEL_list) / sizeof(evTEL_list[0]);
	void 		(* evTEL) ();
	int		nTEL = 0;
	int		xC, decmin;
	TaskHandle_t	xLIVE;

	if (ma.pEX == NULL) {

		decmin = (int) (pm.freq_hz / 25.f + .5f);
		tel.sDEC = (tel.sDEC < decmin) ? decmin : tel.sDEC;

		tel.iEN = 1;
		tel.sCNT = 0;
		tel.pZ = tel.pD;

		stoi(&nTEL, s);
		nTEL = (nTEL < 0) ? 0 : (nTEL > nMAX) ? nMAX : nTEL;
		evTEL = evTEL_list[nTEL];

		halFence();
		ma.pEX = evTEL;

		xTaskCreate(taskLIVE, "tLIVE", configMINIMAL_STACK_SIZE, NULL, 1, &xLIVE);

		do {
			xC = iodef->getc();

			if (xC == 3 || xC == 4)
				break;
		}
		while (1);

		vTaskDelete(xLIVE);
		tel.iEN = 0;
	}
}

SH_DEF(tel_flush)
{
	telFlush();
}

SH_DEF(tel_info)
{
	float		freq, time;

	freq = pm.freq_hz / (float) tel.sDEC;
	time = TELSZ * (float) tel.sDEC / pm.freq_hz;

	printf(	"decimal %i" EOL
		"freq %1f (Hz)" EOL
		"time %3f (Sec)" EOL,
		tel.sDEC, &freq, &time);
}

