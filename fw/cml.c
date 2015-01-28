/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#include "hal/hal.h"
#include "sh.h"
#include "task.h"
#include "lib.h"

void uptime(char *s)
{
	int		Day, Hour, Min, Sec;

	Sec = td.uTICK / 100;

	Day = Sec / 86400;
	Sec -= Day * 86400;
	Hour = Sec / 3600;
	Sec -= Hour * 3600;
	Min = Sec / 60;
	Sec -= Min * 60;

	printf("%id %ih %im %is" EOL,
			Day, Hour, Min, Sec);
}

void pm_impedance(char *s)
{
	complex_t	Zxx, Zxy, Zyx, Zyy;
	float		Lx, Ly;

	pm->IMP.UX = pm->sUX / pm->U;
	pm->IMP.UY = pm->sUY / pm->U;
	pm->IMP.AM = pm->sUAM / pm->U;
	pm->IMP.DF = 2.f * KPI * pm->sFq / pm->hzF;

	Lx = pm->tIX.re * pm->tIX.re + pm->tIX.im * pm->tIX.im;
	Ly = pm->tIY.re * pm->tIY.re + pm->tIY.im * pm->tIY.im;

	Zxx.re = (pm->tUX.re * pm->tIX.re + pm->tUX.im * pm->tIX.im) / Lx;
	Zxx.im = (pm->tUX.re * pm->tIX.im - pm->tUX.im * pm->tIX.re) / Lx;
	Zxy.re = (pm->tUX.re * pm->tIY.re + pm->tUX.im * pm->tIY.im) / Ly;
	Zxy.im = (pm->tUX.re * pm->tIY.im - pm->tUX.im * pm->tIY.re) / Ly;
	Zyx.re = (pm->tUY.re * pm->tIX.re + pm->tUY.im * pm->tIX.im) / Lx;
	Zyx.im = (pm->tUY.re * pm->tIX.im - pm->tUY.im * pm->tIX.re) / Lx;
	Zyy.re = (pm->tUY.re * pm->tIY.re + pm->tUY.im * pm->tIY.im) / Ly;
	Zyy.im = (pm->tUY.re * pm->tIY.im - pm->tUY.im * pm->tIY.re) / Ly;

	/*pm->R = pm->tA.re;
					pm->L = pm->tA.im / 2.f * KPI * pm->sFq;*/
}

void led(char *s)
{
	halLED(LED_BLUE);
}

void prt(char *s)
{
	int		x;

	halLED(LED_GREEN);
	x = 123456789;

	printf("1: long long string %i %i %i \r\n", x, x, x);
	printf("2: long long string %i %i %i \r\n", x, x, x);
	printf("3: long long string %i %i %i \r\n", x, x, x);
	printf("4: long long string %i %i %i \r\n", x, x, x);
	printf("5: long long string %i %i %i \r\n", x, x, x);
	printf("6: long long string %i %i %i \r\n", x, x, x);
	printf("7: long long string %i %i %i \r\n", x, x, x);
	printf("8: long long string %i %i %i \r\n", x, x, x);
	printf("9: long long string %i %i %i \r\n", x, x, x);
}

extern void shHistoryShow();

const shCMD_t		cmList[] = {

	{"uptime", &uptime},

	{"pm_impedance", &pm_impedance},

	{"led", &led},
	{"prt", &prt},

	{NULL, NULL},
};

