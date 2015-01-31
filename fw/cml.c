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

#include "pmc.h"

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
	/*complex_t	Zxx, Zxy, Zyx, Zyy;
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
	Zyy.im = (pm->tUY.re * pm->tIY.im - pm->tUY.im * pm->tIY.re) / Ly;*/

	/*pm->R = pm->tA.re;
					pm->L = pm->tA.im / 2.f * KPI * pm->sFq;*/
}

pmc_t __CCM__			pm;

static void
xDC(int uA, int uB, int uC) { }

static void
xZ(int Z) { }

void pm_test(char *s)
{
	int		t0, t1;

	pm.hzF = 20000.f;
	pm.pwmR = 700;

	pm.pDC = &xDC;
	pm.pZ = &xZ;

	pm.U = 12.f;
	pm.R = 74e-3;
	pm.Ld = 44e-6;
	pm.Lq = pm.Ld;
	pm.E = 64e-4;

	pm.Zp = 11;
	pm.M = 0.f;
	pm.J = 10e-5;

	pmcEnable(&pm);

	pm.mBit = PMC_MODE_EKF_6X_BASE;

	t0 = halSysTick();
	pmcFeedBack(&pm, 2048, 2048, 0);
	t1 = halSysTick();

	printf("pmc %i" EOL, t0 - t1);
}

const shCMD_t		cmList[] = {

	{"uptime", &uptime},

	{"pm_impedance", &pm_impedance},
	{"pm_test", &pm_test},

	{NULL, NULL},
};

