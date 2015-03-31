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
#include "pmc.h"
#include "lib.h"

void uptime(char *s)
{
	int		Day, Hour, Min, Sec;

	Sec = td.uSEC;

	Day = Sec / 86400;
	Sec -= Day * 86400;
	Hour = Sec / 3600;
	Sec -= Hour * 3600;
	Min = Sec / 60;
	Sec -= Min * 60;

	printf("%id %ih %im %is" EOL,
			Day, Hour, Min, Sec);
}

void irqload(char *s)
{
	int			Tirq, Tbase, Rpc;

	Tbase = 2 * halPWM.R;
	Tirq = td.Tirq;
	Rpc = 100 * Tirq / Tbase;

	printf("%i%% (%i/%i)" EOL, Rpc, Tirq, Tbase);
}

void pwm_freq_hz(char *s)
{
	printf("%i%" EOL, halPWM.hzF);
}

void pwm_Tdt_ns(char *s)
{
	printf("%i%" EOL, halPWM.nsD);
}

void pm_req_spinup(char *s)
{
	if (pm.mReq == PMC_REQ_NULL) {

		pm.mReq = PMC_REQ_SPINUP;
	}
}

void pm_req_break(char *s)
{
}

void pm_req_sine(char *s)
{
}

void pm_req_linear(char *s)
{
}

const shCMD_t		cmList[] = {

	{"uptime", &uptime},
	{"irqload", &irqload},

	{"pm_req_spinup", &pm_req_spinup},
	{"pm_req_break", &pm_req_break},
	{"pm_req_sine", &pm_req_sine},
	{"pm_req_linear", &pm_req_linear},

	{NULL, NULL},
};

