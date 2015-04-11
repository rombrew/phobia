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
#include "lib.h"
#include "sh.h"
#include "pmc.h"
#include "task.h"
#include "tel.h"

void uptime(const char *s)
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

void irqload(const char *s)
{
	int			Tirq, Tbase, Rpc;

	Tbase = 2 * halPWM.R;
	Tirq = td.Tirq;
	Rpc = 100 * Tirq / Tbase;

	printf("%i%% (%i/%i)" EOL, Rpc, Tirq, Tbase);
}

void reboot(const char *s)
{
	halReset();
}

void pwm_freq_hz(const char *s)
{
	stoi(&halPWM.hzF, s);
	printf("%i (Hz)" EOL, halPWM.hzF);
}

void pwm_Tdt_ns(const char *s)
{
	stoi(&halPWM.nsD, s);
	printf("%i (ns)" EOL, halPWM.nsD);
}

void pwm_R(const char *s)
{
	printf("%i" EOL, halPWM.R);
}

void pm_Tdrift(const char *s)
{
	stof(&pm.Tdrift, s);
	printf("%3f (Sec)" EOL, &pm.Tdrift);
}

void pm_Thold(const char *s)
{
	stof(&pm.Thold, s);
	printf("%3f (Sec)" EOL, &pm.Thold);
}

void pm_Tsample(const char *s)
{
	stof(&pm.Tsample, s);
	printf("%3f (Sec)" EOL, &pm.Tsample);
}

void pm_Tout(const char *s)
{
	stof(&pm.Tout, s);
	printf("%3f (Sec)" EOL, &pm.Tout);
}

void pm_iHOLD(const char *s)
{
	stof(&pm.iHOLD, s);
	printf("%3f (A)" EOL, &pm.iHOLD);
}

void pm_iSINE(const char *s)
{
	stof(&pm.iSINE, s);
	printf("%3f (A)" EOL, &pm.iSINE);
}

void pm_sineF(const char *s)
{
	stof(&pm.sineF, s);
	printf("%1f (Hz)" EOL, &pm.sineF);
}

void pm_iSPD(const char *s)
{
	stof(&pm.iSPD, s);
	printf("%3f (A)" EOL, &pm.iSPD);
}

void pm_iSPQ(const char *s)
{
	stof(&pm.iSPQ, s);
	printf("%3f (A)" EOL, &pm.iSPQ);
}

void pm_wSP(const char *s)
{
	stof(&pm.wSP, s);
	printf("%3f ()" EOL, &pm.wSP);
}

void pm_drift(const char *s)
{
	printf("Ad %4e (A)" EOL, &pm.Ad);
	printf("Bd %4e (A)" EOL, &pm.Bd);
	printf("Qd %4e (V)" EOL, &pm.Qd);
}

void pm_U(const char *s)
{
	stof(&pm.U, s);
	printf("%3f (V)" EOL, &pm.U);
}

void pm_E_wb(const char *s)
{
	float		Kv;

	stof(&pm.E, s);
	Kv = 5.513289f / (pm.E * pm.Zp);

	printf("E %4e (Wb) Kv %1f (RPM/V)" EOL, &pm.E, &Kv);
}

void pm_E_kv(const char *s)
{
	float		Kv;

	if (stof(&Kv, s) != NULL)
		pm.E = 5.513289f / (Kv * pm.Zp);
	else
		Kv = 5.513289f / (pm.E * pm.Zp);

	printf("E %4e (Wb) Kv %1f (RPM/V)" EOL, &pm.E, &Kv);
}

void pm_R(const char *s)
{
	stof(&pm.R, s);
	printf("%4e (Ohm)" EOL, &pm.R);
}

void pm_Ld(const char *s)
{
	stof(&pm.Ld, s);
	printf("%4e (H)" EOL, &pm.Ld);
}

void pm_Lq(const char *s)
{
	stof(&pm.Lq, s);
	printf("%4e (H)" EOL, &pm.Lq);
}

void pm_Zp(const char *s)
{
	stoi(&pm.Zp, s);
	printf("%i" EOL, pm.Zp);
}

void pm_M(const char *s)
{
	stof(&pm.M, s);
	printf("M %4e (Nm)" EOL, &pm.M);
}

void pm_J(const char *s)
{
	float		J;

	if (stof(&J, s) != NULL)
		pm.IJ = 1.f / J;
	else
		J = 1.f / pm.IJ;

	printf("J %4e (kgm2)" EOL, &J);
}

void pm_const(const char *s)
{
	float		Kv, J;

	Kv = 5.513289f / (pm.E * pm.Zp);
	J = 1.f / pm.IJ;

	printf("U %3f (V)" EOL, &pm.U);
	printf("E %4e (Wb) Kv %1f (RPM/V)" EOL, &pm.E, &Kv);
	printf("R %4e (Ohm)" EOL, &pm.R);
	printf("Ld %4e (H)" EOL, &pm.Ld);
	printf("Lq %4e (H)" EOL, &pm.Lq);

	printf("Zp %i" EOL, pm.Zp);
	printf("M %4e (Nm)" EOL, &pm.M);
	printf("J %4e (kgm2)" EOL, &J);
}

void pm_req_spinup(const char *s)
{
	if (pm.mReq == PMC_REQ_NULL) {

		pm.mReq = PMC_REQ_SPINUP;
	}
}

void pm_req_break(const char *s)
{
}

void pm_req_sine(const char *s)
{
	if (pm.mReq == PMC_REQ_NULL) {

		pm.mReq = PMC_REQ_SINE;
	}
}

void pm_req_linear(const char *s)
{
}

void tel_enable(const char *s)
{
	tel.num = 0;
	tel.en = 1;
}

void tel_show(const char *s)
{
	telShow();
}

const shCMD_t		cmList[] = {

	{"uptime", &uptime},
	{"irqload", &irqload},
	{"reboot", &reboot},

	{"pwm_freq_hz", &pwm_freq_hz},
	{"pwm_Tdt_ns", &pwm_Tdt_ns},
	{"pwm_R", &pwm_R},

	{"pm_Tdrift", &pm_Tdrift},
	{"pm_Thold", &pm_Thold},
	{"pm_Tsample", &pm_Tsample},
	{"pm_Tout", &pm_Tout},
	{"pm_iHOLD", &pm_iHOLD},
	{"pm_iSINE", &pm_iSINE},
	{"pm_sineF", &pm_sineF},

	{"pm_drift", &pm_drift},

	{"pm_U", &pm_U},
	{"pm_E_wb", &pm_E_wb},
	{"pm_E_kv", &pm_E_kv},
	{"pm_R", &pm_R},
	{"pm_Ld", &pm_Lq},
	{"pm_Lq", &pm_Lq},
	{"pm_Zp", &pm_Zp},
	{"pm_M", &pm_M},
	{"pm_J", &pm_J},
	{"pm_const", &pm_const},

	{"pm_iSPD", &pm_iSPD},
	{"pm_iSPQ", &pm_iSPQ},
	{"pm_wSP", &pm_wSP},

	{"pm_req_spinup", &pm_req_spinup},
	{"pm_req_break", &pm_req_break},
	{"pm_req_sine", &pm_req_sine},
	{"pm_req_linear", &pm_req_linear},

	{"tel_enable", &tel_enable},
	{"tel_show", &tel_show},

	{NULL, NULL},
};

