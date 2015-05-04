/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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
	int		Tirq, Tbase, Rpc;

	Tbase = 2 * halPWM.resolution;
	Tirq = td.Tirq;
	Rpc = 100 * Tirq / Tbase;

	printf("%i%% (%i/%i)" EOL, Rpc, Tirq, Tbase);
}

void reboot(const char *s)
{
	int		End, Del = 3;

	printf("Reboot in %i second" EOL, Del);

	End = td.uSEC + Del;

	do {
		taskIOMUX();
	}
	while (td.uSEC < End);

	halReset();
}

void pwm_freq_hz(const char *s)
{
	stoi(&halPWM.freq_hz, s);
	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

void pwm_dead_time_ns(const char *s)
{
	stoi(&halPWM.dead_time_ns, s);
	printf("%i (ns)" EOL, halPWM.dead_time_ns);
}

void pm_m_request_hold(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_HOLD;
	}
}

void pm_m_request_sine(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_SINE;
	}
}

void pm_m_request_bemf(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_BEMF;
	}
}

void pm_m_request_linear(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_LINEAR;
	}
}

void pm_m_request_begin(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_BEGIN;
	}
}

void pm_m_request_end(const char *s)
{
	if (pm.m_request == 0) {

		pm.m_request = PMC_STATE_END;
	}
}

void pm_m_bitmask(const char *s)
{
}

void pm_m_errno(const char *s)
{
}

void pm_pwm_resolution(const char *s)
{
	printf("%i" EOL, pm.pwm_resolution);
}

void pm_pwm_minimal_pulse(const char *s)
{
	float		conv_G;
	int		ns;

	conv_G = 1e-9f * pm.freq_hz * pm.pwm_resolution;

	if (stoi(&ns, s) != NULL)
		pm.pwm_minimal_pulse = (int) (ns * conv_G + .5f);

	ns = (int) (pm.pwm_minimal_pulse / conv_G + .5f);

	printf("%i (ns)" EOL, ns);
}

void pm_T_drift(const char *s)
{
	stof(&pm.T_drift, s);
	printf("%3f (Sec)" EOL, &pm.T_drift);
}

void pm_T_hold(const char *s)
{
	stof(&pm.T_hold, s);
	printf("%3f (Sec)" EOL, &pm.T_hold);
}

void pm_T_rohm(const char *s)
{
	stof(&pm.T_rohm, s);
	printf("%3f (Sec)" EOL, &pm.T_rohm);
}

void pm_T_sine(const char *s)
{
	stof(&pm.T_sine, s);
	printf("%3f (Sec)" EOL, &pm.T_sine);
}

void pm_T_bemf(const char *s)
{
	stof(&pm.T_bemf, s);
	printf("%3f (Sec)" EOL, &pm.T_bemf);
}

void pm_T_end(const char *s)
{
	stof(&pm.Tend, s);
	printf("%3f (Sec)" EOL, &pm.T_end);
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

void pm_iOFSD(const char *s)
{
	stof(&pm.iOFSD, s);
	printf("%3f (A)" EOL, &pm.iOFSD);
}

void pm_iOFSQ(const char *s)
{
	stof(&pm.iOFSQ, s);
	printf("%3f (A)" EOL, &pm.iOFSQ);
}

void pm_sF(const char *s)
{
	stof(&pm.sF, s);
	printf("%1f (Hz)" EOL, &pm.sF);
}

void pm_kX_iD(const char *s)
{
	printf("iD %3f (A)" EOL, &pm.kX[0]);
}

void pm_kX_iQ(const char *s)
{
	printf("iQ %3f (A)" EOL, &pm.kX[1]);
}

void pm_kX_wR(const char *s)
{
	float			RPM;

	RPM = 9.5492969f * pm.kX[4] / pm.Zp;

	printf("wR %4e (Rad/Sec) %1f (RPM) " EOL, &pm.kX[4], &RPM);
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
	float		Kv, Z;

	if (stof(&Kv, s) != NULL) {

		Z = Kv * pm.Zp;
		pm.E = (Z != 0.f) ? 5.513289f / Z : pm.E;
	}

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
	if (stof(&pm.Ld, s) != NULL)
		pm.ILd = 1.f / pm.Ld;

	printf("%4e (H)" EOL, &pm.Ld);
}

void pm_Lq(const char *s)
{
	if (stof(&pm.Lq, s) != NULL)
		pm.ILq = 1.f / pm.Lq;

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
		pm.IJ = (J != 0.f) ? 1.f / J : pm.IJ;

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

void pm_iMAX(const char *s)
{
	stof(&pm.iMAX, s);
	printf("%3f (A)" EOL, &pm.iMAX);
}

void pm_iMIN(const char *s)
{
	stof(&pm.iMIN, s);
	printf("%3f (A)" EOL, &pm.iMIN);
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

void pm_wMAX_rpm(const char *s)
{
	float		RPM;

	if (stof(&RPM, s) != NULL)
		pm.wMAX = RPM * .10471976f;

	RPM = pm.wMAX * 9.5492969f;

	printf("%1f (RPM)" EOL, &RPM);
}

void pm_wMIN_rpm(const char *s)
{
	float		RPM;

	if (stof(&RPM, s) != NULL)
		pm.wMIN = RPM * .10471976f;

	RPM = pm.wMIN * 9.5492969f;

	printf("%1f (RPM)" EOL, &RPM);
}

void pm_wSP_rpm(const char *s)
{
	float		RPM;

	if (stof(&RPM, s) != NULL)
		pm.wSP = RPM * .10471976f;

	RPM = pm.wSP * 9.5492969f;

	printf("%1f (RPM)" EOL, &RPM);
}

void tel_setup(const char *s)
{
	tel.s_clock_scale = 1;

	stoi(&tel.s_clock_scale, s);

	tel.pZ = tel.pD;
	tel.s_clock = 0;
}

void tel_enable(const char *s)
{
	tel.enabled = 1;
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
	{"pwm_dead_time_ns", &pwm_dead_time_ns},

	{"pm_spinup", &pm_spinup},
	{"pm_break", &pm_break},
	{"pm_hold", &pm_hold},
	{"pm_sine", &pm_sine},
	{"pm_e", &pm_e},
	{"pm_linear", &pm_linear},

	{"pm_pwmR", &pm_pwmR},
	{"pm_pwmMP_ns", &pm_pwmMP_ns},

	{"pm_Tdrift", &pm_Tdrift},
	{"pm_Thold", &pm_Thold},
	{"pm_Trohm", &pm_Trohm},
	{"pm_Tsine", &pm_Tsine},
	{"pm_Tout", &pm_Tout},
	{"pm_Tend", &pm_Tend},

	{"pm_iHOLD", &pm_iHOLD},
	{"pm_iSINE", &pm_iSINE},
	{"pm_iOFSD", &pm_iOFSD},
	{"pm_iOFSQ", &pm_iOFSQ},
	{"pm_sF", &pm_sF},

	{"pm_kX_iD", &pm_kX_iD},
	{"pm_kX_iQ", &pm_kX_iQ},
	{"pm_kX_wR", &pm_kX_wR},
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

	{"pm_iMAX", &pm_iMAX},
	{"pm_iMIN", &pm_iMIN},
	{"pm_iSPD", &pm_iSPD},
	{"pm_iSPQ", &pm_iSPQ},

	{"pm_wMAX_rpm", &pm_wMAX_rpm},
	{"pm_wMIN_rpm", &pm_wMIN_rpm},
	{"pm_wSP_rpm", &pm_wSP_rpm},

	{"tel_setup", &tel_setup},
	{"tel_enable", &tel_enable},
	{"tel_show", &tel_show},

	{NULL, NULL},
};

