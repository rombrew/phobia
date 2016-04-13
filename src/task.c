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

#include "hal/hal.h"
#include "lib.h"
#include "pmc.h"
#include "sh.h"
#include "task.h"

taskDATA_t			td;
pmc_t __CCM__			pm;

void halTick()
{
	td.uDS++;
	td.uTIM++;

	if (td.uDS >= 100) {

		td.uDS = 0;
		td.uSEC++;

		td.usage_T = td.usage_S;
		td.usage_S = 0;
	}
}

static void
taskRecvSend(int (* pRecv) (), int (* pSend) (int), int *temp)
{
	int		xC;

	do {
		if (*temp >= 0) {

			if (pSend(*temp) < 0)
				break;
			else
				*temp = -1;
		}

		do {
			xC = pRecv();

			if (xC < 0)
				break;
			else if (pSend(xC) < 0) {

				*temp = xC;
				break;
			}
		}
		while (1);
	}
	while (0);
}

void taskIOMUX()
{
	/* USART <---> SH.
	 * */
	taskRecvSend(&usartRecv, &shExSend, td.mux_TEMP + 0);
	taskRecvSend(&shExRecv, &usartSend, td.mux_TEMP + 1);
	usartFlush();

	/* TODO: CAN.
	 * */
}

void taskYIELD()
{
	taskIOMUX();
	halSleep();
}

void canIRQ()
{
}

void adcIRQ()
{
	int		T, dT;

	T = halSysTick();

	pmc_feedback(&pm, halADC.xA, halADC.xB);
	pmc_voltage(&pm, halADC.xSUPPLY);

	dT = T - halSysTick();
	dT += (dT < 0) ? HZ_AHB / 100 : 0;
	td.usage_S += dT;

	if (td.pEX != NULL)
		td.pEX();
}

void halMain()
{
	td.mux_TEMP[0] = -1;
	td.mux_TEMP[1] = -1;

	/* Config.
	 * */
	halUSART.baudRate = 57600;
	halPWM.freq_hz = 60000;
	halPWM.dead_time_ns = 70;
	td.av_default_time = .2f;

	halLED(LED_RED);

	usartEnable();

	pwmEnable();
	adcEnable();

	memz(&pm, sizeof(pm));
	pm.freq_hz = (float) halPWM.freq_hz;
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_resolution = halPWM.resolution;
	pm.pDC = &pwmDC;
	pm.pZ = &pwmZ;

	pmc_default(&pm);

	do {
		taskIOMUX();
		shTask();
		halSleep();
	}
	while (1);
}

void evAV_8()
{
	int			j;

	if (td.av_sample_N <= td.av_sample_MAX) {

		for (j = 0; j < td.av_variable_N; ++j)
			td.av_VAL[j] += *td.av_IN[j];

		td.av_sample_N++;
	}
	else
		td.pEX = NULL;
}

float task_av_float_1(float *param, float time)
{
	td.av_IN[0] = param;
	td.av_VAL[0] = 0.f;
	td.av_variable_N = 1;
	td.av_sample_N = 0;
	td.av_sample_MAX = pm.freq_hz * time;

	halFence();

	td.pEX = &evAV_8;

	while (td.pEX != NULL)
		taskYIELD();

	td.av_VAL[0] /= (float) td.av_sample_N;

	return td.av_VAL[0];
}

float task_av_float_arg_1(float *param, const char *s)
{
	float			time = td.av_default_time;

	stof(&time, s);

	return task_av_float_1(param, time);
}

SH_DEF(hal_uptime)
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

SH_DEF(hal_cpu_usage)
{
	float		Rpc;

	Rpc = 100.f * (float) td.usage_T / (float) HZ_AHB;

	printf("%1f %% (%i)" EOL, &Rpc, td.usage_T / halPWM.freq_hz);
}

SH_DEF(hal_av_default_time)
{
	stof(&td.av_default_time, s);
	printf("%3f (Sec)" EOL, &td.av_default_time);
}

SH_DEF(hal_reboot)
{
	int		End, Del = 3;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	printf("Reboot in %i second" EOL, Del);

	End = td.uSEC + Del;

	do {
		taskYIELD();
	}
	while (td.uSEC < End);

	halReset();
}

SH_DEF(hal_keycodes)
{
	int		xC;

	do {
		while ((xC = shRecv()) < 0)
			taskYIELD();

		if (xC == 3 || xC == 4)
			break;

		printf("-- %i" EOL, xC);
	}
	while (1);
}

SH_DEF(hal_pwm_freq_hz)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.freq_hz, s) != NULL) {

		pwmDisable();
		pwmEnable();

		pm.freq_hz = (float) halPWM.freq_hz;
		pm.dT = 1.f / pm.freq_hz;
		pm.pwm_resolution = halPWM.resolution;
	}

	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

SH_DEF(hal_pwm_dead_time_ns)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.dead_time_ns, s) != NULL) {

		pwmDisable();
		pwmEnable();
	}

	printf("%i (tk) %i (ns)" EOL, halPWM.dead_time_tk, halPWM.dead_time_ns);
}

SH_DEF(hal_pwm_DC)
{
	const char	*tok;
	int		tokN = 0;
	int		xA, xB, xC, R;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	tok = s;
	s = stoi(&xA, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xB, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xC, tok);
	tokN += (s != NULL) ? 1 : 0;

	if (tokN == 3) {

		R = halPWM.resolution;

		xA = (xA < 0) ? 0 : (xA > R) ? R : xA;
		xB = (xB < 0) ? 0 : (xB > R) ? R : xB;
		xC = (xC < 0) ? 0 : (xC > R) ? R : xC;

		pwmDC(xA, xB, xC);

		printf("DC %i %i %i" EOL, xA, xB, xC);
	}
}

SH_DEF(hal_pwm_Z)
{
	int		Z;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&Z, s) != NULL) {

		pwmZ(Z);

		printf("Z %i" EOL, Z);
	}
}

