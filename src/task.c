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
#include "pmc.h"
#include "sh.h"
#include "task.h"
#include "tel.h"

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

	if (td.pIRQ != NULL)
		td.pIRQ();
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
	td.avg_default_time = .2f;

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

