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

	if (td.uDS >= 100) {

		td.uDS = 0;
		td.uSEC++;
	}
}

void taskIOMUX()
{
	int		xC, xN;
	char		*pX;

	/* From USART to SH.
	 * */
	xN = shExPoll();

	while (xN >= 0) {

		xC = usartRecv();

		if (xC < 0)
			break;
		else
			shExPush(xC);

		xN--;
	}

	/* From SH to USART.
	 * */
	if (usartPoll()) {

		pX = halUSART.TX;
		xN = 0;

		do {
			xC = shExRecv();

			if (xC < 0)
				break;
			else
				*pX++ = (char) xC,
				xN++;

			if (xN >= (USART_TXBUF_SZ - 1))
				break;
		}
		while (1);

		if (xN > 0) {

			usartPushAll(xN);
		}
	}

	halWFI();
}

void canIRQ()
{
}

void adcIRQ()
{
	int		T0, T1;

	T0 = halSysTick();

	pmc_feedback(&pm, halADC.xA, halADC.xB);
	pmc_voltage(&pm, halADC.xU);

	if (tel.enabled) {

		tel.p_list[0] = (short int) (pm.kalman_X[0] * 1000.f);
		tel.p_list[1] = (short int) (pm.kalman_X[1] * 1000.f);
		tel.p_list[2] = (short int) (pm.kalman_X[2] * 1000.f);
		tel.p_list[3] = (short int) (pm.kalman_X[3] * 1000.f);
		tel.p_size = 4;

		tel_capture();
	}

	T1 = halSysTick();

	/* IRQ load ticks.
	 * */
	td.Tirq = T0 - T1;
}

void halMain()
{
	halLED(LED_BLUE);

	/* Config.
	 * */
	halUSART.baudRate = 57600;
	halPWM.freq_hz = 40000;
	halPWM.dead_time_ns = 200;

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

	pm.m_request = PMC_STATE_ZERO_DRIFT;

	do {
		taskIOMUX();
		shTask();
	}
	while (1);
}

