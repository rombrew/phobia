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
	int		xWFI = 1;
	char		*pX;

	/* From USART to SH.
	 * */
	xN = shExPoll();

	while (xN >= 0) {

		xC = usartRecv();

		if (xC < 0)
			break;
		else
			shExPush(xC),
			xWFI = 0;

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
			xWFI = 0;
		}
	}

	/* TODO: From CAN to SH.
	 * */

	/* TODO: From SH to CAN.
	 * */

	xWFI ? halWFI() : 0;
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

	T1 = halSysTick();

	/* IRQ load ticks.
	 * */
	td.Tirq = T0 - T1;

	if (td.pIRQ != NULL)
		td.pIRQ();
}

void halMain()
{
	halLED(LED_BLUE);

	/* Config.
	 * */
	halUSART.baudRate = 57600;
	halPWM.freq_hz = 60000;
	halPWM.dead_time_ns = 70;
	td.avg_default_time = .2f;

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

	pm.m_state = PMC_STATE_ZERO_DRIFT;

	do {
		taskIOMUX();
		shTask();
		halWFI();
	}
	while (1);
}

