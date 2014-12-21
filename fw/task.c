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
#include "task.h"
#include "sh.h"
#include "lib.h"

taskDATA_t			td;

void halTick()
{
	td.uTICK++;
}

void adcIRQ()
{
}

void taskIN()
{
	int		xC;

	if (td.xCAN) {

		/* TODO */
	}
	else {
		do {
			xC = uartRX();

			if (xC < 0)
				break;
			else
				shExSend(xC);
		}
		while (1);
	}
}

void taskOUT()
{
	int		xC, xN;
	char		*pX;

	if (td.xCAN) {

		/* TODO */
	}
	else {
		pX = uartGetTX();

		if (pX != NULL) {

			xN = 0;

			do {
				xC = shExRecv();

				if (xC < 0)
					break;
				else
					*pX++ = (char) xC,
					xN++;

				if (xN >= (UART_TXBUF_SZ - 1))
					break;
			}
			while (1);

			if (xN > 0) {

				uartTX(xN);
			}
		}
	}
}

void taskCAN()
{
}

void taskYield()
{
	if (td.xIN) {

		td.xIN = 0;
		taskIN();
	}

	if (td.xOUT) {

		td.xOUT = 0;
		taskOUT();
	}

	if (td.xSH && !(td.mBUSY & 1UL)) {

		td.xSH = 0;
		td.mBUSY |= 1UL;
		shTask();
		td.mBUSY &= ~1UL;
	}

	halWFI();
}

void halMain()
{
	halLED(LED_BLUE);
	uartEnable(57600UL);

	do {
		taskYield();
	}
	while (1);
}

