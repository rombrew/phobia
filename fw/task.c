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

typedef struct {

	int		mBUSY;
	void		(* listTF[4]) ();

	int		shCAN;
}
task_t;

static task_t		ts;

void halTick()
{
}

void adcIRQ()
{
}

extern void taskYield();

void taskIN()
{
	int		xC;

	if (ts.shCAN) {

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

	if (ts.shCAN) {

		/* TODO */
	}
	else {
		do {
			xC = shExRecv();

			if (xC < 0)
				break;
			else {
				while ((pX = uartTryTX()) == NULL)
					taskYield();

				*pX++ = xC;
				xN = 1;

				do {
					xC = shExRecv();

					if (xC < 0)
						break;
					else
						*pX++ = xC,
						xN++;

					if (xN >= (UART_TXBUF_SZ - 1))
						break;
				}
				while (1);

				uartTX(xN);
			}
		}
		while (1);
	}
}

void taskCAN()
{
}

void taskYield()
{
	void		(** pTF) ();
	int		tsID;

	pTF = ts.listTF;
	tsID = 1;

	while (* pTF) {

		if (!(ts.mBUSY & tsID)) {

			ts.mBUSY |= tsID;
			(* pTF) ();
			ts.mBUSY &= ~tsID;
		}

		pTF++;
		tsID <<= 1;
	}

	halWFI();
}

void halMain()
{
	halLED(LED_BLUE);
	uartEnable(57600UL);

	ts.listTF[0] = &taskIN;
	ts.listTF[1] = &taskOUT;
	ts.listTF[2] = &shTask;

	do {
		taskYield();
	}
	while (1);
}

