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

#ifndef _H_HAL_
#define _H_HAL_

#include "adc.h"
#include "pwm.h"
#include "uart.h"

enum {
	LED_GREEN		= 1,
	LED_ORANGE		= 2,
	LED_RED			= 4,
	LED_BLUE		= 8
};

typedef struct {

	/* Clock frequency (Hz).
	 * */
	unsigned long int	hzAHB;
	unsigned long int	hzAPB1;
	unsigned long int	hzAPB2;

	/* Resest reason.
	 * */
	int		rstR;
}
halBASE_TypeDef;

extern halBASE_TypeDef		halBASE;
extern void			*ldHeap;

void halStart();
void halWFI();
void halLED(int F);

extern void halMain();
extern void halTick();

#endif /* _H_HAL_ */

