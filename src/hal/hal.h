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

#ifndef _H_HAL_
#define _H_HAL_

#define HAL_APB1_HZ		(hal_CLOCK_CPU_HZ / 4UL)
#define HAL_APB2_HZ		(hal_CLOCK_CPU_HZ / 2UL)

#define __CCM__			__attribute__ ((section (".ccm")))

enum {
	LED_RED			= 1,
	LED_GREEN		= 2,
	LED_BLUE		= 4
};

extern unsigned long		hal_CLOCK_CPU_HZ;

void halStart();
void halHalt();
void halReset();
void halSleep();
void halFence();
void halLED(int F);

extern void halMain();

#endif /* _H_HAL_ */

