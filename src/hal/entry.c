/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#include <stddef.h>

#include "cmsis/stm32f4xx.h"
#include "hal/hal.h"

extern long ld_stack;
extern long ld_end_text;
extern long ld_begin_data;
extern long ld_end_data;
extern long ld_begin_bss;
extern long ld_end_bss;
extern long ld_begin_ccm;
extern long ld_end_ccm;

void irqReset() __attribute__ (( noreturn ));
void irqNMI();
void irqHardFault();
void irqMemoryFault();
void irqBusFault();
void irqUsageFault();
void irqDefault();
void irqSVCall();
void irqPendSV();
void irqSysTick();

void irqADC();
void irqCAN1_TX();
void irqCAN1_RX0();
void irqCAN1_RX1();
void irqCAN1_SCE();
void irqTIM1_UP_TIM10();
void irqUSART3();

__attribute__ (( section(".vectors"), used )) void * vectors[] = {

	(void *) &ld_stack,

	irqReset,
	irqNMI,
	irqHardFault,
	irqMemoryFault,
	irqBusFault,
	irqUsageFault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqSVCall,
	irqDefault,
	irqDefault,
	irqPendSV,
	irqSysTick,

	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqADC,
	irqCAN1_TX,
	irqCAN1_RX0,
	irqCAN1_RX1,
	irqCAN1_SCE,
	irqDefault,
	irqDefault,
	irqTIM1_UP_TIM10,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqUSART3,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault,
	irqDefault
};

static inline void
__init_data(const long *s, long *d, long *e)
{
	while (d < e) { *d++ = *s++; }
}

static inline void
__init_bss(long *d, long *e)
{
	while (d < e) { *d++ = 0; }
}

void irqReset()
{
	

	__init_data(&ld_end_text, &ld_begin_data, &ld_end_data);
	__init_bss(&ld_begin_bss, &ld_end_bss);
	__init_bss(&ld_begin_ccm, &ld_end_ccm);

	hal_startup();

	hal_main();
	hal_system_reset();

	for (;;) ;
}

