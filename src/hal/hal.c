/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include "cmsis/stm32f4xx.h"
#include "hal.h"

extern long			ldSvectors;
unsigned long 			hal_CLOCK_CPU_HZ;

extern void debugTRACE(const char *fmt, ...);

void irqNMI()
{
	debugTRACE("irq: NMI");
	halHalt();
}

void irqHardFault()
{
	debugTRACE("irq: HardFault");
	halHalt();
}

void irqMemoryFault()
{
	debugTRACE("irq: MemoryFault");
	halHalt();
}

void irqBusFault()
{
	debugTRACE("irq: BusFault");
	halHalt();
}

void irqUsageFault()
{
	debugTRACE("irq: UsageFault");
	halHalt();
}

void irqDefault()
{
	debugTRACE("irq: Default");
	halHalt();
}

static void
clock_start()
{
	int		HSERDY, HSEN = 0;

	/* Enable HSI.
	 * */
	RCC->CR |= RCC_CR_HSION;

	/* Reset RCC.
	 * */
	RCC->CFGR = 0;
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_PLLON);
	RCC->CIR = 0;

	/* Enable HSE.
	 * */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready.
	 * */
	do {
		HSERDY = RCC->CR & RCC_CR_HSERDY;
		HSEN++;
	}
	while (!HSERDY && HSEN < HSE_STARTUP_TIMEOUT);

	/* Enable power interface clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Regulator voltage scale 1 mode.
	 * */
	PWR->CR |= PWR_CR_VOS;

	/* Set AHB/APB1/APB2 prescalers.
	 * */
	RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	if (HSERDY) {

		/* From HSE.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
			| (7UL << 24) | (0UL << 16)
			| (168UL << 6) | (6UL << 0);
	}
	else {
		/* From HSI.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI
			| (7UL << 24) | (0UL << 16)
			| (168UL << 6) | (8UL << 0);
	}

	/* Enable PLL.
	 * */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready.
	 * */
	while (!(RCC->CR & RCC_CR_PLLRDY)) ;

	/* Configure Flash.
	 * */
	FLASH->ACR = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	/* Select PLL.
	 * */
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till PLL is used.
	 * */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) ;

	/* Define clock frequency.
	 * */
	hal_CLOCK_CPU_HZ = 168000000UL;
}

static void
board_start()
{
	/* Vector table offset.
	 * */
	SCB->VTOR = (unsigned long) &ldSvectors;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(3UL);

	/* Enable Programmable voltage detector.
	 * */
	PWR->CR |= PWR_CR_PLS_LEV7 | PWR_CR_PVDE;

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
		| RCC_AHB1ENR_GPIOCEN;

	/* Enable LED pins.
	 * */
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER5 | GPIO_MODER_MODER6
			| GPIO_MODER_MODER7, GPIO_MODER_MODER5_0
			| GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

	/* Enable FPU.
	 * */
	SCB->CPACR |= (3UL << 20) | (3UL << 22);
}

void hal_startup()
{
	clock_start();
	board_start();
}

void hal_halt()
{
	hal_set_LED(1);

	__disable_irq();
	__WFI();

	for (;;) ;
}

void hal_system_reset()
{
	NVIC_SystemReset();
}

void hal_sleep()
{
	__WFI();
}

void hal_fence()
{
	__DMB();
}

void hal_boost_converter(int x)
{
	if (x != 0) {

		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER2, GPIO_MODER_MODER2_0);
		GPIOB->BSRRL = (1UL << 2);
	}
	else {
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER2, 0);
		GPIOB->BSRRH = (1UL << 2);
	}
}

void hal_set_LED(int x)
{
	if (x != 0) {

		GPIOB->BSRRL = (1UL << 5);
	}
	else {
		GPIOB->BSRRH = (1UL << 5);
	}
}

int hal_get_HALL()
{
	int		HALL = 0;

	if (GPIOC->IDR & (1UL << 6)) {

		HALL |= LEG_A;
	}

	if (GPIOC->IDR & (1UL << 7)) {

		HALL |= LEG_B;
	}

	if (GPIOC->IDR & (1UL << 8)) {

		HALL |= LEG_C;
	}

	return HALL;
}

