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

#include "cmsis/stm32f4xx.h"
#include "hal.h"

extern int ldSvectors;

void irqNMI() { }
void irqHardFault() { }
void irqMemoryFault() { }
void irqBusFault() { }
void irqUsageFault() { }
void irqSVCall() { }
void irqPendSV() { }

void irqSysTick()
{
	halTick();
}

static void
clockStart()
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

	/* If HSE is ready.
	 * */
	if (HSERDY) {

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

		/* Enable PLL.
		 * */
		RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
			| (7UL << 24) | (0UL << 16)
			| (168UL << 6) | (4UL << 0);
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
	}
	else {
		/* HSE fails to start up.
		 * */
	}
}

static void
cortexStart()
{
	/* Enable FPU.
	 * */
	SCB->CPACR |= (3UL << 20) | (3UL << 22);

	__DSB();
	__ISB();

	/* Vector table offset.
	 * */
	SCB->VTOR = (int) &ldSvectors;

	/* Configure priority grouping.
	 * */
	NVIC_SetPriorityGrouping(4);

	/* Configure SysTick (1000 Hz).
	 * */
	SysTick_Config(FREQ_AHB_HZ / 100UL);

	/* Enable interrupts.
	 * */
	__enable_irq();
}

static void
boardStart()
{
	/* Enable Programmable voltage detector.
	 * */
	PWR->CR |= PWR_CR_PLS_LEV7 | PWR_CR_PVDE;

	/* Enable CCM RAM clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;

	/* Enable compensation cell.
	 * */
	/*SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;*/

	/* Enable GPIO clock.
	 * */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
		| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

	/* Enable LED pins.
	 * */
	MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODER12 | GPIO_MODER_MODER13
			| GPIO_MODER_MODER14 | GPIO_MODER_MODER15,
			GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0
			| GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void halStart()
{
	clockStart();
	cortexStart();
	boardStart();
}

int halResetReason()
{
	return RCC->CSR >> 25;
}

void halWFI()
{
	__WFI();
}

int halSysTick()
{
	SysTick->VAL;
}

void halLED(int F)
{
	if (F & LED_GREEN)

		GPIOD->BSRRL = (1UL << 12);
	else
		GPIOD->BSRRH = (1UL << 12);

	if (F & LED_ORANGE)

		GPIOD->BSRRL = (1UL << 13);
	else
		GPIOD->BSRRH = (1UL << 13);

	if (F & LED_RED)

		GPIOD->BSRRL = (1UL << 14);
	else
		GPIOD->BSRRH = (1UL << 14);
	
	if (F & LED_BLUE)

		GPIOD->BSRRL = (1UL << 15);
	else
		GPIOD->BSRRH = (1UL << 15);
}

