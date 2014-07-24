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

halPWM_TypeDef			halPWM;

void pwmEnable(int hzF, int nsD)
{
	const int	FREQ_APB2_KHZ = FREQ_APB2_HZ / 1000UL;
	int		R, D;

	/* Calculate values.
	 * */
	R = FREQ_APB2_HZ / 2 / hzF;
	R = (R & 1) ? R + 1 : R;
	hzF = FREQ_APB2_HZ / R;

	D = (FREQ_APB2_KHZ * nsD + 500000UL) / 1000000UL;
	D = (D > 127) ? 127 : D;
	nsD = D * 1000000UL / FREQ_APB2_KHZ;

	halPWM.hzF = hzF;
	halPWM.R = R;
	halPWM.nsD = nsD;

	/* Enable TIM1 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Enable PE8-PE13 pins.
	 * */
	MODIFY_REG(GPIOE->AFR[1], (15UL << 0) | (15UL << 4)
			| (15UL << 8) | (15UL << 12)
			| (15UL << 16) | (15UL << 20),
			(1UL << 0) | (1UL << 4)
			| (1UL << 8) | (1UL << 12)
			| (1UL << 16) | (1UL << 20));
	/*MODIFY_REG(GPIOE->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9
			| GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11
			| GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR13,
			GPIO_OSPEEDER_OSPEEDR8)*/
	MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODER8 | GPIO_MODER_MODER9
			| GPIO_MODER_MODER10 | GPIO_MODER_MODER11
			| GPIO_MODER_MODER12 | GPIO_MODER_MODER13,
			GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1
			| GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1
			| GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1);

	/* Configure TIM1.
	 * */
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
	TIM1->CR2 = TIM_CR2_CCPC;
	TIM1->SMCR = 0;
	TIM1->DIER = 0;
	TIM1->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE
		| TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
	TIM1->CCER = TIM_CCER_CC3NE | TIM_CCER_CC3E | TIM_CCER_CC2NE
		| TIM_CCER_CC2E | TIM_CCER_CC1NE | TIM_CCER_CC1E;
	TIM1->CNT = 0;
	TIM1->PSC = 0;
	TIM1->ARR = R;
	TIM1->RCR = 1;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	TIM1->BDTR = TIM_BDTR_MOE | D;

	/* Start TIM1.
	 * */
	TIM1->EGR |= TIM_EGR_COMG | TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
}

void pwmDisable()
{
	/* Disable pins.
	 * */
	MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODER8 | GPIO_MODER_MODER9
			| GPIO_MODER_MODER10 | GPIO_MODER_MODER11
			| GPIO_MODER_MODER12 | GPIO_MODER_MODER13, 0);

	/* Disable TIM1.
	 * */
	TIM1->CR1 = 0;
	TIM1->CR2 = 0;

	/* Disable clock.
	 * */
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
}

void pwmDC(int uA, int uB, int uC)
{
	TIM1->CCR1 = uA;
	TIM1->CCR2 = uB;
	TIM1->CCR3 = uC;
}

void pwmZ(int Z)
{
}

