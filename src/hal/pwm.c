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

#include "cmsis/stm32f4xx.h"
#include "hal.h"

halPWM_t			halPWM;

void irqTIM1_UP_TIM10() { }

void pwmEnable()
{
	int		R, D;

	/* Update configuration.
	 * */
	R = HZ_APB2 * 2UL / 2UL / halPWM.freq_hz;
	R = (R & 1) ? R + 1 : R;
	halPWM.freq_hz = HZ_APB2 * 2UL / 2UL / R;
	halPWM.resolution = R;

	D = ((HZ_APB2 * 2UL / 1000UL) * halPWM.dead_time_ns + 500000UL) / 1000000UL;
	D = (D < 128) ? D : (D < 256) ? 128 + (D - 128) / 2 : 191;
	halPWM.dead_time_tk = ((D < 128) ? D : (D < 192) ? 128 + (D - 128) * 2 : 255);
	halPWM.dead_time_ns = halPWM.dead_time_tk * 1000000UL / (HZ_APB2 * 2UL / 1000UL);

	/* Enable TIM1 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Configure TIM1.
	 * */
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CMS_1;
	TIM1->CR2 = TIM_CR2_MMS_1 | TIM_CR2_CCPC;
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
	TIM1->RCR = 0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_OSSR | D;

	/* Start TIM1.
	 * */
	TIM1->EGR |= TIM_EGR_COMG | TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->RCR = 1;

	/* Enable PA8 PA9 PA10 PB13 PB14 PB15 pins.
	 * */
	MODIFY_REG(GPIOA->AFR[1], (15UL << 0) | (15UL << 4) | (15UL << 8),
			(1UL << 0) | (1UL << 4) | (1UL << 8));
	MODIFY_REG(GPIOB->AFR[1], (15UL << 20) | (15UL << 24) | (15UL << 28),
			(1UL << 20) | (1UL << 24) | (1UL << 28));
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR8
			| GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10,
			GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR9_0
			| GPIO_OSPEEDER_OSPEEDR10_0);
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDER_OSPEEDR13
			| GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15,
			GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR14_0
			| GPIO_OSPEEDER_OSPEEDR15_0);
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER8
			| GPIO_MODER_MODER9 | GPIO_MODER_MODER10,
			GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1
			| GPIO_MODER_MODER10_1);
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER13
			| GPIO_MODER_MODER14 | GPIO_MODER_MODER15,
			GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1
			| GPIO_MODER_MODER15_1);

}

void pwmDisable()
{
	/* Disable pins.
	 * */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER8
			| GPIO_MODER_MODER9 | GPIO_MODER_MODER10, 0);
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER13
			| GPIO_MODER_MODER14 | GPIO_MODER_MODER15, 0);

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
	if (Z & PWM_A) {

		TIM1->CCER &= ~TIM_CCER_CC1NE;
		TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_1;
	}
	else {
		TIM1->CCER |= TIM_CCER_CC1NE;
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_1;
	}

	if (Z & PWM_B) {

		TIM1->CCER &= ~TIM_CCER_CC2NE;
		TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_1;
	}
	else {
		TIM1->CCER |= TIM_CCER_CC2NE;
		TIM1->CCMR1 |= TIM_CCMR1_OC2M_1;
	}

	if (Z & PWM_C) {

		TIM1->CCER &= ~TIM_CCER_CC3NE;
		TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_1;
	}
	else {
		TIM1->CCER |= TIM_CCER_CC3NE;
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_1;
	}

	TIM1->EGR |= TIM_EGR_COMG;
}

