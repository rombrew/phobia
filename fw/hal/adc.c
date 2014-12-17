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

halADC_TypeDef			halADC;

void irqADC()
{
	ADC1->SR &= ~ADC_SR_JEOC;

	halADC.xA = ADC1->JDR1;
	GPIOE->ODR ^= (1UL << 7);
}

void adcEnable()
{
	/* Enable ADC1 clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	/* Enable analog PA1, PA2, PA3 pins.
	 * */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1 | GPIO_MODER_MODER2
			| GPIO_MODER_MODER3,
			GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1
			| GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1
			| GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1);

	/* Common configuration.
	 * */
	ADC->CCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_0;

	/* Configure ADC1.
	 * */
	ADC1->CR1 = ADC_CR1_JEOCIE;
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC1->SMPR1 = ADC_SMPR1_SMP16_2;
	ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_0;
	ADC1->JSQR = ADC_JSQR_JSQ4_4;

	/* Enable ADC1.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 5);
	NVIC_EnableIRQ(ADC_IRQn);
}

void adcDisable()
{
}

