/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

halADC_t			halADC;

void irqADC()
{
	if (ADC1->SR & ADC_SR_JEOC) {

		ADC1->SR &= ~ADC_SR_JEOC;
		ADC2->SR &= ~ADC_SR_JEOC;
		ADC3->SR &= ~ADC_SR_JEOC;

		halADC.xA = (1UL << 12) - ADC2->JDR1;
		halADC.xB = (1UL << 12) - ADC3->JDR1;
		halADC.xU = 1821; /* ADC1->JDR1;*/

		//__set_BASEPRI(7);
		adcIRQ();
	}
}

void adcEnable()
{
	/* Enable ADC clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

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

	/* Configure ADC1 on PA3.
	 * */
	ADC1->CR1 = ADC_CR1_JEOCIE;
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = ADC_SMPR2_SMP3_1;
	ADC1->JSQR = ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_1;

	/* Configure ADC2 on PA1.
	 * */
	ADC2->CR1 = 0;
	ADC2->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = ADC_SMPR2_SMP1_1;
	ADC2->JSQR = ADC_JSQR_JSQ4_0;

	/* Configure ADC3 on PA2.
	 * */
	ADC3->CR1 = 0;
	ADC3->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = ADC_SMPR2_SMP2_1;
	ADC3->JSQR = ADC_JSQR_JSQ4_1;

	/* Enable ADC.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 5);
	NVIC_EnableIRQ(ADC_IRQn);
}

void adcDisable()
{
	/* Disable ADC clock.
	 * */
	RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN);
}

