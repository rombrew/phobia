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
#include "adc.h"
#include "hal.h"

halADC_t			halADC;
halADC_CONST_t			halADC_CONST = {

	.A_1 = 2.6855E-2f,
	.B_1 = 2.6855E-2f,
	.U_1 = 2.8981E-2f,
	.HVIN_1 = 2.8981E-2f,

	/* NTC PN: EWTF05-103H3I-N
	 * */
	.NTC = {-6.5874724E-22f,
		1.0000274E-17f,
		-6.2750268E-14f,
		2.1018330E-10f,
		-4.0750417E-7f,
		4.6735139E-4f,
		-3.3537976E-1f,
		1.7679703E+2f},

	.REF_1 = 8.0566E-4f,
};

void irqADC()
{
	float			fc;

	if (ADC1->SR & ADC_SR_JEOC) {

		ADC1->SR &= ~ADC_SR_JEOC;

		halADC.thermal_xNTC = ADC1->JDR1;
		halADC.thermal_xTEMP = ADC1->JDR2;
		halADC.in_xREF = ADC1->JDR3;

		adcIRQ_thermal();
	}

	if (ADC2->SR & ADC_SR_JEOC) {

		ADC2->SR &= ~ADC_SR_JEOC;
		ADC3->SR &= ~ADC_SR_JEOC;

		fc = (float) (ADC2->JDR1 - 2048);
		halADC.sensor_A = fc * halADC_CONST.A_1;

		fc = (float) (ADC2->JDR2);
		halADC.supply_U = fc * halADC_CONST.U_1;

		fc = (float) (ADC3->JDR1 - 2048);
		halADC.sensor_B = fc * halADC_CONST.B_1;

		fc = (float) (ADC3->JDR2);
		halADC.external_HVIN = fc * halADC_CONST.HVIN_1;

		adcIRQ_feedback();
	}
}

static void
adc_temp_calibration()
{
	unsigned short		*CAL_TEMP_30 = (void *) 0x1FFF7A2C;
	unsigned short		*CAL_TEMP_110 = (void *) 0x1FFF7A2E;

	halADC_CONST.TEMP_0 = 1.375f * (*CAL_TEMP_30) - .375f * (*CAL_TEMP_110);
	halADC_CONST.TEMP_1 = .0125f * (*CAL_TEMP_110) - .0125f * (*CAL_TEMP_30);
}

static void
timerEnable()
{
	/* Enable TIM2 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2.
	 * */
	TIM2->CR1 = TIM_CR1_ARPE;
	TIM2->CR2 = TIM_CR2_MMS_1;
	TIM2->CNT = 0;
	TIM2->PSC = 99UL;
	TIM2->ARR = HAL_APB1_HZ * 2UL / 100UL / ADC_THERMAL_FREQ_HZ;
	TIM2->RCR = 0;

	/* Start TIM2.
	 * */
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
}

static void
timerDisable()
{
	/* Disable TIM2.
	 * */
	TIM2->CR1 = 0;

	/* Disable TIM2 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
}


void adcEnable()
{
	/* Enable ADC clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

	/* Enable analog PA0 PA1 PA2 PC2 PC3 pins.
	 * */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER0 | GPIO_MODER_MODER1
			| GPIO_MODER_MODER2,
			GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1
			| GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1
			| GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1);
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER2 | GPIO_MODER_MODER3,
			GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1
			| GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1);

	/* Common configuration (clock = 21 MHz).
	 * */
	ADC->CCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_0;

	/* Configure ADC1 on PC2-IN12 (thermal_NTC) IN16 (thermal_TEMP) IN17 (in_REF).
	 * */
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_1 | ADC_CR2_JEXTSEL_0;
	ADC1->SMPR1 = ADC_SMPR1_SMP17_2 | ADC_SMPR1_SMP17_1
		| ADC_SMPR1_SMP17_0 | ADC_SMPR1_SMP16_2
		| ADC_SMPR1_SMP16_1 | ADC_SMPR1_SMP16_0
		| ADC_SMPR1_SMP12_2 | ADC_SMPR1_SMP12_1
		| ADC_SMPR1_SMP12_0;
	ADC1->SMPR2 = 0;
	ADC1->JSQR = ADC_JSQR_JL_1 | ADC_JSQR_JSQ2_3 | ADC_JSQR_JSQ2_2
		| ADC_JSQR_JSQ3_4 | ADC_JSQR_JSQ4_4 | ADC_JSQR_JSQ4_0;

	/* Configure ADC2 on PC3-IN13 (sensor_A) PA2-IN2 (supply_U).
	 * */
	ADC2->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC2->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = 0;
	ADC2->JSQR = ADC_JSQR_JL_0 | ADC_JSQR_JSQ3_3 | ADC_JSQR_JSQ3_2
		| ADC_JSQR_JSQ3_0 | ADC_JSQR_JSQ4_1;

	/* Configure ADC3 on PA1-IN1 (sensor_B) PA0-IN0 (external_HVIN).
	 * */
	ADC3->CR1 = ADC_CR1_SCAN;
	ADC3->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = 0;
	ADC3->JSQR = ADC_JSQR_JL_0 | ADC_JSQR_JSQ4_0;

	/* Update CONST.
	 * */
	adc_temp_calibration();

	/* Enable ADC.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/* Enable thermal trigger.
	 * */
	timerEnable();

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 3);
	NVIC_EnableIRQ(ADC_IRQn);
}

void adcDisable()
{
	/* Disable thermal trigger.
	 * */
	timerDisable();

	/* Disable PA0 PA1 PA2 PC2 PC3 pins.
	 * */
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER0 | GPIO_MODER_MODER1
			| GPIO_MODER_MODER2, 0);
	MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODER2 | GPIO_MODER_MODER3, 0);

	/* Disable ADC.
	 * */
	ADC1->CR2 = 0;
	ADC2->CR2 = 0;
	ADC3->CR2 = 0;

	/* Disable ADC clock.
	 * */
	RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN);
}

