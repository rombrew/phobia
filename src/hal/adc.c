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

typedef struct {

	int		channel_sel;
}
HAL_ADC_t;

static HAL_ADC_t		hal_ADC;

void irqADC()
{
	unsigned long		CH;
	float			fadc;
	int			adc;

	if (ADC1->SR & ADC_SR_JEOC) {

		ADC1->SR &= ~ADC_SR_JEOC;

		adc = (int) ADC1->JDR1;
		hal.ADC_thermal_PCB_NTC = (float) (adc) * hal.ADC_const.NTC;

		adc = (int) ADC1->JDR2;
		hal.ADC_thermal_EXT_NTC = (float) (adc) * hal.ADC_const.NTC;

		fadc = (float) ADC1->JDR3;
		hal.ADC_thermal_TEMP = hal.ADC_const.TEMP[1] * fadc + hal.ADC_const.TEMP[0];
	}

	if (ADC2->SR & ADC_SR_JEOC) {

		ADC2->SR &= ~ADC_SR_JEOC;
		ADC3->SR &= ~ADC_SR_JEOC;

		adc = (int) ADC2->JDR1;
		hal.ADC_current_A = (float) (adc - 2048) * hal.ADC_const.GA;

		adc = (int) ADC2->JDR2;
		hal.ADC_voltage_U = (float) (adc) * hal.ADC_const.GU;

		adc = (int) ADC3->JDR1;
		hal.ADC_current_B = (float) (adc - 2048) * hal.ADC_const.GA;

		adc = (int) ADC2->JDR2;
		fadc = (float) (adc) * hal.ADC_const.GU;

		switch (hal_ADC.channel_sel) {

			case 0:
				CH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B);
				hal.ADC_voltage_A = fadc;
				hal_ADC.channel_sel = 1;
				break;

			case 1:
				CH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C);
				hal.ADC_voltage_B = fadc;
				hal_ADC.channel_sel = 2;
				break;

			case 2:
				CH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A);
				hal.ADC_voltage_C = fadc;
				hal_ADC.channel_sel = 0;
				break;

			default:
				hal_ADC.channel_sel = 0;
				break;
		}

		MODIFY_REG(ADC3->JSQR, 0x1F << 15, CH << 15);

		ADC_IRQ();
	}
}

static void
ADC_const_setup()
{
	unsigned short		*CAL_TEMP_30 = (void *) 0x1FFF7A2C;
	unsigned short		*CAL_TEMP_110 = (void *) 0x1FFF7A2E;

	hal.ADC_const.GA = hal.ADC_reference_voltage / (float) hal.ADC_resolution
		/ hal.ADC_current_shunt_resistance / hal.ADC_amplifier_gain;

	hal.ADC_const.GU = hal.ADC_reference_voltage / (float) hal.ADC_resolution
		/ hal.ADC_voltage_divider_gain;

	hal.ADC_const.NTC = 1.f / (float) hal.ADC_resolution;

	hal.ADC_const.TEMP[1] = (30.f - 110.f) / (float) (*CAL_TEMP_30 - *CAL_TEMP_110);
	hal.ADC_const.TEMP[0] = 110.f - hal.ADC_const.TEMP[1] * (float) (*CAL_TEMP_110);
}

static void
tim2_enable()
{
	/* Enable TIM2 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2 (1 Hz).
	 * */
	TIM2->CR1 = TIM_CR1_ARPE;
	TIM2->CR2 = TIM_CR2_MMS_1;
	TIM2->CNT = 0;
	TIM2->PSC = 9999UL;
	TIM2->ARR = HAL_APB1_HZ * 2UL / 10000UL;
	TIM2->RCR = 0;

	/* Start TIM2.
	 * */
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CR1 |= TIM_CR1_CEN;
}

static void
tim2_disable()
{
	/* Disable TIM2.
	 * */
	TIM2->CR1 = 0;

	/* Disable TIM2 clock.
	 * */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
}

void ADC_enable()
{
	/* Enable ADC clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

	/* Enable analog GPIO.
	 * */
	GPIO_set_mode_ANALOG(GPIO_ADC_CURRENT_A);
	GPIO_set_mode_ANALOG(GPIO_ADC_CURRENT_B);
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_U);
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_A);
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_B);
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_C);
	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_NTC);
	GPIO_set_mode_ANALOG(GPIO_ADC_EXT_NTC);

	/* Common configuration (21 MHz).
	 * */
	ADC->CCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_0;

	/* Configure ADC1.
	 * */
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_1 | ADC_CR2_JEXTSEL_0;
	ADC1->SMPR1 = 0x07FFFFFF;
	ADC1->SMPR2 = 0x3FFFFFFF;
	ADC1->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_PCB_NTC) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_EXT_NTC) << 10)
		| (16UL << 15);

	/* Configure ADC2.
	 * */
	ADC2->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC2->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = 0;
	ADC2->JSQR = ADC_JSQR_JL_0
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 15);

	/* Configure ADC3.
	 * */
	ADC3->CR1 = ADC_CR1_SCAN;
	ADC3->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = 0;
	ADC3->JSQR = ADC_JSQR_JL_0
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 15);

	/* Update CONST.
	 * */
	ADC_const_setup();

	/* Enable ADC.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/* Enable TIM2.
	 * */
	tim2_enable();

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 3);
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_disable()
{
	/* Disable IRQ.
	 * */
	NVIC_DisableIRQ(ADC_IRQn);

	/* Disable TIM2.
	 * */
	tim2_disable();

	/* Disable GPIO.
	 * */
	GPIO_set_mode_INPUT(GPIO_ADC_CURRENT_A);
	GPIO_set_mode_INPUT(GPIO_ADC_CURRENT_B);
	GPIO_set_mode_INPUT(GPIO_ADC_VOLTAGE_U);
	GPIO_set_mode_INPUT(GPIO_ADC_VOLTAGE_A);
	GPIO_set_mode_INPUT(GPIO_ADC_VOLTAGE_B);
	GPIO_set_mode_INPUT(GPIO_ADC_VOLTAGE_C);
	GPIO_set_mode_INPUT(GPIO_ADC_PCB_NTC);
	GPIO_set_mode_INPUT(GPIO_ADC_EXT_NTC);

	/* Disable ADC.
	 * */
	ADC1->CR2 = 0;
	ADC2->CR2 = 0;
	ADC3->CR2 = 0;

	/* Disable ADC clock.
	 * */
	RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN);
}

