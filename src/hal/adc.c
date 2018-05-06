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

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define ADC_RESOLUTION			4096

typedef struct {

	SemaphoreHandle_t	xSem;
	int			channel_sel;
}
HAL_ADC_t;

static HAL_ADC_t		hal_ADC;

void irqADC()
{
	float			fADC;
	int			xADC, xCH;

	if (ADC2->SR & ADC_SR_JEOC) {

		ADC2->SR &= ~ADC_SR_JEOC;
		ADC3->SR &= ~ADC_SR_JEOC;

		xADC = (int) ADC2->JDR1;
		hal.ADC_current_A = (float) (xADC - 2048) * hal.ADC_const.GA;

		xADC = (int) ADC2->JDR2;
		hal.ADC_voltage_U = (float) (xADC) * hal.ADC_const.GU;

		xADC = (int) ADC3->JDR1;
		hal.ADC_current_B = (float) (xADC - 2048) * hal.ADC_const.GA;

		xADC = (int) ADC2->JDR2;
		fADC = (float) (xADC) * hal.ADC_const.GU;

		switch (hal_ADC.channel_sel) {

			case 0:
				xCH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B);
				hal.ADC_voltage_A = fADC;
				hal_ADC.channel_sel = 1;
				break;

			case 1:
				xCH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C);
				hal.ADC_voltage_B = fADC;
				hal_ADC.channel_sel = 2;
				break;

			case 2:
				xCH = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A);
				hal.ADC_voltage_C = fADC;
				hal_ADC.channel_sel = 0;
				break;

			default:
				hal_ADC.channel_sel = 0;
				break;
		}

		MODIFY_REG(ADC3->JSQR, 0x1F << 15, xCH << 15);

		ADC_IRQ();
	}
}

static void
ADC_const_setup()
{
	unsigned short		*CAL_TEMP_30 = (void *) 0x1FFF7A2C;
	unsigned short		*CAL_TEMP_110 = (void *) 0x1FFF7A2E;

	hal.ADC_const.GA = hal.ADC_reference_voltage / (float) ADC_RESOLUTION
		/ hal.ADC_current_shunt_resistance / hal.ADC_amplifier_gain;

	hal.ADC_const.GU = hal.ADC_reference_voltage / (float) ADC_RESOLUTION
		/ hal.ADC_voltage_divider_gain;

	hal.ADC_const.GS = 1.f / (float) ADC_RESOLUTION;

	hal.ADC_const.TEMP[1] = (30.f - 110.f) / (float) (*CAL_TEMP_30 - *CAL_TEMP_110);
	hal.ADC_const.TEMP[0] = 110.f - hal.ADC_const.TEMP[1] * (float) (*CAL_TEMP_110);
}

void ADC_startup()
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

	/* Common configuration (21 MHz).
	 * */
	ADC->CCR = ADC_CCR_TSVREFE | ADC_CCR_ADCPRE_0;

	/* Configure ADC1.
	 * */
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = 0;
	ADC1->SMPR1 = 0x07FFFFFF;
	ADC1->SMPR2 = 0x3FFFFFFF;

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
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 15);

	/* Update CONST.
	 * */
	ADC_const_setup();

	/* Alloc Semaphore.
	 * */
	hal_ADC.xSem = xSemaphoreCreateMutex();

	/* Enable ADC.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 3);
	NVIC_EnableIRQ(ADC_IRQn);
}

float ADC_get_VALUE(int xGPIO)
{
	float			fADC = 0.f;
	int			xCH;

	if (xSemaphoreTake(hal_ADC.xSem, (TickType_t) 10) == pdTRUE) {

		xCH = (xGPIO == GPIO_ADC_INTERNAL_TEMP) ? 16
			: XGPIO_GET_CH(xGPIO);

		ADC1->SQR3 = xCH;
		ADC1->CR2 |= ADC_CR2_SWSTART;

		while ((ADC1->SR & ADC_SR_EOC) == 0) {

			taskYIELD();
		}

		ADC1->SR &= ~ADC_SR_EOC;
		fADC = (float) ADC1->DR;

		fADC = (xCH != 16) ? fADC * hal.ADC_const.GS
			: hal.ADC_const.TEMP[1] * fADC + hal.ADC_const.TEMP[0];

		xSemaphoreGive(hal_ADC.xSem);
	}

	return fADC;
}

