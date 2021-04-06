#include <stddef.h>

#include "hal.h"
#include "libc.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

typedef struct {

	SemaphoreHandle_t	sem_MUX;
}
HAL_ADC_t;

static HAL_ADC_t		hal_ADC;

void irq_ADC()
{
	int			xADC;

	if (ADC2->SR & ADC_SR_JEOC) {

		ADC2->SR = ~ADC_SR_JEOC;
		ADC3->SR = ~ADC_SR_JEOC;

		xADC = (int) ADC2->JDR1;
		hal.ADC_current_A = (float) (xADC - 2047) * hal.ADC_const.GA;

		xADC = (int) ADC3->JDR1;
		hal.ADC_current_B = (float) (xADC - 2047) * hal.ADC_const.GA;

		xADC = (int) ADC2->JDR2;
		hal.ADC_voltage_U = (float) (xADC) * hal.ADC_const.GU;

		xADC = (int) ADC3->JDR2;
		hal.ADC_voltage_A = (float) (xADC) * hal.ADC_const.GT[1] + hal.ADC_const.GT[0];

		xADC = (int) ADC2->JDR3;
		hal.ADC_voltage_B = (float) (xADC) * hal.ADC_const.GT[1] + hal.ADC_const.GT[0];

		xADC = (int) ADC3->JDR3;
		hal.ADC_voltage_C = (float) (xADC) * hal.ADC_const.GT[1] + hal.ADC_const.GT[0];

		EXTI->SWIER = EXTI_SWIER_SWIER0;
	}
}

void irq_EXTI0()
{
	EXTI->PR = EXTI_PR_PR0;

	ADC_IRQ();
}

static void
ADC_const_setup()
{
#if defined(_HW_STM32F405)
	u16_t			*CAL_TEMP_30 = (void *) 0x1FFF7A2C;
	u16_t			*CAL_TEMP_110 = (void *) 0x1FFF7A2E;
#elif defined(_HW_STM32F722)
	u16_t			*CAL_TEMP_30 = (void *) 0x1FF07A2C;
	u16_t			*CAL_TEMP_110 = (void *) 0x1FF07A2E;
#endif /* _HW_STM32Fxx */

	hal.ADC_const.GA = hal.ADC_reference_voltage / (float) ADC_RESOLUTION
		/ hal.ADC_shunt_resistance / hal.ADC_amplifier_gain;

	hal.ADC_const.GU = hal.ADC_reference_voltage / (float) ADC_RESOLUTION
		/ hal.ADC_voltage_ratio;

	hal.ADC_const.GT[1] = hal.ADC_reference_voltage / (float) ADC_RESOLUTION
		/ hal.ADC_terminal_ratio;

	hal.ADC_const.GT[0] = (- hal.ADC_terminal_bias) / hal.ADC_terminal_ratio;

	hal.ADC_const.GS = 1.f / (float) ADC_RESOLUTION;

	hal.ADC_const.TEMP[1] = (30.f - 110.f) / (float) (*CAL_TEMP_30 - *CAL_TEMP_110);
	hal.ADC_const.TEMP[0] = 110.f - hal.ADC_const.TEMP[1] * (float) (*CAL_TEMP_110);
}

static void
ADC_set_SMPR(ADC_TypeDef *pADC, int xCH, int xSMP)
{
	if (xCH < 10) {

		pADC->SMPR2 |= xSMP << (xCH * 3);
	}
	else {
		pADC->SMPR1 |= xSMP << ((xCH - 10) * 3);
	}
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

	/* Common configuration (overclocked 42 MHz).
	 * */
	ADC->CCR = ADC_CCR_TSVREFE;

	/* Configure ADC1.
	 * */
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = 0;
	ADC1->SMPR1 = 0x07FFFFFFUL;
	ADC1->SMPR2 = 0x3FFFFFFFUL;

	/* Configure ADC2.
	 * */
	ADC2->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC2->CR2 = ADC_CR2_JEXTEN_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = 0;
	ADC2->SQR3 = XGPIO_GET_CH(GPIO_ADC_CURRENT_A);
	ADC2->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B) << 15);

	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_CURRENT_A), 1);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U), 2);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B), 2);

	/* Configure ADC3.
	 * */
	ADC3->CR1 = ADC_CR1_SCAN;
	ADC3->CR2 = ADC_CR2_JEXTEN_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = 0;
	ADC3->SQR3 = XGPIO_GET_CH(GPIO_ADC_CURRENT_B);
	ADC3->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C) << 15);

	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_CURRENT_B), 1);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A), 2);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C), 2);

	/* Update CONST.
	 * */
	ADC_const_setup();

	/* Allocate Semaphore.
	 * */
	hal_ADC.sem_MUX = xSemaphoreCreateMutex();

	/* Enable ADC.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/* Enable EXTI0.
	 * */
	EXTI->IMR = EXTI_IMR_MR0;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void ADC_configure()
{
	ADC_const_setup();
}

float ADC_get_VALUE(int xGPIO)
{
	float			fVAL = 0.f;
	int			xCH, xADC;

	if (xSemaphoreTake(hal_ADC.sem_MUX, (TickType_t) 10) == pdTRUE) {

		xCH = (xGPIO == GPIO_ADC_INTERNAL_TEMP) ? 16
			: XGPIO_GET_CH(xGPIO);

		ADC1->SQR3 = xCH;
		ADC1->CR2 |= ADC_CR2_SWSTART;

		while ((ADC1->SR & ADC_SR_EOC) == 0) {

			taskYIELD();
		}

		ADC1->SR &= ~ADC_SR_EOC;
		xADC = ADC1->DR;

		if (xCH == 16) {

			fVAL = hal.ADC_const.TEMP[1] * (float) xADC
				+ hal.ADC_const.TEMP[0];
		}
		else {
			fVAL = (float) xADC * hal.ADC_const.GS;
		}

		xSemaphoreGive(hal_ADC.sem_MUX);
	}

	return fVAL;
}

