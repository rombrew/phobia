#include "freertos/FreeRTOS.h"
#include "cmsis/stm32f4xx.h"
#include "hal.h"

#define ADC_RESOLUTION			4096

typedef struct {

	SemaphoreHandle_t	xSem;
}
HAL_ADC_t;

static HAL_ADC_t		hal_ADC;

void irqADC()
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
		hal.ADC_voltage_A = (float) (xADC) * hal.ADC_const.GU;

		xADC = (int) ADC2->JDR3;
		hal.ADC_voltage_B = (float) (xADC) * hal.ADC_const.GU;

		xADC = (int) ADC3->JDR3;
		hal.ADC_voltage_C = (float) (xADC) * hal.ADC_const.GU;

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

void ADC_irq_lock()
{
	ADC2->CR1 &= ~ADC_CR1_JEOCIE;
}

void ADC_irq_unlock()
{
	ADC2->CR1 |= ADC_CR1_JEOCIE;
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
	ADC2->CR1 = ADC_CR1_SCAN;
	ADC2->CR2 = ADC_CR2_JEXTEN_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = 0;
	ADC2->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B) << 15);

	/* Configure ADC3.
	 * */
	ADC3->CR1 = ADC_CR1_SCAN;
	ADC3->CR2 = ADC_CR2_JEXTEN_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = 0;
	ADC3->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C) << 15);

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
	NVIC_SetPriority(ADC_IRQn, 0);
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

