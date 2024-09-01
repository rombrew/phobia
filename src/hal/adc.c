#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

typedef struct {

	SemaphoreHandle_t	mutex_sem;

	uint32_t		dmabuf[8] LD_DMA;
}
priv_ADC_t;

static priv_ADC_t		priv_ADC;

void irq_ADC()
{
	if (likely(ADC3->SR & ADC_SR_JEOC)) {

		ADC1->SR = ~ADC_SR_JEOC;
		ADC2->SR = ~ADC_SR_JEOC;
		ADC3->SR = ~ADC_SR_JEOC;

#if (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU)
		hal.ADC_current_A = (float) ((int) ADC1->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_current_B = (float) ((int) ADC2->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_voltage_U = (float) ((int) ADC3->JDR1) * hal.const_ADC.GU;
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU_TTT)
		hal.ADC_current_A = (float) ((int) ADC1->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_current_B = (float) ((int) ADC2->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_voltage_U = (float) ((int) ADC3->JDR1) * hal.const_ADC.GU;
		hal.ADC_voltage_A = (float) ((int) ADC1->JDR2) * hal.const_ADC.GT;
		hal.ADC_voltage_B = (float) ((int) ADC2->JDR2) * hal.const_ADC.GT;
		hal.ADC_voltage_C = (float) ((int) ADC3->JDR2) * hal.const_ADC.GT;
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UXX)
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TXX)
		hal.ADC_current_A = (float) ((int) ADC1->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_current_B = (float) ((int) ADC2->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_current_C = (float) ((int) ADC3->JDR1 - 2047) * hal.const_ADC.GA;
		hal.ADC_voltage_U = (float) ((int) ADC1->JDR2) * hal.const_ADC.GU;
		hal.ADC_voltage_A = (float) ((int) ADC2->JDR2) * hal.const_ADC.GT;
		hal.ADC_voltage_B = (float) ((int) ADC3->JDR2) * hal.const_ADC.GT;
		hal.ADC_voltage_C = (float) ((int) ADC1->JDR3) * hal.const_ADC.GT;
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TSC)
#endif /* HW_ADC_SAMPLING_SEQUENCE */

		EXTI->SWIER = EXTI_SWIER_SWIER0;
	}
}

void irq_EXTI0()
{
	EXTI->PR = EXTI_PR_PR0;

	hal.CNT_raw[0] = TIM1->ARR - TIM1->CNT;
	hal.CNT_raw[1] = TIM7->CNT;

	hal.CNT_raw[2] = hal.CNT_raw[1];

	ADC_IRQ();

	hal.CNT_raw[3] = TIM7->CNT;

	hal.CNT_raw[2] = (hal.CNT_raw[2] - hal.CNT_raw[1]) & 0xFFFFU;
	hal.CNT_raw[3] = (hal.CNT_raw[3] - hal.CNT_raw[1]) & 0xFFFFU;

	hal.CNT_diag[0] = (float) hal.CNT_raw[0] * hal.const_CNT[0];
	hal.CNT_diag[1] = hal.CNT_diag[0] + (float) hal.CNT_raw[2] * hal.const_CNT[1];
	hal.CNT_diag[2] = hal.CNT_diag[0] + (float) hal.CNT_raw[3] * hal.const_CNT[1];
}

static void
ADC_set_SMPR(ADC_TypeDef *pADC, int xCH, int xSMP)
{
	if (xCH < 10) {

		MODIFY_REG(pADC->SMPR2, 7U << (xCH * 3), xSMP << (xCH * 3));
	}
	else {
		MODIFY_REG(pADC->SMPR1, 7U << ((xCH - 10) * 3), xSMP << ((xCH - 10) * 3));
	}
}

void ADC_const_build()
{
#if defined(STM32F4)
	const uint16_t		*TS_30 = (const uint16_t *) 0x1FFF7A2CU;
	const uint16_t		*TS_110 = (const uint16_t *) 0x1FFF7A2EU;
#elif defined(STM32F7)
	const uint16_t		*TS_30 = (const uint16_t *) 0x1FF07A2CU;
	const uint16_t		*TS_110 = (const uint16_t *) 0x1FF07A2EU;
#endif /* STM32Fx */

	float			U_reference, R_equivalent;

	U_reference = hal.ADC_reference_voltage / (float) ADC_RESOLUTION;
	R_equivalent = hal.ADC_shunt_resistance * hal.ADC_amplifier_gain;

	hal.const_ADC.GA = U_reference / R_equivalent;
	hal.const_ADC.GU = U_reference / hal.ADC_voltage_ratio;
	hal.const_ADC.GT = U_reference / hal.ADC_terminal_ratio;
	hal.const_ADC.GS = hal.ADC_reference_voltage / (float) ADC_RESOLUTION;
	hal.const_ADC.TS[1] = 80.f / (float) (*TS_110 - *TS_30);
	hal.const_ADC.TS[0] = 110.f - hal.const_ADC.TS[1] * (float) (*TS_110);

#ifdef STM32F4
	if (		hal.MCU_ID == MCU_ID_GD32F405
			|| *TS_110 == *TS_30) {

		hal.const_ADC.TS[1] = 0.323f;
		hal.const_ADC.TS[0] = -279.f;
	}
#endif /* STM32F4 */

#ifdef HW_HAVE_ANALOG_KNOB
	hal.const_ADC.GK = 1.f / hal.ADC_knob_ratio;
#endif /* HW_HAVE_ANALOG_KNOB */

	hal.const_CNT[0] = 1.f / (float) CLOCK_TIM1_HZ;
	hal.const_CNT[1] = 1.f / (float) CLOCK_TIM7_HZ;

#if (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU)
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_CURRENT_A), hal.ADC_sample_time);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_CURRENT_B), hal.ADC_sample_time);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U), hal.ADC_sample_time);
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU_TTT)
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_CURRENT_A), hal.ADC_sample_time);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_CURRENT_B), hal.ADC_sample_time);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U), hal.ADC_sample_time);
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A), hal.ADC_sample_time);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B), hal.ADC_sample_time);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C), hal.ADC_sample_time);
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UXX)
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TXX)
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_CURRENT_A), hal.ADC_sample_time);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_CURRENT_B), hal.ADC_sample_time);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_CURRENT_C), hal.ADC_sample_time);
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U), hal.ADC_sample_time);
	ADC_set_SMPR(ADC2, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A), hal.ADC_sample_time);
	ADC_set_SMPR(ADC3, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B), hal.ADC_sample_time);
	ADC_set_SMPR(ADC1, XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C), hal.ADC_sample_time);
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TSC)
#endif /* HW_ADC_SAMPLING_SEQUENCE */
}

void ADC_startup()
{
	/* Enable ADC clock.
	 * */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

#ifdef GPIO_ADC_CURRENT_A
	GPIO_set_mode_ANALOG(GPIO_ADC_CURRENT_A);
#endif /* GPIO_ADC_CURRENT_A */

#ifdef GPIO_ADC_CURRENT_B
	GPIO_set_mode_ANALOG(GPIO_ADC_CURRENT_B);
#endif /* GPIO_ADC_CURRENT_B */

#ifdef GPIO_ADC_CURRENT_C
	GPIO_set_mode_ANALOG(GPIO_ADC_CURRENT_C);
#endif /* GPIO_ADC_CURRENT_C */

#ifdef GPIO_ADC_VOLTAGE_U
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_U);
#endif /* GPIO_ADC_VOLTAGE_U */

#ifdef GPIO_ADC_VOLTAGE_A
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_A);
#endif /* GPIO_ADC_VOLTAGE_A */

#ifdef GPIO_ADC_VOLTAGE_B
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_B);
#endif /* GPIO_ADC_VOLTAGE_B */

#ifdef GPIO_ADC_VOLTAGE_C
	GPIO_set_mode_ANALOG(GPIO_ADC_VOLTAGE_C);
#endif /* GPIO_ADC_VOLTAGE_C */

	/* Common configuration (ADC clock = CLOCK_APB2_HZ / 2U).
	 * */
	ADC->CCR = ADC_CCR_TSVREFE;

	/* Configure ADCs.
	 * */
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_DDS | ADC_CR2_DMA;
	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = 0;
	ADC1->SQR1 = 0;

	ADC2->CR1 = ADC_CR1_SCAN;
	ADC2->CR2 = ADC_CR2_JEXTEN_0;
	ADC2->SMPR1 = 0;
	ADC2->SMPR2 = 0;
	ADC2->SQR1 = 0;

	ADC3->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC3->CR2 = ADC_CR2_JEXTEN_0;
	ADC3->SMPR1 = 0;
	ADC3->SMPR2 = 0;
	ADC3->SQR1 = 0;

#if (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU)
	ADC1->JSQR = XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 15;
	ADC2->JSQR = XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 15;
	ADC3->JSQR = XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 15;
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABU_TTT)
	ADC1->JSQR = ADC_JSQR_JL_0
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 15);
	ADC2->JSQR = ADC_JSQR_JL_0
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B) << 15);
	ADC3->JSQR = ADC_JSQR_JL_0
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C) << 15);
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UXX)
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TXX)
	ADC1->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_A) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_U) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_C) << 15);
	ADC2->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_B) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_A) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VREFINT) << 15);
	ADC3->JSQR = ADC_JSQR_JL_1
		| (XGPIO_GET_CH(GPIO_ADC_CURRENT_C) << 5)
		| (XGPIO_GET_CH(GPIO_ADC_VOLTAGE_B) << 10)
		| (XGPIO_GET_CH(GPIO_ADC_VREFINT) << 15);
#elif (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TSC)
#endif /* HW_ADC_SAMPLING_SEQUENCE */

	/* Update CONST.
	 * */
	ADC_const_build();

	/* Allocate semaphore.
	 * */
	priv_ADC.mutex_sem = xSemaphoreCreateMutex();

	/* Enable DMA on ADC1.
	 * */
	DMA2_Stream0->CR = (0U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1
		| (2U << DMA_SxCR_MSIZE_Pos) | (2U << DMA_SxCR_PSIZE_Pos);
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t) &priv_ADC.dmabuf[0];
	DMA2_Stream0->FCR = DMA_SxFCR_DMDIS;

	/* Enable ADCs.
	 * */
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

#ifdef STM32F4
	if (hal.MCU_ID == MCU_ID_GD32F405) {

		TIM_wait_ns(800);

		ADC1->CR2 |= (1U << 3);	/* RSTCLB */
		ADC1->CR2 |= (1U << 2);	/* CLB */

		ADC2->CR2 |= (1U << 3);
		ADC2->CR2 |= (1U << 2);

		ADC3->CR2 |= (1U << 3);
		ADC3->CR2 |= (1U << 2);

		while (		   (ADC1->CR2 & (1U << 2)) == 0U
				&& (ADC2->CR2 & (1U << 2)) == 0U
				&& (ADC3->CR2 & (1U << 2)) == 0U) {

			__NOP();
		}
	}
#endif /* STM32F4 */

	/* Enable EXTI0.
	 * */
	EXTI->IMR = EXTI_IMR_MR0;

	/* Enable IRQs.
	 * */
	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

float ADC_get_sample(int xGPIO)
{
	int			xCH, xADC;

	float			um = 0.f;

	if (xSemaphoreTake(priv_ADC.mutex_sem, (TickType_t) 10) == pdTRUE) {

		DMA2_Stream0->CR &= ~DMA_SxCR_EN;

		__DSB();

#ifdef STM32F7
		/* Invalidate D-Cache on DMABUF.
		 * */
		SCB->DCIMVAC = (uint32_t) &priv_ADC.dmabuf[0];

		__DSB();
		__ISB();
#endif /* STM32F7 */

		xCH = XGPIO_GET_CH(xGPIO);

		ADC_set_SMPR(ADC1, xCH, ADC_SMP_480);

		DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0
			| DMA_LIFCR_CTEIF0 | DMA_LIFCR_CFEIF0;

		DMA2_Stream0->NDTR = 1;
		DMA2_Stream0->CR |= DMA_SxCR_EN;

		ADC1->SQR3 = xCH;
		ADC1->CR2 |= ADC_CR2_SWSTART;

		while ((DMA2->LISR & (DMA_LISR_TCIF0 | DMA_LISR_TEIF0)) == 0U) {

			taskYIELD();
		}

		xADC = priv_ADC.dmabuf[0];

		if (xCH == XGPIO_GET_CH(GPIO_ADC_TEMPINT)) {

			um = (float) (xADC) * hal.const_ADC.TS[1] + hal.const_ADC.TS[0];
		}
		else {
			um = (float) (xADC) * hal.const_ADC.GS;
		}

		xSemaphoreGive(priv_ADC.mutex_sem);
	}

	return um;
}

