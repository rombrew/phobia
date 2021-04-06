#include <stddef.h>

#include "hal.h"
#include "libc.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

typedef struct {

	QueueHandle_t		queue_RX;
	QueueHandle_t		queue_TX;
}
HAL_USART_t;

static HAL_USART_t		hal_USART;

void irq_USART3()
{
	BaseType_t		xWoken = pdFALSE;
	u32_t 			SR;
	char			xC;

#if defined(_HW_STM32F405)
	SR = USART3->SR;
#elif defined(_HW_STM32F722)
	SR = USART3->ISR;
#endif /* _HW_STM32Fxx */

#if defined(_HW_STM32F405)
	if (SR & USART_SR_RXNE) {
#elif defined(_HW_STM32F722)
	if (SR & USART_ISR_RXNE) {
#endif /* _HW_STM32Fxx */

#if defined(_HW_STM32F405)
		xC = USART3->DR;
#elif defined(_HW_STM32F722)
		xC = USART3->RDR;
#endif /* _HW_STM32Fxx */

		xQueueSendToBackFromISR(hal_USART.queue_RX, &xC, &xWoken);

		IODEF_TO_USART();
	}

#if defined(_HW_STM32F405)
	if (SR & USART_SR_TXE) {
#elif defined(_HW_STM32F722)
	if (SR & USART_ISR_TXE) {
#endif /* _HW_STM32Fxx */

		if (xQueueReceiveFromISR(hal_USART.queue_TX, &xC, &xWoken) == pdTRUE) {

#if defined(_HW_STM32F405)
			USART3->DR = xC;
#elif defined(_HW_STM32F722)
			USART3->TDR = xC;
#endif /* _HW_STM32Fxx */

		}
		else {
			USART3->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	portYIELD_FROM_ISR(xWoken);
}

void USART_startup()
{
	/* Enable USART3 clock.
	 * */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	/* Enable USART3 pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_USART3_TX);
	GPIO_set_mode_FUNCTION(GPIO_USART3_RX);
	GPIO_set_mode_PULL_UP(GPIO_USART3_RX);

	/* Alloc queues.
	 * */
	hal_USART.queue_RX = xQueueCreate(40, sizeof(char));
	hal_USART.queue_TX = xQueueCreate(80, sizeof(char));

	/* Configure USART.
	 * */
	USART3->BRR = CLOCK_APB1_HZ / hal.USART_baud_rate;

#if defined(_HW_STM32F405)
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
#elif defined(_HW_STM32F722)
	USART3->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
#endif /* _HW_STM32Fxx */

	USART3->CR2 = 0;
	USART3->CR3 = 0;

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(USART3_IRQn, 11);
	NVIC_EnableIRQ(USART3_IRQn);
}

int USART_getc()
{
	char		xC;

	xQueueReceive(hal_USART.queue_RX, &xC, portMAX_DELAY);

	return (int) xC;
}

void USART_putc(int c)
{
	char		xC = (char) c;

	GPIO_set_HIGH(GPIO_LED);

	if (xQueueSendToBack(hal_USART.queue_TX, &xC, portMAX_DELAY) == pdTRUE) {

		USART3->CR1 |= USART_CR1_TXEIE;
	}

	GPIO_set_LOW(GPIO_LED);
}

QueueHandle_t USART_queue_RX() { return hal_USART.queue_RX; }

