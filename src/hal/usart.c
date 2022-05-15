#include <stddef.h>

#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

typedef struct {

	QueueHandle_t		queue_RX;
	QueueHandle_t		queue_TX;
}
priv_USART_t;

static priv_USART_t		priv_USART;

void irq_USART3()
{
	BaseType_t		xWoken = pdFALSE;
	u32_t 			SR;
	char			xC;

#if defined(STM32F4)
	SR = USART3->SR;
#elif defined(STM32F7)
	SR = USART3->ISR;
#endif /* STM32Fx */

#if defined(STM32F4)
	if (SR & USART_SR_RXNE) {
#elif defined(STM32F7)
	if (SR & USART_ISR_RXNE) {
#endif /* STM32Fx */

#if defined(STM32F4)
		xC = USART3->DR;
#elif defined(STM32F7)
		xC = USART3->RDR;
#endif /* STM32Fx */

		xQueueSendToBackFromISR(priv_USART.queue_RX, &xC, &xWoken);

		IODEF_TO_USART();
	}

#if defined(STM32F4)
	if (SR & USART_SR_TXE) {
#elif defined(STM32F7)
	if (SR & USART_ISR_TXE) {
#endif /* STM32Fx */

		if (xQueueReceiveFromISR(priv_USART.queue_TX, &xC, &xWoken) == pdTRUE) {

#if defined(STM32F4)
			USART3->DR = xC;
#elif defined(STM32F7)
			USART3->TDR = xC;
#endif /* STM32Fx */

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
	priv_USART.queue_RX = xQueueCreate(80, sizeof(char));
	priv_USART.queue_TX = xQueueCreate(80, sizeof(char));

	/* Configure USART.
	 * */
	USART3->BRR = CLOCK_APB1_HZ / hal.USART_baud_rate;

#if defined(STM32F4)
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
#elif defined(STM32F7)
	USART3->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
#endif /* STM32Fx */

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

	xQueueReceive(priv_USART.queue_RX, &xC, portMAX_DELAY);

	return (int) xC;
}

void USART_putc(int c)
{
	char		xC = (char) c;

	GPIO_set_HIGH(GPIO_LED);

	if (xQueueSendToBack(priv_USART.queue_TX, &xC, portMAX_DELAY) == pdTRUE) {

		USART3->CR1 |= USART_CR1_TXEIE;
	}

	GPIO_set_LOW(GPIO_LED);
}

QueueHandle_t USART_queue_RX() { return priv_USART.queue_RX; }

