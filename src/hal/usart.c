#include <stddef.h>

#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

typedef struct {

	QueueHandle_t		rx_queue;
	QueueHandle_t		tx_queue;
}
priv_USART_t;

static priv_USART_t		priv_USART;

void irq_USART3()
{
	BaseType_t		xWoken = pdFALSE;
	uint32_t		SR;
	char			xbyte;

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
		xbyte = USART3->DR;
#elif defined(STM32F7)
		xbyte = USART3->RDR;
#endif /* STM32Fx */

		xQueueSendToBackFromISR(priv_USART.rx_queue, &xbyte, &xWoken);

		IODEF_TO_USART();
	}

#if defined(STM32F4)
	if (SR & USART_SR_TXE) {
#elif defined(STM32F7)
	if (SR & USART_ISR_TXE) {
#endif /* STM32Fx */

		if (xQueueReceiveFromISR(priv_USART.tx_queue, &xbyte, &xWoken) == pdTRUE) {

#if defined(STM32F4)
			USART3->DR = xbyte;
#elif defined(STM32F7)
			USART3->TDR = xbyte;
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
	priv_USART.rx_queue = xQueueCreate(320, sizeof(char));
	priv_USART.tx_queue = xQueueCreate(80, sizeof(char));

	/* Configure USART.
	 * */
	USART3->BRR = CLOCK_APB1_HZ / hal.USART_baud_rate;

#if defined(STM32F4)
	USART3->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_RXNEIE
		| USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
#elif defined(STM32F7)
	USART3->CR1 = USART_CR1_M0 | USART_CR1_PCE | USART_CR1_RXNEIE
		| USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
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
	char		xbyte;

	xQueueReceive(priv_USART.rx_queue, &xbyte, portMAX_DELAY);

	return (int) xbyte;
}

void USART_putc(int c)
{
	char		xbyte = (char) c;

	GPIO_set_HIGH(GPIO_LED_ALERT);

	xQueueSendToBack(priv_USART.tx_queue, &xbyte, portMAX_DELAY);

	USART3->CR1 |= USART_CR1_TXEIE;

	GPIO_set_LOW(GPIO_LED_ALERT);
}

QueueHandle_t USART_public_rx_queue() { return priv_USART.rx_queue; }

