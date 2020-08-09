#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32f4xx.h"
#include "hal.h"
#include "libc.h"

#define GPIO_USART3_TX			XGPIO_DEF4('C', 10, 0, 7)
#define GPIO_USART3_RX			XGPIO_DEF4('C', 11, 0, 7)

#ifdef _HW_KLEN

#undef GPIO_USART3_TX
#undef GPIO_USART3_RX
#define GPIO_USART3_TX			XGPIO_DEF4('B', 10, 0, 7)
#define GPIO_USART3_RX			XGPIO_DEF4('B', 11, 0, 7)

#endif /* _HW_KLEN */

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

	SR = USART3->SR;

	if (SR & USART_SR_RXNE) {

		xC = USART3->DR;
		xQueueSendToBackFromISR(hal_USART.queue_RX, &xC, &xWoken);

		IODEF_TO_USART();
	}

	if (SR & USART_SR_TXE) {

		if (xQueueReceiveFromISR(hal_USART.queue_TX, &xC, &xWoken) == pdTRUE) {

			USART3->DR = xC;
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

	/* Alloc queues.
	 * */
	hal_USART.queue_RX = xQueueCreate(40, sizeof(char));
	hal_USART.queue_TX = xQueueCreate(80, sizeof(char));

	/* Configure USART.
	 * */
	USART3->BRR = CLOCK_APB1_HZ / hal.USART_baud_rate;
	USART3->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
		| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
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

