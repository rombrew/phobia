#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

#define _HW_HAVE_USART1		  ((GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('A', 9)  \
				|| (GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('B', 6))

#define _HW_HAVE_USART2		  ((GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('A', 2)  \
				|| (GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('D', 5))

#define _HW_HAVE_USART3		  ((GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('B', 10)  \
				|| (GPIO_USART_TX & 0x7FU) == XGPIO_DEF2('C', 10))

typedef struct {

	USART_TypeDef		*BASE;

	QueueHandle_t		rx_queue;
	QueueHandle_t		tx_queue;
}
priv_USART_t;

static priv_USART_t		priv_USART;

static void
irq_USART(USART_TypeDef *USART)
{
	BaseType_t		xWoken = pdFALSE;
	uint32_t		SR;
	char			xbyte;

#if defined(STM32F4)
	SR = USART->SR;
#elif defined(STM32F7)
	SR = USART->ISR;
#endif /* STM32Fx */

#if defined(STM32F4)
	if (likely(SR & USART_SR_RXNE)) {
#elif defined(STM32F7)
	if (likely(SR & USART_ISR_RXNE)) {
#endif /* STM32Fx */

#if defined(STM32F4)
		xbyte = USART->DR;
#elif defined(STM32F7)
		xbyte = USART->RDR;
#endif /* STM32Fx */

		xQueueSendToBackFromISR(priv_USART.rx_queue, &xbyte, &xWoken);

		IODEF_TO_USART();
	}

#if defined(STM32F4)
	if (likely(SR & USART_SR_TXE)) {
#elif defined(STM32F7)
	if (likely(SR & USART_ISR_TXE)) {
#endif /* STM32Fx */

		if (xQueueReceiveFromISR(priv_USART.tx_queue, &xbyte, &xWoken) == pdTRUE) {

#if defined(STM32F4)
			USART->DR = xbyte;
#elif defined(STM32F7)
			USART->TDR = xbyte;
#endif /* STM32Fx */

		}
		else {
			USART->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	portYIELD_FROM_ISR(xWoken);
}

void irq_USART1() { irq_USART(USART1); }
void irq_USART2() { irq_USART(USART2); }
void irq_USART3() { irq_USART(USART3); }

void USART_startup()
{
	if (_HW_HAVE_USART1) {

		priv_USART.BASE = USART1;
	}
	else if (_HW_HAVE_USART2) {

		priv_USART.BASE = USART2;
	}
	else if (_HW_HAVE_USART3) {

		priv_USART.BASE = USART3;
	}

	/* Enable USART clock.
	 * */
	if (_HW_HAVE_USART1) {

		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
	else if (_HW_HAVE_USART2) {

		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}
	else if (_HW_HAVE_USART3) {

		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	}

	/* Enable USART pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_USART_TX);
	GPIO_set_mode_FUNCTION(GPIO_USART_RX);

	GPIO_set_mode_PULL_UP(GPIO_USART_RX);

	/* Alloc queues.
	 * */
	priv_USART.rx_queue = xQueueCreate(320, sizeof(char));
	priv_USART.tx_queue = xQueueCreate(80, sizeof(char));

	/* Configure USART.
	 * */
	if (_HW_HAVE_USART1) {

		priv_USART.BASE->BRR = CLOCK_APB2_HZ / hal.USART_baudrate;
	}
	else if (_HW_HAVE_USART2 || _HW_HAVE_USART3) {

		priv_USART.BASE->BRR = CLOCK_APB1_HZ / hal.USART_baudrate;
	}

#if defined(STM32F4)
	if (hal.USART_parity == PARITY_EVEN) {

		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
			| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	}
	else if (hal.USART_parity == PARITY_ODD) {

		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE
			| USART_CR1_PS | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	}
	else {
		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_RXNEIE
			| USART_CR1_TE | USART_CR1_RE;
	}

#elif defined(STM32F7)
	if (hal.USART_parity == PARITY_EVEN) {

		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE
			| USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	}
	else if (hal.USART_parity == PARITY_ODD) {

		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_M0 | USART_CR1_PCE
			| USART_CR1_PS | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	}
	else {
		priv_USART.BASE->CR1 = USART_CR1_UE | USART_CR1_RXNEIE
			| USART_CR1_TE | USART_CR1_RE;
	}
#endif /* STM32Fx */

	priv_USART.BASE->CR2 = 0;
	priv_USART.BASE->CR3 = 0;

	/* Enable IRQ.
	 * */
	if (_HW_HAVE_USART1) {

		NVIC_SetPriority(USART1_IRQn, 11);
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else if (_HW_HAVE_USART2) {

		NVIC_SetPriority(USART2_IRQn, 11);
		NVIC_EnableIRQ(USART2_IRQn);
	}
	else if (_HW_HAVE_USART3) {

		NVIC_SetPriority(USART3_IRQn, 11);
		NVIC_EnableIRQ(USART3_IRQn);
	}
}

int USART_getc()
{
	char		xbyte;

	xQueueReceive(priv_USART.rx_queue, &xbyte, portMAX_DELAY);

	return (int) xbyte;
}

int USART_poll()
{
	return (int) uxQueueMessagesWaiting(priv_USART.rx_queue);
}

void USART_putc(int c)
{
	char		xbyte = (char) c;

	GPIO_set_HIGH(GPIO_LED_ALERT);

	xQueueSendToBack(priv_USART.tx_queue, &xbyte, portMAX_DELAY);

	priv_USART.BASE->CR1 |= USART_CR1_TXEIE;

	GPIO_set_LOW(GPIO_LED_ALERT);
}

QueueHandle_t USART_public_rx_queue() { return priv_USART.rx_queue; }

