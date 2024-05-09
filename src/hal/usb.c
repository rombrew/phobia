#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

#include "cherry/usbd_core.h"
#include "cherry/usbd_cdc.h"

#define CDC_DATA_SZ		64U

#define CDC_IN_EP		0x81U
#define CDC_OUT_EP		0x02U
#define CDC_INT_EP		0x83U

#define USBD_VID		0x0483	/* STMicroelectronics */
#define USBD_PID		0x5740	/* Virtual COM Port */

#define USBD_LANGID_STRING	0x0409	/* English (United States) */

typedef struct {

	QueueHandle_t		rx_queue;
	QueueHandle_t		tx_queue;

	struct usbd_interface	intf0;
	struct usbd_interface	intf1;

	LD_DMA uint8_t		rx_buf[CDC_DATA_SZ];
	LD_DMA uint8_t		tx_buf[CDC_DATA_SZ];

	int			rx_flag;
	int			tx_flag;
}
priv_USB_t;

static priv_USB_t		priv_USB;

static const uint8_t		cdc_acm_descriptor[] = {

	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
	USB_CONFIG_DESCRIPTOR_INIT((0x09 + CDC_ACM_DESCRIPTOR_LEN), 0x02, 0x01, USB_CONFIG_SELF_POWERED, 100),

	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x40, 0x02),

	USB_LANGID_INIT(USBD_LANGID_STRING),
	0x26,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0, 'l', 0,
	'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0, 'c', 0, 's', 0,

	0x2A,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'P', 0, 'M', 0, 'C', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
	'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0, 'o', 0,
	'r', 0, 't', 0,

	0x1A,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'3', 0, '5', 0, '7', 0, '0', 0, '3', 0, '7', 0, '7', 0, '4', 0, '3', 0,
	'1', 0, '3', 0, '1', 0,

	0x00
};

void usb_dc_low_level_init(void)
{
#if defined(STM32F4)
	if (hal.MCU_ID == MCU_ID_STM32F405) {

		USB_OTG_FS->GCCFG = USB_OTG_GCCFG_NOVBUSSENS | USB_OTG_GCCFG_PWRDWN;
	}
	else if (hal.MCU_ID == MCU_ID_GD32F405) {

		USB_OTG_FS->GCCFG = (0x000DU << 16);	/* PWRON | VBUSACEN | VBUSBCEN */
	}
#elif defined(STM32F7)
	USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL | USB_OTG_GOTGCTL_BVALOEN;
	USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN;
#endif /* STM32Fx */
}

void usbd_event_handler(uint8_t event)
{
	switch (event) {

		case USBD_EVENT_CONFIGURED:

			priv_USB.rx_flag = 1;
			priv_USB.tx_flag = 0;

			usbd_ep_start_read(CDC_OUT_EP, priv_USB.rx_buf, CDC_DATA_SZ);
			break;

		default:
			break;
	}
}

static void
usbd_cdc_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
	BaseType_t		xWoken = pdFALSE;
	int			n;

	if (nbytes > 0) {

		for (n = 0; n < nbytes; ++n) {

			xQueueSendToBackFromISR(priv_USB.rx_queue,
					&priv_USB.rx_buf[n], &xWoken);
		}

		IODEF_TO_USB();
	}

	if (uxQueueSpacesAvailableFromISR(priv_USB.rx_queue) >= CDC_DATA_SZ) {

		usbd_ep_start_read(CDC_OUT_EP, priv_USB.rx_buf, CDC_DATA_SZ);
	}
	else {
		priv_USB.rx_flag = 0;
	}

	portYIELD_FROM_ISR(xWoken);
}

static void
usbd_cdc_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
	BaseType_t		xWoken = pdFALSE;
	int			len = 0;

	while (		len < CDC_DATA_SZ
			&& xQueueReceiveFromISR(priv_USB.tx_queue,
				&priv_USB.tx_buf[len], &xWoken) == pdTRUE) {
		len++;
	}

	if (len > 0) {

		usbd_ep_start_write(CDC_IN_EP, priv_USB.tx_buf, len);
	}
	else if (nbytes == CDC_DATA_SZ) {

		usbd_ep_start_write(CDC_IN_EP, NULL, 0);
	}
	else {
		priv_USB.tx_flag = 0;
	}

	portYIELD_FROM_ISR(xWoken);
}

void usbd_cdc_acm_set_line_coding(uint8_t intf, struct cdc_line_coding *line_coding)
{
	/* NOTE: We do nothing */
}

void usbd_cdc_acm_get_line_coding(uint8_t intf, struct cdc_line_coding *line_coding)
{
	line_coding->dwDTERate = hal.USART_baudrate;
	line_coding->bDataBits = 8;
	line_coding->bParityType = hal.USART_parity;
	line_coding->bCharFormat = 0;
}

static void
task_cdc_acm_flag_poll()
{
	int			len;

	if (priv_USB.rx_flag == 0) {

		if (uxQueueSpacesAvailable(priv_USB.rx_queue) >= CDC_DATA_SZ) {

			priv_USB.rx_flag = 1;

			usbd_ep_start_read(CDC_OUT_EP, priv_USB.rx_buf, CDC_DATA_SZ);
		}
	}

	if (priv_USB.tx_flag == 0) {

		len = 0;

		while (		len < CDC_DATA_SZ
				&& xQueueReceive(priv_USB.tx_queue,
					&priv_USB.tx_buf[len],
					(TickType_t) 10) == pdTRUE) {
			len++;
		}

		if (len > 0) {

			priv_USB.tx_flag = 1;

			usbd_ep_start_write(CDC_IN_EP, priv_USB.tx_buf, len);
		}
	}
}

LD_TASK void task_USB_IN(void *pData)
{
	priv_USB.rx_flag = 1;
	priv_USB.tx_flag = 1;

	do {
		vTaskDelay((TickType_t) 10);

		task_cdc_acm_flag_poll();
	}
	while (1);
}

extern QueueHandle_t USART_public_rx_queue();

void USB_startup()
{
	const struct usbd_endpoint	cdc_in_ep = {

		.ep_addr = CDC_IN_EP,
		.ep_cb = usbd_cdc_acm_bulk_in
	};

	const struct usbd_endpoint	cdc_out_ep = {

		.ep_addr = CDC_OUT_EP,
		.ep_cb = usbd_cdc_acm_bulk_out
	};

	/* Enable OTG_FS clock.
	 * */
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

	/* Enable OTG_FS pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_OTG_FS_DM);
	GPIO_set_mode_FUNCTION(GPIO_OTG_FS_DP);

	GPIO_set_mode_SPEED_FAST(GPIO_OTG_FS_DM);
	GPIO_set_mode_SPEED_FAST(GPIO_OTG_FS_DP);

	/* Alloc queues.
	 * */
	priv_USB.rx_queue = USART_public_rx_queue();
	priv_USB.tx_queue = xQueueCreate(320, sizeof(char));

	/* Create USB_IN task.
	 * */
	xTaskCreate(task_USB_IN, "USB_IN", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	/* Startup USB stack.
	 * */
	usbd_desc_register(cdc_acm_descriptor);
	usbd_add_interface(usbd_cdc_acm_init_intf(&priv_USB.intf0));
	usbd_add_interface(usbd_cdc_acm_init_intf(&priv_USB.intf1));
	usbd_add_endpoint(&cdc_in_ep);
	usbd_add_endpoint(&cdc_out_ep);

	usbd_initialize();

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(OTG_FS_IRQn, 11);
	NVIC_EnableIRQ(OTG_FS_IRQn);
}

void USB_putc(int c)
{
	char		xbyte = (char) c;

	GPIO_set_HIGH(GPIO_LED_ALERT);

	if (xQueueSendToBack(priv_USB.tx_queue, &xbyte, (TickType_t) 100) != pdTRUE) {

		log_TRACE("USB queue overflow" EOL);

		xQueueReset(priv_USB.tx_queue);
	}

	GPIO_set_LOW(GPIO_LED_ALERT);
}

