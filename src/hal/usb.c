#include <stddef.h>
#include <stdint.h>

#include "hal.h"

#include "freertos/FreeRTOS.h"
#include "cmsis/stm32xx.h"

#include "cherry/usbd_core.h"
#include "cherry/usbd_cdc.h"

#define CDC_IN_EP		0x81
#define CDC_OUT_EP		0x02
#define CDC_INT_EP		0x83
#define CDC_DATA_SZ		64U

#define USBD_VID		0x0483	/* STMicroelectronics */
#define USBD_PID		0x5740	/* Virtual COM Port */
#define USBD_MAX_POWER		100
#define USBD_LANGID_STRING	0x0409	/* English (United States) */

typedef struct {

	QueueHandle_t		rx_queue;
	QueueHandle_t		tx_queue;

	struct usbd_interface	iface0;
	struct usbd_interface	iface1;

	uint8_t			rx_buf[CDC_DATA_SZ];
	uint8_t			tx_buf[CDC_DATA_SZ];

	int			rx_flag;
	int			tx_flag;
}
priv_USB_t;

static priv_USB_t		priv_USB;

static const uint8_t		cdc_acm_descriptor[] = {

	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01,
			USBD_VID, USBD_PID, 0x0100, 0x01),

	USB_CONFIG_DESCRIPTOR_INIT(0x09 + CDC_ACM_DESCRIPTOR_LEN, 0x02, 0x01,
			USB_CONFIG_SELF_POWERED, USBD_MAX_POWER),

	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, 0x02),

	USB_LANGID_INIT(USBD_LANGID_STRING),
	0x16,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'C', 0, 'h', 0, 'e', 0, 'r', 0, 'r', 0, 'y', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0,

	0x30,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'P', 0, 'h', 0, 'o', 0, 'b', 0, 'i', 0, 'a', 0, ' ', 0, 'M', 0, 'o', 0,
	't', 0, 'o', 0, 'r', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0,
	'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0,

	0x12,					/* bLength */
	USB_DESCRIPTOR_TYPE_STRING,		/* bDescriptorType */
	'5', 0, '2', 0, '1', 0, '5', 0, 'A', 0, '5', 0, '9', 0, 'F', 0,

	0x00
};

void usbd_configure_done_callback()
{
	usbd_ep_start_read(CDC_OUT_EP, priv_USB.rx_buf, CDC_DATA_SZ);
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
		priv_USB.rx_flag = 1;
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

extern QueueHandle_t USART_shared_rx_queue();

void USB_startup()
{
	const struct usbd_endpoint	cdc_out_ep = {

		.ep_addr = CDC_OUT_EP,
		.ep_cb = usbd_cdc_acm_bulk_out
	};

	const struct usbd_endpoint	cdc_in_ep = {

		.ep_addr = CDC_IN_EP,
		.ep_cb = usbd_cdc_acm_bulk_in
	};

	/* Enable OTG_FS clock.
	 * */
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

	/* Enable OTG_FS pins.
	 * */
	GPIO_set_mode_FUNCTION(GPIO_OTG_FS_DM);
	GPIO_set_mode_FUNCTION(GPIO_OTG_FS_DP);

	/* Alloc queues.
	 * */
	priv_USB.rx_queue = USART_shared_rx_queue();
	priv_USB.tx_queue = xQueueCreate(320, sizeof(char));

	/* Startup USB stack.
	 * */
	usbd_desc_register(cdc_acm_descriptor);
	usbd_add_interface(usbd_cdc_acm_init_iface(&priv_USB.iface0));
	usbd_add_interface(usbd_cdc_acm_init_iface(&priv_USB.iface1));
	usbd_add_endpoint(&cdc_out_ep);
	usbd_add_endpoint(&cdc_in_ep);
	usbd_initialize();

	/* Enable IRQ.
	 * */
	NVIC_SetPriority(OTG_FS_IRQn, 11);
	NVIC_EnableIRQ(OTG_FS_IRQn);
}

int USB_getc()
{
	char		xbyte;

	if (priv_USB.rx_flag != 0) {

		if (uxQueueSpacesAvailable(priv_USB.rx_queue) >= CDC_DATA_SZ) {

			priv_USB.rx_flag = 0;

			usbd_ep_start_read(CDC_OUT_EP, priv_USB.rx_buf, CDC_DATA_SZ);
		}
	}

	xQueueReceive(priv_USB.rx_queue, &xbyte, portMAX_DELAY);

	return (int) xbyte;
}

void USB_putc(int c)
{
	char		xbyte = (char) c;
	int		len = 0;

	GPIO_set_HIGH(GPIO_LED_ALERT);

	xQueueSendToBack(priv_USB.tx_queue, &xbyte, portMAX_DELAY);

	if (priv_USB.tx_flag == 0) {

		while (		len < CDC_DATA_SZ
				&& xQueueReceive(priv_USB.tx_queue,
					&priv_USB.tx_buf[len], (TickType_t) 0) == pdTRUE) {
			len++;
		}

		if (len > 0) {

			priv_USB.tx_flag = 1;

			usbd_ep_start_write(CDC_IN_EP, priv_USB.tx_buf, len);
		}
	}

	GPIO_set_LOW(GPIO_LED_ALERT);
}

