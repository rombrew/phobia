#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

/* This is the helper task that reads AS5047 magnetic rotary encoder.
 * */

static int
AS5047_parity(uint16_t data)
{
	int		ones = 0;

	for (; data; ones++)
		data &= data - 1U;

	return (ones & 1) ? 0 : 1;
}

static int
AS5047_read_reg(int addr)
{
	uint16_t	txbuf, rxbuf;
	int		parity, fault, rdata;

	txbuf = addr & 0x3FFFU;
	txbuf |= AS5047_parity(txbuf) << 15;

	rxbuf = SPI_transfer(1, txbuf, 200);

	parity = (rxbuf & 0x8000U) >> 15;
	fault = (rxbuf & 0x4000U) ? 1 : 0;

	if (		parity == AS5047_parity(rxbuf & 0x7FFFU)
			&& fault == 0) {

		rdata = rxbuf & 0x3FFFU;
	}
	else {
		rdata = -1;
	}

	return rdata;
}

static int
AS5047_write_reg(int addr, int data)
{
	/*uint16_t	txbuf, rxbuf;
	int		fault;

	return fault;*/

	return 0;
}

void app_AS5047(void *pData)
{
	/* TODO */

	AS5047_write_reg(0x3FFF, 0);
	AS5047_read_reg(0x3FFF);
	/*EP = AS_read_reg(0x3FFF);

	ap.pulse_EP;*/
}

