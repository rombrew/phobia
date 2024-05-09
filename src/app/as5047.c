#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

/* This is the helper task that reads AS5047 magnetic rotary encoder.
 * */

#define AS5047_FREQUENCY	4000000U	/* (Hz) */

#define AS5047_PARD		0x8000U
#define AS5047_READ		0x4000U
#define AS5047_EF		0x4000U
#define AS5047_DATA		0x3FFFU

#define AS5047_REG_NOP		0x0000U
#define AS5047_REG_ERRFL	0x0001U
#define AS5047_REG_PROG		0x0003U
#define AS5047_REG_DIAAGC	0x3FFCU
#define AS5047_REG_MAG		0x3FFDU
#define AS5047_REG_ANGLEUNC	0x3FFEU
#define AS5047_REG_ANGLECOM	0x3FFFU

typedef struct {

	int		EP;

	uint16_t	txbuf[16] LD_DMA;
	uint16_t	rxbuf[16] LD_DMA;

	int		EF_errcnt;
	int		PA_errcnt;
}
priv_AS5047_t;

static priv_AS5047_t		priv_AS5047;

static int
AS5047_parity(uint16_t x)
{
	x ^= x >> 8;
	x ^= x >> 4;

	x = (0x6996U >> (x & 0xFU)) & 1U;

	return (int) x;
}

int AS5047_get_EP()
{
	const uint16_t	rxbuf_ANGLE = priv_AS5047.rxbuf[1];

	if ((rxbuf_ANGLE & AS5047_EF) == 0U) {

		if (AS5047_parity(rxbuf_ANGLE) == 0) {

			priv_AS5047.EP = (int) (rxbuf_ANGLE & AS5047_DATA);
		}
		else {
			priv_AS5047.PA_errcnt++;
		}
	}
	else {
		priv_AS5047.EF_errcnt++;
	}

	priv_AS5047.txbuf[0] = AS5047_PARD | AS5047_READ | AS5047_REG_ANGLECOM;
	priv_AS5047.txbuf[1] = AS5047_PARD | AS5047_READ | AS5047_REG_NOP;

	SPI_transfer_dma(HW_SPI_EXT_ID, priv_AS5047.txbuf, priv_AS5047.rxbuf, 2);

	return priv_AS5047.EP;
}

LD_TASK void app_AS5047(void *pData)
{
	volatile int		*knob = (volatile int *) pData;

	SPI_startup(HW_SPI_EXT_ID, AS5047_FREQUENCY, SPI_LOW_FALLING | SPI_DMA | SPI_NSS_ON);

	hal_memory_fence();

	ap.proc_get_EP = &AS5047_get_EP;

	do {
		vTaskDelay((TickType_t) 1000);

		if (		   priv_AS5047.EF_errcnt != 0
				|| priv_AS5047.PA_errcnt != 0) {

			if (		hal.DPS_mode == DPS_DRIVE_ON_SPI
					&& pm.lu_MODE != PM_LU_DISABLED) {

				if (		   priv_AS5047.EF_errcnt >= 10
						|| priv_AS5047.PA_errcnt >= 10) {

					pm.fsm_errno = PM_ERROR_SPI_DATA_FAULT;
					pm.fsm_req = PM_STATE_HALT;
				}
			}

			log_TRACE("AS5047 errate EF %i PA %i" EOL,
					priv_AS5047.EF_errcnt,
					priv_AS5047.PA_errcnt);

			priv_AS5047.EF_errcnt = 0;
			priv_AS5047.PA_errcnt = 0;
		}
	}
	while (*knob != 0);

	ap.proc_get_EP = NULL;

	vTaskDelay((TickType_t) 10);

	SPI_halt(HW_SPI_EXT_ID);

	vTaskDelete(NULL);
}

