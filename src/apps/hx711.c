#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

/* This is the helper task that reads HX711 ADC.
 * */

void app_HX711(void *pData)
{
	int			*onquit = (int *) pData;

	const int		gpio_DOUT = GPIO_SPI_EXT_MISO;
	const int		gpio_PD_SCK = GPIO_SPI_EXT_SCK;

	int			DOUT, ADC, N;

	GPIO_set_mode_INPUT(gpio_DOUT);
	GPIO_set_mode_OUTPUT(gpio_PD_SCK);

	GPIO_set_LOW(gpio_PD_SCK);

	do {
		vTaskDelay((TickType_t) 5);

		DOUT = GPIO_get_VALUE(gpio_DOUT);

		if (DOUT == 0) {

			/* Get ADC result.
			 *
			 * +-----------+---------+------+
			 * | Number of | Input   | Gain |
			 * | pulses    | channel |      |
			 * +-----------+---------+------+
			 * |    25     |    A    | 128  |
			 * +-----------+---------+------+
			 * |    26     |    B    |  32  |
			 * +-----------+---------+------+
			 * |    27     |    A    |  64  |
			 * +-----------+---------+------+
			 * */

			for (N = 0; N < 25; ++N) {

				GPIO_set_HIGH(gpio_PD_SCK);
				TIM_wait_ns(500);

				DOUT = GPIO_get_VALUE(gpio_DOUT);

				if (N == 0) {

					ADC = (DOUT != 0) ? -1 : 0;
				}
				else if (N < 24) {

					ADC <<= 1;
					ADC |= (DOUT != 0) ? 1 : 0;
				}

				GPIO_set_LOW(gpio_PD_SCK);
				TIM_wait_ns(500);
			}

			/* Convert the ADC code into KG.
			 * */
			ap.hx711_kg = (float) ADC * ap.hx711_scale[1] + ap.hx711_scale[0];
		}
	}
	while (*onquit == 0);

	GPIO_set_mode_INPUT(gpio_DOUT);
	GPIO_set_mode_INPUT(gpio_PD_SCK);

	vTaskDelete(NULL);
}

SH_DEF(hx711_adjust_offset)
{
	float			avgkg = 0.f;
	int			N;

	/* Adjust ZERO offset.
	 * */

	for (N = 0; N < 10; ++N) {

		vTaskDelay((TickType_t) 100);

		avgkg += ap.hx711_kg;
	}

	ap.hx711_scale[0] += - avgkg / (float) N;

	reg_format(&regfile[ID_AP_HX711_SCALE_0]);
}

