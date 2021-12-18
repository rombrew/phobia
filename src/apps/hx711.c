#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

/* This is the helper that reads HX711 ADC.
 * */

void ap_HX711(void *pData)
{
	const int		gpio_DOUT = GPIO_SPI_MISO;
	const int		gpio_PD_SCK = GPIO_SPI_SCK;

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
				hal_futile_ns(500);

				DOUT = GPIO_get_VALUE(gpio_DOUT);

				if (N == 0) {

					ADC = (DOUT != 0) ? -1 : 0;
				}
				else if (N < 24) {

					ADC <<= 1;
					ADC |= (DOUT != 0) ? 1 : 0;
				}

				GPIO_set_LOW(gpio_PD_SCK);
				hal_futile_ns(500);
			}

			/* Convert the ADC code into KG.
			 * */
			ap.hx711_kg = (float) ADC * ap.hx711_scale[1] + ap.hx711_scale[0];
		}
	}
	while (1);
}

SH_DEF(hx711_startup)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("HX711");

	if (xHandle == NULL) {

		xTaskCreate(ap_HX711, "HX711", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	}
}

SH_DEF(hx711_halt)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("HX711");

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

SH_DEF(hx711_adjust_null)
{
	float			S = 0.f;
	int			j, N = 100;

	/* Update MEAN value.
	 * */

	for (j = 0; j < N; ++j) {

		vTaskDelay((TickType_t) 10);

		S += ap.hx711_kg;
	}

	ap.hx711_scale[0] += - S / (float) N;

	reg_format(&regfile[ID_AP_HX711_SCALE_0]);
}

