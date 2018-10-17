/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

void apHX711(void *pData)
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
			 * */

			for (N = 0; N < 25; ++N) {

				GPIO_set_HIGH(gpio_PD_SCK);
				hal_delay_us(1);

				DOUT = GPIO_get_VALUE(gpio_DOUT);

				if (N == 0) {

					ADC = (DOUT != 0) ? -1 : 0;
				}
				else if (N < 24) {

					ADC <<= 1;
					ADC |= (DOUT != 0) ? 1 : 0;
				}

				GPIO_set_LOW(gpio_PD_SCK);
				hal_delay_us(1);
			}

			ap.load_thrust_gram = (float) ADC * ap.load_transform[1]
				+ ap.load_transform[0];
		}
	}
	while (1);
}

static TaskHandle_t		xHandle;

SH_DEF(ap_hx711_startup)
{
	if (xHandle == NULL) {

		xTaskCreate(apHX711, "apHX711", configMINIMAL_STACK_SIZE, NULL, 1, &xHandle);
	}
}

SH_DEF(ap_hx711_halt)
{
	if (xHandle != NULL) {

		vTaskDelete(xHandle);
		xHandle = NULL;
	}
}

SH_DEF(ap_hx711_drift)
{
	float			drift = 0.f;
	int			J, N = 100;

	for (J = 0; J < N; ++J) {

		vTaskDelay((TickType_t) 10);

		drift += ap.load_thrust_gram;
	}

	ap.load_transform[0] += - drift / (float) N;
	reg_format(&regfile[ID_AP_LOAD_TRANSFORM_0]);
}

