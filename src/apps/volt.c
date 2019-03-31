#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "shell.h"

void ap_VOLT(void *pData)
{
	int			xDC, xMIN, xMAX;
	TickType_t		xTk;

	xMIN = pm.dc_minimal;
	xMAX = (int) (pm.dc_resolution * pm.volt_maximal / pm.const_lpf_U);

	xDC = xMIN;
	xTk = 5000 / (xMAX - xMIN);

	PWM_set_DC(xDC, xDC, xDC);
	PWM_set_Z(0);

	do {
		vTaskDelay(xTk);

		xDC = (xDC < xMAX) ? xDC + 1 : xMIN;
		pm.vsi_X = xDC * pm.const_lpf_U / pm.dc_resolution;

		PWM_set_DC(xDC, xDC, xDC);
	}
	while (1);
}

SH_DEF(ap_volt_startup)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("ap_VOLT");

	if (xHandle == NULL) {

		xTaskCreate(ap_VOLT, "ap_VOLT", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	}
}

SH_DEF(ap_volt_halt)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("ap_VOLT");

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

