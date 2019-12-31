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
	xMAX = (int) (pm.dc_resolution * pm.tvm_range);

	xDC = xMIN;
	xTk = 5000 / (xMAX - xMIN);

	PWM_set_DC(xDC, xDC, xDC);
	PWM_set_Z(0);

	pm.vsi_UF = 0;
	pm.vsi_AZ = 0;
	pm.vsi_BZ = 0;
	pm.vsi_CZ = 0;

	do {
		vTaskDelay(xTk);

		xDC = (xDC < xMAX) ? xDC + 1 : xMIN;
		pm.vsi_X = xDC * pm.const_fb_U * pm.ts_inverted;

		PWM_set_DC(xDC, xDC, xDC);
	}
	while (1);
}

SH_DEF(ap_volt_startup)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("VOLT");

	if (xHandle == NULL) {

		xTaskCreate(ap_VOLT, "VOLT", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	}
}

SH_DEF(ap_volt_halt)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("VOLT");

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

