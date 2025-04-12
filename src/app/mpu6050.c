#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

/* This is the helper task that reads MPU6050 inertial sensor.
 * */

LD_TASK void app_MPU6050(void *pData)
{
	volatile int		*lknob = (volatile int *) pData;

	if (SPI_is_halted(HW_SPI_EXT_ID) != HAL_OK) {

		printf("Unable to start application when SPI is busy" EOL);

		*lknob = PM_DISABLED;
		vTaskDelete(NULL);
	}

	do {
		/* TODO */
	}
	while (*lknob == PM_ENABLED);

	vTaskDelete(NULL);
}

