#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

/* This is the helper task that reads MPU6050 inertial sensor.
 * */

LD_TASK void app_MPU6050(void *pData)
{
	volatile int		*knob = (volatile int *) pData;

	do {
		/* TODO */
	}
	while (*knob != 0);

	vTaskDelete(NULL);
}

