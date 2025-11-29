#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"

#include "taskdefs.h"

/* This is the helper task that reads MPU6050 inertial sensor.
 * */

AP_TASK_DEF(MPU6050)
{
	AP_KNOB(knob);

	if (SPI_is_halted(HW_SPI_EXT_ID) != HAL_OK) {

		printf("Unable to start application when SPI is busy" EOL);

		AP_TERMINATE(knob);
	}

	do {
		/* TODO */
	}
	while (AP_CONDITION(knob));

	AP_TERMINATE(knob);
}

