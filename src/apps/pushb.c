#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

/* The application allows you to control the speed using two push-buttons.
 * Such control is convenient for a drill machine or grass trimmer.
 *
 * [A]		- START or switch the speed.
 * [B]		- STOP.
 * [B]+[A]	- START in reverse.
 * */

void ap_PUSHB(void *pData)
{
	TickType_t		xWake;

	const int		gpio_A = GPIO_HALL_A;
	const int		gpio_B = GPIO_HALL_B;

	int			pushed_A, pushed_B;
	int			value_A, value_B;

	const float		rpm_table[] = {

		3000.f,
		4000.f,
		5000.f,
		6000.f,
		7000.f
	};

#define rpm_table_MAX		(sizeof(rpm_table) / sizeof(rpm_table[0]))

	int			N = 0, reverse = 0;
	float			rpm;

	GPIO_set_mode_INPUT(gpio_A);
	GPIO_set_mode_INPUT(gpio_B);

	pushed_A = GPIO_get_VALUE(gpio_A);
	pushed_B = GPIO_get_VALUE(gpio_B);

	xWake = xTaskGetTickCount();

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		value_A = GPIO_get_VALUE(gpio_A);
		value_B = GPIO_get_VALUE(gpio_B);

		/* Detect if button [A] is pressed.
		 * */
		if (pushed_A != 0 && value_A == 0) {

			pushed_A = 0;

			if (pm.lu_mode == PM_LU_DISABLED) {

				N = 0;
				reverse = (pushed_B == 0) ? -1 : 1;

				pm.fsm_req = PM_STATE_LU_STARTUP;
				pm_wait_for_IDLE();
			}
			else {
				N = (N < rpm_table_MAX - 1) ? N + 1 : 0;
			}

			rpm = rpm_table[N] * (float) reverse;

			reg_SET_F(ID_PM_S_SETPOINT_RPM, rpm);
		}
		else if (pushed_A == 0 && value_A != 0) {

			pushed_A = 1;
		}

		/* Detect if button [B] is pressed.
		 * */
		if (pushed_B != 0 && value_B == 0) {

			pushed_B = 0;

			if (pm.lu_mode != PM_LU_DISABLED) {

				reverse = 0;

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				pm_wait_for_IDLE();
			}
		}
		else if (pushed_B == 0 && value_B != 0) {

			pushed_B = 1;
		}
	}
	while (1);
}

SH_DEF(pushb_startup)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("PUSHB");

	if (xHandle == NULL) {

		xTaskCreate(ap_PUSHB, "PUSHB", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	}
}

SH_DEF(pushb_halt)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle("PUSHB");

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

