#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"

/* The application allows you to control the speed using two push-buttons.
 * Such control is convenient for drill machine and other tools.
 *
 * [A]		- START or step up the speed
 * [B]		- STOP
 * [B] + [A]	- START with reverse direction
 * */

#define BUTTON_DEBOUNCE		5

LD_TASK void app_BUTTON(void *pData)
{
	volatile int		*knob = (volatile int *) pData;

	TickType_t		xWake;

	const float		rpm_table[] = {

		1000.f, 2000.f, 3000.f, 4000.f, 5000.f
	};

	const int		gpio_A = GPIO_HALL_A;
	const int		gpio_B = GPIO_HALL_B;

	int			pushed_A, value_A, count_A, event_A;
	int			pushed_B, value_B, count_B, event_B;

	int			reverse, rpm_knob;
	float			total_rpm;

	GPIO_set_mode_INPUT(gpio_A);
	GPIO_set_mode_INPUT(gpio_B);

	pushed_A = 0;
	pushed_B = 0;

	count_A = 0;
	count_B = 0;

	event_A = 0;
	event_B = 0;

	reverse = 0;
	rpm_knob = 0;

	xWake = xTaskGetTickCount();

	do {
		/* 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		value_A = GPIO_get_STATE(gpio_A);
		value_B = GPIO_get_STATE(gpio_B);

		/* Detect if button [A] is pressed.
		 * */
		if (pushed_A == 0) {

			count_A = (value_A == 0) ? count_A + 1 : 0;

			if (count_A >= BUTTON_DEBOUNCE) {

				pushed_A = 1;
				count_A = 0;
				event_A = 1;
			}
		}
		else {
			count_A = (value_A != 0) ? count_A + 1 : 0;

			if (count_A >= BUTTON_DEBOUNCE) {

				pushed_A = 0;
				count_A = 0;
			}
		}

		if (event_A != 0) {

			if (pm.lu_MODE == PM_LU_DISABLED) {

				reverse = pushed_B;
				rpm_knob = 0;

				pm.fsm_req = PM_STATE_LU_STARTUP;

				vTaskDelay((TickType_t) 10);
			}

			if (pm.lu_MODE != PM_LU_DISABLED) {

				rpm_knob = (rpm_knob < 4) ? rpm_knob + 1 : 0;

				if (reverse != 0) {

					total_rpm = - rpm_table[rpm_knob];
				}
				else {
					total_rpm = rpm_table[rpm_knob];
				}

				reg_SET_F(ID_PM_S_SETPOINT_SPEED_RPM, total_rpm);
			}

			event_A = 0;
		}

		/* Detect if button [B] is pressed.
		 * */
		if (pushed_B == 0) {

			count_B = (value_B == 0) ? count_B + 1 : 0;

			if (count_B >= BUTTON_DEBOUNCE) {

				pushed_B = 1;
				count_B = 0;
				event_B = 1;
			}
		}
		else {
			count_B = (value_B != 0) ? count_B + 1 : 0;

			if (count_B >= BUTTON_DEBOUNCE) {

				pushed_B = 0;
				count_B = 0;
			}
		}

		if (event_B != 0) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;

				vTaskDelay((TickType_t) 10);
			}

			event_B = 0;
		}
	}
	while (*knob != 0);

	vTaskDelete(NULL);
}

