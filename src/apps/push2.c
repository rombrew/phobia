#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"

/* The application allows you to control the speed using TWO push-buttons.
 * Such control is convenient for a drill machine or other tool.
 *
 * [A]		- START or switch the speed.
 * [B]		- STOP.
 * [B] + [A]	- START in reverse.
 * */

#define PUSH_DEBOUNCE		5
#define PUSH_RPM_TABLE_MAX	(sizeof(rpm_table) / sizeof(rpm_table[0]) - 1U)

static const float		rpm_table[] = {

	3000.f,
	4000.f,
	5000.f,
	6000.f,
	7000.f
};

void app_PUSH2(void *pData)
{
	int			*onquit = (int *) pData;

	TickType_t		xWake;

	const int		gpio_A = GPIO_PPM;
	const int		gpio_B = GPIO_DIR;

	int			pushed_A, value_A, count_A, event_A;
	int			pushed_B, value_B, count_B, event_B;

	int			rpm_N = 0, direction = 0;
	float			total_rpm;

	GPIO_set_mode_INPUT(gpio_A);
	GPIO_set_mode_INPUT(gpio_B);

	pushed_A = 0;
	pushed_B = 0;

	count_A = 0;
	count_B = 0;

	event_A = 0;
	event_B = 0;

	xWake = xTaskGetTickCount();

	do {
		/* 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		value_A = GPIO_get_VALUE(gpio_A);
		value_B = GPIO_get_VALUE(gpio_B);

		/* Detect if button [A] is pressed.
		 * */
		if (pushed_A == 0) {

			count_A = (value_A == 0) ? count_A + 1 : 0;

			if (count_A >= PUSH_DEBOUNCE) {

				pushed_A = 1;
				count_A = 0;
				event_A = 1;
			}
		}
		else {
			count_A = (value_A != 0) ? count_A + 1 : 0;

			if (count_A >= PUSH_DEBOUNCE) {

				pushed_A = 0;
				count_A = 0;
			}
		}

		if (event_A != 0) {

			if (pm.lu_MODE == PM_LU_DISABLED) {

				rpm_N = 0;
				direction = (pushed_B == 0) ? -1 : 1;

				pm.fsm_req = PM_STATE_LU_STARTUP;
				pm_wait_for_IDLE();
			}
			else {
				rpm_N = (rpm_N < PUSH_RPM_TABLE_MAX) ? rpm_N + 1 : 0;
			}

			total_rpm = rpm_table[rpm_N] * (float) direction;

			reg_SET_F(ID_PM_S_SETPOINT_SPEED_RPM, total_rpm);

			event_A = 0;
		}

		/* Detect if button [B] is pressed.
		 * */
		if (pushed_B == 0) {

			count_B = (value_B == 0) ? count_B + 1 : 0;

			if (count_B >= PUSH_DEBOUNCE) {

				pushed_B = 1;
				count_B = 0;
				event_B = 1;
			}
		}
		else {
			count_B = (value_B != 0) ? count_B + 1 : 0;

			if (count_B >= PUSH_DEBOUNCE) {

				pushed_B = 0;
				count_B = 0;
			}
		}

		if (event_B != 0) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				direction = 0;

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				pm_wait_for_IDLE();
			}

			event_B = 0;
		}
	}
	while (*onquit == 0);

	vTaskDelete(NULL);
}

