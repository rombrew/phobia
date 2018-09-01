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

#include "lib.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

void apPUSH(void *pData)
{
	TickType_t			xWake;

	unsigned long			pushed_A = 1;
	unsigned long			pushed_B = 1;

	unsigned long			value_A, value_B;

	float				rpm = 0.f;

	GPIO_set_mode_INPUT(GPIO_HALL_A);
	GPIO_set_mode_INPUT(GPIO_HALL_B);

	xWake = xTaskGetTickCount();

	do {
		/* Clock 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		value_A = GPIO_get_VALUE(GPIO_HALL_A);
		value_B = GPIO_get_VALUE(GPIO_HALL_B);

		/* Detect if buttons are pressed.
		 * */

		if (pushed_A != 0 && value_A == 0) {

			pushed_A = 0;

			if (pm.lu_mode == PM_LU_DISABLED) {

				if (pushed_B == 0) {

					rpm = -3000.f;
				}
				else {
					rpm = 3000.f;
				}

				pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
				pm_wait_for_IDLE();
			}
			else {
				if (rpm < 0.f) {

					rpm =	(rpm == -3000.f) ? -4000.f :
						(rpm == -4000.f) ? -5000.f :
						(rpm == -5000.f) ? -6000.f :
						(rpm == -6000.f) ? -7000.f : -3000.f ;
				}
				else {
					rpm =	(rpm == 3000.f) ? 4000.f :
						(rpm == 4000.f) ? 5000.f :
						(rpm == 5000.f) ? 6000.f :
						(rpm == 6000.f) ? 7000.f : 3000.f ;
				}
			}

			reg_SET(ID_PM_S_SETPOINT_RPM, &rpm);
		}
		else if (pushed_A == 0 && value_A != 0) {

			pushed_A = 1;
		}

		if (pushed_B != 0 && value_B == 0) {

			pushed_B = 0;

			if (pm.lu_mode != PM_LU_DISABLED) {

				pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
				pm_wait_for_IDLE();
			}
		}
		else if (pushed_B == 0 && value_B != 0) {

			pushed_B = 1;
		}
	}
	while (1);
}

SH_DEF(ap_pushbutton_startup)
{
	if (xTaskGetHandle("apPUSH") == NULL) {

		xTaskCreate(apPUSH, "apPUSH", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	}
}

