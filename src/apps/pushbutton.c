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

void taskPUSH(void *pData)
{
	TickType_t			xWake;

	unsigned long			bA = 0;
	unsigned long			bB = 0;

	float				rpm = 0.f;

	GPIO_set_mode_INPUT(GPIO_HALL_A);
	GPIO_set_mode_INPUT(GPIO_HALL_B);

	xWake = xTaskGetTickCount();

	do {
		/* Clock 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		bA = (bA & 8UL) | ((bA << 1) & 7UL);
		bB = (bB & 8UL) | ((bB << 1) & 7UL);

		bA |= (GPIO_get_VALUE(GPIO_HALL_A) != 0) ? 1UL : 0UL;
		bB |= (GPIO_get_VALUE(GPIO_HALL_B) != 0) ? 1UL : 0UL;

		/* Detect when the button is pressed for 3 clock cycles.
		 * */

		if ((bA & 15UL) == 7UL) {

			bA |= 8UL;

			if (rpm == 0.f) {

				if ((bB & 7) == 7) {

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
		else if ((bA & 15UL) == 8UL) {

			bA &= ~8UL;
		}

		if ((bB & 15UL) == 7UL) {

			bB |= 8UL;

			rpm = 0.f;
			pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
			pm_wait_for_IDLE();
		}
		else if ((bB & 15UL) == 8UL) {

			bB &= ~8UL;
		}
	}
	while (1);
}

SH_DEF(init_pushbutton)
{
	xTaskCreate(taskSH, "tPUSH", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

