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
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "main.h"
#include "shell.h"

#define LOAD_COUNT_DELAY		100

application_t			ap;
pmc_t __RAM_CCM			pm;
telinfo_t			ti;

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap);

void lowTRACE(const char *fmt, ...)
{
	va_list		ap;
	io_ops_t	ops = {

		.getc = NULL,
		.putc = &USART_debug_putc
	};

        va_start(ap, fmt);
	xvprintf(&ops, fmt, ap);
        va_end(ap);
}

void vAssertCalled(const char *file, int line)
{
	lowTRACE("FreeRTOS: Wrong condition in %s:%i" EOL, file, line);
}

void vApplicationMallocFailedHook()
{
	lowTRACE("FreeRTOS Hook: Heap Allocation Failed" EOL);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
	lowTRACE("FreeRTOS Hook: Stack Overflow in \"%s\" Task" EOL, pcTaskName);
}

void taskTERM(void *pData)
{
	TickType_t			xWake;

	xWake = xTaskGetTickCount();

	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_NTC);
	GPIO_set_mode_ANALOG(GPIO_ADC_EXT_NTC);

	do {
		vTaskDelayUntil(&xWake, (TickType_t) 1000);

		ap.tc_PCB = ntc_temperature(&ap.ntc_PCB, ADC_get_VALUE(GPIO_ADC_PCB_NTC));
		ap.tc_EXT = ntc_temperature(&ap.ntc_EXT, ADC_get_VALUE(GPIO_ADC_EXT_NTC));
		ap.tc_TEMP = ADC_get_VALUE(GPIO_ADC_INTERNAL_TEMP);
	}
	while (1);
}

void taskINIT(void *pData)
{
	int			rc_flash;

	GPIO_set_mode_OUTPUT(GPIO_LED);
	GPIO_set_HIGH(GPIO_LED);

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(LOAD_COUNT_DELAY);

	ap.lc_flag = 0;
	ap.lc_idle = ap.lc_tick;

	ap.io_USART.getc = &USART_getc;
	ap.io_USART.putc = &USART_putc;
	iodef = &ap.io_USART;

	rc_flash = flash_block_load();

	if (rc_flash < 0) {

		/* Default.
		 * */

		hal.USART_baud_rate = 57600;
		hal.PWM_freq_hz = 62500;
		hal.PWM_dead_time_ns = 70;
		hal.ADC_reference_voltage = 3.3f;
		hal.ADC_current_shunt_resistance = 500E-6f;
		hal.ADC_amplifier_gain = 60.f;
		hal.ADC_voltage_divider_gain = 27.f / (470.f + 27.f);

		ap.ntc_PCB.r_balance = 10000.f;
		ap.ntc_PCB.r_ntc_0 = 10000.f;
		ap.ntc_PCB.ta_0 = 25.f;
		ap.ntc_PCB.betta = 3435.f;

		memcpy(&ap.ntc_EXT, &ap.ntc_PCB, sizeof(ntc_t));
	}

	ADC_startup();
	PWM_startup();
	USART_startup();

	pm.freq_hz = (float) hal.PWM_freq_hz;
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_R = hal.PWM_resolution;
	pm.pDC = &PWM_set_DC;
	pm.pZ = &PWM_set_Z;

	if (rc_flash < 0) {

		/* Default.
		 * */

		pm_default(&pm);
	}

	GPIO_set_mode_OUTPUT(GPIO_BOOST_12V);
	GPIO_set_HIGH(GPIO_BOOST_12V);

	GPIO_set_LOW(GPIO_LED);

	xTaskCreate(taskSH, "tSH", 1024, NULL, 1, NULL);
	xTaskCreate(taskTERM, "tTERM", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	vTaskDelete(NULL);
}

void ADC_IRQ()
{
	pmfb_t		fb;

	fb.current_A = hal.ADC_current_A;
	fb.current_B = hal.ADC_current_B;
	fb.voltage_U = hal.ADC_voltage_U;

	fb.voltage_A = hal.ADC_voltage_A;
	fb.voltage_B = hal.ADC_voltage_B;
	fb.voltage_C = hal.ADC_voltage_C;

	fb.hall_A = GPIO_get_VALUE(GPIO_HALL_A);
	fb.hall_B = GPIO_get_VALUE(GPIO_HALL_B);
	fb.hall_C = GPIO_get_VALUE(GPIO_HALL_C);

	pm_feedback(&pm, &fb);
	telinfo_capture(&ti);
}

void hal_main()
{
	xTaskCreate(taskINIT, "tINIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

SH_DEF(ap_uptime)
{
	TickType_t	xTick;
	int		Day, Hour, Min, Sec;

	xTick = xTaskGetTickCount();

	Sec = xTick / configTICK_RATE_HZ;
	Day = Sec / 86400;
	Sec -= Day * 86400;
	Hour = Sec / 3600;
	Sec -= Hour * 3600;
	Min = Sec / 60;
	Sec -= Min * 60;

	printf("%id %ih %im %is" EOL,
			Day, Hour, Min, Sec);
}

void vApplicationIdleHook()
{
	if (ap.lc_flag != 0) {

		ap.lc_tick++;
		hal_fence();
	}
	else {

		hal_sleep();
	}
}

SH_DEF(ap_cpu_usage)
{
	float		pc;

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(LOAD_COUNT_DELAY);

	ap.lc_flag = 0;

	pc = 100.f * (float) (ap.lc_idle - ap.lc_tick)
		/ (float) ap.lc_idle;

	printf("%1f %%" EOL, &pc);
}

SH_DEF(ap_thermal)
{
	reg_print_fmt(&regfile[ID_AP_TC_PCB], 1);
	reg_print_fmt(&regfile[ID_AP_TC_EXT], 1);
	reg_print_fmt(&regfile[ID_AP_TC_TEMP], 1);
}

SH_DEF(ap_reboot)
{
	if (pm.lu_mode != PM_LU_DISABLED)
		return ;

	hal_system_reset();
}

