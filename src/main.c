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
pmc_t __CCM__			pm;
telinfo_t			ti;

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap);

void debugTRACE(const char *fmt, ...)
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
	debugTRACE("FreeRTOS: ASSERT %s:%i \r\n", file, line);
}

void vApplicationMallocFailedHook()
{
	debugTRACE("FreeRTOS Hook: Heap Allocation Failed \r\n");
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
	debugTRACE("FreeRTOS Hook: Stack Overflow in \"%s\" Task \r\n", pcTaskName);
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
		hal.PWM_freq_hz = 60000;
		hal.PWM_dead_time_ns = 70;
		hal.ADC_reference_voltage = 3.3f;
		hal.ADC_resolution = 4096;
		hal.ADC_current_shunt_resistance = 500E-6f;
		hal.ADC_amplifier_gain = 60.f;
		hal.ADC_voltage_divider_gain = 27.f / (470.f + 27.f);

		ap.ntc_PCB.r_balance = 10000.f;
		ap.ntc_PCB.r_ntc_0 = 10000.f;
		ap.ntc_PCB.ta_0 = 25.f;
		ap.ntc_PCB.betta = 3435.f;

		memcpy(&ap.ntc_EXT, &ap.ntc_PCB, sizeof(ntc_t));
	}

	USART_enable();
	PWM_enable();

	GPIO_set_mode_OUTPUT(GPIO_BOOST_CONVERTER);
	GPIO_set_HIGH(GPIO_BOOST_CONVERTER);

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

	ADC_enable();
	GPIO_set_LOW(GPIO_LED);

	xTaskCreate(taskSH, "tSH", 1024, NULL, 1, NULL);

	vTaskDelete(NULL);
}

void ADC_IRQ()
{
	pmfb_t		fb;

	fb.current_A = hal.ADC_current_A;
	fb.current_B = hal.ADC_current_B;
	fb.voltage_U = hal.ADC_voltage_U;

	pm_feedback(&pm, &fb);
	telinfo_capture(&ti);
}

void hal_main()
{
	xTaskCreate(taskINIT, "tINIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

SH_DEF(hal_uptime)
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

SH_DEF(hal_cpu_usage)
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

SH_DEF(hal_thermal)
{
	float		temp_PCB, temp_EXT;

	temp_PCB = ntc_temperature(&ap.ntc_PCB, hal.ADC_thermal_PCB_NTC);
	temp_EXT = ntc_temperature(&ap.ntc_EXT, hal.ADC_thermal_EXT_NTC);

	printf("PCB NTC %1f (C)" EOL, &temp_PCB);
	printf("EXT NTC %1f (C)" EOL, &temp_EXT);
	printf("TEMP    %1f (C)" EOL, &hal.ADC_thermal_TEMP);
}

SH_DEF(hal_reboot)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return ;

	vTaskDelay(100);
	hal_system_reset();
}

/*
SH_DEF(hal_pwm_freq_hz)
{
	if (pm.lu_region != PM_LU_DISABLED)
		return ;

	if (stoi(&halPWM.freq_hz, s) != NULL) {

		pwmDisable();
		pwmEnable();

		pm.freq_hz = (float) halPWM.freq_hz;
		pm.dT = 1.f / pm.freq_hz;
		pm.pwm_R = halPWM.resolution;
	}

	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

SH_DEF(hal_pwm_dead_time_ns)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	if (stoi(&halPWM.dead_time_ns, s) != NULL) {

		pwmDisable();
		pwmEnable();
	}

	printf("%i (tk) %i (ns)" EOL, halPWM.dead_time_tk, halPWM.dead_time_ns);
}
*/
/*
SH_DEF(hal_pwm_DC)
{
	int		xA, xB, xC, R;
	int		allf = 0;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	if (stoi(&xA, s) != NULL) {

		s = strtok(s, " ");

		if (stoi(&xB, s) != NULL) {

			s = strtok(s, " ");

			if (stoi(&xC, s) != NULL) {

				allf = 1;
			}
		}
	}

	if (allf) {

		R = halPWM.resolution;

		xA = (xA < 0) ? 0 : (xA > R) ? R : xA;
		xB = (xB < 0) ? 0 : (xB > R) ? R : xB;
		xC = (xC < 0) ? 0 : (xC > R) ? R : xC;

		pwmDC(xA, xB, xC);

		printf("DC %i %i %i" EOL, xA, xB, xC);
	}
}

SH_DEF(hal_pwm_Z)
{
	int		Z;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	if (stoi(&Z, s) != NULL) {

		pwmZ(Z);

		printf("Z %i" EOL, Z);
	}
}
*/
