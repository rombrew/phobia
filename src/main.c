/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#include "hal/hal.h"
#include "hal/adc.h"
#include "hal/pwm.h"
#include "hal/usart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "main.h"
#include "lib.h"
#include "pmc.h"
#include "sh.h"

char 				ucHeap[configTOTAL_HEAP_SIZE];

main_t				ma;
pmc_t __CCM__			pm;

extern void xvprintf(io_ops_t *_io, const char *fmt, va_list ap);

void debugTRACE(const char *fmt, ...)
{
	va_list		ap;
	io_ops_t	ops = {

		.getc = NULL,
		.putc = &usart_debug_putc
	};

        va_start(ap, fmt);
	xvprintf(&ops, fmt, ap);
        va_end(ap);
}

void vAssertCalled(const char *file, int line)
{
	debugTRACE("freertos: assert %s:%i" EOL, file, line);
	halHalt();
}

void vApplicationMallocFailedHook()
{
	debugTRACE("freertos hook: heap allocation failed" EOL);
	halHalt();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
	debugTRACE("freertos hook: stack overflow in \"%s\" task" EOL, pcTaskName);
	halHalt();
}

void vApplicationIdleHook()
{
	if (ma.load_count_flag) {

		ma.load_count_value += 1;

		halFence();
	}
	else {

		halSleep();
	}
}

void taskINIT(void *pvParameters)
{
	halLED(LED_RED);

	ma.load_count_flag = 1;
	ma.load_count_value = 0;

	vTaskDelay(100);

	ma.load_count_flag = 0;
	ma.load_count_limit = ma.load_count_value * 10;

	ma.io_usart.getc = &usart_getc;
	ma.io_usart.putc = &usart_putc;
	iodef = &ma.io_usart;

	/* Config.
	 * */
	usartEnable(57600);

	halPWM.freq_hz = 60000;
	halPWM.dead_time_ns = 70;

	ma.av_default_time = .2f;
	ma.ap_J_measure_T = .1f;

	pwmEnable();
	adcEnable();

	pm.freq_hz = (float) halPWM.freq_hz;
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_resolution = halPWM.resolution;
	pm.pDC = &pwmDC;
	pm.pZ = &pwmZ;

	pmc_default(&pm);

	xTaskCreate(taskSH, "tSH", 1024, NULL, 1, NULL);

	vTaskDelete(NULL);
}

void adcIRQ()
{
	pmc_feedback(&pm, halADC.xA, halADC.xB);
	pmc_voltage(&pm, halADC.xSUPPLY);

	if (ma.pEX != NULL)
		ma.pEX();
}

void halMain()
{
	xTaskCreate(taskINIT, "tINIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

void ma_av_EH_8()
{
	int			j;

	if (ma.av_sample_N <= ma.av_sample_MAX) {

		for (j = 0; j < ma.av_variable_N; ++j)
			ma.av_VAL[j] += *ma.av_IN[j];

		ma.av_sample_N++;
	}
	else
		ma.pEX = NULL;
}

float ma_av_float_1(float *param, float time)
{
	ma.av_IN[0] = param;
	ma.av_VAL[0] = 0.f;
	ma.av_variable_N = 1;
	ma.av_sample_N = 0;
	ma.av_sample_MAX = pm.freq_hz * time;

	halFence();

	ma.pEX = &ma_av_EH_8;

	while (ma.pEX != NULL)
		vTaskDelay(1);

	ma.av_VAL[0] /= (float) ma.av_sample_N;

	return ma.av_VAL[0];
}

float ma_av_float_arg_1(float *param, const char *s)
{
	float			time = ma.av_default_time;

	stof(&time, s);

	return ma_av_float_1(param, time);
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

SH_DEF(hal_cpu_usage)
{
	float		pc;

	ma.load_count_flag = 1;
	ma.load_count_value = 0;

	vTaskDelay(1000);

	ma.load_count_flag = 0;
	pc = 100.f * (float) (ma.load_count_limit - ma.load_count_value)
		/ (float) ma.load_count_limit;

	printf("%1f %%" EOL, &pc);
}

SH_DEF(hal_reboot)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	vTaskDelay(100);
	halReset();
}

SH_DEF(hal_keycodes)
{
	int		xC;

	do {
		xC = iodef->getc();

		if (xC == K_ETX || xC == K_EOT)
			break;

		xprintf(iodef, "-- %i" EOL, xC);
	}
	while (1);
}

SH_DEF(hal_pwm_freq_hz)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.freq_hz, s) != NULL) {

		pwmDisable();
		pwmEnable();

		pm.freq_hz = (float) halPWM.freq_hz;
		pm.dT = 1.f / pm.freq_hz;
		pm.pwm_resolution = halPWM.resolution;
	}

	printf("%i (Hz)" EOL, halPWM.freq_hz);
}

SH_DEF(hal_pwm_dead_time_ns)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&halPWM.dead_time_ns, s) != NULL) {

		pwmDisable();
		pwmEnable();
	}

	printf("%i (tk) %i (ns)" EOL, halPWM.dead_time_tk, halPWM.dead_time_ns);
}

SH_DEF(hal_pwm_DC)
{
	const char	*tok;
	int		tokN = 0;
	int		xA, xB, xC, R;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	tok = s;
	s = stoi(&xA, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xB, tok);
	tokN += (s != NULL) ? 1 : 0;

	tok = strtok(tok, " ");
	s = stoi(&xC, tok);
	tokN += (s != NULL) ? 1 : 0;

	if (tokN == 3) {

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

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	if (stoi(&Z, s) != NULL) {

		pwmZ(Z);

		printf("Z %i" EOL, Z);
	}
}

