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

#define MA_LOAD_COUNT_DELAY		200

char __CCM__ 			ucHeap[configTOTAL_HEAP_SIZE];

main_t __CCM__			ma;
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

extern int conf_block_load();

void taskINIT(void *pvParameters)
{
	int			rc_conf;

	halLED(LED_RED);

	ma.load_count_flag = 1;
	ma.load_count_value = 0;

	vTaskDelay(MA_LOAD_COUNT_DELAY);

	ma.load_count_flag = 0;
	ma.load_count_limit = ma.load_count_value;

	ma.io_usart.getc = &usart_getc;
	ma.io_usart.putc = &usart_putc;
	iodef = &ma.io_usart;

	rc_conf = conf_block_load();

	if (rc_conf < 0) {

		/* Default.
		 * */

		halPWM.freq_hz = 60000;
		halPWM.dead_time_ns = 70;
		halUSART_baudRate = 57600;
		ma.av_default_time = .2f;
		ma.ap_J_measure_T = .1f;
	}

	usartEnable();
	pwmEnable();

	pm.freq_hz = (float) halPWM.freq_hz;
	pm.dT = 1.f / pm.freq_hz;
	pm.pwm_resolution = halPWM.resolution;
	pm.pDC = &pwmDC;
	pm.pZ = &pwmZ;

	if (rc_conf < 0) {

		/* Default.
		 * */

		pmc_default(&pm);
	}

	adcEnable();

	xTaskCreate(taskSH, "tSH", 1024, NULL, 1, NULL);

	vTaskDelete(NULL);
}

void adcIRQ_feedback()
{
	pmc_feedback(&pm, halADC.sensor_A, halADC.sensor_B);
	pmc_voltage(&pm, halADC.supply_U);

	if (ma.pEX != NULL)
		ma.pEX();
}

static void
ma_thermal_job(int xNTC, int xTEMP, int xREF)
{
	float			fc, temp;

	ma.thermal_xAVG[0] += xNTC;
	ma.thermal_xAVG[1] += xTEMP;
	ma.thermal_xAVG[2] += xREF;

	ma.thermal_sample_N += 1;

	if (ma.thermal_sample_N >= ADC_THERMAL_FREQ_HZ) {

		fc = (float) (ma.thermal_xAVG[0] / ma.thermal_sample_N);
		ma.thermal_xAVG[0] = 0;

		temp = halADC_CONST.NTC[1] + halADC_CONST.NTC[0] * fc;
		temp = halADC_CONST.NTC[2] + temp * fc;
		temp = halADC_CONST.NTC[3] + temp * fc;
		temp = halADC_CONST.NTC[4] + temp * fc;
		temp = halADC_CONST.NTC[5] + temp * fc;
		temp = halADC_CONST.NTC[6] + temp * fc;
		temp = halADC_CONST.NTC[7] + temp * fc;
		ma.thermal_NTC = temp;

		fc = (float) (ma.thermal_xAVG[1] / ma.thermal_sample_N);
		ma.thermal_xAVG[1] = 0;

		ma.thermal_TEMP = halADC_CONST.TEMP_1 * fc + halADC_CONST.TEMP_0;

		fc = (float) (ma.thermal_xAVG[2] / ma.thermal_sample_N);
		ma.thermal_xAVG[2] = 0;

		ma.thermal_REF = halADC_CONST.REF_1 * fc;

		ma.thermal_sample_N = 0;
	}
}

void adcIRQ_thermal()
{
	ma_thermal_job(halADC.thermal_xNTC, halADC.thermal_xTEMP, halADC.in_xREF);
}

void halMain()
{
	xTaskCreate(taskINIT, "tINIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

void ma_av_EH()
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
	if (ma.pEX == NULL) {

		ma.av_IN[0] = param;
		ma.av_VAL[0] = 0.f;
		ma.av_variable_N = 1;
		ma.av_sample_N = 0;
		ma.av_sample_MAX = pm.freq_hz * time;

		halFence();
		ma.pEX = &ma_av_EH;

		while (ma.pEX != NULL)
			vTaskDelay(1);

		ma.av_VAL[0] /= (float) ma.av_sample_N;

		return ma.av_VAL[0];
	}
	else {

		return 0.f;
	}
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

	vTaskDelay(MA_LOAD_COUNT_DELAY);

	ma.load_count_flag = 0;
	pc = 100.f * (float) (ma.load_count_limit - ma.load_count_value)
		/ (float) ma.load_count_limit;

	printf("%1f %%" EOL, &pc);
}

SH_DEF(hal_reboot)
{
	AP_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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
	AP_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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
	AP_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	if (stoi(&halPWM.dead_time_ns, s) != NULL) {

		pwmDisable();
		pwmEnable();
	}

	printf("%i (tk) %i (ns)" EOL, halPWM.dead_time_tk, halPWM.dead_time_ns);
}

SH_DEF(hal_pwm_DC)
{
	int		xA, xB, xC, R;
	int		allf = 0;

	AP_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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

	AP_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	if (stoi(&Z, s) != NULL) {

		pwmZ(Z);

		printf("Z %i" EOL, Z);
	}
}

SH_DEF(hal_thermal)
{
	printf("NTC %1f (C)" EOL, &ma.thermal_NTC);
	printf("TEMP %1f (C)" EOL, &ma.thermal_TEMP);
	printf("REF %3f (V)" EOL, &ma.thermal_REF);
}

SH_DEF(hal_version)
{
	printf("VERSION %s" EOL, PMC_VERSION);
	printf("CONFIG %i" EOL, PMC_CONFIG_VERSION);
	printf("HAL %s" EOL, HAL_REVISION);
}

