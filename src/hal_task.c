/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include "hal_task.h"
#include "lib.h"
#include "pm_control.h"
#include "shell.h"

#define TS_LOAD_COUNT_DELAY		200

char __CCM__ 			ucHeap[configTOTAL_HEAP_SIZE];

task_data_t __CCM__		ts;
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
	if (ts.load_count_flag) {

		ts.load_count_value += 1;

		halFence();
	}
	else {

		halSleep();
	}
}

void taskAPP(void *pData)
{
	int		xENC;
	int		RPM;
	int		iDIR = 1;

	do {
		vTaskDelay(50);

		xENC = halENC();

		if ((xENC & ENC_B) == 0) {

			if (RPM == 0) {

				if ((xENC & ENC_A) == 0) {

					iDIR = (iDIR < 0) ? 1 : -1;
					vTaskDelay(500);
				}
			}
			else {
				RPM = 0;
				pm.s_set_point = 0;
				vTaskDelay(300);

				pmc_request(&pm, PMC_STATE_STOP);
				vTaskDelay(200);
			}
		}
		else if ((xENC & ENC_A) == 0) {

			if (pm.lu_region == PMC_LU_DISABLED) {

				pm.m_errno = PMC_OK;

				pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
				vTaskDelay(250);

				pmc_request(&pm, PMC_STATE_START);
				vTaskDelay(100);

				RPM = 3000;
			}
			else {
				RPM =	(RPM == 3000) ? 4000 :
					(RPM == 4000) ? 5000 :
					(RPM == 5000) ? 6000 :
					(RPM == 6000) ? 7000 : 3000 ;
			}

			pm.s_set_point = .10471976f * iDIR * RPM * pm.const_Zp;
			vTaskDelay(500);
		}

		if (pm.lu_region != PMC_LU_DISABLED) {

			halLED(LED_GREEN);
		}
		else {
			halLED(LED_RED);
		}
	}
	while (1);
}

extern int conf_block_load();

void taskINIT(void *pData)
{
	int			rc_conf;

	halLED(LED_BLUE);

	ts.load_count_flag = 1;
	ts.load_count_value = 0;

	vTaskDelay(TS_LOAD_COUNT_DELAY);

	ts.load_count_flag = 0;
	ts.load_count_limit = ts.load_count_value;

	ts.io_usart.getc = &usart_getc;
	ts.io_usart.putc = &usart_putc;
	iodef = &ts.io_usart;

	rc_conf = conf_block_load();

	if (rc_conf < 0) {

		/* Default.
		 * */

		halPWM.freq_hz = 60000;
		halPWM.dead_time_ns = 70;
		halUSART_baudRate = 57600;
		ts.av_default_time = .2f;
	}

	usartEnable();
	pwmEnable();
	halBoostConverter(1);

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
	//xTaskCreate(taskAPP, "tAPP", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	vTaskDelete(NULL);
}

extern void tel_collect();

void adcIRQ_feedback()
{
	pmc_feedback(&pm, halADC.sensor_A, halADC.sensor_B);
	pmc_voltage(&pm, halADC.supply_U);

	tel_collect();

	if (ts.p_irq_callback != NULL)
		ts.p_irq_callback();
}

void halMain()
{
	xTaskCreate(taskINIT, "tINIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

void ts_thermal_convert()
{
	float			fc, temp;

	fc = (float) halADC.thermal_xNTC;
	temp = halADC_CONST.NTC[1] + halADC_CONST.NTC[0] * fc;
	temp = halADC_CONST.NTC[2] + temp * fc;
	temp = halADC_CONST.NTC[3] + temp * fc;
	temp = halADC_CONST.NTC[4] + temp * fc;
	temp = halADC_CONST.NTC[5] + temp * fc;
	temp = halADC_CONST.NTC[6] + temp * fc;
	temp = halADC_CONST.NTC[7] + temp * fc;
	ts.thermal_NTC = temp;

	fc = (float) halADC.thermal_xTEMP;
	ts.thermal_TEMP = halADC_CONST.TEMP_1 * fc + halADC_CONST.TEMP_0;
}

void ts_av_handler()
{
	int			j;

	if (ts.av_sample_N <= ts.av_sample_MAX) {

		for (j = 0; j < ts.av_variable_N; ++j)
			ts.av_VAL[j] += *ts.av_IN[j];

		ts.av_sample_N++;
	}
	else
		ts.p_irq_callback = NULL;
}

float ts_av_float_1(float *param, float time)
{
	if (ts.p_irq_callback == NULL) {

		ts.av_IN[0] = param;
		ts.av_VAL[0] = 0.f;
		ts.av_variable_N = 1;
		ts.av_sample_N = 0;
		ts.av_sample_MAX = pm.freq_hz * time;

		halFence();
		ts.p_irq_callback = &ts_av_handler;

		while (ts.p_irq_callback != NULL)
			vTaskDelay(1);

		ts.av_VAL[0] /= (float) ts.av_sample_N;

		return ts.av_VAL[0];
	}
	else {

		return 0.f;
	}
}

int ts_av_float_4(float *param_0, float *param_1, float *param_2,
		float *param_3, float *result, float time)
{
	if (ts.p_irq_callback == NULL) {

		ts.av_IN[0] = param_0;
		ts.av_IN[1] = param_1;
		ts.av_IN[2] = param_2;
		ts.av_IN[3] = param_3;

		ts.av_VAL[0] = 0.f;
		ts.av_VAL[1] = 0.f;
		ts.av_VAL[2] = 0.f;
		ts.av_VAL[3] = 0.f;

		ts.av_variable_N = 4;
		ts.av_sample_N = 0;
		ts.av_sample_MAX = pm.freq_hz * time;

		halFence();
		ts.p_irq_callback = &ts_av_handler;

		while (ts.p_irq_callback != NULL)
			vTaskDelay(1);

		result[0] = ts.av_VAL[0] / (float) ts.av_sample_N;
		result[1] = ts.av_VAL[1] / (float) ts.av_sample_N;
		result[2] = ts.av_VAL[2] / (float) ts.av_sample_N;
		result[3] = ts.av_VAL[3] / (float) ts.av_sample_N;

		return 4;
	}
	else {

		return 0;
	}
}

float ts_av_float_arg_1(float *param, const char *s)
{
	float			time = ts.av_default_time;

	stof(&time, s);

	return ts_av_float_1(param, time);
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

	ts.load_count_flag = 1;
	ts.load_count_value = 0;

	vTaskDelay(TS_LOAD_COUNT_DELAY);

	ts.load_count_flag = 0;
	pc = 100.f * (float) (ts.load_count_limit - ts.load_count_value)
		/ (float) ts.load_count_limit;

	printf("%1f %%" EOL, &pc);
}

SH_DEF(hal_reboot)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

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

SH_DEF(hal_thermal)
{
	ts_thermal_convert();

	printf("NTC %1f (C)" EOL, &ts.thermal_NTC);
	printf("TEMP %1f (C)" EOL, &ts.thermal_TEMP);
}

