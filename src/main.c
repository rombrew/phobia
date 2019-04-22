#include <stddef.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "main.h"
#include "shell.h"

#define LOAD_COUNT_DELAY		((TickType_t) 100)

application_t			ap;
pmc_t 				pm __section_ccmram;
tel_t				ti;

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap);

void log_TRACE(const char *fmt, ...)
{
	va_list		ap;
	io_ops_t	ops = {

		.getc = NULL,
		.putc = &log_putc
	};

        va_start(ap, fmt);
	xvprintf(&ops, fmt, ap);
        va_end(ap);
}

void vAssertCalled(const char *file, int line)
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS: Assert %s:%i" EOL, file, line);

	hal_system_reset();
}

void vApplicationMallocFailedHook()
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS: Heap Allocation Failed" EOL);

	hal_system_reset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS: Stack Overflow in \"%s\" task" EOL, pcTaskName);

	hal_system_reset();
}

void task_TERM(void *pData)
{
	TickType_t		xWake;
	float			i_temp_PCB, i_temp_EXT;

	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_NTC);
	GPIO_set_mode_ANALOG(GPIO_ADC_EXT_NTC);

	xWake = xTaskGetTickCount();

	i_temp_PCB = PM_UNRESTRICTED;
	i_temp_EXT = PM_UNRESTRICTED;

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		ap.temp_PCB = ntc_temperature(&ap.ntc_PCB, ADC_get_VALUE(GPIO_ADC_PCB_NTC));
		ap.temp_EXT = ntc_temperature(&ap.ntc_EXT, ADC_get_VALUE(GPIO_ADC_EXT_NTC));
		ap.temp_INT = ADC_get_VALUE(GPIO_ADC_INTERNAL_TEMP);

		if (pm.lu_mode != PM_LU_DISABLED) {

			/* Derate current if PCB is overheat.
			 * */
			if (ap.temp_PCB > ap.heat_PCB) {

				i_temp_PCB = ap.heat_PCB_derated;
			}
			else if (ap.temp_PCB < (ap.heat_PCB - ap.heat_hysteresis)) {

				i_temp_PCB = PM_UNRESTRICTED;
			}

			/* Derate current if EXT is overheat.
			 * */
			if (ap.temp_EXT > ap.heat_EXT) {

				i_temp_EXT = ap.heat_EXT_derated;
			}
			else if (ap.temp_EXT < (ap.heat_EXT - ap.heat_hysteresis)) {

				i_temp_EXT = PM_UNRESTRICTED;
			}

			pm.i_derated = (i_temp_PCB < i_temp_EXT) ? i_temp_PCB : i_temp_EXT;

			/* Enable FAN if PCB is overheat.
			 * */
			if (ap.temp_PCB > ap.heat_PCB_FAN) {

			}
			else if (ap.temp_PCB < (ap.heat_PCB_FAN - ap.heat_hysteresis)) {

			}

			/* Derate power consumption if battery voltage is low.
			 * */
			if (pm.const_lpf_U < ap.batt_voltage_low) {

				pm.i_watt_derated = ap.batt_derated;
			}
			else if (pm.const_lpf_U > (ap.batt_voltage_low + ap.batt_hysteresis)) {

				pm.i_watt_derated = PM_UNRESTRICTED;
			}
		}
	}
	while (1);
}

void task_ANALOG(void *pData)
{
	TickType_t		xWake, xTime;
	float			voltage, control, range, scaled;

	GPIO_set_mode_ANALOG(GPIO_ADC_ANALOG);

	xWake = xTaskGetTickCount();
	xTime = 0;

	do {
		/* 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		if (ap.analog_reg_ID != ID_NULL) {

			voltage = ADC_get_VALUE(GPIO_ADC_ANALOG) * hal.ADC_reference_voltage;

			range = ap.analog_voltage_range[1] - ap.analog_voltage_range[0];
			scaled = (voltage - ap.analog_voltage_range[0]) / range;
			scaled = (scaled < 0.f) ? 0.f : (scaled > 1.f) ? 1.f : scaled;

			range = ap.analog_control_range[1] - ap.analog_control_range[0];
			control = ap.analog_control_range[0] + range * scaled;

			reg_SET(ap.analog_reg_ID, &control);

			if (		control > ap.analog_safe_range[0]
					&& control < ap.analog_safe_range[1]) {

				if (xTime < (TickType_t) (ap.analog_timeout * 1000.f)) {

					pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
				}
				else {
					pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
				}

				xTime += (TickType_t) 10;
			}
			else {
				xTime = (TickType_t) 0;
			}
		}
		else {
			vTaskDelayUntil(&xWake, (TickType_t) 100);
		}
	}
	while (1);
}

void task_INIT(void *pData)
{
	int			rc_flash;

	GPIO_set_mode_OUTPUT(GPIO_LED);
	GPIO_set_HIGH(GPIO_LED);

	GPIO_set_mode_OUTPUT(GPIO_BOOST_12V);
	GPIO_set_HIGH(GPIO_BOOST_12V);

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(LOAD_COUNT_DELAY);

	ap.lc_flag = 0;
	ap.lc_idle = ap.lc_tick;

	ap.io_USART.getc = &USART_getc;
	ap.io_USART.putc = &USART_putc;
	iodef = &ap.io_USART;

	if (log_validate() != 0) {

		/* Slow down the startup to indicate a problem.
		 * */
		vTaskDelay((TickType_t) 1000);
	}

	rc_flash = flash_block_load();

	if (rc_flash < 0) {

		/* Default.
		 * */

		hal.USART_baud_rate = 57600;
		hal.PWM_frequency = 30000.f;
		hal.PWM_deadtime = 190;
		hal.ADC_reference_voltage = 3.3f;
		hal.ADC_shunt_resistance = 340E-6f;
		hal.ADC_amplifier_gain = 60.f;
		hal.ADC_voltage_ratio = 27.f / (470.f + 27.f);

		hal.PPM_mode = PPM_DISABLED;
		hal.PPM_timebase = 2000000UL;

		ap.ppm_reg_ID = ID_PM_S_SETPOINT_PC;
		ap.ppm_pulse_range[0] = 1000.f;
		ap.ppm_pulse_range[1] = 2000.f;
		ap.ppm_control_range[0] = 0.f;
		ap.ppm_control_range[1] = 100.f;
		ap.ppm_safe_range[0] = -1.f;
		ap.ppm_safe_range[1] = 1.f;

		ap.analog_reg_ID = ID_NULL;
		ap.analog_voltage_range[0] = 0.f;
		ap.analog_voltage_range[1] = 5.f;
		ap.analog_control_range[0] = 0.f;
		ap.analog_control_range[1] = 100.f;
		ap.analog_safe_range[0] = -1.f;
		ap.analog_safe_range[1] = 1.f;
		ap.analog_timeout = 5.f;

		ap.ntc_PCB.r_balance = 10000.f;
		ap.ntc_PCB.r_ntc_0 = 10000.f;
		ap.ntc_PCB.ta_0 = 25.f;
		ap.ntc_PCB.betta = 3435.f;

		memcpy(&ap.ntc_EXT, &ap.ntc_PCB, sizeof(ntc_t));

		ap.heat_PCB = 120.f;
		ap.heat_PCB_derated = 30.f;
		ap.heat_EXT = 90.f;
		ap.heat_EXT_derated = 30.f;
		ap.heat_PCB_FAN = 60.f;
		ap.heat_hysteresis = 5.f;

		ap.batt_voltage_low = 6.0f;
		ap.batt_hysteresis = 1.f;
		ap.batt_derated = 50.f;

		ap.pull_adjust[0] = 0.f;
		ap.pull_adjust[1] = 4.545E-3f;
	}

	USART_startup();
	ADC_startup();
	PWM_startup();

	pm.freq_hz = hal.PWM_frequency;
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = hal.PWM_resolution;
	pm.proc_set_DC = &PWM_set_DC;
	pm.proc_set_Z = &PWM_set_Z;

	if (rc_flash < 0) {

		/* Default.
		 * */

		reg_SET_F(ID_PM_FB_CURRENT_CLAMP, -1.f);

		pm_default(&pm);
		tel_reg_default(&ti);
	}

	if (hal.PPM_mode != PPM_DISABLED) {

		PPM_startup();
	}

	WD_startup();

	ADC_irq_unlock();
	GPIO_set_LOW(GPIO_LED);

	pm_fsm_req(&pm, PM_STATE_ZERO_DRIFT);

	xTaskCreate(task_TERM, "TERM", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_ANALOG, "ANALOG", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_SH, "SH", 400, NULL, 1, NULL);

	vTaskDelete(NULL);
}

static void
input_PULSE_WIDTH()
{
	float		pulse, control, range, scaled;

	if (hal.PPM_signal_caught != 0) {

		pulse = PPM_get_PULSE();

		if (pulse != ap.ppm_pulse_cached) {

			ap.ppm_pulse_cached = pulse;

			range = ap.ppm_pulse_range[1] - ap.ppm_pulse_range[0];
			scaled = (pulse - ap.ppm_pulse_range[0]) / range;
			scaled = (scaled < 0.f) ? 0.f : (scaled > 1.f) ? 1.f : scaled;

			range = ap.ppm_control_range[1] - ap.ppm_control_range[0];
			control = ap.ppm_control_range[0] + range * scaled;

			reg_SET(ap.ppm_reg_ID, &control);

			if (pm.lu_mode == PM_LU_DISABLED) {

				if (		control > ap.ppm_safe_range[0]
						&& control < ap.ppm_safe_range[1]) {

					pm_fsm_req(&pm, PM_STATE_LU_INITIATE);
				}
			}
		}
	}
	else {
		pm_fsm_req(&pm, PM_STATE_LU_SHUTDOWN);
	}
}

static void
input_STEP_DIR()
{
	/* TODO */
}

static void
input_CONTROL_QEP()
{
	/* TODO */
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

	if (hal.HALL_mode == HALL_SENSOR) {

		fb.hall_code = GPIO_get_HALL();
	}

	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		input_PULSE_WIDTH();
	}
	else if (hal.PPM_mode == PPM_STEP_DIR) {

		input_STEP_DIR();
	}
	else if (hal.PPM_mode == PPM_CONTROL_QEP) {

		input_CONTROL_QEP();
	}

	pm_feedback(&pm, &fb);
	tel_reg_grab(&ti);

	WD_kick();
}

void app_MAIN()
{
	xTaskCreate(task_INIT, "INIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

SH_DEF(rtos_uptime)
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

	printf("%id %ih %im %is" EOL, Day, Hour, Min, Sec);
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

SH_DEF(rtos_cpu_usage)
{
	float		pc;

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(LOAD_COUNT_DELAY);

	ap.lc_flag = 0;

	pc = 100.f * (float) (ap.lc_idle - ap.lc_tick)
		/ (float) ap.lc_idle;

	printf("%1f (%%)" EOL, &pc);
}

SH_DEF(rtos_list)
{
	TaskStatus_t		*pLIST;
	int			xSIZE, xState, N;

	xSIZE = uxTaskGetNumberOfTasks();
	pLIST = pvPortMalloc(xSIZE * sizeof(TaskStatus_t));

	if (pLIST != NULL) {

		xSIZE = uxTaskGetSystemState(pLIST, xSIZE, NULL);

		printf("TCB ID Name Stat Prio Stack Free" EOL);

		for (N = 0; N < xSIZE; ++N) {

			switch (pLIST[N].eCurrentState) {

				case eRunning:
					xState = 'R';
					break;

				case eReady:
					xState = 'E';
					break;

				case eBlocked:
					xState = 'B';
					break;

				case eSuspended:
					xState = 'S';
					break;

				case eDeleted:
					xState = 'D';
					break;

				case eInvalid:
				default:
					xState = 'N';
					break;
			}

			printf("%8x %i %s %c %i %8x %i" EOL,
					(unsigned long) pLIST[N].xHandle,
					(int) pLIST[N].xTaskNumber,
					(const char *) pLIST[N].pcTaskName,
					(int) xState,
					(int) pLIST[N].uxCurrentPriority,
					(unsigned long) pLIST[N].pxStackBase,
					(int) pLIST[N].usStackHighWaterMark);
		}

		vPortFree(pLIST);
	}
}

SH_DEF(rtos_kill)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle(s);

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

SH_DEF(rtos_heap)
{
	printf("Free %i (Minimum %i)" EOL, xPortGetFreeHeapSize(),
			xPortGetMinimumEverFreeHeapSize());
}

SH_DEF(rtos_test_hardfault)
{
	printf("%i" EOL, *((int *) 0x55555555UL));
}

SH_DEF(rtos_test_watchdog)
{
	ADC_irq_lock();
}

SH_DEF(rtos_test_printf)
{
	printf("%8x" EOL, 0x00001111UL);
	printf("%8x" EOL, 0x12341234UL);
	printf("%8x" EOL, 0x77777777UL);
	printf("%8x" EOL, 0x88888888UL);
	printf("%8x" EOL, 0xFFFFFFFFUL);
}

SH_DEF(rtos_log)
{
	if (log_validate() != 0) {

		puts(log.text);
		puts(EOL);
	}
}

SH_DEF(rtos_log_reset)
{
	if (log_validate() != 0) {

		log.signature = 0;
	}
}

SH_DEF(rtos_reboot)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	hal_system_reset();
}

