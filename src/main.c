#include <stddef.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "main.h"
#include "ifcan.h"
#include "libc.h"
#include "shell.h"

#define LOAD_COUNT_DELAY		((TickType_t) 100)

app_main_t			ap;
pmc_t 				pm	LD_CCMRAM;
TLM_t				tlm;

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap);

void log_TRACE(const char *fmt, ...)
{
	va_list		ap;
	io_ops_t	ops = {

		.getc = NULL,
		.putc = &log_putc
	};

	xprintf(&ops, "[%i.%i] ", log.boot_COUNT, xTaskGetTickCount());

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
	log_TRACE("FreeRTOS: Stack Overflow in %8x task" EOL, (u32_t) xTask);

	hal_system_reset();
}

float ADC_get_analog_ANG()
{
	float			analog;

	analog = ADC_get_VALUE(GPIO_ADC_ANALOG_ANG);

	return analog * ap.analog_const_GU;
}

float ADC_get_analog_BRK()
{
	float			analog;

	analog = ADC_get_VALUE(GPIO_ADC_ANALOG_BRK);

	return analog * ap.analog_const_GU;
}

void task_TERM(void *pData)
{
	TickType_t		xWake;
	float			i_temp_PCB, i_temp_EXT;

	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_NTC);
	GPIO_set_mode_ANALOG(GPIO_ADC_EXT_NTC);

	xWake = xTaskGetTickCount();

	i_temp_PCB = PM_MAX_F;
	i_temp_EXT = PM_MAX_F;

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

				i_temp_PCB = ap.heat_PCB_derated_1;
			}
			else if (ap.temp_PCB < (ap.heat_PCB - ap.heat_recovery_gap)) {

				i_temp_PCB = PM_MAX_F;
			}

			/* Derate current if EXT is overheat.
			 * */
			if (ap.temp_EXT > ap.heat_EXT) {

				i_temp_EXT = ap.heat_EXT_derated_1;
			}
			else if (ap.temp_EXT < (ap.heat_EXT - ap.heat_recovery_gap)) {

				i_temp_EXT = PM_MAX_F;
			}

			pm.i_derated_1 = (i_temp_PCB < i_temp_EXT) ? i_temp_PCB : i_temp_EXT;

			/* Enable FAN if PCB is overheat.
			 * */
			if (ap.temp_PCB > ap.heat_PCB_FAN) {

				GPIO_set_LOW(GPIO_FAN);
			}
			else if (ap.temp_PCB < (ap.heat_PCB_FAN - ap.heat_recovery_gap)) {

				GPIO_set_HIGH(GPIO_FAN);
			}
		}
	}
	while (1);
}

void task_ERROR(void *pData)
{
	do {
		if (pm.fail_reason != PM_OK) {

			GPIO_set_HIGH(GPIO_LED);
			vTaskDelay((TickType_t) 200);

			GPIO_set_LOW(GPIO_LED);
			vTaskDelay((TickType_t) 800);
		}
		else if (log.textbuf[0] != 0) {

			GPIO_set_HIGH(GPIO_LED);
			vTaskDelay((TickType_t) 200);

			GPIO_set_LOW(GPIO_LED);
			vTaskDelay((TickType_t) 300);

			GPIO_set_HIGH(GPIO_LED);
			vTaskDelay((TickType_t) 200);

			GPIO_set_LOW(GPIO_LED);
			vTaskDelay((TickType_t) 800);
		}
		else {
			vTaskDelay((TickType_t) 200);
		}
	}
	while (1);
}

static void
inner_TIMEOUT()
{
	float			tIDLE;

	ap.timeout_TIME += (TickType_t) 10;

	do {
		if (pm.im_revol_total != ap.timeout_revol_cached) {

			ap.timeout_TIME = 0;
			ap.timeout_revol_cached = pm.im_revol_total;
			break;
		}

		if (m_fabsf(pm.lu_iQ) > ap.timeout_current_tol) {

			ap.timeout_TIME = 0;
			break;
		}

		tIDLE = ap.timeout_TIME * (1.f / (float) configTICK_RATE_HZ);

		if (tIDLE > ap.timeout_IDLE_s) {

			ap.timeout_TIME = 0;
			pm.fsm_req = PM_STATE_LU_SHUTDOWN;
		}
	}
	while (0);
}

static void
inner_ANALOG(float in_ANG, float in_BRK)
{
	float			control, range, scaled;

	if (		in_ANG < ap.analog_in_lost[0]
			|| in_ANG > ap.analog_in_lost[1]) {

		/* Loss of ANALOG signal.
		 * */

		scaled = - 1.f;
	}
	else {
		if (in_ANG < ap.analog_in_ANG[1]) {

			range = ap.analog_in_ANG[0] - ap.analog_in_ANG[1];
			scaled = (ap.analog_in_ANG[1] - in_ANG) / range;
		}
		else {
			range = ap.analog_in_ANG[2] - ap.analog_in_ANG[1];
			scaled = (in_ANG - ap.analog_in_ANG[1]) / range;
		}

		if (scaled < - 1.f) {

			scaled = - 1.f;
		}
		else if (scaled > 1.f) {

			scaled = 1.f;
		}
		else if (	ap.analog_STARTUP == PM_ENABLED
				&& pm.lu_mode == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;
		}
	}

	if (scaled < 0.f) {

		range = ap.analog_control_ANG[1] - ap.analog_control_ANG[0];
		control = ap.analog_control_ANG[1] + range * scaled;
	}
	else {
		range = ap.analog_control_ANG[2] - ap.analog_control_ANG[1];
		control = ap.analog_control_ANG[1] + range * scaled;
	}

	if (		in_BRK < ap.analog_in_lost[0]
			|| in_BRK > ap.analog_in_lost[1]) {

		/* Loss of BRAKE signal.
		 * */

		scaled = 0.f;
	}
	else {
		range = ap.analog_in_BRK[1] - ap.analog_in_BRK[0];
		scaled = (in_BRK - ap.analog_in_BRK[0]) / range;

		scaled = (scaled < 0.f) ? 0.f : (scaled > 1.f) ? 1.f : scaled;
	}

	control += (ap.analog_control_BRK - control) * scaled;

	reg_SET_F(ap.analog_reg_ID, control);
}

void task_ANALOG(void *pData)
{
	TickType_t		xWake;
	float			in_ANG, in_BRK;

	ap.analog_const_GU = hal.ADC_reference_voltage / hal.ADC_analog_ratio;

	GPIO_set_mode_ANALOG(GPIO_ADC_ANALOG_ANG);
	GPIO_set_mode_ANALOG(GPIO_ADC_ANALOG_BRK);

	xWake = xTaskGetTickCount();

	do {
		/* 100 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 10);

		if (ap.analog_ENABLED == PM_ENABLED) {

			in_ANG = ADC_get_analog_ANG();
			in_BRK = ADC_get_analog_BRK();

			if (ap.analog_reg_ID != ID_NULL) {

				inner_ANALOG(in_ANG, in_BRK);
			}

			if (		pm.lu_mode != PM_LU_DISABLED
					&& ap.timeout_IDLE_s > M_EPS_F) {

				inner_TIMEOUT();
			}
		}
		else {
			vTaskDelayUntil(&xWake, (TickType_t) 100);
		}
	}
	while (1);
}

/* Terminal voltage measurement schematic.
 *
                        +------< vREF
                        |
                        |
                       | |
                       | | R3 (1%)
                       |_|
              R1 (1%)   |               +---+
               _____    |              /    |
     vIN >----|_____|---+--------+----- ADC |
                        |        |     \    |
                        |        |      +---+
                       | |       |
               R2 (1%) | |     -----
                       |_|     -----
                        |        |    C1 (C0G)
                        |        |
                        +--------+
                        |
                       ---
                       \ /  AGND
 */

static void
app_flash_load()
{
	float		vm_R1 = 470000.f;
	float		vm_R2 = 27000.f;
	float		vm_R3 = 470000.f;
	float		vm_D = (vm_R1 * vm_R2 + vm_R2 * vm_R3 + vm_R1 * vm_R3);
	float		in_R1 = 10000.f;
	float		in_R2 = 10000.f;

	hal.USART_baud_rate = 57600;
	hal.PWM_frequency = 30000.f;
	hal.PWM_deadtime = 190.f;
	hal.ADC_reference_voltage = 3.3f;
	hal.ADC_shunt_resistance = 0.5E-3f;
	hal.ADC_amplifier_gain = 20.f;
	hal.ADC_voltage_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_ratio = vm_R2 * vm_R3 / vm_D;
	hal.ADC_terminal_bias = vm_R1 * vm_R2 * hal.ADC_reference_voltage / vm_D;
	hal.ADC_analog_ratio = in_R2 / (in_R1 + in_R2);

#ifdef _HW_REV2

	hal.PWM_deadtime = 90.f;
	hal.ADC_shunt_resistance = 0.5E-3f;
	hal.ADC_amplifier_gain = 60.f;
	hal.ADC_voltage_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_bias = 0.f;

#endif /* _HW_REV2 */

#ifdef _HW_KLEN

	vm_R1 = 47000.f;
	vm_R2 = 2200.f;

	hal.PWM_deadtime = 90.f;
	hal.ADC_shunt_resistance = 5E-3f;
	hal.ADC_amplifier_gain = - 5.f;
	hal.ADC_voltage_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_bias = 0.f;

#endif /* _HW_KLEN */

#ifdef _HW_REV4B

	hal.ADC_shunt_resistance = 0.5E-3f;
	hal.ADC_amplifier_gain = 60.f;
	hal.ADC_voltage_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_ratio = vm_R2 / (vm_R1 + vm_R2);
	hal.ADC_terminal_bias = 0.f;

#endif /* _HW_REV4B */

#ifdef _HW_REV4C

	/* Default */

#endif /* _HW_REV4C */

	hal.TIM_mode = TIM_DISABLED;
	hal.CAN_mode_NART = CAN_MODE_STANDARD;
	hal.PPM_mode = PPM_DISABLED;
	hal.PPM_timebase = 2000000UL;

	can.node_ID = 0;
	can.log_MODE = IFCAN_LOG_DISABLED;
	can.startup_LOST = 3000;

	can.pipe[0].ID = 10;
	can.pipe[0].tim = 30;
	can.pipe[1].ID = 20;
	can.pipe[1].tim = 30;
	can.pipe[2].ID = 30;
	can.pipe[2].tim = 30;
	can.pipe[3].ID = 40;
	can.pipe[3].tim = 30;
	can.pipe[4].ID = 50;
	can.pipe[4].tim = 30;
	can.pipe[5].ID = 60;
	can.pipe[5].tim = 30;
	can.pipe[6].ID = 70;
	can.pipe[6].tim = 30;
	can.pipe[7].ID = 80;
	can.pipe[7].tim = 30;

	ap.ppm_reg_ID = ID_PM_S_SETPOINT_PC;
	ap.ppm_STARTUP = PM_DISABLED;
	ap.ppm_in_range[0] = 1000.f;
	ap.ppm_in_range[1] = 1500.f;
	ap.ppm_in_range[2] = 2000.f;
	ap.ppm_control_range[0] = 0.f;
	ap.ppm_control_range[1] = 50.f;
	ap.ppm_control_range[2] = 100.f;

	ap.step_reg_ID = ID_PM_X_SETPOINT_F_MM;
	ap.step_STARTUP = PM_DISABLED;
	ap.step_const_ld_EP = 0.f;

	ap.analog_ENABLED = PM_DISABLED;
	ap.analog_reg_ID = ID_PM_I_SETPOINT_Q_PC;
	ap.analog_STARTUP = PM_DISABLED;
	ap.analog_in_ANG[0] = 1.0f;
	ap.analog_in_ANG[1] = 2.5f;
	ap.analog_in_ANG[2] = 4.0f;
	ap.analog_in_BRK[0] = 2.0f;
	ap.analog_in_BRK[1] = 4.0f;
	ap.analog_in_lost[0] = 0.2f;
	ap.analog_in_lost[1] = 4.8f;
	ap.analog_control_ANG[0] = 0.f;
	ap.analog_control_ANG[1] = 50.f;
	ap.analog_control_ANG[2] = 100.f;
	ap.analog_control_BRK = - 100.f;

	ap.timeout_current_tol = 2.f;
	ap.timeout_IDLE_s = 10.f;

	ap.ntc_PCB.r_balance = 10000.f;
	ap.ntc_PCB.r_ntc_0 = 10000.f;
	ap.ntc_PCB.ta_0 = 25.f;
	ap.ntc_PCB.betta = 3435.f;

	ap.ntc_EXT.r_balance = 10000.f;
	ap.ntc_EXT.r_ntc_0 = 10000.f;
	ap.ntc_EXT.ta_0 = 25.f;
	ap.ntc_EXT.betta = 3380.f;

	ap.heat_PCB = 90.f;
	ap.heat_PCB_derated_1 = 20.f;
	ap.heat_EXT = 90.f;
	ap.heat_EXT_derated_1 = 20.f;
	ap.heat_PCB_FAN = 60.f;
	ap.heat_recovery_gap = 5.f;

	ap.hx711_scale[0] = 0.f;
	ap.hx711_scale[1] = 4.545E-6f;

	ap.servo_SPAN_mm[0] = - 25.f;
	ap.servo_SPAN_mm[1] = 25.f;
	ap.servo_UNIFORM_mmps = 20.f;
	ap.servo_mice_role = 0;

	ap.FT_grab_hz = 200;

	pm.freq_hz = hal.PWM_frequency;
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = hal.PWM_resolution;
	pm.proc_set_DC = &PWM_set_DC;
	pm.proc_set_Z = &PWM_set_Z;

	pm_default(&pm);
	TLM_reg_default(&tlm);

	/* Try to load all above params from flash.
	 * */
	flash_block_regs_load();
}

void task_INIT(void *pData)
{
	int			irq;

	GPIO_set_mode_OUTPUT(GPIO_BOOST_12V);
	GPIO_set_HIGH(GPIO_BOOST_12V);

	GPIO_set_mode_OPEN_DRAIN(GPIO_FAN);
	GPIO_set_HIGH(GPIO_FAN);
	GPIO_set_mode_OUTPUT(GPIO_FAN);

	GPIO_set_mode_OUTPUT(GPIO_LED);
	GPIO_set_HIGH(GPIO_LED);

	io_USART.getc = &USART_getc;
	io_USART.putc = &USART_putc;
	io_CAN.getc = &IFCAN_getc;
	io_CAN.putc = &IFCAN_putc;

	/* Default to USART.
	 * */
	iodef = &io_USART;
	iodef_ECHO = 1;

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(LOAD_COUNT_DELAY);

	ap.lc_flag = 0;
	ap.lc_idle = ap.lc_tick;

	/* Load the configuration.
	 * */
	app_flash_load();

	irq = hal_lock_irq();

	ADC_startup();
	PWM_startup();
	WD_startup();

	pm.freq_hz = hal.PWM_frequency;
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = hal.PWM_resolution;

	hal_unlock_irq(irq);

	PPM_startup();
	TIM_startup();

	USART_startup();
	IFCAN_startup();

	xTaskCreate(task_TERM, "TERM", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_ERROR, "ERROR", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(task_ANALOG, "ANALOG", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_SH, "SH", 400, NULL, 1, NULL);

	if (log_bootup() != 0) {

		/* Slow down the startup to indicate a problem.
		 * */
		vTaskDelay((TickType_t) 1000);
	}

	GPIO_set_LOW(GPIO_LED);

	pm.fsm_req = PM_STATE_ZERO_DRIFT;

	/* Do apps startup here.
	 * */

	/*
	pushb_startup(EOL);
	*/

	vTaskDelete(NULL);
}

static void
inner_PULSE_WIDTH(float pulse)
{
	float		control, range, scaled;

	if (pulse < ap.ppm_in_range[1]) {

		range = ap.ppm_in_range[0] - ap.ppm_in_range[1];
		scaled = (ap.ppm_in_range[1] - pulse) / range;
	}
	else {
		range = ap.ppm_in_range[2] - ap.ppm_in_range[1];
		scaled = (pulse - ap.ppm_in_range[1]) / range;
	}

	scaled = (scaled < - 1.f) ? - 1.f : (scaled > 1.f) ? 1.f : scaled;

	if (scaled < 0.f) {

		range = ap.ppm_control_range[1] - ap.ppm_control_range[0];
		control = ap.ppm_control_range[1] + range * scaled;
	}
	else {
		range = ap.ppm_control_range[2] - ap.ppm_control_range[1];
		control = ap.ppm_control_range[1] + range * scaled;
	}

	reg_SET_F(ap.ppm_reg_ID, control);
}

static void
in_PULSE_WIDTH()
{
	float		pulse;

	if (hal.PPM_signal_caught != 0) {

		pulse = PPM_get_PULSE();

		if (pulse != ap.ppm_in_cached) {

			ap.ppm_in_cached = pulse;

			if (ap.ppm_reg_ID != ID_NULL) {

				inner_PULSE_WIDTH(pulse);

				if (		ap.ppm_STARTUP == PM_ENABLED
						&& pm.lu_mode == PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_STARTUP;
				}
			}
		}
	}
	else {
		if (		ap.ppm_STARTUP == PM_ENABLED
				&& pm.lu_mode != PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_SHUTDOWN;
		}
	}
}

static void
in_STEP_DIR()
{
	int		EP, relEP;
	float		xSP;

	EP = PPM_get_STEP_DIR();

	relEP = (short int) (EP - ap.step_baseEP);
	ap.step_baseEP = EP;

	if (relEP != 0) {

		ap.step_accuEP += relEP;

		if (ap.step_reg_ID != ID_NULL) {

			xSP = ap.step_accuEP * ap.step_const_ld_EP;

			reg_SET_F(ap.step_reg_ID, xSP);
		}

		if (		ap.step_STARTUP == PM_ENABLED
				&& pm.lu_mode == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;
		}
	}

	/*if (ap.step_reg_ID == ID_PM_X_SETPOINT_F_MM) {

		wSP = relEP * pm.freq_hz * ap.step_const_ld_EP;

		reg_SET_F(ID_PM_X_SETPOINT_WS_MMPS, wSP);
	}*/
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

	fb.pulse_HS = (hal.TIM_mode == TIM_DRIVE_HALL) ? GPIO_get_HALL() : 0;
	fb.pulse_EP = (hal.TIM_mode == TIM_DRIVE_ABI) ? TIM_get_EP() :
		(hal.PPM_mode == PPM_BACKUP_ABI) ? PPM_get_backup_EP() : 0;

	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		in_PULSE_WIDTH();
	}
	else if (hal.PPM_mode == PPM_STEP_DIR) {

		in_STEP_DIR();
	}

	pm_feedback(&pm, &fb);

	IFCAN_pipes_REGULAR();
	TLM_reg_grab(&tlm);

	WD_kick();
}

void app_MAIN()
{
	xTaskCreate(task_INIT, "INIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

SH_DEF(rtos_firmware)
{
	u32_t		*flash = (u32_t *) &ld_begin_vectors;
	u32_t		FW_sizeof, FW_crc32;

	FW_sizeof = * (flash + 8) - * (flash + 7);
	FW_crc32 = crc32b(flash, FW_sizeof);

	printf("FW_sizeof %i" EOL, FW_sizeof);
	printf("FW_crc32 %8x" EOL, FW_crc32);
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

	printf("[%i] %id %ih %im %is" EOL, log.boot_COUNT, Day, Hour, Min, Sec);
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

SH_DEF(rtos_task_list)
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
					(u32_t) pLIST[N].xHandle,
					(int) pLIST[N].xTaskNumber,
					(const char *) pLIST[N].pcTaskName,
					(int) xState,
					(int) pLIST[N].uxCurrentPriority,
					(u32_t) pLIST[N].pxStackBase,
					(int) pLIST[N].usStackHighWaterMark);
		}

		vPortFree(pLIST);
	}
}

SH_DEF(rtos_task_kill)
{
	TaskHandle_t		xHandle;

	xHandle = xTaskGetHandle(s);

	if (xHandle != NULL) {

		vTaskDelete(xHandle);
	}
}

SH_DEF(rtos_freeheap)
{
	printf("FreeHeap %i" EOL, xPortGetFreeHeapSize());
	printf("Minimum %i" EOL, xPortGetMinimumEverFreeHeapSize());
}

SH_DEF(rtos_log_flush)
{
	if (log.textbuf[0] != 0) {

		puts(log.textbuf);
		puts(EOL);
	}
}

SH_DEF(rtos_log_cleanup)
{
	if (log.textbuf[0] != 0) {

		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.tail = 0;
	}
}

SH_DEF(rtos_reboot)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	GPIO_set_LOW(GPIO_BOOST_12V);
	vTaskDelay((TickType_t) 50);

	hal_system_reset();
}

SH_DEF(rtos_bootload)
{
	if (pm.lu_mode != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	GPIO_set_LOW(GPIO_BOOST_12V);
	vTaskDelay((TickType_t) 50);

	hal_bootload_jump();
}

