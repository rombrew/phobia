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
pmc_t 				pm	LD_CCRAM;
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

void task_TEMP(void *pData)
{
	TickType_t		xWake;
	float			i_PCB, i_EXT;

#if defined(HW_HAVE_NTC_ON_PCB)
	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_NTC);
#elif defined(HW_HAVE_ATS_ON_PCB)
	GPIO_set_mode_ANALOG(GPIO_ADC_PCB_ATS);
#endif /* HW_HAVE_XX_ON_PCB */

	GPIO_set_mode_ANALOG(GPIO_ADC_EXT_NTC);

	xWake = xTaskGetTickCount();

	i_PCB = PM_MAX_F;
	i_EXT = PM_MAX_F;

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		ap.temp_INT = ADC_get_VALUE(GPIO_ADC_INTERNAL_TEMP);

#if defined(HW_HAVE_NTC_ON_PCB)
		ap.temp_PCB = ntc_temperature(&ap.ntc_PCB, ADC_get_VALUE(GPIO_ADC_PCB_NTC));
#elif defined(HW_HAVE_ATS_ON_PCB)
		ap.temp_PCB = ats_temperature(&ap.ntc_PCB, ADC_get_VALUE(GPIO_ADC_PCB_ATS));
#else /* HW_HAVE_XX_ON_PCB */
		ap.temp_PCB = ap.temp_INT;
#endif

		ap.temp_EXT = ntc_temperature(&ap.ntc_EXT, ADC_get_VALUE(GPIO_ADC_EXT_NTC));

		if (pm.lu_mode != PM_LU_DISABLED) {

			/* Derate current if PCB is overheat.
			 * */
			if (ap.temp_PCB > ap.heat_PCB_halt) {

				i_PCB = ap.heat_PCB_derated;
			}
			else if (ap.temp_PCB < (ap.heat_PCB_halt - ap.heat_recovery_gap)) {

				i_PCB = PM_MAX_F;
			}

			/* Enable FAN if PCB is warm.
			 * */
			if (ap.temp_PCB > ap.heat_PCB_on_FAN) {

				GPIO_set_LOW(GPIO_FAN);
			}
			else if (ap.temp_PCB < (ap.heat_PCB_on_FAN - ap.heat_recovery_gap)) {

				GPIO_set_HIGH(GPIO_FAN);
			}

			if (ap.heat_EXT_halt > M_EPS_F) {

				/* Derate current if EXT is overheat.
				 * */
				if (ap.temp_EXT > ap.heat_EXT_halt) {

					i_EXT = ap.heat_EXT_derated;
				}
				else if (ap.temp_EXT < (ap.heat_EXT_halt - ap.heat_recovery_gap)) {

					i_EXT = PM_MAX_F;
				}
			}

			pm.i_derated_PCB = (i_PCB < i_EXT) ? i_PCB : i_EXT;
		}
	}
	while (1);
}

void task_ERROR(void *pData)
{
	do {
		if (pm.fsm_errno != PM_OK) {

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
		if (pm.im_total_revol != ap.timeout_revol_cached) {

			ap.timeout_TIME = 0;
			ap.timeout_revol_cached = pm.im_total_revol;
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
	float			control, range, scaled_ANG, scaled_BRK;

	if (		in_ANG < ap.analog_in_lost[0]
			|| in_ANG > ap.analog_in_lost[1]) {

		/* Loss of ANALOG signal.
		 * */

		scaled_ANG = - 1.f;
	}
	else {
		if (in_ANG < ap.analog_in_ANG[1]) {

			range = ap.analog_in_ANG[0] - ap.analog_in_ANG[1];
			scaled_ANG = (ap.analog_in_ANG[1] - in_ANG) / range;
		}
		else {
			range = ap.analog_in_ANG[2] - ap.analog_in_ANG[1];
			scaled_ANG = (in_ANG - ap.analog_in_ANG[1]) / range;
		}

		if (scaled_ANG < - 1.f) {

			scaled_ANG = - 1.f;
		}
		else if (scaled_ANG > 1.f) {

			scaled_ANG = 1.f;
		}
		else if (	ap.analog_STARTUP == PM_ENABLED
				&& pm.lu_mode == PM_LU_DISABLED) {

			pm.fsm_req = PM_STATE_LU_STARTUP;
		}
	}

	if (scaled_ANG < 0.f) {

		range = ap.analog_control_ANG[1] - ap.analog_control_ANG[0];
		control = ap.analog_control_ANG[1] + range * scaled_ANG;
	}
	else {
		range = ap.analog_control_ANG[2] - ap.analog_control_ANG[1];
		control = ap.analog_control_ANG[1] + range * scaled_ANG;
	}

	if (		in_BRK < ap.analog_in_lost[0]
			|| in_BRK > ap.analog_in_lost[1]) {

		/* Loss of BRAKE signal.
		 * */

		scaled_BRK = 0.f;
	}
	else {
		range = ap.analog_in_BRK[1] - ap.analog_in_BRK[0];
		scaled_BRK = (in_BRK - ap.analog_in_BRK[0]) / range;

		scaled_BRK = (scaled_BRK < 0.f) ? 0.f
			: (scaled_BRK > 1.f) ? 1.f : scaled_BRK;
	}

	control += (ap.analog_control_BRK - control) * scaled_BRK;

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

static void
app_flash_load()
{
	float			vm_D, halt_I, halt_U;

	hal.USART_baud_rate = 57600;
	hal.PWM_frequency = 30000.f;

	hal.PWM_deadtime = HW_PWM_DEADTIME_NS;
	hal.ADC_reference_voltage = HW_ADC_REFERENCE_VOLTAGE;
	hal.ADC_shunt_resistance = HW_ADC_SHUNT_RESISTANCE;
	hal.ADC_amplifier_gain = HW_ADC_AMPLIFIER_GAIN;
	hal.ADC_voltage_ratio = HW_ADC_VOLTAGE_R2 / (HW_ADC_VOLTAGE_R1 + HW_ADC_VOLTAGE_R2);

	vm_D = (HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2 + HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_BIAS_R3
			+ HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_BIAS_R3);

	hal.ADC_terminal_ratio = HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_BIAS_R3 / vm_D;
	hal.ADC_terminal_bias = HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2 * hal.ADC_reference_voltage / vm_D;

#ifdef HW_HAVE_ANALOG_KNOB
	hal.ADC_analog_ratio = HW_ADC_ANALOG_R2 / (HW_ADC_ANALOG_R1 + HW_ADC_ANALOG_R2);
#else /* HW_HAVE_ANALOG_KNOB */
	hal.ADC_analog_ratio = 1.f;
#endif

	hal.TIM_mode = TIM_DISABLED;
	hal.CAN_mode_NART = CAN_MODE_STANDARD;
	hal.PPM_mode = PPM_DISABLED;
	hal.PPM_timebase = 2000000UL;

	net.node_ID = 0;
	net.log_MODE = IFCAN_LOG_DISABLED;
	net.flash_MODE = PM_ENABLED;
	net.startup_LOST = 3000;

	net.pipe[0].ID = 10;
	net.pipe[0].tim = 30;
	net.pipe[1].ID = 20;
	net.pipe[1].tim = 30;
	net.pipe[2].ID = 30;
	net.pipe[2].tim = 30;
	net.pipe[3].ID = 40;
	net.pipe[3].tim = 30;
	net.pipe[4].ID = 50;
	net.pipe[4].tim = 30;
	net.pipe[5].ID = 60;
	net.pipe[5].tim = 30;
	net.pipe[6].ID = 70;
	net.pipe[6].tim = 30;
	net.pipe[7].ID = 80;
	net.pipe[7].tim = 30;

	ap.ppm_reg_ID = ID_PM_S_SETPOINT_SPEED_PC;
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
	ap.analog_reg_ID = ID_PM_I_SETPOINT_TORQUE_PC;
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
	ap.timeout_IDLE_s = 5.f;

#if defined(HW_HAVE_NTC_ON_PCB)
	ap.ntc_PCB.r_balance = HW_NTC_PCB_R_BALANCE;
	ap.ntc_PCB.r_ntc_0 = HW_NTC_PCB_R_NTC_0;
	ap.ntc_PCB.ta_0 = HW_NTC_PCB_TA_0;
	ap.ntc_PCB.betta = HW_NTC_PCB_BETTA;
#elif defined(HW_HAVE_ATS_ON_PCB)
	ap.ntc_PCB.ta_0 = HW_ATS_PCB_TA_0;
	ap.ntc_PCB.betta = HW_ATS_PCB_BETTA;
#endif /* HW_HAVE_XX_ON_PCB */

	ap.ntc_EXT.r_balance = 10000.f;
	ap.ntc_EXT.r_ntc_0 = 10000.f;
	ap.ntc_EXT.ta_0 = 25.f;
	ap.ntc_EXT.betta = 3380.f;

	ap.heat_PCB_halt = 90.f;
	ap.heat_PCB_on_FAN = 50.f;
	ap.heat_PCB_derated = 10.f;
	ap.heat_EXT_halt = 0.f;
	ap.heat_EXT_derated = 10.f;
	ap.heat_recovery_gap = 5.f;

	ap.hx711_scale[0] = 0.f;
	ap.hx711_scale[1] = 4.6493E-6f;

	ap.servo_SPAN_mm[0] = - 25.f;
	ap.servo_SPAN_mm[1] = 25.f;
	ap.servo_UNIFORM_mmps = 20.f;

	pm.freq_hz = hal.PWM_frequency;
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = hal.PWM_resolution;
	pm.proc_set_DC = &PWM_set_DC;
	pm.proc_set_Z = &PWM_set_Z;

	/* Default PMC configuration.
	 * */
	pm_default(&pm);

	pm.dc_bootstrap = HW_PWM_BOOTSTRAP_TIME;

#ifndef HW_HAVE_TVM_CIRCUIT
	pm.config_TVM = PM_DISABLED;
#endif /* HW_HAVE_TVM_CIRCUIT */

	halt_I = hal.ADC_reference_voltage / hal.ADC_shunt_resistance / hal.ADC_amplifier_gain / 2.f;
	halt_U = hal.ADC_reference_voltage / hal.ADC_voltage_ratio;

	pm.fault_current_halt = (float) (int) (.95f * m_fabsf(halt_I));
	pm.fault_voltage_halt = (float) (int) (.95f * halt_U);

	/* Default telemetry.
	 * */
	TLM_reg_default(&tlm);

	/* Try to load all above params from flash.
	 * */
	flash_block_regs_load();
}

void task_INIT(void *pData)
{
	u32_t			seed[3];
	int			irq;

	GPIO_set_mode_OUTPUT(GPIO_BOOST_12V);
	GPIO_set_HIGH(GPIO_BOOST_12V);

	GPIO_set_mode_OPEN_DRAIN(GPIO_FAN);
	GPIO_set_HIGH(GPIO_FAN);
	GPIO_set_mode_OUTPUT(GPIO_FAN);

	GPIO_set_mode_OUTPUT(GPIO_LED);
	GPIO_set_HIGH(GPIO_LED);

	RNG_startup();

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

	seed[0] = RNG_urand();

	/* We drop the first value and check if RNG
	 * gives the same number each time.
	 * */
	seed[1] = RNG_urand();
	seed[2] = RNG_urand();

	if (seed[1] == seed[2]) {

		log_TRACE("RNG failed" EOL);
	}

	/* Initial SEED.
	 * */
	rseed = seed[2];

	if (log_bootup() != 0) {

		/* Slow the startup to show down a problem.
		 * */
		vTaskDelay((TickType_t) 1000);
	}

	/* Load the configuration.
	 * */
	app_flash_load();

	irq = hal_lock_irq();

	/* Do CORE startup.
	 * */
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

#ifdef HW_HAVE_TRANSCEIVER_CAN
	IFCAN_startup();
#endif /* HW_HAVE_TRANSCEIVER_CAN */

	xTaskCreate(task_TEMP, "TEMP", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_ERROR, "ERROR", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

#ifdef HW_HAVE_ANALOG_KNOB
	xTaskCreate(task_ANALOG, "ANALOG", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
#endif /* HW_HAVE_ANALOG_KNOB */

	xTaskCreate(task_SH, "SH", 400, NULL, 1, NULL);

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

	printf("HW_revision \"%s\"" EOL, HW_REVISION_NAME);

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

		printf("TCB      ID Name              Stat Prio Stack    Free" EOL);

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

			printf("%8x %2i %17s %c    %2i   %8x %i" EOL,
					(u32_t) pLIST[N].xHandle,
					(int) pLIST[N].xTaskNumber,
					pLIST[N].pcTaskName,
					(int) xState,
					(int) pLIST[N].uxCurrentPriority,
					(u32_t) pLIST[N].pxStackBase,
					(int) pLIST[N].usStackHighWaterMark);
		}

		vPortFree(pLIST);
	}
}

SH_DEF(rtos_hexdump)
{
	u8_t			*m_dump;
	int			l, j, b_ascii, n_lines = 4;

	if (htoi((int *) &m_dump, s) != NULL) {

		stoi(&n_lines, sh_next_arg(s));

		for (l = 0; l < n_lines; ++l) {

			printf("%8x  ", (u32_t) m_dump);

			for (j = 0; j < 8; ++j) {

				printf("%2x ", m_dump[j]);
			}

			puts(" ");

			for (j = 0; j < 8; ++j) {

				printf("%2x ", m_dump[j + 8]);
			}

			puts(" |");

			for (j = 0; j < 16; ++j) {

				b_ascii = m_dump[j];
				b_ascii = (b_ascii >= 0x20 && b_ascii <= 0x7E)
					? b_ascii : '.';

				putc(b_ascii);
			}

			puts("|" EOL);

			m_dump += 16;
		}
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

		log.len = 0;
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

