#include <stddef.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "main.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "epcan.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "libc.h"
#include "shell.h"

#define RTOS_LOAD_DELAY		((TickType_t) 100)
#define APP_LIST_MAX		(sizeof(ap_list) / sizeof(ap_list[0]) - 1U)

#undef APP_DEF
#define APP_DEF(name)	void app_ ## name(void *);
#include "apps/apdefs.h"

const app_task_t		ap_list[] = {

	{"APP_NONE", NULL},

#undef APP_DEF
#define APP_DEF(name)	{ "APP_" #name, &app_ ## name},
#include "apps/apdefs.h"

	{NULL, NULL}
};

app_main_t			ap;
app_run_t			ap_run[APP_LIST_MAX];

pmc_t 				pm LD_CCRAM;
tlm_t				tlm;

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
	log_TRACE("FreeRTOS: Stack Overflow in %8x task" EOL, (uint32_t) xTask);

	hal_system_reset();
}

#ifdef HW_HAVE_ANALOG_KNOB
float ADC_get_knob_ANG()
{
	float			knob;

	knob = ADC_get_VALUE(GPIO_ADC_KNOB_ANG);

	return knob * hal.const_ADC.GK;
}

float ADC_get_knob_BRK()
{
	float			knob;

#ifndef HW_HAVE_NO_BRAKE_KNOB
	knob = ADC_get_VALUE(GPIO_ADC_KNOB_BRK);
#else /* HW_HAVE_NO_BRAKE_KNOB */
	knob = 0.f;
#endif

	return knob * hal.const_ADC.GK;
}
#endif /* HW_HAVE_ANALOG_KNOB */

static int
elapsed_IDLE()
{
	TickType_t		xIDLE, xNOW;
	int			elapsed = 0;

	xIDLE = (TickType_t) (ap.idle_TIME_s * (float) configTICK_RATE_HZ);

	if (xIDLE > 0) {

		xNOW = xTaskGetTickCount();

		/* RESET if the function is not called for too long.
		 * */
		if (xNOW - (TickType_t) ap.idle_INVOKE > (TickType_t) 100) {

			ap.idle_RESET = xNOW;
		}

		ap.idle_INVOKE = xNOW;

		/* RESET if motor is still spinning.
		 * */
		if (pm.lu_total_revol != ap.idle_revol_cached) {

			ap.idle_RESET = xNOW;
			ap.idle_revol_cached = pm.lu_total_revol;
		}

		if (xNOW - (TickType_t) ap.idle_RESET > xIDLE) {

			/* We are IDLE for specified time.
			 * */
			elapsed = 1;
		}
	}

	return elapsed;
}

static int
elapsed_SAFETY()
{
	TickType_t		xNOW;
	int			elapsed = 0;

	xNOW = xTaskGetTickCount();

	if (xNOW - (TickType_t) ap.safety_INVOKE > (TickType_t) 100) {

		ap.safety_RESET = xNOW;
	}

	ap.safety_INVOKE = xNOW;

	if (xNOW - (TickType_t) ap.safety_RESET > (TickType_t) 1000) {

		elapsed = 1;
	}

	return elapsed;
}

void task_TEMP(void *pData)
{
	TickType_t		xWake;
	float			x_PCB, x_EXT, t_HYS;

	if (ap.ntc_PCB.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_PCB.gpio);
	}

	if (ap.ntc_EXT.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_EXT.gpio);
	}

	xWake = xTaskGetTickCount();

	x_PCB = PM_MAX_F;
	x_EXT = PM_MAX_F;
	t_HYS = 0.f;

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		ap.temp_INT = ADC_get_VALUE(GPIO_ADC_TEMPINT);

		if (ap.ntc_PCB.type != NTC_NONE) {

			ap.temp_PCB = ntc_read_temperature(&ap.ntc_PCB);
		}
		else {
			ap.temp_PCB = ap.temp_INT;
		}

		if (ap.ntc_EXT.type != NTC_NONE) {

			ap.temp_EXT = ntc_read_temperature(&ap.ntc_EXT);
		}
		else {
			ap.temp_EXT = 0.f;
		}

		if (ap.temp_PCB > ap.tpro_PCB_temp_halt - t_HYS) {

			x_PCB = 0.f;

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_errno = PM_ERROR_HW_OVERTEMPERATURE;
				pm.fsm_req = PM_STATE_HALT;
			}

			t_HYS = ap.tpro_temp_recovery;
		}
		else {
			if (ap.temp_PCB > ap.tpro_PCB_temp_derate) {

				/* Derate current in case of PCB thermal overload.
				 * */
				x_PCB = ap.tpro_derated_PCB;
			}
			else if (ap.temp_PCB < ap.tpro_PCB_temp_derate - ap.tpro_temp_recovery) {

				x_PCB = PM_MAX_F;
			}

			t_HYS = 0.f;
		}

#ifdef GPIO_FAN_EN
		/* Enable FAN in case of PCB is warm enough.
		 * */
		if (ap.temp_PCB > ap.tpro_PCB_temp_FAN) {

			GPIO_set_LOW(GPIO_FAN_EN);
		}
		else if (ap.temp_PCB < ap.tpro_PCB_temp_FAN - ap.tpro_temp_recovery) {

			GPIO_set_HIGH(GPIO_FAN_EN);
		}
#endif /* GPIO_FAN_EN */

		if (ap.tpro_EXT_temp_derate > M_EPS_F) {

			if (ap.temp_EXT > ap.tpro_EXT_temp_derate) {

				/* Derate current in case of external thermal
				 * overload (motor overheat).
				 * */
				x_EXT = ap.tpro_derated_EXT;
			}
			else if (ap.temp_EXT < ap.tpro_EXT_temp_derate - ap.tpro_temp_recovery) {

				x_EXT = PM_MAX_F;
			}
		}

		pm.i_derated_PCB = (x_PCB < x_EXT) ? x_PCB : x_EXT;

#ifdef HW_HAVE_PART_DRV_XX
		if (DRV_fault() != 0) {

			/* Read DRV status registers.
			 * */
			DRV_status();
		}
#endif /* HW_HAVE_PART_DRV_XX */

#ifdef GPIO_LED_MODE
		if (pm.lu_MODE != PM_LU_DISABLED) {

			GPIO_set_HIGH(GPIO_LED_MODE);
		}
		else {
			GPIO_set_LOW(GPIO_LED_MODE);
		}
#endif /* GPIO_LED_MODE */
	}
	while (1);
}

void task_ALERT(void *pData)
{
	do {
		if (pm.fsm_errno != PM_OK) {

			GPIO_set_HIGH(GPIO_LED_ALERT);
			vTaskDelay((TickType_t) 400);

			GPIO_set_LOW(GPIO_LED_ALERT);
			vTaskDelay((TickType_t) 400);
		}
		else if (log_status() != 0) {

			GPIO_set_HIGH(GPIO_LED_ALERT);
			vTaskDelay((TickType_t) 100);

			GPIO_set_LOW(GPIO_LED_ALERT);
			vTaskDelay((TickType_t) 700);
		}
		else {
			vTaskDelay((TickType_t) 100);
		}
	}
	while (1);
}

#ifdef HW_HAVE_ANALOG_KNOB
static void
inner_KNOB(float in_ANG, float in_BRK)
{
	float			control, range, scaled_ANG, scaled_BRK;

	if (		   in_ANG < ap.knob_in_lost[0]
			|| in_ANG > ap.knob_in_lost[1]) {

		/* Loss of KNOB signal.
		 * */
		scaled_ANG = - 1.f;

		if (ap.knob_ACTIVE == PM_ENABLED) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}

			ap.knob_ACTIVE = PM_DISABLED;
		}
	}
	else {
		if (in_ANG < ap.knob_in_ANG[1]) {

			range = ap.knob_in_ANG[0] - ap.knob_in_ANG[1];
			scaled_ANG = (ap.knob_in_ANG[1] - in_ANG) / range;
		}
		else {
			range = ap.knob_in_ANG[2] - ap.knob_in_ANG[1];
			scaled_ANG = (in_ANG - ap.knob_in_ANG[1]) / range;
		}

		if (scaled_ANG < - 1.f) {

			scaled_ANG = - 1.f;

			if (ap.knob_ACTIVE == PM_ENABLED) {

				if (elapsed_IDLE() != 0) {

					if (pm.lu_MODE != PM_LU_DISABLED) {

						pm.fsm_req = PM_STATE_LU_SHUTDOWN;
					}

					ap.knob_ACTIVE = PM_DISABLED;
				}
			}
		}
		else {
			if (scaled_ANG > 1.f) {

				scaled_ANG = 1.f;
			}

			if (		ap.knob_STARTUP == PM_ENABLED
					&& ap.knob_ACTIVE != PM_ENABLED) {

				if (pm.lu_MODE == PM_LU_DISABLED) {

					ap.knob_ACTIVE = PM_ENABLED;
					pm.fsm_req = PM_STATE_LU_STARTUP;
				}
			}
		}
	}

	if (scaled_ANG < 0.f) {

		range = ap.knob_control_ANG[1] - ap.knob_control_ANG[0];
		control = ap.knob_control_ANG[1] + range * scaled_ANG;
	}
	else {
		range = ap.knob_control_ANG[2] - ap.knob_control_ANG[1];
		control = ap.knob_control_ANG[1] + range * scaled_ANG;
	}

	if (		   in_BRK < ap.knob_in_lost[0]
			|| in_BRK > ap.knob_in_lost[1]) {

		/* Loss of BRAKE signal.
		 * */
		scaled_BRK = 0.f;
	}
	else {
		range = ap.knob_in_BRK[1] - ap.knob_in_BRK[0];
		scaled_BRK = (in_BRK - ap.knob_in_BRK[0]) / range;

		scaled_BRK = (scaled_BRK < 0.f) ? 0.f
			: (scaled_BRK > 1.f) ? 1.f : scaled_BRK;

		control += (ap.knob_control_BRK - control) * scaled_BRK;
	}

	if (		ap.knob_STARTUP != PM_ENABLED
			|| ap.knob_ACTIVE == PM_ENABLED) {

		reg_SET_F(ap.knob_reg_ID, control);
	}
}

void task_KNOB(void *pData)
{
	TickType_t		xWake;
	float			in_ANG, in_BRK;

	GPIO_set_mode_ANALOG(GPIO_ADC_KNOB_ANG);

#ifndef HW_HAVE_NO_BRAKE_KNOB
	GPIO_set_mode_ANALOG(GPIO_ADC_KNOB_BRK);
#endif /* HW_HAVE_NO_BRAKE_KNOB */

	xWake = xTaskGetTickCount();

	do {
		/* 200 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 5);

		if (ap.knob_ENABLED == PM_ENABLED) {

			in_ANG = ADC_get_knob_ANG();
			in_BRK = ADC_get_knob_BRK();

			if (ap.knob_reg_ID != ID_NULL) {

				inner_KNOB(in_ANG, in_BRK);
			}
		}
		else {
			vTaskDelayUntil(&xWake, (TickType_t) 100);
		}
	}
	while (1);
}
#endif /* HW_HAVE_ANALOG_KNOB */

static void
default_flash_load()
{
	float			vm_D, halt_I, halt_U;

	hal.USART_baud_rate = 57600;

	hal.PWM_frequency = HW_PWM_FREQUENCY_HZ;
	hal.PWM_deadtime = HW_PWM_DEADTIME_NS;
	hal.ADC_reference_voltage = HW_ADC_REFERENCE_VOLTAGE;
	hal.ADC_shunt_resistance = HW_ADC_SHUNT_RESISTANCE;
	hal.ADC_amplifier_gain = HW_ADC_AMPLIFIER_GAIN;
	hal.ADC_voltage_ratio = HW_ADC_VOLTAGE_R2 / (HW_ADC_VOLTAGE_R1 + HW_ADC_VOLTAGE_R2);

	vm_D =    HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2
		+ HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_BIAS_R3
		+ HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_BIAS_R3;

	hal.ADC_terminal_ratio = HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_BIAS_R3 / vm_D;
	hal.ADC_terminal_bias = HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2
				* hal.ADC_reference_voltage / vm_D;

#ifdef HW_HAVE_ANALOG_KNOB
	hal.ADC_knob_ratio = HW_ADC_KNOB_R2 / (HW_ADC_KNOB_R1 + HW_ADC_KNOB_R2);
#endif /* HW_HAVE_ANALOG_KNOB */

	hal.DPS_mode = DPS_DISABLED;
	hal.PPM_mode = PPM_DISABLED;
	hal.PPM_timebase = 2000000UL;

#ifdef HW_HAVE_PART_DRV8303
	hal.DRV.part = DRV_PART_DRV8303;
	hal.DRV.auto_RESET = 0;
	hal.DRV.gpio_GATE_EN = GPIO_DRV8303_GATE_EN;
	hal.DRV.gpio_FAULT = GPIO_DRV8303_FAULT;
	hal.DRV.gate_current = HW_DRV8303_GATE_CURRENT;
	hal.DRV.ocp_level = HW_DRV8303_OCP_LEVEL;
#endif /* HW_HAVE_PART_DRV8303 */

	hal.OPT = 0;

#ifdef HW_HAVE_NETWORK_EPCAN
	net.node_ID = 0;
	net.log_MODE = EPCAN_LOG_DISABLED;
	net.startup_LOST = 100 * hal.PWM_frequency / 1000;
	net.ep[0].ID = 10;
	net.ep[0].TIM = hal.PWM_frequency / 1000;
	net.ep[1].ID = 20;
	net.ep[1].TIM = net.ep[0].TIM;
	net.ep[2].ID = 30;
	net.ep[2].TIM = net.ep[0].TIM;
	net.ep[3].ID = 40;
	net.ep[3].TIM = net.ep[0].TIM;
	net.ep[4].ID = 50;
	net.ep[4].TIM = net.ep[0].TIM;
	net.ep[5].ID = 60;
	net.ep[5].TIM = net.ep[0].TIM;
	net.ep[6].ID = 70;
	net.ep[6].TIM = net.ep[0].TIM;
	net.ep[7].ID = 80;
	net.ep[7].TIM = net.ep[0].TIM;
#endif /* HW_HAVE_NETWORK_EPCAN */

	ap.ppm_reg_ID = ID_PM_S_SETPOINT_SPEED_PC;
	ap.ppm_STARTUP = PM_DISABLED;
	ap.ppm_LOCKED = PM_ENABLED;
	ap.ppm_in_range[0] = 1000.f;
	ap.ppm_in_range[1] = 1500.f;
	ap.ppm_in_range[2] = 2000.f;
	ap.ppm_control_range[0] = 0.f;
	ap.ppm_control_range[1] = 50.f;
	ap.ppm_control_range[2] = 100.f;

	ap.step_reg_ID = ID_PM_X_SETPOINT_LOCATION_MM;
	ap.step_STARTUP = PM_DISABLED;
	ap.step_const_ld_EP = 0.f;

	ap.knob_ENABLED = PM_DISABLED;
	ap.knob_reg_ID = ID_PM_I_SETPOINT_CURRENT_PC;
	ap.knob_STARTUP = PM_DISABLED;
	ap.knob_in_ANG[0] = 1.0f;
	ap.knob_in_ANG[1] = 2.5f;
	ap.knob_in_ANG[2] = 4.0f;
	ap.knob_in_BRK[0] = 2.0f;
	ap.knob_in_BRK[1] = 4.0f;
	ap.knob_in_lost[0] = 0.2f;
	ap.knob_in_lost[1] = 4.8f;
	ap.knob_control_ANG[0] = 0.f;
	ap.knob_control_ANG[1] = 50.f;
	ap.knob_control_ANG[2] = 100.f;
	ap.knob_control_BRK = - 100.f;

	ap.idle_TIME_s = 2.f;

#ifdef HW_HAVE_NTC_ON_PCB
	ap.ntc_PCB.type = HW_NTC_PCB_TYPE;
	ap.ntc_PCB.gpio = GPIO_ADC_NTC_PCB;
	ap.ntc_PCB.balance = HW_NTC_PCB_BALANCE;
	ap.ntc_PCB.ntc0 = HW_NTC_PCB_NTC0;
	ap.ntc_PCB.ta0 = HW_NTC_PCB_TA0;
	ap.ntc_PCB.betta = HW_NTC_PCB_BETTA;
#endif /* HW_HAVE_NTC_ON_PCB */

#ifdef HW_HAVE_NTC_MOTOR
	ap.ntc_EXT.type = NTC_GND;
	ap.ntc_EXT.gpio = GPIO_ADC_NTC_EXT;
	ap.ntc_EXT.balance = 10000.f;
	ap.ntc_EXT.ntc0 = 10000.f;
	ap.ntc_EXT.ta0 = 25.f;
	ap.ntc_EXT.betta = 3380.f;
#endif /* HW_HAVE_NTC_MOTOR */

	ap.tpro_PCB_temp_halt = 110.f;
	ap.tpro_PCB_temp_derate = 90.f;
	ap.tpro_PCB_temp_FAN = 50.f;
	ap.tpro_EXT_temp_derate = 0.f;
	ap.tpro_derated_PCB = 20.f;
	ap.tpro_derated_EXT = 20.f;
	ap.tpro_temp_recovery = 5.f;

	ap.auto_APP[0] = 0;
	ap.auto_APP[1] = 0;

	ap.adc_load_scale[0] = 0.f;
	ap.adc_load_scale[1] = 4.65E-6f;

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
	pm_auto(&pm, PM_AUTO_BASIC_DEFAULT);
	pm_auto(&pm, PM_AUTO_CONFIG_DEFAULT);

	pm.dc_minimal = HW_PWM_MINIMAL_PULSE;
	pm.dc_clearance = HW_PWM_CLEARANCE_ZONE;
	pm.dc_skip = HW_PWM_SKIP_ZONE;
	pm.dc_bootstrap = HW_PWM_BOOTSTRAP_RETENTION;

#if ADC_HAVE_SEQUENCE__ABC(HW_ADC_SAMPLING_SCHEME)
	pm.config_IFB = PM_IFB_ABC_INLINE;
#else /* ADC_HAVE_SEQUENCE__ABC(HW_ADC_SAMPLING_SCHEME) */
	pm.config_IFB = PM_IFB_AB_INLINE;
#endif

#if ADC_HAVE_SEQUENCE__TTT(HW_ADC_SAMPLING_SCHEME)
	pm.config_TVM = PM_ENABLED;
#else /* ADC_HAVE_SEQUENCE__TTT(HW_ADC_SAMPLING_SCHEME) */
	pm.config_TVM = PM_DISABLED;
#endif

#ifdef HW_HAVE_SHUNT_ON_GND
	pm.config_IFB =   (pm.config_IFB == PM_IFB_AB_INLINE) ? PM_IFB_AB_GND
			: (pm.config_IFB == PM_IFB_ABC_INLINE) ? PM_IFB_ABC_GND
			:  pm.config_IFB;
#endif /* HW_HAVE_SHUNT_ON_GND */

	ADC_const_build();

	halt_I = m_fabsf(hal.const_ADC.GA * ADC_RESOLUTION / 2.f);
	halt_U = m_fabsf(hal.const_ADC.GU * ADC_RESOLUTION);

	pm.fault_current_halt = (float) (int) (.95f * halt_I);
	pm.fault_voltage_halt = (float) (int) (.95f * halt_U);

	pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);

	pm.watt_iDC_maximal = (float) (int) (0.66666667f * pm.i_maximal);
	pm.watt_iDC_reverse = pm.watt_iDC_maximal;

	pm.watt_wP_maximal = (float) (int) (pm.watt_iDC_maximal * pm.fault_voltage_halt);
	pm.watt_wP_reverse = pm.watt_wP_maximal;

	pm.watt_dclink_HI = (float) (int) (pm.fault_voltage_halt - 5.f);

	/* Default telemetry.
	 * */
	tlm_reg_default(&tlm);

	/* Try to load all above params from flash.
	 * */
	flash_block_regs_load();
}

void task_INIT(void *pData)
{
	uint32_t		seed[3];

	GPIO_set_mode_OUTPUT(GPIO_LED_ALERT);
	GPIO_set_HIGH(GPIO_LED_ALERT);

#ifdef GPIO_LED_MODE
	GPIO_set_mode_OUTPUT(GPIO_LED_MODE);
	GPIO_set_LOW(GPIO_LED_MODE);
#endif /* GPIO_LED_MODE */

	RNG_startup();
	TIM_startup();

	log_bootup();

	io_USART.getc = &USART_getc;
	io_USART.putc = &USART_putc;

#ifdef HW_HAVE_USB_OTG_FS
	io_USB.getc = &USB_getc;
	io_USB.putc = &USB_putc;
#endif /* HW_HAVE_USB_OTG_FS */

#ifdef HW_HAVE_NETWORK_EPCAN
	io_CAN.getc = &EPCAN_getc;
	io_CAN.putc = &EPCAN_putc;
#endif /* HW_HAVE_NETWORK_EPCAN */

	/* Default to USART.
	 * */
	iodef = &io_USART;

	iodef_ECHO = 1;
	iodef_PRETTY = 1;

	ap.lc_flag = 1;
	ap.lc_tick = 0;

	vTaskDelay(RTOS_LOAD_DELAY);

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
	rseed = seed[2] ^ RNG_make_UID();

	/* Load the configuration.
	 * */
	default_flash_load();

#ifdef HW_HAVE_PART_DRV_XX
	DRV_startup();
#endif /* HW_HAVE_PART_DRV_XX */

	OPT_startup();

#ifdef GPIO_BOOST_EN
	GPIO_set_mode_OUTPUT(GPIO_BOOST_EN);
	GPIO_set_HIGH(GPIO_BOOST_EN);
#endif /* GPIO_BOOST_EN */

#ifdef GPIO_FAN_EN
	GPIO_set_mode_OPEN_DRAIN(GPIO_FAN_EN);
	GPIO_set_mode_OUTPUT(GPIO_FAN_EN);
	GPIO_set_HIGH(GPIO_FAN_EN);
#endif /* GPIO_FAN_EN */

	hal_lock_irq();

	/* Do CORE startup.
	 * */
	ADC_startup();
	PWM_startup();
	WD_startup();

	pm.freq_hz = hal.PWM_frequency;
	pm.dT = 1.f / pm.freq_hz;
	pm.dc_resolution = hal.PWM_resolution;

	hal_unlock_irq(0);

	DPS_startup();
	PPM_startup();

	USART_startup();

#ifdef HW_HAVE_USB_OTG_FS
	USB_startup();
#endif /* HW_HAVE_USB_OTG_FS */

#ifdef HW_HAVE_NETWORK_EPCAN
	EPCAN_startup();
#endif /* HW_HAVE_NETWORK_EPCAN */

	xTaskCreate(task_TEMP, "TEMP", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_ALERT, "ALERT", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

#ifdef HW_HAVE_ANALOG_KNOB
	xTaskCreate(task_KNOB, "KNOB", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
#endif /* HW_HAVE_ANALOG_KNOB */

	xTaskCreate(task_SH, "SH", 400, NULL, 1, NULL);

	GPIO_set_LOW(GPIO_LED_ALERT);

	pm.fsm_req = PM_STATE_ZERO_DRIFT;
	pm_wait_for_IDLE();

	app_startup_by_ID(ap.auto_APP[0]);
	app_startup_by_ID(ap.auto_APP[1]);

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

	if (scaled < - 1.f) {

		scaled = - 1.f;

		if (ap.ppm_ACTIVE == PM_ENABLED) {

			if (elapsed_IDLE() != 0) {

				if (pm.lu_MODE != PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				}

				ap.ppm_ACTIVE = PM_DISABLED;
			}
		}
		else if (ap.ppm_LOCKED == PM_ENABLED) {

			if (elapsed_SAFETY() != 0) {

				ap.ppm_LOCKED = PM_DISABLED;
			}
		}
	}
	else {
		if (scaled > 1.f) {

			scaled = 1.f;
		}

		if (		ap.ppm_STARTUP == PM_ENABLED
				&& ap.ppm_ACTIVE != PM_ENABLED) {

			if (ap.ppm_LOCKED == PM_ENABLED) {

				/* Safety LOCK */
			}
			else if (pm.lu_MODE == PM_LU_DISABLED) {

				ap.ppm_ACTIVE = PM_ENABLED;
				pm.fsm_req = PM_STATE_LU_STARTUP;
			}
		}
	}

	if (scaled < 0.f) {

		range = ap.ppm_control_range[1] - ap.ppm_control_range[0];
		control = ap.ppm_control_range[1] + range * scaled;
	}
	else {
		range = ap.ppm_control_range[2] - ap.ppm_control_range[1];
		control = ap.ppm_control_range[1] + range * scaled;
	}

	if (		ap.ppm_STARTUP != PM_ENABLED
			|| ap.ppm_ACTIVE == PM_ENABLED) {

		reg_SET_F(ap.ppm_reg_ID, control);
	}
}

static void
in_PULSE_WIDTH()
{
	float		pulse;

	if (hal.PPM_caught != 0) {

		pulse = PPM_get_PULSE();

		if (ap.ppm_reg_ID != ID_NULL) {

			inner_PULSE_WIDTH(pulse);
		}
	}
	else {
		if (ap.ppm_ACTIVE == PM_ENABLED) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}

			ap.ppm_ACTIVE = PM_DISABLED;
		}

		if (ap.ppm_LOCKED != PM_ENABLED) {

			ap.ppm_LOCKED = PM_ENABLED;
		}
	}
}

static void
in_STEP_DIR()
{
	int		EP, relEP;
	float		xSP;

	EP = PPM_get_STEP_DIR();

	relEP = (EP - ap.step_baseEP) & 0xFFFFU;
	ap.step_baseEP = EP;

	if (relEP != 0) {

		ap.step_accuEP += relEP;

		if (ap.step_reg_ID != ID_NULL) {

			xSP = ap.step_accuEP * ap.step_const_ld_EP;

			if (		ap.step_STARTUP != PM_ENABLED
					|| ap.step_ACTIVE == PM_ENABLED) {

				reg_SET_F(ap.step_reg_ID, xSP);
			}
		}

		if (ap.step_STARTUP == PM_ENABLED) {

			if (		pm.lu_MODE == PM_LU_DISABLED
					&& pm.fsm_errno == PM_OK) {

				ap.step_ACTIVE = PM_ENABLED;
				pm.fsm_req = PM_STATE_LU_STARTUP;
			}
		}
	}
}

void ADC_IRQ()
{
	pmfb_t		fb;

	fb.current_A = hal.ADC_current_A;
	fb.current_B = hal.ADC_current_B;
	fb.current_C = hal.ADC_current_C;
	fb.voltage_U = hal.ADC_voltage_U;
	fb.voltage_A = hal.ADC_voltage_A;
	fb.voltage_B = hal.ADC_voltage_B;
	fb.voltage_C = hal.ADC_voltage_C;

	fb.pulse_HS = 0;
	fb.pulse_EP = 0;

	if (hal.DPS_mode == DPS_DRIVE_HALL) {

		fb.pulse_HS = DPS_get_HALL();
	}
	else if (hal.DPS_mode == DPS_DRIVE_ABI) {

		fb.pulse_EP = DPS_get_EP();
	}
	else if (hal.DPS_mode == DPS_DRIVE_SOFTWARE) {

		fb.pulse_EP = ap.pulse_EP;
	}

	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		in_PULSE_WIDTH();
	}
	else if (hal.PPM_mode == PPM_STEP_DIR) {

		in_STEP_DIR();
	}
	else if (hal.PPM_mode == PPM_BACKUP_ABI) {

		fb.pulse_EP = PPM_get_backup_EP();
	}

#ifdef HW_HAVE_PART_DRV_XX
	if (		pm.lu_MODE != PM_LU_DISABLED
			&& DRV_fault() != 0) {

		pm.fsm_errno = PM_ERROR_HW_OVERCURRENT;
		pm.fsm_req = PM_STATE_HALT;
	}
#endif /* HW_HAVE_PART_DRV_XX */

	pm_feedback(&pm, &fb);

#ifdef HW_HAVE_NETWORK_EPCAN
	EPCAN_pipe_REGULAR();
#endif /* HW_HAVE_NETWORK_EPCAN */

	tlm_reg_grab(&tlm);

	WD_kick();
}

void app_MAIN()
{
	xTaskCreate(task_INIT, "INIT", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();
}

const char *app_name_by_ID(int app_ID)
{
	const char		*name = ap_list[0].name;

	if (app_ID > 0 && app_ID < APP_LIST_MAX) {

		name = ap_list[app_ID].name;
	}

	return name;
}

void app_startup_by_ID(int app_ID)
{
	TaskHandle_t		xHandle;

	if (app_ID > 0 && app_ID < APP_LIST_MAX) {

		xHandle = xTaskGetHandle(ap_list[app_ID].name);

		if (xHandle == NULL) {

			ap_run[app_ID].task = &ap_list[app_ID];
			ap_run[app_ID].onquit = 0;

			xTaskCreate(ap_list[app_ID].ptask, ap_list[app_ID].name,
					configMINIMAL_STACK_SIZE,
					(void *) &ap_run[app_ID], 1, NULL);
		}
	}
}

void app_halt()
{
	int			N = 1;

	while (ap_list[N].name != NULL) {

		ap_run[N].onquit = 1;

		N++;
	}

#ifdef GPIO_BOOST_EN
	GPIO_set_LOW(GPIO_BOOST_EN);
#endif /* GPIO_BOOST_EN */

#ifdef HW_HAVE_PART_DRV_XX
	DRV_halt();
#endif /* HW_HAVE_PART_DRV_XX */

	vTaskDelay((TickType_t) 50);
}

SH_DEF(rtos_version)
{
	uint32_t	flash_sizeof, flash_crc32;
	int		verified;

	printf("HW_revision \"%s\"" EOL, fw.hwrevision);
	printf("FW_build \"%s\"" EOL, fw.build);

	flash_sizeof = fw.ld_end - fw.ld_begin;
	flash_crc32 = * (uint32_t *) fw.ld_end;

	verified = (crc32b((const void *) fw.ld_begin, flash_sizeof) == flash_crc32) ? 1 : 0;

	printf("FW_sizeof %i" EOL, flash_sizeof);
	printf("FW_crc32 %8x (%s)" EOL, flash_crc32, (verified != 0) ? "verified" : "corrupted");
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

	vTaskDelay(RTOS_LOAD_DELAY);

	ap.lc_flag = 0;

	pc = 100.f * (float) (ap.lc_idle - ap.lc_tick) / (float) ap.lc_idle;

	printf("%1f (%%)" EOL, &pc);
}

SH_DEF(rtos_task_info)
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
					(uint32_t) pLIST[N].xHandle,
					(int) pLIST[N].xTaskNumber,
					pLIST[N].pcTaskName,
					(int) xState,
					(int) pLIST[N].uxCurrentPriority,
					(uint32_t) pLIST[N].pxStackBase,
					(int) pLIST[N].usStackHighWaterMark);
		}

		vPortFree(pLIST);
	}
}

SH_DEF(rtos_app_info)
{
	const char		*sRUN;
	int			N = 1;

	printf("ID Name              Stat" EOL);

	while (ap_list[N].name != NULL) {

		sRUN = (xTaskGetHandle(ap_list[N].name) != NULL) ? "RUN" : "   ";

		printf("%2i %17s %s" EOL, N, ap_list[N].name, sRUN);

		N++;
	}
}

SH_DEF(rtos_app_start)
{
	int			N = 1;

	if (stoi(&N, s) != NULL) {

		app_startup_by_ID(N);
	}
	else {
		while (ap_list[N].name != NULL) {

			if (strcmp(s, ap_list[N].name) == 0) {

				app_startup_by_ID(N);
				break;
			}

			N++;
		}
	}
}

SH_DEF(rtos_app_stop)
{
	int			N = 1;

	if (stoi(&N, s) != NULL) {

		ap_run[N].onquit = 1;
	}
	else {
		while (ap_list[N].name != NULL) {

			if (strcmp(s, ap_list[N].name) == 0) {

				ap_run[N].onquit = 1;
				break;
			}

			N++;
		}
	}
}

SH_DEF(rtos_hexdump)
{
	uint8_t			*m_dump;
	int			l, j, b_ascii, n_lines = 4;

	if (htoi((int *) &m_dump, s) != NULL) {

		stoi(&n_lines, sh_next_arg(s));

		for (l = 0; l < n_lines; ++l) {

			printf("%8x  ", (uint32_t) m_dump);

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

SH_DEF(rtos_heap_info)
{
	printf("FreeHeap %iK %iK" EOL, xPortGetFreeHeapSize() / 1024U,
			xPortGetMinimumEverFreeHeapSize() / 1024U);
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
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	app_halt();
	hal_system_reset();
}

SH_DEF(rtos_bootload)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	app_halt();
	hal_bootload_reset();
}

