#include <stddef.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "main.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "epcan.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "libc.h"
#include "regfile.h"
#include "shell.h"

app_main_t			ap;
pmc_t 				pm LD_CCRAM;
tlm_t				tlm;

uint8_t				ucHeap[configTOTAL_HEAP_SIZE] LD_CCRAM;

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

void vAssertHook(const char *file, int line)
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS: Assert in %s:%i" EOL, file, line);

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

void vApplicationIdleHook()
{
	hal_cpu_sleep();
}

#ifdef HW_HAVE_ANALOG_KNOB
static float
ADC_get_knob_ANG()
{
	float			knob;

	knob = ADC_get_sample(GPIO_ADC_KNOB_ANG);

	return knob * hal.const_ADC.GK;
}

#ifdef HW_HAVE_BRAKE_KNOB
static float
ADC_get_knob_BRK()
{
	float			knob;

	knob = ADC_get_sample(GPIO_ADC_KNOB_BRK);

	return knob * hal.const_ADC.GK;
}
#endif /* HW_HAVE_BRAKE_KNOB */
#endif /* HW_HAVE_ANALOG_KNOB */

#ifdef HW_HAVE_FAN_CONTROL
static void
GPIO_set_mode_FAN()
{
	GPIO_set_mode_OUTPUT(GPIO_FAN_EN);

#ifdef HW_FAN_OPEN_DRAIN
	GPIO_set_mode_OPEN_DRAIN(GPIO_FAN_EN);
#else /* HW_HW_FAN_OPEN_DRAIN */
	GPIO_set_mode_PUSH_PULL(GPIO_FAN_EN);
#endif
}

static void
GPIO_set_state_FAN(int knob)
{
#ifdef HW_FAN_OPEN_DRAIN
	if (knob == PM_ENABLED) {
#else /* HW_HW_FAN_OPEN_DRAIN */
	if (knob != PM_ENABLED) {
#endif
		GPIO_set_LOW(GPIO_FAN_EN);
	}
	else {
		GPIO_set_HIGH(GPIO_FAN_EN);
	}
}
#endif /* HW_HAVE_FAN_CONTROL */

static int
elapsed_IDLE()
{
	TickType_t		xIDLE, xNOW;

	xIDLE = (TickType_t) (ap.idle_timeout * (float) configTICK_RATE_HZ);

	if (xIDLE > 0) {

		xNOW = xTaskGetTickCount();

		if (xNOW - (TickType_t) ap.idle_INVOKE > (TickType_t) 50) {

			ap.idle_RESET = xNOW;
		}

		ap.idle_INVOKE = xNOW;

		if (pm.lu_total_revol != ap.idle_revol_cached) {

			ap.idle_RESET = xNOW;
			ap.idle_revol_cached = pm.lu_total_revol;
		}

		if (xNOW - (TickType_t) ap.idle_RESET > xIDLE) {

			return PM_ENABLED;
		}
	}

	return PM_DISABLED;
}

static int
elapsed_DISARM()
{
	TickType_t		xDISARM, xNOW;

	xDISARM = (TickType_t) (ap.disarm_timeout * (float) configTICK_RATE_HZ);

	if (xDISARM > 0) {

		xNOW = xTaskGetTickCount();

		if (xNOW - (TickType_t) ap.disarm_INVOKE > (TickType_t) 50) {

			ap.disarm_RESET = xNOW;
		}

		ap.disarm_INVOKE = xNOW;

		if (xNOW - (TickType_t) ap.disarm_RESET > xDISARM) {

			return PM_ENABLED;
		}
	}
	else {
		return PM_ENABLED;
	}

	return PM_DISABLED;
}

LD_TASK void task_TEMP(void *pData)
{
	TickType_t		xWake;
	float			x_PCB, x_EXT, lock_PCB;

	if (ap.ntc_PCB.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_PCB.gpio);
	}

	if (ap.ntc_EXT.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_EXT.gpio);
	}

	xWake = xTaskGetTickCount();

	x_PCB = PM_MAX_F;
	x_EXT = PM_MAX_F;

	lock_PCB = 0.f;

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		ap.temp_MCU = ADC_get_sample(GPIO_ADC_TEMPINT);

		if (ap.ntc_PCB.type != NTC_NONE) {

			ap.temp_PCB = ntc_read_temperature(&ap.ntc_PCB);
		}
		else {
			ap.temp_PCB = ap.temp_MCU;
		}

		if (ap.ntc_EXT.type != NTC_NONE) {

			ap.temp_EXT = ntc_read_temperature(&ap.ntc_EXT);
		}
		else {
			ap.temp_EXT = 0.f;
		}

		if (ap.temp_PCB > ap.heat_PCB_temp_halt - lock_PCB) {

			x_PCB = 0.f;

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_errno = PM_ERROR_HW_OVERTEMPERATURE;
				pm.fsm_req = PM_STATE_HALT;
			}

			lock_PCB = ap.heat_temp_recovery;
		}
		else {
			if (ap.temp_PCB > ap.heat_PCB_temp_derate) {

				/* Derate current in case of PCB thermal overload.
				 * */
				x_PCB = ap.heat_derated_PCB;
			}
			else if (ap.temp_PCB < ap.heat_PCB_temp_derate - ap.heat_temp_recovery) {

				x_PCB = PM_MAX_F;
			}

			lock_PCB = 0.f;
		}

#ifdef HW_HAVE_FAN_CONTROL
		/* Enable FAN in case of PCB is warm enough.
		 * */
		if (ap.temp_PCB > ap.heat_PCB_temp_FAN) {

			GPIO_set_state_FAN(PM_ENABLED);
		}
		else if (ap.temp_PCB < ap.heat_PCB_temp_FAN - ap.heat_temp_recovery) {

			GPIO_set_state_FAN(PM_DISABLED);
		}
#endif /* HW_HAVE_FAN_CONTROL */

		if (ap.heat_EXT_temp_derate > M_EPSILON) {

			if (ap.temp_EXT > ap.heat_EXT_temp_derate) {

				/* Derate current in case of external thermal
				 * overload (machine overheat protection).
				 * */
				x_EXT = ap.heat_derated_EXT;
			}
			else if (ap.temp_EXT < ap.heat_EXT_temp_derate - ap.heat_temp_recovery) {

				x_EXT = PM_MAX_F;
			}
		}

		pm.i_derate_on_PCB = (x_PCB < x_EXT) ? x_PCB : x_EXT;

#ifdef HW_HAVE_DRV_ON_PCB
		if (		hal.DRV.auto_RESET == PM_ENABLED
				&& DRV_fault() != 0) {

			DRV_status();

			log_TRACE("DRV fault %4x" EOL, hal.DRV.status_raw);

			DRV_halt();
			DRV_startup();
		}
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef GPIO_LED_MODE
		if (pm.lu_MODE != PM_LU_DISABLED) {

			GPIO_set_HIGH(GPIO_LED_MODE);
		}
		else {
			GPIO_set_LOW(GPIO_LED_MODE);
		}
#endif /* GPIO_LED_MODE */

		if (		pm.fsm_errno != PM_OK
				|| log_status() != 0) {

			if ((xWake & (TickType_t) 0x3FFU) < (TickType_t) 205) {

				GPIO_set_HIGH(GPIO_LED_ALERT);
			}
			else {
				GPIO_set_LOW(GPIO_LED_ALERT);
			}
		}
	}
	while (1);
}

#ifdef HW_HAVE_ANALOG_KNOB
static void
conv_KNOB()
{
	float			control, range, scaled;

	if (		ap.knob_ACTIVE == PM_ENABLED
			&& pm.lu_MODE == PM_LU_DISABLED) {

		ap.knob_ACTIVE = PM_DISABLED;
		ap.knob_DISARM = PM_ENABLED;
	}

	if (		   ap.knob_in_ANG < ap.knob_range_LST[0]
			|| ap.knob_in_ANG > ap.knob_range_LST[1]) {

		/* Loss of KNOB signal.
		 * */
		scaled = - 1.f;

		if (ap.knob_ACTIVE == PM_ENABLED) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_errno = PM_ERROR_KNOB_CONTROL_FAULT;
				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}

			ap.knob_ACTIVE = PM_DISABLED;
		}

		ap.knob_DISARM = PM_ENABLED;
	}
	else {
		if (ap.knob_in_ANG < ap.knob_range_ANG[1]) {

			range = ap.knob_range_ANG[0] - ap.knob_range_ANG[1];
			scaled = (ap.knob_range_ANG[1] - ap.knob_in_ANG) / range;
		}
		else {
			range = ap.knob_range_ANG[2] - ap.knob_range_ANG[1];
			scaled = (ap.knob_in_ANG - ap.knob_range_ANG[1]) / range;
		}

		if (scaled < - 1.f) {

			scaled = - 1.f;

			if (ap.knob_ACTIVE == PM_ENABLED) {

				if (elapsed_IDLE() == PM_ENABLED) {

					if (pm.lu_MODE != PM_LU_DISABLED) {

						pm.fsm_req = PM_STATE_LU_SHUTDOWN;
					}

					ap.knob_ACTIVE = PM_DISABLED;
				}
			}
			else if (ap.knob_DISARM == PM_ENABLED) {

				if (elapsed_DISARM() == PM_ENABLED) {

					ap.knob_DISARM = PM_DISABLED;
				}
			}
		}
		else {
			if (scaled > 1.f) {

				scaled = 1.f;
			}

			if (		ap.knob_STARTUP == PM_ENABLED
					&& ap.knob_ACTIVE != PM_ENABLED) {

				if (ap.knob_DISARM == PM_ENABLED) {

					/* DISARMED */
				}
				else if (pm.lu_MODE == PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_STARTUP;
					ap.knob_ACTIVE = PM_ENABLED;
				}
			}
		}
	}

	if (scaled < 0.f) {

		range = ap.knob_control_ANG[1] - ap.knob_control_ANG[0];
		control = ap.knob_control_ANG[1] + range * scaled;
	}
	else {
		range = ap.knob_control_ANG[2] - ap.knob_control_ANG[1];
		control = ap.knob_control_ANG[1] + range * scaled;
	}

#ifdef HW_HAVE_BRAKE_KNOB
	if (		   ap.knob_in_BRK < ap.knob_range_LST[0]
			|| ap.knob_in_BRK > ap.knob_range_LST[1]) {

		/* Loss of BRAKE signal.
		 * */
	}
	else if (ap.knob_BRAKE == PM_ENABLED) {

		range = ap.knob_range_BRK[1] - ap.knob_range_BRK[0];
		scaled = (ap.knob_in_BRK - ap.knob_range_BRK[0]) / range;

		scaled = (scaled < 0.f) ? 0.f : (scaled > 1.f) ? 1.f : scaled;

		control += (ap.knob_control_BRK - control) * scaled;
	}
#endif /* HW_HAVE_BRAKE_KNOB */

	if (ap.knob_reg_DATA != control) {

		ap.knob_reg_DATA = control;

		if (ap.knob_reg_ID != ID_NULL) {

			reg_SET_F(ap.knob_reg_ID, ap.knob_reg_DATA);
		}
	}
}

LD_TASK void task_KNOB(void *pData)
{
	TickType_t		xWake;

	GPIO_set_mode_ANALOG(GPIO_ADC_KNOB_ANG);

#ifdef HW_HAVE_BRAKE_KNOB
	GPIO_set_mode_ANALOG(GPIO_ADC_KNOB_BRK);
#endif /* HW_HAVE_BRAKE_KNOB */

	xWake = xTaskGetTickCount();

	do {
		if (ap.knob_ENABLED == PM_ENABLED) {

			/* 100 Hz.
			 * */
			vTaskDelayUntil(&xWake, (TickType_t) 10);

			ap.knob_in_ANG = ADC_get_knob_ANG();

#ifdef HW_HAVE_BRAKE_KNOB
			ap.knob_in_BRK = ADC_get_knob_BRK();
#endif /* HW_HAVE_BRAKE_KNOB */

			conv_KNOB();
		}
		else {
			/* 10 Hz.
			 * */
			vTaskDelayUntil(&xWake, (TickType_t) 100);

			ap.knob_in_ANG = ADC_get_knob_ANG();

#ifdef HW_HAVE_BRAKE_KNOB
			ap.knob_in_BRK = ADC_get_knob_BRK();
#endif /* HW_HAVE_BRAKE_KNOB */
		}
	}
	while (1);
}
#endif /* HW_HAVE_ANALOG_KNOB */

static void
default_flash_load()
{
	float			volt_D, halt_I, halt_U;

	hal.USART_baudrate = 57600;
	hal.USART_parity = PARITY_EVEN;

	hal.PWM_frequency = HW_PWM_FREQUENCY_HZ;
	hal.PWM_deadtime = HW_PWM_DEADTIME_NS;
	hal.ADC_reference_voltage = HW_ADC_REFERENCE_VOLTAGE;
	hal.ADC_shunt_resistance = HW_ADC_SHUNT_RESISTANCE;
	hal.ADC_amplifier_gain = HW_ADC_AMPLIFIER_GAIN;
	hal.ADC_voltage_ratio = HW_ADC_VOLTAGE_R2 / (HW_ADC_VOLTAGE_R1 + HW_ADC_VOLTAGE_R2);

	volt_D =  HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2
		+ HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_R3
		+ HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R3;

	hal.ADC_terminal_ratio = HW_ADC_VOLTAGE_R2 * HW_ADC_VOLTAGE_R3 / volt_D;
	hal.ADC_terminal_bias = HW_ADC_VOLTAGE_R1 * HW_ADC_VOLTAGE_R2
				* hal.ADC_reference_voltage / volt_D;

#ifdef HW_HAVE_ANALOG_KNOB
	hal.ADC_knob_ratio = HW_ADC_KNOB_R2 / (HW_ADC_KNOB_R1 + HW_ADC_KNOB_R2);
#endif /* HW_HAVE_ANALOG_KNOB */

	hal.ADC_sample_time = ADC_SMP_28;
	hal.ADC_sample_advance = ADC_SAMPLE_ADVANCE;

	hal.DPS_mode = DPS_DISABLED;
	hal.PPM_mode = PPM_DISABLED;
	hal.PPM_frequency = 2000000;	/* (Hz) */

#ifdef HW_HAVE_STEP_DIR_KNOB
	hal.STEP_mode = STEP_DISABLED;
	hal.STEP_frequency = 500000;	/* (Hz) */
#endif /* HW_HAVE_STEP_DIR_KNOB */

#ifdef HW_HAVE_DRV_ON_PCB
	hal.DRV.part = HW_DRV_PARTNO;
	hal.DRV.auto_RESET = PM_ENABLED;
	hal.DRV.gpio_GATE_EN = GPIO_DRV_GATE_EN;
	hal.DRV.gpio_FAULT = GPIO_DRV_FAULT;
	hal.DRV.gate_current = HW_DRV_GATE_CURRENT;
	hal.DRV.ocp_level = HW_DRV_OCP_LEVEL;
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef HW_HAVE_NETWORK_EPCAN
	net.node_ID = 0;
	net.log_MODE = EPCAN_LOG_DISABLED;
	net.timeout_EP = 100 * HW_PWM_FREQUENCY_HZ / 1000;
	net.ep[0].ID = 10;
	net.ep[0].rate = HW_PWM_FREQUENCY_HZ / 1000;
	net.ep[1].ID = 20;
	net.ep[1].rate = net.ep[0].rate;
	net.ep[2].ID = 30;
	net.ep[2].rate = net.ep[0].rate;
	net.ep[3].ID = 40;
	net.ep[3].rate = net.ep[0].rate;
	net.ep[4].ID = 50;
	net.ep[4].rate = net.ep[0].rate;
	net.ep[5].ID = 60;
	net.ep[5].rate = net.ep[0].rate;
	net.ep[6].ID = 70;
	net.ep[6].rate = net.ep[0].rate;
	net.ep[7].ID = 80;
	net.ep[7].rate = net.ep[0].rate;
#endif /* HW_HAVE_NETWORK_EPCAN */

	ap.ppm_reg_ID = ID_PM_S_SETPOINT_SPEED_KNOB;
	ap.ppm_STARTUP = PM_DISABLED;
	ap.ppm_DISARM = PM_ENABLED;
	ap.ppm_range[0] = 1.0f;		/* (ms) */
	ap.ppm_range[1] = 1.5f;		/* (ms) */
	ap.ppm_range[2] = 2.0f;		/* (ms) */
	ap.ppm_control[0] = 0.f;
	ap.ppm_control[1] = 50.f;
	ap.ppm_control[2] = 100.f;

#ifdef HW_HAVE_STEP_DIR_KNOB
	ap.step_reg_ID = ID_PM_X_SETPOINT_LOCATION;
	ap.step_STARTUP = PM_DISABLED;
	ap.step_const_S = 0.f;
#endif /* HW_HAVE_STEP_DIR_KNOB */

#ifdef HW_HAVE_ANALOG_KNOB
	ap.knob_reg_ID = ID_PM_S_SETPOINT_SPEED_KNOB;
	ap.knob_ENABLED = PM_DISABLED;
#ifdef HW_HAVE_BRAKE_KNOB
	ap.knob_BRAKE = PM_DISABLED;
#endif /* HW_HAVE_BRAKE_KNOB */
	ap.knob_STARTUP = PM_DISABLED;
	ap.knob_DISARM = PM_ENABLED;
	ap.knob_range_ANG[0] = 1.0f;	/* (V) */
	ap.knob_range_ANG[1] = 2.5f;	/* (V) */
	ap.knob_range_ANG[2] = 4.0f;	/* (V) */
#ifdef HW_HAVE_BRAKE_KNOB
	ap.knob_range_BRK[0] = 2.0f;	/* (V) */
	ap.knob_range_BRK[1] = 4.0f;	/* (V) */
#endif /* HW_HAVE_BRAKE_KNOB */
	ap.knob_range_LST[0] = 0.2f;	/* (V) */
	ap.knob_range_LST[1] = 4.8f;	/* (V) */
	ap.knob_control_ANG[0] = 0.f;
	ap.knob_control_ANG[1] = 50.f;
	ap.knob_control_ANG[2] = 100.f;
	ap.knob_control_BRK = - 100.f;
#endif /* HW_HAVE_ANALOG_KNOB */

	ap.idle_timeout = 2.f;		/* (s) */
	ap.disarm_timeout = 1.f;	/* (s) */

	ap.auto_reg_DATA = 0.f;
	ap.auto_reg_ID = ID_PM_S_SETPOINT_SPEED_KNOB;

#ifdef HW_HAVE_NTC_ON_PCB
	ap.ntc_PCB.type = HW_NTC_PCB_TYPE;
	ap.ntc_PCB.gpio = GPIO_ADC_NTC_PCB;
	ap.ntc_PCB.balance = HW_NTC_PCB_BALANCE;
	ap.ntc_PCB.ntc0 = HW_NTC_PCB_NTC0;
	ap.ntc_PCB.ta0 = HW_NTC_PCB_TA0;
	ap.ntc_PCB.betta = HW_NTC_PCB_BETTA;
#endif /* HW_HAVE_NTC_ON_PCB */

#ifdef HW_HAVE_NTC_MACHINE
	ap.ntc_EXT.type = NTC_GND;
	ap.ntc_EXT.gpio = GPIO_ADC_NTC_EXT;
	ap.ntc_EXT.balance = HW_NTC_EXT_BALANCE;
	ap.ntc_EXT.ntc0 = 10000.f;
	ap.ntc_EXT.ta0 = 25.f;
	ap.ntc_EXT.betta = 3380.f;
#endif /* HW_HAVE_NTC_MACHINE */

	ap.heat_PCB_temp_halt = 110.f;	/* (C) */
	ap.heat_PCB_temp_derate = 90.f;	/* (C) */
	ap.heat_PCB_temp_FAN = 60.f;	/* (C) */
	ap.heat_EXT_temp_derate = 0.f;	/* (C) */
	ap.heat_derated_PCB = 20.f;	/* (A) */
	ap.heat_derated_EXT = 20.f;	/* (A) */
	ap.heat_temp_recovery = 5.f;	/* (C) */

	pm.m_freq = hal.PWM_frequency;
	pm.m_dT = 1.f / pm.m_freq;
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

#if ADC_HAVE_SEQUENCE__ABC(HW_ADC_SAMPLING_SEQUENCE)
#ifdef HW_HAVE_LOW_SIDE_SHUNT
	pm.config_IFB = PM_IFB_ABC_GND;
#else /* HW_HAVE_LOW_SIDE_SHUNT */
	pm.config_IFB = PM_IFB_ABC_INLINE;
#endif
#else /* ADC_HAVE_SEQUENCE__ABC(HW_ADC_SAMPLING_SEQUENCE) */
#ifdef HW_HAVE_LOW_SIDE_SHUNT
	pm.config_IFB = PM_IFB_AB_GND;
#else /* HW_HAVE_LOW_SIDE_SHUNT */
	pm.config_IFB = PM_IFB_AB_INLINE;
#endif
#endif

#if ADC_HAVE_SEQUENCE__TTT(HW_ADC_SAMPLING_SEQUENCE)
	pm.config_TVM = PM_ENABLED;
#else /* ADC_HAVE_SEQUENCE__TTT(HW_ADC_SAMPLING_SEQUENCE) */
	pm.config_TVM = PM_DISABLED;
#endif

	ADC_const_build();

	halt_I = m_fabsf(hal.const_ADC.GA * ADC_RESOLUTION / 2.f);
	halt_U = m_fabsf(hal.const_ADC.GU * ADC_RESOLUTION);

	pm.fault_current_halt = (float) (int) (.95f * halt_I);
	pm.fault_voltage_halt = (float) (int) (.95f * halt_U);

	pm_auto(&pm, PM_AUTO_MAXIMAL_CURRENT);

	pm.watt_wA_maximal = (float) (int) (0.66666667f * pm.i_maximal);
	pm.watt_wA_reverse = pm.watt_wA_maximal;

	pm.watt_uDC_maximal = (float) (int) (pm.fault_voltage_halt - 5.f);

	pm.watt_wP_maximal = (float) (int) (pm.watt_wA_maximal * pm.watt_uDC_maximal);
	pm.watt_wP_reverse = pm.watt_wP_maximal;

#ifdef HW_CONFIG_INLINE
	HW_CONFIG_INLINE;
#endif /* HW_CONFIG_INLINE */

	/* Default telemetry.
	 * */
	tlm_reg_default(&tlm);

	/* Try to load all above params from flash.
	 * */
	flash_block_regs_load();
}

LD_TASK void task_INIT(void *pData)
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

#ifdef HW_HAVE_DRV_ON_PCB
	DRV_startup();
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef GPIO_GATE_EN
	GPIO_set_mode_OUTPUT(GPIO_GATE_EN);
	GPIO_set_HIGH(GPIO_GATE_EN);
#endif /* GPIO_GATE_EN */

#ifdef HW_HAVE_FAN_CONTROL
	GPIO_set_mode_FAN();
	GPIO_set_state_FAN(PM_DISABLED);
#endif /* HW_HAVE_FAN_CONTROL */

	hal_lock_irq();

	/* Do CORE startup.
	 * */
	ADC_startup();
	PWM_startup();
	WD_startup();

	pm.m_freq = hal.PWM_frequency;
	pm.m_dT = 1.f / pm.m_freq;
	pm.dc_resolution = hal.PWM_resolution;

	pm.fsm_req = PM_STATE_ZERO_DRIFT;

	hal_unlock_irq(0);

	DPS_startup();
	PPM_startup();

	io_USART.getc = &USART_getc;
	io_USART.poll = &USART_poll;
	io_USART.putc = &USART_putc;

#ifdef HW_HAVE_USB_CDC_ACM
	io_USB.getc = &USART_getc;
	io_USB.poll = &USART_poll;
	io_USB.putc = &USB_putc;
#endif /* HW_HAVE_USB_CDC_ACM */

#ifdef HW_HAVE_NETWORK_EPCAN
	io_CAN.getc = &USART_getc;
	io_CAN.poll = &USART_poll;
	io_CAN.putc = &EPCAN_putc;
#endif /* HW_HAVE_NETWORK_EPCAN */

	/* Default to USART.
	 * */
	iodef = &io_USART;

	USART_startup();

#ifdef HW_HAVE_USB_CDC_ACM
	USB_startup();
#endif /* HW_HAVE_USB_CDC_ACM */

#ifdef HW_HAVE_NETWORK_EPCAN
	EPCAN_startup();
#endif /* HW_HAVE_NETWORK_EPCAN */

	xTaskCreate(task_TEMP, "TEMP", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

#ifdef HW_HAVE_ANALOG_KNOB
	xTaskCreate(task_KNOB, "KNOB", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
#endif /* HW_HAVE_ANALOG_KNOB */

	xTaskCreate(task_CMDSH, "CMDSH", 220, NULL, 1, NULL);

	GPIO_set_LOW(GPIO_LED_ALERT);

#undef APP_DEF
#define APP_DEF(name)		reg_TOUCH_I(ID_AP_TASK_ ## name);
#include "app/apdefs.h"

	vTaskDelete(NULL);
}

static void
conv_PULSE_WIDTH()
{
	float		control, range, scaled;

	if (		ap.ppm_ACTIVE == PM_ENABLED
			&& pm.lu_MODE == PM_LU_DISABLED) {

		ap.ppm_ACTIVE = PM_DISABLED;
		ap.ppm_DISARM = PM_ENABLED;
	}

	if (ap.ppm_PULSE < ap.ppm_range[1]) {

		range = ap.ppm_range[0] - ap.ppm_range[1];
		scaled = (ap.ppm_range[1] - ap.ppm_PULSE) / range;
	}
	else {
		range = ap.ppm_range[2] - ap.ppm_range[1];
		scaled = (ap.ppm_PULSE - ap.ppm_range[1]) / range;
	}

	if (scaled < - 1.f) {

		scaled = - 1.f;

		if (ap.ppm_ACTIVE == PM_ENABLED) {

			if (elapsed_IDLE() == PM_ENABLED) {

				if (pm.lu_MODE != PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				}

				ap.ppm_ACTIVE = PM_DISABLED;
			}
		}
		else if (ap.ppm_DISARM == PM_ENABLED) {

			if (elapsed_DISARM() == PM_ENABLED) {

				ap.ppm_DISARM = PM_DISABLED;
			}
		}
	}
	else {
		if (scaled > 1.f) {

			scaled = 1.f;
		}

		if (		ap.ppm_STARTUP == PM_ENABLED
				&& ap.ppm_ACTIVE != PM_ENABLED) {

			if (ap.ppm_DISARM == PM_ENABLED) {

				/* DISARMED */
			}
			else if (pm.lu_MODE == PM_LU_DISABLED) {

				pm.fsm_req = PM_STATE_LU_STARTUP;
				ap.ppm_ACTIVE = PM_ENABLED;
			}
		}
	}

	if (scaled < 0.f) {

		range = ap.ppm_control[1] - ap.ppm_control[0];
		control = ap.ppm_control[1] + range * scaled;
	}
	else {
		range = ap.ppm_control[2] - ap.ppm_control[1];
		control = ap.ppm_control[1] + range * scaled;
	}

	if (ap.ppm_reg_DATA != control) {

		ap.ppm_reg_DATA = control;

		if (ap.ppm_reg_ID != ID_NULL) {

			reg_SET_F(ap.ppm_reg_ID, ap.ppm_reg_DATA);
		}
	}
}

static void
in_PULSE_WIDTH()
{
	ap.ppm_PULSE = PPM_get_PULSE() * 1000.f;	/* (ms) */
	ap.ppm_FREQ = PPM_get_PERIOD();

	if (ap.ppm_FREQ > M_EPSILON) {

		conv_PULSE_WIDTH();
	}
	else {
		if (ap.ppm_ACTIVE == PM_ENABLED) {

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_errno = PM_ERROR_KNOB_CONTROL_FAULT;
				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}

			ap.ppm_ACTIVE = PM_DISABLED;
		}

		ap.ppm_DISARM = PM_ENABLED;
	}
}

#ifdef HW_HAVE_STEP_DIR_KNOB
static void
in_STEP_DIR()
{
	int		EP, relEP;
	float		xSP;

	EP = STEP_get_POSITION();

	relEP = EP - ap.step_bEP;
	relEP +=  unlikely(relEP > 0x10000 / 2 - 1) ? - 0x10000
		: unlikely(relEP < - 0x10000 / 2) ? 0x10000 : 0;

	ap.step_bEP = EP;

	if (relEP != 0) {

		ap.step_POS += relEP;

		xSP = ap.step_POS * ap.step_const_S;

		if (ap.step_reg_DATA != xSP) {

			ap.step_reg_DATA = xSP;

			if (ap.step_reg_ID != ID_NULL) {

				reg_SET_F(ap.step_reg_ID, ap.step_reg_DATA);
			}
		}

		if (ap.step_STARTUP == PM_ENABLED) {

			if (		pm.lu_MODE == PM_LU_DISABLED
					&& pm.fsm_errno == PM_OK) {

				pm.fsm_req = PM_STATE_LU_STARTUP;
			}
		}
	}
}
#endif /* HW_HAVE_STEP_DIR_KNOB */

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
	else if (hal.DPS_mode == DPS_DRIVE_EABI) {

		fb.pulse_EP = DPS_get_EP();
	}
	else if (	hal.DPS_mode == DPS_DRIVE_ON_SPI
			&& ap.proc_get_EP != NULL) {

		fb.pulse_EP = ap.proc_get_EP();
	}

	if (hal.PPM_mode == PPM_PULSE_WIDTH) {

		in_PULSE_WIDTH();
	}

#ifdef HW_HAVE_STEP_DIR_KNOB
	if (		hal.STEP_mode == STEP_ON_STEP_DIR
			|| hal.STEP_mode == STEP_ON_CW_CCW) {

		in_STEP_DIR();
	}
#endif /* HW_HAVE_STEP_DIR_KNOB */

	if (unlikely(hal.CNT_diag[1] >= pm.m_dT)) {

		pm.fsm_errno = PM_ERROR_HW_UNMANAGED_IRQ;
		pm.fsm_req = PM_STATE_HALT;
	}

#ifdef HW_HAVE_PWM_BKIN
	if (unlikely(PWM_fault() != 0)) {

		pm.fsm_errno = PM_ERROR_HW_EMERGENCY_STOP;
		pm.fsm_req = PM_STATE_HALT;
	}
#endif /* HW_HAVE_PWM_BKIN */

#ifdef HW_HAVE_DRV_ON_PCB
	if (unlikely(		pm.lu_MODE != PM_LU_DISABLED
				&& DRV_fault() != 0)) {

		pm.fsm_errno = PM_ERROR_HW_OVERCURRENT;
		pm.fsm_req = PM_STATE_HALT;
	}
#endif /* HW_HAVE_DRV_ON_PCB */

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

void app_control(const reg_t *reg, void (* pvTask) (void *), const char *pcName)
{
	TaskHandle_t		xHandle;

	if (reg->link->i == PM_ENABLED) {

		xHandle = xTaskGetHandle(pcName);

		if (xHandle == NULL) {

			xTaskCreate(pvTask, pcName, configMINIMAL_STACK_SIZE,
					(void *) reg->link, 1, NULL);
		}
	}
}

void app_halt()
{
#ifdef HW_HAVE_DRV_ON_PCB
	DRV_halt();
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef GPIO_GATE_EN
	GPIO_set_LOW(GPIO_GATE_EN);
	vTaskDelay((TickType_t) 100);
#endif /* GPIO_GATE_EN */
}

SH_DEF(ap_version)
{
	uint32_t	flash_sizeof, flash_crc32;
	int		rc;

	printf("Revision \"%s\"" EOL, fw.hwrevision);
	printf("Build \"%s\"" EOL, fw.build);

	flash_sizeof = fw.ld_end - fw.ld_begin;
	flash_crc32 = * (uint32_t *) fw.ld_end;

	rc = (crc32b((const void *) fw.ld_begin, flash_sizeof) == flash_crc32) ? 1 : 0;

	printf("CRC32 %8x (%s)" EOL, flash_crc32, (rc != 0) ? "OK" : "does NOT match");
}

SH_DEF(ap_clock)
{
	printf("Clock %i %i" EOL, log.boot_COUNT, xTaskGetTickCount());
}

SH_DEF(ap_task_info)
{
	TaskStatus_t		*list;
	int			len, symStat, n;

	len = uxTaskGetNumberOfTasks();
	list = pvPortMalloc(len * sizeof(TaskStatus_t));

	if (list != NULL) {

		len = uxTaskGetSystemState(list, len, NULL);

		printf("TCB      ID Name              Stat Prio Stack    Free" EOL);

		for (n = 0; n < len; ++n) {

			switch (list[n].eCurrentState) {

				case eRunning:
					symStat = 'R';
					break;

				case eReady:
					symStat = 'E';
					break;

				case eBlocked:
					symStat = 'B';
					break;

				case eSuspended:
					symStat = 'S';
					break;

				case eDeleted:
					symStat = 'D';
					break;

				case eInvalid:
				default:
					symStat = 'N';
					break;
			}

			printf("%8x %2i %17s %c    %2i   %8x %i" EOL,
					(uint32_t) list[n].xHandle,
					(int) list[n].xTaskNumber,
					list[n].pcTaskName, (int) symStat,
					(int) list[n].uxCurrentPriority,
					(uint32_t) list[n].pxStackBase,
					(int) list[n].usStackHighWaterMark);
		}

		vPortFree(list);
	}
}

SH_DEF(ap_heap_info)
{
	HeapStats_t	info;

	vPortGetHeapStats(&info);

	printf("RAM      Total  Free   Minimum" EOL);

	printf("%8x %6i %6i %6i" EOL,
			(uint32_t) ucHeap, configTOTAL_HEAP_SIZE,
			info.xAvailableHeapSpaceInBytes,
			info.xMinimumEverFreeBytesRemaining);

	printf("         %6i %6i %2i %2i %2i" EOL,
			info.xSizeOfLargestFreeBlockInBytes,
			info.xSizeOfSmallestFreeBlockInBytes,
			info.xNumberOfFreeBlocks,
			info.xNumberOfSuccessfulAllocations,
			info.xNumberOfSuccessfulFrees);
}

SH_DEF(ap_log_flush)
{
	if (log.textbuf[0] != 0) {

		puts(log.textbuf);
		puts(EOL);
	}
}

SH_DEF(ap_log_clean)
{
	if (log.textbuf[0] != 0) {

		memset(log.textbuf, 0, sizeof(log.textbuf));

		log.len = 0;
	}
}

SH_DEF(ap_hexdump)
{
	uint8_t		*maddr, mdata[16];
	int		sym, n, i, count = 4;

	if (htoi((int *) &maddr, s) != NULL) {

		stoi(&count, sh_next_arg(s));

		for (n = 0; n < count; ++n) {

			memcpy(mdata, maddr, 16);

			printf("%8x  ", (uint32_t) maddr);

			for (i = 0; i < 8; ++i)
				printf("%2x ", mdata[i]);

			puts(" ");

			for (i = 8; i < 16; ++i)
				printf("%2x ", mdata[i]);

			puts(" |");

			for (i = 0; i < 16; ++i) {

				sym = mdata[i];
				sym = (sym >= 0x20 && sym <= 0x7E) ? sym : '.';

				putc(sym);
			}

			puts("|" EOL);

			maddr += 16;
		}
	}
}

SH_DEF(ap_reboot)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	app_halt();
	hal_system_reset();
}

SH_DEF(ap_bootload)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	app_halt();
	hal_bootload_reset();
}

