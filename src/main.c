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

#include "app/taskdefs.h"

app_t				ap;
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
	log_TRACE("FreeRTOS assert in %s:%i" EOL, file, line);

	hal_system_reset();
}

void vApplicationMallocFailedHook()
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS heap allocation fault" EOL);

	hal_system_reset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	taskDISABLE_INTERRUPTS();
	log_TRACE("FreeRTOS stack overflow in %8x task" EOL, (uint32_t) xTask);

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

	knob = ADC_analog_sample(GPIO_ADC_KNOB_ANG);

	return knob * hal.const_ADC.GK;
}

#ifdef HW_HAVE_BRAKE_KNOB
static float
ADC_get_knob_BRK()
{
	float			knob;

	knob = ADC_analog_sample(GPIO_ADC_KNOB_BRK);

	return knob * hal.const_ADC.GK;
}
#endif /* HW_HAVE_BRAKE_KNOB */
#endif /* HW_HAVE_ANALOG_KNOB */

static int
timeout_DISARM()
{
	TickType_t		xOUT, xNOW;

	xOUT = (TickType_t) (ap.timeout_DISARM * ((float) configTICK_RATE_HZ / 1000.f));

	if (xOUT != 0) {

		xNOW = xTaskGetTickCount();

		if (xNOW - (TickType_t) ap.disarm_INK > (TickType_t) 40) {

			ap.disarm_RST = xNOW;
		}

		ap.disarm_INK = xNOW;

		if (xNOW - (TickType_t) ap.disarm_RST < xOUT) {

			return PM_DISABLED;
		}
	}

	return PM_ENABLED;
}

static int
timeout_IDLE()
{
	TickType_t		xOUT, xNOW;

	xOUT = (TickType_t) (ap.timeout_IDLE * ((float) configTICK_RATE_HZ / 1000.f));

	if (xOUT != 0) {

		xNOW = xTaskGetTickCount();

		if (xNOW - (TickType_t) ap.idle_INK > (TickType_t) 40) {

			ap.idle_RST = xNOW;
		}

		ap.idle_INK = xNOW;

		if (pm.lu_total_revol != ap.idle_revol) {

			ap.idle_RST = xNOW;
			ap.idle_revol = pm.lu_total_revol;
		}

		if (xNOW - (TickType_t) ap.idle_RST > xOUT) {

			return PM_ENABLED;
		}
	}

	return PM_DISABLED;
}

LD_TASK void task_TEMP(void *pData)
{
	TickType_t		xWake;

	float			temp_NTC, maximal_PCB, maximal_EXT;
	float			blend_A, lock_halt_PCB;

	int			last_fsm_errno;

	if (ap.ntc_PCB.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_PCB.gpio);
	}

	if (ap.ntc_EXT.type != NTC_NONE) {

		GPIO_set_mode_ANALOG(ap.ntc_EXT.gpio);
	}

#ifdef HW_HAVE_ALT_FUNCTION
#ifdef GPIO_ALT_CURRENT
	GPIO_set_mode_OUTPUT(GPIO_ALT_CURRENT);
#endif /* GPIO_ALT_CURRENT */
#ifdef GPIO_ALT_VOLTAGE
	GPIO_set_mode_OUTPUT(GPIO_ALT_VOLTAGE);
#endif /* GPIO_ALT_VOLTAGE */
#endif /* HW_HAVE_ALT_FUNCTION  */

	xWake = xTaskGetTickCount();

	maximal_PCB = PM_MAX_F;
	maximal_EXT = PM_MAX_F;

	lock_halt_PCB = 0.f;
	last_fsm_errno = PM_OK;

	do {
		/* 10 Hz.
		 * */
		vTaskDelayUntil(&xWake, (TickType_t) 100);

		ap.temp_MCU = ADC_analog_sample(GPIO_ADC_TEMPINT);

		if (ap.ntc_PCB.type != NTC_NONE) {

			temp_NTC = ntc_read_temperature(&ap.ntc_PCB);

			ap.temp_PCB += (temp_NTC - ap.temp_PCB) * ap.temp_gain_LP;
		}
		else {
			ap.temp_PCB = ap.temp_MCU;
		}

		if (ap.ntc_EXT.type != NTC_NONE) {

			temp_NTC = ntc_read_temperature(&ap.ntc_EXT);

			ap.temp_EXT += (temp_NTC - ap.temp_EXT) * ap.temp_gain_LP;
		}
		else {
			ap.temp_EXT = 0.f;
		}

		if (ap.temp_PCB > ap.otp_PCB_halt - lock_halt_PCB) {

			maximal_PCB = 0.f;

			if (pm.lu_MODE != PM_LU_DISABLED) {

				pm.fsm_errno = PM_ERROR_HW_OVERTEMPERATURE;
				pm.fsm_req = PM_STATE_HALT;
			}

			lock_halt_PCB = ap.otp_derate_tol;
		}
		else {
			temp_NTC = ap.temp_PCB - ap.otp_PCB_derate;

			if (temp_NTC < 0.f) {

				maximal_PCB = PM_MAX_F;
			}
			else {
				blend_A = temp_NTC / ap.otp_derate_tol;
				blend_A = (blend_A > 1.f) ? 1.f : blend_A;

				/* Derate current in case of PCB thermal overload.
				 * */
				maximal_PCB = pm.i_maximal * (1.f - blend_A);
			}

			lock_halt_PCB = 0.f;
		}

#ifdef HW_HAVE_FAN_CONTROL
		/* Enable FAN in case of PCB is warm enough.
		 * */
		if (ap.temp_PCB > ap.otp_PCB_fan) {

			GPIO_set_HIGH(GPIO_FAN_EN);
		}
		else if (ap.temp_PCB < ap.otp_PCB_fan - ap.otp_derate_tol) {

			GPIO_set_LOW(GPIO_FAN_EN);
		}
#endif /* HW_HAVE_FAN_CONTROL */

		if (ap.otp_EXT_derate > M_EPSILON) {

			temp_NTC = ap.temp_EXT - ap.otp_EXT_derate;

			if (temp_NTC < 0.f) {

				maximal_EXT = PM_MAX_F;
			}
			else {
				blend_A = temp_NTC / ap.otp_derate_tol;
				blend_A = (blend_A > 1.f) ? 1.f : blend_A;

				/* Derate current in case of external thermal
				 * overload (machine overheat protection).
				 * */
				maximal_EXT = pm.i_maximal * (1.f - blend_A);
			}
		}

		if (maximal_PCB < maximal_EXT) {

			pm.i_maximal_on_PCB = maximal_PCB;
		}
		else {
			pm.i_maximal_on_PCB = maximal_EXT;
		}

#ifdef HW_HAVE_DRV_ON_PCB
		if (		hal.DRV.auto_RESTART == PM_ENABLED
				&& pm.lu_MODE == PM_LU_DISABLED
				&& DRV_fault() != 0) {

			DRV_status();

			log_TRACE("DRV fault %4x" EOL, hal.DRV.status_raw);

			DRV_halt();
			DRV_startup();
		}
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef HW_HAVE_ALT_FUNCTION
#ifdef GPIO_ALT_CURRENT
		if (hal.ALT_current == PM_ENABLED) {

			GPIO_set_HIGH(GPIO_ALT_CURRENT);
		}
		else {
			GPIO_set_LOW(GPIO_ALT_CURRENT);
		}
#endif /* GPIO_ALT_CURRENT */
#ifdef GPIO_ALT_VOLTAGE
		if (hal.ALT_voltage == PM_ENABLED) {

			GPIO_set_HIGH(GPIO_ALT_VOLTAGE);
		}
		else {
			GPIO_set_LOW(GPIO_ALT_VOLTAGE);
		}
#endif /* GPIO_ALT_VOLTAGE */
#endif /* HW_HAVE_ALT_FUNCTION  */

#ifdef GPIO_LED_MODE
		if (pm.lu_MODE != PM_LU_DISABLED) {

			GPIO_set_HIGH(GPIO_LED_MODE);
		}
		else {
			GPIO_set_LOW(GPIO_LED_MODE);
		}
#endif /* GPIO_LED_MODE */

		if (pm.fsm_errno != PM_OK) {

			if (pm.fsm_errno != last_fsm_errno) {

				log_TRACE("FSM errno %s" EOL, pm_strerror(pm.fsm_errno));
			}

			if ((xWake & (TickType_t) 0x3FFU) < (TickType_t) 0xF0U) {

				GPIO_set_HIGH(GPIO_LED_ALERT);
			}
			else {
				GPIO_set_LOW(GPIO_LED_ALERT);
			}
		}
		else {
			if (		tlm.watch_AUTO == PM_ENABLED
					&& tlm.mode == TLM_MODE_DISABLED
					&& pm.lu_MODE != PM_LU_DISABLED) {

				tlm_startup(&tlm, tlm.rate_watch, TLM_MODE_WATCH);
			}
		}

		last_fsm_errno = pm.fsm_errno;
	}
	while (1);
}

#ifdef HW_HAVE_ANALOG_KNOB
static void
conv_KNOB()
{
	float			control, range, scaled;

	int			hold_FLAG = PM_DISABLED;

	if (		ap.knob_ACTIVE == PM_ENABLED
			&& pm.lu_MODE == PM_LU_DISABLED) {

		ap.knob_ACTIVE = PM_DISABLED;
		ap.knob_DISARM = PM_ENABLED;
	}

	if (		   ap.knob_in_ANG < ap.knob_range_LOS[0]
			|| ap.knob_in_ANG > ap.knob_range_LOS[1]) {

		scaled = - 1.f;

		/* Loss of KNOB signal.
		 * */

		if (ap.knob_ACTIVE == PM_ENABLED) {

			ap.knob_NFAULT++;
		}

		ap.knob_DISARM = PM_ENABLED;
	}
	else {
		if (ap.knob_in_ANG < ap.knob_range_ANG[1]) {

			range = ap.knob_range_ANG[0] - ap.knob_range_ANG[1];
			scaled = (ap.knob_range_ANG[1] - ap.knob_in_ANG) / range;
		}
		else if (ap.knob_in_ANG < ap.knob_range_ANG[2]) {

			scaled = 0.f;
		}
		else {
			range = ap.knob_range_ANG[3] - ap.knob_range_ANG[2];
			scaled = (ap.knob_in_ANG - ap.knob_range_ANG[2]) / range;
		}

		if (scaled < - 1.f) {

			scaled = - 1.f;

			if (ap.knob_ACTIVE == PM_ENABLED) {

				if (timeout_IDLE() == PM_ENABLED) {

					if (pm.lu_MODE != PM_LU_DISABLED) {

						pm.fsm_req = PM_STATE_LU_SHUTDOWN;
					}

					ap.knob_ACTIVE = PM_DISABLED;
				}
			}
			else if (ap.knob_DISARM == PM_ENABLED) {

				if (timeout_DISARM() == PM_ENABLED) {

					ap.knob_DISARM = PM_DISABLED;
				}
			}
		}
		else {
			hold_FLAG = PM_ENABLED;

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

		ap.knob_NFAULT = 0;
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
	if (		   ap.knob_in_BRK < ap.knob_range_LOS[0]
			|| ap.knob_in_BRK > ap.knob_range_LOS[1]) {

		/* Loss of BRAKE signal.
		 * */
	}
	else if (ap.knob_BRAKE == PM_ENABLED) {

		range = ap.knob_range_BRK[1] - ap.knob_range_BRK[0];

		scaled = (ap.knob_in_BRK - ap.knob_range_BRK[0]) / range;
		scaled = (scaled < 0.f) ? 0.f : (scaled > 1.f) ? 1.f : scaled;

		if (scaled > 0.f) {

			hold_FLAG = PM_ENABLED;

			control += (ap.knob_control_BRK - control) * scaled;
		}

		ap.knob_NFAULT = 0;
	}
#endif /* HW_HAVE_BRAKE_KNOB */

	if (ap.knob_ACTIVE == PM_ENABLED) {

		if (unlikely(ap.knob_NFAULT >= 10)) {

			pm.fsm_errno = PM_ERROR_KNOB_SIGNAL_FAULT;
			pm.fsm_req = PM_STATE_LU_SHUTDOWN;

			ap.knob_ACTIVE = PM_DISABLED;
		}
	}

	if (		ap.knob_reg_DATA != control
			|| hold_FLAG == PM_ENABLED) {

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
	float			halt_A, halt_U, tol_A, tol_U;

	hal.USART_baudrate = 57600;
	hal.USART_parity = PARITY_EVEN;

	hal.PWM_frequency = HW_PWM_FREQUENCY_HZ;
	hal.PWM_deadtime = HW_PWM_DEADTIME_NS;
	hal.ADC_reference_voltage = HW_ADC_REFERENCE_VOLTAGE;
	hal.ADC_shunt_resistance = HW_ADC_SHUNT_RESISTANCE;
	hal.ADC_amplifier_gain = HW_ADC_AMPLIFIER_GAIN;
	hal.ADC_voltage_ratio = HW_ADC_VOLTAGE_R2 / (HW_ADC_VOLTAGE_R1 + HW_ADC_VOLTAGE_R2);

#ifndef HW_ADC_TERMINAL_R1
#define HW_ADC_TERMINAL_R1	HW_ADC_VOLTAGE_R1
#endif /* HW_ADC_TERMINAL_R1 */

#ifndef HW_ADC_TERMINAL_R2
#define HW_ADC_TERMINAL_R2	HW_ADC_VOLTAGE_R2
#endif /* HW_ADC_TERMINAL_R2 */

	hal.ADC_terminal_ratio = HW_ADC_TERMINAL_R2 / (HW_ADC_TERMINAL_R1 + HW_ADC_TERMINAL_R2);

#ifdef HW_HAVE_ANALOG_KNOB
	hal.ADC_knob_ratio = HW_ADC_KNOB_R2 / (HW_ADC_KNOB_R1 + HW_ADC_KNOB_R2);
#endif /* HW_HAVE_ANALOG_KNOB */

	hal.ADC_sample_time = ADC_SMP_28;
	hal.ADC_sample_advance = ADC_SAMPLE_ADVANCE;

#ifdef HW_HAVE_NETWORK_EPCAN
	hal.CAN_bitfreq = CAN_BITFREQ_HZ;
#endif /* HW_HAVE_NETWORK_EPCAN */

	hal.DPS_mode = DPS_DISABLED;
	hal.PPM_mode = PPM_DISABLED;
	hal.PPM_frequency = 2000000;	/* (Hz) */

#ifdef HW_HAVE_STEP_DIR_KNOB
	hal.STEP_mode = STEP_DISABLED;
	hal.STEP_frequency = 500000;	/* (Hz) */
#endif /* HW_HAVE_STEP_DIR_KNOB */

#ifdef HW_HAVE_DRV_ON_PCB
	hal.DRV.partno = HW_DRV_PARTNO;
	hal.DRV.auto_RESTART = PM_ENABLED;
	hal.DRV.gpio_GATE_EN = GPIO_DRV_GATE_EN;
	hal.DRV.gpio_FAULT = GPIO_DRV_FAULT;
	hal.DRV.gate_current = HW_DRV_GATE_CURRENT;
	hal.DRV.ocp_level = HW_DRV_OCP_LEVEL;
#endif /* HW_HAVE_DRV_ON_PCB */

#ifdef HW_HAVE_ALT_FUNCTION
	hal.ALT_current = PM_DISABLED;
	hal.ALT_voltage = PM_DISABLED;
#endif /* HW_HAVE_ALT_FUNCTION */

#ifdef HW_HAVE_NETWORK_EPCAN
	net.offset_ID = EPCAN_OFFSET_DEFAULT;
	net.node_ID = 0;
	net.log_MSG = EPCAN_LOG_DISABLED;
	net.timeout_EP = 100 * HW_PWM_FREQUENCY_HZ / 1000;
	net.tlm_ID = EPCAN_TLM_ID_DEFAULT;
	net.ep[0].ID = 0;
	net.ep[0].rate = HW_PWM_FREQUENCY_HZ / 1000;
	net.ep[0].range[0] = 0.f;
	net.ep[0].range[1] = 1.f;
	net.ep[1].ID = 0;
	net.ep[1].rate = net.ep[0].rate;
	net.ep[1].range[0] = 0.f;
	net.ep[1].range[1] = 1.f;
	net.ep[2].ID = 0;
	net.ep[2].rate = net.ep[0].rate;
	net.ep[2].range[0] = 0.f;
	net.ep[2].range[1] = 1.f;
	net.ep[3].ID = 0;
	net.ep[3].rate = net.ep[0].rate;
	net.ep[3].range[0] = 0.f;
	net.ep[3].range[1] = 1.f;
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
	ap.step_const_Sm = M_2_PI_F / 400.f;	/* (rad/step) */
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
	ap.knob_range_ANG[2] = 2.5f;	/* (V) */
	ap.knob_range_ANG[3] = 4.0f;	/* (V) */
#ifdef HW_HAVE_BRAKE_KNOB
	ap.knob_range_BRK[0] = 2.0f;	/* (V) */
	ap.knob_range_BRK[1] = 4.0f;	/* (V) */
#endif /* HW_HAVE_BRAKE_KNOB */
	ap.knob_range_LOS[0] = 0.2f;	/* (V) */
	ap.knob_range_LOS[1] = 4.8f;	/* (V) */
	ap.knob_control_ANG[0] = 0.f;
	ap.knob_control_ANG[1] = 50.f;
	ap.knob_control_ANG[2] = 100.f;
	ap.knob_control_BRK = - 100.f;
#endif /* HW_HAVE_ANALOG_KNOB */

	ap.timeout_DISARM = 1000.f;	/* (ms) */
	ap.timeout_IDLE = 5000.f;	/* (ms) */

#ifdef HW_HAVE_NTC_ON_PCB
	ap.ntc_PCB.type = HW_NTC_PCB_TYPE;
	ap.ntc_PCB.gpio = GPIO_ADC_NTC_PCB;
	ap.ntc_PCB.balance = HW_NTC_PCB_BALANCE;
	ap.ntc_PCB.ntc0 = HW_NTC_PCB_NTC0;
	ap.ntc_PCB.ta0 = HW_NTC_PCB_TA0;
	ap.ntc_PCB.betta = HW_NTC_PCB_BETTA;
#endif /* HW_HAVE_NTC_ON_PCB */

#ifdef HW_HAVE_NTC_MACHINE
	ap.ntc_EXT.type = NTC_ON_GND;
	ap.ntc_EXT.gpio = GPIO_ADC_NTC_EXT;
	ap.ntc_EXT.balance = HW_NTC_EXT_BALANCE;
	ap.ntc_EXT.ntc0 = 10000.f;
	ap.ntc_EXT.ta0 = 25.f;
	ap.ntc_EXT.betta = 3380.f;
#endif /* HW_HAVE_NTC_MACHINE */

	ap.temp_gain_LP = 1.e-1f;

	ap.otp_PCB_halt = 110.f;	/* (C) */
	ap.otp_PCB_derate = 90.f;	/* (C) */
	ap.otp_PCB_fan = 60.f;		/* (C) */
	ap.otp_EXT_derate = 0.f;	/* (C) */
	ap.otp_derate_tol = 10.f;	/* (C) */

	ap.auto_reg_DATA = 0.f;
	ap.auto_reg_ID = ID_PM_S_SETPOINT_SPEED_KNOB;

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

	reg_TOUCH_F(ID_PM_DC_THRESHOLD);

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

	halt_A = m_fabsf(hal.const_ADC.GA * ADC_RESOLUTION / 2.f);
	halt_U = m_fabsf(hal.const_ADC.GU * ADC_RESOLUTION);

	tol_A = (float) (int) (0.02f * halt_A);
	tol_U = (float) (int) (0.02f * halt_U);

	pm.fault_current_tol = (tol_A < 5.f) ? 5.f : tol_A;
	pm.fault_voltage_tol = (tol_U < 5.f) ? 5.f : tol_U;

	pm.fault_current_halt = (float) (int) (0.95f * halt_A);
	pm.fault_voltage_halt = (float) (int) (0.95f * halt_U);

	pm.dcu_deadband = HW_PWM_DEADTIME_NS;

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

	if (unlikely(seed[1] == seed[2])) {

		log_TRACE("RNG fault %8x" EOL, seed[2]);
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
	GPIO_set_mode_OUTPUT(GPIO_FAN_EN);
	GPIO_set_LOW(GPIO_FAN_EN);
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

	xTaskCreate(task_CMDSH, "CMDSH", configHUGE_STACK_SIZE, NULL, 1, NULL);

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

	int		hold_FLAG = PM_DISABLED;

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

			if (timeout_IDLE() == PM_ENABLED) {

				if (pm.lu_MODE != PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				}

				ap.ppm_ACTIVE = PM_DISABLED;
			}
		}
		else if (ap.ppm_DISARM == PM_ENABLED) {

			if (timeout_DISARM() == PM_ENABLED) {

				ap.ppm_DISARM = PM_DISABLED;
			}
		}
	}
	else {
		hold_FLAG = PM_ENABLED;

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

	if (		ap.ppm_reg_DATA != control
			|| hold_FLAG == PM_ENABLED) {

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

		ap.ppm_NFAULT = 0;
	}
	else {
		if (		ap.ppm_ACTIVE == PM_ENABLED
				&& pm.lu_MODE != PM_LU_DISABLED) {

			ap.ppm_NFAULT++;

			if (unlikely(ap.ppm_NFAULT >= 10)) {

				pm.fsm_errno = PM_ERROR_KNOB_SIGNAL_FAULT;
				pm.fsm_req = PM_STATE_LU_SHUTDOWN;

				ap.ppm_ACTIVE = PM_DISABLED;
			}
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

		xSP = ap.step_POS * ap.step_const_Sm;

		if (ap.step_reg_DATA != xSP) {

			ap.step_reg_DATA = xSP;

			if (ap.step_reg_ID != ID_NULL) {

				reg_SET_F(ap.step_reg_ID, ap.step_reg_DATA);
			}
		}

		if (ap.step_STARTUP == PM_ENABLED) {

			if (		pm.lu_MODE == PM_LU_DISABLED
					&& pm.fsm_errno == PM_OK) {

				ap.step_POS = 0;

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

#if (HW_ADC_SAMPLING_SEQUENCE == ADC_SEQUENCE__ABC_UTT_TSC)
	fb.analog_SIN = hal.ADC_analog_SIN;
	fb.analog_COS = hal.ADC_analog_COS;
#endif /* ADC_SEQUENCE__ABC_UTT_TSC */

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

	if (pm.lu_MODE != PM_LU_DISABLED) {

		if (unlikely(hal.CNT_diag[1] >= pm.m_dT)) {

			pm.fsm_errno = PM_ERROR_HW_UNMANAGED_IRQ;
			pm.fsm_req = PM_STATE_HALT;
		}

		if (unlikely(PWM_fault() != HAL_OK)) {

			pm.fsm_errno = PM_ERROR_HW_EMERGENCY_STOP;
			pm.fsm_req = PM_STATE_HALT;
		}

#ifdef HW_HAVE_DRV_ON_PCB
		if (unlikely(DRV_fault() != HAL_OK)) {

			pm.fsm_errno = PM_ERROR_HW_OVERCURRENT;
			pm.fsm_req = PM_STATE_HALT;
		}
#endif /* HW_HAVE_DRV_ON_PCB */
	}

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
					(void *) reg->link, AP_TASK_PRIORITY, NULL);

			vTaskDelay((TickType_t) 10);
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
	uint32_t	ld_sizeof, ld_crc32;
	int		rc;

	printf("Hardware \"%s\"" EOL, fw.hardware);
	printf("Revision \"%s\"" EOL, fw.revision);
	printf("Build \"%s\"" EOL, fw.build);

	ld_sizeof = fw.ld_crc32 - fw.ld_begin;
	ld_crc32 = * (uint32_t *) fw.ld_crc32;

	rc = (crc32u((const void *) fw.ld_begin, ld_sizeof) == ld_crc32) ? 1 : 0;

	printf("CRC32 %8x (%s)" EOL, ld_crc32, (rc != 0) ? "OK" : "FAULT");
}

SH_DEF(ap_gettick)
{
	printf("TN %i %i" EOL, log.boot_COUNT, xTaskGetTickCount());
}

SH_DEF(ap_dbg_task)
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

SH_DEF(ap_dbg_heap)
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

SH_DEF(ap_dbg_hexdump)
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

SH_DEF(ap_log_flush)
{
	if (log_status() != HAL_OK) {

		log_flush();
	}
}

SH_DEF(ap_log_clean)
{
	if (log_status() != HAL_OK) {

		log_clean();
	}
}

SH_DEF(ap_reboot)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	vTaskDelay((TickType_t) 10);

	app_halt();
	hal_system_reset();
}

SH_DEF(ap_bootload)
{
	if (pm.lu_MODE != PM_LU_DISABLED) {

		printf("Unable when PM is running" EOL);
		return ;
	}

	vTaskDelay((TickType_t) 10);

	app_halt();
	hal_bootload_reset();
}

