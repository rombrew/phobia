#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tel.h"

typedef struct {

	/* Serial IO interfaces.
	 * */
	io_ops_t		io_USART;
	io_ops_t		io_CAN;

	/* CAN interface.
	 * */
	int			can_node_ID;

	/* PPM interface.
	 * */
	int			ppm_reg_ID;
	int			ppm_locked;
	float			ppm_pulse_cached;
	float			ppm_pulse_range[3];
	float			ppm_pulse_lost[2];
	float			ppm_control_range[3];
	float			ppm_startup_range[2];

	/* STEP/DIR interface.
	 * */
	int			step_reg_ID;
	int			step_locked;
	int			step_baseEP;
	int			step_accuEP;
	float			step_const_ld_EP;

	/* Analog interface.
	 * */
	int			analog_enabled;
	int			analog_reg_ID;
	int			analog_locked;
	float			analog_voltage_ratio;
	float			analog_timeout;
	float			analog_voltage_ANG[3];
	float			analog_voltage_BRK[3];
	float			analog_voltage_lost[2];
	float			analog_control_ANG[3];
	float			analog_control_BRK[3];
	float			analog_startup_range[2];

	/* CPU load.
	 * */
	int			lc_flag;
	int			lc_tick;
	int			lc_idle;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_INT;

	/* Heat control.
	 * */
	float			heat_PCB;
	float			heat_PCB_derated_1;
	float			heat_EXT;
	float			heat_EXT_derated_1;
	float			heat_PCB_FAN;
	float			heat_recovery_gap;

	/* Servo drive.
	 * */
	float			servo_span_mm[2];
	float			servo_uniform_mmps;
	int			servo_mice_role;

	/* FT constants.
	 * */
	int			FT_grab_hz;

	/* HX711 (load cell amplifier).
	 * */
	float			hx711_kg;
	float			hx711_gain[2];
}
application_t;

extern application_t		ap;
extern pmc_t			pm;
extern tel_t			ti;

extern int flash_block_load();
extern int pm_wait_for_IDLE();

float ADC_get_analog_ANG();
float ADC_get_analog_BRK();

void lowTRACE(const char *fmt, ...);

#endif /* _H_MAIN_ */

