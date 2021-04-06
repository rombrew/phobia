#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tlm.h"

typedef struct {

	/* PPM interface (PWM).
	 * */
	int			ppm_reg_ID;
	int			ppm_STARTUP;
	float			ppm_in_cached;
	float			ppm_in_range[3];
	float			ppm_control_range[3];

	/* STEP/DIR interface.
	 * */
	int			step_reg_ID;
	int			step_STARTUP;
	int			step_baseEP;
	int			step_accuEP;
	float			step_const_ld_EP;

	/* Analog interface.
	 * */
	float			analog_const_GU;
	int			analog_ENABLED;
	int			analog_reg_ID;
	int			analog_STARTUP;
	float			analog_in_ANG[3];
	float			analog_in_BRK[2];
	float			analog_in_lost[2];
	float			analog_control_ANG[3];
	float			analog_control_BRK;

	/* Timeout control.
	 * */
	int			timeout_TIME;
	int			timeout_revol_cached;
	float			timeout_current_tol;
	float			timeout_IDLE_s;

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
	float			heat_PCB_halt;
	float			heat_PCB_on_FAN;
	float			heat_PCB_derated;
	float			heat_EXT_halt;
	float			heat_EXT_derated;
	float			heat_recovery_gap;

	/* Servo drive.
	 * */
	float			servo_SPAN_mm[2];
	float			servo_UNIFORM_mmps;

	/* HX711 (load cell amplifier).
	 * */
	float			hx711_kg;
	float			hx711_scale[2];
}
app_main_t;

extern app_main_t		ap;
extern pmc_t			pm;
extern TLM_t			tlm;

extern int flash_block_regs_load();
extern int flash_block_relocate(u32_t len);
extern int pm_wait_for_IDLE();

float ADC_get_analog_ANG();
float ADC_get_analog_BRK();

#endif /* _H_MAIN_ */

