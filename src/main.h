#ifndef _H_MAIN_
#define _H_MAIN_

#include "phobia/pm.h"
#include "phobia/libm.h"

#include "libc.h"
#include "ntc.h"
#include "tlm.h"

typedef struct {

	/* CPU load (internals).
	 * */
	int			lc_flag;
	int			lc_tick;
	int			lc_idle;

	/* PPM interface (PWM).
	 * */
	int			ppm_reg_ID;
	int			ppm_STARTUP;
	int			ppm_ACTIVE;
	int			ppm_LOCKED;
	float			ppm_in_range[3];
	float			ppm_control_range[3];

	/* STEP/DIR interface.
	 * */
	int			step_reg_ID;
	int			step_STARTUP;
	int			step_ACTIVE;
	int			step_baseEP;
	int			step_accuEP;
	float			step_const_ld_EP;

	/* Knob analog interface.
	 * */
	int			knob_ENABLED;
	int			knob_reg_ID;
	int			knob_STARTUP;
	int			knob_ACTIVE;
	float			knob_in_ANG[3];
	float			knob_in_BRK[2];
	float			knob_in_lost[2];
	float			knob_control_ANG[3];
	float			knob_control_BRK;

	/* IDLE control.
	 * */
	float			idle_TIME_s;
	int			idle_RESET;
	int			idle_INVOKE;
	int			idle_revol_cached;

	/* SAFETY control.
	 * */
	int			safety_RESET;
	int			safety_INVOKE;

	/* NTC constants.
	 * */
	ntc_t			ntc_PCB;
	ntc_t			ntc_EXT;

	/* Thermal info.
	 * */
	float			temp_PCB;
	float			temp_EXT;
	float			temp_INT;

	/* Thermal protection.
	 * */
	float			tpro_PCB_temp_halt;
	float			tpro_PCB_temp_derate;
	float			tpro_PCB_temp_FAN;
	float			tpro_EXT_temp_derate;
	float			tpro_derated_PCB;
	float			tpro_derated_EXT;
	float			tpro_temp_recovery;

	/* Servo DRIVE.
	 * */
	float			servo_SPAN_mm[2];
	float			servo_UNIFORM_mmps;

	/* Application AUTORUN.
	 * */
	int			autorun_APP[2];

	/* HX711 (load cell ADC).
	 * */
	float			hx711_kg;
	float			hx711_scale[2];

	/* AS5047 (magnetic rotary position sensor).
	 * */
	int			as5047_EP;
}
app_main_t;

typedef struct {

	const char		*name;
	void			(* ptask) (void *);
}
app_task_t;

extern app_main_t		ap;
extern pmc_t			pm;
extern TLM_t			tlm;

extern int flash_block_regs_load();
extern int pm_wait_for_IDLE();

const char *app_name_by_ID(int app_ID);
void app_startup_by_ID(int app_ID);
void app_halt();

float ADC_get_knob_ANG();
float ADC_get_knob_BRK();

#endif /* _H_MAIN_ */

