/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#include "hal/hal.h"

#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "teli.h"

#define REG_DEF_E(link, extra, unit, fmt, mode, proc)	{ #link extra "\0" unit, fmt, mode, \
								(void *) &link, (void *) proc }

#define REG_DEF(link, unit, fmt, mode, proc)	REG_DEF_E(link, "", unit, fmt, mode, proc)
#define REG_MAX					(sizeof(regfile) / sizeof(reg_t) - 1UL)

static int		null;

static void
reg_proc_pwm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f;
	}

	if (rval != NULL) {

		reg->link->f = *rval;
		hal_fence();

		ADC_irq_lock();
		PWM_set_configuration();

		pm.freq_hz = hal.PWM_frequency;
		pm.dT = 1.f / pm.freq_hz;
		pm.pwm_resolution = hal.PWM_resolution;
		pm.pwm_compensation = hal.PWM_deadtime_tik;

		ADC_irq_unlock();
	}
}

static void
reg_proc_ppm(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}

	if (rval != NULL) {

		reg->link->i = *rval;
		hal_fence();

		PPM_set_configuration();
	}
}

static void
reg_proc_tik(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = (float) (reg->link->i) * 1000000000.f
			/ (hal.PWM_frequency * (float) hal.PWM_resolution);
	}

	if (rval != NULL) {

		reg->link->i = (int) ((float) (*rval) * hal.PWM_frequency
				* (float) hal.PWM_resolution / 1000000000.f + .5f);
	}
}

static void
reg_proc_fsm_state(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = reg->link->i;
	}

	if (rval != NULL) {

		pm_fsm_req(&pm, *rval);
	}
}

static void
reg_proc_rpm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = reg->link->f * 9.5492969f / pm.const_Zp;
	}

	if (rval != NULL) {

		reg->link->f = (*rval) * 0.10471976f * pm.const_Zp;
	}
}

static void
reg_proc_rpm_pc(const reg_t *reg, float *lval, const float *rval)
{
	const float		kpc = .57735027f / 100.f;

	if (lval != NULL) {

		*lval = reg->link->f * pm.const_E / (kpc * pm.const_lpf_U);
	}

	if (rval != NULL) {

		reg->link->f = (*rval) * kpc * pm.const_lpf_U / pm.const_E;
	}
}

static void
reg_proc_kv(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = 5.513289f / (reg->link->f * pm.const_Zp);
	}

	if (rval != NULL) {

		reg->link->f = 5.513289f / ((*rval) * pm.const_Zp);
	}
}

const reg_t		regfile[] = {

	REG_DEF(null,					"",	"%i",	REG_READ_ONLY, NULL),

	REG_DEF(hal.HSE_crystal_clock,			"Hz",	"%i",	REG_READ_ONLY, NULL),
	REG_DEF(hal.USART_baud_rate,			"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(hal.PWM_frequency,			"Hz",	"%1f",	REG_CONFIG, &reg_proc_pwm),
	REG_DEF(hal.PWM_deadtime,			"ns",	"%1f",	REG_CONFIG, &reg_proc_pwm),
	REG_DEF(hal.ADC_reference_voltage,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_current_shunt_resistance,	"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_amplifier_gain,			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_voltage_divider_gain,		"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(hal.PPM_mode,				"",	"%i",	REG_CONFIG, &reg_proc_ppm),
	REG_DEF(hal.PPM_timebase,			"Hz",	"%i",	REG_CONFIG, NULL),
	REG_DEF(hal.PPM_signal_caught,			"",	"%i",	REG_READ_ONLY, NULL),

	REG_DEF(ap.ppm_reg_ID,				"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ap.ppm_pulse_range[0],			"us",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(ap.ppm_pulse_range[1],			"us",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(ap.ppm_control_range[0],		"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(ap.ppm_control_range[1],		"",	"%4e",	REG_CONFIG, NULL),

	REG_DEF(ap.ntc_PCB.r_balance,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.betta,			"",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.r_balance,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.betta,			"",	"%1f",	REG_CONFIG, NULL),

	REG_DEF(ap.temp_PCB,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG_DEF(ap.temp_EXT,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG_DEF(ap.temp_INT,				"C",	"%1f",	REG_READ_ONLY, NULL),

	REG_DEF(ap.temp_PCB_overheat,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.temp_superheat,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.temp_current_PCB_derated,		"A",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(ap.batt_voltage_low,			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(ap.batt_voltage_high,			"V",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(pm.pwm_resolution,			"",	"%i",	REG_READ_ONLY, NULL),
	REG_DEF(pm.pwm_compensation,			"ns",	"%1f",	0, &reg_proc_tik),
	REG_DEF(pm.pwm_minimal_pulse,			"ns",	"%1f",	REG_CONFIG, &reg_proc_tik),
	REG_DEF(pm.pwm_silence_gap,			"ns",	"%1f",	REG_CONFIG, &reg_proc_tik),

	REG_DEF(pm.fail_reason,				"",	"%i",	REG_READ_ONLY, NULL),
	REG_DEF(pm.config_ABC,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_LDQ,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_HALL,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_HFI,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_LOOP,				"",	"%i",	REG_CONFIG, NULL),

	REG_DEF(pm.fsm_state,				"",	"%i",	0, &reg_proc_fsm_state),
	REG_DEF(pm.fsm_phase,				"",	"%i",	0, NULL),

	REG_DEF(pm.tm_transient_skip, 			"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_voltage_hold, 			"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_current_hold, 			"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_instant_probe, 			"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_average_probe, 			"s",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(pm.adjust_IA[0],			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_IA[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_IB[0],			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_IB[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_US[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_US[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UA[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UA[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UB[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UB[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UC[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.adjust_UC[1],			"",	"%4e",	REG_CONFIG, NULL),

	REG_DEF(pm.fb_current_clamp,			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fb_current_A,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.fb_current_B,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.fb_voltage_A,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.fb_voltage_B,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.fb_voltage_C,			"V",	"%3f",	REG_READ_ONLY, NULL),

	REG_DEF(pm.probe_current_hold,			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_current_hold_Q,		"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_current_sine,			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_freq_sine_hz,			"Hz",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_speed_low,		"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_DEF_E(pm.probe_speed_low, "_rpm",		"rpm",	"%2f",	0, &reg_proc_rpm),
	REG_DEF(pm.probe_speed_ramp,		"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_DEF_E(pm.probe_speed_ramp, "_rpm",		"rpm",	"%2f",	0, &reg_proc_rpm),
	REG_DEF(pm.probe_gain_P,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_gain_I,			"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.fault_voltage_tolerance,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_current_tolerance,		"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_current_halt_level,		"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_adjust_tolerance,		"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_flux_residual_maximal,		"A",	"%2f",	REG_CONFIG, NULL),

	REG_DEF(pm.vsi_clamp_to_null,			"",	"%i",	0, NULL),
	REG_DEF(pm.vsi_lpf_D,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.vsi_lpf_Q,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.vsi_lpf_watt,			"W",	"%1f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.vsi_gain_LP,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.vsi_gain_LW,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.lu_fb_X,				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_fb_Y,				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_X[0],				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_X[1],				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_X[2],				"",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_X[3],				"",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.lu_X[4],			"rad/s",	"%2f",	REG_READ_ONLY, NULL),
	REG_DEF_E(pm.lu_X[4], "_rpm",			"rpm",	"%2f",	REG_READ_ONLY, &reg_proc_rpm),
	REG_DEF(pm.lu_mode,				"",	"%i",	REG_READ_ONLY, NULL),

	REG_DEF(pm.forced_hold_D,			"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.forced_accel,		"rad/s/s",	"%3e",	REG_CONFIG, NULL),
	REG_DEF_E(pm.forced_accel, "_rpm",	"rpm/s",	"%3e",	0, &reg_proc_rpm),
	REG_DEF(pm.forced_setpoint,		"rad/s",	"%2f",	0, NULL),
	REG_DEF_E(pm.forced_setpoint, "_rpm",		"rpm",	"%2f",	0, &reg_proc_rpm),

	REG_DEF(pm.flux_drift_Q,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.flux_residual_D,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.flux_residual_Q,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.flux_residual_lpf,			"",	"%2f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.flux_gain_LP,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_DA,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_QA,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_DP,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_DS,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_QS,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_gain_QZ,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_bemf_low_unlock,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_bemf_low_lock,			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_bemf_high,			"V",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(pm.hfi_freq_hz,				"Hz",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_swing_D,				"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_flux_polarity,			"",	"%4e",	REG_READ_ONLY, NULL),
	REG_DEF(pm.hfi_gain_P,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_gain_S,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_gain_F,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.const_lpf_U,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.const_gain_LP,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_E,				"Wb",	"%4e",	REG_CONFIG, NULL),
	REG_DEF_E(pm.const_E, "_kv",		"rpm/v",	"%1f",	0, &reg_proc_kv),
	REG_DEF(pm.const_R,				"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Ld,				"H",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Lq,				"H",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Zp,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.const_J,			"kg*m*m",	"%4e",	REG_CONFIG, NULL),

	REG_DEF(pm.i_maximal,				"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_watt_consumption_maximal,		"W",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_watt_regeneration_maximal,		"W",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_setpoint_D,			"A",	"%3f",	0, NULL),
	REG_DEF(pm.i_setpoint_Q,			"A",	"%3f",	0, NULL),
	REG_DEF(pm.i_gain_PD,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_ID,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_PQ,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_IQ,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.s_maximal,			"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_DEF_E(pm.s_maximal, "_rpm",			"rpm",	"%2f",	0, &reg_proc_rpm),
	REG_DEF(pm.s_setpoint,			"rad/s",	"%2f",	0, NULL),
	REG_DEF_E(pm.s_setpoint, "_rpm",		"rpm",	"%2f",	0, &reg_proc_rpm),
	REG_DEF_E(pm.s_setpoint, "_pc",			"%",	"%2f",	0, &reg_proc_rpm_pc),
	REG_DEF(pm.s_accel,			"rad/s/s",	"%3e",	REG_CONFIG, NULL),
	REG_DEF_E(pm.s_accel, "_rpm" ,		"rpm/s",	"%3e",	0, &reg_proc_rpm),
	REG_DEF(pm.s_gain_P,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.s_gain_I,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(ti.reg_ID[0],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[1],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[2],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[3],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[4],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[5],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[6],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[7],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[8],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),
	REG_DEF(ti.reg_ID[9],			"",	"%i",	REG_CONFIG | REG_LINKED, NULL),

	{ NULL, NULL, 0, NULL, NULL }
};

void reg_getval(const reg_t *reg, void *lval)
{
	if (reg->proc != NULL) {

		reg->proc(reg, lval, NULL);
	}
	else {
		*(reg_val_t *) lval = *reg->link;
	}
}

void reg_setval(const reg_t *reg, const void *rval)
{
	if ((reg->mode & REG_READ_ONLY) == 0) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			*reg->link = *(reg_val_t *) rval;
		}
	}
}

void reg_format_rval(const reg_t *reg, const void *rval)
{
	reg_val_t		*link = (reg_val_t *) rval;

	if (reg->fmt[1] == 'i') {

		printf(reg->fmt, link->i);
	}
	else {
		printf(reg->fmt, &link->f);
	}
}

void reg_format(const reg_t *reg)
{
	reg_val_t		rval;
	const char		*su;

	if (reg != NULL) {

		printf("%c%c%c [%i] %s = ",
			(int) (reg->mode & REG_CONFIG) 		? 'C' : ' ',
			(int) (reg->mode & REG_READ_ONLY)	? 'R' : ' ',
			(int) (reg->mode & REG_LINKED)		? 'L' : ' ',
			(int) (reg - regfile), reg->sym);

		reg_getval(reg, &rval);
		reg_format_rval(reg, &rval);

		if (reg->mode & REG_LINKED) {

			if (rval.i >= 0 && rval.i < REG_MAX) {

				printf(" (%s)", regfile[rval.i].sym);
			}
		}

		su = reg->sym + strlen(reg->sym) + 1;

		if (*su != 0) {

			printf(" (%s)", su);
		}

		puts(EOL);
	}
}

const reg_t *reg_search(const char *sym)
{
	const reg_t		*reg, *found = NULL;
	int			n;

	if (stoi(&n, sym) != NULL) {

		if (n >= 0 && n < REG_MAX)
			found = regfile + n;
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strstr(reg->sym, sym) != NULL) {

				if (found == NULL) {

					found = reg;
				}
				else {
					found = NULL;
					break;
				}
			}
		}
	}

	return found;
}

void reg_GET(int n, void *lval)
{
	if (n >= 0 && n < REG_MAX) {

		reg_getval(regfile + n, lval);
	}
}

void reg_SET(int n, const void *rval)
{
	if (n >= 0 && n < REG_MAX) {

		reg_setval(regfile + n, rval);
	}
}

void reg_SET_F(int n, float rval)
{
	reg_SET(n, &rval);
}

SH_DEF(reg)
{
	reg_val_t		rval;
	const reg_t		*reg, *lreg;

	reg = reg_search(s);

	if (reg != NULL) {

		s = sh_next_arg(s);

		if (reg->fmt[1] == 'i') {

			if (reg->mode & REG_LINKED) {

				lreg = reg_search(s);

				if (lreg != NULL) {

					rval.i = (int) (lreg - regfile);
					reg_setval(reg, &rval);
				}
			}
			else if (stoi(&rval.i, s) != NULL) {

				reg_setval(reg, &rval);
			}
		}
		else {
			if (stof(&rval.f, s) != NULL) {

				reg_setval(reg, &rval);
			}
		}

		reg_format(reg);
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strstr(reg->sym, s) != NULL) {

				reg_format(reg);
			}
		}
	}
}

static void
reg_print_help(int n)
{
	switch (n) {

		case ID_PM_CONFIG_ABC:
			printf("%i = Three Phase" EOL, PM_ABC_THREE_PHASE);
			printf("%i = Two Phase" EOL, PM_ABC_TWO_PHASE);
			break;

		case ID_PM_CONFIG_LDQ:
			printf("%i = Non Salient Pole (Ld = Lq)" EOL, PM_LDQ_NON_SALIENT_POLE);
			printf("%i = Salient Pole (Ld > Lq)" EOL, PM_LDQ_SALIENT_POLE);
			printf("%i = Saturation Saliency (Ld < Lq)" EOL, PM_LDQ_SATURATION_SALIENCY);
			break;

		case ID_PM_CONFIG_HFI:
			printf("%i = Diabled" EOL, PM_HFI_DISABLED);
			printf("%i = Enabled" EOL, PM_HFI_ENABLED);
			break;

		case ID_PM_CONFIG_LOOP:
			printf("%i = Current Control" EOL, PM_LOOP_CURRENT_CONTROL);
			printf("%i = Speed Control" EOL, PM_LOOP_SPEED_CONTROL);
			break;

		case ID_PM_FSM_STATE:
			printf("%i = IDLE" EOL, PM_STATE_IDLE);
			printf("%i = Zero Drift" EOL, PM_STATE_ZERO_DRIFT);
			printf("%i = Power Stage Self Test" EOL, PM_STATE_POWER_STAGE_SELF_TEST);
			printf("%i = Adjust Voltage" EOL, PM_STATE_ADJUST_VOLTAGE);
			printf("%i = Adjust Current" EOL, PM_STATE_ADJUST_CURRENT);
			printf("%i = Probe Const R" EOL, PM_STATE_PROBE_CONST_R);
			printf("%i = Probe Const L" EOL, PM_STATE_PROBE_CONST_L);
			printf("%i = LU Initiate" EOL, PM_STATE_LU_INITIATE);
			printf("%i = LU Shutdown" EOL, PM_STATE_LU_SHUTDOWN);
			printf("%i = Probe Const E" EOL, PM_STATE_PROBE_CONST_E);
			printf("%i = HALT" EOL, PM_STATE_HALT);
			break;

		default:
			break;
	}
}

SH_DEF(reg_help)
{
	const reg_t		*reg = NULL;
	int			n;

	if (stoi(&n, s) != NULL) {

		if (n >= 0 && n < REG_MAX)
			reg = regfile + n;
	}
	else {
		reg = reg_search(s);
	}

	if (reg != NULL) {

		reg_print_help((int) (reg - regfile));
	}
}

SH_DEF(reg_export)
{
	reg_val_t		rval;
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode == REG_CONFIG) {

			printf("reg %s ", reg->sym);

			reg_getval(reg, &rval);

			if (reg->mode & REG_LINKED) {

				if (rval.i >= 0 && rval.i < REG_MAX) {

					puts(regfile[rval.i].sym);
				}
			}
			else {
				reg_format_rval(reg, &rval);
			}

			puts(EOL);
		}
	}
}

