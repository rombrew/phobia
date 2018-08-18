/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include "lib.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "teli.h"

#define REG_DEF_E(link, extra, unit, fmt, mode, proc)	{ #link extra "\0" unit, fmt, mode, \
							{ (void *) &link }, (void *) proc }

#define REG_DEF(link, unit, fmt, mode, proc)	REG_DEF_E(link, "", unit, fmt, mode, proc)
#define REG_MAX					(sizeof(regfile) / sizeof(reg_t) - 1UL)

static void
reg_proc_fsm_state(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = (*reg->link.i);
	}

	if (rval != NULL) {

		pm_fsm_req(&pm, (*rval));
	}
}

static void
reg_proc_rpm(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = (*reg->link.f) * 9.5492969f / pm.const_Zp;
	}

	if (rval != NULL) {

		*reg->link.f = (*rval) * 0.10471976f * pm.const_Zp;
	}
}

static void
reg_proc_kv(const reg_t *reg, float *lval, const float *rval)
{
	if (lval != NULL) {

		*lval = 5.513289f / ((*reg->link.f) * pm.const_Zp);
	}

	if (rval != NULL) {

		*reg->link.f = 5.513289f / ((*rval) * pm.const_Zp);
	}
}

const reg_t		regfile[] = {

	REG_DEF(hal.USART_baud_rate,			"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(hal.PWM_freq_hz,			"Hz",	"%i",	REG_CONFIG, NULL),
	REG_DEF(hal.PWM_dead_time_ns,			"ns",	"%i",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_reference_voltage,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_current_shunt_resistance,	"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_amplifier_gain,			"",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(hal.ADC_voltage_divider_gain,		"",	"%4e",	REG_CONFIG, NULL),

	REG_DEF(ap.ntc_PCB.r_balance,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_PCB.betta,			"",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.r_balance,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(ap.ntc_EXT.betta,			"",	"%1f",	REG_CONFIG, NULL),

	REG_DEF(ap.tc_PCB,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG_DEF(ap.tc_EXT,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG_DEF(ap.tc_TEMP,				"C",	"%1f",	REG_READ_ONLY, NULL),

	REG_DEF(pm.pwm_R,				"",	"%i",	REG_READ_ONLY, NULL),
	REG_DEF(pm.pwm_MP,				"",	"%i",	REG_CONFIG, NULL),

	REG_DEF(pm.err_reason,				"",	"%i",	REG_READ_ONLY, NULL),
	REG_DEF(pm.config_ABC,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_LDQ,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_HALL,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_HFI,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.config_LOOP,				"",	"%i",	REG_CONFIG, NULL),

	REG_DEF(pm.fsm_state,				"",	"%i",	REG_VIRTUAL, &reg_proc_fsm_state),

	REG_DEF(pm.tm_skip, 				"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_probe, 				"s",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.tm_hold, 				"s",	"%3f",	REG_CONFIG, NULL),

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
	REG_DEF_E(pm.probe_speed_low, "_rpm",		"rpm",	"%2f",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.probe_speed_ramp,		"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_DEF_E(pm.probe_speed_ramp, "_rpm",		"rpm",	"%2f",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.probe_gain_P,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.probe_gain_I,			"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.fault_zero_drift_maximal,		"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_voltage_tolerance,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_current_tolerance,		"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_adjust_tolerance,		"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_flux_residual_maximal,		"A",	"%2f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_supply_voltage_low,		"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.fault_supply_voltage_high,		"V",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(pm.vsi_clamp_to_null,			"",	"%i",	REG_VIRTUAL, NULL),
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
	REG_DEF_E(pm.forced_accel, "_rpm",	"rpm/s",	"%3e",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.forced_setpoint,		"rad/s",	"%2f",	REG_VIRTUAL, NULL),
	REG_DEF_E(pm.forced_setpoint, "_rpm",		"rpm",	"%2f",	REG_VIRTUAL, &reg_proc_rpm),

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
	REG_DEF(pm.flux_BEMF_low,			"V",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.flux_BEMF_high,			"V",	"%3f",	REG_CONFIG, NULL),

	REG_DEF(pm.hfi_freq_hz,				"Hz",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_swing_D,				"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_flux_polarity,			"",	"%4e",	REG_READ_ONLY, NULL),
	REG_DEF(pm.hfi_gain_P,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_gain_S,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.hfi_gain_F,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.const_lpf_U,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG_DEF(pm.const_gain_LP,			"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_E,				"Wb",	"%4e",	REG_CONFIG, NULL),
	REG_DEF_E(pm.const_E, "_kv",		"rpm/v",	"%1f",	REG_VIRTUAL, &reg_proc_kv),
	REG_DEF(pm.const_R,				"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Ld,				"H",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Lq,				"H",	"%4e",	REG_CONFIG, NULL),
	REG_DEF(pm.const_Zp,				"",	"%i",	REG_CONFIG, NULL),
	REG_DEF(pm.const_J,			"kg*m*m",	"%4e",	REG_CONFIG, NULL),

	REG_DEF(pm.i_maximal,				"A",	"%3f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_watt_consumption_maximal,		"W",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_watt_regeneration_maximal,		"W",	"%1f",	REG_CONFIG, NULL),
	REG_DEF(pm.i_setpoint_D,			"A",	"%3f",	REG_VIRTUAL, NULL),
	REG_DEF(pm.i_setpoint_Q,			"A",	"%3f",	REG_VIRTUAL, NULL),
	REG_DEF(pm.i_slew_rate_D,			"A/s",	"%3e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_slew_rate_Q,			"A/s",	"%3e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_PD,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_ID,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_PQ,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.i_gain_IQ,				"",	"%2e",	REG_CONFIG, NULL),

	REG_DEF(pm.s_maximal,			"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_DEF_E(pm.s_maximal, "_rpm",			"rpm",	"%2f",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.s_setpoint,			"rad/s",	"%2f",	REG_VIRTUAL, NULL),
	REG_DEF_E(pm.s_setpoint, "_rpm",		"rpm",	"%2f",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.s_accel,			"rad/s/s",	"%3e",	REG_CONFIG, NULL),
	REG_DEF_E(pm.s_accel, "_rpm" ,		"rpm/s",	"%3e",	REG_VIRTUAL, &reg_proc_rpm),
	REG_DEF(pm.s_gain_P,				"",	"%2e",	REG_CONFIG, NULL),
	REG_DEF(pm.s_gain_I,				"",	"%2e",	REG_CONFIG, NULL),

	{ NULL, NULL, 0, { NULL }, NULL }
};

void reg_getval(const reg_t *reg, void *lval)
{
	if (reg->proc != NULL) {

		reg->proc(reg, lval, NULL);
	}
	else {
		* (unsigned long *) lval = * (unsigned long *) reg->link.i;
	}
}

void reg_setval(const reg_t *reg, const void *rval)
{
	if (reg->mode != REG_READ_ONLY) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			* (unsigned long *) reg->link.i = * (unsigned long *) rval;
		}
	}
}

void reg_print_fmt(const reg_t *reg, int full)
{
	union {
		float		f;
		int		i;
	} u;

	const char		*su;

	if (reg != NULL) {

		if (full != 0) {

			printf("%c [%i] %s = ", (int) "NCR " [reg->mode],
					(int) (reg - regfile), reg->sym);
		}

		reg_getval(reg, &u);

		if (reg->fmt[1] == 'i') {

			printf(reg->fmt, u.i);
		}
		else {
			printf(reg->fmt, &u.f);
		}

		if (full != 0) {

			su = reg->sym + strlen(reg->sym) + 1;

			if (*su != 0) {

				printf(" (%s)", su);
			}
		}

		puts(EOL);
	}
}

static const reg_t *
reg_search(const char *sym)
{
	const reg_t		*reg, *found = NULL;

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

SH_DEF(reg)
{
	union {
		float		f;
		int		i;
	} u;

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

		s = sh_next_arg(s);

		if (reg->fmt[1] == 'i') {

			if (stoi(&u.i, s) != NULL) {

				reg_setval(reg, &u);
			}
		}
		else {
			if (stof(&u.f, s) != NULL) {

				reg_setval(reg, &u);
			}
		}

		reg_print_fmt(reg, 1);
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strstr(reg->sym, s) != NULL) {

				reg_print_fmt(reg, 1);
			}
		}
	}
}

SH_DEF(reg_export)
{
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode == REG_CONFIG) {

			printf("reg %s ", reg->sym);

			reg_print_fmt(reg, 0);
		}
	}
}

