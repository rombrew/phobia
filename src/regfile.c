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
#include "pm/pm.h"

#include "lib.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"
#include "telinfo.h"

#define REG_E(link, extra, unit, fmt, mode, proc)	{ #link extra "\0" unit, fmt, mode, \
							{ (void *) &link }, (void *) proc }

#define REG(link, unit, fmt, mode, proc)	REG_E(link, "", unit, fmt, mode, proc)
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

	REG(hal.USART_baud_rate,		"",	"%i",	REG_CONFIG, NULL),
	REG(hal.PWM_freq_hz,			"Hz",	"%i",	REG_CONFIG, NULL),
	REG(hal.PWM_dead_time_ns,		"ns",	"%i",	REG_CONFIG, NULL),
	REG(hal.ADC_reference_voltage,		"V",	"%3f",	REG_CONFIG, NULL),
	REG(hal.ADC_current_shunt_resistance,	"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG(hal.ADC_amplifier_gain,		"",	"%4e",	REG_CONFIG, NULL),
	REG(hal.ADC_voltage_divider_gain,	"",	"%4e",	REG_CONFIG, NULL),

	REG(ap.ntc_PCB.r_balance,		"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_PCB.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_PCB.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_PCB.betta,			"",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_EXT.r_balance,		"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_EXT.r_ntc_0,			"Ohm",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_EXT.ta_0,			"C",	"%1f",	REG_CONFIG, NULL),
	REG(ap.ntc_EXT.betta,			"",	"%1f",	REG_CONFIG, NULL),
	REG(ap.t_PCB,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG(ap.t_EXT,				"C",	"%1f",	REG_READ_ONLY, NULL),
	REG(ap.t_TEMP,				"C",	"%1f",	REG_READ_ONLY, NULL),

	REG(pm.pwm_R,				"",	"%i",	REG_READ_ONLY, NULL),
	REG(pm.pwm_MP,				"",	"%i",	REG_CONFIG, NULL),
	REG(pm.err_no,				"",	"%i",	REG_READ_ONLY, NULL),
	REG(pm.b_FORCED,			"",	"%i",	REG_CONFIG, NULL),
	REG(pm.b_HFI,				"",	"%i",	REG_CONFIG, NULL),
	REG(pm.b_SENSOR,			"",	"%i",	REG_CONFIG, NULL),
	REG(pm.b_LOOP,				"",	"%i",	REG_CONFIG, NULL),
	REG(pm.fsm_state,			"",	"%i",	REG_NORMAL, &reg_proc_fsm_state),
	REG(pm.tm_skip,				"s",	"%3f",	REG_CONFIG, NULL),
	REG(pm.tm_probe,			"s",	"%3f",	REG_CONFIG, NULL),
	REG(pm.tm_hold,				"s",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_IA[0],			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_IA[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.adjust_IB[0],			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_IB[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.adjust_US[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_US[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.adjust_UA[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_UA[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.adjust_UB[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_UB[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.adjust_UC[0],			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.adjust_UC[1],			"",	"%4e",	REG_CONFIG, NULL),
	REG(pm.fb_i_range,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fb_iA,				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.fb_iB,				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.fb_uA,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.fb_uB,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.fb_uC,				"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.probe_i_hold,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.probe_i_hold_Q,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.probe_i_sine,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.probe_freq_sine_hz,		"Hz",	"%1f",	REG_CONFIG, NULL),
	REG(pm.probe_speed_ramp,	"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_E(pm.probe_speed_ramp, "_rpm",	"rpm",	"%2f",	REG_NORMAL, &reg_proc_rpm),
	REG(pm.probe_gain_P,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.probe_gain_I,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.fault_zero_drift_maximal,	"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fault_voltage_tolerance,		"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fault_current_tolerance,		"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fault_adjust_tolerance,		"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.fault_lu_residual_maximal,	"A",	"%2f",	REG_CONFIG, NULL),
	REG(pm.fault_lu_drift_Q_maximal,	"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fault_supply_voltage_low,	"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.fault_supply_voltage_high,	"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.vsi_lpf_D,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.vsi_lpf_Q,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_power_lpf,			"W",	"%1f",	REG_READ_ONLY, NULL),
	REG(pm.lu_residual_D,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_residual_Q,			"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_residual_lpf,			"",	"%2f",	REG_READ_ONLY, NULL),
	REG(pm.lu_region,			"",	"%i",	REG_READ_ONLY, NULL),
	REG(pm.lu_X[0],				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_X[1],				"A",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_X[2],				"",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_X[3],				"",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_X[4],			"rad/s",	"%2f",	REG_READ_ONLY, NULL),
	REG_E(pm.lu_X[4], "_rpm",	"rpm",		"%2f",	REG_READ_ONLY, &reg_proc_rpm),
	REG(pm.lu_drift_Q,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.lu_revol,			"",	"%i",	REG_READ_ONLY, NULL),
	REG(pm.lu_gain_DA,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_gain_QA,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_gain_DP,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_gain_DS,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_gain_QS,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_gain_QZ,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lu_BEMF_low,			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.lu_BEMF_high,			"V",	"%3f",	REG_CONFIG, NULL),
	REG(pm.lu_forced_D,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.lu_forced_accel,		"rad/s/s",	"%3e",	REG_CONFIG, NULL),
	REG_E(pm.lu_forced_accel, "_rpm","rpm/s",	"%3e",	REG_NORMAL, &reg_proc_rpm),
	REG(pm.hf_freq_hz,			"Hz",	"%1f",	REG_CONFIG, NULL),
	REG(pm.hf_swing_D,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.hf_flux_polarity,		"",	"%4e",	REG_READ_ONLY, NULL),
	REG(pm.hf_gain_P,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.hf_gain_S,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.hf_gain_F,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.const_lpf_U,			"V",	"%3f",	REG_READ_ONLY, NULL),
	REG(pm.const_E,				"Wb",	"%4e",	REG_CONFIG, NULL),
	REG_E(pm.const_E, "_kv",	"rpm/v",	"%1f",	REG_NORMAL, &reg_proc_kv),
	REG(pm.const_R,				"Ohm",	"%4e",	REG_CONFIG, NULL),
	REG(pm.const_Ld,			"H",	"%4e",	REG_CONFIG, NULL),
	REG(pm.const_Lq,			"H",	"%4e",	REG_CONFIG, NULL),
	REG(pm.const_Zp,			"",	"%i",	REG_CONFIG, NULL),
	REG(pm.const_J,			"kg*m*m",	"%4e",	REG_CONFIG, NULL),
	REG(pm.i_maximal,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.i_maximal_weak,			"A",	"%3f",	REG_CONFIG, NULL),
	REG(pm.i_power_consumption_maximal,	"W",	"%1f",	REG_CONFIG, NULL),
	REG(pm.i_power_regeneration_maximal,	"W",	"%1f",	REG_CONFIG, NULL),
	REG(pm.i_set_point_D,			"A",	"%3f",	REG_NORMAL, NULL),
	REG(pm.i_set_point_Q,			"A",	"%3f",	REG_NORMAL, NULL),
	REG(pm.i_slew_rate_D,			"A/s",	"%3e",	REG_CONFIG, NULL),
	REG(pm.i_slew_rate_Q,			"A/s",	"%3e",	REG_CONFIG, NULL),
	REG(pm.i_gain_PD,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.i_gain_ID,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.i_gain_PQ,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.i_gain_IQ,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.s_maximal,		"rad/s",	"%2f",	REG_CONFIG, NULL),
	REG_E(pm.s_maximal, "_rpm",	"rpm",		"%2f",	REG_NORMAL, &reg_proc_rpm),
	REG(pm.s_set_point,		"rad/s",	"%2f",	REG_NORMAL, NULL),
	REG_E(pm.s_set_point, "_rpm",	"rpm",		"%2f",	REG_NORMAL, &reg_proc_rpm),
	REG(pm.s_slew_rate,		"rad/s/s",	"%3e",	REG_CONFIG, NULL),
	REG_E(pm.s_slew_rate, "_rpm",	"rpm/s",	"%3e",	REG_NORMAL, &reg_proc_rpm),
	REG(pm.s_gain_P,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.s_gain_I,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lpf_gain_POWER,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lpf_gain_LU,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lpf_gain_VSI,			"",	"%2e",	REG_CONFIG, NULL),
	REG(pm.lpf_gain_US,			"",	"%2e",	REG_CONFIG, NULL),

	{ NULL, NULL, 0, { NULL }, NULL }
};

static void
reg_getval(const reg_t *reg, void *lval)
{
	if (reg->proc != NULL) {

		reg->proc((reg_t *) reg, lval, NULL);
	}
	else {
		* (int *) lval = *reg->link.i;
	}
}

static void
reg_setval(const reg_t *reg, const void *rval)
{
	if (reg->mode != REG_READ_ONLY) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			*reg->link.i = * (int *) rval;
		}
	}
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

static void
reg_print_info(const reg_t *reg)
{
	printf("%c [%i] %s = ", (int) "NCR " [reg->mode],
			(int) (reg - regfile), reg->sym);
}

void reg_print_fmt(const reg_t *reg)
{
	const char		*unit;
	float			f;
	int			n;

	if (reg->fmt[1] == 'i') {

		reg_getval(reg, &n);
		printf(reg->fmt, n);
	}
	else {
		reg_getval(reg, &f);
		printf(reg->fmt, &f);
	}

	unit = reg->sym + strlen(reg->sym) + 1;

	if (*unit != 0) {

		printf(" (%s)", unit);
	}

	puts(EOL);
}

SH_DEF(reg_list)
{
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (strstr(reg->sym, s) != NULL) {

			reg_print_info(reg);
			reg_print_fmt(reg);
		}
	}
}

SH_DEF(reg_set)
{
	const reg_t		*reg = NULL;
	float			f;
	int			n;

	if (stoi(&n, s) != NULL) {

		if (n >= 0 && n < REG_MAX)
			reg = regfile + n;
	}
	else {
		for (reg = regfile; reg->sym != NULL; ++reg) {

			if (strcmp(reg->sym, s) == 0)
				break;
		}

		if (reg->sym == NULL)
			reg = NULL;
	}

	if (reg != NULL) {

		s = sh_args(s);

		if (reg->fmt[1] == 'i') {

			if (stoi(&n, s) != NULL) {

				reg_setval(reg, &n);
			}
		}
		else {
			if (stof(&f, s) != NULL) {

				reg_setval(reg, &f);
			}
		}

		reg_print_info(reg);
		reg_print_fmt(reg);
	}
}

SH_DEF(reg_export)
{
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->mode == REG_CONFIG) {

			printf("reg_set %s ", reg->sym);

			reg_print_fmt(reg);
		}
	}
}

