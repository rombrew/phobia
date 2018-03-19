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
#include "hal/pwm.h"
#include "hal/usart.h"

#include "hal_task.h"
#include "lib.h"
#include "pm_control.h"
#include "regfile.h"
#include "shell.h"

#define REG(link, unit, bits, proc)	{ #link "\0" unit, bits, { (void *) &link }, proc }
#define REG_MAX				(sizeof(regfile) / sizeof(reg_t) - 1UL)

static void
reg_proc_default(reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = *reg->link.i;
	}

	if (rval != NULL) {

		*reg->link.i = *rval;
	}
}

const reg_t		regfile[] = {

	REG(hal.usart_baud_rate, "", REG_INT | REG_CONFIG, NULL),
	REG(hal.pwm_freq_hz, "Hz", REG_INT | REG_CONFIG, NULL),
	REG(hal.pwm_dead_time_ns, "ns", REG_INT | REG_CONFIG, NULL),
	REG(ts.av_default_time, "s", REG_CONFIG | REG_F_LONG, NULL),
	REG(pm.b_zero_drift_estimation, "", REG_INT | REG_CONFIG, NULL),

	/*
	(void *) &pm.m_bitmask,
	(void *) &pm.tm_drift,
	(void *) &pm.tm_hold,
	(void *) &pm.tm_sine,
	(void *) &pm.tm_measure,
	(void *) &pm.tm_end,
	(void *) &pm.fb_range,
	(void *) &pm.pb_i_hold,
	(void *) &pm.pb_i_hold_Q,
	(void *) &pm.pb_i_sine,
	(void *) &pm.pb_freq_sine_hz,
	(void *) &pm.pb_speed_low,
	(void *) &pm.pb_speed_high,
	(void *) &pm.pb_settle_threshold,
	(void *) &pm.pb_gain_P,
	(void *) &pm.pb_gain_I,
	(void *) &pm.scal_A_0,
	(void *) &pm.scal_A_1,
	(void *) &pm.scal_B_0,
	(void *) &pm.scal_B_1,
	(void *) &pm.scal_U_0,
	(void *) &pm.scal_U_1,
	(void *) &pm.fault_residual_maximal,
	(void *) &pm.fault_residual_maximal_hold,
	(void *) &pm.fault_drift_maximal,
	(void *) &pm.fault_low_voltage,
	(void *) &pm.fault_low_voltage_hold,
	(void *) &pm.fault_high_voltage,
	(void *) &pm.fault_high_voltage_hold,
	(void *) &pm.lu_gain_DA,
	(void *) &pm.lu_gain_QA,
	(void *) &pm.lu_gain_DP,
	(void *) &pm.lu_gain_DS,
	(void *) &pm.lu_gain_QS,
	(void *) &pm.lu_gain_QZ,
	(void *) &pm.lu_boost_slope,
	(void *) &pm.lu_boost_gain,
	(void *) &pm.lu_threshold_low,
	(void *) &pm.lu_threshold_high,
	(void *) &pm.hf_freq_hz,
	(void *) &pm.hf_swing_D,
	(void *) &pm.hf_gain_P,
	(void *) &pm.hf_gain_S,
	(void *) &pm.hf_gain_F,
	(void *) &pm.bemf_DFT[0],
	(void *) &pm.bemf_DFT[1],
	(void *) &pm.bemf_DFT[2],
	(void *) &pm.bemf_DFT[3],
	(void *) &pm.bemf_DFT[4],
	(void *) &pm.bemf_DFT[5],
	(void *) &pm.bemf_DFT[6],
	(void *) &pm.bemf_DFT[7],
	(void *) &pm.bemf_DFT[8],
	(void *) &pm.bemf_DFT[9],
	(void *) &pm.bemf_DFT[10],
	(void *) &pm.bemf_DFT[11],
	(void *) &pm.bemf_DFT[12],
	(void *) &pm.bemf_DFT[13],
	(void *) &pm.bemf_DFT[14],
	(void *) &pm.bemf_DFT[15],
	(void *) &pm.bemf_DFT[16],
	(void *) &pm.bemf_DFT[17],
	(void *) &pm.bemf_DFT[18],
	(void *) &pm.bemf_DFT[19],
	(void *) &pm.bemf_DFT[20],
	(void *) &pm.bemf_DFT[21],
	(void *) &pm.bemf_DFT[22],
	(void *) &pm.bemf_DFT[23],
	(void *) &pm.bemf_DFT[24],
	(void *) &pm.bemf_DFT[25],
	(void *) &pm.bemf_DFT[26],
	(void *) &pm.bemf_DFT[27],
	(void *) &pm.bemf_DFT[28],
	(void *) &pm.bemf_DFT[29],
	(void *) &pm.bemf_DFT[30],
	(void *) &pm.bemf_DFT[31],
	(void *) &pm.bemf_DFT[32],
	(void *) &pm.bemf_DFT[33],
	(void *) &pm.bemf_DFT[34],
	(void *) &pm.bemf_DFT[35],
	(void *) &pm.bemf_DFT[36],
	(void *) &pm.bemf_DFT[37],
	(void *) &pm.bemf_DFT[38],
	(void *) &pm.bemf_DFT[39],
	(void *) &pm.bemf_DFT[40],
	(void *) &pm.bemf_DFT[41],
	(void *) &pm.bemf_DFT[42],
	(void *) &pm.bemf_DFT[43],
	(void *) &pm.bemf_DFT[44],
	(void *) &pm.bemf_DFT[45],
	(void *) &pm.bemf_DFT[46],
	(void *) &pm.bemf_DFT[47],
	(void *) &pm.bemf_DFT[48],
	(void *) &pm.bemf_DFT[49],
	(void *) &pm.bemf_DFT[50],
	(void *) &pm.bemf_DFT[51],
	(void *) &pm.bemf_DFT[52],
	(void *) &pm.bemf_DFT[53],
	(void *) &pm.bemf_DFT[54],
	(void *) &pm.bemf_DFT[55],
	(void *) &pm.bemf_DFT[56],
	(void *) &pm.bemf_DFT[57],
	(void *) &pm.bemf_DFT[58],
	(void *) &pm.bemf_DFT[59],
	(void *) &pm.bemf_DFT[60],
	(void *) &pm.bemf_DFT[61],
	(void *) &pm.bemf_DFT[62],
	(void *) &pm.bemf_DFT[63],
	(void *) &pm.bemf_gain_D,
	(void *) &pm.bemf_gain_Q,
	(void *) &pm.bemf_N,
	(void *) &pm.const_lpf_U,
	(void *) &pm.const_E,
	(void *) &pm.const_R,
	(void *) &pm.const_Ld,
	(void *) &pm.const_Lq,
	(void *) &pm.const_Zp,
	(void *) &pm.const_J,
	(void *) &pm.i_maximal,
	(void *) &pm.i_maximal_low,
	(void *) &pm.i_power_consumption_maximal,
	(void *) &pm.i_power_regeneration_maximal,
	(void *) &pm.i_slew_rate_D,
	(void *) &pm.i_slew_rate_Q,
	(void *) &pm.i_gain_P_D,
	(void *) &pm.i_gain_I_D,
	(void *) &pm.i_gain_P_Q,
	(void *) &pm.i_gain_I_Q,
	(void *) &pm.s_maximal,
	(void *) &pm.s_slew_rate,
	(void *) &pm.s_slew_rate_forced,
	(void *) &pm.s_forced_D,
	(void *) &pm.s_nonl_gain_F,
	(void *) &pm.s_nonl_range,
	(void *) &pm.s_gain_P,
	(void *) &pm.s_gain_I,
	(void *) &pm.p_gain_P,
	(void *) &pm.lp_gain_0,
	(void *) &pm.lp_gain_1,
	(void *) &pm.lp_gain_2,
	(void *) &pm.lp_gain_3,
	*/

	{ NULL, 0, { NULL }, NULL }
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
reg_setval(reg_t *reg, const void *rval)
{
	if ((reg->bits & REG_READ_ONLY) == 0) {

		if (reg->proc != NULL) {

			reg->proc(reg, NULL, rval);
		}
		else {
			*reg->link.i = * (int *) rval;
		}
	}
}

static void
reg_printout(const reg_t *reg)
{
	const char		*fmt;
	float			f;
	int			n;

	printf("%s%s%s [%i] %s = ",
			(reg->bits & REG_INT) ? "I" : "F",
			(reg->bits & REG_CONFIG) ? "C" : " ",
			(reg->bits & REG_READ_ONLY) ? "R" : " ",
			reg - regfile, reg->sym);

	if (reg->bits & REG_INT) {

		reg_getval(reg, &n);

		fmt = (reg->bits & REG_I_HEX) ? "%8x" : "%i";

		printf(fmt, n);
	}
	else {
		reg_getval(reg, &f);

		if (reg->bits & REG_F_EXP) {

			fmt = (reg->bits & REG_F_LONG) ? "%4e" : "%2e";
		}
		else {
			fmt = (reg->bits & REG_F_LONG) ? "%3f" : "%1f";
		}

		printf(fmt, &f);
	}

	fmt = reg->sym + strlen(reg->sym) + 1;

	if (*fmt != 0) {

		printf(" (%s)", fmt);
	}

	puts(EOL);
}

SH_DEF(reg_list)
{
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (strstr(reg->sym, s) != NULL) {

			reg_printout(reg);
		}
	}
}

SH_DEF(reg_set)
{
	const reg_t		*reg;
	float			f;
	int			n;

	if (stoi(&n, s) != NULL) {

		if (n >= 0 && n < REG_MAX) {

			reg = regfile + n;
			s = strtok(s, " ");

			if (reg->bits & REG_INT) {

				if (stoi(&n, s) != NULL) {

					reg_setval(reg, &n);
				}
			}
			else {
				if (stof(&f, s) != NULL) {

					reg_setval(reg, &f);
				}
			}

			reg_printout(reg);
		}
	}
}

SH_DEF(reg_import)
{
	const reg_t		*reg;

}

