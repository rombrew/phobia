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
#include "pm_control.h"
#include "regfile.h"
#include "shell.h"

#define REG(link, comment, bits, proc)	{ #link "\0" comment, bits, { (void *) &link }, proc }
#define REG_MAX				(sizeof(regfile) / sizeof(reg_t) - 1UL)

static void
reg_proc_default(const reg_t *reg, int *lval, const int *rval)
{
	if (lval != NULL) {

		*lval = *reg->link.i;
	}

	if (rval != NULL) {

		*reg->link.i = *rval;
	}
}

const reg_t		regfile[] = {

	REG(hal.USART_baud_rate,	"",	REG_INT | REG_CONFIG, NULL),
	REG(hal.PWM_freq_hz,		"Hz",	REG_INT | REG_CONFIG, NULL),
	REG(hal.PWM_dead_time_ns,	"ns",	REG_INT | REG_CONFIG, NULL),
	REG(ap.ntc_PCB.r_balance,	"Ohm",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_PCB.r_ntc_0,		"Ohm",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_PCB.ta_0,		"C",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_PCB.betta,		"",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_EXT.r_balance,	"Ohm",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_EXT.r_ntc_0,		"Ohm",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_EXT.ta_0,		"C",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(ap.ntc_EXT.betta,		"",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(pm.pwm_R,			"",	REG_INT | REG_READ_ONLY, NULL),
	REG(pm.pwm_MP,			"",	REG_INT | REG_CONFIG, NULL),
	REG(pm.error,			"",	REG_INT | REG_READ_ONLY, NULL),
	REG(pm.b_FORCED,		"",	REG_INT | REG_CONFIG, NULL),
	REG(pm.b_HFI,			"",	REG_INT | REG_CONFIG, NULL),
	REG(pm.b_SENSOR,		"",	REG_INT | REG_CONFIG, NULL),
	REG(pm.b_LOOP,			"",	REG_INT | REG_CONFIG, NULL),
	REG(pm.tm_skip,			"s",	REG_CONFIG, NULL),
	REG(pm.tm_probe,		"s",	REG_CONFIG, NULL),
	REG(pm.tm_hold,			"s",	REG_CONFIG, NULL),
	REG(pm.adjust_IA[0],		"A",	REG_CONFIG, NULL),
	REG(pm.adjust_IA[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.adjust_IB[0],		"A",	REG_CONFIG, NULL),
	REG(pm.adjust_IB[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.adjust_US[0],		"V",	REG_CONFIG, NULL),
	REG(pm.adjust_US[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.adjust_UA[0],		"V",	REG_CONFIG, NULL),
	REG(pm.adjust_UA[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.adjust_UB[0],		"V",	REG_CONFIG, NULL),
	REG(pm.adjust_UB[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.adjust_UC[0],		"V",	REG_CONFIG, NULL),
	REG(pm.adjust_UC[1],		"",	REG_F_EXP | REG_CONFIG, NULL),
	REG(pm.fb_i_range,		"A",	REG_CONFIG, NULL),
	REG(pm.fb_iA,			"A",	REG_READ_ONLY, NULL),
	REG(pm.fb_iB,			"A",	REG_READ_ONLY, NULL),
	REG(pm.fb_uA,			"V",	REG_READ_ONLY, NULL),
	REG(pm.fb_uB,			"V",	REG_READ_ONLY, NULL),
	REG(pm.fb_uC,			"V",	REG_READ_ONLY, NULL),
	REG(pm.probe_i_hold,		"A",	REG_CONFIG, NULL),
	REG(pm.probe_i_hold_Q,		"A",	REG_CONFIG, NULL),
	REG(pm.probe_i_sine,		"A",	REG_CONFIG, NULL),
	REG(pm.probe_freq_sine_hz,	"Hz",	REG_CONFIG, NULL),
	REG(pm.probe_gain_P,		"",	REG_F_EXP | REG_F_SHORT | REG_CONFIG, NULL),
	REG(pm.probe_gain_I,		"",	REG_F_EXP | REG_F_SHORT | REG_CONFIG, NULL),
	REG(pm.fault_zero_drift_maximal,	"A",	REG_CONFIG, NULL),
	REG(pm.fault_voltage_tolerance,		"V",	REG_CONFIG, NULL),
	REG(pm.fault_current_tolerance,		"A",	REG_CONFIG, NULL),
	REG(pm.fault_adjust_tolerance,		"",	REG_F_EXP | REG_F_SHORT | REG_CONFIG, NULL),
	REG(pm.fault_lu_residual_maximal,	"A",	REG_F_SHORT | REG_CONFIG, NULL),
	REG(pm.fault_lu_drift_Q_maximal,	"V",	REG_CONFIG, NULL),
	REG(pm.fault_supply_voltage_low,	"V",	REG_CONFIG, NULL),
	REG(pm.fault_supply_voltage_high,	"V",	REG_CONFIG, NULL),
	REG(pm.vsi_lpf_D,		"V",	REG_READ_ONLY, NULL),
	REG(pm.vsi_lpf_Q,		"V",	REG_READ_ONLY, NULL),
	REG(pm.lu_power_lpf,		"W",	REG_F_SHORT | REG_READ_ONLY, NULL),
	REG(pm.lu_residual_D,		"A",	REG_READ_ONLY, NULL),


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
reg_setval(const reg_t *reg, const void *rval)
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

void reg_print_fmt(const reg_t *reg)
{
	const char		*fmt;
	float			f;
	int			n;

	if (reg->bits & REG_INT) {

		reg_getval(reg, &n);

		fmt = (reg->bits & REG_I_HEX) ? "%8x" : "%i";

		printf(fmt, n);
	}
	else {
		reg_getval(reg, &f);

		fmt = (reg->bits & REG_F_EXP) ?
			(reg->bits & REG_F_SHORT) ? "%2e" : "%4e" :
			(reg->bits & REG_F_SHORT) ? "%1f" : "%3f" ;

		printf(fmt, &f);
	}
}

SH_DEF(reg_list)
{
	const reg_t		*reg;
	const char		*comment;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (strstr(reg->sym, s) != NULL) {

			printf("%s%s%s [%i] %s = ",
					(reg->bits & REG_INT) ? "I" : "F",
					(reg->bits & REG_CONFIG) ? "C" : ".",
					(reg->bits & REG_READ_ONLY) ? "R" : ".",
					(int) (reg - regfile), reg->sym);

			reg_print_fmt(reg);

			comment = reg->sym + strlen(reg->sym) + 1;

			if (*comment != 0) {

				printf(" (%s)", comment);
			}

			puts(EOL);
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

		reg_print_fmt(reg);
		puts(EOL);
	}
}

SH_DEF(reg_export)
{
	const reg_t		*reg;

	for (reg = regfile; reg->sym != NULL; ++reg) {

		if (reg->bits & REG_CONFIG) {

			printf("reg_set %s ", reg->sym);

			reg_print_fmt(reg);
			puts(EOL);
		}
	}
}

