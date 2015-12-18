/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

#include "hal/hal.h"
#include "lib.h"
#include "m.h"
#include "pmc.h"
#include "task.h"

extern void irq_avg_value_8();
extern void pm_const_E_wb(const char *s);
extern void pm_i_slew_rate_auto(const char *s);
extern void pm_i_gain_auto(const char *s);

static int
ap_wait_for_idle()
{
	while (pm.m_state != PMC_STATE_IDLE)
		taskIOMUX();

	return pm.m_errno;
}

#define AP_WAIT_FOR_IDLE()	\
{ if (ap_wait_for_idle() != PMC_OK) { printf("ERROR %i: %s" EOL, \
		pm.m_errno, pmc_strerror(pm.m_errno)); break; } }

void ap_identify_base(const char *s)
{
	float			IMP[6];

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();

		printf("A0 %3f (A)" EOL, &pm.scal_A[0]);
		printf("B0 %3f (A)" EOL, &pm.scal_B[0]);

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_resistance(pm.wave_DFT, IMP);
		pm.const_R += IMP[0];

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_resistance(pm.wave_DFT, IMP);
		pm.const_R += IMP[0];

		printf("R %4e (Ohm)" EOL, &pm.const_R);

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

		if (fabsf(IMP[2]) < 45.f) {

			pm.const_Ld = IMP[0];
			pm.const_Lq = IMP[1];
		}
		else {
			pm.const_Ld = IMP[1];
			pm.const_Lq = IMP[0];
		}

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

		if (fabsf(IMP[2]) < 45.f) {

			pm.const_Ld = IMP[0];
			pm.const_Lq = IMP[1];
		}
		else {
			pm.const_Ld = IMP[1];
			pm.const_Lq = IMP[0];
		}

		pm.const_Ld_inversed = 1.f / pm.const_Ld;
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

		printf("Ld %4e (H)" EOL, &pm.const_Ld);
		printf("Lq %4e (H)" EOL, &pm.const_Lq);

		pm_i_slew_rate_auto(EOL);
		pm_i_gain_auto(EOL);

		break;
	}
	while (0);
}

void ap_identify_const_R_abc(const char *s)
{
	float			temp[2], iSP, dU, R[3], STD;
	int			xPWM;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
		temp[0] = pm.wave_i_hold_X;
		temp[1] = pm.wave_i_hold_Y;

		iSP = pm.wave_i_hold_X;
		stof(&iSP, s);

		printf("iSP %3f (A)" EOL, &iSP);

		dU = iSP * pm.const_R / pm.const_U;
		xPWM = dU * pm.pwm_resolution;
		dU *= 100.f;
		printf("U: %i (tk) %2f %%" EOL, xPWM, &dU);

		pm.wave_i_hold_X = iSP;
		pm.wave_i_hold_Y = 0.f;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_resistance(pm.wave_DFT, R + 0);
		R[0] += pm.const_R;

		printf("R[a] %4e (Ohm)" EOL, &R[0]);

		pm.wave_i_hold_X = - .5f * iSP;
		pm.wave_i_hold_Y = .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_resistance(pm.wave_DFT, R + 1);
		R[1] += pm.const_R;

		printf("R[b] %4e (Ohm)" EOL, &R[1]);

		pm.wave_i_hold_X = - .5f * iSP;
		pm.wave_i_hold_Y = - .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_resistance(pm.wave_DFT, R + 2);
		R[2] += pm.const_R;

		printf("R[c] %4e (Ohm)" EOL, &R[2]);

		pm.const_R = (R[0] + R[1] + R[2]) / 3.f;
		STD = sqrtf((R[0] - pm.const_R) * (R[0] - pm.const_R)
			+ (R[1] - pm.const_R) * (R[1] - pm.const_R)
			+ (R[2] - pm.const_R) * (R[2] - pm.const_R));
		STD = 100.f * STD / pm.const_R;

		/*pm.abc_DR_A = R[0] / pm.const_R;
		pm.abc_DR_B = R[1] / pm.const_R;
		pm.abc_DR_C = R[2] / pm.const_R;*/

		printf("R %4e (Ohm) STD %1f %%" EOL, &pm.const_R, &STD);

		pm.wave_i_hold_X = temp[0];
		pm.wave_i_hold_Y = temp[1];
	}
	while (0);
}

void ap_identify_const_L_(const char *s)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
	}
	while (0);
}

void ap_identify_const_E(const char *s)
{
	if (pm.lu_region == PMC_LU_DISABLED)
		return ;

	if (td.pIRQ != NULL)
		return ;

	td.avgIN[0] = &pm.lu_X[0];
	td.avgIN[1] = &pm.lu_X[1];
	td.avgIN[2] = &pm.lu_X[4];
	td.avgIN[3] = &pm.drift_Q;

	td.avgSUM[0] = 0.f;
	td.avgSUM[1] = 0.f;
	td.avgSUM[2] = 0.f;
	td.avgSUM[3] = 0.f;

	td.avgK = 4;
	td.avgN = 0;
	td.avgMAX = pm.freq_hz * pm.T_measure;

	td.pIRQ = &irq_avg_value_8;

	while (td.pIRQ != NULL)
		taskIOMUX();

	td.avgSUM[0] /= (float) td.avgN;
	td.avgSUM[1] /= (float) td.avgN;
	td.avgSUM[2] /= (float) td.avgN;
	td.avgSUM[3] /= (float) td.avgN;

	pm.const_E -= td.avgSUM[3] / td.avgSUM[2];
	pm_const_E_wb(EOL);
}

