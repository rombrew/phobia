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
#include "sh.h"
#include "task.h"

extern void irq_avg_value_8();

SH_DEF(pm_lu_threshold_auto);
SH_DEF(pm_const_E_wb);
SH_DEF(pm_i_slew_rate_auto);
SH_DEF(pm_i_gain_auto);

static int
ap_wait_for_idle()
{
	while (pm.m_state != PMC_STATE_IDLE)
		taskYIELD();

	return pm.m_errno;
}

#define AP_PRINT_ERROR()	\
	printf("ERROR %i: %s" EOL, pm.m_errno, pmc_strerror(pm.m_errno))

#define AP_WAIT_FOR_IDLE()	\
	{ if (ap_wait_for_idle() != PMC_OK) { AP_PRINT_ERROR(); break; } }

#define AP_EXIT_IF_ERROR()	{ if (pm.m_errno != PMC_OK) break; }

SH_DEF(ap_identify_base)
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

SH_DEF(ap_identify_const_R_abc)
{
	float			temp[2], iSP, U, R[3], SD;
	int			xPWM;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
		temp[0] = pm.wave_i_hold_X;
		temp[1] = pm.wave_i_hold_Y;

		iSP = pm.wave_i_hold_X;
		stof(&iSP, s);

		printf("SP %3f (A)" EOL, &iSP);

		U = iSP * pm.const_R / pm.const_U;
		xPWM = U * pm.pwm_resolution;
		U *= 100.f;

		printf("U: %i (tk) %2f %%" EOL, xPWM, &U);

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
		SD = sqrtf((R[0] - pm.const_R) * (R[0] - pm.const_R)
			+ (R[1] - pm.const_R) * (R[1] - pm.const_R)
			+ (R[2] - pm.const_R) * (R[2] - pm.const_R));
		SD = 100.f * SD / pm.const_R;

		/*pm.abc_DR_A = R[0] / pm.const_R;
		pm.abc_DR_B = R[1] / pm.const_R;
		pm.abc_DR_C = R[2] / pm.const_R;*/

		printf("R %4e (Ohm) SD %1f %%" EOL, &pm.const_R, &SD);

		pm.wave_i_hold_X = temp[0];
		pm.wave_i_hold_Y = temp[1];
	}
	while (0);
}

SH_DEF(ap_identify_const_E)
{
	if (pm.lu_region == PMC_LU_DISABLED)
		return ;

	if (td.pIRQ != NULL)
		return ;

	td.avg_IN[0] = &pm.lu_X[0];
	td.avg_IN[1] = &pm.lu_X[1];
	td.avg_IN[2] = &pm.lu_X[4];
	td.avg_IN[3] = &pm.drift_Q;

	td.avg_SUM[0] = 0.f;
	td.avg_SUM[1] = 0.f;
	td.avg_SUM[2] = 0.f;
	td.avg_SUM[3] = 0.f;

	td.avg_K = 4;
	td.avg_N = 0;
	td.avg_MAX = pm.freq_hz * pm.T_measure;

	halFence();

	td.pIRQ = &irq_avg_value_8;

	while (td.pIRQ != NULL)
		taskYIELD();

	td.avg_SUM[0] /= (float) td.avg_N;
	td.avg_SUM[1] /= (float) td.avg_N;
	td.avg_SUM[2] /= (float) td.avg_N;
	td.avg_SUM[3] /= (float) td.avg_N;

	pm.const_E -= td.avg_SUM[3] / td.avg_SUM[2];
	pm_const_E_wb(EOL);
}

SH_DEF(ap_identify_const_J)
{
	/*if (pm.lu_region == PMC_LU_DISABLED)
		return ;

	if (td.pIRQ != NULL)
		return ;

	do {
	}
	while (0);*/
}

static int
ap_wait_for_settle(float wSP, int xT)
{
	float			SR;
	int			uEND;

	pm.p_set_point_w = wSP;

	td.uTIM = 0;
	SR = (pm.m_bitmask & PMC_BIT_SERVO_FORCED_CONTROL)
		? pm.p_forced_slew_rate_w : pm.p_slew_rate_w;
	uEND = (int) (100.f * pm.p_set_point_w / SR) + xT;

	do {
		taskYIELD();
		AP_EXIT_IF_ERROR();
	}
	while (td.uTIM < uEND);

	if (pm.m_errno != PMC_OK)
		AP_PRINT_ERROR();

	return pm.m_errno;
}

SH_DEF(ap_blind_spinup)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pm.m_bitmask |= PMC_BIT_SERVO_CONTROL_LOOP;
		pm.m_bitmask |= (pm.m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION)
			? 0 : PMC_BIT_SERVO_FORCED_CONTROL;

		pmc_request(&pm, PMC_STATE_START);
		AP_WAIT_FOR_IDLE();

		ap_wait_for_settle(2.f * pm.lu_threshold_high, 100);

		pm.m_bitmask &= ~PMC_BIT_SERVO_FORCED_CONTROL;
	}
	while (0);
}

SH_DEF(ap_probe_base)
{
	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	do {
		ap_identify_base(EOL);
		AP_EXIT_IF_ERROR();

		ap_blind_spinup(EOL);
		AP_EXIT_IF_ERROR();

		ap_identify_const_E(EOL);
		AP_EXIT_IF_ERROR();

		ap_wait_for_settle(1.f / pm.const_E, 100);
		AP_EXIT_IF_ERROR();

		ap_identify_const_E(EOL);
		AP_EXIT_IF_ERROR();

		pm_lu_threshold_auto(EOL);

		pmc_request(&pm, PMC_STATE_STOP);
		pm.m_bitmask &= ~PMC_BIT_SERVO_CONTROL_LOOP;
	}
	while (0);
}

SH_DEF(ap_probe_hfi)
{
	float			Qpc;

	if (pm.lu_region != PMC_LU_DISABLED)
		return ;

	Qpc = fabsf(1.f - pm.const_Ld / pm.const_Lq) * 100.f;

	printf("Q %.1f %%" EOL, &Qpc);

	do {
		pm.m_bitmask |= PMC_BIT_HIGH_FREQUENCY_INJECTION;

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_START);
		AP_WAIT_FOR_IDLE();

	}
	while (0);
}

