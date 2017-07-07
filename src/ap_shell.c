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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "hal/hal.h"
#include "hal_task.h"
#include "lib.h"
#include "pm_control.h"
#include "pm_math.h"
#include "shell.h"

#define AP_ERROR_BARRIER()	\
	{ if (pm.m_errno != PMC_OK) { break; } }

#define AP_PRINT_ERROR()	\
	{ if (pm.m_errno != PMC_OK) { printf("ERROR %i: %s" EOL, \
		pm.m_errno, pmc_strerror(pm.m_errno)); } }

#define AP_WAIT_FOR(expr)	\
	{ do { if (pm.m_errno != PMC_OK || (expr)) break; vTaskDelay(1); } while (1); }

#define AP_WAIT_FOR_IDLE()	AP_WAIT_FOR(pm.m_state == PMC_STATE_IDLE)

void pm_i_slew_rate_auto(const char *s);
void pm_i_gain_auto(const char *s);
void pm_const_E_wb(const char *s);
void pm_s_slew_rate_auto(const char *s);
void pm_update_const_L(const char *s);
void pm_update_scal_AB(const char *s);

SH_DEF(ap_calibrate)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_CALIBRATION);

		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm_update_scal_AB(EOL);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_blind_turn)
{
	float		temp[2], R;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	temp[0] = pm.pb_freq_sine_hz;
	temp[1] = pm.tm_sine;

	do {
		AP_ERROR_BARRIER();

		pm.pb_freq_sine_hz = 1.f;
		pm.tm_sine = 1.f;

		if (stof(&pm.tm_sine, s) != NULL) {

			s = strtok(s, " ");
			stof(&pm.pb_freq_sine_hz, s);

			if (pm.pb_freq_sine_hz > M_EPS_F)
				pm.tm_sine /= pm.pb_freq_sine_hz;

			pm.tm_sine = (pm.tm_sine < 0.f) ? 0.f :
				(pm.tm_sine > 60.f) ? 60.f : pm.tm_sine ;
		}

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, &R);
		pm.const_R += R;

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();
	}
	while (0);

	pm.pb_freq_sine_hz = temp[0];
	pm.tm_sine = temp[1];

	AP_PRINT_ERROR();
}

SH_DEF(ap_identify_base)
{
	float			IMP[6];

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, IMP);
		pm.const_R += IMP[0];

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, IMP);
		pm.const_R += IMP[0];

		printf("R %4e (Ohm)" EOL, &pm.const_R);

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm_update_const_L(EOL);

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pm_update_const_L(EOL);
		pm_i_slew_rate_auto(EOL);
		pm_i_gain_auto(EOL);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_identify_const_R_abc)
{
	float			temp[2], iSP, U, R[3], SD;
	int			xPWM;

	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	temp[0] = pm.pb_i_hold;
	temp[1] = pm.pb_i_hold_Q;

	do {
		AP_ERROR_BARRIER();

		iSP = pm.pb_i_hold;
		stof(&iSP, s);

		printf("SP %3f (A)" EOL, &iSP);

		U = iSP * pm.const_R / pm.const_U;
		xPWM = U * pm.pwm_resolution;
		U *= 100.f;

		printf("U: %i (tk) %2f %%" EOL, xPWM, &U);

		pm.pb_i_hold = iSP;
		pm.pb_i_hold_Q = 0.f;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, R + 0);
		R[0] += pm.const_R;

		printf("R[a] %4e (Ohm)" EOL, &R[0]);

		pm.pb_i_hold = - .5f * iSP;
		pm.pb_i_hold_Q = .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, R + 1);
		R[1] += pm.const_R;

		printf("R[b] %4e (Ohm)" EOL, &R[1]);

		pm.pb_i_hold = - .5f * iSP;
		pm.pb_i_hold_Q = - .8660254f * iSP;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		pmc_resistance(pm.pb_DFT, R + 2);
		R[2] += pm.const_R;

		printf("R[c] %4e (Ohm)" EOL, &R[2]);

		pm.const_R = (R[0] + R[1] + R[2]) / 3.f;
		SD = sqrtf((R[0] - pm.const_R) * (R[0] - pm.const_R)
			+ (R[1] - pm.const_R) * (R[1] - pm.const_R)
			+ (R[2] - pm.const_R) * (R[2] - pm.const_R));
		SD = 100.f * SD / pm.const_R;

		printf("R %4e (Ohm) SD %1f %%" EOL, &pm.const_R, &SD);
	}
	while (0);

	pm.pb_i_hold = temp[0];
	pm.pb_i_hold_Q = temp[1];

	AP_PRINT_ERROR();
}

SH_DEF(ap_identify_const_E)
{
	float			AVG[4];

	SH_ASSERT(pm.lu_region != PMC_LU_DISABLED);

	do {
		ts_av_float_4(&pm.lu_X[0], &pm.lu_X[1], &pm.lu_X[4],
				&pm.drift_Q, AVG, pm.tm_measure);

		AP_ERROR_BARRIER();

		pm.const_E -= AVG[3] / AVG[2];
		pm_const_E_wb(EOL);
	}
	while (0);

	AP_PRINT_ERROR();
}

static void
ap_wait_for_settle(float wSP)
{
	const float		dT = .1f;
	float			leftT, X4, lastX4, dX4;

	pm.s_set_point = wSP;

	leftT = 3.f;
	lastX4 = ts_av_float_1(pm.lu_X + 4, dT);

	do {
		X4 = ts_av_float_1(pm.lu_X + 4, dT);

		AP_ERROR_BARRIER();

		dX4 = X4 - lastX4;
		lastX4 = X4;

		if (fabsf(dX4) < pm.pb_settle_threshold)
			break;

		leftT -= dT;
	}
	while (leftT > 0.f);
}

static void
ap_J_handler()
{
	if (ts.av_sample_N <= ts.av_sample_MAX) {

		if (ts.av_sample_N == 0)
			ts.av_VAL[1] = pm.lu_X[4];

		ts.av_VAL[0] += pm.lu_X[1];
		ts.av_sample_N++;
	}
	else {
		ts.av_VAL[2] = pm.lu_X[4];
		ts.p_irq_callback = NULL;
	}
}

static void
ap_J_float_1(float *X1, float *X40, float *X41, float time)
{
	if (ts.p_irq_callback == NULL) {

		ts.av_VAL[0] = 0.f;
		ts.av_sample_N = 0;
		ts.av_sample_MAX = pm.freq_hz * time;

		halFence();
		ts.p_irq_callback = &ap_J_handler;

		while (ts.p_irq_callback != NULL)
			vTaskDelay(1);

		*X1 = ts.av_VAL[0] / (float) ts.av_sample_N;
		*X40 = ts.av_VAL[1];
		*X41 = ts.av_VAL[2];
	}
}

SH_DEF(ap_identify_const_J)
{
	float			J0, J1, X1, X40, X41;

	SH_ASSERT(pm.lu_region != PMC_LU_DISABLED);
	SH_ASSERT(pm.m_bitmask & PMC_BIT_SPEED_CONTROL_LOOP);
	SH_ASSERT((pm.m_bitmask & PMC_BIT_POSITION_CONTROL_LOOP) == 0);
	SH_ASSERT((pm.m_bitmask & PMC_BIT_FORCED_CONTROL) == 0);
	SH_ASSERT((pm.m_bitmask & PMC_BIT_POWER_CONTROL_LOOP) == 0);

	do {
		AP_ERROR_BARRIER();

		ap_wait_for_settle(pm.pb_speed_low);
		AP_ERROR_BARRIER();

		pm.s_set_point = pm.pb_speed_high;
		ap_J_float_1(&X1, &X40, &X41, pm.tm_measure);
		AP_ERROR_BARRIER();

		J0 = pm.const_E * (X1 * pm.tm_measure)
			* 1.5f * (pm.const_Zp * pm.const_Zp)
			* pm.dT / (X41 - X40);

		pm.s_set_point = pm.pb_speed_low;
		ap_J_float_1(&X1, &X40, &X41, pm.tm_measure);
		AP_ERROR_BARRIER();

		J1 = pm.const_E * (X1 * pm.tm_measure)
			* 1.5f * (pm.const_Zp * pm.const_Zp)
			* pm.dT / (X41 - X40);

		pm.const_J = (J0 + J1) / 2.f;
		printf("J %4e (kgm2)" EOL, &pm.const_J);
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_blind_spinup)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	pm.m_bitmask &= ~(PMC_BIT_POSITION_CONTROL_LOOP | PMC_BIT_POWER_CONTROL_LOOP);
	pm.m_bitmask |= PMC_BIT_SPEED_CONTROL_LOOP | PMC_BIT_FORCED_CONTROL;

	do {
		AP_ERROR_BARRIER();

		if (strchr(s, 'f') == NULL) {

			pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
			AP_WAIT_FOR_IDLE();
			AP_ERROR_BARRIER();

			pmc_request(&pm, PMC_STATE_WAVE_HOLD);
			AP_WAIT_FOR_IDLE();
			AP_ERROR_BARRIER();
		}

		pmc_request(&pm, PMC_STATE_START);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();

		ap_wait_for_settle(pm.pb_speed_low);
		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
}

SH_DEF(ap_probe_base)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		ap_identify_base(EOL);
		AP_ERROR_BARRIER();

		ap_blind_spinup("f");
		AP_ERROR_BARRIER();

		pm.m_bitmask &= ~PMC_BIT_FORCED_CONTROL;

		ap_identify_const_E(EOL);
		AP_ERROR_BARRIER();

		ap_wait_for_settle(pm.pb_speed_high);
		AP_ERROR_BARRIER();

		ap_identify_const_E(EOL);
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_STOP);
		AP_WAIT_FOR_IDLE();
		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
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

		pmc_request(&pm, PMC_STATE_START);
		AP_WAIT_FOR_IDLE();

	}
	while (0);
}

SH_DEF(ap_probe_speed_control)
{
	SH_ASSERT(pm.lu_region == PMC_LU_DISABLED);

	do {
		AP_ERROR_BARRIER();

		ap_blind_spinup(EOL);
		AP_ERROR_BARRIER();

		pm.m_bitmask &= ~PMC_BIT_FORCED_CONTROL;

		ap_identify_const_J(EOL);
		AP_ERROR_BARRIER();

		pmc_request(&pm, PMC_STATE_STOP);

		AP_ERROR_BARRIER();
	}
	while (0);

	AP_PRINT_ERROR();
}

