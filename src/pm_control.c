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

#include "pm_control.h"
#include "pm_fsm.h"
#include "pm_math.h"

void pm_default(pmc_t *pm)
{
	pm->pwm_MP = 8;

	pm->b_FORCED = 1;
	pm->b_HFI = 0;
	pm->b_LOOP = 1;

	pm->tm_skip = .1f;
	pm->tm_probe = .2f;
	pm->tm_hold = .5f;

	pm->adjust_IA[0] = 0.f;
	pm->adjust_IA[1] = 1.f;
	pm->adjust_IB[0] = 0.f;
	pm->adjust_IB[1] = 1.f;
	pm->adjust_US[0] = 0.f;
	pm->adjust_US[1] = 1.f;
	pm->adjust_UA[0] = 0.f;
	pm->adjust_UA[1] = 1.f;
	pm->adjust_UB[0] = 0.f;
	pm->adjust_UB[1] = 1.f;
	pm->adjust_UC[0] = 0.f;
	pm->adjust_UC[1] = 1.f;

	pm->fb_i_range = 50.f;

	pm->probe_i_hold = 5.f;
	pm->probe_i_hold_Q = 0.f;
	pm->probe_i_sine = 1.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 16.f;
	pm->probe_speed_ramp = 1100.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_zero_drift_maximal = 1.f;
	pm->fault_voltage_tolerance = 1.f;
	pm->fault_current_tolerance = 1.f;
	pm->fault_adjust_tolerance = 3E-2f;
	pm->fault_lu_residual_maximal = 50.f;
	pm->fault_lu_drift_Q_maximal = 7.f;
	pm->fault_supply_voltage_low = 5.f;
	pm->fault_supply_voltage_high = 55.f;

	pm->lu_gain_DA = 5E-1f;
	pm->lu_gain_QA = 5E-1f;
	pm->lu_gain_DP = 5E-3f;
	pm->lu_gain_DS = 5E+0f;
	pm->lu_gain_QS = 1E+0f;
	pm->lu_gain_QZ = 5E-3f;
	pm->lu_boost_slope = .2f;
	pm->lu_boost_gain = 7.f;
	pm->lu_BEMF_low = .2f;
	pm->lu_BEMF_high = .3f;
	pm->lu_forced_D = 5.f;
	pm->lu_forced_accel = 1500.f;

	pm->hf_freq_hz = pm->freq_hz / 12.f;
	pm->hf_swing_D = 1.f;
	pm->hf_gain_P = 5E-2f;
	pm->hf_gain_S = 5E+1f;
	pm->hf_gain_F = 2E-3f;

	pm->const_lpf_U = 0.f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->i_maximal = 50.f;
	pm->i_maximal_weak = 10.f;
	pm->i_power_consumption_maximal = 2050.f;
	pm->i_power_regeneration_maximal = -1050.f;
	pm->i_slew_rate_D = 5E+4f;
	pm->i_slew_rate_Q = 5E+4f;
	pm->i_gain_P_D = 2E-1f;
	pm->i_gain_I_D = 3E-2f;
	pm->i_gain_P_Q = 2E-1f;
	pm->i_gain_I_Q = 3E-2f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 16.f);
	pm->s_slew_rate = 5E+6f;
	pm->s_gain_P = 5E-2f;
	pm->s_gain_I = 2E-3f;
	pm->p_gain_P = 50.f;
	pm->p_gain_I = 0.f;

	pm->lpf_gain_POWER = .1f;
	pm->lpf_gain_LU = .1f;
	pm->lpf_gain_VSI = .1f;
	pm->lpf_gain_U = .1f;
}

void pm_tune_current_loop(pmc_t *pm)
{
	pm->i_slew_rate_D = .2f * pm->const_lpf_U / pm->const_Ld;
	pm->i_slew_rate_Q = .2f * pm->const_lpf_U / pm->const_Lq;
	pm->i_gain_P_D = (.5f * pm->const_Ld * pm->freq_hz - pm->const_R);
	pm->i_gain_I_D = 5E-2f * pm->const_Ld * pm->freq_hz;
	pm->i_gain_P_Q = (.5f * pm->const_Lq * pm->freq_hz - pm->const_R);
	pm->i_gain_I_Q = 5E-2f * pm->const_Lq * pm->freq_hz;
}

static void
pm_equation_3(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ, X4, X5, R1, E1, fD, fQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = pm->lu_X[4];
	X5 = pm->lu_drift_Q;
	R1 = pm->const_R;
	E1 = pm->const_E;

	fD = pm->const_Ld * X[0] + E1;
	fQ = pm->const_Lq * X[1];

	D[0] = (uD - R1 * X[0] + fQ * X4) / pm->const_Ld;
	D[1] = (uQ - R1 * X[1] - fD * X4 + X5) / pm->const_Lq;
	D[2] = X4;
}

static void
pm_solve_2(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		D1[3], D2[3], X2[4];
	float		dT;

	/* Second-order ODE solver.
	 * */

	pm_equation_3(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	pm_rotf(X2 + 2, D1[2] * dT, X + 2);

	pm_equation_3(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	pm_rotf(X + 2, (D1[2] + D2[2]) * dT, X + 2);
}

static void
pm_HFI_update(pmc_t *pm, float eD, float eQ)
{
	float		*X = pm->lu_X;
	float		eR, dR, C2;

	eR = pm->hf_CS[1] * eQ;
	dR = pm->hf_gain_P * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	pm_rotf(X + 2, dR, X + 2);

	X[4] += pm->hf_gain_S * eR;

	if (pm->b_HFI == 2) {

		C2 = pm->hf_CS[0] * pm->hf_CS[0] - pm->hf_CS[1] * pm->hf_CS[1];
		pm->hf_flux_polarity += eD * C2 * pm->hf_gain_F;

		if (pm->hf_flux_polarity > 1.f) {

			X[2] = - X[2];
			X[3] = - X[3];
			pm->hf_flux_polarity = 0.f;
		}
		else if (pm->hf_flux_polarity < 0.f)
			pm->hf_flux_polarity = 0.f;
	}
}

static void
pm_LU_measure(pmc_t *pm, float Z[2])
{
	float		*X = pm->lu_X;
	float		iA, iB, iX, iY;

	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	iA = iX;
	iB = - .5f * iX + .8660254f * iY;

	iA = (iA < - pm->fb_i_range) ? - pm->fb_i_range :
		(iA > pm->fb_i_range) ? pm->fb_i_range : iA;
	iB = (iB < - pm->fb_i_range) ? - pm->fb_i_range :
		(iB > pm->fb_i_range) ? pm->fb_i_range : iB;

	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	Z[0] = X[2] * iX + X[3] * iY;
	Z[1] = X[2] * iY - X[3] * iX;
}

static void
pm_LU_update(pmc_t *pm)
{
	float		*X = pm->lu_X, Z[2];
	float		iX, iY, iD, iQ;
	float		eD, eQ, eR, dR, dS, bF;

	iX = pm->fb_iA;
	iY = .57735027f * iX + 1.1547005f * pm->fb_iB;

	pm->lu_power_lpf += (1.5f * (iX * pm->vsi_X + iY * pm->vsi_Y) - pm->lu_power_lpf)
		* pm->lpf_gain_POWER;

	iD = X[2] * iX + X[3] * iY;
	iQ = X[2] * iY - X[3] * iX;

	pm_LU_measure(pm, Z);

	eD = iD - Z[0];
	eQ = iQ - Z[1];

	pm->lu_residual_D = eD;
	pm->lu_residual_Q = eQ;
	pm->lu_residual_lpf += (eD * eD + eQ * eQ - pm->lu_residual_lpf)
		* pm->lpf_gain_LU;

	if (pm->lu_residual_lpf > pm->fault_lu_residual_maximal) {

		pm->error = PM_ERROR_LU_RESIDUAL_UNSTABLE;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	X[0] += pm->lu_gain_DA * eD;
	X[1] += pm->lu_gain_QA * eQ;

	if (pm->lu_region == PM_LU_OPEN_LOOP) {

		dS = pm->lu_forced_accel * pm->dT;
		dS = (pm->i_set_point_Q < - M_EPS_F) ? - dS :
			(pm->i_set_point_Q > M_EPS_F) ? dS : 0.f;

		X[4] += dS;
	}
	else {
		if (pm->b_HFI != 0 && pm->lu_region == PM_LU_CLOSED_LOW) {

			pm_HFI_update(pm, eD, eQ);
		}
		else {
			eR = (X[4] < 0.f) ? - eD : eD;
			dR = pm->lu_gain_DP * eR;
			dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
			pm_rotf(X + 2, dR, X + 2);

			dR = pm->lu_gain_DS * eR - pm->lu_gain_QS * eQ;
			bF = pm->lu_residual_lpf * pm->lu_boost_slope;
			bF = (bF > 1.f) ? 1.f : bF;
			X[4] += dR * (1.f + bF * pm->lu_boost_gain);

			if (pm->lu_region == PM_LU_CLOSED_HIGH) {

				pm->lu_drift_Q += pm->lu_gain_QZ * eQ;
			}
		}
	}

	pm_solve_2(pm);
}

static void
pm_LU_job(pmc_t *pm)
{
	float			BEMF, X4MAX;

	BEMF = fabsf(pm->lu_X[4] * pm->const_E);

	if (pm->lu_region == PM_LU_CLOSED_HIGH) {

		if (BEMF < pm->lu_BEMF_low) {

			if (pm->b_FORCED != 0) {

				pm->lu_region = PM_LU_OPEN_LOOP;
			}
			else {
				pm->lu_region = PM_LU_CLOSED_LOW;
			}
		}
	}
	else {
		if (BEMF > pm->lu_BEMF_high) {

			pm->lu_region = PM_LU_CLOSED_HIGH;
			pm->lu_drift_Q = 0.f;
		}
	}

	X4MAX = pm->freq_hz * (2.f * M_PI_F / 12.f);

	if (fabsf(pm->lu_X[4]) > X4MAX) {

		pm->error = PM_ERROR_LU_SPEED_HIGH;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	if (fabsf(pm->lu_drift_Q) > pm->fault_lu_drift_Q_maximal) {

		pm->error = PM_ERROR_LU_DRIFT_HIGH;
		pm_fsm_req(pm, PM_STATE_HALT);
	}
}

void pm_VSI_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, uD, uQ, Q;
	float		uMIN, uMAX;
	int		xA, xB, xC;
	int		xMIN, xMAX;

	uA = uX;
	uB = - .5f * uX + .8660254f * uY;
	uC = - .5f * uX - .8660254f * uY;

	if (uA < uB) {

		uMIN = uA;
		uMAX = uB;
	}
	else {
		uMIN = uB;
		uMAX = uA;
	}

	if (uC < uMIN)

		uMIN = uC;

	else if (uMAX < uC)

		uMAX = uC;

	Q = uMAX - uMIN;

	if (Q > 1.f) {

		Q = 1.f / Q;
		uA *= Q;
		uB *= Q;
		uC *= Q;
		uMIN *= Q;
	}

	Q = 0.f - uMIN;

	uA += Q;
	uB += Q;
	uC += Q;

	xA = (int) (pm->pwm_R * uA + .5f);
	xB = (int) (pm->pwm_R * uB + .5f);
	xC = (int) (pm->pwm_R * uC + .5f);

	xMIN = pm->pwm_MP;
	xMAX = pm->pwm_R - xMIN;

	xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->pwm_R : xA;
	xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->pwm_R : xB;
	xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->pwm_R : xC;

	pm->pDC(xA, xB, xC);

	Q = (1.f / 3.f) * (xA + xB + xC);
	uA = (xA - Q) * pm->const_lpf_U / pm->pwm_R;
	uB = (xB - Q) * pm->const_lpf_U / pm->pwm_R;

	pm->vsi_X = uA;
	pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;

	uD = pm->lu_X[2] * pm->vsi_X + pm->lu_X[3] * pm->vsi_Y;
	uQ = pm->lu_X[2] * pm->vsi_Y - pm->lu_X[3] * pm->vsi_X;

	pm->vsi_lpf_D += (uD - pm->vsi_lpf_D) * pm->lpf_gain_VSI;
	pm->vsi_lpf_Q += (uQ - pm->vsi_lpf_Q) * pm->lpf_gain_VSI;
}

static void
pm_current_control(pmc_t *pm)
{
	float		sD, sQ, eD, eQ;
	float		uD, uQ, uX, uY, wP;
	float		uHF = 0.f, temp;

	sD = pm->i_set_point_D;
	sQ = pm->i_set_point_Q;

	if (pm->lu_region == PM_LU_OPEN_LOOP) {

		sD = pm->lu_forced_D;
		sQ = 0.f;
	}

	temp = (pm->lu_region != PM_LU_CLOSED_HIGH)
		? pm->i_maximal_weak : pm->i_maximal;
	sD = (sD > temp) ? temp : (sD < - temp) ? - temp : sD;
	sQ = (sQ > temp) ? temp : (sQ < - temp) ? - temp : sQ;

	wP = 1.5f * (sD * pm->vsi_lpf_D + sQ * pm->vsi_lpf_Q);

	if (wP > pm->i_power_consumption_maximal) {

		temp = pm->i_power_consumption_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}
	else if (wP < pm->i_power_regeneration_maximal) {

		temp = pm->i_power_regeneration_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}

	temp = pm->i_slew_rate_D * pm->dT;
	pm->i_track_point_D = (pm->i_track_point_D < sD - temp)
		? pm->i_track_point_D + temp : (pm->i_track_point_D > sD + temp)
		? pm->i_track_point_D - temp : sD;

	temp = pm->i_slew_rate_Q * pm->dT;
	pm->i_track_point_Q = (pm->i_track_point_Q < sQ - temp)
		? pm->i_track_point_Q + temp : (pm->i_track_point_Q > sQ + temp)
		? pm->i_track_point_Q - temp : sQ;

	/* Obtain discrepancy.
	 * */
	eD = pm->i_track_point_D - pm->lu_X[0];
	eQ = pm->i_track_point_Q - pm->lu_X[1];

	if (pm->b_HFI != 0 && pm->lu_region == PM_LU_CLOSED_LOW) {

		temp = 2.f * M_PI_F * pm->hf_freq_hz;
		pm_rotf(pm->hf_CS, temp * pm->dT, pm->hf_CS);

		eD = (fabsf(eD) < pm->hf_swing_D) ? 0.f : eD;
		uHF = pm->hf_CS[0] * pm->hf_swing_D * temp * pm->const_Ld;
	}

	uD = pm->i_gain_P_D * eD + uHF;
	uQ = pm->i_gain_P_Q * eQ;

	temp = (2.f / 3.f) * pm->const_lpf_U;

	pm->i_integral_D += pm->i_gain_I_D * eD;
	pm->i_integral_D = (pm->i_integral_D > temp) ? temp :
		(pm->i_integral_D < - temp) ? - temp : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I_Q * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > temp) ? temp :
		(pm->i_integral_Q < - temp) ? - temp : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	uX = pm->lu_X[2] * uD - pm->lu_X[3] * uQ;
	uY = pm->lu_X[3] * uD + pm->lu_X[2] * uQ;

	pm_VSI_control(pm, uX / pm->const_lpf_U, uY / pm->const_lpf_U);
}

static void
pm_speed_control(pmc_t *pm)
{
	float		iSP, wSP, eD, temp;

	wSP = pm->s_set_point;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	if (1) {

		temp = pm->s_slew_rate * pm->dT;
		pm->s_track_point = (pm->s_track_point < wSP - temp)
			? pm->s_track_point + temp : (pm->s_track_point > wSP + temp)
			? pm->s_track_point - temp : wSP;

		/* Obtain discrepancy.
		 * */
		eD = pm->s_track_point - pm->lu_X[4];

		iSP = pm->s_gain_P * eD;

		pm->s_integral += (pm->lu_X[1] - pm->s_integral) * pm->s_gain_I;
		iSP += pm->s_integral;

		pm->i_set_point_D = 0.f;
		pm->i_set_point_Q = iSP;
	}
}

static void
pm_position_control(pmc_t *pm)
{
/*	float		dX, dY, eP;

	pm_rotf(pm->p_set_point, pm->p_set_point_s * pm->dT, pm->p_set_point);

	if (pm->lu_X[2] < 0.f) {

		if (pm->lu_X[3] < 0.f) {

			if (pm->lu_temp[0] >= 0.f)
				pm->lu_revol += 1;
		}
		else {
			if (pm->lu_temp[0] < 0.f)
				pm->lu_revol -= 1;
		}
	}

	if (pm->p_set_point[0] < 0.f) {

		if (pm->p_set_point[1] < 0.f) {

			if (pm->lu_temp[1] >= 0.f)
				pm->p_set_point_revol += 1;
		}
		else {
			if (pm->lu_temp[1] < 0.f)
				pm->p_set_point_revol -= 1;
		}
	}

	pm->lu_temp[0] = pm->lu_X[3];
	pm->lu_temp[1] = pm->p_set_point[1];
*/
	/* Obtain discrepancy.
	 * */
/*	dX = pm->p_set_point[0] * pm->lu_X[2] +
		pm->p_set_point[1] * pm->lu_X[3];
	dY = pm->p_set_point[1] * pm->lu_X[2] -
		pm->p_set_point[0] * pm->lu_X[3];

	eP = atan2f(dY, dX);

	if (dY < 0.f) {

		if (pm->lu_X[3] < 0.f && pm->p_set_point[1] >= 0.f)
			eP += 2.f * M_PI_F;
	}
	else {
		if (pm->lu_X[3] >= 0.f && pm->p_set_point[1] < 0.f)
			eP -= 2.f * M_PI_F;
	}

	eP += (pm->p_set_point_revol - pm->lu_revol) * 2.f * M_PI_F;

	pm->s_set_point = pm->p_gain_P * eP;*/
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		A, B, U;

	A = pm->adjust_IA[1] * fb->current_A + pm->adjust_IA[0];
	B = pm->adjust_IB[1] * fb->current_B + pm->adjust_IB[0];

	pm->fb_iA = (A < - pm->fb_i_range) ? - pm->fb_i_range :
		(A > pm->fb_i_range) ? pm->fb_i_range : A;
	pm->fb_iB = (B < - pm->fb_i_range) ? - pm->fb_i_range :
		(B > pm->fb_i_range) ? pm->fb_i_range : B;

	U = pm->adjust_US[1] * fb->voltage_U + pm->adjust_US[0];
	pm->const_lpf_U += (U - pm->const_lpf_U) * pm->lpf_gain_U;

	pm->fb_uA = pm->adjust_UA[1] * fb->voltage_A + pm->adjust_UA[0];
	pm->fb_uB = pm->adjust_UB[1] * fb->voltage_B + pm->adjust_UB[0];
	pm->fb_uC = pm->adjust_UC[1] * fb->voltage_C + pm->adjust_UC[0];

	pm_FSM(pm);

	if (pm->lu_region != PM_LU_DISABLED) {

		pm_LU_update(pm);
		pm_current_control(pm);

		pm_LU_job(pm);

		if (pm->const_lpf_U < pm->fault_supply_voltage_low) {

			pm->error = PM_ERROR_SUPPLY_VOLTAGE_LOW;
			pm_fsm_req(pm, PM_STATE_HALT);
		}

		if (pm->const_lpf_U > pm->fault_supply_voltage_high) {

			pm->error = PM_ERROR_SUPPLY_VOLTAGE_HIGH;
			pm_fsm_req(pm, PM_STATE_HALT);
		}

		if (pm->b_LOOP != 0) {

			if (pm->b_LOOP == 2) {

				pm_position_control(pm);
			}

			pm_speed_control(pm);
		}
	}
}

