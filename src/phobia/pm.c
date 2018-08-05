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

#include "pm.h"
#include "pm_m.h"

void pm_default(pmc_t *pm)
{
	pm->pwm_MP = 8;

	pm->config_ABC = PM_ABC_THREE_PHASE;
	pm->config_HALL = PM_HALL_DISABLED;
	pm->config_HFI = PM_HFI_DISABLED;
	pm->config_LOOP = PM_LOOP_SPEED_CONTROL;

	pm->tm_skip = 50E-3f;
	pm->tm_probe = 200E-3f;
	pm->tm_hold = 500E-3;

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

	pm->fb_current_clamp = 50.f;

	pm->probe_current_hold = 10.f;
	pm->probe_current_hold_Q = 0.f;
	pm->probe_current_sine = 10.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 16.f;
	pm->probe_speed_ramp = 1700.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_zero_drift_maximal = 1.f;
	pm->fault_voltage_tolerance = 1.f;
	pm->fault_current_tolerance = 1.f;
	pm->fault_adjust_tolerance = 3E-2f;
	pm->fault_flux_residual_maximal = 90.f;
	pm->fault_flux_drift_Q_maximal = 7.f;
	pm->fault_supply_voltage_low = 5.f;
	pm->fault_supply_voltage_high = 55.f;

	pm->vsi_gain_LP = 1E-1f;
	pm->vsi_gain_LW = 5E-1f;

	pm->forced_hold_D = 10.f;
	pm->forced_accel = 1E+3f;
	pm->forced_tolerance = 1.f;

	pm->flux_gain_LP = 1E-1f;
	pm->flux_gain_DA = 5E-1f;
	pm->flux_gain_QA = 5E-1f;
	pm->flux_gain_DP = 5E-3f;
	pm->flux_gain_DS = 5E+0f;
	pm->flux_gain_QS = 1E+0f;
	pm->flux_gain_QZ = 5E-3f;
	pm->flux_BEMF_low = 200E-3f;
	pm->flux_BEMF_high = 400E-3f;

	pm->hfi_freq_hz = pm->freq_hz / 12.f;
	pm->hfi_swing_D = 1.f;
	pm->hfi_gain_P = 5E-2f;
	pm->hfi_gain_S = 7E+1f;
	pm->hfi_gain_F = 2E-3f;

	pm->const_gain_LP = 1E-1f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->i_maximal = 30.f;
	pm->i_power_consumption_maximal = 2050.f;
	pm->i_power_regeneration_maximal = -1.f;
	pm->i_slew_rate_D = 5E+4f;
	pm->i_slew_rate_Q = 5E+4f;
	pm->i_gain_PD = 2E-1f;
	pm->i_gain_ID = 3E-2f;
	pm->i_gain_PQ = 2E-1f;
	pm->i_gain_IQ = 3E-2f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 16.f);
	pm->s_accel = 5E+6f;
	pm->s_gain_P = 2E-2f;
	pm->s_gain_I = 1E-3f;

	pm->p_gain_P = 50.f;
	pm->p_gain_I = 0.f;
}

void pm_tune_current_loop(pmc_t *pm)
{
	pm->i_slew_rate_D = .2f * pm->const_lpf_U / pm->const_Ld;
	pm->i_slew_rate_Q = .2f * pm->const_lpf_U / pm->const_Lq;
	pm->i_gain_PD = (.5f * pm->const_Ld * pm->freq_hz - pm->const_R);
	pm->i_gain_ID = 5E-2f * pm->const_Ld * pm->freq_hz;
	pm->i_gain_PQ = (.5f * pm->const_Lq * pm->freq_hz - pm->const_R);
	pm->i_gain_IQ = 5E-2f * pm->const_Lq * pm->freq_hz;
}

static void
pm_equation_3(pmc_t *pm, float D[3], const float X[5])
{
	float		uD, uQ, X4, X5, R1, E1, fluxD, fluxQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = X[4];
	X5 = pm->flux_drift_Q;
	R1 = pm->const_R;
	E1 = pm->const_E;

	fluxD = pm->const_Ld * X[0] + E1;
	fluxQ = pm->const_Lq * X[1];

	D[0] = (uD - R1 * X[0] + fluxQ * X4) / pm->const_Ld;
	D[1] = (uQ - R1 * X[1] - fluxD * X4 + X5) / pm->const_Lq;
	D[2] = X4;
}

static void
pm_solve_2(pmc_t *pm, float X[5])
{
	float		D1[3], D2[3], X2[5];
	float		dT = pm->dT;

	/* Second-order ODE solver.
	 * */

	pm_equation_3(pm, D1, X);

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	pm_rotf(X2 + 2, D1[2] * dT, X + 2);
	X2[4] = X[4];

	pm_equation_3(pm, D2, X2);

	dT *= .5f;
	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	pm_rotf(X + 2, (D1[2] + D2[2]) * dT, X + 2);
}

static void
pm_clamped_measure(pmc_t *pm, float X[5], float Z[2])
{
	float		iA, iB, iX, iY;

	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	iA = iX;
	iB = - .5f * iX + .8660254f * iY;

	iA = (iA < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(iA > pm->fb_current_clamp) ? pm->fb_current_clamp : iA;
	iB = (iB < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(iB > pm->fb_current_clamp) ? pm->fb_current_clamp : iB;

	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	Z[0] = X[2] * iX + X[3] * iY;
	Z[1] = X[2] * iY - X[3] * iX;
}

static void
pm_flux_update(pmc_t *pm)
{
	float		*X = pm->flux_X, Z[2];
	float		iD, iQ, eD, eQ, eR, dR;

	iD = X[2] * pm->lu_fb_X + X[3] * pm->lu_fb_Y;
	iQ = X[2] * pm->lu_fb_Y - X[3] * pm->lu_fb_X;

	pm_clamped_measure(pm, X, Z);

	eD = iD - Z[0];
	eQ = iQ - Z[1];

	pm->flux_residual_D = eD;
	pm->flux_residual_Q = eQ;
	pm->flux_residual_lpf += (eD * eD + eQ * eQ - pm->flux_residual_lpf)
		* pm->flux_gain_LP;

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	if (pm->const_E != 0.f) {

		eR = (X[4] < 0.f) ? - eD : eD;
		dR = pm->flux_gain_DP * eR;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		pm_rotf(X + 2, dR, X + 2);

		dR = pm->flux_gain_DS * eR - pm->flux_gain_QS * eQ;
		X[4] += dR;

		if (pm_fabsf(X[4] * pm->const_E) > pm->flux_BEMF_high) {

			pm->flux_drift_Q += pm->flux_gain_QZ * eQ;
		}
		else {
			pm->flux_drift_Q = 0.f;
		}
	}
	else if (pm->lu_mode == PM_LU_OPEN_LOOP) {

		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		pm->flux_drift_Q += pm->flux_gain_QZ * eQ;
	}

	pm_solve_2(pm, X);
}

static void
pm_hfi_update(pmc_t *pm)
{
	float		*X = pm->hfi_X;
	float		iD, iQ, eD, eQ, eR, dR, C2;

	iD = X[2] * pm->lu_fb_X + X[3] * pm->lu_fb_Y;
	iQ = X[2] * pm->lu_fb_Y - X[3] * pm->lu_fb_X;

	eD = iD - X[0];
	eQ = iQ - X[1];

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	eR = pm->hfi_CS[1] * eQ;
	dR = pm->hfi_gain_P * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	pm_rotf(X + 2, dR, X + 2);

	X[4] += pm->hfi_gain_S * eR;

	if (pm->config_HFI == PM_HFI_ENABLED_WITH_FLUX_POLARITY_DETECTION) {

		C2 = pm->hfi_CS[0] * pm->hfi_CS[0] - pm->hfi_CS[1] * pm->hfi_CS[1];
		pm->hfi_flux_polarity += eD * C2 * pm->hfi_gain_F;

		if (pm->hfi_flux_polarity > 1.f) {

			X[2] = - X[2];
			X[3] = - X[3];

			pm->hfi_flux_polarity = 0.f;
		}
		else if (pm->hfi_flux_polarity < 0.f) {

			pm->hfi_flux_polarity = 0.f;
		}
	}

	pm_solve_2(pm, X);
}

static void
pm_hall_update(pmc_t *pm)
{
	/* TODO
	 * */
}

static void
pm_forced_update(pmc_t *pm)
{
	float		*X = pm->forced_X;
	float		flux_X, flux_Y, dS, bMAX;

	flux_X = pm->flux_X[2] * pm->flux_X[0] - pm->flux_X[3] * pm->flux_X[1];
	flux_Y = pm->flux_X[3] * pm->flux_X[0] + pm->flux_X[2] * pm->flux_X[1];

	X[0] = X[2] * flux_X + X[3] * flux_Y;
	X[1] = X[2] * flux_Y - X[3] * flux_X;

	dS = pm->forced_accel * pm->dT;
	bMAX = pm->flux_BEMF_high * 2.f - pm->flux_BEMF_low;

	if (pm->i_set_point_Q < - pm->forced_tolerance) {

		if (X[4] * pm->const_E > - bMAX) {

			X[4] += - dS;
		}
	}
	else if (pm->i_set_point_Q > pm->forced_tolerance) {

		if (X[4] * pm->const_E < bMAX) {

			X[4] += + dS;
		}
	}

	pm_rotf(X + 2, X[4] * pm->dT, X + 2);
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float		*X = pm->lu_X;

	pm->lu_fb_X = pm->fb_current_A;
	pm->lu_fb_Y = .57735027f * pm->fb_current_A
		+ 1.1547005f * pm->fb_current_B;

	if (pm->lu_mode == PM_LU_CLOSED_ESTIMATE_FLUX) {

		pm_flux_update(pm);

		X[0] = pm->flux_X[0];
		X[1] = pm->flux_X[1];
		X[2] = pm->flux_X[2];
		X[3] = pm->flux_X[3];
		X[4] = pm->flux_X[4];

		if (pm_fabsf(X[4] * pm->const_E) < pm->flux_BEMF_low) {

			if (pm->config_HALL != PM_HALL_DISABLED) {

				pm->lu_mode = PM_LU_CLOSED_SENSOR_HALL;
			}
			else if (pm->config_HFI != PM_HFI_DISABLED) {

				pm->lu_mode = PM_LU_CLOSED_ESTIMATE_HFI;

				pm->hfi_X[0] = X[0];
				pm->hfi_X[1] = X[1];
				pm->hfi_X[2] = X[2];
				pm->hfi_X[3] = X[3];
				pm->hfi_X[4] = X[4];
			}
			else {
				pm->lu_mode = PM_LU_OPEN_LOOP;

				pm->forced_X[0] = X[0];
				pm->forced_X[1] = X[1];
				pm->forced_X[2] = X[2];
				pm->forced_X[3] = X[3];
				pm->forced_X[4] = X[4];
			}
		}
	}
	else if (pm->lu_mode == PM_LU_CLOSED_ESTIMATE_HFI) {

		pm_hfi_update(pm);

		X[0] = pm->hfi_X[0];
		X[1] = pm->hfi_X[1];
		X[2] = pm->hfi_X[2];
		X[3] = pm->hfi_X[3];
		X[4] = pm->hfi_X[4];

		if (pm_fabsf(X[4] * pm->const_E) > pm->flux_BEMF_high) {

			pm->lu_mode = PM_LU_CLOSED_ESTIMATE_FLUX;

			pm->flux_X[0] = X[0];
			pm->flux_X[1] = X[1];
			pm->flux_X[2] = X[2];
			pm->flux_X[3] = X[3];
			pm->flux_X[4] = X[4];
		}
	}
	else if (pm->lu_mode == PM_LU_CLOSED_SENSOR_HALL) {

		pm_hall_update(pm);
	}
	else if (pm->lu_mode == PM_LU_OPEN_LOOP) {

		pm_flux_update(pm);
		pm_forced_update(pm);

		X[0] = pm->forced_X[0];
		X[1] = pm->forced_X[1];
		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		if (pm_fabsf(pm->flux_X[4] * pm->const_E) > pm->flux_BEMF_high) {

			pm->lu_mode = PM_LU_CLOSED_ESTIMATE_FLUX;
		}
	}
}

static void
pm_lu_validate(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		sMAX;

	if (pm->flux_residual_lpf > pm->fault_flux_residual_maximal) {

		pm->err_reason = PM_ERROR_LU_RESIDUAL_UNSTABLE;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	if (X[0] == X[0]) ;
	else {
		pm->err_reason = PM_ERROR_LU_INVALID_OPERATION;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	sMAX = pm->freq_hz * (2.f * M_PI_F / 12.f);

	if (pm_fabsf(X[4]) > sMAX) {

		pm->err_reason = PM_ERROR_LU_SPEED_HIGH;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	if (pm_fabsf(pm->flux_drift_Q) > pm->fault_flux_drift_Q_maximal) {

		pm->err_reason = PM_ERROR_LU_DRIFT_HIGH;
		pm_fsm_req(pm, PM_STATE_HALT);
	}
}

void pm_voltage_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC;
	float		uMIN, uMAX, Q;
	int		xA, xB, xC;
	int		xMIN, xMAX;

	uX /= pm->const_lpf_U;
	uY /= pm->const_lpf_U;

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

	xA = (int) (pm->pwm_R * uA);
	xB = (int) (pm->pwm_R * uB);
	xC = (int) (pm->pwm_R * uC);

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
}

static void
pm_current_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		sD, sQ, eD, eQ;
	float		uD, uQ, uX, uY, wP;
	float		iMAX, uMAX, temp;

	if (pm->lu_mode != PM_LU_OPEN_LOOP) {

		sD = pm->i_set_point_D;
		sQ = pm->i_set_point_Q;
	}
	else {
		sD = pm->forced_hold_D;
		sQ = 0.f;
	}

	iMAX = pm->i_maximal;

	sD = (sD > iMAX) ? iMAX : (sD < - iMAX) ? - iMAX : sD;
	sQ = (sQ > iMAX) ? iMAX : (sQ < - iMAX) ? - iMAX : sQ;

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
	eD = pm->i_track_point_D - X[0];
	eQ = pm->i_track_point_Q - X[1];

	if (pm->lu_mode == PM_LU_CLOSED_ESTIMATE_HFI) {

		temp = 2.f * M_PI_F * pm->hfi_freq_hz;
		pm_rotf(pm->hfi_CS, temp * pm->dT, pm->hfi_CS);

		eD += pm->hfi_CS[1] * pm->hfi_swing_D;
	}

	uD = pm->i_gain_PD * eD;
	uQ = pm->i_gain_PQ * eQ;

	uMAX = (2.f / 3.f) * pm->const_lpf_U;

	pm->i_integral_D += pm->i_gain_ID * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_IQ * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	if (pm->lu_mode == PM_LU_CLOSED_ESTIMATE_HFI) {

		uD += pm->hfi_CS[0] * pm->hfi_swing_D * temp * pm->const_Ld;
	}

	uX = X[2] * uD - X[3] * uQ;
	uY = X[3] * uD + X[2] * uQ;

	pm_voltage_control(pm, uX, uY);

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	pm->vsi_lpf_D += (uD - pm->vsi_lpf_D) * pm->vsi_gain_LP;
	pm->vsi_lpf_Q += (uQ - pm->vsi_lpf_Q) * pm->vsi_gain_LP;

	wP = 1.5f * (X[0] * pm->vsi_lpf_D + X[1] * pm->vsi_lpf_Q);
	pm->vsi_lpf_watt += (wP - pm->vsi_lpf_watt) * pm->vsi_gain_LW;
}

static void
pm_speed_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		iSP, wSP, eD, temp;

	wSP = pm->s_set_point;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	if (1) {

		temp = pm->s_accel * pm->dT;
		pm->s_track_point = (pm->s_track_point < wSP - temp)
			? pm->s_track_point + temp : (pm->s_track_point > wSP + temp)
			? pm->s_track_point - temp : wSP;

		/* Obtain discrepancy.
		 * */
		eD = pm->s_track_point - X[4];

		iSP = pm->s_gain_P * eD;

		pm->s_integral += (X[1] - pm->s_integral) * pm->s_gain_I;
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

	A = (A < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(A > pm->fb_current_clamp) ? pm->fb_current_clamp : A;
	B = (B < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
		(B > pm->fb_current_clamp) ? pm->fb_current_clamp : B;

	pm->fb_current_A = A;
	pm->fb_current_B = B;

	U = pm->adjust_US[1] * fb->voltage_U + pm->adjust_US[0];
	pm->const_lpf_U += (U - pm->const_lpf_U) * pm->const_gain_LP;

	pm->fb_voltage_A = pm->adjust_UA[1] * fb->voltage_A + pm->adjust_UA[0];
	pm->fb_voltage_B = pm->adjust_UB[1] * fb->voltage_B + pm->adjust_UB[0];
	pm->fb_voltage_C = pm->adjust_UC[1] * fb->voltage_C + pm->adjust_UC[0];

	pm_FSM(pm);

	if (pm->lu_mode != PM_LU_DISABLED) {

		pm_lu_FSM(pm);
		pm_current_control(pm);

		pm_lu_validate(pm);

		if (pm->const_lpf_U < pm->fault_supply_voltage_low) {

			pm->err_reason = PM_ERROR_SUPPLY_VOLTAGE_LOW;
			pm_fsm_req(pm, PM_STATE_HALT);
		}

		if (pm->const_lpf_U > pm->fault_supply_voltage_high) {

			pm->err_reason = PM_ERROR_SUPPLY_VOLTAGE_HIGH;
			pm_fsm_req(pm, PM_STATE_HALT);
		}

		if (pm->config_LOOP == PM_LOOP_SPEED_CONTROL) {

			pm_speed_control(pm);
		}
	}
}

