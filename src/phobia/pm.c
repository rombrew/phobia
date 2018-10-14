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

#include "pm.h"
#include "pm_m.h"

void pm_config_default(pmc_t *pm)
{
	pm->config_ABC = PM_ABC_THREE_PHASE;
	pm->config_LDQ = PM_LDQ_SATURATION_SALIENCY;

	pm->config_HALL = PM_HALL_DISABLED;
	pm->config_HFI = PM_HFI_DISABLED;
	pm->config_LOOP = PM_LOOP_SPEED_CONTROL;

	pm->tm_transient_skip = .05f;
	pm->tm_voltage_hold = .05f;
	pm->tm_current_hold = .5f;
	pm->tm_instant_probe = .01f;
	pm->tm_average_probe = .2f;
	pm->tm_startup = .1f;

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

	pm->probe_current_hold = 20.f;
	pm->probe_current_hold_Q = 0.f;
	pm->probe_current_sine = 5.f;
	pm->probe_freq_sine_hz = pm->freq_hz / 16.f;
	pm->probe_speed_low = 700.f;
	pm->probe_speed_ramp = 1700.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tolerance = 1.f;
	pm->fault_current_tolerance = 2.f;
	pm->fault_current_halt_level = 20.f;
	pm->fault_adjust_tolerance = 2E-2f;
	pm->fault_flux_residual_maximal = 90.f;

	pm->vsi_gain_LP = 1E-1f;
	pm->vsi_gain_LW = 5E-1f;

	pm->forced_hold_D = 20.f;
	pm->forced_accel = 5E+3f;

	pm->flux_gain_LP = 1E-1f;
	pm->flux_gain_DA = 5E-1f;
	pm->flux_gain_QA = 5E-1f;
	pm->flux_gain_DP = 5E-3f;
	pm->flux_gain_DS = 1E+1f;
	pm->flux_gain_QS = 1E+1f;
	pm->flux_gain_QZ = 5E-2f;
	pm->flux_bemf_low_unlock = .1f;
	pm->flux_bemf_low_lock = .2f;
	pm->flux_bemf_high = 1.f;

	pm->hfi_freq_hz = pm->freq_hz / 12.f;
	pm->hfi_swing_D = 2.f;
	pm->hfi_gain_P = 3E-1f;
	pm->hfi_gain_S = 7E+1f;
	pm->hfi_gain_F = 1E-3f;

	pm->const_gain_LP = 1E-1f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;
	pm->const_J = 0.f;

	pm->i_maximal = pm->fb_current_clamp;
	pm->i_watt_consumption_maximal = 3750.f;
	pm->i_watt_regeneration_maximal = -1000.f;
	pm->i_gain_PD = 5E-2f;
	pm->i_gain_ID = 5E-3f;
	pm->i_gain_PQ = 5E-2f;
	pm->i_gain_IQ = 5E-3f;

	pm->s_maximal = pm->freq_hz * (2.f * M_PI_F / 12.f);
	pm->s_accel = 5E+6f;
	pm->s_gain_P = 2E-2f;
	pm->s_gain_I = 1E-4f;

	pm->p_gain_P = 50.f;
	pm->p_gain_I = 0.f;
}

void pm_config_tune_current_loop(pmc_t *pm)
{
	pm->i_gain_PD = .5f * pm->const_Ld * pm->freq_hz - pm->const_R;
	pm->i_gain_ID = 2E-2f * pm->const_Ld * pm->freq_hz;
	pm->i_gain_PQ = .5f * pm->const_Lq * pm->freq_hz - pm->const_R;
	pm->i_gain_IQ = 2E-2f * pm->const_Lq * pm->freq_hz;
}

void pm_config_tune_flux_observer(pmc_t *pm)
{
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
pm_get_clamped_measure(pmc_t *pm, float X[5], float Z[2])
{
	float		iA, iB, iX, iY;

	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		iA = iX;
		iB = - .5f * iX + .8660254f * iY;

		iA = (iA < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iA > pm->fb_current_clamp) ? pm->fb_current_clamp : iA;
		iB = (iB < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iB > pm->fb_current_clamp) ? pm->fb_current_clamp : iB;

		iX = iA;
		iY = .57735027f * iA + 1.1547005f * iB;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		iX = (iX < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iX > pm->fb_current_clamp) ? pm->fb_current_clamp : iX;
		iY = (iY < - pm->fb_current_clamp) ? - pm->fb_current_clamp :
			(iY > pm->fb_current_clamp) ? pm->fb_current_clamp : iY;
	}

	Z[0] = X[2] * iX + X[3] * iY;
	Z[1] = X[2] * iY - X[3] * iX;
}

static void
pm_forced_update(pmc_t *pm)
{
	float		*X = pm->forced_X;
	float		iX, iY, wSP, dS, wMAX;

	iX = pm->flux_X[2] * pm->flux_X[0] - pm->flux_X[3] * pm->flux_X[1];
	iY = pm->flux_X[3] * pm->flux_X[0] + pm->flux_X[2] * pm->flux_X[1];

	X[0] = X[2] * iX + X[3] * iY;
	X[1] = X[2] * iY - X[3] * iX;

	dS = pm->forced_accel * pm->dT;
	wSP = pm->forced_setpoint;

	if (pm->const_E != 0.f) {

		wMAX = pm->flux_bemf_high / pm->const_E;
		wSP = (wSP < - wMAX) ? - wMAX : (wSP > wMAX) ? wMAX : wSP;
	}

	X[4] = (X[4] < wSP - dS) ? X[4] + dS : (X[4] > wSP + dS) ? X[4] - dS : wSP;

	pm_rotf(X + 2, X[4] * pm->dT, X + 2);
}

static void
pm_flux_update(pmc_t *pm)
{
	float		*X = pm->flux_X, Z[2];
	float		iD, iQ, eD, eQ, eR, dR, qS;

	iD = X[2] * pm->lu_fb_X + X[3] * pm->lu_fb_Y;
	iQ = X[2] * pm->lu_fb_Y - X[3] * pm->lu_fb_X;

	pm_get_clamped_measure(pm, X, Z);

	eD = iD - Z[0];
	eQ = iQ - Z[1];

	pm->flux_residual_D = eD;
	pm->flux_residual_Q = eQ;
	pm->flux_residual_lpf += (eD * eD + eQ * eQ - pm->flux_residual_lpf)
		* pm->flux_gain_LP;

	X[0] += pm->flux_gain_DA * eD;
	X[1] += pm->flux_gain_QA * eQ;

	if (pm->const_E != 0.f) {

		qS = (pm->lu_mode == PM_LU_FORCED) ? pm->forced_X[4] : X[4];
		eR = (qS < 0.f) ? - eD : eD;

		dR = pm->flux_gain_DP * eR;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		pm_rotf(X + 2, dR, X + 2);

		qS = pm_fabsf(X[4] * pm->const_E) / (.57735027f * pm->const_lpf_U);
		qS = (qS > 1.f) ? 1.f : qS;

		dR = pm->flux_gain_DS * qS * eR - pm->flux_gain_QS * (1.f - qS) * eQ;
		X[4] += dR;

		if (pm_fabsf(X[4] * pm->const_E) > pm->flux_bemf_high) {

			pm->flux_drift_Q += pm->flux_gain_QZ * (eR + eQ);
		}
		else {
			pm->flux_drift_Q = 0.f;
		}
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

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

	if (1) {

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
	/* TODO */
}

void pm_lu_get_current(pmc_t *pm)
{
	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		pm->lu_fb_X = pm->fb_current_A;
		pm->lu_fb_Y = .57735027f * pm->fb_current_A
			+ 1.1547005f * pm->fb_current_B;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		pm->lu_fb_X = pm->fb_current_A;
		pm->lu_fb_Y = pm->fb_current_B;
	}
}

void pm_lu_get_voltage(pmc_t *pm)
{
	float			uA, uB, uQ;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uQ = (1.f / 3.f) * (pm->fb_voltage_A + pm->fb_voltage_B + pm->fb_voltage_C);
		uA = pm->fb_voltage_A - uQ;
		uB = pm->fb_voltage_B - uQ;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = pm->fb_voltage_A - pm->fb_voltage_C;
		uB = pm->fb_voltage_B - pm->fb_voltage_C;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float		*X = pm->lu_X;

	if (pm->lu_mode == PM_LU_DETACHED) {

		pm->lu_fb_X = 0.f;
		pm->lu_fb_Y = 0.f;

		pm_lu_get_voltage(pm);
		pm_flux_update(pm);
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		pm_lu_get_current(pm);
		pm_flux_update(pm);
		pm_forced_update(pm);

		X[0] = pm->forced_X[0];
		X[1] = pm->forced_X[1];
		X[2] = pm->forced_X[2];
		X[3] = pm->forced_X[3];
		X[4] = pm->forced_X[4];

		if (pm_fabsf(X[4] * pm->const_E) > pm->flux_bemf_low_lock
				&& pm_fabsf(pm->flux_X[4] * pm->const_E) > pm->flux_bemf_low_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

		pm_lu_get_current(pm);
		pm_flux_update(pm);

		X[0] = pm->flux_X[0];
		X[1] = pm->flux_X[1];
		X[2] = pm->flux_X[2];
		X[3] = pm->flux_X[3];
		X[4] = pm->flux_X[4];

		if (pm_fabsf(X[4] * pm->const_E) < pm->flux_bemf_low_unlock) {

			if (pm->config_HALL != PM_HALL_DISABLED) {

				pm->lu_mode = PM_LU_SENSORED_HALL;
			}
			else if (pm->config_HFI != PM_HFI_DISABLED) {

				pm->lu_mode = PM_LU_ESTIMATE_HFI;

				pm->hfi_X[0] = X[0];
				pm->hfi_X[1] = X[1];
				pm->hfi_X[2] = X[2];
				pm->hfi_X[3] = X[3];
				pm->hfi_X[4] = X[4];
			}
			else {
				pm->lu_mode = PM_LU_FORCED;

				pm->forced_X[0] = X[0];
				pm->forced_X[1] = X[1];
				pm->forced_X[2] = X[2];
				pm->forced_X[3] = X[3];
				pm->forced_X[4] = X[4];
			}
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		pm_lu_get_current(pm);
		pm_hfi_update(pm);

		X[0] = pm->hfi_X[0];
		X[1] = pm->hfi_X[1];
		X[2] = pm->hfi_X[2];
		X[3] = pm->hfi_X[3];
		X[4] = pm->hfi_X[4];

		if (pm_fabsf(X[4] * pm->const_E) > pm->flux_bemf_low_lock) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;

			pm->flux_X[0] = X[0];
			pm->flux_X[1] = X[1];
			pm->flux_X[2] = X[2];
			pm->flux_X[3] = X[3];
			pm->flux_X[4] = X[4];
		}
	}
	else if (pm->lu_mode == PM_LU_SENSORED_HALL) {

		pm_hall_update(pm);
	}
}

static void
pm_lu_validate(pmc_t *pm)
{
	float		*X = pm->lu_X;

	if (pm->flux_residual_lpf > pm->fault_flux_residual_maximal) {

		pm->fail_reason = PM_ERROR_LU_RESIDUAL_UNSTABLE;
		pm_fsm_req(pm, PM_STATE_HALT);
	}

	if (X[1] == X[1]) ;
	else {
		pm->fail_reason = PM_ERROR_LU_INVALID_OPERATION;
		pm_fsm_req(pm, PM_STATE_HALT);
	}
}

static int
pm_get_SG(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		iX, iY, iA, iB, iC;
	int		xSG = 0;

	if (pm->lu_mode != PM_LU_DISABLED) {

		iX = X[2] * X[0] - X[3] * X[1];
		iY = X[3] * X[0] + X[2] * X[1];

		if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

			iA = iX;
			iB = - .5f * iX + .8660254f * iY;
			iC = - .5f * iX - .8660254f * iY;
		}
		else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

			iA = iX;
			iB = iY;
			iC = - (iX + iY);
		}
	}
	else {
		iA = pm->fb_current_A;
		iB = pm->fb_current_B;
		iC = - (pm->fb_current_A + pm->fb_current_B);
	}

	if (iA < 0.f) xSG |= 1;
	if (iB < 0.f) xSG |= 2;
	if (iC < 0.f) xSG |= 4;

	return xSG;
}

void pm_voltage_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC;
	float		uMIN, uMAX, uQ;
	int		xA, xB, xC;
	int		xMIN, xMAX, xSG;

	uX /= pm->const_lpf_U;
	uY /= pm->const_lpf_U;

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uA = uX;
		uB = - .5f * uX + .8660254f * uY;
		uC = - .5f * uX - .8660254f * uY;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = uX;
		uB = uY;
		uC = 0.f;
	}

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

	uQ = uMAX - uMIN;

	if (uQ > 1.f) {

		uQ = 1.f / uQ;
		uA *= uQ;
		uB *= uQ;
		uC *= uQ;
		uMIN *= uQ;
	}

	if (pm->vsi_clamp_to_null != 0) {

		uQ = 0.f - uMIN;
	}
	else {
		uQ = .5f;
	}

	uA += uQ;
	uB += uQ;
	uC += uQ;

	xA = (int) (pm->pwm_resolution * uA);
	xB = (int) (pm->pwm_resolution * uB);
	xC = (int) (pm->pwm_resolution * uC);

	xMIN = pm->pwm_minimal_pulse;
	xMAX = pm->pwm_resolution - pm->pwm_silence_gap;

	xA = (xA < xMIN) ? 0 : (xA > xMAX) ? pm->pwm_resolution : xA;
	xB = (xB < xMIN) ? 0 : (xB > xMAX) ? pm->pwm_resolution : xB;

	xMAX = pm->pwm_resolution - pm->pwm_minimal_pulse;
	xC = (xC < xMIN) ? 0 : (xC > xMAX) ? pm->pwm_resolution : xC;

	pm->proc_set_DC(xA, xB, xC);

	xMAX = pm->pwm_compensation;
	xMIN = 1;

	xSG = pm_get_SG(pm);

	if (xA > 0 && xA < pm->pwm_resolution) {

		if (xSG & 1)

			xA += - xMIN;
		else
			xA += xMIN - xMAX;
	}

	if (xB > 0 && xB < pm->pwm_resolution) {

		if (xSG & 2)

			xB += - xMIN;
		else
			xB += xMIN - xMAX;
	}

	if (xC > 0 && xC < pm->pwm_resolution) {

		if (xSG & 4)

			xC += - xMIN;
		else
			xC += xMIN - xMAX;
	}

	if (PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) {

		uQ = (1.f / 3.f) * (xA + xB + xC);
		uA = (xA - uQ) * pm->const_lpf_U / pm->pwm_resolution;
		uB = (xB - uQ) * pm->const_lpf_U / pm->pwm_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else if (PM_CONFIG_ABC(pm) == PM_ABC_TWO_PHASE) {

		uA = (xA - xC) * pm->const_lpf_U / pm->pwm_resolution;
		uB = (xB - xC) * pm->const_lpf_U / pm->pwm_resolution;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}
}

static void
pm_current_control(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		sD, sQ, eD, eQ;
	float		uD, uQ, uX, uY, wP;
	float		iMAX, uMAX, temp;

	if (pm->lu_mode == PM_LU_FORCED) {

		sD = pm->forced_hold_D;
		sQ = 0.f;
	}
	else {
		sD = pm->i_setpoint_D;
		sQ = pm->i_setpoint_Q;
	}

	iMAX = (pm->i_maximal < pm->i_derated) ? pm->i_maximal : pm->i_derated;

	sD = (sD > iMAX) ? iMAX : (sD < - iMAX) ? - iMAX : sD;
	sQ = (sQ > iMAX) ? iMAX : (sQ < - iMAX) ? - iMAX : sQ;

	wP = 1.5f * (sD * pm->vsi_lpf_D + sQ * pm->vsi_lpf_Q);

	if (wP > pm->i_watt_consumption_maximal) {

		temp = pm->i_watt_consumption_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}
	else if (wP < pm->i_watt_regeneration_maximal) {

		temp = pm->i_watt_regeneration_maximal / wP;
		sD *= temp;
		sQ *= temp;
	}

	/* Obtain discrepancy.
	 * */
	eD = sD - X[0];
	eQ = sQ - X[1];

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

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

	if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

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
	float		iSP, wSP, D, dS;

	wSP = pm->s_setpoint;
	wSP = (wSP < - pm->s_maximal) ? - pm->s_maximal :
		(wSP > pm->s_maximal) ? pm->s_maximal : wSP;

	if (1) {

		dS = pm->s_accel * pm->dT;
		pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
			: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;

		if (pm->lu_mode == PM_LU_FORCED) {

			pm->forced_setpoint = pm->s_track;
		}
		else {
			/* Obtain discrepancy.
			 * */
			D = pm->s_track - X[4];

			iSP = pm->s_gain_P * D;

			pm->s_integral += (X[1] - pm->s_integral) * pm->s_gain_I;
			iSP += pm->s_integral;

			pm->i_setpoint_D = 0.f;
			pm->i_setpoint_Q = iSP;
		}
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

		if (pm->lu_mode != PM_LU_DETACHED) {

			pm_current_control(pm);

			if (pm->config_LOOP == PM_LOOP_SPEED_CONTROL) {

				pm_speed_control(pm);
			}
		}

		pm_lu_validate(pm);
	}
}

