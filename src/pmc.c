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

#include "pmc.h"
#include "m.h"

void pmc_default(pmc_t *pm)
{
	pm->pwm_minimal_pulse = 8;
	pm->pwm_clean_zone = 0;

	pm->m_bitmask = 0
		| PMC_BIT_HIGH_FREQUENCY_INJECTION
		| PMC_BIT_FLUX_POLARITY_DETECTION
		| PMC_BIT_POSITION_CONTROL_LOOP;
	pm->m_state = PMC_STATE_IDLE;
	pm->m_phase = 0;
	pm->m_errno = 0;

	pm->T_drift = .1f;
	pm->T_hold = .5f;
	pm->T_sine = .2f;
	pm->T_measure = .2f;
	pm->T_end = .1f;

	pm->wave_i_hold = 1.f;
	pm->wave_i_sine = 1.f;
	pm->wave_i_offset_D = 0.f;
	pm->wave_i_offset_Q = 0.f;
	pm->wave_freq_sine_hz = pm->freq_hz / 12.f;
	pm->wave_gain_P = 1e-1f;
	pm->wave_gain_I = 1e-2f;

	pm->scal_A[0] = 0.f;
	pm->scal_A[1] = 1.332e-2f;// * 1.11f;
	pm->scal_B[0] = 0.f;
	pm->scal_B[1] = 1.332e-2f;// * 1.15f;
	pm->scal_U[0] = 0.f;
	pm->scal_U[1] = 6.592e-3f;

	pm->lu_gain_K[0] = .2f;
	pm->lu_gain_K[1] = .2f;
	pm->lu_gain_K[2] = .2f;
	pm->lu_gain_K[3] = 80.f;
	pm->lu_gain_K[4] = 2e-2f;
	pm->lu_gain_K[5] = 170.f;
	pm->lu_gain_K[6] = - 180.f;
	pm->lu_gain_K[7] = 4e-3f;
	pm->lu_low_threshold = 304.f;
	pm->lu_low_hysteresis = 147.f;

	pm->fault_iab_maximal = 20.f;
	pm->fault_residual_maximal = 0.f;

	pm->hf_freq_hz = pm->freq_hz / 12.f;
	pm->hf_swing_D = 1.f;

	pm->drift_AB_maximal = 2.f;
	pm->drift_Q_maximal = 2.f;

	pm->const_U = 5.f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;

	pm->i_maximal = 10.f;
	pm->i_power_consumption_maximal = 10.f;
	pm->i_power_regeneration_maximal = - 1.f;
	pm->i_slew_rate_D = 4e+3f;
	pm->i_slew_rate_Q = 4e+3f;
	pm->i_gain_P_D = 2e-1f;
	pm->i_gain_I_D = 3e-2f;
	pm->i_gain_P_Q = 2e-1f;
	pm->i_gain_I_Q = 3e-2f;

	pm->p_gain_D = 1e-2f;
	pm->p_gain_P = 1e-1f;

	pm->fi_gain[0] = .1f;
	pm->fi_gain[1] = .1f;
	pm->fi_gain[2] = .1f;
}

static void
pm_equation(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ, X4, X5, fluxD, fluxQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = pm->lu_X[4];
	X5 = pm->drift_Q;

	fluxD = pm->const_Ld * X[0] + pm->const_E;
	fluxQ = pm->const_Lq * X[1];

	D[0] = (uD - pm->const_R * X[0] + fluxQ * X4) * pm->const_Ld_inversed;
	D[1] = (uQ - pm->const_R * X[1] - fluxD * X4 + X5) * pm->const_Lq_inversed;
	D[2] = X4;
}

static void
pm_update(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		D1[3], D2[3], X2[4];
	float		dT;

	/* Second-order ODE solver.
	 * */

	pm_equation(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	rotatef(X2 + 2, D1[2] * dT, X + 2);

	pm_equation(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	rotatef(X + 2, (D1[2] + D2[2]) * dT, X + 2);
}

static void
luenberger_update(pmc_t *pm, float iA, float iB)
{
	float		*X = pm->lu_X;
	float		iX, iY, iD, iQ, C2;
	float		eD, eQ, eR, dR, temp;

	iA -= pm->drift_A;
	iB -= pm->drift_B;

	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	iD = X[2] * iX + X[3] * iY;
	iQ = X[2] * iY - X[3] * iX;

	/* Obtain residual.
	 * */
	eD = iD - X[0];
	eQ = iQ - X[1];

	pm->lu_residual_D = eD;
	pm->lu_residual_Q = eQ;
	pm->lu_residual_variance += (eD * eD + eQ * eQ - pm->lu_residual_variance)
		* pm->fi_gain[1];

	X[0] += pm->lu_gain_K[0] * eD;
	X[1] += pm->lu_gain_K[1] * eQ;

	if (pm->lu_region == PMC_LU_LOW_REGION) {

		if (pm->m_bitmask & PMC_BIT_FLUX_POLARITY_DETECTION) {

			C2 = pm->hf_CS[0] * pm->hf_CS[0] - pm->hf_CS[1] * pm->hf_CS[1];
			pm->lu_flux_polarity += (C2 * eQ - pm->lu_flux_polarity) * pm->fi_gain[2];
		}

		eR = pm->hf_CS[1] * eQ;
		dR = pm->lu_gain_K[2] * eR;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		rotatef(X + 2, dR, X + 2);
		X[4] += pm->lu_gain_K[3] * eR;
	}
	else {
		eD = (X[4] < 0.f) ? - eD : eD;
		dR = pm->lu_gain_K[4] * eD;
		dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
		rotatef(X + 2, dR, X + 2);
		X[4] += pm->lu_gain_K[5] * eD + pm->lu_gain_K[6] * eQ;
		pm->drift_Q += pm->lu_gain_K[7] * eQ;
	}

	temp = pm->freq_hz * (2.f * MPIF / 12.f);
	X[4] = (X[4] > temp) ? temp : (X[4] < - temp) ? - temp : X[4];

	pm_update(pm);

	if (pm->lu_region == PMC_LU_LOW_REGION) {

		temp = fabsf(pm->lu_X[4]) - pm->lu_low_hysteresis;

		if (temp > pm->lu_low_threshold)
			pm->lu_region = PMC_LU_HIGH_REGION;
	}
	else {
		temp = fabsf(pm->lu_X[4]) + pm->lu_low_hysteresis;

		if (temp < pm->lu_low_threshold)
			pm->lu_region = PMC_LU_LOW_REGION;
	}

	if (pm->lu_X[2] < 0.f) {

		if (pm->lu_X[3] < 0.f) {

			if (pm->wave_temp[0] >= 0.f)
				pm->lu_revol += 1;
		}
		else {
			if (pm->wave_temp[0] < 0.f)
				pm->lu_revol -= 1;
		}
	}

	pm->wave_temp[0] = pm->lu_X[3];
}

static void
vsi_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, Q;
	float		uMIN, uMAX;
	int		xA, xB, xC, temp;

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

	/* Always clamp to GND.
	 * */
	Q = 0.f - uMIN;

	uA += Q;
	uB += Q;
	uC += Q;

	xA = (int) (pm->pwm_resolution * uA + .5f);
	xB = (int) (pm->pwm_resolution * uB + .5f);
	xC = (int) (pm->pwm_resolution * uC + .5f);

	temp = pm->pwm_resolution - (pm->pwm_minimal_pulse + pm->pwm_clean_zone);
	xA = (xA < pm->pwm_minimal_pulse) ? 0 : (xA > temp) ? pm->pwm_resolution : xA;
	xB = (xB < pm->pwm_minimal_pulse) ? 0 : (xB > temp) ? pm->pwm_resolution : xB;
	xC = (xC < pm->pwm_minimal_pulse) ? 0 : (xC > temp) ? pm->pwm_resolution : xC;

	/*
	 * */
	/*{
		float	iX, iY, iA, iB, iC, DT;

		iX = pm->lu_X[2] * pm->lu_X[0] - pm->lu_X[3] * pm->lu_X[1];
		iY = pm->lu_X[3] * pm->lu_X[0] + pm->lu_X[2] * pm->lu_X[1];

		iA = iX;
		iB = - .5f * iX + .8660254f * iY;
		iC = - .5f * iX - .8660254f * iY;

		DT = 90e-9 * pm->freq_hz * (pm->const_U - .9f);

		uA += (iA < 0.f) ? DT : - DT;
		uB += (iB < 0.f) ? DT : - DT;
		uC += (iC < 0.f) ? DT : - DT;
	}*/

	pm->pDC(xA, xB, xC);

	Q = .33333333f * (xA + xB + xC);
	uA = (xA - Q) * pm->const_U / pm->pwm_resolution;
	uB = (xB - Q) * pm->const_U / pm->pwm_resolution;

	pm->vsi_X = uA;
	pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
}

static void
i_control(pmc_t *pm)
{
	float		sp_D, sp_Q, eD, eQ, uX, uY;
	float		uD, uQ, wP, temp;

	sp_D = pm->i_set_point_D;
	sp_Q = pm->i_set_point_Q;

	/* Current constraints.
	 * */
	temp = pm->i_maximal;
	sp_D = (sp_D > temp) ? temp : (sp_D < - temp) ? - temp : sp_D;
	sp_Q = (sp_Q > temp) ? temp : (sp_Q < - temp) ? - temp : sp_Q;

	/* Power constraints.
	 * */
	uD = pm->wave_temp[2] * pm->vsi_X + pm->wave_temp[3] * pm->vsi_Y;
	uQ = pm->wave_temp[2] * pm->vsi_Y - pm->wave_temp[3] * pm->vsi_X;
	pm->i_power_watt = pm->lu_X[0] * uD + pm->lu_X[1] * uQ;
	wP = sp_D * uD + sp_Q * uQ;

	if (wP > pm->i_power_consumption_maximal) {

		temp = pm->i_power_consumption_maximal / wP;
		sp_D *= temp;
		sp_Q *= temp;
	}
	else if (wP < pm->i_power_regeneration_maximal) {

		temp = pm->i_power_regeneration_maximal / wP;
		sp_D *= temp;
		sp_Q *= temp;
	}

	/* Slew rate.
	 * */
	temp = pm->i_slew_rate_D * pm->dT;
	pm->i_track_point_D = (pm->i_track_point_D < sp_D - temp)
		? pm->i_track_point_D + temp : (pm->i_track_point_D > sp_D + temp)
		? pm->i_track_point_D - temp : sp_D;
	temp = pm->i_slew_rate_Q * pm->dT;
	pm->i_track_point_Q = (pm->i_track_point_Q < sp_Q - temp)
		? pm->i_track_point_Q + temp : (pm->i_track_point_Q > sp_Q + temp)
		? pm->i_track_point_Q - temp : sp_Q;

	/* Obtain residual.
	 * */
	eD = pm->i_track_point_D - pm->lu_X[0];
	eQ = pm->i_track_point_Q - pm->lu_X[1];

	if (pm->m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION
			&& pm->lu_region == PMC_LU_LOW_REGION)
		eD = (fabsf(eD) < pm->hf_swing_D) ? 0.f : eD;

	/* PI controller.
	 * */
	uD = pm->i_gain_P_D * eD;
	uQ = pm->i_gain_P_Q * eQ;

	temp = .667f * pm->const_U;

	pm->i_integral_D += pm->i_gain_I_D * eD;
	pm->i_integral_D = (pm->i_integral_D > temp) ? temp :
		(pm->i_integral_D < - temp) ? - temp : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I_Q * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > temp) ? temp :
		(pm->i_integral_Q < - temp) ? - temp : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	/* High Frequency Injection.
	 * */
	if (pm->m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION
			&& pm->lu_region == PMC_LU_LOW_REGION) {

		temp = 2.f * MPIF * pm->const_Ld * pm->hf_freq_hz;
		uD += pm->hf_CS[0] * pm->hf_swing_D * temp;
		rotatef(pm->hf_CS, 2.f * MPIF * pm->hf_freq_hz * pm->dT, pm->hf_CS);
	}

	uX = pm->lu_X[2] * uD - pm->lu_X[3] * uQ;
	uY = pm->lu_X[3] * uD + pm->lu_X[2] * uQ;

	vsi_control(pm, uX * pm->const_U_inversed, uY * pm->const_U_inversed);

	pm->wave_temp[2] = pm->lu_X[2];
	pm->wave_temp[3] = pm->lu_X[3];
}

static void
p_control(pmc_t *pm)
{
	float		dX, dY, eP, iSP;

	rotatef(pm->p_set_point_x, pm->p_set_point_w * pm->dT, pm->p_set_point_x);

	if (pm->p_set_point_x[0] < 0.f) {

		if (pm->p_set_point_x[1] < 0.f) {

			if (pm->wave_temp[1] >= 0.f)
				pm->p_set_point_revol += 1;
		}
		else {
			if (pm->wave_temp[1] < 0.f)
				pm->p_set_point_revol -= 1;
		}
	}

	pm->wave_temp[1] = pm->p_set_point_x[1];

	dX = pm->p_set_point_x[0] * pm->lu_X[2] + pm->p_set_point_x[1] * pm->lu_X[3];
	dY = pm->p_set_point_x[1] * pm->lu_X[2] - pm->p_set_point_x[0] * pm->lu_X[3];

	eP = arctanf(dY, dX);

	if (dY < 0.) {

		if (pm->lu_X[3] < 0. && pm->p_set_point_x[1] >= 0.)
			eP += 2.f * MPIF;
	}
	else {
		if (pm->lu_X[3] >= 0. && pm->p_set_point_x[1] < 0.)
			eP -= 2.f * MPIF;
	}

	eP += (pm->p_set_point_revol - pm->lu_revol) * 2.f * MPIF;

	/* PD controller.
	 * */
	iSP = pm->p_gain_P * eP + pm->p_gain_D * (pm->p_set_point_w - pm->lu_X[4]);

	pm->i_set_point_Q = iSP;
}

static void
pm_FSM(pmc_t *pm, float iA, float iB)
{
	float			uX, uY, temp;

	switch (pm->m_state) {

		case PMC_STATE_IDLE:
			break;

		case PMC_STATE_ZERO_DRIFT:

			if (pm->m_phase == 0) {

				pm->pZ(7);

				pm->wave_temp[0] = 0.f;
				pm->wave_temp[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_drift;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->wave_temp[0] += - iA;
				pm->wave_temp[1] += - iB;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					/* Zero Drift.
					 * */
					pm->scal_A[0] += pm->wave_temp[0] / pm->t_end;
					pm->scal_B[0] += pm->wave_temp[1] / pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;

					if (fabsf(pm->scal_A[0]) > pm->fault_iab_maximal)
						pm->m_errno = PMC_ERROR_CURRENT_SENSOR;

					if (fabsf(pm->scal_B[0]) > pm->fault_iab_maximal)
						pm->m_errno = PMC_ERROR_CURRENT_SENSOR;
				}
			}
			break;

		case PMC_STATE_WAVE_HOLD:

			if (pm->m_phase == 0) {

				pm->pZ(0);

				pm->wave_temp[0] = 0.f;
				pm->wave_temp[1] = 0.f;
				pm->wave_temp[2] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_hold;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->wave_temp[2] += pm->vsi_X - pm->wave_i_hold * pm->const_R;

				temp = pm->wave_i_hold - iA;
				pm->wave_temp[0] += pm->wave_gain_I * temp;
				uX = pm->wave_gain_P * temp + pm->wave_temp[0];

				temp = 0.f - .57735027f * iA - 1.1547005f * iB;
				pm->wave_temp[1] += pm->wave_gain_I * temp;
				uY = pm->wave_gain_P * temp + pm->wave_temp[1];

				temp = .667f * pm->const_U;

				if (fabsf(uX) > temp || fabsf(uY) > temp) {

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
					pm->m_errno = PMC_ERROR_OPEN_CIRCUIT;
				}

				vsi_control(pm, uX * pm->const_U_inversed,
						uY * pm->const_U_inversed);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					if (pm->m_phase == 1) {

						pm->wave_temp[2] = 0.f;

						pm->t_value = 0;
						pm->t_end = pm->freq_hz * pm->T_measure;

						pm->m_phase++;
					}
					else {
						pm->wave_temp[2] /= pm->t_end * pm->wave_i_hold;

						pm->m_state = PMC_STATE_END;
						pm->m_phase = 0;
					}
				}
			}
			break;

		case PMC_STATE_WAVE_SINE:

			if (pm->m_phase == 0) {

				pm->pZ(0);

				pm->lu_X[2] = 1.f;
				pm->lu_X[3] = 0.f;
				pm->lu_X[4] = 2.f * MPIF * pm->wave_freq_sine_hz / pm->freq_hz;

				pm->wave_DFT[0] = 0.f;
				pm->wave_DFT[1] = 0.f;
				pm->wave_DFT[2] = 0.f;
				pm->wave_DFT[3] = 0.f;
				pm->wave_DFT[4] = 0.f;
				pm->wave_DFT[5] = 0.f;
				pm->wave_DFT[6] = 0.f;
				pm->wave_DFT[7] = 0.f;

				temp = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
				temp = 2.f * MPIF * temp * pm->wave_freq_sine_hz;
				temp = pm->const_R * pm->const_R + temp * temp;
				pm->wave_temp[0] = pm->wave_i_sine * sqrtf(temp);
				pm->wave_temp[2] = pm->wave_i_offset_D * pm->const_R;
				pm->wave_temp[3] = pm->wave_i_offset_Q * pm->const_R;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_sine;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->lu_X[0] = iA;
				pm->lu_X[1] = .57735027f * iA + 1.1547005f * iB;

				pm->wave_DFT[0] += pm->lu_X[0] * pm->lu_X[2];
				pm->wave_DFT[1] += pm->lu_X[0] * pm->lu_X[3];
				pm->wave_DFT[2] += pm->vsi_X * pm->lu_X[2];
				pm->wave_DFT[3] += pm->vsi_X * pm->lu_X[3];
				pm->wave_DFT[4] += pm->lu_X[1] * pm->lu_X[2];
				pm->wave_DFT[5] += pm->lu_X[1] * pm->lu_X[3];
				pm->wave_DFT[6] += pm->vsi_Y * pm->lu_X[2];
				pm->wave_DFT[7] += pm->vsi_Y * pm->lu_X[3];

				uX = pm->wave_temp[2] + pm->wave_temp[0] * pm->lu_X[2];
				uY = pm->wave_temp[3] + pm->wave_temp[0] * pm->lu_X[3];

				vsi_control(pm, uX * pm->const_U_inversed,
						uY * pm->const_U_inversed);

				rotatef(pm->lu_X + 2, pm->lu_X[4], pm->lu_X + 2);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_CALIBRATION:

			if (pm->m_phase == 0) {

				pm->pZ(4);

				pm->wave_temp[0] = 0.f;
				pm->wave_temp[1] = 0.f;
				pm->wave_temp[2] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_measure;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->wave_temp[1] += iA;
				pm->wave_temp[2] += iB;

				temp = pm->wave_i_hold - iA;
				pm->wave_temp[0] += pm->wave_gain_I * temp;
				uX = pm->wave_gain_P * temp + pm->wave_temp[0];

				temp = .667f * pm->const_U;

				if (fabsf(uX) > temp) {

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
					pm->m_errno = PMC_ERROR_OPEN_CIRCUIT;
				}

				vsi_control(pm, uX * pm->const_U_inversed, 0.f);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->wave_temp[1] /= pm->t_end;
					pm->wave_temp[2] /= pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_START:

			if (pm->m_errno != PMC_OK)
				break;

			pm->lu_region = PMC_LU_LOW_REGION;
			pm->pZ(0);

			pm->lu_X[0] = 0.f;
			pm->lu_X[1] = 0.f;
			pm->lu_X[2] = 1.f;
			pm->lu_X[3] = 0.f;
			pm->lu_X[4] = 0.f;
			pm->lu_revol = 0;
			pm->lu_flux_polarity = 0.f;
			pm->lu_residual_variance = 0.f;

			pm->drift_A = 0.f;
			pm->drift_B = 0.f;
			pm->drift_Q = 0.f;

			pm->hf_CS[0] = 1.;
			pm->hf_CS[1] = 0.;

			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 0.f;
			pm->i_track_point_D = 0.f;
			pm->i_track_point_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			pm->p_set_point_x[0] = 1.f;
			pm->p_set_point_x[1] = 0.f;
			pm->p_set_point_revol = 0;
			pm->p_set_point_w = 0.f;

			pm->m_state = PMC_STATE_IDLE;
			pm->m_phase = 0;
			break;

		case PMC_STATE_STOP:

			if (pm->m_phase == 0) {

				pm->i_set_point_D = 0.f;
				pm->i_set_point_Q = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_end;

				pm->m_phase++;
			}
			else {
				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->lu_region = PMC_LU_DISABLED;
					pm->pZ(7);

					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_END:

			if (pm->m_phase == 0) {

				pm->pDC(0, 0, 0);

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_end;

				pm->m_phase++;
			}
			else {
				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->lu_region = PMC_LU_DISABLED;
					pm->pZ(7);

					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}
			break;

		default:
			pm->m_state = PMC_STATE_END;
			pm->m_phase = 0;
			break;
	}
}

static void
pm_tail(pmc_t *pm)
{
	pm->drift_A = (pm->drift_A < - pm->drift_AB_maximal) ? - pm->drift_AB_maximal
		: (pm->drift_A > pm->drift_AB_maximal) ? pm->drift_AB_maximal : pm->drift_A;
	pm->drift_B = (pm->drift_B < - pm->drift_AB_maximal) ? - pm->drift_AB_maximal
		: (pm->drift_B > pm->drift_AB_maximal) ? pm->drift_AB_maximal : pm->drift_B;
	pm->drift_Q = (pm->drift_Q < - pm->drift_Q_maximal) ? - pm->drift_Q_maximal
		: (pm->drift_Q > pm->drift_Q_maximal) ? pm->drift_Q_maximal : pm->drift_Q;
}

void pmc_feedback(pmc_t *pm, int xA, int xB)
{
	float		iA, iB;

	iA = pm->scal_A[1] * (xA - 2048) + pm->scal_A[0];
	iB = pm->scal_B[1] * (xB - 2048) + pm->scal_B[0];

	/* Overcurrent protection.
	 * */
	if (fabsf(iA) > pm->fault_iab_maximal || fabsf(iB) > pm->fault_iab_maximal) {

		pm->m_state = PMC_STATE_END;
		pm->m_phase = 0;
		pm->m_errno = PMC_ERROR_OVERCURRENT;
	}

	pm_FSM(pm, iA, iB);

	if (pm->lu_region != PMC_LU_DISABLED) {

		luenberger_update(pm, iA, iB);
		i_control(pm);
		(pm->m_bitmask & PMC_BIT_POSITION_CONTROL_LOOP) ? p_control(pm) : 0 ;
		pm_tail(pm);
	}
}

void pmc_voltage(pmc_t *pm, int xU)
{
	float		uS;

	uS = pm->scal_U[1] * xU + pm->scal_U[0];

	pm->const_U += (uS - pm->const_U) * pm->fi_gain[0];
	pm->const_U_inversed = 1.f / pm->const_U;
}

void pmc_request(pmc_t *pm, int req)
{
	if (pm->m_state != PMC_STATE_IDLE)
		return ;

	if (pm->lu_region == PMC_LU_DISABLED) {

		if (req == PMC_STATE_ZERO_DRIFT
			|| req == PMC_STATE_WAVE_HOLD
			|| req == PMC_STATE_WAVE_SINE
			|| req == PMC_STATE_CALIBRATION
			|| req == PMC_STATE_START) {

			pm->m_state = req;
		}
	}
	else {
		if (req == PMC_STATE_STOP) {

			pm->m_state = req;
		}
	}
}

static void
pm_eigenvalues(float X, float Y, float XY, float *DQA)
{
	float		B, D, la1, la2;

	B = X + Y;
	D = B * B - 4.f * (X * Y - XY * XY);

	if (D > 0.f) {

		D = sqrtf(D);
		la1 = (B - D) * .5f;
		la2 = (B + D) * .5f;

		B = Y - la1;
		D = sqrtf(B * B + XY * XY);
		B /= D;
		D = - XY / D;
		B = arctanf(D, B) * 180.f / MPIF;

		*DQA++ = la1;
		*DQA++ = la2;
		*DQA = B;
	}
}

void pmc_impedance(const float *DFT, float hz, float *IMP)
{
	float			Dx, Dy, w;
	float			Lx, Ly, Lm, Rx, Ry, Rm;

	Dx = DFT[0] * DFT[0] + DFT[1] * DFT[1];
	Dy = DFT[4] * DFT[4] + DFT[5] * DFT[5];
	w = 2.f * MPIF * hz;

	/* Inductance on XY axes.
	 * */
	Lx = (DFT[2] * DFT[1] - DFT[3] * DFT[0]) / (Dx * w);
	Ly = (DFT[6] * DFT[5] - DFT[7] * DFT[4]) / (Dy * w);
	Lm = (DFT[6] * DFT[1] - DFT[7] * DFT[0]) / (Dx * w);
	Lm += (DFT[2] * DFT[5] - DFT[3] * DFT[4]) / (Dy * w);
	Lm *= .5f;

	/* Resistance on XY axes.
	 * */
	Rx = (DFT[2] * DFT[0] + DFT[3] * DFT[1]) / Dx;
	Ry = (DFT[6] * DFT[4] + DFT[7] * DFT[5]) / Dy;
	Rm = (DFT[6] * DFT[0] + DFT[7] * DFT[1]) / Dx;
	Rm += (DFT[2] * DFT[4] + DFT[3] * DFT[5]) / Dy;
	Rm *= .5f;

	/* Get eigenvalues.
	 * */
	pm_eigenvalues(Lx, Ly, Lm, IMP + 0);
	pm_eigenvalues(Rx, Ry, Rm, IMP + 3);
}

const char *pmc_strerror(int errno)
{
	const char *list[] = {

		"No error",
		"Open circuit",
		"Overcurrent"
	};

	return (errno >= 0 && errno < 3) ? list[errno] : "";
}

