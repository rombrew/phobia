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

static void
lib_eigenvalues(float X, float Y, float XY, float *DQA)
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
		B = matan2f(D, B) * 180.f / MPIF;

		*DQA++ = la1;
		*DQA++ = la2;
		*DQA = B;
	}
}

void pmc_default(pmc_t *pm)
{
	pm->pwm_minimal_pulse = 8;
	pm->pwm_dead_time = 0;

	pm->m_bitmask = 0;
	pm->m_state = PMC_STATE_IDLE;
	pm->m_phase = 0;
	pm->m_errno = 0;

	pm->T_drift = .1f;
	pm->T_hold = .5f;
	pm->T_sine = .2f;
	pm->T_measure = .5f;
	pm->T_end = .1f;

	pm->wave_i_hold_X = 2.f;
	pm->wave_i_hold_Y = 0.f;
	pm->wave_i_sine = 1.f;
	pm->wave_freq_sine_hz = pm->freq_hz / 12.f;
	pm->wave_gain_P = 1E-2f;
	pm->wave_gain_I = 1E-3f;

	pm->scal_A[0] = 0.f;
	pm->scal_A[1] = 1.4648E-2f;
	pm->scal_B[0] = 0.f;
	pm->scal_B[1] = 1.4648E-2f;
	pm->scal_U[0] = 0.f;
	pm->scal_U[1] = 1.4830E-2f;

	pm->fault_iab_maximal = 25.f;
	pm->fault_residual_maximal = 0.f;
	pm->fault_drift_maximal = 1.f;
	pm->fault_low_voltage =  5.f;
	pm->fault_high_voltage = 50.f;

	pm->vsi_u_maximal = 5.7735027E-1f;

	pm->lu_gain_K[0] = 2E-1f;
	pm->lu_gain_K[1] = 2E-1f;
	pm->lu_gain_K[2] = 5E-3f;
	pm->lu_gain_K[3] = 5E+0f;
	pm->lu_gain_K[4] = 7E+0f;
	pm->lu_gain_K[5] = 1E-3f;
	pm->lu_gain_K[6] = 2E-3f;
	pm->lu_threshold_low = 1E+2f;
	pm->lu_threshold_high = 2E+2f;

	pm->hf_freq_hz = pm->freq_hz / 12.f;
	pm->hf_swing_D = 1.f;
	pm->hf_gain_K[0] = 1E-1f;
	pm->hf_gain_K[1] = 7E+0f;
	pm->hf_gain_K[2] = 1E-3f;

        pm->bemf_gain_K = 1E-4f;
	pm->bemf_N = 3;

	pm->thermal_gain_R[0] = 20.f;
	pm->thermal_gain_R[1] = 233.64f;

	pm->const_U = 0.f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_Zp = 1;

	pm->i_high_maximal = 20.f;
	pm->i_low_maximal = 10.f;
	pm->i_power_consumption_maximal = 200.f;
	pm->i_power_regeneration_maximal = - 1.f;
	pm->i_slew_rate_D = 4E+3f;
	pm->i_slew_rate_Q = 4E+3f;
	pm->i_gain_P_D = 2E-1f;
	pm->i_gain_I_D = 3E-2f;
	pm->i_gain_P_Q = 2E-1f;
	pm->i_gain_I_Q = 3E-2f;

	pm->p_slew_rate_w = 1E+2f;
	pm->p_gain_D = 1E-2f;
	pm->p_gain_P = 1E-1f;
	pm->p_revol_limit = 10;

	pm->lp_gain[0] = .1f;
	pm->lp_gain[1] = .1f;
}

static void
pm_equation(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ, X4, X5, R1, E1, fluxD, fluxQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = pm->lu_X[4];
	X5 = pm->drift_Q;
	R1 = pm->const_R * (1.f + pm->thermal_R);
	E1 = pm->const_E * (1.f + pm->thermal_E - pm->bemf_Q);

	fluxD = pm->const_Ld * X[0] + E1;
	fluxQ = pm->const_Lq * X[1];

	D[0] = (uD - R1 * X[0] + fluxQ * X4) * pm->const_Ld_inversed;
	D[1] = (uQ - R1 * X[1] - fluxD * X4 + X5) * pm->const_Lq_inversed;
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

	pm_equation(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	mrotf(X2 + 2, D1[2] * dT, X + 2);

	pm_equation(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	mrotf(X + 2, (D1[2] + D2[2]) * dT, X + 2);
}

static void
hf_update(pmc_t *pm, float eD, float eQ)
{
	float		*X = pm->lu_X;
	float		eR, dR, C2;

	eR = pm->hf_CS[1] * eQ;
	dR = pm->hf_gain_K[0] * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	mrotf(X + 2, dR, X + 2);
	X[4] += pm->hf_gain_K[1] * eR;

	if (pm->m_bitmask & PMC_BIT_FLUX_POLARITY_DETECTION) {

		C2 = pm->hf_CS[0] * pm->hf_CS[0] - pm->hf_CS[1] * pm->hf_CS[1];
		pm->hf_flux_polarity += (C2 * eD - pm->hf_flux_polarity)
			* pm->hf_gain_K[2];
	}
}

static void
bemf_tune(pmc_t *pm, float eD, float eQ)
{
	float           *DFT = pm->bemf_DFT;
        float           *TEMP = pm->bemf_TEMP;
	float		A, B, Q;
	int		nu = 0;

	/* FIXME: Observer may be unstable.
	 * */

	A = pm->const_Lq * pm->freq_hz;
	B = - (pm->const_Lq * pm->freq_hz - pm->const_R);

	eQ = (pm->lu_X[4] < 0.f) ? - eQ : eQ;
	Q = A * eQ + B * TEMP[8];
	TEMP[8] = eQ;
	Q *= pm->bemf_gain_K;

	while (nu < pm->bemf_N) {

		*DFT++ += Q * *TEMP++;
		*DFT++ += Q * *TEMP++;

		++nu;
	}
}

static void
bemf_update(pmc_t *pm, float eD, float eQ)
{
        float           *DFT = pm->bemf_DFT;
        float           *TEMP = pm->bemf_TEMP;
        float           X[2], E[2], Q, temp;
        int             nu = 0;

	if (pm->bemf_tune_T > 0) {

		bemf_tune(pm, eD, eQ);
		pm->bemf_tune_T--;
	}

        /* Obtain the rotor position with advance.
         * */
        mrotf(X, .5f * pm->lu_X[4] * pm->dT, pm->lu_X + 2);

        E[0] = X[0];
        E[1] = X[1];

	temp = X[0] * X[0] - X[1] * X[1];
	X[1] = X[0] * X[1] + X[1] * X[0];
	X[0] = temp;

	temp = (3.f - X[0] * X[0] - X[1] * X[1]) * .5f;
	X[0] *= temp;
	X[1] *= temp;

	Q = 0.f;

	while (nu < pm->bemf_N) {

		/*temp = fabsf((float) (3 + 2 * nu) * pm->lu_X[4] * pm->dT * .5f);

		S = 2.9472E-2f;
		S = -1.9667E-1f + S * temp;
		S = 8.6695E-3f + S * temp;
		S = 1.f + S * temp;

		if (S < 0.f)
			break;*/

		/* Basis functions.
		 * */
		temp = E[0] * X[0] - E[1] * X[1];
		E[1] = E[0] * X[1] + E[1] * X[0];
		E[0] = temp;

		/* Inverse DFT.
		 * */
		Q += *DFT++ * E[0];
		Q += *DFT++ * E[1];
		*TEMP++ = E[0];
		*TEMP++ = E[1];

		++nu;
	}

	pm->bemf_Q = Q;
}

static void
lu_update(pmc_t *pm)
{
	float		*X = pm->lu_X;
	float		iX, iY, iD, iQ;
	float		eD, eQ, eR, dR;

	iX = pm->fb_iA - pm->drift_A;
	iY = .57735027f * iX + 1.1547005f * (pm->fb_iB - pm->drift_B);

	iD = X[2] * iX + X[3] * iY;
	iQ = X[2] * iY - X[3] * iX;

	/* Obtain residual.
	 * */
	eD = iD - X[0];
	eQ = iQ - X[1];

	pm->lu_residual_D = eD;
	pm->lu_residual_Q = eQ;
	pm->lu_residual_variance += (eD * eD + eQ * eQ - pm->lu_residual_variance)
		* pm->lp_gain[1];

	/* Measurement update.
	 * */
	X[0] += pm->lu_gain_K[0] * eD;
	X[1] += pm->lu_gain_K[1] * eQ;

	eR = (X[4] < 0.f) ? - eD : eD;
	dR = pm->lu_gain_K[2] * eR;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	mrotf(X + 2, dR, X + 2);
	X[4] += pm->lu_gain_K[3] * eR - pm->lu_gain_K[4] * eQ;

	if (pm->m_bitmask & PMC_BIT_SERVO_FORCED_CONTROL) {

		X[2] = pm->p_track_point_x[0];
		X[3] = pm->p_track_point_x[1];
		X[4] = pm->p_track_point_w;
	}

	if (pm->lu_region == PMC_LU_LOW_REGION) {

		eR = eQ * X[1];

		if (pm->m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION) {

			hf_update(pm, eD, eQ);
			eR += eD * X[0];
		}

		if (pm->m_bitmask & PMC_BIT_THERMAL_DRIFT_ESTIMATION) {

			pm->thermal_R += - eR * pm->lu_gain_K[5];
		}
	}
	else {

		pm->drift_Q += pm->lu_gain_K[6] * eQ;
	}

	/* Time update.
	 * */
	pm_solve_2(pm);

	/* Update BEMF.
	 * */
	if (pm->m_bitmask & PMC_BIT_BEMF_WAVEFORM_COMPENSATION) {

		bemf_update(pm, eD, eQ);
	}
}

static void
lu_post(pmc_t *pm)
{
	float			temp;

	/* Change the region according to the speed.
	 * */
	temp = fabsf(pm->lu_X[4]);

	if (pm->lu_region == PMC_LU_LOW_REGION) {

		if (temp > pm->lu_threshold_high)
			pm->lu_region = PMC_LU_HIGH_REGION;
	}
	else {
		if (temp < pm->lu_threshold_low) {

			pm->lu_region = PMC_LU_LOW_REGION;
			pm->drift_Q = 0.f;
		}
	}

	/* Do not allow the out of range.
	 * */
	temp = pm->freq_hz * (2.f * MPIF / 12.f);
	pm->lu_X[4] = (pm->lu_X[4] > temp) ? temp : (pm->lu_X[4] < - temp) ? - temp : pm->lu_X[4];

	pm->drift_Q = (pm->drift_Q < - pm->const_U) ? - pm->const_U
		: (pm->drift_Q > pm->const_U) ? pm->const_U : pm->drift_Q;

	if (pm->m_bitmask & PMC_BIT_THERMAL_DRIFT_ESTIMATION) {

		pm->thermal_R = (pm->thermal_R < - .3f) ? - .3f
			: (pm->thermal_R > .5f) ? .5f : pm->thermal_R;
	}

	/* Power conversion (Watt).
	 * */
	pm->n_power_watt = 1.5f * (pm->lu_X[0] * pm->vsi_D + pm->lu_X[1] * pm->vsi_Q);

	/* Winding temperature (Celsius).
	 * */
	pm->n_temperature_c = pm->thermal_R * pm->thermal_gain_R[1] + pm->thermal_gain_R[0];
}

static void
vsi_control(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, Q;
	float		uMIN, uMAX;
	int		xA, xB, xC, xMP, xMPH;

	if (0) {

		Q = sqrtf(uX * uX + uY * uY);

		/* Voltage utilisation limit.
		 * */
		if (Q > pm->vsi_u_maximal) {

			Q = pm->vsi_u_maximal / Q;
			uX *= Q;
			uY *= Q;
		}
	}

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

		/* Preserve vector direction if it is overlong.
		 * */
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

	/* Suppress minimal pulses.
	 * */
	xMP = pm->pwm_minimal_pulse;
	xMPH = pm->pwm_minimal_pulse >> 1;

	xA = (xA < xMP) ? (xA < xMPH) ? 0 : xMP : xA;
	xB = (xB < xMP) ? (xB < xMPH) ? 0 : xMP : xB;
	xC = (xC < xMP) ? (xC < xMPH) ? 0 : xMP : xC;

	xMP = pm->pwm_resolution - pm->pwm_minimal_pulse;
	xMPH = pm->pwm_resolution - (pm->pwm_minimal_pulse >> 1);

	xA = (xA > xMP) ? (xA > xMPH) ? pm->pwm_resolution : xMP : xA;
	xB = (xB > xMP) ? (xB > xMPH) ? pm->pwm_resolution : xMP : xB;
	xC = (xC > xMP) ? (xC > xMPH) ? pm->pwm_resolution : xMP : xC;

	pm->pDC(xA, xB, xC);

	/* Dead Time.
	 * */
	/*xMP = pm->pwm_dead_time;

	xA += (xA > 0 && xA < pm->pwm_resolution)
		? (pm->fb_iA < 0.f) ? - xMP : xMP : 0;
	xB += (xB > 0 && xB < pm->pwm_resolution)
		? (pm->fb_iB < 0.f) ? - xMP : xMP : 0;
	xC += (xC > 0 && xC < pm->pwm_resolution)
		? (pm->fb_iA + pm->fb_iB > 0.f) ? - xMP : xMP : 0;*/

	Q = (1.f / 3.f) * (xA + xB + xC);
	uA = (xA - Q) * pm->const_U / pm->pwm_resolution;
	uB = (xB - Q) * pm->const_U / pm->pwm_resolution;

	pm->vsi_X = uA;
	pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;

	pm->vsi_D = pm->lu_X[2] * pm->vsi_X + pm->lu_X[3] * pm->vsi_Y;
	pm->vsi_Q = pm->lu_X[2] * pm->vsi_Y - pm->lu_X[3] * pm->vsi_X;
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
	temp = (pm->lu_region == PMC_LU_HIGH_REGION)
		? pm->i_high_maximal : pm->i_low_maximal;
	sp_D = (sp_D > temp) ? temp : (sp_D < - temp) ? - temp : sp_D;
	sp_Q = (sp_Q > temp) ? temp : (sp_Q < - temp) ? - temp : sp_Q;

	/* Power constraints.
	 * */
	wP = 1.5f * (sp_D * pm->vsi_D + sp_Q * pm->vsi_Q);

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

	temp = (2.f / 3.f) * pm->const_U;

	pm->i_integral_D += pm->i_gain_I_D * eD;
	pm->i_integral_D = (pm->i_integral_D > temp) ? temp :
		(pm->i_integral_D < - temp) ? - temp : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I_Q * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > temp) ? temp :
		(pm->i_integral_Q < - temp) ? - temp : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	/*if (0) {

		uD += pm->const_R * pm->lu_X[0] - pm->const_Lq * pm->lu_X[1] * pm->lu_X[4];
		uQ += pm->const_R * pm->lu_X[1] + (pm->const_Ld * pm->lu_X[0]
				+ pm->const_E) * pm->lu_X[4] + pm->drift_Q;
	}*/

	/* BEMF waveform compensation.
	 * */
	if (pm->m_bitmask & PMC_BIT_BEMF_WAVEFORM_COMPENSATION) {

		uQ += - pm->lu_X[4] * pm->const_E * pm->bemf_Q;
	}

	/* High Frequency Injection.
	 * */
	if (pm->m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION
			&& pm->lu_region == PMC_LU_LOW_REGION) {

		temp = 2.f * MPIF * pm->const_Ld * pm->hf_freq_hz;
		uD += pm->hf_CS[0] * pm->hf_swing_D * temp;
		mrotf(pm->hf_CS, 2.f * MPIF * pm->hf_freq_hz * pm->dT, pm->hf_CS);
	}

	uX = pm->lu_X[2] * uD - pm->lu_X[3] * uQ;
	uY = pm->lu_X[3] * uD + pm->lu_X[2] * uQ;

	vsi_control(pm, uX * pm->const_U_inversed, uY * pm->const_U_inversed);
}

static void
p_control(pmc_t *pm)
{
	float		dX, dY, eP, eD, iSP, temp;
	int		revol;

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

	pm->lu_temp[0] = pm->lu_X[3];

	/*if (1) {

		dX = pm->p_set_point_x[0] * pm->p_track_point_x[0] +
			pm->p_set_point_x[1] * pm->p_track_point_x[1];
		dY = pm->p_set_point_x[1] * pm->p_track_point_x[0] -
			pm->p_set_point_x[0] * pm->p_track_point_x[1];

		eP = matan2f(dY, dX);

		if (dY < 0.) {

			if (pm->p_track_point_x[1] < 0. && pm->p_set_point_x[1] >= 0.)
				eP += 2.f * MPIF;
		}
		else {
			if (pm->p_track_point_x[1] >= 0. && pm->p_set_point_x[1] < 0.)
				eP -= 2.f * MPIF;
		}

		eP += (pm->p_set_point_revol - pm->p_track_point_revol) * 2.f * MPIF;

		// TODO:
		//	pm->p_set_point_w = ;
	}*/

	/* Slew rate (acceleration).
	 * */
	temp = pm->p_slew_rate_w * pm->dT;
	pm->p_track_point_w = (pm->p_track_point_w < pm->p_set_point_w - temp)
		? pm->p_track_point_w + temp : (pm->p_track_point_w > pm->p_set_point_w + temp)
		? pm->p_track_point_w - temp : pm->p_set_point_w;

	mrotf(pm->p_track_point_x, pm->p_track_point_w * pm->dT, pm->p_track_point_x);

	if (pm->p_track_point_x[0] < 0.f) {

		if (pm->p_track_point_x[1] < 0.f) {

			if (pm->lu_temp[1] >= 0.f)
				pm->p_track_point_revol += 1;
		}
		else {
			if (pm->lu_temp[1] < 0.f)
				pm->p_track_point_revol -= 1;
		}
	}

	pm->lu_temp[1] = pm->p_track_point_x[1];

	dX = pm->p_track_point_x[0] * pm->lu_X[2] + pm->p_track_point_x[1] * pm->lu_X[3];
	dY = pm->p_track_point_x[1] * pm->lu_X[2] - pm->p_track_point_x[0] * pm->lu_X[3];

	eP = matan2f(dY, dX);

	if (dY < 0.) {

		if (pm->lu_X[3] < 0. && pm->p_track_point_x[1] >= 0.)
			eP += 2.f * MPIF;
	}
	else {
		if (pm->lu_X[3] >= 0. && pm->p_track_point_x[1] < 0.)
			eP -= 2.f * MPIF;
	}

	revol = pm->p_track_point_revol - pm->lu_revol;
	pm->p_track_point_revol += (revol < - pm->p_revol_limit) ? 1 :
		(revol > pm->p_revol_limit) ? - 1 : 0;

	eP += (pm->p_track_point_revol - pm->lu_revol) * 2.f * MPIF;
	eD = pm->p_track_point_w - pm->lu_X[4];

	/* PD controller.
	 * */
	iSP = pm->p_gain_P * eP + pm->p_gain_D * eD;

	pm->i_set_point_Q = iSP;
}

static void
pm_FSM(pmc_t *pm)
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
				pm->wave_temp[0] += - pm->fb_iA;
				pm->wave_temp[1] += - pm->fb_iB;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					/* Zero Drift.
					 * */
					pm->scal_A[0] += pm->wave_temp[0] / pm->t_end;
					pm->scal_B[0] += pm->wave_temp[1] / pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;

					if (fabsf(pm->scal_A[0]) > pm->fault_drift_maximal)
						pm->m_errno = PMC_ERROR_CURRENT_SENSOR_A;

					if (fabsf(pm->scal_B[0]) > pm->fault_drift_maximal)
						pm->m_errno = PMC_ERROR_CURRENT_SENSOR_B;
				}
			}
			break;

		case PMC_STATE_WAVE_HOLD:

			if (pm->m_phase == 0) {

				pm->pZ(0);

				pm->wave_temp[0] = 0.f;
				pm->wave_temp[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_hold;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->lu_X[0] = pm->fb_iA;
				pm->lu_X[1] = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;

				pm->wave_DFT[0] += pm->vsi_X - pm->wave_i_hold_X * pm->const_R;
				pm->wave_DFT[1] += pm->vsi_Y - pm->wave_i_hold_Y * pm->const_R;

				temp = pm->wave_i_hold_X - pm->lu_X[0];
				pm->wave_temp[0] += pm->wave_gain_I * temp;
				uX = pm->wave_gain_P * temp + pm->wave_temp[0];

				temp = pm->wave_i_hold_Y - pm->lu_X[1];
				pm->wave_temp[1] += pm->wave_gain_I * temp;
				uY = pm->wave_gain_P * temp + pm->wave_temp[1];

				temp = (2.f / 3.f) * pm->const_U;

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

						pm->wave_DFT[0] = 0.f;
						pm->wave_DFT[1] = 0.f;
						pm->wave_DFT[2] = pm->wave_i_hold_X;
						pm->wave_DFT[3] = pm->wave_i_hold_Y;

						pm->t_value = 0;
						pm->t_end = pm->freq_hz * pm->T_measure;

						pm->m_phase++;
					}
					else {
						pm->wave_DFT[0] /= pm->t_end;
						pm->wave_DFT[1] /= pm->t_end;

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

				if (0) {

					pm->wave_temp[2] = pm->wave_i_hold_X * pm->const_R;
					pm->wave_temp[3] = pm->wave_i_hold_Y * pm->const_R;
				}
				else {
					pm->wave_temp[2] = 0.f;
					pm->wave_temp[3] = 0.f;
				}

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_sine;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->lu_X[0] = pm->fb_iA;
				pm->lu_X[1] = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;

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

				mrotf(pm->lu_X + 2, pm->lu_X[4], pm->lu_X + 2);

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
				pm->wave_DFT[0] = 0.f;
				pm->wave_DFT[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_measure;

				pm->m_errno = PMC_OK;
				pm->m_phase++;
			}
			else {
				pm->wave_DFT[0] += pm->fb_iA;
				pm->wave_DFT[1] += - pm->fb_iB;

				temp = pm->wave_i_hold_X - pm->fb_iA;
				pm->wave_temp[0] += pm->wave_gain_I * temp;
				uX = pm->wave_gain_P * temp + pm->wave_temp[0];

				temp = (2.f / 3.f) * pm->const_U;

				if (fabsf(uX) > temp) {

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
					pm->m_errno = PMC_ERROR_OPEN_CIRCUIT;
				}

				vsi_control(pm, uX * pm->const_U_inversed, 0.f);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->wave_DFT[0] /= pm->t_end;
					pm->wave_DFT[1] /= pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_START:

			if (pm->m_errno != PMC_OK) {

				pm->m_state = PMC_STATE_END;
				pm->m_phase = 0;

				break;
			}

			pm->lu_region = PMC_LU_LOW_REGION;
			pm->pZ(0);

			pm->lu_X[0] = 0.f;
			pm->lu_X[1] = 0.f;
			pm->lu_X[2] = 1.f;
			pm->lu_X[3] = 0.f;
			pm->lu_X[4] = 0.f;
			pm->lu_revol = 0;
			pm->lu_temp[0] = 0.f;
			pm->lu_temp[1] = 0.f;
			pm->lu_residual_variance = 0.f;

			pm->drift_A = 0.f;
			pm->drift_B = 0.f;
			pm->drift_Q = 0.f;

			if (pm->m_bitmask & PMC_BIT_HIGH_FREQUENCY_INJECTION) {

				pm->hf_CS[0] = 1.;
				pm->hf_CS[1] = 0.;
				pm->hf_flux_polarity = 0.f;
			}

			if (pm->m_bitmask & PMC_BIT_BEMF_WAVEFORM_COMPENSATION) {

				pm->bemf_Q = 0.f;
			}

			pm->thermal_R = 0.f;
			pm->thermal_E = 0.f;

			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 0.f;
			pm->i_track_point_D = 0.f;
			pm->i_track_point_Q = 0.f;
			pm->i_integral_D = 0.f;
			pm->i_integral_Q = 0.f;

			if (pm->m_bitmask & PMC_BIT_SERVO_CONTROL_LOOP) {

				pm->p_set_point_w = 0.f;
				pm->p_track_point_x[0] = 1.f;
				pm->p_track_point_x[1] = 0.f;
				pm->p_track_point_revol = 0;
				pm->p_track_point_w = 0.f;
			}

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

				pm->lu_region = PMC_LU_DISABLED;
				pm->pDC(0, 0, 0);

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_end;

				pm->m_phase++;
			}
			else {
				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->pZ(7);

					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}
			break;

		default:
			pm->m_state = PMC_STATE_END;
			pm->m_phase = 0;
	}
}

void pmc_feedback(pmc_t *pm, int xA, int xB)
{
	pm->fb_iA = pm->scal_A[1] * (xA - 2048) + pm->scal_A[0];
	pm->fb_iB = pm->scal_B[1] * (xB - 2048) + pm->scal_B[0];

	/* Overcurrent protection.
	 * */
	if (fabsf(pm->fb_iA) > pm->fault_iab_maximal
			|| fabsf(pm->fb_iB) > pm->fault_iab_maximal) {

		if (pm->lu_region != PMC_LU_DISABLED
				|| pm->m_state == PMC_STATE_WAVE_HOLD
				|| pm->m_state == PMC_STATE_WAVE_SINE
				|| pm->m_state == PMC_STATE_CALIBRATION) {

			pm->m_state = PMC_STATE_END;
			pm->m_phase = 0;
			pm->m_errno = PMC_ERROR_OVERCURRENT;
		}
	}

	pm_FSM(pm);

	if (pm->lu_region != PMC_LU_DISABLED) {

		lu_update(pm);
		i_control(pm);
		(pm->m_bitmask & PMC_BIT_SERVO_CONTROL_LOOP) ? p_control(pm) : 0 ;
		lu_post(pm);
	}
}

void pmc_voltage(pmc_t *pm, int xU)
{
	float		uS;

	uS = pm->scal_U[1] * xU + pm->scal_U[0];

	pm->const_U += (uS - pm->const_U) * pm->lp_gain[0];
	pm->const_U_inversed = 1.f / pm->const_U;

	/* Low voltage.
	 * */
	if (pm->const_U < pm->fault_low_voltage) {

		if (pm->lu_region != PMC_LU_DISABLED
				|| pm->m_state != PMC_STATE_IDLE) {

			pm->m_state = PMC_STATE_END;
			pm->m_phase = 0;
			pm->m_errno = PMC_ERROR_LOW_VOLTAGE;
		}
	}

	/* High voltage.
	 * */
	if (pm->const_U > pm->fault_high_voltage) {

		if (pm->lu_region != PMC_LU_DISABLED
				|| pm->m_state != PMC_STATE_IDLE) {

			pm->m_state = PMC_STATE_END;
			pm->m_phase = 0;
			pm->m_errno = PMC_ERROR_LOW_VOLTAGE;
		}
	}
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

void pmc_resistance(const float *DFT, float *R)
{
	float			D, X, Y, E;

	D = sqrtf(DFT[2] * DFT[2] + DFT[3] * DFT[3]);

	if (D > 0.f) {

		X = DFT[2] / D;
		Y = DFT[3] / D;

		E = DFT[0] * X + DFT[1] * Y;
		*R = E / D;
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
	lib_eigenvalues(Lx, Ly, Lm, IMP + 0);
	lib_eigenvalues(Rx, Ry, Rm, IMP + 3);
}

const char *pmc_strerror(int errno)
{
	const char	*list[] = {

		"No Error",
		"Current Sensor A",
		"Current Sensor B",
		"Open Circuit",
		"Over Current",
		"Low Voltage",
		"High Voltage",

		"(AP) Timeout"
	};

	const int 	emax = sizeof(list) / sizeof(list[0]);

	return (errno >= 0 && errno < emax) ? list[errno] : "";
}

