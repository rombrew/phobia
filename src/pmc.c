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

inline float
fabsf(float x) { return __builtin_fabsf(x); }

#if __ARM_FP >= 4

inline float
sqrtf(float x)
{
	float		y;

	asm volatile ("vsqrt.f32 %0, %1 \r\n"
			: "=w" (y) : "w" (x));

	return y;
}

#else

inline float
sqrtf(float x) { return __builtin_sqrtf(x); }

#endif

static void
rotatef(float y[2], float rad, const float x[2])
{
	float		q, s, c, a, b;

	q = rad * rad;
	s = rad - .16666667f * q * rad;
	c = 1.f - .5f * q;

	a = c * x[0] - s * x[1];
	b = s * x[0] + c * x[1];

	q = (3.f - a * a - b * b) * .5f;
	y[0] = a * q;
	y[1] = b * q;
}

void pmc_default(pmc_t *pm)
{
	pm->pwm_minimal_pulse = 8;

	pm->m_request = 0;
	pm->m_bitmask = 0
		| PMC_BIT_EFFICIENT_MODULATION
		//| PMC_BIT_SPEED_CONTROL_LOOP
		//| PMC_BIT_DIRECT_INJECTION
		//| PMC_BIT_FREQUENCY_INJECTION
		| PMC_BIT_UPDATE_R_AFTER_HOLD
		| PMC_BIT_UPDATE_L_AFTER_SINE;
	pm->m_bitmode = 0;
	pm->m_state = PMC_STATE_IDLE;
	pm->m_phase = 0;
	pm->m_errno = 0;

	pm->T_drift = .1f;
	pm->T_hold = .5f;
	pm->T_avg = .2f;
	pm->T_end = .1f;

	pm->i_hold = 1.f;
	pm->i_sine = 1.f;
	pm->i_offset_D = 0.f;
	pm->i_offset_Q = 0.f;
	pm->freq_sine_hz = 4000.f;

	pm->scal_A[0] = 0.f;
	pm->scal_A[1] = 1.0f * 1.332e-2f;// * 1.11f;
	pm->scal_B[0] = 0.f;
	pm->scal_B[1] = 1.332e-2f;// * 1.15f;
	pm->scal_U[0] = 0.f;
	pm->scal_U[1] = 6.592e-3f;

	pm->residual_variance = 0.f;
	pm->residual_gain_F = 5e-2f;

	pm->kalman_Q[0] = 1e-5f;
	pm->kalman_Q[1] = 1e-5f;
	pm->kalman_Q[2] = 1e-4f;
	pm->kalman_Q[3] = 1e+4f;
	pm->kalman_Q[4] = 1e-7f;
	pm->kalman_R = 4e-2f;

	pm->saliency_boost_D = 2.f;
	pm->saliency_boost_Q = .5f;

	pm->drift_AB_maximal = 2.f;
	pm->drift_Q_maximal = 2.f;

	pm->const_U = 0.f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ld = 0.f;
	pm->const_Lq = 0.f;
	pm->const_gain_U = 1e-1f;
	pm->const_Zp = 1;

	pm->i_maximal = 10.f;
	pm->i_low_maximal = 10.f;
	pm->i_power_consumption_maximal = 50.f;
	pm->i_power_regeneration_maximal = - 1.f;
	pm->i_set_point_D = 0.f;
	pm->i_set_point_Q = 0.f;
	pm->i_slew_rate_D = 4e+3f;
	pm->i_slew_rate_Q = 4e+3f;
	pm->i_track_point_D = 0.f;
	pm->i_track_point_Q = 0.f;
	pm->i_integral_D = 0.f;
	pm->i_integral_Q = 0.f;
	pm->i_gain_P_D = 2e-1f;
	pm->i_gain_I_D = 3e-2f;
	pm->i_gain_P_Q = 2e-1f;
	pm->i_gain_I_Q = 3e-2f;

	pm->i_inject_D = 1.f;
	pm->i_inject_gain_K = 0.f;

	pm->h_freq_hz = 4000.f;
	pm->h_swing_D = 1.f;

	pm->w_low_threshold = 104.f;
	pm->w_low_hysteresis = 47.f;
	pm->w_set_point = 0.f;
	pm->w_integral = 0.f;
	pm->w_gain_P = 1e-3f;
	pm->w_gain_I = 1e-5f;
}

static void
pm_equation(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ, X4, X5;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	X4 = pm->kalman_X[4];
	X5 = pm->drift_Q;

	D[0] = (uD - pm->const_R * X[0] + pm->const_Lq * X4 * X[1]) * pm->const_ILd;
	D[1] = (uQ - pm->const_R * X[1] - pm->const_Ld * X4 * X[0]
			- pm->const_E * X4 + X5) * pm->const_ILq;
	D[2] = X4;
}

static void
pm_update(pmc_t *pm)
{
	float		*X = pm->kalman_X;
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
kalman_measurement_update(pmc_t *pm, float iA, float iB)
{
	float		*X = pm->kalman_X, *P = pm->kalman_P;
	float		PC[10], S[3], L[3], K[10];
	float		iX, iY, iD, iQ, eD, eQ, dR, temp;

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

	pm->residual_D = eD;
	pm->residual_Q = eQ;
	pm->residual_variance += (eD * eD + eQ * eQ - pm->residual_variance)
		* pm->residual_gain_F;

	/* S = C * P * C' + R;
	 * */
	PC[0] = P[0] - P[3] * X[1];
	PC[1] = P[1] + P[3] * X[0];
	PC[2] = P[1] - P[4] * X[1];
	PC[3] = P[2] + P[4] * X[0];
	PC[4] = P[3] - P[5] * X[1];
	PC[5] = P[4] + P[5] * X[0];
	PC[6] = P[6] - P[8] * X[1];
	PC[7] = P[7] + P[8] * X[0];
	PC[8] = P[10] - P[12] * X[1];
	PC[9] = P[11] + P[12] * X[0];

	S[0] = PC[0] - X[1] * PC[4] + pm->kalman_R;
	S[1] = PC[1] - X[1] * PC[5];
	S[2] = PC[3] + X[0] * PC[5] + pm->kalman_R;

	/* K = P * C' / S;
	 * */
	L[0] = 1.f / S[0];
	L[1] = S[1] * L[0];
	L[2] = 1.f / (S[2] - S[1] * L[1]);

	K[1] = (PC[1] - PC[0] * L[1]) * L[2];
	K[0] = PC[0] * L[0] - K[1] * L[1];
	K[3] = (PC[3] - PC[2] * L[1]) * L[2];
	K[2] = PC[2] * L[0] - K[3] * L[1];
	K[5] = (PC[5] - PC[4] * L[1]) * L[2];
	K[4] = PC[4] * L[0] - K[5] * L[1];
	K[7] = (PC[7] - PC[6] * L[1]) * L[2];
	K[6] = PC[6] * L[0] - K[7] * L[1];
	K[9] = (PC[9] - PC[8] * L[1]) * L[2];
	K[8] = PC[8] * L[0] - K[9] * L[1];

	pm->temp_B[0] = K[5];

	/* X = X + K * e;
	 * */
	/*X[0] += K[0] * eD + K[1] * eQ;
	X[1] += K[2] * eD + K[3] * eQ;
	dR = K[4] * eD + K[5] * eQ;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	rotatef(X + 2, dR, X + 2);
	X[4] += K[6] * eD + K[7] * eQ;
	pm->drift_Q += K[8] * eD + K[9] * eQ;*/

	X[0] += .5f * eD;
	X[1] += .5f * eQ;
	dR = .02f * eD + .0f * eQ;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	rotatef(X + 2, dR, X + 2);
	X[4] += 0.f * eD + -197.f * eQ;
	pm->drift_Q += 0e-3f * eD + 5e-4f * eQ;

	/* P = P - K * C * P.
	 * */
	P[0] -= K[0] * PC[0] + K[1] * PC[1];
	P[1] -= K[2] * PC[0] + K[3] * PC[1];
	P[2] -= K[2] * PC[2] + K[3] * PC[3];
	P[3] -= K[4] * PC[0] + K[5] * PC[1];
	P[4] -= K[4] * PC[2] + K[5] * PC[3];
	P[5] -= K[4] * PC[4] + K[5] * PC[5];
	P[6] -= K[6] * PC[0] + K[7] * PC[1];
	P[7] -= K[6] * PC[2] + K[7] * PC[3];
	P[8] -= K[6] * PC[4] + K[7] * PC[5];
	P[9] -= K[6] * PC[6] + K[7] * PC[7];
	P[10] -= K[8] * PC[0] + K[9] * PC[1];
	P[11] -= K[8] * PC[2] + K[9] * PC[3];
	P[12] -= K[8] * PC[4] + K[9] * PC[5];
	P[13] -= K[8] * PC[6] + K[9] * PC[7];
	P[14] -= K[8] * PC[8] + K[9] * PC[9];

	pm->temp_A[0] = X[0];
	pm->temp_A[1] = X[1];
	pm->temp_A[2] = X[2];
	pm->temp_A[3] = X[3];
	pm->temp_A[4] = X[4];

	pm_update(pm);

	pm->temp_A[0] = (pm->temp_A[0] + X[0]) * .5f;
	pm->temp_A[1] = (pm->temp_A[1] + X[1]) * .5f;
	pm->temp_A[2] = (pm->temp_A[2] + X[2]) * .5f;
	pm->temp_A[3] = (pm->temp_A[3] + X[3]) * .5f;
	pm->temp_A[4] = (pm->temp_A[4] + X[4]) * .5f;

	temp = (3.f - pm->temp_A[2] * pm->temp_A[2]
			- pm->temp_A[3] * pm->temp_A[3]) * .5f;
	pm->temp_A[2] *= temp;
	pm->temp_A[3] *= temp;
}

static void
kalman_time_update(pmc_t *pm)
{
	float		*P = pm->kalman_P;
	float		A[9], PA[15];
	float		dT, dTLd, dTLq;

	dT = pm->dT;
	dTLd = dT * pm->const_ILd;
	dTLq = dT * pm->const_ILq;

	if (pm->m_bitmode & PMC_BIT_FREQUENCY_INJECTION) {

		dTLd *= pm->saliency_boost_D;
		dTLq *= pm->saliency_boost_Q;
	}

	A[0] = 1.f - pm->const_R * dTLd;
	A[1] = pm->temp_A[4] * pm->const_Lq * dTLd;
	A[2] = (pm->temp_A[2] * pm->vsi_Y - pm->temp_A[3] * pm->vsi_X) * dTLd;
	A[3] = pm->temp_A[1] * pm->const_Lq * dTLd;

	A[4] = - pm->temp_A[4] * pm->const_Ld * dTLq;
	A[5] = 1.f - pm->const_R * dTLq;
	A[6] = - (pm->temp_A[2] * pm->vsi_X + pm->temp_A[3] * pm->vsi_Y) * dTLq;
	A[7] = - (pm->const_E + pm->temp_A[0] * pm->const_Ld) * dTLq;
	A[8] = dTLq;

	/* P = A * P * A' + Q.
	 * */
	PA[0] = P[0] * A[0] + P[1] * A[1] + P[3] * A[2] + P[6] * A[3];
	PA[1] = P[0] * A[4] + P[1] * A[5] + P[3] * A[6] + P[6] * A[7] + P[10] * A[8];
	PA[2] = P[3] + P[6] * dT;

	PA[3] = P[1] * A[0] + P[2] * A[1] + P[4] * A[2] + P[7] * A[3];
	PA[4] = P[1] * A[4] + P[2] * A[5] + P[4] * A[6] + P[7] * A[7] + P[11] * A[8];
	PA[5] = P[4] + P[7] * dT;

	PA[6] = P[3] * A[0] + P[4] * A[1] + P[5] * A[2] + P[8] * A[3];
	PA[7] = P[3] * A[4] + P[4] * A[5] + P[5] * A[6] + P[8] * A[7] + P[12] * A[8];
	PA[8] = P[5] + P[8] * dT;

	PA[9] = P[6] * A[0] + P[7] * A[1] + P[8] * A[2] + P[9] * A[3];
	PA[10] = P[6] * A[4] + P[7] * A[5] + P[8] * A[6] + P[9] * A[7] + P[13] * A[8];
	PA[11] = P[8] + P[9] * dT;

	PA[12] = P[10] * A[0] + P[11] * A[1] + P[12] * A[2] + P[13] * A[3];
	PA[13] = P[10] * A[4] + P[11] * A[5] + P[12] * A[6] + P[13] * A[7] + P[14] * A[8];
	PA[14] = P[12] + P[13] * dT;

	P[0] = A[0] * PA[0] + A[1] * PA[3] + A[2] * PA[6] + A[3] * PA[9];
	P[1] = A[0] * PA[1] + A[1] * PA[4] + A[2] * PA[7] + A[3] * PA[10];
	P[2] = A[4] * PA[1] + A[5] * PA[4] + A[6] * PA[7] + A[7] * PA[10] + A[8] * PA[13];
	P[3] = PA[6] + dT * PA[9];
	P[4] = PA[7] + dT * PA[10];
	P[5] = PA[8] + dT * PA[11];
	P[6] = PA[9];
	P[7] = PA[10];
	P[8] = PA[11];
	P[10] = PA[12];
	P[11] = PA[13];
	P[12] = PA[14];

	P[0] += pm->kalman_Q[0];
	P[2] += pm->kalman_Q[1];
	P[5] += pm->kalman_Q[2];
	P[9] += pm->kalman_Q[3];
	P[14] += /*(pm->m_bitmode & PMC_BIT_LOW_SPEED_REGION) ? 0.f : */pm->kalman_Q[4];
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

	if (Q < 1.f) {

		if (pm->m_bitmode & PMC_BIT_EFFICIENT_MODULATION) {

			Q = uMIN + uMAX;
			Q = (Q < 0.f) ? 0.f - uMIN : 1.f - uMAX;
		}
		else {
			Q = (uMIN < - .5f) ? 0.f - uMIN :
				(uMAX > .5f) ? 1.f - uMAX : .5f;
		}
	}
	else {
		Q = 1.f / Q;
		uA *= Q;
		uB *= Q;
		uC *= Q;

		Q = .5f - (uMIN + uMAX) * Q * .5f;
	}

	uA += Q;
	uB += Q;
	uC += Q;

	xA = (int) (pm->pwm_resolution * uA + .5f);
	xB = (int) (pm->pwm_resolution * uB + .5f);
	xC = (int) (pm->pwm_resolution * uC + .5f);

	temp = pm->pwm_resolution - pm->pwm_minimal_pulse;
	xA = (xA < pm->pwm_minimal_pulse) ? 0 : (xA > temp) ? pm->pwm_resolution : xA;
	xB = (xB < pm->pwm_minimal_pulse) ? 0 : (xB > temp) ? pm->pwm_resolution : xB;
	xC = (xC < pm->pwm_minimal_pulse) ? 0 : (xC > temp) ? pm->pwm_resolution : xC;

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

	if (pm->m_bitmode & PMC_BIT_DIRECT_INJECTION)
		sp_D += pm->i_inject_gain_K * fabsf(sp_Q) + pm->i_inject_D;

	/* Current constraints.
	 * */
	temp = (pm->m_bitmode & PMC_BIT_LOW_SPEED_REGION) ? pm->i_low_maximal : pm->i_maximal;
	sp_D = (sp_D > temp) ? temp : (sp_D < - temp) ? - temp : sp_D;
	sp_Q = (sp_Q > temp) ? temp : (sp_Q < - temp) ? - temp : sp_Q;

	/* Power constraints.
	 * */
	uD = pm->temp_A[2] * pm->vsi_X + pm->temp_A[3] * pm->vsi_Y;
	uQ = pm->temp_A[2] * pm->vsi_Y - pm->temp_A[3] * pm->vsi_X;
	pm->i_power_watt = pm->kalman_X[0] * uD + pm->kalman_X[1] * uQ;
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
	eD = pm->i_track_point_D - pm->kalman_X[0];
	eQ = pm->i_track_point_Q - pm->kalman_X[1];

	if (pm->m_bitmode & PMC_BIT_FREQUENCY_INJECTION)
		eD = (fabsf(eD) < pm->h_swing_D) ? 0.f : eD;

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
	if (pm->m_bitmode & PMC_BIT_FREQUENCY_INJECTION) {

		temp = 6.2831853f * pm->const_Ld * pm->h_freq_hz;
		uD += pm->h_CS[1] * pm->h_swing_D * temp;
		rotatef(pm->h_CS, 6.2831853f * pm->h_freq_hz * pm->dT, pm->h_CS);
	}

	uX = pm->kalman_X[2] * uD - pm->kalman_X[3] * uQ;
	uY = pm->kalman_X[3] * uD + pm->kalman_X[2] * uQ;

	vsi_control(pm, uX * pm->const_IU, uY * pm->const_IU);
}

static void
w_control(pmc_t *pm)
{
	float		eW, iSP;

	pm->w_avg_speed += pm->kalman_X[4];

	if (pm->t_clock == 0) {

		pm->w_avg_speed *= .2f;

		/* Obtain residual.
		 * */
		eW = pm->w_set_point - pm->w_avg_speed;

		/* PI controller.
		 * */
		iSP = pm->w_gain_P * eW;

		if (fabsf(iSP) < pm->i_maximal) {

			pm->w_integral += pm->w_gain_I * eW;
			pm->w_integral = (pm->w_integral > pm->i_maximal) ? pm->i_maximal :
				(pm->w_integral < - pm->i_maximal) ? - pm->i_maximal : pm->w_integral;
			iSP += pm->w_integral;
		}

		pm->i_set_point_D = iSP;
		pm->i_set_point_Q = iSP;

		pm->w_avg_speed = 0.f;
	}
}

static void
pmc_inductance(pmc_t *pm, float *ft, float freq_hz)
{
	float			Dx, Dy, Lx, Ly, Lm;
	float			b, D, la1, la2;

	/* Inductance on XY axes.
	 * */
	Dx = ft[0] * ft[0] + ft[1] * ft[1];
	Dy = ft[4] * ft[4] + ft[5] * ft[5];
	D = 6.2831853f * freq_hz;
	Dx *= D; Dy *= D;
	Lx = (ft[2] * ft[1] - ft[3] * ft[0]) / Dx;
	Ly = (ft[6] * ft[5] - ft[7] * ft[4]) / Dy;
	Lm = (ft[6] * ft[1] - ft[7] * ft[0]) / Dx;
	Lm += (ft[2] * ft[5] - ft[3] * ft[4]) / Dy;
	Lm *= .5f;

	/* Get eigenvalues.
	 * */
	b = - (Lx + Ly);
	D = b * b - 4.f * (Lx * Ly - Lm * Lm);

	if (D > 0.f) {

		D = sqrtf(D);
		la1 = (- b - D) * .5f;
		la2 = (- b + D) * .5f;

		b = Lx - la1;
		b = sqrtf(Lm * Lm + b * b);
		D = fabsf(Lm / b);

		b = Lx - la2;
		b = sqrtf(Lm * Lm + b * b);
		b = fabsf(Lm / b);

		if (D < b) {

			/* Reluctance saliency.
			 * */
			pm->const_Ld = la2;
			pm->const_Lq = la1;
		}
		else {
			/* Saturation saliency.
			 * */
			pm->const_Ld = la1;
			pm->const_Lq = la2;
		}

		pm->const_ILd = 1.f / pm->const_Ld;
		pm->const_ILq = 1.f / pm->const_Lq;
	}
}

static void
pmc_FSM(pmc_t *pm, float iA, float iB)
{
	float			uX, uY, temp;

	switch (pm->m_state) {

		case PMC_STATE_IDLE:

			if (pm->m_request != 0) {

				if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

					if (pm->m_request == PMC_STATE_ESTIMATE_AB
							|| pm->m_request == PMC_STATE_ESTIMATE_R
							|| pm->m_request == PMC_STATE_ESTIMATE_E
							|| pm->m_request == PMC_STATE_KALMAN_STOP)
						pm->m_state = pm->m_request;
					else
						pm->m_request = 0;
				}
				else {
					if (pm->m_request == PMC_STATE_ZERO_DRIFT
							|| pm->m_request == PMC_STATE_WAVE_HOLD
							|| pm->m_request == PMC_STATE_WAVE_SINE
							|| pm->m_request == PMC_STATE_CALIBRATION
							|| pm->m_request == PMC_STATE_KALMAN_START)
						pm->m_state = pm->m_request;
					else
						pm->m_request = 0;
				}
			}
			break;

		case PMC_STATE_ZERO_DRIFT:

			if (pm->m_phase == 0) {

				pm->pZ(7);

				pm->temp_A[0] = 0.f;
				pm->temp_A[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_drift;

				pm->m_phase++;
			}
			else {
				pm->temp_A[0] += - iA;
				pm->temp_A[1] += - iB;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					/* Zero Drift.
					 * */
					pm->scal_A[0] += pm->temp_A[0] / pm->t_end;
					pm->scal_B[0] += pm->temp_A[1] / pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_WAVE_HOLD:

			if (pm->m_phase == 0) {

				pm->pDC(0, 0, 0);
				pm->pZ(0);

				pm->kalman_X[2] = 1.f;
				pm->kalman_X[3] = 0.f;

				pm->i_set_point_D = pm->i_hold;
				pm->i_set_point_Q = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_hold;

				pm->m_phase++;
			}
			else {
				pm->kalman_X[0] = iA;
				pm->kalman_X[1] = .57735027f * iA + 1.1547005f * iB;

				pm->temp_A[0] += pm->vsi_X - pm->i_set_point_D * pm->const_R;

				i_control(pm);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					if (pm->m_phase == 1) {

						if (pm->m_bitmask & PMC_BIT_UPDATE_R_AFTER_HOLD) {

							pm->temp_A[0] = 0.f;

							pm->t_value = 0;
							pm->t_end = pm->freq_hz * pm->T_avg;

							pm->m_phase++;
						}
						else {
							pm->m_state = PMC_STATE_END;
							pm->m_phase = 0;
						}
					}
					else {
						/* Winding Resistance.
						 * */
						pm->const_R += pm->temp_A[0] / pm->t_end
							/ pm->i_set_point_D;

						pm->m_state = PMC_STATE_END;
						pm->m_phase = 0;
					}
				}
			}
			break;

		case PMC_STATE_WAVE_SINE:

			if (pm->m_phase == 0) {

				pm->pDC(0, 0, 0);
				pm->pZ(0);

				pm->kalman_X[2] = 1.f;
				pm->kalman_X[3] = 0.f;
				pm->kalman_X[4] = 6.2831853f * pm->freq_sine_hz / pm->freq_hz;

				pm->temp_B[0] = 0.f;
				pm->temp_B[1] = 0.f;
				pm->temp_B[2] = 0.f;
				pm->temp_B[3] = 0.f;
				pm->temp_B[4] = 0.f;
				pm->temp_B[5] = 0.f;
				pm->temp_B[6] = 0.f;
				pm->temp_B[7] = 0.f;

				temp = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
				temp = 6.2831853f * temp * pm->freq_sine_hz;
				temp = pm->const_R * pm->const_R + temp * temp;
				pm->temp_A[0] = pm->i_sine * sqrtf(temp);
				pm->temp_A[1] = pm->temp_A[0];
				pm->temp_A[2] = pm->i_offset_D * pm->const_R;
				pm->temp_A[3] = pm->i_offset_Q * pm->const_R;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_avg;

				pm->m_phase++;
			}
			else {
				pm->kalman_X[0] = iA;
				pm->kalman_X[1] = .57735027f * iA + 1.1547005f * iB;

				pm->temp_B[0] += pm->kalman_X[0] * pm->kalman_X[2];
				pm->temp_B[1] += pm->kalman_X[0] * pm->kalman_X[3];
				pm->temp_B[2] += pm->vsi_X * pm->kalman_X[2];
				pm->temp_B[3] += pm->vsi_X * pm->kalman_X[3];
				pm->temp_B[4] += pm->kalman_X[1] * pm->kalman_X[2];
				pm->temp_B[5] += pm->kalman_X[1] * pm->kalman_X[3];
				pm->temp_B[6] += pm->vsi_Y * pm->kalman_X[2];
				pm->temp_B[7] += pm->vsi_Y * pm->kalman_X[3];

				rotatef(pm->kalman_X + 2, pm->kalman_X[4], pm->kalman_X + 2);

				uX = pm->temp_A[2] + pm->temp_A[0] * pm->kalman_X[2];
				uX *= pm->const_IU;
				uY = pm->temp_A[3] + pm->temp_A[1] * pm->kalman_X[3];
				uY *= pm->const_IU;

				vsi_control(pm, uX, uY);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					if (pm->m_bitmask & PMC_BIT_UPDATE_L_AFTER_SINE) {

						/* Winding Inductance.
						 * */
						pmc_inductance(pm, pm->temp_B,
								pm->freq_sine_hz);
					}

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_CALIBRATION:

			if (pm->m_phase == 0) {

				/* FIXME: !
				 * */

				if (pm->m_bitmask & PMC_BIT_DIRECT_INJECTION)
					pm->pDC(pm->pwm_resolution, 0, 0);
				else
					pm->pDC(0, pm->pwm_resolution, 0);

				pm->pZ(4);

				pm->temp_A[0] = 0.f;
				pm->temp_A[1] = 0.f;
				pm->temp_A[2] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_avg;

				pm->m_phase++;
			}
			else {
				pm->temp_A[0] += iA;
				pm->temp_A[1] += iB;
				pm->temp_A[2] += pm->const_U;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					pm->temp_A[0] /= pm->t_end;
					pm->temp_A[1] /= pm->t_end;
					pm->temp_A[2] /= pm->t_end;

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_ESTIMATE_AB:
			break;

		case PMC_STATE_ESTIMATE_R:
		case PMC_STATE_ESTIMATE_E:

			if (pm->m_phase == 0) {

				pm->temp_B[0] = 0.f;
				pm->temp_B[1] = 0.f;
				pm->temp_B[2] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_avg;

				pm->m_phase++;
			}
			else {
				pm->temp_B[0] += pm->kalman_X[1];
				pm->temp_B[1] += pm->kalman_X[4];
				pm->temp_B[2] += pm->drift_Q;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					if (pm->m_state == PMC_STATE_ESTIMATE_R) {

						pm->const_R += - pm->temp_B[2] / pm->temp_B[0];
					}
					else if (pm->m_state == PMC_STATE_ESTIMATE_E) {

						pm->const_E += - pm->temp_B[2] / pm->temp_B[1];
					}

					pm->m_request = 0;
					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_KALMAN_START:

			pm->m_bitmode |= PMC_BIT_KALMAN_FILTER;
			pm->m_bitmode |= (PMC_BIT_SPEED_CONTROL_LOOP
					| PMC_BIT_POSITION_CONTROL_LOOP)
				& pm->m_bitmask;

			pm->pDC(0, 0, 0);
			pm->pZ(0);

			/* X(0).
			 * */
			pm->kalman_X[0] = 0.f;
			pm->kalman_X[1] = 0.f;
			pm->kalman_X[2] = 1.f;
			pm->kalman_X[3] = 0.f;
			pm->kalman_X[4] = 0.f;

			pm->drift_A = 0.f;
			pm->drift_B = 0.f;
			pm->drift_Q = 0.f;

			/* P(0).
			 * */
			pm->kalman_P[1] = 0.f;
			pm->kalman_P[3] = 0.f;
			pm->kalman_P[4] = 0.f;
			pm->kalman_P[6] = 0.f;
			pm->kalman_P[7] = 0.f;
			pm->kalman_P[8] = 0.f;
			pm->kalman_P[10] = 0.f;
			pm->kalman_P[11] = 0.f;
			pm->kalman_P[12] = 0.f;
			pm->kalman_P[13] = 0.f;

			pm->kalman_P[0] = 0.f;
			pm->kalman_P[2] = 0.f;
			pm->kalman_P[5] = 1.f;
			pm->kalman_P[9] = 0.f;
			pm->kalman_P[14] = 0.f;

			pm->t_clock = 0;

			pm->h_CS[0] = 1.;
			pm->h_CS[1] = 0.;

			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 0.f;
			pm->w_set_point = 0.f;

			pm->m_request = 0;
			pm->m_state = PMC_STATE_IDLE;
			pm->m_phase = 0;
			break;

		case PMC_STATE_KALMAN_STOP:

			if (pm->m_phase == 0) {

				pm->m_bitmode &= ~(PMC_BIT_SPEED_CONTROL_LOOP
						| PMC_BIT_POSITION_CONTROL_LOOP);

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
					pm->pZ(7);

					pm->m_request = 0;
					pm->m_bitmode = 0;
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
					pm->pZ(7);

					pm->m_request = 0;
					pm->m_bitmode = 0;
					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}
			break;
	}
}

static void
pmc_underway(pmc_t *pm)
{
	float		w_low, w_high;

	if (pm->t_clock == 1) {

		w_low = fabsf(pm->kalman_X[4]);
		w_high = w_low + pm->w_low_hysteresis;
		w_low = w_low - pm->w_low_hysteresis;

		if (pm->m_bitmode & PMC_BIT_LOW_SPEED_REGION) {

			if (w_low > pm->w_low_threshold) {

				pm->m_bitmode &= ~PMC_BIT_LOW_SPEED_REGION;
				pm->m_bitmode &= ~(PMC_BIT_DIRECT_INJECTION
						| PMC_BIT_FREQUENCY_INJECTION);
				pm->m_bitmode |= (PMC_BIT_EFFICIENT_MODULATION)
					& pm->m_bitmask;
			}
		}
		else {
			if (w_high < pm->w_low_threshold) {

				pm->m_bitmode |= PMC_BIT_LOW_SPEED_REGION;
				pm->m_bitmode &= ~(PMC_BIT_EFFICIENT_MODULATION);
				pm->m_bitmode |= (PMC_BIT_DIRECT_INJECTION
						| PMC_BIT_FREQUENCY_INJECTION)
					& pm->m_bitmask;

				/*pm->drift_Q = 0.f;

				pm->kalman_P[10] = 0.f;
				pm->kalman_P[11] = 0.f;
				pm->kalman_P[12] = 0.f;
				pm->kalman_P[13] = 0.f;
				pm->kalman_P[14] = 0.f;*/
			}
		}
	}
	else if (pm->t_clock == 2) {

		pm->drift_A = (pm->drift_A < - pm->drift_AB_maximal) ? - pm->drift_AB_maximal
			: (pm->drift_A > pm->drift_AB_maximal) ? pm->drift_AB_maximal : pm->drift_A;
		pm->drift_B = (pm->drift_B < - pm->drift_AB_maximal) ? - pm->drift_AB_maximal
			: (pm->drift_B > pm->drift_AB_maximal) ? pm->drift_AB_maximal : pm->drift_B;
		pm->drift_Q = (pm->drift_Q < - pm->drift_Q_maximal) ? - pm->drift_Q_maximal
			: (pm->drift_Q > pm->drift_Q_maximal) ? pm->drift_Q_maximal : pm->drift_Q;

	}
	else if (pm->t_clock == 3) {

		if (pm->residual_variance > 10.f) {

			/*pm->kalman_P[0] += 100.f;
			  pm->kalman_P[2] += 100.f;
			  pm->kalman_P[5] += 2.f;
			  pm->kalman_P[9] += 1000.f;
			  pm->kalman_P[14] += 1.f;
			  pm->kalman_P[20] += 1.f;*/

			pm->residual_variance = 0.f;
		}
	}
}

void pmc_feedback(pmc_t *pm, int xA, int xB)
{
	float		iA, iB;

	iA = pm->scal_A[1] * (xA - 2048) + pm->scal_A[0];
	iB = pm->scal_B[1] * (xB - 2048) + pm->scal_B[0];

	pmc_FSM(pm, iA, iB);

	if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

		kalman_measurement_update(pm, iA, iB);
		i_control(pm);
		kalman_time_update(pm);

		if (pm->m_bitmode & PMC_BIT_SPEED_CONTROL_LOOP) {

			w_control(pm);
		}
		else if (pm->m_bitmode & PMC_BIT_POSITION_CONTROL_LOOP) {
		}

		pmc_underway(pm);
		pm->t_clock = (pm->t_clock < 4) ? pm->t_clock + 1 : 0;
	}
}

void pmc_voltage(pmc_t *pm, int xU)
{
	float		uS;

	uS = pm->scal_U[1] * xU + pm->scal_U[0];

	pm->const_U += (uS - pm->const_U) * 2e-2f;
	pm->const_IU = 1.f / pm->const_U;
}

void pmc_tune(pmc_t *pm)
{
	pm->i_slew_rate_D = .2f * pm->const_U / pm->const_Ld;
	pm->i_slew_rate_Q = .2f * pm->const_U / pm->const_Lq;
	pm->i_gain_P_D = (.5f * pm->const_Ld * pm->freq_hz - pm->const_R);
	pm->i_gain_I_D = 5e-2f * pm->const_Ld * pm->freq_hz;
	pm->i_gain_P_Q = (.5f * pm->const_Lq * pm->freq_hz - pm->const_R);
	pm->i_gain_I_Q = 5e-2f * pm->const_Lq * pm->freq_hz;
}

