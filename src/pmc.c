/*
   Phobia DC Motor Controller for RC and robotics.
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
	pm->m_bitmask = PMC_BIT_QAXIS_DRIFT
		| PMC_BIT_FREQUENCY_INJECTION;
	pm->m_bitmode = 0;
	pm->m_state = PMC_STATE_IDLE;
	pm->m_phase = 0;
	pm->m_errno = 0;

	pm->T_drift = .1f;
	pm->T_hold = .5f;
	pm->T_rohm = .5f;
	pm->T_sine = .5f;
	pm->T_bemf = .5f;
	pm->T_end = .1f;

	pm->i_hold = 1.f;
	pm->i_sine = 1.f;
	pm->i_offset_D = 0.f;
	pm->i_offset_Q = 0.f;
	pm->freq_sine_hz = 2000.f;

	pm->conv_A[0] = 0.f;
	pm->conv_A[1] = 1.332e-2f;
	pm->conv_B[0] = 0.f;
	pm->conv_B[1] = 1.332e-2f;
	pm->conv_U[0] = 0.f;
	pm->conv_U[1] = 6.592e-3f;

	pm->kalman_Q[0] = 1e-4f;
	pm->kalman_Q[1] = 1e-4f;
	pm->kalman_Q[2] = 1e-6f;
	pm->kalman_Q[3] = 1e-6f;
	pm->kalman_Q[4] = 1e-5f;
	pm->kalman_Q[5] = 1e-7f;
	pm->kalman_R = 4e-2f;

	pm->const_U = 0.f;
	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Q = 0.f;
	pm->const_Zp = 1;

	pm->i_maximal = 5.f;
	pm->i_power_consumption_maximal = 200.f;
	pm->i_power_regeneration_maximal = - 20.f;
	pm->i_set_point_D = 0.f;
	pm->i_set_point_Q = 0.f;
	pm->i_integral_D = 0.f;
	pm->i_integral_Q = 0.f;
	pm->i_KP = 2e-2f;
	pm->i_KI = 3e-3f;

	pm->h_freq_hz = 500.f;
	pm->h_set_point = .5f;

	pm->w_clock = 0;
	pm->w_clock_scale = 10;
	pm->w_maximal = 20944.f;
	pm->w_low_threshold = 524.f;
	pm->w_low_hysteresis = 52.f;
	pm->w_set_point = 0.f;
	pm->w_integral = 0.f;
	pm->w_KP = 2e-3f;
	pm->w_KI = 2e-6f;
}

static void
pm_equation(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ;

	uD = X[2] * pm->vsi_X + X[3] * pm->vsi_Y;
	uQ = X[2] * pm->vsi_Y - X[3] * pm->vsi_X;

	/* Electrical equations.
	 * */
	D[0] = (uD - pm->const_R * X[0] + pm->const_Lq * X[4] * X[1]) * pm->const_ILd;
	D[1] = (uQ - pm->const_R * X[1] - pm->const_Ld * X[4] * X[0]
			- pm->const_E * X[4] + pm->drift_Q) * pm->const_ILq;

	/* Mechanical equations.
	 * */
	D[2] = X[4];
	D[3] = pm->const_Zp * (1.5f * pm->const_Zp
			* (pm->const_E - (pm->const_Lq - pm->const_Ld) * X[0])
			* X[1] - pm->var_M) * pm->const_IJ;
}

static void
pm_update(pmc_t *pm)
{
	float		*X = pm->kalman_X;
	float		D1[4], D2[4], X2[5];
	float		dT;

	/* Second-order ODE solver.
	 * */

	pm_equation(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	rotatef(X2 + 2, D1[2] * dT, X + 2);
	X2[4] = X[4] + D1[3] * dT;

	pm_equation(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	rotatef(X + 2, (D1[2] + D2[2]) * dT, X + 2);
	X[4] += (D1[3] + D2[3]) * dT;
}

static void
kalman_measurement_update(pmc_t *pm, float iA, float iB)
{
	float		*X = pm->kalman_X, *P = pm->kalman_P;
	float		PC[12], S[3], L[3], K[12];
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
	PC[10] = P[15] - P[17] * X[1];
	PC[11] = P[16] + P[17] * X[0];

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
	K[11] = (PC[11] - PC[10] * L[1]) * L[2];
	K[10] = PC[10] * L[0] - K[11] * L[1];

	/* X = X + K * e;
	 * */
	X[0] += K[0] * eD + K[1] * eQ;
	X[1] += K[2] * eD + K[3] * eQ;
	dR = K[4] * eD + K[5] * eQ;
	dR = (dR < - 1.f) ? - 1.f : (dR > 1.f) ? 1.f : dR;
	rotatef(X + 2, dR, X + 2);
	X[4] += K[6] * eD + K[7] * eQ;
	pm->var_M += K[8] * eD + K[9] * eQ;
	pm->drift_Q += K[10] * eD + K[11] * eQ;

	//pm->Qd = (pm->Qd < - 1.f) ? - 1.f : (pm->Qd > 1.f) ? 1.f : pm->Qd;

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
	P[15] -= K[10] * PC[0] + K[11] * PC[1];
	P[16] -= K[10] * PC[2] + K[11] * PC[3];
	P[17] -= K[10] * PC[4] + K[11] * PC[5];
	P[18] -= K[10] * PC[6] + K[11] * PC[7];
	P[19] -= K[10] * PC[8] + K[11] * PC[9];
	P[20] -= K[10] * PC[10] + K[11] * PC[11];

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
	float		A[12], PA[24];
	float		dT, dTLd, dTLq, dTIJ, Zp2;

	dT = pm->dT;
	dTLd = dT * pm->const_ILd;
	dTLq = dT * pm->const_ILq;
	dTIJ = dT * pm->const_IJ;
	Zp2 = 1.5f * pm->const_Zp * pm->const_Zp * dTIJ;

	A[0] = 1.f - pm->const_R * dTLd;
	A[1] = pm->temp_A[4] * pm->const_Lq * dTLd;
	A[2] = (pm->temp_A[2] * pm->vsi_Y - pm->temp_A[3] * pm->vsi_X) * dTLd;
	A[3] = pm->temp_A[1] * pm->const_Lq * dTLd;

	A[4] = - pm->temp_A[4] * pm->const_Ld * dTLq;
	A[5] = 1.f - pm->const_R * dTLq;
	A[6] = - (pm->temp_A[2] * pm->vsi_X + pm->temp_A[3] * pm->vsi_Y) * dTLq;
	A[7] = - (pm->const_E + pm->temp_A[0] * pm->const_Ld) * dTLq;
	A[8] = dTLq;

	A[9] = pm->temp_A[1] * (pm->const_Ld - pm->const_Lq) * Zp2;
	A[10] = Zp2 * (pm->const_E - pm->temp_A[0] * (pm->const_Lq - pm->const_Ld));
	A[11] = - pm->const_Zp * dTIJ;

	/* P = A * P * A' + Q.
	 * */
	PA[0] = P[0] * A[0] + P[1] * A[1] + P[3] * A[2] + P[6] * A[3];
	PA[1] = P[0] * A[4] + P[1] * A[5] + P[3] * A[6] + P[6] * A[7]
		+ P[15] * A[8];
	PA[2] = P[3] + P[6] * dT;
	PA[3] = P[0] * A[9] + P[1] * A[10] + P[6] + P[10] * A[11];

	PA[4] = P[1] * A[0] + P[2] * A[1] + P[4] * A[2] + P[7] * A[3];
	PA[5] = P[1] * A[4] + P[2] * A[5] + P[4] * A[6] + P[7] * A[7]
		+ P[16] * A[8];
	PA[6] = P[4] + P[7] * dT;
	PA[7] = P[1] * A[9] + P[2] * A[10] + P[7] + P[11] * A[11];

	PA[8] = P[3] * A[0] + P[4] * A[1] + P[5] * A[2] + P[8] * A[3];
	PA[9] = P[3] * A[4] + P[4] * A[5] + P[5] * A[6] + P[8] * A[7]
		+ P[17] * A[8];
	PA[10] = P[5] + P[8] * dT;

	PA[12] = P[6] * A[0] + P[7] * A[1] + P[8] * A[2] + P[9] * A[3];
	PA[13] = P[6] * A[4] + P[7] * A[5] + P[8] * A[6] + P[9] * A[7]
		+ P[18] * A[8];
	PA[14] = P[8] + P[9] * dT;
	PA[15] = P[6] * A[9] + P[7] * A[10] + P[9] + P[13] * A[11];

	PA[16] = P[10] * A[0] + P[11] * A[1] + P[12] * A[2] + P[13] * A[3];
	PA[17] = P[10] * A[4] + P[11] * A[5] + P[12] * A[6] + P[13] * A[7]
		+ P[19] * A[8];
	PA[18] = P[12] + P[13] * dT;
	PA[19] = P[10] * A[9] + P[11] * A[10] + P[13] + P[14] * A[11];

	PA[20] = P[15] * A[0] + P[16] * A[1] + P[17] * A[2] + P[18] * A[3];
	PA[21] = P[15] * A[4] + P[16] * A[5] + P[17] * A[6] + P[18] * A[7]
		+ P[20] * A[8];
	PA[22] = P[17] + P[18] * dT;
	PA[23] = P[15] * A[9] + P[16] * A[10] + P[18] + P[19] * A[11];

	P[0] = A[0] * PA[0] + A[1] * PA[4] + A[2] * PA[8] + A[3] * PA[12]
		+ pm->kalman_Q[0];
	P[1] = A[0] * PA[1] + A[1] * PA[5] + A[2] * PA[9] + A[3] * PA[13];
	P[2] = A[4] * PA[1] + A[5] * PA[5] + A[6] * PA[9] + A[7] * PA[13]
		+ A[8] * PA[21] + pm->kalman_Q[1];
	P[3] = PA[8] + dT * PA[12];
	P[4] = PA[9] + dT * PA[13];
	P[5] = PA[10] + dT * PA[14] + pm->kalman_Q[2];
	P[6] = A[9] * PA[0] + A[10] * PA[4] + PA[12] + A[11] * PA[16];
	P[7] = A[9] * PA[1] + A[10] * PA[5] + PA[13] + A[11] * PA[17];
	P[8] = A[9] * PA[2] + A[10] * PA[6] + PA[14] + A[11] * PA[18];
	P[9] = A[9] * PA[3] + A[10] * PA[7] + PA[15] + A[11] * PA[19]
		+ pm->kalman_Q[3];
	P[10] = PA[16];
	P[11] = PA[17];
	P[12] = PA[18];
	P[13] = PA[19];
	P[14] += pm->kalman_Q[4];
	P[15] = PA[20];
	P[16] = PA[21];
	P[17] = PA[22];
	P[18] = PA[23];
	P[20] += (pm->m_bitmode & PMC_BIT_QAXIS_DRIFT) ? pm->kalman_Q[5] : 0.f;
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
	const float	uMAX = .667f;
	float		eD, eQ, uX, uY;
	float		uD, uQ, wP, S;

	/*
	 * */
	pm->i_set_point_D = (pm->i_set_point_D > pm->i_maximal) ? pm->i_maximal :
		(pm->i_set_point_D < - pm->i_maximal) ? - pm->i_maximal : pm->i_set_point_D;
	pm->i_set_point_Q = (pm->i_set_point_Q > pm->i_maximal) ? pm->i_maximal :
		(pm->i_set_point_Q < - pm->i_maximal) ? - pm->i_maximal : pm->i_set_point_Q;

	uD = pm->temp_A[2] * pm->vsi_X + pm->temp_A[3] * pm->vsi_Y;
	uQ = pm->temp_A[2] * pm->vsi_Y - pm->temp_A[3] * pm->vsi_X;

	pm->i_power_watt = pm->kalman_X[0] * uD + pm->kalman_X[1] * uQ;
	wP = pm->i_set_point_D * uD + pm->i_set_point_Q * uQ;

	if (wP > pm->i_power_consumption_maximal) {

		S = pm->i_power_consumption_maximal / wP;
		pm->i_set_point_D *= S;
		pm->i_set_point_Q *= S;
	}
	else if (wP < pm->i_power_regeneration_maximal) {

		S = pm->i_power_regeneration_maximal / wP;
		pm->i_set_point_D *= S;
		pm->i_set_point_Q *= S;
	}

	/* Obtain residual.
	 * */
	eD = pm->i_set_point_D - pm->kalman_X[0];
	eQ = pm->i_set_point_Q - pm->kalman_X[1];

	if (pm->m_bitmode & PMC_BIT_DIRECT_INJECTION) {

		eD += 1.f * fabsf(eQ);
	}

	if (pm->m_bitmode & PMC_BIT_FREQUENCY_INJECTION) {

		eD += pm->h_CS[1] * pm->h_set_point;
		rotatef(pm->h_CS, 6.2831853f * pm->h_freq_hz * pm->dT, pm->h_CS);
	}

	/* PI controller.
	 * */
	uD = pm->i_KP * eD;
	uQ = pm->i_KP * eQ;

	if (fabsf(uD) < uMAX) {

		pm->i_integral_D += pm->i_KI * eD;
		pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
			(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
		uD += pm->i_integral_D;
	}

	if (fabsf(uQ) < uMAX) {

		pm->i_integral_Q += pm->i_KI * eQ;
		pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
			(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
		uQ += pm->i_integral_Q;
	}

	uX = pm->kalman_X[2] * uD - pm->kalman_X[3] * uQ;
	uY = pm->kalman_X[3] * uD + pm->kalman_X[2] * uQ;

	vsi_control(pm, uX, uY);
}

static void
w_control(pmc_t *pm)
{
	float		eW, iSP;

	/* Obtain residual.
	 * */
	eW = pm->w_set_point - pm->kalman_X[4];

	/* Speed PI regulator.
	 * */
	pm->w_integral += pm->w_KI * eW;
	pm->w_integral = (pm->w_integral > pm->i_maximal) ? pm->i_maximal :
		(pm->w_integral < - pm->i_maximal) ? - pm->i_maximal : pm->w_integral;
	iSP = pm->w_KP * eW + pm->w_integral;

	/* Current limit.
	 * */
	iSP = (iSP > pm->i_maximal) ? pm->i_maximal :
		(iSP < - pm->i_maximal) ? - pm->i_maximal : iSP;

	pm->i_set_point_D = 0.f;
	pm->i_set_point_Q = iSP;
}

static void
pmc_FSM(pmc_t *pm, float iA, float iB)
{
	float			tA, tB;

	switch (pm->m_state) {

		case PMC_STATE_IDLE:

			if (pm->m_request != PMC_STATE_IDLE) {

				if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

					if (pm->m_request == PMC_STATE_BEMF)
						pm->m_state = PMC_STATE_BEMF;
					else if (pm->m_request == PMC_STATE_END)
						pm->m_state = PMC_STATE_END;
					else
						pm->m_request = 0;
				}
				else {
					if (pm->m_request == PMC_STATE_SPINUP
							|| pm->m_request == PMC_STATE_HOLD
							|| pm->m_request == PMC_STATE_SINE
							|| pm->m_request == PMC_STATE_LINEAR)
						pm->m_state = PMC_STATE_DRIFT;
					else
						pm->m_request = 0;
				}
			}

			if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

				tA = fabsf(pm->kalman_X[4]);
				tB = tA + pm->w_low_hysteresis;
				tA = tA - pm->w_low_hysteresis;

				if (tA > pm->w_low_threshold) {

					pm->m_bitmode &= ~(PMC_BIT_DIRECT_INJECTION
							| PMC_BIT_FREQUENCY_INJECTION);
					pm->m_bitmode |= (PMC_BIT_QAXIS_DRIFT
							| PMC_BIT_EFFICIENT_MODULATION)
						& pm->m_bitmask;
				}
				else if (tB < pm->w_low_threshold) {

					pm->m_bitmode &= ~(PMC_BIT_QAXIS_DRIFT
							| PMC_BIT_EFFICIENT_MODULATION);
					pm->m_bitmode |= (PMC_BIT_DIRECT_INJECTION
							| PMC_BIT_FREQUENCY_INJECTION)
						& pm->m_bitmask;

					pm->kalman_P[15] = 0.f;
					pm->kalman_P[16] = 0.f;
					pm->kalman_P[17] = 0.f;
					pm->kalman_P[18] = 0.f;
					pm->kalman_P[19] = 0.f;
					pm->kalman_P[20] = 0.f;
				}
			}
			break;

		case PMC_STATE_DRIFT:

			if (pm->m_phase == 0) {

				vsi_control(pm, 0.f, 0.f);
				pm->pZ(7);

				pm->temp_A[0] = 0.f;
				pm->temp_A[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = 64;

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
					pm->conv_A[0] += pm->temp_A[0] / pm->t_end;
					pm->conv_B[0] += pm->temp_A[1] / pm->t_end;

					if (pm->m_phase == 1) {

						pm->temp_A[0] = 0.f;
						pm->temp_A[1] = 0.f;

						pm->t_value = 0;
						pm->t_end = pm->freq_hz * pm->T_drift;

						pm->m_phase++;
					}
					else {
						pm->pZ(0);

						if (pm->m_request == PMC_STATE_SPINUP)
							pm->m_state = PMC_STATE_HOLD;
						else if (pm->m_request == PMC_STATE_HOLD)
							pm->m_state = PMC_STATE_HOLD;
						else if (pm->m_request == PMC_STATE_SINE)
							pm->m_state = PMC_STATE_SINE;
						else if (pm->m_request == PMC_STATE_LINEAR)
							pm->m_state = PMC_STATE_LINEAR;
						else
							pm->m_state = PMC_STATE_END;

						pm->m_phase = 0;
					}
				}
			}
			break;

		case PMC_STATE_HOLD:

			if (pm->m_phase == 0) {

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

						pm->temp_A[0] = 0.f;

						pm->t_value = 0;
						pm->t_end = pm->freq_hz * pm->T_rohm;

						pm->m_phase++;
					}
					else {
						/* Winding Resistance.
						 * */
						pm->const_R += pm->temp_A[0] / pm->t_end
							/ pm->i_set_point_D;

						if (pm->m_request == PMC_STATE_SPINUP)
							pm->m_state = PMC_STATE_SPINUP;
						else
							pm->m_state = PMC_STATE_END;

						pm->m_phase = 0;
					}
				}
			}
			break;

		case PMC_STATE_SINE:

			if (pm->m_phase == 0) {

				pm->kalman_X[2] = 1.f;
				pm->kalman_X[3] = 0.f;
				pm->kalman_X[4] = 6.2831853f * pm->freq_sine_hz / pm->freq_hz;

				pm->kalman_P[0] = 0.f;
				pm->kalman_P[1] = 0.f;
				pm->kalman_P[2] = 0.f;
				pm->kalman_P[3] = 0.f;
				pm->kalman_P[4] = 0.f;
				pm->kalman_P[5] = 0.f;
				pm->kalman_P[6] = 0.f;
				pm->kalman_P[7] = 0.f;

				tA = 6.2831853f * pm->const_Ld * pm->freq_sine_hz;
				tA = pm->const_R * pm->const_R + tA * tA;
				pm->temp_A[0] = pm->i_sine * sqrtf(tA);
				tA = 6.2831853f * pm->const_Lq * pm->freq_sine_hz;
				tA = pm->const_R * pm->const_R + tA * tA;
				pm->temp_A[1] = pm->i_sine * sqrtf(tA);
				pm->temp_A[2] = pm->i_offset_D * pm->const_R;
				pm->temp_A[3] = pm->i_offset_Q * pm->const_R;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_sine;

				pm->m_phase++;
			}
			else {
				pm->kalman_X[0] = iA;
				pm->kalman_X[1] = .57735027f * iA + 1.1547005f * iB;

				pm->kalman_P[0] += pm->kalman_X[0] * pm->kalman_X[2];
				pm->kalman_P[1] += pm->kalman_X[0] * pm->kalman_X[3];
				pm->kalman_P[2] += pm->vsi_X * pm->kalman_X[2];
				pm->kalman_P[3] += pm->vsi_X * pm->kalman_X[3];
				pm->kalman_P[4] += pm->kalman_X[1] * pm->kalman_X[2];
				pm->kalman_P[5] += pm->kalman_X[1] * pm->kalman_X[3];
				pm->kalman_P[6] += pm->vsi_Y * pm->kalman_X[2];
				pm->kalman_P[7] += pm->vsi_Y * pm->kalman_X[3];

				rotatef(pm->kalman_X + 2, pm->kalman_X[4], pm->kalman_X + 2);

				tA = pm->temp_A[2] + pm->temp_A[0] * pm->kalman_X[2];
				tA /= pm->const_U;
				tB = pm->temp_A[3] + pm->temp_A[1] * pm->kalman_X[3];
				tB /= pm->const_U;

				vsi_control(pm, tA, tB);

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					/* Impedance on X (D) axis.
					 * */
					pm->temp_A[0] = pm->kalman_P[2] * pm->kalman_P[0]
						+ pm->kalman_P[3] * pm->kalman_P[1];
					pm->temp_A[1] = pm->kalman_P[2] * pm->kalman_P[1]
						- pm->kalman_P[3] * pm->kalman_P[0];

					tA = pm->kalman_P[0] * pm->kalman_P[0]
						+ pm->kalman_P[1] * pm->kalman_P[1];
					pm->temp_A[0] /= tA;
					pm->temp_A[1] /= tA;

					/* Impedance on Y (Q) axis.
					 * */
					pm->temp_A[2] = pm->kalman_P[6] * pm->kalman_P[4]
						+ pm->kalman_P[7] * pm->kalman_P[5];
					pm->temp_A[3] = pm->kalman_P[6] * pm->kalman_P[5]
						- pm->kalman_P[7] * pm->kalman_P[4];

					tA = pm->kalman_P[4] * pm->kalman_P[4]
						+ pm->kalman_P[5] * pm->kalman_P[5];
					pm->temp_A[2] /= tA;
					pm->temp_A[3] /= tA;

					{
						tA = 6.2831853f * pm->freq_sine_hz;
						pm->const_Ld = pm->temp_A[1] / tA;
						pm->const_Lq = pm->temp_A[3] / tA;

						pm->const_ILd = 1.f / pm->const_Ld;
						pm->const_ILq = 1.f / pm->const_Lq;
					}

					pm->m_state = PMC_STATE_END;
					pm->m_phase = 0;
				}
			}
			break;

		case PMC_STATE_BEMF:

			if (pm->m_phase == 0) {

				pm->temp_A[0] = 0.f;
				pm->temp_A[1] = 0.f;

				pm->t_value = 0;
				pm->t_end = pm->freq_hz * pm->T_bemf;

				pm->m_phase++;
			}
			else {
				pm->temp_A[0] += pm->kalman_X[4];
				pm->temp_A[1] += pm->drift_Q;

				pm->t_value++;

				if (pm->t_value < pm->t_end) ;
				else {
					/* BEMF constant.
					 * */
					pm->const_E += - pm->temp_A[1] / pm->temp_A[0];

					pm->m_request = 0;
					pm->m_state = PMC_STATE_IDLE;
					pm->m_phase = 0;
				}
			}

			break;

		case PMC_STATE_LINEAR:
			break;

		case PMC_STATE_SPINUP:

			pm->m_bitmode |= PMC_BIT_KALMAN_FILTER;

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
			pm->var_M = 0.f;

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
			pm->kalman_P[15] = 0.f;
			pm->kalman_P[16] = 0.f;
			pm->kalman_P[17] = 0.f;
			pm->kalman_P[18] = 0.f;
			pm->kalman_P[19] = 0.f;

			pm->kalman_P[0] = 40000.f;
			pm->kalman_P[2] = 40000.f;
			pm->kalman_P[5] = 1.f;
			pm->kalman_P[9] = 0.f;
			pm->kalman_P[14] = 0.f;
			pm->kalman_P[20] = 0.f;

			pm->h_CS[0] = 1.;
			pm->h_CS[1] = 0.;

			pm->i_set_point_D = 0.f;
			pm->i_set_point_Q = 5.f;
			pm->w_set_point = 0.f;

			pm->m_request = 0;
			pm->m_state = PMC_STATE_IDLE;
			pm->m_phase = 0;
			break;

		case PMC_STATE_END:

			if (pm->m_phase == 0) {

				if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

					pm->m_bitmode &= ~(PMC_BIT_SPEED_CONTROL_LOOP);

					pm->i_set_point_D = 0.f;
					pm->i_set_point_Q = 0.f;
				}
				else {

					vsi_control(pm, 0.f, 0.f);
				}

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

void pmc_feedback(pmc_t *pm, int xA, int xB)
{
	float		iA, iB;

	iA = pm->conv_A[1] * (xA - 2048) + pm->conv_A[0];
	iB = pm->conv_B[1] * (xB - 2048) + pm->conv_B[0];

	pmc_FSM(pm, iA, iB);

	if (pm->m_bitmode & PMC_BIT_KALMAN_FILTER) {

		kalman_measurement_update(pm, iA, iB);
		i_control(pm);

		if (pm->m_bitmode & PMC_BIT_SPEED_CONTROL_LOOP) {

			pm->w_clock++;

			if (pm->w_clock >= pm->w_clock_scale) {

				pm->w_clock = 0;
				w_control(pm);
			}
		}

		kalman_time_update(pm);
	}
}

void pmc_voltage(pmc_t *pm, int xU)
{
	float		uS;

	uS = pm->conv_U[1] * xU + pm->conv_U[0];

	pm->const_U += (uS - pm->const_U) * 2e-2f;
}

void pmc_gain_tune(pmc_t *pm)
{
	float		min_L, KP, KI;

	min_L = (pm->const_Ld < pm->const_Lq) ? pm->const_Ld : pm->const_Lq;
	KP = (.5f * min_L * pm->freq_hz - pm->const_R) / pm->const_U;
	KI = 5e-2f * min_L * pm->freq_hz / pm->const_U;

	pm->i_KP = KP;
	pm->i_KI = KI;
}

