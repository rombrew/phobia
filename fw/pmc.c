/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#define KPI			3.1415927f

static float
ksin(float x)
{
	float		u;
	int		s = 0;

	if (x < 0.f) {

		x = - x;
		s = 1;
	}

	if (x > KPI / 2.f) {

		x = KPI - x;
	}

	u = -1.3741951e-4f;
	u = -2.0621440e-4f + u * x;
	u =  8.6430385e-3f + u * x;
	u = -2.4749696e-4f + u * x;
	u = -1.6655975e-1f + u * x;
	u = -2.3177562e-5f + u * x;
	u =  1.0000021e+0f + u * x;
	u = -4.0553596e-8f + u * x;

	return s ? - u : u;
}

static float
kcos(float x)
{
	float		u;
	int		s = 0;

	if (x < 0.f) {

		x = - x;
	}

	if (x > KPI / 2.f) {

		x = KPI - x;
		s = 1;
	}

	u =  1.3804255e-4f;
	u = -1.7206567e-3f + u * x;
	u =  4.2851990e-4f + u * x;
	u =  4.1352723e-2f + u * x;
	u =  1.2810877e-4f + u * x;
	u = -5.0002667e-1f + u * x;
	u =  2.2899566e-6f + u * x;
	u =  9.9999996e-1f + u * x;

	return s ? - u : u;
}

static void
dROT(float B[2], float R, const float A[2])
{
	float		Q, S, C, X, Y;

	Q = R * R;
	S = R - .16666667f * Q * R;
	C = 1.f - .5f * Q;

	X = C * A[0] - S * A[1];
	Y = S * A[0] + C * A[1];

	Q = (3.f - X * X - Y * Y) * .5f;
	B[0] = X * Q;
	B[1] = Y * Q;
}

void pmcEnable(pmc_t *pm)
{
	pm->dT = 1.f / pm->hzF;

	/* Default configuration.
	 * */
	pm->sTdrift = .1f;
	pm->sThold = .5f;
	pm->sTend = .1f;

	pm->pwmEPS = 1e-2f;
	pm->pwmMP = (int) (2e-7f * pm->hzF * pm->pwmR + .5f);

	/* ADC conversion constants.
	 * */
	pm->cA0 = 0.f;
	pm->cA1 = .01464844f;
	pm->cB0 = 0.f;
	pm->cB1 = .01464844f;
	pm->cU0 = 0.f;
	pm->cU1 = .00725098f;

	/* Model covariance.
	 * */
	pm->kQ[0] = 1e-2;
	pm->kQ[1] = 1e-2;
	pm->kQ[2] = 1e-4;
	pm->kQ[3] = 1e-4;
	pm->kQ[4] = 1e-6;
	pm->kQ[5] = 1e-12;

	pm->kQ[6] = 1e-2;
	pm->kQ[7] = 1e-16;
	pm->kQ[8] = 0e-10;

	/* Measurement covariance.
	 * */
	pm->kR = 1e-2;

	/* PI constants.
	 * */
	pm->iKP = 1e-2f;
	pm->iKI = 1e-3f;
	pm->wKP = 1e-1f;
	pm->wKI = 0e-3f;

	pm->iMAX = 20.f;
	pm->wMAX = 2e+4f;
}

static void
dEq(pmc_t *pm, float D[], float X[])
{
	float		uD, uQ;

	/* Transform voltage to DQ axes.
	 * */
	uD = X[2] * pm->uX + X[3] * pm->uY;
	uQ = X[2] * pm->uY - X[3] * pm->uX;

	/* Electrical equations.
	 * */
	D[0] = (uD - pm->R * X[0] + pm->Lq * X[4] * X[1]) / pm->Ld;
	D[1] = (uQ - pm->R * X[1] - pm->Ld * X[4] * X[0] - pm->E * X[4]) / pm->Lq;

	/* Mechanical equations.
	 * */
	D[2] = X[4];
	D[3] = pm->Zp * (1.5f * pm->Zp * (pm->E - (pm->Lq - pm->Ld) * X[0]) * X[1] - pm->M) / pm->J;
}

static void
sFC(pmc_t *pm)
{
	float		*X = pm->kX;
	float		D1[4], D2[4], X2[5];
	float		dT;

	/* Second-order ODE solver.
	 * */

	dEq(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	dROT(X2 + 2, D1[2] * dT, X + 2);
	X2[4] = X[4] + D1[3] * dT;

	dEq(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	dROT(X + 2, (D1[2] + D2[2]) * dT, X + 2);
	X[4] += (D1[3] + D2[3]) * dT;
}

static void
kFB(pmc_t *pm, float iA, float iB)
{
	float		*X = pm->kX, *P = pm->kP;
	float		iX, iY, xA, xB, eA, eB, dR;
	float		C[6], PC[12], S[3], iS[3], K[12], D;

	/* Get model output.
	 * */
	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	xA = iX - pm->zA;
	xB = - .5f * iX + .8660254f * iY - pm->zB;

	/* Obtain residual.
	 * */
	eA = iA - xA;
	eB = iB - xB;

	/* Output Jacobian matrix.
	 * */
	C[0] = X[2];
	C[1] = - X[3];
	C[2] = - X[2] * X[1] - X[3] * X[0];
	C[3] = - .5f * X[2] + .8660254f * X[3];
	C[4] = .5f * X[3] + .8660254f * X[2];
	C[5] = - .5f * (C[2]) + .8660254f * (- X[3] * X[1] + X[2] * X[0]);

	if (1) {

		/* S = C * P * C' + R;
		 * */
		PC[0] = P[0] * C[0] + P[1] * C[1] + P[3] * C[2];
		PC[1] = P[0] * C[3] + P[1] * C[4] + P[3] * C[5];
		PC[2] = P[1] * C[0] + P[2] * C[1] + P[4] * C[2];
		PC[3] = P[1] * C[3] + P[2] * C[4] + P[4] * C[5];
		PC[4] = P[3] * C[0] + P[4] * C[1] + P[5] * C[2];
		PC[5] = P[3] * C[3] + P[4] * C[4] + P[5] * C[5];
		PC[6] = P[6] * C[0] + P[7] * C[1] + P[8] * C[2];
		PC[7] = P[6] * C[3] + P[7] * C[4] + P[8] * C[5];
		PC[8] = P[10] * C[0] + P[11] * C[1] + P[12] * C[2];
		PC[9] = P[10] * C[3] + P[11] * C[4] + P[12] * C[5];
		PC[10] = P[15] * C[0] + P[16] * C[1] + P[17] * C[2];
		PC[11] = P[15] * C[3] + P[16] * C[4] + P[17] * C[5];

		S[0] = C[0] * PC[0] + C[1] * PC[2] + C[2] * PC[4] + pm->kR;
		S[1] = C[0] * PC[1] + C[1] * PC[3] + C[2] * PC[5];
		S[2] = C[3] * PC[1] + C[4] * PC[3] + C[5] * PC[5] + pm->kR;

		D = S[0] * S[2] - S[1] * S[1];
		iS[0] = S[2] / D;
		iS[1] = - S[1] / D;
		iS[2] = S[0] / D;

		/* K = P * C' / S;
		 * */
		K[0] = PC[0] * iS[0] + PC[1] * iS[1];
		K[1] = PC[0] * iS[1] + PC[1] * iS[2];
		K[2] = PC[2] * iS[0] + PC[3] * iS[1];
		K[3] = PC[2] * iS[1] + PC[3] * iS[2];
		K[4] = PC[4] * iS[0] + PC[5] * iS[1];
		K[5] = PC[4] * iS[1] + PC[5] * iS[2];
		K[6] = PC[6] * iS[0] + PC[7] * iS[1];
		K[7] = PC[6] * iS[1] + PC[7] * iS[2];
		K[8] = PC[8] * iS[0] + PC[9] * iS[1];
		K[9] = PC[8] * iS[1] + PC[9] * iS[2];
		K[10] = PC[10] * iS[0] + PC[11] * iS[1];
		K[11] = PC[10] * iS[1] + PC[11] * iS[2];

		/* X = X + K * e;
		 * */
		X[0] += K[0] * eA + K[1] * eB;
		X[1] += K[2] * eA + K[3] * eB;
		dR = K[4] * eA + K[5] * eB;
		dR = (dR < -1.f) ? -1.f : (dR > 1.f) ? 1.f : dR;
		dROT(X + 2, dR, X + 2);
		X[4] += K[6] * eA + K[7] * eB;
		pm->M += K[8] * eA + K[9] * eB;
		pm->E += K[10] * eA + K[11] * eB;

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
	}
	else if (1) {

		/* TODO: Single output case */
	}

	/* Temporal.
	 * */
	pm->kT[0] = X[0];
	pm->kT[1] = X[1];
	pm->kT[2] = X[2];
	pm->kT[3] = X[3];
	pm->kT[4] = X[4];

	/* Update state estimate.
	 * */
	sFC(pm); // 725 ticks
}

static void
kAT(pmc_t *pm)
{
	float		*X = pm->kX, *P = pm->kP, *Q = pm->kQ;
	float		dT, iD, iQ, rX, rY, wR, L;
	float		dToLd, dToLq, dToJ, Zp2;
	float		A[13], PA[36];

	dT = pm->dT;

	/* Average variables.
	 * */
	iD = (pm->kT[0] + X[0]) * .5f;
	iQ = (pm->kT[1] + X[1]) * .5f;
	rX = (pm->kT[2] + X[2]) * .5f;
	rY = (pm->kT[3] + X[3]) * .5f;
	wR = (pm->kT[4] + X[4]) * .5f;

	L = (3.f - rX * rX - rY * rY) * .5f;
	rX *= L;
	rY *= L;

	/* Common subexpressions.
	 * */
	dToLd = dT / pm->Ld;
	dToLq = dT / pm->Lq;
	dToJ = dT / pm->J;
	Zp2 = 1.5f * pm->Zp * pm->Zp * dToJ;

	/* Transition Jacobian matrix.
	 * */
	A[0] = 1.f - pm->R * dToLd;
	A[1] = wR * pm->Lq * dToLd;
	A[2] = (rX * pm->uY - rY * pm->uX) * dToLd;
	A[3] = iQ * pm->Lq * dToLd;

	A[4] = - wR * pm->Ld * dToLq;
	A[5] = 1.f - pm->R * dToLq;
	A[6] = (- rY * pm->uY - rX * pm->uX) * dToLq;
	A[7] = (- pm->E - iD * pm->Ld) * dToLq;
	A[8] = - wR * dToLq;

	A[9] = iQ * (pm->Ld - pm->Lq) * Zp2;
	A[10] = Zp2 * (pm->E - iD * (pm->Lq - pm->Ld));
	A[11] = - pm->Zp * dToJ;
	A[12] = iQ * Zp2;

	/* P = A * P * A' + Q.
	 * */
	PA[0] = P[0] * A[0] + P[1] * A[1] + P[3] * A[2] + P[6] * A[3];
	PA[1] = P[0] * A[4] + P[1] * A[5] + P[3] * A[6] + P[6] * A[7] + P[15] * A[8];
	PA[2] = P[3] + P[6] * dT;
	PA[3] = P[0] * A[9] + P[1] * A[10] + P[6] + P[10] * A[11] + P[15] * A[12];
	PA[4] = P[10];
	PA[5] = P[15];

	PA[6] = P[1] * A[0] + P[2] * A[1] + P[4] * A[2] + P[7] * A[3];
	PA[7] = P[1] * A[4] + P[2] * A[5] + P[4] * A[6] + P[7] * A[7] + P[16] * A[8];
	PA[8] = P[4] + P[7] * dT;
	PA[9] = P[1] * A[9] + P[2] * A[10] + P[7] + P[11] * A[11] + P[16] * A[12];
	PA[10] = P[11];
	PA[11] = P[16];

	PA[12] = P[3] * A[0] + P[4] * A[1] + P[5] * A[2] + P[8] * A[3];
	PA[13] = P[3] * A[4] + P[4] * A[5] + P[5] * A[6] + P[8] * A[7] + P[17] * A[8];
	PA[14] = P[5] + P[8] * dT;
	PA[15] = P[3] * A[9] + P[4] * A[10] + P[8] + P[12] * A[11] + P[17] * A[12];
	PA[16] = P[12];
	PA[17] = P[17];

	PA[18] = P[6] * A[0] + P[7] * A[1] + P[8] * A[2] + P[9] * A[3];
	PA[19] = P[6] * A[4] + P[7] * A[5] + P[8] * A[6] + P[9] * A[7] + P[18] * A[8];
	PA[20] = P[8] + P[9] * dT;
	PA[21] = P[6] * A[9] + P[7] * A[10] + P[9] + P[13] * A[11] + P[18] * A[12];
	PA[22] = P[13];
	PA[23] = P[18];

	PA[24] = P[10] * A[0] + P[11] * A[1] + P[12] * A[2] + P[13] * A[3];
	PA[25] = P[10] * A[4] + P[11] * A[5] + P[12] * A[6] + P[13] * A[7] + P[19] * A[8];
	PA[26] = P[12] + P[13] * dT;
	PA[27] = P[10] * A[9] + P[11] * A[10] + P[13] + P[14] * A[11] + P[19] * A[12];
	PA[28] = P[14];
	PA[29] = P[19];

	PA[30] = P[15] * A[0] + P[16] * A[1] + P[17] * A[2] + P[18] * A[3];
	PA[31] = P[15] * A[4] + P[16] * A[5] + P[17] * A[6] + P[18] * A[7] + P[20] * A[8];
	PA[32] = P[17] + P[18] * dT;
	PA[33] = P[15] * A[9] + P[16] * A[10] + P[18] + P[19] * A[11] + P[20] * A[12];
	PA[34] = P[19];
	PA[35] = P[20];

	P[0] = A[0] * PA[0] + A[1] * PA[6] + A[2] * PA[12] + A[3] * PA[18] + Q[0];
	P[1] = A[4] * PA[0] + A[5] * PA[6] + A[6] * PA[12] + A[7] * PA[18] + A[8] * PA[30];
	P[2] = A[4] * PA[1] + A[5] * PA[7] + A[6] * PA[13] + A[7] * PA[19] + A[8] * PA[31] + Q[1];
	P[3] = PA[12] + dT * PA[18];
	P[4] = PA[13] + dT * PA[19];
	P[5] = PA[14] + dT * PA[20] + Q[2];
	P[6] = A[9] * PA[0] + A[10] * PA[6] + PA[18] + A[11] * PA[24] + A[12] * PA[30];
	P[7] = A[9] * PA[1] + A[10] * PA[7] + PA[19] + A[11] * PA[25] + A[12] * PA[31];
	P[8] = A[9] * PA[2] + A[10] * PA[8] + PA[20] + A[11] * PA[26] + A[12] * PA[32];
	P[9] = A[9] * PA[3] + A[10] * PA[9] + PA[21] + A[11] * PA[27] + A[12] * PA[33] + Q[3];
	P[10] = PA[24];
	P[11] = PA[25];
	P[12] = PA[26];
	P[13] = PA[27];
	P[14] = PA[28] + Q[4];
	P[15] = PA[30];
	P[16] = PA[31];
	P[17] = PA[32];
	P[18] = PA[33];
	P[19] = PA[34];
	P[20] = PA[35] + Q[5];
}

static void
uFB(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, Q;
	float		uMIN, uMAX, uMID;
	int		xA, xB, xC, xU;

	/* Transform the voltage vector to ABC axes.
	 * */
	uA = uX;
	uB = - .5f * uX + .8660254f * uY;
	uC = - .5f * uX - .8660254f * uY;

	/* Get the boundaries.
	 * */
	if (uA < uB) {

		uMIN = uA;
		uMAX = uB;
	}
	else {
		uMIN = uB;
		uMAX = uA;
	}

	if (uC < uMIN) {

		uMID = uMIN;
		uMIN = uC;
	}
	else
		uMID = uC;

	if (uMAX < uMID) {

		Q = uMAX;
		uMAX = uMID;
		uMID = Q;
	}

	/* Voltage swing.
	 * */
	Q = uMAX - uMIN;

	if (Q < 1.f) {

		if (pm->mBit & PMC_MODE_EFFICIENT_MODULATION) {

			Q = uMIN + uMAX - pm->pwmEPS;

			/* Always snap neutral to GND or VCC to reduce switching losses.
			 * */
			Q = (Q < 0.f) ? 0.f - uMIN : 1.f - uMAX;
		}
		else {
			/* Only snap if voltage vector is overlong.
			 * */
			Q = (uMIN < -.5f) ? 0.f - uMIN : (uMAX > .5f) ? 1.f - uMAX : .5f;
		}
	}
	else {
		/* Crop the voltage vector.
		 * */
		Q = 1.f / Q;
		uA *= Q;
		uB *= Q;
		uC *= Q;

		/* The only possible neutral.
		 * */
		Q = .5f - (uMIN + uMAX) * Q * .5f;
	}

	uA += Q;
	uB += Q;
	uC += Q;

	/* Get PWM codes.
	 * */
	xA = (int) (pm->pwmR * uA + .5f);
	xB = (int) (pm->pwmR * uB + .5f);
	xC = (int) (pm->pwmR * uC + .5f);

	/* Minimal pulse width.
	 * */
	xU = pm->pwmR - pm->pwmMP;
	xA = (xA < pm->pwmMP) ? 0 : (xA > xU) ? pm->pwmR : xA;
	xB = (xB < pm->pwmMP) ? 0 : (xB > xU) ? pm->pwmR : xB;
	xC = (xC < pm->pwmMP) ? 0 : (xC > xU) ? pm->pwmR : xC;

	/* Update PWM duty cycle.
	 * */
	pm->pDC(xA, xB, xC);

	/* Reconstruct the actual voltage vector.
	 * */
	Q = .33333333f * (xA + xB + xC);
	uA = (xA - Q) * pm->U / pm->pwmR;
	uB = (xB - Q) * pm->U / pm->pwmR;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	pm->uX = uX;
	pm->uY = uY;
}

static void
iFB(pmc_t *pm)
{
	float		eD, eQ, uX, uY;
	float		uD, uQ;

	/* Obtain residual.
	 * */
	eD = pm->iSPD - pm->kX[0];
	eQ = pm->iSPQ - pm->kX[1];

	/* D axis PI+ regulator.
	 * */
	pm->iXD += pm->iKI * eD;
	pm->iXD = (pm->iXD > .5f) ? .5f : (pm->iXD < - .5f) ? - .5f : pm->iXD;
	uD = pm->iKP * eD + pm->iXD;
	//uD += (pm->iSPD * pm->R - pm->kX[4] * pm->iSPQ * pm->Lq) / pm->U;

	/* Q axis.
	 * */
	pm->iXQ += pm->iKI * eQ;
	pm->iXQ = (pm->iXQ > .5f) ? .5f : (pm->iXQ < - .5f) ? - .5f : pm->iXQ;
	uQ = pm->iKP * eQ + pm->iXQ;
	//uQ += (pm->iSPQ * pm->R + pm->kX[4] * (pm->iSPD * pm->Ld + pm->E)) / pm->U;

	/* Transform to XY axes.
	 * */
	uX = pm->kX[2] * uD - pm->kX[3] * uQ;
	uY = pm->kX[3] * uD + pm->kX[2] * uQ;

	uFB(pm, uX, uY);
}

static void
wFB(pmc_t *pm)
{
	float		eW, iSP;

	/* Obtain residual.
	 * */
	eW = pm->wSP - pm->kX[4];

	/* Speed PI+ regulator.
	 * */
	pm->wXX += pm->wKI * eW;
	pm->wXX = (pm->wXX > pm->iMAX) ? pm->iMAX : (pm->wXX < - pm->iMAX) ? - pm->iMAX : pm->wXX;
	iSP = pm->wKP * eW + pm->wXX;
	//iSP += pm->M / (pm->Zp * pm->E * 1.5f);

	/* Current limit.
	 * */
	iSP = (iSP > pm->iMAX) ? pm->iMAX : (iSP < - pm->iMAX) ? - pm->iMAX : iSP;

	pm->iSPD = 0.f;
	pm->iSPQ = iSP;
}

static void
bFSM(pmc_t *pm, float iA, float iB, float uS)
{
	float		iX, iY, L;

	switch (pm->mS1) {

		case PMC_STATE_IDLE:

			if (pm->mReq != PMC_REQ_NULL) {

				if (pm->mBit & PMC_MODE_EKF_6X_BASE) {

					if (pm->mReq == PMC_REQ_BREAK)
						pm->mS1 = PMC_STATE_BREAK;
					else
						pm->mReq = PMC_REQ_NULL;
				}
				else {
					if (pm->mReq == PMC_REQ_IMPEDANCE
							|| pm->mReq == PMC_REQ_CALIBRATE
							|| pm->mReq == PMC_REQ_SPINUP)
						pm->mS1 = PMC_STATE_DRIFT;
					else
						pm->mReq = PMC_REQ_NULL;
				}
			}
			break;

		case PMC_STATE_DRIFT:


			if (pm->mS2 == 0) {

				uFB(pm, 0.f, 0.f);

				pm->zA = 0.f;
				pm->zB = 0.f;
				pm->kT[0] = 0.f;

				pm->timVal = 0;
				pm->timEnd = 64;

				pm->mS2++;
			}
			else {
				pm->zA += -iA;
				pm->zB += -iB;
				pm->kT[0] += uS - pm->U;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					/* Zero Drift.
					 * */
					pm->cA0 += pm->zA / pm->timEnd;
					pm->cB0 += pm->zB / pm->timEnd;

					/* Supply Voltage.
					 * */
					pm->U += pm->kT[0] / pm->timEnd;

					if (pm->mS2 == 1) {

						pm->zA = 0.f;
						pm->zB = 0.f;
						pm->kT[0] = 0.f;

						pm->timVal = 0;
						pm->timEnd = pm->hzF * pm->sTdrift;

						pm->mS2++;
					}
					else {
						if (pm->mReq == PMC_REQ_IMPEDANCE)
							pm->mS1 = PMC_STATE_IMPEDANCE;
						else if (pm->mReq == PMC_REQ_CALIBRATE)
							pm->mS1 = PMC_STATE_CALIBRATE;
						else if (pm->mReq == PMC_REQ_SPINUP)
							pm->mS1 = PMC_STATE_SPINUP;

						pm->mS2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_IMPEDANCE:
#if 0
			/* Transform from ABC to XY axes.
			 * */
			iX = iA;
			iY = .57735027f * iA + 1.1547005f * iB;

			if (pm->mS2 == 0) {

				L = 2.f * KPI * pm->jFq / pm->hzF;
				pm->jCOS = kcos(L);
				pm->jSIN = ksin(L);

				pm->jX = iX;
				pm->jY = iY;

				pm->jIXre = 0.f;
				pm->jIXim = 0.f;
				pm->jIYre = 0.f;
				pm->jIYim = 0.f;
				pm->jUXre = 0.f;
				pm->jUXim = 0.f;
				pm->jUYim = 0.f;
				pm->jUYim = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->jTskip;

				pm->mS2++;
			}
			else {
				iX = .5f * (pm->jX + iX);
				iY = .5f * (pm->jY + iY);

				pm->rX = pm->jCOS * pm->rX - pm->jSIN * pm->rY;
				pm->rY = pm->jSIN * pm->rX + pm->jCOS * pm->rY;

				L = (3.f - pm->rX * pm->rX - pm->rY * pm->rY) * .5f;
				pm->rX *= L;
				pm->rY *= L;

				if (pm->mS2 == 2) {

					/* Current DFT.
					 * */
					pm->jIXre += iX * pm->rX;
					pm->jIXim += iX * pm->rY;
					pm->jIYre += iY * pm->rX;
					pm->jIYim += iY * pm->rY;

					/* Voltage DFT.
					 * */
					pm->jUXre += pm->uX * pm->rX;
					pm->jUXim += pm->uX * pm->rY;
					pm->jUYre += pm->uY * pm->rX;
					pm->jUYim += pm->uY * pm->rY;
				}

				uFB(pm, (pm->jUX + pm->rX * pm->jAmp) / pm->U,
					(pm->jUY + pm->rY * pm->jAmp) / pm->U);

				pm->jX = iX;
				pm->jY = iY;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					if (pm->mS2 == 1) {

						pm->timVal = 0;
						pm->timEnd = pm->hzF * pm->jTcap;

						pm->mS2++;
					}
					else {
						pm->mS1 = PMC_STATE_END;
						pm->mS2 = 0;
					}
				}
			}
#endif
			break;

		case PMC_STATE_CALIBRATE:
			break;

		case PMC_STATE_SPINUP:

			if (pm->mS2 == 0) {

				int			j;

				pm->mBit |= PMC_MODE_EKF_6X_BASE;

				pm->kX[0] = 0.f;
				pm->kX[1] = 0.f;
				pm->kX[2] = 1.f;
				pm->kX[3] = 0.f;
				pm->kX[4] = 0.f;

				pm->zA = 0.f;
				pm->zB = 0.f;

				for (j = 0; j < 21; ++j)
					pm->kP[j] = 0.f;

				pm->kP[0] = 1e+4f;
				pm->kP[2] = 1e+4f;
				pm->kP[5] = 5.f;
				pm->kP[9] = 5.f;

				pm->iSPD = 1.f;
				pm->iSPQ = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sThold;

				pm->mS2++;
			}
			else if (pm->mS2 == 1) {

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->mBit |= PMC_MODE_SPEED_CONTROL_LOOP;

					pm->iSPD = 0.f;
					pm->iSPQ = 1.f;

					pm->wSP = 3000.f;

					pm->mReq = PMC_REQ_NULL;
					pm->mS1 = PMC_STATE_IDLE;
					pm->mS2 = 0;
				}
			}
			break;

		case PMC_STATE_BREAK:
			break;

		case PMC_STATE_END:

			uFB(pm, 0.f, 0.f);

			pm->mReq = PMC_REQ_NULL;
			pm->mBit = 0;
			pm->mS1 = PMC_STATE_IDLE;
			pm->mS2 = 0;

			break;
	}
}

void pmcFeedBack(pmc_t *pm, int xA, int xB, int xU)
{
	float		iA, iB, uS;

	/* Conversion to Ampere and Volt.
	 * */
	iA = pm->cA1 * (xA - 2048) + pm->cA0;
	iB = pm->cB1 * (xB - 2048) + pm->cB0;
	uS = pm->cU1 * xU + pm->cU0;

	/* Call FSM.
	 * */
	bFSM(pm, iA, iB, uS);

	if (pm->mBit & PMC_MODE_EKF_6X_BASE) {

		/* EKF.
		 * */
		kFB(pm, iA, iB); // 1658 ticks

		/* Current control loop.
		 * */
		iFB(pm);

		/* Speed control loop.
		 * */
		if (pm->mBit & PMC_MODE_SPEED_CONTROL_LOOP) {

			wFB(pm);
		}

		/* EKF.
		 * */
		kAT(pm); // 664 ticks
	}
}

