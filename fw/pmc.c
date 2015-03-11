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

static inline float
xabs(float x) { return (x < 0.f) ? - x : x; }

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
	pm->Tdrift = .1f;
	pm->Thold = .5f;
	pm->Tsample = .2f;
	pm->Tout = 1.f;
	pm->Tend = .1f;
	pm->pwmMP = (int) (2e-7f * pm->hzF * pm->pwmR + .5f);

	/* ADC conversion constants.
	 * */
	pm->cA0 = 0.f;
	pm->cA1 = .01464844f;
	pm->cB0 = 0.f;
	pm->cB1 = .01464844f;
	pm->cU0 = 0.f;
	pm->cU1 = .00725098f;

	/* Q covariance.
	 * */
	pm->kQ[0] = 1e-4f;
	pm->kQ[1] = 1e-4f;
	pm->kQ[2] = 1e-8f;
	pm->kQ[3] = 1e-8f;
	pm->kQ[4] = 1e-5f;
	pm->kQ[5] = 1e-9f;

	/* R covariance.
	 * */
	pm->kR = 1e-2f;

	/* PI constants.
	 * */
	pm->iKP = 2e-2f;
	pm->iKI = 4e-3f;
	pm->wKP = 1e-2f;
	pm->wKI = 0e-3f;

	pm->iMAX = 5.f;
	pm->wMAX = 2e+4f;
	pm->wMIN = 5e+2f;
}

static void
dEq(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ;

	/* Transform voltage to DQ axes.
	 * */
	uD = X[2] * pm->uX + X[3] * pm->uY;
	uQ = X[2] * pm->uY - X[3] * pm->uX;

	/* Electrical equations.
	 * */
	D[0] = (uD - pm->R * X[0] + pm->Lq * X[4] * X[1]) / pm->Ld;
	D[1] = (uQ - pm->R * X[1] - pm->Ld * X[4] * X[0]
			- pm->E * X[4] + pm->Qd) / pm->Lq;

	/* Mechanical equations.
	 * */
	D[2] = X[4];
	D[3] = pm->Zp * (1.5f * pm->Zp * (pm->E - (pm->Lq - pm->Ld)
				* X[0]) * X[1] - pm->M) * pm->IJ;
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
	float		PC[12], S[3], L[3], K[12];
	float		iX, iY, iD, iQ, eD, eQ, dR;

	/* Zero Drift.
	 * */
	iA -= pm->Ad;
	iB -= pm->Bd;

	/* Transform current to XY axes.
	 * */
	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	/* Transform current to DQ axes.
	 * */
	iD = X[2] * iX + X[3] * iY;
	iQ = X[2] * iY - X[3] * iX;

	/* Obtain residual.
	 * */
	eD = iD - X[0];
	eQ = iQ - X[1];

	pm->eD = eD;
	pm->eQ = eQ;

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

	S[0] = PC[0] - X[1] * PC[4] + pm->kR;
	S[1] = PC[1] - X[1] * PC[5];
	S[2] = PC[3] + X[0] * PC[5] + pm->kR;

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
	dROT(X + 2, dR, X + 2);
	X[4] += K[6] * eD + K[7] * eQ;
	pm->M += K[8] * eD + K[9] * eQ;
	pm->Qd += K[10] * eD + K[11] * eQ;

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

	/* Temporal.
	 * */
	pm->kT[0] = X[0];
	pm->kT[1] = X[1];
	pm->kT[2] = X[2];
	pm->kT[3] = X[3];
	pm->kT[4] = X[4];

	/* Update state estimate.
	 * */
	sFC(pm);
}

static void
kAT(pmc_t *pm)
{
	float		*X = pm->kX, *P = pm->kP;
	float		A[12], PA[24];
	float		dT, iD, iQ, rX, rY, wR;
	float		dTLd, dTLq, dTIJ, Zp2, L;

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
	dTLd = dT / pm->Ld;
	dTLq = dT / pm->Lq;
	dTIJ = dT * pm->IJ;
	Zp2 = 1.5f * pm->Zp * pm->Zp * dTIJ;

	/* Transition matrix Jacobian.
	 * */
	A[0] = 1.f - pm->R * dTLd;
	A[1] = wR * pm->Lq * dTLd;
	A[2] = (rX * pm->uY - rY * pm->uX) * dTLd;
	A[3] = iQ * pm->Lq * dTLd;

	A[4] = - wR * pm->Ld * dTLq;
	A[5] = 1.f - pm->R * dTLq;
	A[6] = - (rX * pm->uX + rY * pm->uY) * dTLq;
	A[7] = - (pm->E + iD * pm->Ld) * dTLq;
	A[8] = dTLq;

	A[9] = iQ * (pm->Ld - pm->Lq) * Zp2;
	A[10] = Zp2 * (pm->E - iD * (pm->Lq - pm->Ld));
	A[11] = - pm->Zp * dTIJ;

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
		+ pm->kQ[0];
	P[1] = A[0] * PA[1] + A[1] * PA[5] + A[2] * PA[9] + A[3] * PA[13];
	P[2] = A[4] * PA[1] + A[5] * PA[5] + A[6] * PA[9] + A[7] * PA[13]
		+ A[8] * PA[21] + pm->kQ[1];
	P[3] = PA[8] + dT * PA[12];
	P[4] = PA[9] + dT * PA[13];
	P[5] = PA[10] + dT * PA[14] + pm->kQ[2];
	P[6] = A[9] * PA[0] + A[10] * PA[4] + PA[12] + A[11] * PA[16];
	P[7] = A[9] * PA[1] + A[10] * PA[5] + PA[13] + A[11] * PA[17];
	P[8] = A[9] * PA[2] + A[10] * PA[6] + PA[14] + A[11] * PA[18];
	P[9] = A[9] * PA[3] + A[10] * PA[7] + PA[15] + A[11] * PA[19]
		+ pm->kQ[3];
	P[10] = PA[16];
	P[11] = PA[17];
	P[12] = PA[18];
	P[13] = PA[19];
	P[14] += pm->kQ[4];
	P[15] = PA[20];
	P[16] = PA[21];
	P[17] = PA[22];
	P[18] = PA[23];
	P[20] += pm->kQ[5];
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

			Q = uMIN + uMAX;

			/* Always snap neutral to GND or VCC to
			 * reduce switching losses.
			 * */
			Q = (Q < 0.f) ? 0.f - uMIN : 1.f - uMAX;
		}
		else {
			/* Only snap if voltage vector is overlong.
			 * */
			Q = (uMIN < - .5f) ? 0.f - uMIN :
				(uMAX > .5f) ? 1.f - uMAX : .5f;
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

	/* Frequency injection.
	 * */
	if (pm->mBit & PMC_MODE_FREQUENCY_INJECTION) {

		if (pm->hSP > 0.f)
			eD += pm->hCS[0] * pm->hSP;
		else
			eQ += pm->hCS[0] * pm->hSP;

		dROT(pm->hCS, pm->hT, pm->hCS);
	}

	/* D axis PI regulator.
	 * */
	pm->iXD += pm->iKI * eD;
	pm->iXD = (pm->iXD > .67f) ? .67f : (pm->iXD < - .67f) ? - .67f : pm->iXD;
	uD = pm->iKP * eD + pm->iXD;

	/* Q axis.
	 * */
	pm->iXQ += pm->iKI * eQ;
	pm->iXQ = (pm->iXQ > .67f) ? .67f : (pm->iXQ < - .67f) ? - .67f : pm->iXQ;
	uQ = pm->iKP * eQ + pm->iXQ;

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

	/* Speed PI regulator.
	 * */
	pm->wXX += pm->wKI * eW;
	pm->wXX = (pm->wXX > pm->iMAX) ? pm->iMAX : (pm->wXX < - pm->iMAX) ? - pm->iMAX : pm->wXX;
	iSP = pm->wKP * eW + pm->wXX + pm->wKD * pm->kX[1];

	/* Current limit.
	 * */
	iSP = (iSP > pm->iMAX) ? pm->iMAX : (iSP < - pm->iMAX) ? - pm->iMAX : iSP;

	pm->iSPD = 0.f;
	pm->iSPQ = iSP;
}

static void
bFSM(pmc_t *pm, float iA, float iB, float uS)
{
	switch (pm->mS1) {

		case PMC_STATE_IDLE:

			if (pm->mReq != PMC_REQ_NULL) {

				if (pm->mBit & PMC_MODE_EKF_6X_DQ) {

					if (pm->mReq == PMC_REQ_BREAK)
						pm->mS1 = PMC_STATE_BREAK;
					else
						pm->mReq = PMC_REQ_NULL;
				}
				else {
					if (pm->mReq == PMC_REQ_SPINUP
							|| pm->mReq == PMC_REQ_SINE
							|| pm->mReq == PMC_REQ_LINEAR)
						pm->mS1 = PMC_STATE_DRIFT;
					else
						pm->mReq = PMC_REQ_NULL;
				}
			}
			break;

		case PMC_STATE_DRIFT:

			if (pm->mS2 == 0) {

				uFB(pm, 0.f, 0.f);

				pm->Ad = 0.f;
				pm->Bd = 0.f;
				pm->kT[0] = 0.f;

				pm->tVal = 0;
				pm->tEnd = 64;

				pm->mS2++;
			}
			else {
				pm->Ad += - iA;
				pm->Bd += - iB;
				pm->kT[0] += uS - pm->U;

				pm->tVal++;

				if (pm->tVal < pm->tEnd) ;
				else {
					/* Current sensors Zero Drift.
					 * */
					pm->cA0 += pm->Ad / pm->tEnd;
					pm->cB0 += pm->Bd / pm->tEnd;

					/* Supply Voltage.
					 * */
					pm->U += pm->kT[0] / pm->tEnd;

					if (pm->mS2 == 1) {

						pm->Ad = 0.f;
						pm->Bd = 0.f;
						pm->kT[0] = 0.f;

						pm->tVal = 0;
						pm->tEnd = pm->hzF * pm->Tdrift;

						pm->mS2++;
					}
					else {
						if (pm->mReq == PMC_REQ_SPINUP)
							pm->mS1 = PMC_STATE_HOLD;
						else if (pm->mReq == PMC_REQ_SINE)
							pm->mS1 = PMC_STATE_SINE;
						else if (pm->mReq == PMC_REQ_LINEAR)
							pm->mS1 = PMC_STATE_LINEAR;
						else
							pm->mS1 = PMC_STATE_END;

						pm->mS2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_HOLD:

			if (pm->mS2 == 0) {

				pm->kX[2] = 1.f;
				pm->kX[3] = 0.f;

				pm->iSPD = pm->iHOLD;
				pm->iSPQ = 0.f;

				pm->tVal = 0;
				pm->tEnd = pm->hzF * pm->Thold;

				pm->mS2++;
			}
			else {
				pm->kX[0] = iA;
				pm->kX[1] = .57735027f * iA + 1.1547005f * iB;

				pm->kT[0] += pm->uX;

				iFB(pm);

				pm->tVal++;

				if (pm->tVal < pm->tEnd) ;
				else {
					if (pm->mS2 == 1) {

						pm->kT[0] = 0.f;

						pm->tVal = 0;
						pm->tEnd = pm->hzF * pm->Tsample;

						pm->mS2++;
					}
					else {
						/* Winding Resistance.
						 * */
						pm->R = pm->kT[0] / pm->tEnd / pm->iSPD;

						if (pm->mReq == PMC_REQ_SPINUP)
							pm->mS1 = PMC_STATE_SPINUP;
						else
							pm->mS1 = PMC_STATE_END;

						pm->mS2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_SINE:

			if (pm->mS2 == 0) {

				pm->kX[2] = 1.f;
				pm->kX[3] = 0.f;

				pm->tVal = 0;
				pm->tEnd = pm->hzF * pm->Thold;

				pm->mS2++;
			}
			else {
				float			iX, iY, L;

				pm->kX[0] = iA;
				pm->kX[1] = .57735027f * iA + 1.1547005f * iB;

				iX = .5f * (pm->kX[0] + pm->kP[0]);
				iY = .5f * (pm->kX[1] + pm->kP[1]);

				pm->kP[0] = pm->kX[0];
				pm->kP[1] = pm->kX[1];

				if (pm->mS2 == 2) {

					if (pm->hSP > 0.f) {

						pm->kT[0] += iX * pm->hCS[0];
						pm->kT[1] += iX * pm->hCS[1];
						pm->kT[2] += pm->uX * pm->hCS[0];
						pm->kT[3] += pm->uX * pm->hCS[1];
					}
					else {
						pm->kT[0] += iY * pm->hCS[0];
						pm->kT[1] += iY * pm->hCS[1];
						pm->kT[2] += pm->uY * pm->hCS[0];
						pm->kT[3] += pm->uY * pm->hCS[1];
					}
				}

				iFB(pm);

				pm->tVal++;

				if (pm->tVal < pm->tEnd) ;
				else {
					if (pm->mS2 == 1) {

						pm->mBit |= PMC_MODE_FREQUENCY_INJECTION;

						pm->hT = 6.2831853f * pm->sineF / pm->hzF;
						pm->hCS[0] = 0.f;
						pm->hCS[1] = - 1.f;

						pm->kT[0] = 0.f;
						pm->kT[1] = 0.f;
						pm->kT[2] = 0.f;
						pm->kT[3] = 0.f;

						pm->tVal = 0;
						pm->tEnd = pm->hzF * pm->Tsample;

						pm->mS2++;
					}
					else {
						pm->kP[0] = pm->kT[2] * pm->kT[0] + pm->kT[3] * pm->kT[1];
						pm->kP[1] = pm->kT[2] * pm->kT[1] - pm->kT[3] * pm->kT[0];

						L = pm->kT[0] * pm->kT[0] + pm->kT[1] * pm->kT[1];
						pm->kP[0] /= L;
						pm->kP[1] /= L;

						pm->R = pm->kP[0];
						L = pm->kP[1] / (6.2831853f * pm->sineF);

						printf("R %e \tL %e\n", pm->R, L);

						if (pm->hSP > 0.f) {

							pm->Ld = L;
							pm->ILd = 1.f / L;
						}
						else {
							pm->Lq = L;
							pm->ILq = 1.f / L;
						}

						pm->mReq = PMC_REQ_NULL;
						pm->mS1 = PMC_STATE_END;
						pm->mS2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_LINEAR:
			break;

		case PMC_STATE_SPINUP:

			if (pm->mS2 == 0) {

				pm->mBit |= PMC_MODE_EKF_6X_DQ
					| PMC_MODE_SPEED_CONTROL_LOOP;

				/* X(0).
				 * */
				pm->kX[0] = 0.f;
				pm->kX[1] = 0.f;
				pm->kX[2] = 1.f;
				pm->kX[3] = 0.f;
				pm->kX[4] = 0.f;

				pm->Ad = 0.f;
				pm->Bd = 0.f;
				pm->Qd = 0.f;

				pm->M = 0.f;

				/* P(0).
				 * */
				pm->kP[1] = 0.f;
				pm->kP[3] = 0.f;
				pm->kP[4] = 0.f;
				pm->kP[6] = 0.f;
				pm->kP[7] = 0.f;
				pm->kP[8] = 0.f;
				pm->kP[10] = 0.f;
				pm->kP[11] = 0.f;
				pm->kP[12] = 0.f;
				pm->kP[13] = 0.f;
				pm->kP[15] = 0.f;
				pm->kP[16] = 0.f;
				pm->kP[17] = 0.f;
				pm->kP[18] = 0.f;
				pm->kP[19] = 0.f;

				pm->kP[0] = 2e+6f;
				pm->kP[2] = 2e+6f;
				pm->kP[5] = 5.f;
				pm->kP[9] = 4e+1f;
				pm->kP[14] = 1e-2f;
				pm->kP[20] = 0.f;

				pm->wSP = pm->wMIN;

				pm->tVal = 0;
				pm->tEnd = pm->hzF * pm->Tout;

				pm->mS2++;
			}
			else {
				if (xabs(pm->kX[4]) > (pm->wMIN * .7f)) {

					pm->mReq = PMC_REQ_NULL;
					pm->mS1 = PMC_STATE_IDLE;
					pm->mS2 = 0;
				}

				pm->tVal++;

				if (pm->tVal < pm->tEnd) ;
				else {
					pm->mReq = PMC_REQ_NULL;
					pm->mS1 = PMC_STATE_END;
					pm->mS2 = 0;
				}
			}
			break;

		case PMC_STATE_BREAK:

			pm->mBit |= PMC_MODE_SPEED_CONTROL_LOOP;
			pm->wSP = 0.f;

			/* TODO */

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

	if (pm->mBit & PMC_MODE_EKF_6X_DQ) {

		/* EKF.
		 * */
		kFB(pm, iA, iB);

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
		kAT(pm);
	}
}

