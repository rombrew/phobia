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
	pm->Tend = .1f;

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

	/* Additive covariance.
	 * */
	pm->kQ[0] = 2e-4f;
	pm->kQ[1] = 2e-4f;
	pm->kQ[2] = 1e-7f;
	pm->kQ[3] = 1e-7f;

	pm->kQ[4] = 1e-6f;
	pm->kQ[5] = 1e-18f;
	pm->kQ[6] = 1e-18f;
	pm->kQ[7] = 1e-18f;
	pm->kQ[8] = 1e-18f;

	/* Maximal covariance.
	 * */
	pm->E_COV = 5e-9f;
	pm->R_COV = 2e-3f;
	pm->Ld_COV = 0e-9f;
	pm->Lq_COV = 0e-9f;

	pm->E_MIN = 3e-4;
	pm->E_MAX = 3e-3;
	pm->R_MIN = 10e-3;
	pm->R_MAX = .4f;

	/* Measurement covariance.
	 * */
	pm->kR = 1e-2;

	/* PI constants.
	 * */
	pm->iKP = 2e-2f;
	pm->iKI = 4e-3f;
	pm->wKP = 1e-1f;
	pm->wKI = 0e-3f;

	pm->iMAX = 5.f;
	pm->wMAX = 2e+4f;
	pm->wMIN = 0.f;
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
	D[1] = (uQ - pm->R * X[1] - pm->Ld * X[4] * X[0] - pm->E * X[4]) / pm->Lq;

	/* Mechanical equations.
	 * */
	D[2] = X[4];
	D[3] = pm->Zp * (1.5f * pm->Zp * (pm->E - (pm->Lq - pm->Ld) * X[0]) * X[1]
			- pm->M) * pm->IJ;
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
pE(pmc_t *pm, float D[], const float X[])
{
	float		uD, uQ;

	/* Transform voltage to DQ axes.
	 * */
	uD = X[2] * pm->uX + X[3] * pm->uY;
	uQ = X[2] * pm->uY - X[3] * pm->uX;

	/* Electrical equations.
	 * */
	D[0] = (uD - X[7] * X[0] + X[9] * X[4] * X[1]) / X[8];
	D[1] = (uQ - X[7] * X[1] - X[8] * X[4] * X[0] - X[6] * X[4]) / X[9];

	/* Mechanical equations.
	 * */
	D[2] = X[4];
	D[3] = pm->Zp * (1.5f * pm->Zp * (X[6] - (X[9] - X[8]) * X[0]) * X[1]
			- X[5]) * pm->IJ;
}


static void
pF(pmc_t *pm, float Y[], const float X[])
{
	float		D1[4], D2[4], X2[10];
	float		dT;

	/* Second-order ODE solver.
	 * */

	pE(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	dROT(X2 + 2, D1[2] * dT, X + 2);
	X2[4] = X[4] + D1[3] * dT;

	X2[5] = X[5];
	X2[6] = X[6];
	X2[7] = X[7];
	X2[8] = X[8];
	X2[9] = X[9];

	pE(pm, D2, X2);
	dT *= .5f;

	Y[0] = X[0] + (D1[0] + D2[0]) * dT;
	Y[1] = X[1] + (D1[1] + D2[1]) * dT;
	dROT(Y + 2, (D1[2] + D2[2]) * dT, X + 2);
	Y[4] = X[4] + (D1[3] + D2[3]) * dT;

	/*Y[0] = X[0] + D1[0] * dT;
	Y[1] = X[1] + D1[1] * dT;
	dROT(Y + 2, D1[2] * dT, X + 2);
	Y[4] = X[4] + D1[3] * dT;*/

	Y[5] = X[5];
	Y[6] = X[6];
	Y[7] = X[7];
	Y[8] = X[8];
	Y[9] = X[9];
}

static void
kFB(pmc_t *pm, float iA, float iB)
{
	float		*X = pm->kX, *P = pm->kP;
	float		C[6], PC[18], S[3], iS[3], K[18], D;
	float		iX, iY, xA, xB, eA, eB, dR;

	/* Get model output.
	 * */
	iX = X[2] * X[0] - X[3] * X[1];
	iY = X[3] * X[0] + X[2] * X[1];

	xA = iX - pm->Ad;
	xB = - .5f * iX + .8660254f * iY - pm->Bd;

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
		PC[12] = P[21] * C[0] + P[22] * C[1] + P[23] * C[2];
		PC[13] = P[21] * C[3] + P[22] * C[4] + P[23] * C[5];
		PC[14] = P[28] * C[0] + P[29] * C[1] + P[30] * C[2];
		PC[15] = P[28] * C[3] + P[29] * C[4] + P[30] * C[5];
		PC[16] = P[36] * C[0] + P[37] * C[1] + P[38] * C[2];
		PC[17] = P[36] * C[3] + P[37] * C[4] + P[38] * C[5];

		S[0] = C[0] * PC[0] + C[1] * PC[2] + C[2] * PC[4] + pm->kR;
		S[1] = C[0] * PC[1] + C[1] * PC[3] + C[2] * PC[5];
		S[2] = C[3] * PC[1] + C[4] * PC[3] + C[5] * PC[5] + pm->kR;

		/* K = P * C' / S;
		 * */
		D = S[0] * S[2] - S[1] * S[1];
		iS[0] = S[2] / D;
		iS[1] = - S[1] / D;
		iS[2] = S[0] / D;

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
		K[12] = PC[12] * iS[0] + PC[13] * iS[1];
		K[13] = PC[12] * iS[1] + PC[13] * iS[2];
		K[14] = PC[14] * iS[0] + PC[15] * iS[1];
		K[15] = PC[14] * iS[1] + PC[15] * iS[2];
		K[16] = PC[16] * iS[0] + PC[17] * iS[1];
		K[17] = PC[16] * iS[1] + PC[17] * iS[2];

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
		pm->R += K[12] * eA + K[13] * eB;
		pm->Ld += K[14] * eA + K[15] * eB;
		pm->Lq += K[16] * eA + K[17] * eB;

		pm->E = (pm->E < pm->E_MIN) ? pm->E_MIN :
			(pm->E > pm->E_MAX) ? pm->E_MAX : pm->E;
		pm->R = (pm->R < pm->R_MIN) ? pm->R_MIN :
			(pm->R > pm->R_MAX) ? pm->R_MAX : pm->R;


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
		P[21] -= K[12] * PC[0] + K[13] * PC[1];
		P[22] -= K[12] * PC[2] + K[13] * PC[3];
		P[23] -= K[12] * PC[4] + K[13] * PC[5];
		P[24] -= K[12] * PC[6] + K[13] * PC[7];
		P[25] -= K[12] * PC[8] + K[13] * PC[9];
		P[26] -= K[12] * PC[10] + K[13] * PC[11];
		P[27] -= K[12] * PC[12] + K[13] * PC[13];
		P[28] -= K[14] * PC[0] + K[15] * PC[1];
		P[29] -= K[14] * PC[2] + K[15] * PC[3];
		P[30] -= K[14] * PC[4] + K[15] * PC[5];
		P[31] -= K[14] * PC[6] + K[15] * PC[7];
		P[32] -= K[14] * PC[8] + K[15] * PC[9];
		P[33] -= K[14] * PC[10] + K[15] * PC[11];
		P[34] -= K[14] * PC[12] + K[15] * PC[13];
		P[35] -= K[14] * PC[14] + K[15] * PC[15];
		P[36] -= K[16] * PC[0] + K[17] * PC[1];
		P[37] -= K[16] * PC[2] + K[17] * PC[3];
		P[38] -= K[16] * PC[4] + K[17] * PC[5];
		P[39] -= K[16] * PC[6] + K[17] * PC[7];
		P[40] -= K[16] * PC[8] + K[17] * PC[9];
		P[41] -= K[16] * PC[10] + K[17] * PC[11];
		P[42] -= K[16] * PC[12] + K[17] * PC[13];
		P[43] -= K[16] * PC[14] + K[17] * PC[15];
		P[44] -= K[16] * PC[16] + K[17] * PC[17];
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
	sFC(pm);
}

#include <stdlib.h>
#include <stdio.h>

#define LOW(A, I, J)		((J < I) ? A[(I) * ((I) + 1) / 2 + (J)] : A[(J) * ((J) + 1) / 2 + (I)])
#define LOW2(A, I, J)		A[(I) * ((I) + 1) / 2 + (J)]

static void
kAT(pmc_t *pm)
{
	float		*X = pm->kX, *P = pm->kP;
	float		A[20], PA[36];
	float		dT, iD, iQ, rX, rY, wR;
	float		dTLd, dTLq, dTIJ, Zp2;
	float		uD, uQ, L;

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

	{
		float		A[9][9], PA[9][9], S;
		float		X0[10], Y0[10], X[10], Y[10], dX;
		int		i, j, k;

		X0[0] = iD;
		X0[1] = iQ;
		X0[2] = rX;
		X0[3] = rY;
		X0[4] = wR;
		X0[5] = pm->M;
		X0[6] = pm->E;
		X0[7] = pm->R;
		X0[8] = pm->Ld;
		X0[9] = pm->Lq;

		pF(pm, Y0, X0);

		dX = 1e-4f;

		for (i = 0; i < 9; ++i) {

			for (j = 0; j < 10; ++j)
				X[j] = X0[j];

			if (i > 7)
				dX = 1e-11;

			if (i == 2) {
				X[2] += - X[3] * dX;
				X[3] += X[2] * dX;
			}
			else if (i > 2)
				X[i + 1] += dX;
			else
				X[i] += dX;

			pF(pm, Y, X);

			for (j = 0; j < 2; ++j)
				A[j][i] = (Y[j] - Y0[j]) / dX;
			for (j = 4; j < 10; ++j)
				A[j - 1][i] = (Y[j] - Y0[j]) / dX;
			A[2][i] = ((Y[2] - Y0[2]) * - X[3]
					+ (Y[3] - Y0[3]) * X[2]) / dX;
		}

		/*for (i = 0; i < 9; ++i) {
			for (j = 0; j < 9; ++j) {
				printf("%f ", A[i][j]);
			}
			printf("\n");
		}

		printf("\n");*/

		/*for (i = 0; i < 9; ++i)
			for (j = 0; j < 9; ++j) {

				S = 0.f;
				for (k = 0; k < 9; ++k)
					S += LOW(P, i, k) * A[j][k];

				PA[i][j] = S;
			}

		for (i = 0; i < 9; ++i)
			for (j = 0; j < i + 1; ++j) {

				S = 0.f;
				for (k = 0; k < 9; ++k)
					S += A[i][k] * PA[k][j];

				LOW2(P, i, j) = S;
			}*/
	}

	if (1) {

	/* Common subexpressions.
	 * */
	dTLd = dT / pm->Ld;
	dTLq = dT / pm->Lq;
	dTIJ = dT * pm->IJ;
	Zp2 = 1.5f * pm->Zp * pm->Zp * dTIJ;
	uD = rX * pm->uX + rY * pm->uY;
	uQ = rX * pm->uY - rY * pm->uX;

	/* Transition Jacobian matrix.
	 * */
	A[0] = 1.f - pm->R * dTLd;
	A[1] = wR * pm->Lq * dTLd;
	A[2] = uQ * dTLd;
	A[3] = iQ * pm->Lq * dTLd;
	A[4] = - iD * dTLd;
	A[5] = - (uD - iD * pm->R + iQ * wR * pm->Lq) * dTLd / pm->Ld;
	A[6] = iQ * wR * dTLd;

	A[7] = - wR * pm->Ld * dTLq;
	A[8] = 1.f - pm->R * dTLq;
	A[9] = - uD * dTLq;
	A[10] = (- pm->E - iD * pm->Ld) * dTLq;
	A[11] = - wR * dTLq;
	A[12] = - iQ * dTLq;
	A[13] = - iD * wR * dTLq;
	A[14] = - (uQ - iQ * pm->R - wR * (iD * pm->Ld + pm->E)) * dTLq / pm->Lq;

	A[15] = iQ * (pm->Ld - pm->Lq) * Zp2;
	A[16] = Zp2 * (pm->E - iD * (pm->Lq - pm->Ld));
	A[17] = - pm->Zp * dTIJ;
	A[18] = iQ * Zp2;
	A[19] = iD * iQ * Zp2;

	/* P = A * P * A'.
	 * */
	PA[0] = P[0] * A[0] + P[1] * A[1] + P[3] * A[2] + P[6] * A[3] + P[21] * A[4]
		+ P[28] * A[5] + P[36] * A[6];
	PA[1] = P[0] * A[7] + P[1] * A[8] + P[3] * A[9] + P[6] * A[10] + P[15] * A[11]
		+ P[21] * A[12] + P[28] * A[13] + P[36] * A[14];
	PA[2] = P[3] + P[6] * dT;
	PA[3] = P[0] * A[15] + P[1] * A[16] + P[6] + P[10] * A[17] + P[15] * A[18]
		+ (P[28] - P[36]) * A[19];

	PA[4] = P[1] * A[0] + P[2] * A[1] + P[4] * A[2] + P[7] * A[3] + P[22] * A[4]
		+ P[29] * A[5] + P[37] * A[6];
	PA[5] = P[1] * A[7] + P[2] * A[8] + P[4] * A[9] + P[7] * A[10] + P[16] * A[11]
		+ P[22] * A[12] + P[29] * A[13] + P[37] * A[14];
	PA[6] = P[4] + P[7] * dT;
	PA[7] = P[1] * A[15] + P[2] * A[16] + P[7] + P[11] * A[17] + P[16] * A[18]
		+ (P[29] - P[37]) * A[19];

	PA[8] = P[3] * A[0] + P[4] * A[1] + P[5] * A[2] + P[8] * A[3] + P[23] * A[4]
		+ P[30] * A[5] + P[38] * A[6];
	PA[9] = P[3] * A[7] + P[4] * A[8] + P[5] * A[9] + P[8] * A[10] + P[17] * A[11]
		+ P[23] * A[12] + P[30] * A[13] + P[38] * A[14];
	PA[10] = P[5] + P[8] * dT;
	PA[11] = P[3] * A[15] + P[4] * A[16] + P[8] + P[12] * A[17] + P[17] * A[18]
		+ (P[30] - P[38]) * A[19];

	PA[12] = P[6] * A[0] + P[7] * A[1] + P[8] * A[2] + P[9] * A[3] + P[24] * A[4]
		+ P[31] * A[5] + P[39] * A[6];
	PA[13] = P[6] * A[7] + P[7] * A[8] + P[8] * A[9] + P[9] * A[10] + P[18] * A[11]
		+ P[24] * A[12] + P[31] * A[13] + P[39] * A[14];
	PA[14] = P[8] + P[9] * dT;
	PA[15] = P[6] * A[15] + P[7] * A[16] + P[9] + P[13] * A[17] + P[18] * A[18]
		+ (P[31] - P[39]) * A[19];

	PA[16] = P[10] * A[0] + P[11] * A[1] + P[12] * A[2] + P[13] * A[3] + P[25] * A[4]
		+ P[32] * A[5] + P[40] * A[6];
	PA[17] = P[10] * A[7] + P[11] * A[8] + P[12] * A[9] + P[13] * A[10] + P[19] * A[11]
		+ P[25] * A[12] + P[32] * A[13] + P[40] * A[14];
	PA[18] = P[12] + P[13] * dT;
	PA[19] = P[10] * A[15] + P[11] * A[16] + P[13] + P[14] * A[17] + P[19] * A[18]
		+ (P[32] - P[40]) * A[19];

	PA[20] = P[15] * A[0] + P[16] * A[1] + P[17] * A[2] + P[18] * A[3] + P[26] * A[4]
		+ P[33] * A[5] + P[41] * A[6];
	PA[21] = P[15] * A[7] + P[16] * A[8] + P[17] * A[9] + P[18] * A[10] + P[20] * A[11]
		+ P[26] * A[12] + P[33] * A[13] + P[41] * A[14];
	PA[22] = P[17] + P[18] * dT;
	PA[23] = P[15] * A[15] + P[16] * A[16] + P[18] + P[19] * A[17] + P[20] * A[18]
		+ (P[33] - P[41]) * A[19];

	PA[24] = P[21] * A[0] + P[22] * A[1] + P[23] * A[2] + P[24] * A[3] + P[27] * A[4]
		+ P[34] * A[5] + P[42] * A[6];
	PA[25] = P[21] * A[7] + P[22] * A[8] + P[23] * A[9] + P[24] * A[10] + P[26] * A[11]
		+ P[27] * A[12] + P[34] * A[13] + P[42] * A[14];
	PA[26] = P[23] + P[24] * dT;
	PA[27] = P[21] * A[15] + P[22] * A[16] + P[24] + P[25] * A[17] + P[26] * A[18]
		+ (P[34] - P[42]) * A[19];

	PA[28] = P[28] * A[0] + P[29] * A[1] + P[30] * A[2] + P[31] * A[3] + P[34] * A[4]
		+ P[35] * A[5] + P[43] * A[6];
	PA[29] = P[28] * A[7] + P[29] * A[8] + P[30] * A[9] + P[31] * A[10] + P[33] * A[11]
		+ P[34] * A[12] + P[35] * A[13] + P[43] * A[14];
	PA[30] = P[30] + P[31] * dT;
	PA[31] = P[28] * A[15] + P[29] * A[16] + P[31] + P[32] * A[17] + P[33] * A[18]
		+ (P[35] - P[43]) * A[19];

	PA[32] = P[36] * A[0] + P[37] * A[1] + P[38] * A[2] + P[39] * A[3] + P[42] * A[4]
		+ P[43] * A[5] + P[44] * A[6];
	PA[33] = P[36] * A[7] + P[37] * A[8] + P[38] * A[9] + P[39] * A[10] + P[41] * A[11]
		+ P[42] * A[12] + P[43] * A[13] + P[44] * A[14];
	PA[34] = P[38] + P[39] * dT;
	PA[35] = P[36] * A[15] + P[37] * A[16] + P[39] + P[40] * A[17] + P[41] * A[18]
		+ (P[43] - P[44]) * A[19];

	P[0] = A[0] * PA[0] + A[1] * PA[4] + A[2] * PA[8] + A[3] * PA[12] + A[4] * PA[24]
		+ A[5] * PA[28] + A[6] * PA[32];
	P[1] = A[7] * PA[0] + A[8] * PA[4] + A[9] * PA[8] + A[10] * PA[12] + A[11] * PA[20]
		+ A[12] * PA[24] + A[13] * PA[28] + A[14] * PA[32];
	P[2] = A[7] * PA[1] + A[8] * PA[5] + A[9] * PA[9] + A[10] * PA[13] + A[11] * PA[21]
		+ A[12] * PA[25] + A[13] * PA[29] + A[14] * PA[33];
	P[3] = PA[8] + dT * PA[12];
	P[4] = PA[9] + dT * PA[13];
	P[5] = PA[10] + dT * PA[14];
	P[6] = A[15] * PA[0] + A[16] * PA[4] + PA[12] + A[17] * PA[16] + A[18] * PA[20]
		+ A[19] * (PA[28] - PA[32]);
	P[7] = A[15] * PA[1] + A[16] * PA[5] + PA[13] + A[17] * PA[17] + A[18] * PA[21]
		+ A[19] * (PA[29] - PA[33]);
	P[8] = A[15] * PA[2] + A[16] * PA[6] + PA[14] + A[17] * PA[18] + A[18] * PA[22]
		+ A[19] * (PA[30] - PA[34]);
	P[9] = A[15] * PA[3] + A[16] * PA[7] + PA[15] + A[17] * PA[19] + A[18] * PA[23]
		+ A[19] * (PA[31] - PA[35]);
	P[10] = PA[16];
	P[11] = PA[17];
	P[12] = PA[18];
	P[13] = PA[19];
	P[15] = PA[20];
	P[16] = PA[21];
	P[17] = PA[22];
	P[18] = PA[23];
	P[21] = PA[24];
	P[22] = PA[25];
	P[23] = PA[26];
	P[24] = PA[27];
	P[28] = PA[28];
	P[29] = PA[29];
	P[30] = PA[30];
	P[31] = PA[31];
	P[36] = PA[32];
	P[37] = PA[33];
	P[38] = PA[34];
	P[39] = PA[35];
	}

	/* P = P + Q.
	 * */
	P[0] += pm->kQ[0];
	P[2] += pm->kQ[1];
	P[5] += pm->kQ[2];
	P[9] += pm->kQ[3];
	P[14] += pm->kQ[4];

	P[20] += (P[20] < pm->E_COV) ? pm->kQ[5] : 0.f;
	P[27] += (P[27] < pm->R_COV) ? pm->kQ[6] : 0.f;
	P[35] += (P[35] < pm->Ld_COV) ? pm->kQ[7] : 0.f;
	P[44] += (P[44] < pm->Lq_COV) ? pm->kQ[8] : 0.f;
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
	iSP = pm->wKP * eW + pm->wXX;

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

				if (pm->mBit & PMC_MODE_EKF_9X_BASE) {

					if (pm->mReq == PMC_REQ_BREAK)
						pm->mS1 = PMC_STATE_BREAK;
					else
						pm->mReq = PMC_REQ_NULL;
				}
				else {
					if (pm->mReq == PMC_REQ_CALIBRATE
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

				pm->Ad = 0.f;
				pm->Bd = 0.f;
				pm->kT[0] = 0.f;

				pm->timVal = 0;
				pm->timEnd = 64;

				pm->mS2++;
			}
			else {
				pm->Ad += -iA;
				pm->Bd += -iB;
				pm->kT[0] += uS - pm->U;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					/* Zero Drift.
					 * */
					pm->cA0 += pm->Ad / pm->timEnd;
					pm->cB0 += pm->Bd / pm->timEnd;

					/* Supply Voltage.
					 * */
					pm->U += pm->kT[0] / pm->timEnd;

					if (pm->mS2 == 1) {

						pm->Ad = 0.f;
						pm->Bd = 0.f;
						pm->kT[0] = 0.f;

						pm->timVal = 0;
						pm->timEnd = pm->hzF * pm->Tdrift;

						pm->mS2++;
					}
					else {
						if (pm->mReq == PMC_REQ_CALIBRATE)
							pm->mS1 = PMC_STATE_CALIBRATE;
						else if (pm->mReq == PMC_REQ_SPINUP)
							pm->mS1 = PMC_STATE_SPINUP;

						pm->mS2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_CALIBRATE:
			break;

		case PMC_STATE_SPINUP:

			if (pm->mS2 == 0) {

				int			j;

				pm->mBit |= PMC_MODE_EKF_9X_BASE
					| PMC_MODE_SPEED_CONTROL_LOOP;

				pm->kX[0] = 0.f;
				pm->kX[1] = 0.f;
				pm->kX[2] = 1.f;
				pm->kX[3] = 0.f;
				pm->kX[4] = 0.f;

				pm->Ad = 0.f;
				pm->Bd = 0.f;
				pm->M = 0.f;

				for (j = 0; j < 45; ++j)
					pm->kP[j] = 0.f;

				pm->kP[0] = 5e+6f;
				pm->kP[2] = 5e+6f;
				pm->kP[5] = 9.f;
				pm->kP[9] = 2e+1f;

				pm->kP[14] = 1.f;
				pm->kP[20] = pm->E_COV;
				pm->kP[27] = pm->R_COV;
				pm->kP[35] = pm->Ld_COV;
				pm->kP[44] = pm->Lq_COV;

				pm->wSP = 3000.f;

				pm->mReq = PMC_REQ_NULL;
				pm->mS1 = PMC_STATE_IDLE;
				pm->mS2 = 0;
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

	if (pm->mBit & PMC_MODE_EKF_9X_BASE) {

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

