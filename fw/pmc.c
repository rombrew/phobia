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

#include <stdio.h>
#include <math.h>

#define KPI			3.1415927f

inline float
klamp(float x, float h, float l)
{
	return (x > h) ? h : (x < l) ? l : x;
}

static float
ksin(float x)
{
	return sin(x);
}

static float
kcos(float x)
{
	return cos(x);
}

void pmcEnable(pmc_t *pm)
{
	pm->dT = 1.f / pm->hzF;

	pm->sT0 = .1f;		/* Zero Drift */
	pm->sT1 = .4f;		/* Ramp Up */
	pm->sT2 = .7f;		/* Hold */
	pm->sT3 = .5f;		/* Estimate RL */
	pm->sT4 = .0f;		/* Spin Up */
	pm->sT5 = .0f;		/*  */
	pm->sISP = 1.f;		/* Current SetPoint */
	pm->sFq = 20.f;		/* Frequency (Hz)*/
	pm->sAq = 75.f;		/* Acceleration */

	pm->cA0 = 0.f;
	pm->cA1 = .01464844f;
	pm->cB0 = 0.f;
	pm->cB1 = .01464844f;
	pm->cU0 = 0.f;
	pm->cU1 = .00725098f;

	pm->iKP = 1e-5f;
	pm->iKI = 2e-3f;

	pm->kQ[0] = 1e-8;
	pm->kQ[1] = 1e-8;
	pm->kQ[2] = 1e-8;
	pm->kQ[3] = 1e-8;
	pm->kQ[4] = 1e-6;
	pm->kQ[5] = 1e-12;
	pm->kQ[6] = 1e-2;
	pm->kQ[7] = 1e-16;
	pm->kQ[8] = 0e-10;
	pm->kQ[9] = 0e-10;
	pm->kR = 1e-2;
}

static void
dEq(pmc_t *pm, float D[], float X[])
{
	float		pX, pY, uD, uQ;

	pX = kcos(X[2]);
	pY = ksin(X[2]);

	uD = pX * pm->uX + pY * pm->uY;
	uQ = pX * pm->uY - pY * pm->uX;

	D[0] = X[6] * (uD - X[0] * X[5]) + X[3] * X[1];
	D[1] = X[6] * (uQ - X[1] * X[5] - X[3] * X[7]) - X[3] * X[0];
	D[2] = X[3];
	D[3] = pm->Zp * pm->IJ * (1.5f * X[7] * pm->Zp * X[1] - X[4]);
}

static void
sFC(pmc_t *pm)
{
	float		*X = pm->kX;
	float		D1[4], D2[4], X2[8];
	float		dT;

	dEq(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	X2[2] = X[2] + D1[2] * dT;
	X2[3] = X[3] + D1[3] * dT;
	X2[4] = X[4];
	X2[5] = X[5];
	X2[6] = X[6];
	X2[7] = X[7];
	X2[8] = X[8];
	X2[9] = X[9];

	X2[2] = (X2[2] < -KPI) ? X2[2] + 2.f * KPI : (X2[2] > KPI) ? X2[2] - 2.f * KPI : X2[2];

	dEq(pm, D2, X2);
	dT *= .5f;

	X[0] += (D1[0] + D2[0]) * dT;
	X[1] += (D1[1] + D2[1]) * dT;
	X[2] += (D1[2] + D2[2]) * dT;
	X[3] += (D1[3] + D2[3]) * dT;

	X[2] = (X[2] < -KPI) ? X[2] + 2.f * KPI : (X[2] > KPI) ? X[2] - 2.f * KPI : X[2];
}

static void
sFB(pmc_t *pm, float iX, float iY)
{
	float		*X = pm->kX, *K = pm->kK;
	float		iD, iQ, eD, eQ;

	iD = pm->pX * iX + pm->pY * iY;
	iQ = pm->pX * iY - pm->pY * iX;

	eD = iD - (X[0] + pm->pX * X[8] + pm->pY * X[9]);
	eQ = iQ - (X[1] - pm->pY * X[8] + pm->pX * X[9]);

	pm->eD = eD;
	pm->eQ = eQ;

	X[0] += K[0] * eD + K[1] * eQ;
	X[1] += K[2] * eD + K[3] * eQ;
	X[2] += K[4] * eD + K[5] * eQ;
	X[3] += K[6] * eD + K[7] * eQ;
	X[4] += K[8] * eD + K[9] * eQ;
	X[5] += K[10] * eD + K[11] * eQ;
	X[6] += K[12] * eD + K[13] * eQ;
	X[7] += K[14] * eD + K[15] * eQ;
	X[8] += K[16] * eD + K[17] * eQ;
	X[9] += K[18] * eD + K[19] * eQ;

	X[2] = (X[2] < -KPI) ? X[2] + 2.f * KPI : (X[2] > KPI) ? X[2] - 2.f * KPI : X[2];

	sFC(pm);
}

static void
mTrans(float *X, const float *A, int M, int N)
{
	int		i, j;

	for (i = 0; i < M; ++i)
		for (j = 0; j < N; ++j) {

			X[j * M + i] = A[i * N + j];
		}
}

static void
mMul(float *X, const float *A, const float *B, int M, int N, int K)
{
	int		i, j, k;
	float		s;

	for (i = 0; i < M; ++i)
		for (j = 0; j < N; ++j) {

			s = 0.f;
			for (k = 0; k < K; ++k)
				s += A[i * K + k] * B[k * N + j];

			X[i * N + j] = s;
		}
}

static void
mSub(float *X, const float *A, int M, int N)
{
	int		i, j;

	for (i = 0; i < M; ++i)
		for (j = 0; j < N; ++j) {

			X[i * N + j] -= A[i * N + j];
		}
}

static void
mZero(float *X, int M, int N)
{
	int		i, j;

	for (i = 0; i < M; ++i)
		for (j = 0; j < N; ++j) {

			X[i * N + j] = 0.f;
		}
}

static void
mPrt(const char *title, const float *X, int M, int N)
{
	int		i, j;

	printf("%s:\n", title);

	for (i = 0; i < M; ++i) {
		for (j = 0; j < N; ++j) {

			printf("%e ", X[i * N + j]);
		}
		printf("\n");
	}

	printf("\n");
}

static void
sKF(pmc_t *pm)
{
	float		*X = pm->kX, *P = pm->kP;
	float		*Q = pm->kQ, *K = pm->kK;
	float		A[100], C[20], AT[100], AP[100];
	float		CP[20], CT[20], S[4], IS[4];
	float		PC[20], KCP[100];
	float		Sd, dT = pm->dT;

	A[0*10 + 0] = 1.f - X[5] * X[6] * dT;
	A[0*10 + 1] = X[3] * dT;
	A[0*10 + 2] = X[6] * (- pm->pY * pm->uX + pm->pX * pm->uY) * dT;
	A[0*10 + 3] = X[1] * dT;
	A[0*10 + 4] = 0.f;
	A[0*10 + 5] = - X[0] * X[6] * dT;
	A[0*10 + 6] = (pm->pX * pm->uX + pm->pY * pm->uY - X[0] * X[5]) * dT;
	A[0*10 + 7] = 0.f;
	A[0*10 + 8] = 0.f;
	A[0*10 + 9] = 0.f;

	A[1*10 + 0] = - X[3] * dT;
	A[1*10 + 1] = 1.f - X[5] * X[6] * dT;
	A[1*10 + 2] = X[6] * (- pm->pX * pm->uX - pm->pY * pm->uY) * dT;
	A[1*10 + 3] = - (X[7] * X[6] + X[0]) * dT;
	A[1*10 + 4] = 0.f;
	A[1*10 + 5] = - X[1] * X[6] * dT;
	A[1*10 + 6] = (pm->pX * pm->uY - pm->pY * pm->uX - X[1] * X[5] - X[3] * X[7]) * dT;
	A[1*10 + 7] = - X[3] * X[6] * dT;
	A[1*10 + 8] = 0.f;
	A[1*10 + 9] = 0.f;

	A[2*10 + 0] = 0.f;
	A[2*10 + 1] = 0.f;
	A[2*10 + 2] = 1.f;
	A[2*10 + 3] = dT;
	A[2*10 + 4] = 0.f;
	A[2*10 + 5] = 0.f;
	A[2*10 + 6] = 0.f;
	A[2*10 + 7] = 0.f;
	A[2*10 + 8] = 0.f;
	A[2*10 + 9] = 0.f;

	A[3*10 + 0] = 0.f;
	A[3*10 + 1] = 1.5f * pm->Zp * pm->Zp * pm->IJ * X[7] * dT;
	A[3*10 + 2] = 0.f;
	A[3*10 + 3] = 1.f;
	A[3*10 + 4] = - pm->Zp * pm->IJ * dT;
	A[3*10 + 5] = 0.f;
	A[3*10 + 6] = 0.f;
	A[3*10 + 7] = 1.5f * pm->Zp * pm->Zp * pm->IJ * X[1] * dT;
	A[3*10 + 8] = 0.f;
	A[3*10 + 9] = 0.f;

	A[4*10 + 0] = 0.f;
	A[4*10 + 1] = 0.f;
	A[4*10 + 2] = 0.f;
	A[4*10 + 3] = 0.f;
	A[4*10 + 4] = 1.f;
	A[4*10 + 5] = 0.f;
	A[4*10 + 6] = 0.f;
	A[4*10 + 7] = 0.f;
	A[4*10 + 8] = 0.f;
	A[4*10 + 9] = 0.f;

	A[5*10 + 0] = 0.f;
	A[5*10 + 1] = 0.f;
	A[5*10 + 2] = 0.f;
	A[5*10 + 3] = 0.f;
	A[5*10 + 4] = 0.f;
	A[5*10 + 5] = 1.f;
	A[5*10 + 6] = 0.f;
	A[5*10 + 7] = 0.f;
	A[5*10 + 8] = 0.f;
	A[5*10 + 9] = 0.f;

	A[6*10 + 0] = 0.f;
	A[6*10 + 1] = 0.f;
	A[6*10 + 2] = 0.f;
	A[6*10 + 3] = 0.f;
	A[6*10 + 4] = 0.f;
	A[6*10 + 5] = 0.f;
	A[6*10 + 6] = 1.f;
	A[6*10 + 7] = 0.f;
	A[6*10 + 8] = 0.f;
	A[6*10 + 9] = 0.f;

	A[7*10 + 0] = 0.f;
	A[7*10 + 1] = 0.f;
	A[7*10 + 2] = 0.f;
	A[7*10 + 3] = 0.f;
	A[7*10 + 4] = 0.f;
	A[7*10 + 5] = 0.f;
	A[7*10 + 6] = 0.f;
	A[7*10 + 7] = 1.f;
	A[7*10 + 8] = 0.f;
	A[7*10 + 9] = 0.f;

	A[8*10 + 0] = 0.f;
	A[8*10 + 1] = 0.f;
	A[8*10 + 2] = 0.f;
	A[8*10 + 3] = 0.f;
	A[8*10 + 4] = 0.f;
	A[8*10 + 5] = 0.f;
	A[8*10 + 6] = 0.f;
	A[8*10 + 7] = 0.f;
	A[8*10 + 8] = 1.f;
	A[8*10 + 9] = 0.f;

	A[9*10 + 0] = 0.f;
	A[9*10 + 1] = 0.f;
	A[9*10 + 2] = 0.f;
	A[9*10 + 3] = 0.f;
	A[9*10 + 4] = 0.f;
	A[9*10 + 5] = 0.f;
	A[9*10 + 6] = 0.f;
	A[9*10 + 7] = 0.f;
	A[9*10 + 8] = 0.f;
	A[9*10 + 9] = 1.f;

	C[0*10 + 0] = 1.f;
	C[0*10 + 1] = 0.f;
	C[0*10 + 2] = - X[1];
	C[0*10 + 3] = 0.f;
	C[0*10 + 4] = 0.f;
	C[0*10 + 5] = 0.f;
	C[0*10 + 6] = 0.f;
	C[0*10 + 7] = 0.f;
	C[0*10 + 8] = pm->pX;
	C[0*10 + 9] = pm->pY;

	C[1*10 + 0] = 0.f;
	C[1*10 + 1] = 1.f;
	C[1*10 + 2] = X[0];
	C[1*10 + 3] = 0.f;
	C[1*10 + 4] = 0.f;
	C[1*10 + 5] = 0.f;
	C[1*10 + 6] = 0.f;
	C[1*10 + 7] = 0.f;
	C[1*10 + 8] = - pm->pY;
	C[1*10 + 9] = pm->pX;

	mTrans(AT, A, 10, 10);
	mMul(AP, A, P, 10, 10, 10);
	mMul(P, AP, AT, 10, 10, 10);

	P[0*10 + 0] += Q[0];
	P[1*10 + 1] += Q[1];
	P[2*10 + 2] += Q[2];
	P[3*10 + 3] += Q[3];
	P[4*10 + 4] += Q[4];
	P[5*10 + 5] += Q[5];
	P[6*10 + 6] += Q[6];
	P[7*10 + 7] += Q[7];
	P[8*10 + 8] += Q[8];
	P[9*10 + 9] += Q[9];

	mTrans(CT, C, 2, 10);
	mMul(CP, C, P, 2, 10, 10);
	mMul(S, CP, CT, 2, 2, 10);

	S[0] += pm->kR;
	S[3] += pm->kR;

	Sd = S[0] * S[3] - S[1] * S[2];
	IS[0] = S[3] / Sd;
	IS[1] = -S[1] / Sd;
	IS[2] = -S[2] / Sd;
	IS[3] = S[0] / Sd;

	mMul(PC, P, CT, 10, 2, 10);
	mMul(K, PC, IS, 10, 2, 2);

	mMul(KCP, K, CP, 10, 10, 2);
	mSub(P, KCP, 10, 10);
}

static void
iFB(pmc_t *pm)
{
	float		eD, eQ, uD, uQ, Q;
	float		uA, uB, uC, uX, uY;
	int		xA, xB, xC;

	eD = pm->iSPD - pm->kX[0];
	eQ = pm->iSPQ - pm->kX[1];

	pm->iXD = klamp(pm->iXD + pm->iKI * eD, 1.f, -1.f);
	uD = pm->iXD + pm->iKP * eD;

	pm->iXQ = klamp(pm->iXQ + pm->iKI * eQ, 1.f, -1.f);
	uQ = pm->iXQ + pm->iKP * eQ;

	/*{
		static int	j = 0;

		++j;

		if (j > 20000 * 2) {

			uD = -.2f;
			uQ = 1.1f;
		}
	}*/

	uD = klamp(uD, 1.f, -1.f);
	uQ = klamp(uQ, 1.f, -1.f);

	uX = pm->pX * uD - pm->pY * uQ;
	uY = pm->pY * uD + pm->pX * uQ;

	uA = uX;
	uB = -.5f * uX + .8660254f * uY;
	uC = -.5f * uX - .8660254f * uY;

	uA = klamp(.5f * uA + .5f, 1.f, 0.f);
	uB = klamp(.5f * uB + .5f, 1.f, 0.f);
	uC = klamp(.5f * uC + .5f, 1.f, 0.f);

	xA = (int) (pm->pwmR * uA + .5f);
	xB = (int) (pm->pwmR * uB + .5f);
	xC = (int) (pm->pwmR * uC + .5f);

	pm->pDC(xA, xB, xC);

	Q = .33333333f * (xA + xB + xC);
	uA = (xA - Q) * pm->U / pm->pwmR;
	uB = (xB - Q) * pm->U / pm->pwmR;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	pm->uX = uX;
	pm->uY = uY;
}

static void
wFB(pmc_t *pm)
{
}

static void
bFSM(pmc_t *pm, float iA, float iB, float uS, float iX, float iY)
{
	float		L, P;

	switch (pm->fST1) {

		case PMC_STATE_IDLE:
			break;

		case PMC_STATE_DRIFT:

			if (pm->fST2 == 0) {

				pm->tA.re = 0.f;
				pm->tB.re = 0.f;
				pm->tC.re = 0.f;

				pm->timVal = 0;
				pm->timEnd = 64;

				pm->fST2++;
			}
			else {
				pm->tA.re += -iA;
				pm->tB.re += -iB;
				pm->tC.re += uS - pm->U;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					/* Zero Drift */

					pm->cA0 += pm->tA.re / pm->timEnd;
					pm->cB0 += pm->tB.re / pm->timEnd;
					pm->U += pm->tC.re / pm->timEnd;

					if (pm->fST2 == 1) {

						pm->tA.re = 0.f;
						pm->tB.re = 0.f;
						pm->tC.re = 0.f;

						pm->timVal = 0;
						pm->timEnd = pm->hzF * pm->sT0;

						pm->fST2++;
					}
					else {
						pm->fMOF |= PMC_MODE_VOLTAGE_ESTIMATE;

						if (pm->fMOF & PMC_MODE_CALIBRATE)
							pm->fST1 = PMC_STATE_CALIBRATE;
						else
							pm->fST1 = PMC_STATE_ALIGN;

						pm->fST2 = 0;
					}
				}
			}
			break;

		case PMC_STATE_CALIBRATE:
			break;

		case PMC_STATE_ALIGN:

			if (pm->fST2 == 0) {

				pm->fMOF |= PMC_MODE_CURRENT_LOOP;

				pm->iSPD = 0.f;
				pm->iSPQ = 0.f;

				pm->iXD = 0.f;
				pm->iXQ = 0.f;

				pm->pX = 1.f;
				pm->pY = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT1;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				/* Ramp Up */

				pm->iSPD = pm->sISP * pm->timVal / pm->timEnd;
				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->timVal = 0;
					pm->timEnd = pm->hzF * pm->sT2;

					pm->fST2++;
				}
			}
			else if (pm->fST2 == 2) {

				/* Hold */

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					if (pm->fMOF & PMC_MODE_ESTIMATE_RL)
						pm->fST1 = PMC_STATE_ESTIMATE_RL;
					else
						pm->fST1 = PMC_STATE_SPINUP;

					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_ESTIMATE_RL:

			if (pm->fST2 == 0) {

				pm->tA.re = 1.f;
				pm->tA.im = 0.f;
				pm->tB.re = 0.f;
				pm->tB.im = 0.f;
				pm->tC.re = 0.f;
				pm->tC.im = 0.f;
				pm->tD.re = iX;
				pm->tD.im = 6.2831853f * pm->sFq * pm->dT;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT3;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				pm->tA.re += -pm->tA.im * pm->tD.im;
				pm->tA.im += pm->tA.re * pm->tD.im;

				L = (3.f - pm->tA.re * pm->tA.re - pm->tA.im * pm->tA.im) * .5f;
				pm->tA.re *= L;
				pm->tA.im *= L;

				L = .5f * (pm->tD.re + iX);
				pm->tB.re += L * pm->tA.re;
				pm->tB.im += L * pm->tA.im;

				L = pm->uX;
				pm->tC.re += L * pm->tA.re;
				pm->tC.im += L * pm->tA.im;

				pm->iSPD = pm->sISP * (pm->tA.re + 1.f) * .5f;
				pm->tD.re = iX;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->tA.re = pm->tC.re * pm->tB.re + pm->tC.im * pm->tB.im;
					pm->tA.im = pm->tC.re * pm->tB.im - pm->tC.im * pm->tB.re;

					L = pm->tB.re * pm->tB.re + pm->tB.im * pm->tB.im;
					pm->tA.re /= L;
					pm->tA.im /= L;

					pm->R = pm->tA.re;
					pm->IL = 6.2831853f * pm->sFq / pm->tA.im;

					pm->fST1 = PMC_STATE_END;
					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_SPINUP:

			if (pm->fST2 == 0) {

				pm->tA.re = 0.f;

				pm->timVal = 0;
				pm->timEnd = pm->hzF * pm->sT4;

				pm->fST2++;
			}
			else if (pm->fST2 == 1) {

				P = pm->sAq * pm->dT;
				L = (pm->tA.re + .5f * P) * pm->dT;
				pm->tA.re += P;

				pm->pX += -pm->pY * L;
				pm->pY += pm->pX * L;

				L = (3.f - pm->pX * pm->pX - pm->pY * pm->pY) * .5f;
				pm->pX *= L;
				pm->pY *= L;

				pm->timVal++;

				if (pm->timVal < pm->timEnd) ;
				else {
					pm->fMOF |= PMC_MODE_OBSERVER;

					pm->iSPD = 0.f;
					pm->iSPQ = pm->sISP;

					pm->kX[0] = 0.f;
					pm->kX[1] = 0.f;
					pm->kX[2] = 0.f;
					pm->kX[3] = 0.f;
					pm->kX[4] = 0.f;
					pm->kX[5] = pm->R;
					pm->kX[6] = pm->IL;
					pm->kX[7] = pm->E;
					pm->kX[8] = 0.f;
					pm->kX[9] = 0.f;

					mZero(pm->kP, 10, 10);

					pm->kP[0*10 + 0] = 1e+2;
					pm->kP[1*10 + 1] = 1e+2;
					pm->kP[2*10 + 2] = 1e-2;
					pm->kP[3*10 + 3] = 1e-2;
					pm->kP[4*10 + 4] = 1e-2;
					pm->kP[5*10 + 5] = 0.f + 0e-4;
					pm->kP[6*10 + 6] = 0.f + 0e+6;
					pm->kP[7*10 + 7] = 0.f + 0e-8;
					pm->kP[8*10 + 8] = 0.f;
					pm->kP[9*10 + 9] = 0.f;

					sKF(pm);

					pm->fST1 = PMC_STATE_IDLE;
					pm->fST2 = 0;
				}
			}
			break;

		case PMC_STATE_END:
			break;
	}
}

void pmcFeedBack(pmc_t *pm, int xA, int xB, int xU)
{
	float		iA, iB, uS;
	float		iX, iY;

	/* Conversion to Ampere and Volt.
	 * */
	iA = pm->cA1 * (xA - 2048) + pm->cA0;
	iB = pm->cB1 * (xB - 2048) + pm->cB0;
	uS = pm->cU1 * xU + pm->cU0;

	/* Transform from ABC to XY axes.
	 * */
	iX = iA;
	iY = .57735027f * iA + 1.1547005f * iB;

	/* Call FSM.
	 * */
	bFSM(pm, iA, iB, uS, iX, iY);

	/* State observer.
	 * */
	if (pm->fMOF & PMC_MODE_OBSERVER) {

		/*static int j = 0;

		++j;

		if (j > 20000 * 3) {

			pm->iSPD = ksin((float) j * 5.f / 20e+3);
			pm->iSPQ = 2.f + kcos((float) j * 5.f / 20e+3);
		}*/

		sFB(pm, iX, iY);

		pm->pX = kcos(pm->kX[2]);
		pm->pY = ksin(pm->kX[2]);
	}
	else {
		pm->kX[0] = pm->pX * iX + pm->pY * iY;
		pm->kX[1] = pm->pX * iY - pm->pY * iX;
	}

	/* Source voltage estimate.
	 * */
	if (pm->fMOF & PMC_MODE_VOLTAGE_ESTIMATE) {

		pm->U += (uS - pm->U) * pm->gainU;
	}

	/* Current control loop.
	 * */
	if (pm->fMOF & PMC_MODE_CURRENT_LOOP) {

		iFB(pm);

		/* Speed control loop.
		 * */
		if (pm->fMOF & PMC_MODE_SPEED_LOOP) {
		}
	}
	else
		pm->pDC(0, 0, 0);

	if (pm->fMOF & PMC_MODE_OBSERVER) {

		/* Kalman filter.
		 * */
		sKF(pm);
	}
}

