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

	pm->iKP = 2e-2f;
	pm->iKI = 5e-3f;

	pm->kQ[0] = 1e-4;
	pm->kQ[1] = 1e-4;
	pm->kQ[2] = 1e-6;
	pm->kQ[3] = 1e-6;
	pm->kQ[4] = 1e-4;
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

	D[0] = pm->IL * (uD - X[0] * pm->R) + X[3] * X[1];
	D[1] = pm->IL * (uQ - X[1] * pm->R - X[3] * pm->E) - X[3] * X[0];
	D[2] = X[3];
	D[3] = pm->Zp * pm->IJ * (1.5f * pm->E * pm->Zp * X[1] - X[4]);
}

static void
sFC(pmc_t *pm)
{
	float		*X = pm->kX;
	float		D1[4], D2[4], X2[5];
	float		dT;

	dEq(pm, D1, X);
	dT = pm->dT;

	X2[0] = X[0] + D1[0] * dT;
	X2[1] = X[1] + D1[1] * dT;
	X2[2] = X[2] + D1[2] * dT;
	X2[3] = X[3] + D1[3] * dT;
	X2[4] = X[4];

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
	float		pX, pY, hX, hY, eX, eY;

	pX = kcos(X[2]);
	pY = ksin(X[2]);

	hX = pX * X[0] - pY * X[1];
	hY = pY * X[0] + pX * X[1];

	eX = iX - hX;
	eY = iY - hY;

	X[0] += K[0] * eX + K[1] * eY;
	X[1] += K[2] * eX + K[3] * eY;
	X[2] += K[4] * eX + K[5] * eY;
	X[3] += K[6] * eX + K[7] * eY;
	X[4] += K[8] * eX + K[9] * eY;

	X[2] = (X[2] < -KPI) ? X[2] + 2.f * KPI : (X[2] > KPI) ? X[2] - 2.f * KPI : X[2];

	/* FIXME */
	pm->E += (- pY * eX + pX * eY) * -1e-5f;

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
	float		A[25], C[10], AT[25], AP[25];
	float		CP[10], CT[10], S[4], IS[4];
	float		PC[10], KCP[25];
	float		pX, pY;
	float		Sd, dT = pm->dT;

	pX = kcos(X[2]);
	pY = ksin(X[2]);

	A[0*5 + 0] = 1.f - pm->R * pm->IL * dT;
	A[0*5 + 1] = X[3] * dT;
	A[0*5 + 2] = - pY * pm->uX + pX * pm->uY;
	A[0*5 + 3] = X[1] * dT;
	A[0*5 + 4] = 0.f;

	A[1*5 + 0] = - X[3] * dT;
	A[1*5 + 1] = 1.f - pm->R * pm->IL * dT;
	A[1*5 + 2] = - pX * pm->uX - pY * pm->uY;
	A[1*5 + 3] = - (pm->E * pm->IL + X[0]) * dT;
	A[1*5 + 4] = 0.f;

	A[2*5 + 0] = 0.f;
	A[2*5 + 1] = 0.f;
	A[2*5 + 2] = 1.f;
	A[2*5 + 3] = dT;
	A[2*5 + 4] = 0.f;

	A[3*5 + 0] = 0.f;
	A[3*5 + 1] = 1.5f * pm->Zp * pm->Zp * pm->IJ * pm->E * dT;
	A[3*5 + 2] = 0.f;
	A[3*5 + 3] = 1.f;
	A[3*5 + 4] = - pm->Zp * pm->IJ * dT;

	A[4*5 + 0] = 0.f;
	A[4*5 + 1] = 0.f;
	A[4*5 + 2] = 0.f;
	A[4*5 + 3] = 0.f;
	A[4*5 + 4] = 1.f;

	C[0*5 + 0] = pX;
	C[0*5 + 1] = - pY;
	C[0*5 + 2] = - pY * X[0] - pX * X[1];
	C[0*5 + 3] = 0.f;
	C[0*5 + 4] = 0.f;

	C[1*5 + 0] = pY;
	C[1*5 + 1] = pX;
	C[1*5 + 2] = pX * X[0] - pY * X[1];
	C[1*5 + 3] = 0.f;
	C[1*5 + 4] = 0.f;

	/*mPrt("_X", X, 5, 1);
	mPrt("_A", A, 5, 5);
	mPrt("_C", C, 2, 5);*/
	
	/*{
		double		Y0[5], X0[5], Y[5], X[5], Z0[2], Z[2], dX;
		int		i, j;

		X0[0] = pm->kX[0];
		X0[1] = pm->kX[1];
		X0[2] = pm->kX[2];
		X0[3] = pm->kX[3];
		X0[4] = pm->kX[4];

		pmF(Y0, X0, 0);
		pmH(Z0, X0);

		dX = 1e-3;

		for (i = 0; i < 5; ++i) {

			for (j = 0; j < 5; ++j)
				X[j] = X0[j];

			X[i] += dX;

			pmF(Y, X, 0);
			pmH(Z, X);

			for (j = 0; j < 5; ++j)
				A[j * 5 + i] = (Y[j] - Y0[j]) / dX;

			for (j = 0; j < 2; ++j)
				C[j * 5 + i] = (Z[j] - Z0[j]) / dX;
		}
	}*/

	/*mPrt("A", A, 5, 5);
	mPrt("C", C, 2, 5);
	exit(0);*/

	mTrans(AT, A, 5, 5);
	mMul(AP, A, P, 5, 5, 5);
	mMul(P, AP, AT, 5, 5, 5);

	P[0*5 + 0] += Q[0];
	P[1*5 + 1] += Q[1];
	P[2*5 + 2] += Q[2];
	P[3*5 + 3] += Q[3];
	P[4*5 + 4] += Q[4];

	mTrans(CT, C, 2, 5);
	mMul(CP, C, P, 2, 5, 5);
	mMul(S, CP, CT, 2, 2, 5);

	S[0] += pm->kR;
	S[3] += pm->kR;

	Sd = S[0] * S[3] - S[1] * S[2];
	IS[0] = S[3] / Sd;
	IS[1] = -S[1] / Sd;
	IS[2] = -S[2] / Sd;
	IS[3] = S[0] / Sd;

	mMul(PC, P, CT, 5, 2, 5);
	mMul(K, PC, IS, 5, 2, 2);

	mMul(KCP, K, CP, 5, 5, 2);
	mSub(P, KCP, 5, 5);
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

					mZero(pm->kP, 5, 5);

					pm->kP[0*5 + 0] = 1e+2;
					pm->kP[1*5 + 1] = 1e+2;
					pm->kP[2*5 + 2] = 1e-2;
					pm->kP[3*5 + 3] = 1e-2;
					pm->kP[4*5 + 4] = 1e-2;

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

