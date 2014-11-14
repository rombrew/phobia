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

#include <math.h>

#include "ukf.h"

__inline double
SQD(double X) { return X * X; }

static int
mLen(int N) { return N * (N + 1) / 2; }

static void
vZero(double *X, int N)
{
	int		i;

	for (i = 0; i < N; ++i)
		X[i] = 0.;
}

static void
vCopy(double *X, const double *A, int N)
{
	int		i;

	for (i = 0; i < N; ++i)
		X[i] = A[i];
}

static void
vDiv(double *X, double D, int N)
{
	int		i;

	for (i = 0; i < N; ++i)
		X[i] /= D;
}

static void
vAdd(double *X, const double *A, int N)
{
	int		i;

	for (i = 0; i < N; ++i)
		X[i] += A[i];
}

static void
vSub(double *X, const double *A, const double *B, int N)
{
	int		i;

	for (i = 0; i < N; ++i)
		X[i] = A[i] - B[i];
}

static void
mChol(double *A, int N)
{
	double		S;
	int		i, j, k;
	
	for (k = 0; k < N; ++k) {
	
		for (i = 0; i < k; ++i) {
		
			S = 0.;
			for (j = 0; j < i; ++j)
				S += LOW(A, i, j) * LOW(A, k, j);
			LOW(A, k, i) = (LOW(A, k, i) - S) / LOW(A, i, i);
		}
		
		S = 0.;
		for (j = 0; j < k; ++j)
			S += SQD(LOW(A, k, j));
		LOW(A, k, k) = sqrt(LOW(A, k, k) - S);
	}
}

static void
mRDiv(double *X, const double *A, double *B, int N, int M)
{
	double		S;
	int		k, i, j;
	
	mChol(B, N);

	for (k = 0; k < M; ++k) {
	
		for (i = 0; i < N; ++i) {
		
			S = 0.;
			for (j = 0; j < i; ++j)
				S += X[j] * LOW(B, i, j);
			X[i] = (A[i] - S) / LOW(B, i, i);
		}
		
		for (i = N - 1; i >= 0; --i) {
		
			S = 0.;
			for (j = N - 1; j > i; --j)
				S += X[j] * LOW(B, j, i);
			X[i] = (X[i] - S) / LOW(B, i, i);
		}
	
		X += N;
		A += N;
	}
}

static void *
dAlloc(void **pMem, int sZ)
{
	void	*pA = *pMem;

	sZ = (sZ + 7) & ~7;
	*pMem = ((char *) *pMem) + sZ;

	return pA;
}

ukfT *ukfAlloc(void *pMem, int N, int M)
{
	ukfT		*Kf;
	
	Kf = dAlloc(&pMem, sizeof(ukfT));
	Kf->N = N;
	Kf->M = M;
	
	Kf->P = dAlloc(&pMem, sizeof(double) * N * (N + 1) / 2);
	Kf->X = dAlloc(&pMem, sizeof(double) * N);
	
	Kf->Q = dAlloc(&pMem, sizeof(double) * N * (N + 1) / 2);
	Kf->R = dAlloc(&pMem, sizeof(double) * M * (M + 1) / 2);
	
	vZero(Kf->P, mLen(N));
	vZero(Kf->X, N);
	
	vZero(Kf->Q, mLen(N));
	vZero(Kf->R, mLen(M));
	
	Kf->K = dAlloc(&pMem, sizeof(double) * N * M);
	
	Kf->YL = dAlloc(&pMem, sizeof(double) * 2 * N * N);
	Kf->ZL = dAlloc(&pMem, sizeof(double) * 2 * N * M);
	
	Kf->Z = dAlloc(&pMem, sizeof(double) * M);
	Kf->PZZ = dAlloc(&pMem, sizeof(double) * M * (M + 1) / 2);
	Kf->PYZ = dAlloc(&pMem, sizeof(double) * N * M);
	
	Kf->pEnd = pMem;
	
	return Kf;
}

void ukfForecast(ukfT *Kf, const double *U)
{
	double		*X = Kf->PZZ, *pYL, *pZL;
	int		k, j;

	mChol(Kf->P, Kf->N);
	
	pYL = Kf->YL;
	pZL = Kf->ZL;

	for (k = 0; k < Kf->N; ++k) {
	
		for (j = k; j < Kf->N; ++j)
			X[j] = Kf->X[j] + LOW(Kf->P, j, k);
		
		Kf->pF(pYL, X, U);
		Kf->pH(pZL, pYL);
		
		pYL += Kf->N;
		pZL += Kf->M;
		
		for (j = k; j < Kf->N; ++j)
			X[j] = Kf->X[j] - LOW(Kf->P, j, k);
		
		Kf->pF(pYL, X, U);
		Kf->pH(pZL, pYL);
		
		pYL += Kf->N;
		pZL += Kf->M;
		
		X[k] = Kf->X[k];
	}
	
	vZero(Kf->X, Kf->N);
	vZero(Kf->Z, Kf->M);
	
	pYL = Kf->YL;
	pZL = Kf->ZL;
	
	for (k = 0; k < Kf->N * 2; ++k) {
	
		vAdd(Kf->X, pYL, Kf->N);
		vAdd(Kf->Z, pZL, Kf->M);
		
		pYL += Kf->N;
		pZL += Kf->M;
	}
	
	vDiv(Kf->X, Kf->N * 2., Kf->N);
	vDiv(Kf->Z, Kf->N * 2., Kf->M);
}

void ukfUpdate(ukfT *Kf)
{
	double		S, *pYL, *pZL;
	int		k, i, j;
	
	vZero(Kf->P, mLen(Kf->N));
	vZero(Kf->PZZ, mLen(Kf->M));
	vZero(Kf->PYZ, Kf->N * Kf->M);
	
	pYL = Kf->YL;
	pZL = Kf->ZL;
	
	for (k = 0; k < Kf->N * 2; ++k) {
	
		for (i = 0; i < Kf->N; ++i)
			for (j = 0; j < i + 1; ++j) {
			
				LOW(Kf->P, i, j) += (pYL[i] - Kf->X[i])
					* (pYL[j] - Kf->X[j]);
			}
			
		for (i = 0; i < Kf->M; ++i)
			for (j = 0; j < i + 1; ++j) {
			
				LOW(Kf->PZZ, i, j) += (pZL[i] - Kf->Z[i])
					* (pZL[j] - Kf->Z[j]);
			}
			
		for (i = 0; i < Kf->N; ++i)
			for (j = 0; j < Kf->M; ++j) {
			
				Kf->PYZ[i * Kf->M + j] += (pYL[i] - Kf->X[i])
					* (pZL[j] - Kf->Z[j]);
			}
			
		pYL += Kf->N;
		pZL += Kf->M;
	}
	
	vDiv(Kf->P, 2., mLen(Kf->N));
	vAdd(Kf->P, Kf->Q, mLen(Kf->N));
	
	vDiv(Kf->PZZ, 2., mLen(Kf->M));
	vAdd(Kf->PZZ, Kf->R, mLen(Kf->M));
	vDiv(Kf->PYZ, 2., Kf->N * Kf->M);

	mRDiv(Kf->K, Kf->PYZ, Kf->PZZ, Kf->M, Kf->N);
	
	for (i = 0; i < Kf->N; ++i)
		for (j = 0; j < i + 1; ++j) {
		
			S = 0.;
			for (k = 0; k < Kf->M; ++k)
				S += Kf->PYZ[i * Kf->M + k]
					* Kf->K[j * Kf->M + k];
			LOW(Kf->P, i, j) -= S;
		}
}

void ukfCorrect(ukfT *Kf, const double *Z)
{
	double		S;
	int		k, j;
	
	vSub(Kf->Z, Z, Kf->Z, Kf->M);
	
	for (k = 0; k < Kf->N; ++k) {
	
		S = 0.;
		for (j = 0; j < Kf->M; ++j)
			S += Kf->K[k * Kf->M + j] * Kf->Z[j];
		Kf->X[k] += S;
	}
}

/*
void testF(double *Y, const double *X, const double *U)
{
	const double		dT = 1e-2;
	
	Y[1] = X[1] + X[0] * dT + U[0] * dT * dT * .5;
	Y[0] = X[0] + U[0] * dT;
}

void testH(double *Z, const double *X)
{
	Z[0] = X[1];
}

double gauss();

void ukfTest()
{
	void		*pMem;
	ukfT		*Kf;
	int		j;
	
	double		T[1];
	
	pMem = malloc(1UL << 20);
	
	Kf = ukfAlloc(pMem, 2, 1);
	
	printf("MEM %i\n", (char *) Kf->pEnd - (char *) pMem);
	
	LOW(Kf->P, 0, 0) = 1.e+2;
	LOW(Kf->P, 1, 1) = 1.e+4;
	
	LOW(Kf->R, 0, 0) = 1.;
	
	Kf->pF = &testF;
	Kf->pH = &testH;
	
	for (j = 0; j < 100; ++j) {
	
		T[0] = 0.;
		ukfForecast(Kf, T);
		ukfUpdate(Kf);
		T[0] = 100. + gauss();
		ukfCorrect(Kf, T);
	
		printf("X = %lf %lf\n",
			Kf->X[0], Kf->X[1]);
		printf("P = %lf %lf %lf\n",
			Kf->P[0], Kf->P[1], Kf->P[2]);
		printf("K = %lf %lf\n",
			Kf->K[0], Kf->K[1]);
		printf("\n\n");
	}
	
	system("pause");
		
	free(pMem);
}
*/
