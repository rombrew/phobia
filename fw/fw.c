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

#include "hal/hal.h"
#include "lib.h"
#include "shell.h"

#include "hal/cmsis/stm32f4xx.h"

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
mPrt(const char *title, const float *X, int M, int N)
{
	int		i, j;

	printf("%s:\n", title);

	for (i = 0; i < M; ++i) {
		for (j = 0; j < N; ++j) {

			printf("%i ", (int) (X[i * N + j] * 1e+6));
		}
		printf("\r\n");
	}

	printf("\r\n");
}

static float		P[25];

void sKF()
{
	float		A[25], AT[25], AP[25];
	int		i, j;
	int		T0, T1;
	float		dT;

	for (i = 0; i < 5; ++i)
		for (j = 0; j < 5; ++j) {

			P[i * 5 + j] = 1.f / (float) SysTick->VAL;
		}

	for (i = 0; i < 5; ++i)
		for (j = 0; j < 5; ++j) {

			A[i * 5 + j] = 1.f / (float) SysTick->VAL;
		}

	T0 = SysTick->VAL;

	mTrans(AT, A, 5, 5);
	mMul(AP, A, P, 5, 5, 5);
	mMul(P, AP, AT, 5, 5, 5);

	T1 = SysTick->VAL;

	printf("TICK (%i %i) %i \r\n", T0, T1, T0 - T1);
}

void halTick()
{
	if (uartReceive() != -1) {

		sKF();
	}
}

void halMain()
{
	halLED(LED_BLUE);
	uartEnable(57600UL);

	do {
		halTick();
		//shellTask();
		//halWFI();
	}
	while (1);
}

