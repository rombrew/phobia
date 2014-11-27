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

float			P[25];

void sKF()
{
	float		A[25], B[25];
	int		j, T0, T1;
	float		dT;

	for (j = 0; j < 25; ++j)
		A[j] = 1.f / (float) SysTick->VAL;

	for (j = 0; j < 25; ++j)
		B[j] = 1.f / (float) SysTick->VAL;

	T0 = SysTick->VAL;
	__DSB();

	P[0] = A[0] * B[0] + A[1] * B[5] + A[2] * B[10] + A[3] * B[15] + A[4] * B[20];
	P[1] = A[0] * B[1] + A[1] * B[6] + A[2] * B[11] + A[3] * B[16] + A[4] * B[21];
	P[2] = A[0] * B[2] + A[1] * B[7] + A[2] * B[12] + A[3] * B[17] + A[4] * B[22];
	P[3] = A[0] * B[3] + A[1] * B[8] + A[2] * B[13] + A[3] * B[18] + A[4] * B[23];
	P[4] = A[0] * B[4] + A[1] * B[9] + A[2] * B[14] + A[3] * B[19] + A[4] * B[24];

	P[5] = A[5] * B[0] + A[6] * B[5] + A[7] * B[10] + A[8] * B[15] + A[9] * B[20];
	P[6] = A[5] * B[1] + A[6] * B[6] + A[7] * B[11] + A[8] * B[16] + A[9] * B[21];
	P[7] = A[5] * B[2] + A[6] * B[7] + A[7] * B[12] + A[8] * B[17] + A[9] * B[22];
	P[8] = A[5] * B[3] + A[6] * B[8] + A[7] * B[13] + A[8] * B[18] + A[9] * B[23];
	P[9] = A[5] * B[4] + A[6] * B[9] + A[7] * B[14] + A[8] * B[19] + A[9] * B[24];

	P[10] = A[10] * B[0] + A[11] * B[5] + A[12] * B[10] + A[13] * B[15] + A[14] * B[20];
	P[11] = A[10] * B[1] + A[11] * B[6] + A[12] * B[11] + A[13] * B[16] + A[14] * B[21];
	P[12] = A[10] * B[2] + A[11] * B[7] + A[12] * B[12] + A[13] * B[17] + A[14] * B[22];
	P[13] = A[10] * B[3] + A[11] * B[8] + A[12] * B[13] + A[13] * B[18] + A[14] * B[23];
	P[14] = A[10] * B[4] + A[11] * B[9] + A[12] * B[14] + A[13] * B[19] + A[14] * B[24];

	P[15] = A[15] * B[0] + A[16] * B[5] + A[17] * B[10] + A[18] * B[15] + A[19] * B[20];
	P[16] = A[15] * B[1] + A[16] * B[6] + A[17] * B[11] + A[18] * B[16] + A[19] * B[21];
	P[17] = A[15] * B[2] + A[16] * B[7] + A[17] * B[12] + A[18] * B[17] + A[19] * B[22];
	P[18] = A[15] * B[3] + A[16] * B[8] + A[17] * B[13] + A[18] * B[18] + A[19] * B[23];
	P[19] = A[15] * B[4] + A[16] * B[9] + A[17] * B[14] + A[18] * B[19] + A[19] * B[24];

	P[20] = A[20] * B[0] + A[21] * B[5] + A[22] * B[10] + A[23] * B[15] + A[24] * B[20];
	P[21] = A[20] * B[1] + A[21] * B[6] + A[22] * B[11] + A[23] * B[16] + A[24] * B[21];
	P[22] = A[20] * B[2] + A[21] * B[7] + A[22] * B[12] + A[23] * B[17] + A[24] * B[22];
	P[23] = A[20] * B[3] + A[21] * B[8] + A[22] * B[13] + A[23] * B[18] + A[24] * B[23];
	P[24] = A[20] * B[4] + A[21] * B[9] + A[22] * B[14] + A[23] * B[19] + A[24] * B[24];

	__DSB();
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

