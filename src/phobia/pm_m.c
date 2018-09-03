/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#include "pm_m.h"

static const float	lt_atanf[] = {

	-8.9550074E-3f,
	 4.5295657E-2f,
	-1.0939535E-1f,
	 1.9063993E-1f,
	-3.3214334E-1f,
	 9.9995555E-1f,
};

static const float	lt_sincosf[] = {

	-1.3772950E-4f,
	-2.0450985E-4f,
	 8.6392885E-3f,
	-2.4328724E-4f,
	-1.6656229E-1f,
	-2.2378746E-5f,
	 1.0000019E+0f,
	-3.5525078E-8f,
};

void pm_rotf(float y[2], float r, const float x[2])
{
	float           q, s, c, a, b;

	q = r * r;
	s = r - q * r * (.16666667f - 8.3333333E-3f * q);
	c = 1.f - q * (.5f - 4.1666667E-2f * q);

	a = c * x[0] - s * x[1];
	b = s * x[0] + c * x[1];

	q = (3.f - a * a - b * b) * .5f;
	y[0] = a * q;
	y[1] = b * q;
}

static float
pm_atanf(float x)
{
	float		u, x2;

	x2 = x * x;

	u = lt_atanf[0];
	u = lt_atanf[1] + u * x2;
	u = lt_atanf[2] + u * x2;
	u = lt_atanf[3] + u * x2;
	u = lt_atanf[4] + u * x2;
	u = lt_atanf[5] + u * x2;

	return u * x;
}

float pm_atan2f(float y, float x)
{
	float		u;

	if (pm_fabsf(x) > pm_fabsf(y)) {

		u = pm_atanf(y / x);
		u += (x < 0.f) ? (y < 0.f) ? - M_PI_F : M_PI_F : 0.f;
	}
	else {
		u = - pm_atanf(x / y);
		u += (y < 0.f) ? - M_PI_F / 2.f : M_PI_F / 2.f;
	}

	return u;
}

static float
pm_sincosf(float x)
{
	float		u;

	u = lt_sincosf[0];
        u = lt_sincosf[1] + u * x;
        u = lt_sincosf[2] + u * x;
        u = lt_sincosf[3] + u * x;
	u = lt_sincosf[4] + u * x;
	u = lt_sincosf[5] + u * x;
	u = lt_sincosf[6] + u * x;
	u = lt_sincosf[7] + u * x;

	return u;
}

float pm_sinf(float x)
{
        float           x_abs, u;

	x_abs = pm_fabsf(x);

	if (x_abs > (M_PI_F / 2.f))
		x_abs = M_PI_F - x_abs;

	u = pm_sincosf(x_abs);
	u = (x < 0.f) ? - u : u;

	return u;
}

float pm_cosf(float x)
{
        float           u;

	x = (M_PI_F / 2.f) - pm_fabsf(x);
	u = (x < 0.f) ? - pm_sincosf(- x) : pm_sincosf(x);

	return u;
}

float pm_DFT_const_R(const float DFT[8])
{
	float			D, X, Y, E, R = 0.;

	D = pm_sqrtf(DFT[2] * DFT[2] + DFT[3] * DFT[3]);

	if (D > 0.f) {

		X = DFT[2] / D;
		Y = DFT[3] / D;

		E = DFT[0] * X + DFT[1] * Y;
		R = E / D;
	}

	return R;
}

static void
pm_DFT_eigenvalues(float X, float Y, float M, float DQ[4])
{
	float		B, D, la1, la2;

	B = X + Y;
	D = B * B - 4.f * (X * Y - M * M);

	if (D > 0.f) {

		D = pm_sqrtf(D);
		la1 = (B - D) / 2.f;
		la2 = (B + D) / 2.f;

		B = Y - la1;
		D = pm_sqrtf(B * B + M * M);

		DQ[0] = la1;
		DQ[1] = la2;
		DQ[2] = B / D;
		DQ[3] = - M / D;
	}
}

void pm_DFT_const_L(const float DFT[8], float freq_hz, float LDQ[4])
{
	float			DX, DY, WR;
	float			LX, LY, LM;

	WR = 2.f * M_PI_F * freq_hz;

	DX = (DFT[0] * DFT[0] + DFT[1] * DFT[1]) * WR;
	DY = (DFT[4] * DFT[4] + DFT[5] * DFT[5]) * WR;

	LX = (DFT[2] * DFT[1] - DFT[3] * DFT[0]) / DX;
	LY = (DFT[6] * DFT[5] - DFT[7] * DFT[4]) / DY;
	LM = (DFT[6] * DFT[1] - DFT[7] * DFT[0]) / DX;
	LM += (DFT[2] * DFT[5] - DFT[3] * DFT[4]) / DY;
	LM /= 2.f;

	pm_DFT_eigenvalues(LX, LY, LM, LDQ);
}

