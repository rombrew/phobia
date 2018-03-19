/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include "pm_math.h"

static const float	lt_atan2f[] = {

	2.9009235E-1f,
	- 6.5717929E-2f,
	6.6529250E-2f,
	9.9397328E-1f,
	1.5707963E+0f
};

static const float	lt_sincosf[] = {

	-1.37729499E-4f,
	-2.04509846E-4f,
	8.63928854E-3f,
	-2.43287243E-4f,
	-1.66562291E-1f,
	-2.23787462E-5f,
	1.00000193E+0f,
	-3.55250780E-8f,
};

void pm_rotf(float y[2], float angle, const float x[2])
{
	float		q, s, c, a, b;

	q = angle * angle;
	s = angle - q * angle * (.16666667f - 8.3333333E-3f * q);
	c = 1.f - q * (.5f - 4.1666667E-2f * q);

	a = c * x[0] - s * x[1];
	b = s * x[0] + c * x[1];

	q = (3.f - a * a - b * b) * .5f;
	y[0] = a * q;
	y[1] = b * q;
}

float pm_atan2f(float y, float x)
{
	float		y_abs, u, f = 0.f;

	/* NOTE: Only normalized input is acceptable ||x|| = 1.
	 * */

	y_abs = fabsf(y);

	if (x < 0.f) {

		f = x;
		x = y_abs;
		y_abs = - f;
		f = lt_atan2f[4];
	}

	if (y_abs < x) {

		u = lt_atan2f[1] + lt_atan2f[0] * y_abs;
		u = lt_atan2f[2] + u * y_abs;
		u = lt_atan2f[3] + u * y_abs;
		f += u * y_abs;
	}
	else {
		u = lt_atan2f[1] + lt_atan2f[0] * x;
		u = lt_atan2f[2] + u * x;
		u = lt_atan2f[3] + u * x;
		f += lt_atan2f[4] - u * x;
	}

	return (y < 0.f) ? - f : f;
}

float pm_sinf(float angle)
{
        float           u;
        int             m = 0;

	/* NOTE: Only +PI/-PI range is acceptable.
	 * */

        if (angle < 0.f) {

                m = 1;
                angle = - angle;
        }

        if (angle > (M_PI_F / 2.f))
                angle = M_PI_F - angle;

        u = lt_sincosf[1] + lt_sincosf[0] * angle;
        u = lt_sincosf[2] + u * angle;
        u = lt_sincosf[3] + u * angle;
	u = lt_sincosf[4] + u * angle;
	u = lt_sincosf[5] + u * angle;
	u = lt_sincosf[6] + u * angle;
	u = lt_sincosf[7] + u * angle;

	return m ? - u : u;
}

float pm_cosf(float angle)
{
        float           u;
        int             m = 0;

        if (angle < 0.f)
                angle = - angle;

        if (angle > (M_PI_F / 2.f)) {

                m = 1;
                angle = M_PI_F - angle;
        }

	angle = (M_PI_F / 2.f) - angle;

	u = lt_sincosf[1] + lt_sincosf[0] * angle;
	u = lt_sincosf[2] + u * angle;
	u = lt_sincosf[3] + u * angle;
	u = lt_sincosf[4] + u * angle;
	u = lt_sincosf[5] + u * angle;
	u = lt_sincosf[6] + u * angle;
	u = lt_sincosf[7] + u * angle;

	return m ? - u : u;
}

float pm_DFT_const_R(const float DFT[8])
{
	float			D, X, Y, E, R = 0.;

	D = sqrtf(DFT[2] * DFT[2] + DFT[3] * DFT[3]);

	if (D > 0.f) {

		X = DFT[2] / D;
		Y = DFT[3] / D;

		E = DFT[0] * X + DFT[1] * Y;
		R = E / D;
	}

	return R;
}

static void
pm_DFT_eigenvalues(float X, float Y, float M, float DQA[3])
{
	float		B, D, la1, la2;

	B = X + Y;
	D = B * B - 4.f * (X * Y - M * M);

	if (D > 0.f) {

		D = sqrtf(D);
		la1 = (B - D) * .5f;
		la2 = (B + D) * .5f;

		B = Y - la1;
		D = sqrtf(B * B + M * M);
		B /= D;
		D = - M / D;
		B = pm_atan2f(D, B);

		DQA[0] = la1;
		DQA[1] = la2;
		DQA[2] = B;
	}
}

void pm_DFT_const_L(const float DFT[8], float freq, float LDQ[3])
{
	float			DX, DY, W;
	float			LX, LY, LM;

	DX = DFT[0] * DFT[0] + DFT[1] * DFT[1];
	DY = DFT[4] * DFT[4] + DFT[5] * DFT[5];
	W = 2.f * M_PI_F * freq;

	LX = (DFT[2] * DFT[1] - DFT[3] * DFT[0]) / (DX * W);
	LY = (DFT[6] * DFT[5] - DFT[7] * DFT[4]) / (DY * W);
	LM = (DFT[6] * DFT[1] - DFT[7] * DFT[0]) / (DX * W);
	LM += (DFT[2] * DFT[5] - DFT[3] * DFT[4]) / (DY * W);
	LM *= .5f;

	pm_DFT_eigenvalues(LX, LY, LM, LDQ);
}

