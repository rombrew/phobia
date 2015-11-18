/*
   Phobia Motor Controller for RC and robotics.
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

#include "m.h"

void mrotf(float y[2], float angle, const float x[2])
{
	float		q, s, c, a, b;

	q = angle * angle;
	s = angle - q * angle * (.16666667f - 8.3333333e-3f * q);
	c = 1.f - q * (.5f - 4.1666667e-2f * q);

	a = c * x[0] - s * x[1];
	b = s * x[0] + c * x[1];

	q = (3.f - a * a - b * b) * .5f;
	y[0] = a * q;
	y[1] = b * q;
}

static const float	patan2[] = {

	2.9009235E-1f,
	- 6.5717929E-2f,
	6.6529250E-2f,
	9.9397328E-1f,
	1.5707963E+0f
};

float matan2f(float y, float x)
{
	float		y_abs, u, f = 0.f;

	y_abs = fabsf(y);

	if (x < 0.f) {

		f = x;
		x = y_abs;
		y_abs = - f;
		f = patan2[4];
	}

	if (y_abs < x) {

		u = patan2[1] + patan2[0] * y_abs;
		u = patan2[2] + u * y_abs;
		u = patan2[3] + u * y_abs;
		f += u * y_abs;
	}
	else {
		u = patan2[1] + patan2[0] * x;
		u = patan2[2] + u * x;
		u = patan2[3] + u * x;
		f += patan2[4] - u * x;
	}

	return (y < 0.f) ? - f : f;
}

static const float	psincos[8] = {

	-1.37729499E-4f,
	-2.04509846E-4f,
	8.63928854E-3f,
	-2.43287243E-4f,
	-1.66562291E-1f,
	-2.23787462E-5f,
	1.00000193E+0f,
	-3.55250780E-8f,
};

float msinf(float angle)
{
        float           u;
        int             m = 0;

        if (angle < 0.f) {

                m = 1;
                angle = - angle;
        }

        if (angle > (MPIF / 2.f))
                angle = MPIF - angle;

        u = psincos[1] + psincos[0] * angle;
        u = psincos[2] + u * angle;
        u = psincos[3] + u * angle;
	u = psincos[4] + u * angle;
	u = psincos[5] + u * angle;
	u = psincos[6] + u * angle;
	u = psincos[7] + u * angle;

	return m ? - u : u;
}

float mcosf(float angle)
{
        float           u;
        int             m = 0;

        if (angle < 0.f)
                angle = - angle;

        if (angle > (MPIF / 2.f)) {

                m = 1;
                angle = MPIF - angle;
        }

	angle = (MPIF / 2.f) - angle;

	u = psincos[1] + psincos[0] * angle;
	u = psincos[2] + u * angle;
	u = psincos[3] + u * angle;
	u = psincos[4] + u * angle;
	u = psincos[5] + u * angle;
	u = psincos[6] + u * angle;
	u = psincos[7] + u * angle;

	return m ? - u : u;
}

