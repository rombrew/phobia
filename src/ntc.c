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

#include "ntc.h"

#define M_LOG2F		0.69314718f

static float
fastlog2f(float x)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x }, m;

	float			y;

	y = (float) u.i * (1.f / (1UL << 23)) - 127.f;

	if (1) {

		m.i = (u.i & 0x7FFFFF) | (0x7F << 23);
		y += m.f * (1.f - .33333333f * m.f) - .66666667f;
	}

	return y;
}

float ntc_temperature(ntc_t *ntc, float u)
{
	float			r_ntc, log, temp;

	r_ntc = (u > 0.f) ? ntc->r_balance / (1.f / u - 1.f) : 0.f;

	log = fastlog2f(r_ntc / ntc->r_ntc_0) * M_LOG2F;
	temp = 1.f / (1.f / (ntc->ta_0 + 273.f) + log / ntc->betta) - 273.f;

	return temp;
}

