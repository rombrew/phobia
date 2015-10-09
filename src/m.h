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

#ifndef _H_M_
#define _H_M_

#define MPIF			3.14159265f

inline float
fabsf(float x) { return __builtin_fabsf(x); }

#if __ARM_FP >= 4

inline float
sqrtf(float x)
{
	float		y;

	asm volatile ("vsqrt.f32 %0, %1 \r\n"
			: "=w" (y) : "w" (x));

	return y;
}

#else

inline float
sqrtf(float x) { return __builtin_sqrtf(x); }

#endif

void rotatef(float y[2], float angle, const float x[2]);
float arctanf(float y, float x);

#endif /* _H_M_ */

