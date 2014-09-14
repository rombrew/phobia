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

#ifndef _H_MATH_
#define _H_MATH_

#ifdef __MATH_C
#define __SSAT(x, q)	ssat((x), (q))
#define __USAT(x, q)	usat((x), (q))
#define __SMULL(a, b)	smull((a), (b))
#define __CLZ(x)	clz((x))
#else
#include "hal/hal.h"
#endif /* __MATH_C */

inline int
clamp(int x, int h, int l)
{
	return (x > h) ? h : (x < l) ? l : x;
}

inline int
ssat(int x, int q)
{
	return clamp(x, (1UL << q) - 1, -(1UL << q));
}

inline int
usat(int x, int q)
{
	return clamp(x, (1UL << q) - 1, 0);
}

inline long long int
smull(int a, int b)
{
	return (long long int) a * (long long int) b;
}

inline int
clz(int x) { return __builtin_clz(x); }

short int xsdivi(int a, int d);

char *itoa(char *s, int x);
char *etoa(char *s, int x, int e);

#endif /* _H_MATH_ */

