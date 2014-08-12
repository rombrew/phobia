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

#ifdef _MATH_NATIVE
#include "hal/hal.h"
#else
#define __SSAT(x, m)	ssat((x), (m))
#define __USAT(x, m)	usat((x), (m))
#define __SMULL(x, m)	smull((x), (m))
#endif

inline int
ssat(int x, int m)
{
	m = (1 << m) - 1;

	return (x < 0) ? x | ~m : x & m;
}

inline int
usat(int x, int m)
{
	m = (1 << m) - 1;

	return (x < 0) ? 0 : x & m;
}

inline long long int
smull(int a, int b)
{
	return (long long int) a * (long long int) b;
}

#endif /* _H_MATH_ */

