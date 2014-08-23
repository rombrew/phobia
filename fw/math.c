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

#include "math.h"

short int x115divi(int a, int d)
{
	int		x, e;

	x = clx(a) - 1;
	x = (x < 0) ? 0 : x;
	a <<= x, e = x;

	x = 21 - clx(d);
	x = (x < 0) ? 0 : x;
	d >>= x, e += x;

	a /= d;

	x = 22 - clx(a);
	x = (x < 0) ? 0 : x;
	a >>= x, e -= x;

	e = __USAT(e, 5);
	a = (a << 5) + e;

	return a;
}

