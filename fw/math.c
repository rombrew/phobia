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

short int xsdivi(int a, int d)
{
	int		x, e;

	/* FIXME: Negative inputs are not handled correctly by CLZ.
	 * */

	x = __CLZ(a) - 1;
	x = (x < 0) ? 0 : x;
	a <<= x, e = x;

	x = 21 - __CLZ(d);
	x = (x < 0) ? 0 : x;
	d >>= x, e += x;

	a /= d;

	x = 22 - __CLZ(a);
	x = (x < 0) ? 0 : x;
	a >>= x, e -= x;

	e = __USAT(e, 5);
	a = (a << 5) + e;

	return a;
}

char *itoa(char *s, int x)
{
	char		*p;

	x = (x < 0) ? *s++ = '-', -x : x;

	p = s + 14;
	*--p = 0;

	do {
		*--p = '0' + x % 10;
		x /= 10;
	}
	while (x);

	while ((*s++ = *p++));

	return s;
}

char *etoa(char *s, int x, int e)
{
	int		d = 0;

	while (e > 0) {
	}
}

