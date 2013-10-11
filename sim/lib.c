/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2013 Roman Belov <romblv@gmail.com>

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

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include "lib.h"

static double rseed[55];

static double
sqd(double x)
{
        return x * x;
}

void lib_enable()
{
	unsigned int	r;
	int		j;

	r = (unsigned int) time(NULL);
	r = r * 17317 + 1;

	for (j = 0; j < 55; ++j) {

		r = r * 17317 + 1;

		rseed[j] = (double) r / (double) UINT_MAX;
	}
}

double rand1()
{
	static int	ra = 0, rb = 31;
	double		x, a, b;

	a = rseed[ra];
	b = rseed[rb];

	x = (a < b) ? a - b + 1.0 : a - b;

	rseed[ra] = x;

	ra = (ra < 54) ? ra + 1 : 0;
	rb = (rb < 54) ? rb + 1 : 0;

	return x;
}

double rand2()
{
	return 2.0 * rand1() - 1.0;
}

double gauss()
{
	double		s, x;

	do {
		x = rand2();
		s = sqd(x) + sqd(rand2());
	}
	while (s >= 1.0);

	x *= sqrt(-2.0 * log(s) / s);

	return x;
}

