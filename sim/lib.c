#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include "lib.h"

#define FSEED_FILE		"/tmp/fseed"

typedef struct {

	double		seed[55];
	int		ra, rb;
}
lib_t;

static lib_t		lib;

static unsigned int
lib_lcgu(unsigned int rseed)
{
	/* Linear congruential generator.
	 * */

	return rseed * 17317UL + 1UL;
}

void lib_start()
{
	FILE		*fseed;
	unsigned int	r = 0;
	int		j;

	fseed = fopen(FSEED_FILE, "rb");

	if (fseed != NULL) {

		r = fread(&lib, sizeof(lib_t), 1, fseed);
		fclose(fseed);
	}

	if (r != 1) {

		/* Initial SEED.
		 * */

		r = (unsigned int) time(NULL);
		r = lib_lcgu(r);

		for (j = 0; j < 55; ++j) {

			r = lib_lcgu(r);
			lib.seed[j] = (double) r / (double) UINT_MAX;
		}

		lib.ra = 0;
		lib.rb = 31;
	}
}

void lib_stop()
{
	FILE		*fseed;

	fseed = fopen(FSEED_FILE, "wb");

	if (fseed != NULL) {

		fwrite(&lib, sizeof(lib_t), 1, fseed);
		fclose(fseed);
	}
}

double lib_rand()
{
	double		x, a, b;

	/* Lagged Fibonacci generator.
	 * */

	a = lib.seed[lib.ra];
	b = lib.seed[lib.rb];

	x = (a < b) ? a - b + 1. : a - b;

	lib.seed[lib.ra] = x;

	lib.ra = (lib.ra < 54) ? lib.ra + 1 : 0;
	lib.rb = (lib.rb < 54) ? lib.rb + 1 : 0;

	return x;
}

double lib_gauss()
{
	double		s, x;

	/* Box-Muller transform.
	 * */

	do {
		s = 2. * lib_rand() - 1.;
		x = 2. * lib_rand() - 1.;
		s = s * s + x * x;
	}
	while (s > 1. || s == 0.);

	x *= sqrt(- 2. * log(s) / s);

	return x;
}

