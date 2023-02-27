#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include "lfg.h"

typedef struct {

	double		seed[55];
	int		ra, rb;
}
lfg_t;

static lfg_t		lfg;

static unsigned int
lfg_lcgu(unsigned int rseed)
{
	/* Linear Congruential generator.
	 * */

	return rseed * 17317U + 1U;
}

void lfg_start(int rseed)
{
	unsigned int	rlcg;
	int		j;

	rlcg = lfg_lcgu(rseed);
	rlcg = lfg_lcgu(rlcg);

	for (j = 0; j < 55; ++j) {

		rlcg = lfg_lcgu(rlcg);
		lfg.seed[j] = (double) rlcg / (double) UINT_MAX;
	}

	lfg.ra = 0;
	lfg.rb = 31;
}

double lfg_rand()
{
	double		x, a, b;

	/* Lagged Fibonacci generator.
	 * */

	a = lfg.seed[lfg.ra];
	b = lfg.seed[lfg.rb];

	x = (a < b) ? a - b + 1. : a - b;

	lfg.seed[lfg.ra] = x;

	lfg.ra = (lfg.ra < 54) ? lfg.ra + 1 : 0;
	lfg.rb = (lfg.rb < 54) ? lfg.rb + 1 : 0;

	return x;
}

double lfg_gauss()
{
	double		s, x;

	/* Box-Muller transform.
	 * */

	do {
		s = 2. * lfg_rand() - 1.;
		x = 2. * lfg_rand() - 1.;

		s = s * s + x * x;

		if (s > 0. && s < 1.)
			break;
	}
	while (1);

	x *= sqrt(- 2. * log(s) / s);

	return x;
}

