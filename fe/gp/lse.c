/*
   Graph Plotter for numerical data analysis.
   Copyright (C) 2022 Roman Belov <romblv@gmail.com>

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

#include <math.h>

#include "lse.h"

#define lse_fabsf(x)		fabs(x)
#define lse_sqrtf(x)		sqrt(x)

static lse_float_t
lse_hypotf(lse_float_t a, lse_float_t b)
{
	lse_float_t		u;

	/* FIXME: We use naive implementation because it is the fastest and
	 * quite ulp-accurate. Beware of overflow and underflow.
	 * */
	u = lse_sqrtf(a * a + b * b);

	return u;
}

static void
lse_cholupdate(const lse_t *lse, lse_triu_t *triu, lse_float_t *v)
{
	lse_float_t	*m = triu->m;
	lse_float_t	sg, cg, t, u;

	int		i, j, n;

	n = (lse->n_full < triu->n_keep) ? lse->n_full : triu->n_keep;

	for (i = 0; i < n; ++i) {

		m += - i;

		sg = - v[i];
		cg = m[i];

		u = lse_hypotf(sg, cg);

		if (u > (lse_float_t) 0.) {

			m[i] = u;

			sg /= u;
			cg /= u;

			for (j = i + 1; j < lse->n_full; ++j) {

				t = cg * m[j] - sg * v[j];
				u = sg * m[j] + cg * v[j];

				v[j] = u;
				m[j] = t;
			}
		}

		m += lse->n_full;
	}

	m += - n;

	for (i = n; i < lse->n_full; ++i)
		m[i] = v[i];

	triu->n_keep += 1;
}

static void
lse_cholmerge(const lse_t *lse, lse_triu_t *triu, lse_triu_t *triv)
{
	lse_float_t	*m, *v = triv->m;
	lse_float_t	sg, cg, t, u;

	int		i, j, k, n;

	n = (lse->n_full < triv->n_keep) ? lse->n_full : triv->n_keep;

	if (triu->n_keep != 0) {

		for (k = 0; k < n; ++k) {

			m = triu->m + k * lse->n_full - k * (k - 1) / 2;
			v += - k;

			for (i = k; i < lse->n_full; ++i) {

				m += - i;

				sg = - v[i];
				cg = m[i];

				u = lse_hypotf(sg, cg);

				if (u > (lse_float_t) 0.) {

					m[i] = u;

					sg /= u;
					cg /= u;

					for (j = i + 1; j < lse->n_full; ++j) {

						t = cg * m[j] - sg * v[j];
						u = sg * m[j] + cg * v[j];

						v[j] = u;
						m[j] = t;
					}
				}

				m += lse->n_full;
			}

			v += lse->n_full;
		}
	}
	else {
		m = triu->m;
		k = lse->n_full * (lse->n_full + 1) / 2;

		if (n == lse->n_full) {

			for (i = 0; i < k; ++i)
				m[i] = v[i];
		}
		else {
			n = n * lse->n_full - n * (n - 1) / 2;

			for (i = 0; i < n; ++i)
				m[i] = v[i];

			for (i = n; i < k; ++i)
				m[i] = (lse_float_t) 0.;
		}
	}

	triu->n_keep += lse->n_full;
	triv->n_keep = 0;
}

void lse_initiate(lse_t *lse, int n_cascades, int n_size_of_x, int n_size_of_z)
{
	lse_float_t	*vm = lse->vm;
	int		i;

	lse->n_cascades = n_cascades;

	lse->n_size_of_x = n_size_of_x;
	lse->n_size_of_z = n_size_of_z;
	lse->n_full = n_size_of_x + n_size_of_z;

	lse->n_threshold = lse->n_full * 2;
	lse->n_total = 0;

	for (i = 0; i < lse->n_cascades; ++i) {

		lse->triu[i].n_keep = 0;
		lse->triu[i].m = vm;

		vm += lse->n_full * (lse->n_full + 1) / 2;
	}
}

void lse_finalise(lse_t *lse)
{
	lse_float_t	*b, *e, *mq, *m;
	lse_float_t	ratio, u;

	int		i, j, k;

	if (lse->n_total > lse->n_size_of_x) {

		/* The normal case of large amount of data. We merge all
		 * cascades into the final upper-triangular matrix.
		 * */
		for (i = 1; i < lse->n_cascades; ++i)
			lse_cholmerge(lse, &lse->triu[i], &lse->triu[i - 1]);

		b = lse->triu[0].m;
		lse->b = b;

		mq = lse->triu[lse->n_cascades - 1].m;
	}
	else {
		/* The case of exact solution.
		 * */
		b = lse->triu[1].m;
		lse->b = b;

		mq = lse->triu[0].m;
	}

	mq += (lse->n_size_of_x - 1) * (lse->n_full - 1)
		- (lse->n_size_of_x - 1) * (lse->n_size_of_x - 2) / 2;

	/* Obtain the final LS solution \b with backward substitution.
	 * */
	for (k = 0; k < lse->n_size_of_z; ++k) {

		m = mq;

		for (i = lse->n_size_of_x - 1; i >= 0; --i) {

			u = (lse_float_t) 0.;

			for (j = i + 1; j < lse->n_size_of_x; ++j)
				u += b[j] * m[j];

			b[i] = (m[lse->n_size_of_x + k] - u) / m[i];

			m += i - lse->n_full;
		}

		b += lse->n_size_of_x;
	}

	if (lse->n_total > lse->n_size_of_x) {

		/* The normal case of large amount of data.
		 * */
		e = lse->triu[0].m + lse->n_size_of_x * lse->n_size_of_z;
		lse->e = e;

		mq = lse->triu[lse->n_cascades - 1].m;
		mq += lse->n_size_of_x * lse->n_full
			- lse->n_size_of_x * (lse->n_size_of_x - 1) / 2;

		ratio = lse_sqrtf((lse_float_t) (lse->n_total - 1));

		/* Obtain the standard deviation \e of the row-vector \z.
		 * */
		for (i = 0; i < lse->n_size_of_z; ++i) {

			m = mq;
			u = lse_fabsf(mq[0]);

			for (j = i - 1; j >= 0; --j) {

				m += (lse->n_size_of_x + j) - lse->n_full + 1;
				u = lse_hypotf(u, m[0]);
			}

			e[i] = u / ratio;

			mq += lse->n_full - (lse->n_size_of_x + i);
		}
	}
}

void lse_insert(lse_t *lse, lse_float_t *v)
{
	int		i, n;

	lse_cholupdate(lse, &lse->triu[0], v);

	for (i = 1; i < lse->n_cascades; ++i) {

		if (lse->triu[i - 1].n_keep >= lse->n_threshold) {

			/* If data rows is too many we merge it into higher
			 * cascade.
			 * */
			lse_cholmerge(lse, &lse->triu[i], &lse->triu[i - 1]);

			if (i == lse->n_cascades - 1) {

				/* Update the cascade threshold based on amount
				 * of data in top cascade.
				 * */
				n = lse->triu[lse->n_cascades - 1].n_keep;
				lse->n_threshold = (n > lse->n_threshold)
					? n : lse->n_threshold;
			}

			break;
		}
	}

	lse->n_total += 1;
}

