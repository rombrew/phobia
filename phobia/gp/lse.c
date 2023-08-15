/*
   Graph Plotter is a tool to analyse numerical data.
   Copyright (C) 2023 Roman Belov <romblv@gmail.com>

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

#if LSE_FAST_TRANSFORM != 0
/* Define the maixmal allowed scale of the fast transformation. The input data
 * range is reduced by this number. Large number allows us to do scaling rarely.
 * */
#define LSE_DMAX 		((lse_float_t) 1048576)
#endif /* LSE_FAST_TRANSFORM */

/* Define what external math functions to use in LSE.
 * */
#define lse_fabsf(x)		fabs(x)
#define lse_sqrtf(x)		sqrt(x)

static void
#if LSE_FAST_TRANSFORM != 0
lse_qrupdate(lse_t *ls, lse_upper_t *rm, lse_float_t *xz, lse_float_t d0, int nz)
#else /* LSE_FAST_TRANSFORM */
lse_qrupdate(lse_t *ls, lse_upper_t *rm, lse_float_t *xz, int nz)
#endif
{
	lse_float_t	*m = rm->m;
#if LSE_FAST_TRANSFORM != 0
	lse_float_t	*d = rm->d;
#endif /* LSE_FAST_TRANSFORM */

	lse_float_t	x0, xi, alpa, beta;
	int		n, i, j;

#if LSE_FAST_TRANSFORM != 0
	lse_float_t	di;
	int		type;
#endif /* LSE_FAST_TRANSFORM */

	n = (rm->len < rm->keep) ? rm->len : rm->keep;

	/* Do we have leading zeros?
	 * */
	if (nz > 0) {

		m += nz * rm->len - nz * (nz - 1) / 2;
	}

	for (i = nz; i < n; ++i) {

		m += - i;

		x0 = - xz[i];
		xi = m[i];

		if (x0 != (lse_float_t) 0) {

#if LSE_FAST_TRANSFORM != 0
			di = d[i];

			/* We build the fast orthogonal transformation.
			 * */
			if (lse_fabsf(x0) < lse_fabsf(xi)) {

				alpa = x0 / xi;
				beta = alpa * di;

				if (alpa * beta < d0) {

					beta = - beta / d0;
					type = 1;
				}
				else {
					alpa = xi / x0;
					beta = - alpa * d0 / di;
					type = 0;
				}
			}
			else {
				alpa = xi / x0;
				beta = alpa * d0;

				if (alpa * beta < di) {

					beta = - beta / di;
					type = 0;
				}
				else {
					alpa = x0 / xi;
					beta = - alpa * di / d0;
					type = 1;
				}
			}

			/* Apply the transformation.
			 * */
			if (type != 0) {

				m[i] = m[i] + beta * xz[i];

				for (j = i + 1; j < rm->len; ++j) {

					xi = m[j] + beta * xz[j];
					x0 = alpa * m[j] + xz[j];

					xz[j] = x0;
					 m[j] = xi;
				}

				x0 = (lse_float_t) 1 - alpa * beta;

				d[i] = di * x0;
				  d0 = d0 * x0;
			}
			else {
				m[i] = beta * m[i] + xz[i];

				for (j = i + 1; j < rm->len; ++j) {

					xi = beta * m[j] + xz[j];
					x0 = m[j] + alpa * xz[j];

					xz[j] = x0;
					 m[j] = xi;
				}

				x0 = (lse_float_t) 1 - alpa * beta;

				d[i] = d0 * x0;
				  d0 = di * x0;
			}

			/* Keep diagonal is in allowed range.
			 * */
			if (d[i] > LSE_DMAX * LSE_DMAX) {

				alpa = (lse_float_t) 1 / LSE_DMAX;

				for (j = i; j < rm->len; ++j) {

					x0 = m[j];
					m[j] = x0 * alpa;
				}

				d[i] *= (alpa * alpa);
			}

			if (d0 > LSE_DMAX * LSE_DMAX) {

				alpa = (lse_float_t) 1 / LSE_DMAX;

				for (j = i + 1; j < rm->len; ++j) {

					x0 = xz[j];
					xz[j] = x0 * alpa;
				}

				d0 *= (alpa * alpa);
			}
#else /* LSE_FAST_TRANSFORM */

			/* WARNING: We use NAIVE hypot implementation as it is
			 * the fastest one and quite ulp-accurate.
			 */
			alpa = lse_sqrtf(x0 * x0 + xi * xi);
			beta = (lse_float_t) 1 / alpa;

			m[i] = alpa;

			/* We build the orthogonal transformation.
			 * */
			alpa = x0 * beta;
			beta = xi * beta;

			for (j = i + 1; j < rm->len; ++j) {

				xi = beta * m[j] - alpa * xz[j];
				x0 = alpa * m[j] + beta * xz[j];

				xz[j] = x0;
				 m[j] = xi;
			}
#endif /* LSE_FAST_TRANSFORM */
		}

		m += rm->len;
	}

	if (n < rm->len) {

		m += - n;

		if (rm->lazy != 0) {

			/* We merge the retained row-vector into the upper
			 * cascade matrix before copying the new content.
			 * */
#if LSE_FAST_TRANSFORM != 0
			lse_qrupdate(ls, rm + 1, m, d[n], n);
#else /* LSE_FAST_TRANSFORM */
			lse_qrupdate(ls, rm + 1, m, n);
#endif
		}

		/* Copy the tail content.
		 * */
		for (i = n; i < rm->len; ++i)
			m[i] = xz[i];

#if LSE_FAST_TRANSFORM != 0
		d[n] = d0;
#endif /* LSE_FAST_TRANSFORM */
	}

	rm->keep += 1;

	if (rm->keep >= ls->n_threshold) {

		if (rm < ls->rm + ls->n_cascades - 1) {

			/* Mark the cascade matrix content as lazily merged.
			 * */
			rm->keep = 0;
			rm->lazy = 1;
		}
		else {
			/* Update the threshold value based on amount of data
			 * rows in top cascade.
			 * */
			ls->n_threshold = (rm->keep > ls->n_threshold)
				? rm->keep : ls->n_threshold;
		}
	}
}

static void
lse_qrmerge(lse_t *ls, lse_upper_t *rm)
{
	lse_float_t	*m = rm->m;
#if LSE_FAST_TRANSFORM != 0
	lse_float_t	*d = rm->d;
#endif /* LSE_FAST_TRANSFORM */

	int		n0, i;

	n0 = (rm->lazy != 0) ? rm->len
		: (rm->len < rm->keep) ? rm->len : rm->keep;

	for (i = 0; i < n0; ++i) {

		m += - i;

		/* We extract one by one the row-vectors from cascade
		 * matrix and merge them into the upper cascade matrix.
		 * */
#if LSE_FAST_TRANSFORM != 0
		lse_qrupdate(ls, rm + 1, m, d[i], i);
#else /* LSE_FAST_TRANSFORM */
		lse_qrupdate(ls, rm + 1, m, i);
#endif

		m += rm->len;
	}

	rm->keep = 0;
	rm->lazy = 0;
}

int lse_getsize(int n_cascades, int n_full)
{
	int		n_lse_len, n_vma_len;

	n_lse_len = sizeof(lse_t) - sizeof(((lse_t *) 0)->vm);

	n_vma_len = n_cascades * n_full * (n_full + 1) / 2

#if LSE_FAST_TRANSFORM != 0
		  + n_cascades * n_full
#endif /* LSE_FAST_TRANSFORM */

		  + n_full * n_full / 4 + n_full / 2 + 1;

	return n_lse_len + sizeof(lse_float_t) * n_vma_len;
}

void lse_construct(lse_t *ls, int n_cascades, int n_len_of_x, int n_len_of_z)
{
	lse_float_t	*vm = ls->vm;

	int		i, n_full;

	ls->n_cascades = n_cascades;
	ls->n_len_of_x = n_len_of_x;
	ls->n_len_of_z = n_len_of_z;

	n_full = n_len_of_x + n_len_of_z;

	ls->n_threshold = n_full * 10;
	ls->n_total = 0;

	for (i = 0; i < ls->n_cascades; ++i) {

		ls->rm[i].len = n_full;
		ls->rm[i].keep = 0;
		ls->rm[i].lazy = 0;
		ls->rm[i].m = vm;

		vm += n_full * (n_full + 1) / 2;

#if LSE_FAST_TRANSFORM != 0
		ls->rm[i].d = vm;
		vm += n_full;
#endif /* LSE_FAST_TRANSFORM */
	}

	ls->sol.len = ls->n_len_of_x * ls->n_len_of_z;
	ls->sol.m = vm;

	ls->std.len = ls->n_len_of_z;
	ls->std.m = vm + ls->sol.len;

	ls->esv.max = (lse_float_t) 0;
	ls->esv.min = (lse_float_t) 0;
}

void lse_insert(lse_t *ls, lse_float_t *xz)
{
#if LSE_FAST_TRANSFORM != 0
	lse_qrupdate(ls, ls->rm, xz, (lse_float_t) 1, 0);
#else /* LSE_FAST_TRANSFORM */
	lse_qrupdate(ls, ls->rm, xz, 0);
#endif

	ls->n_total += 1;
}

void lse_ridge(lse_t *ls, lse_float_t la)
{
	lse_float_t	*xz = ls->sol.m;

	int		i, j;

	/* Add bias using the unit matrix multiplied by \la.
	 * */
	for (i = 0; i < ls->n_len_of_x; ++i) {

		xz[i] = la;

		for (j = i + 1; j < ls->rm[0].len; ++j)
			xz[j] = (lse_float_t) 0;

#if LSE_FAST_TRANSFORM != 0
		lse_qrupdate(ls, ls->rm, xz, (lse_float_t) 1, i);
#else /* LSE_FAST_TRANSFORM */
		lse_qrupdate(ls, ls->rm, xz, i);
#endif
	}
}

void lse_forget(lse_t *ls, lse_float_t la)
{
	lse_upper_t	*rm;

	int		n0, i, j, len;

	for (i = 0; i < ls->n_cascades; ++i) {

		rm = &ls->rm[i];

		n0 = (rm->lazy != 0) ? rm->len
			: (rm->len < rm->keep) ? rm->len : rm->keep;

		if (n0 != 0) {

			len = n0 * rm->len - n0 * (n0 - 1) / 2;

			/* We just scale \R matrices with factor \la.
			 * */
			for (j = 0; j < len; ++j)
				rm->m[j] *= la;
		}
	}
}

static void
lse_merge(lse_t *ls)
{
	lse_upper_t	*rm = ls->rm + ls->n_cascades - 1;

	int		i, len, nul;

	for (i = 0; i < ls->n_cascades - 1; ++i) {

		/* We merge all cascades into the top \R matrix.
		 * */
		lse_qrmerge(ls, ls->rm + i);
	}

	if (rm->keep < rm->len) {

		/* Zero out uninitialized tail content.
		 * */
		len = rm->keep * rm->len - rm->keep * (rm->keep - 1) / 2;
		nul = rm->len * (rm->len + 1) / 2;

		for (i = len; i < nul; ++i)
			rm->m[i] = (lse_float_t) 0;

#if LSE_FAST_TRANSFORM != 0
		for (i = rm->keep; i < rm->len; ++i)
			rm->d[i] = (lse_float_t) 1;
#endif /* LSE_FAST_TRANSFORM */

		rm->keep = rm->len;
	}
}

void lse_solve(lse_t *ls)
{
	lse_upper_t	*rm = ls->rm + ls->n_cascades - 1;

	lse_float_t	*sol = ls->sol.m;
	lse_float_t	*mq, *m, u;

	int		n, i, j;

	lse_merge(ls);

	mq = rm->m + (ls->n_len_of_x - 1) * (rm->len - 1)
		- (ls->n_len_of_x - 1) * (ls->n_len_of_x - 2) / 2;

	/* We calculate solution \b with backward substitution.
	 * */
	for (n = 0; n < ls->n_len_of_z; ++n) {

		m = mq;

		for (i = ls->n_len_of_x - 1; i >= 0; --i) {

			u = (lse_float_t) 0;

			for (j = i + 1; j < ls->n_len_of_x; ++j)
				u += sol[j] * m[j];

			sol[i] = (m[ls->n_len_of_x + n] - u) / m[i];

			m += i - rm->len;
		}

		sol += ls->n_len_of_x;
	}
}

void lse_std(lse_t *ls)
{
	lse_upper_t	*rm = ls->rm + ls->n_cascades - 1;

	lse_float_t	*std = ls->std.m;
	lse_float_t	*mq, *m, u, ratio;

#if LSE_FAST_TRANSFORM != 0
	lse_float_t	*d = rm->d + ls->n_len_of_x;
#endif /* LSE_FAST_TRANSFORM */

	int		i, j;

	lse_merge(ls);

	mq = rm->m + ls->n_len_of_x * rm->len
		- ls->n_len_of_x * (ls->n_len_of_x - 1) / 2;

	ratio = (lse_float_t) 1 / (lse_float_t) (ls->n_total - 1);

	/* We calculate l2 norm over \Rz columns.
	 * */
	for (i = 0; i < ls->n_len_of_z; ++i) {

		m = mq;

#if LSE_FAST_TRANSFORM != 0
		u = m[0] * m[0] / d[0];
#else /* LSE_FAST_TRANSFORM */
		u = m[0] * m[0];
#endif

		for (j = 1; j < i + 1; ++j) {

			m += rm->len - (ls->n_len_of_x + j);

#if LSE_FAST_TRANSFORM != 0
			u += m[0] * m[0] / d[j];
#else /* LSE_FAST_TRANSFORM */
			u += m[0] * m[0];
#endif
		}

		std[i] = lse_sqrtf(u * ratio);

		mq += 1;
	}
}

static void
lse_qrstep(lse_t *ls, lse_upper_t *um, lse_upper_t *im, lse_float_t *vm)
{
	lse_float_t	*mq = im->m;
#if LSE_FAST_TRANSFORM != 0
	lse_float_t	*ud = um->d;
#endif /* LSE_FAST_TRANSFORM */
	lse_float_t	*m;

	int		i, j;

	um->keep = 0;
	um->lazy = 0;

	/* Here we transpose the input matrix \im and bring it to the
	 * upper-triangular form again and store into \um.
	 * */
	for (i = 0; i < um->len; ++i) {

		m = mq;

		for (j = 0; j < i + 1; ++j) {

			vm[j] = m[0];
			m += im->len - (j + 1);
		}

		for (j = i + 1; j < um->len; ++j)
			vm[j] = (lse_float_t) 0;

#if LSE_FAST_TRANSFORM != 0
		lse_qrupdate(ls, um, vm, ud[i], 0);
#else /* LSE_FAST_TRANSFORM */
		lse_qrupdate(ls, um, vm, 0);
#endif

		mq += 1;
	}
}

void lse_esv(lse_t *ls, int n_approx)
{
	lse_upper_t	um, im, *rm = ls->rm + ls->n_cascades - 1;
	lse_float_t	*m, u;

	int		len, i;

	lse_merge(ls);

	len = ls->n_len_of_x * (ls->n_len_of_x + 1) + ls->n_len_of_x * 3;

	if (ls->rm[0].m + len <= rm->m) {

		/* We allocate temporal \Rx matrices instead of \R
		 * cascades that are empty after merge.
		 * */
		m = ls->rm[0].m;
	}
	else {
		/* WARNING: We allocate temporal \Rx matrices in tail
		 * of LSE memory instead of \b and so on.
		 * */
		m = ls->sol.m;
	}

	um.len = ls->n_len_of_x;
	um.m = m;

	m += ls->n_len_of_x * (ls->n_len_of_x + 1) / 2;

#if LSE_FAST_TRANSFORM != 0
	um.d = m;
	m += ls->n_len_of_x;
#endif /* LSE_FAST_TRANSFORM */

	im.len = ls->n_len_of_x;
	im.m = m;

	m += ls->n_len_of_x * (ls->n_len_of_x + 1) / 2;

#if LSE_FAST_TRANSFORM != 0
	im.d = m;
	m += ls->n_len_of_x;
#endif /* LSE_FAST_TRANSFORM */

#if LSE_FAST_TRANSFORM != 0
	for (i = 0; i < ls->n_len_of_x; ++i) {

		um.d[i] = (lse_float_t) 1;
		im.d[i] = rm->d[i];
	}
#endif /* LSE_FAST_TRANSFORM */

	/* First step of QR algorithm.
	 * */
	lse_qrstep(ls, &um, rm, m);

	for (i = 1; i < n_approx; ++i) {

		/* Swap the matrices content.
		 * */
		{ lse_upper_t qm = um; um = im; im = qm; }

		/* We run the reduced form of QR algorithm. With each
		 * iteration off-diagonal elements tend to zero so the
		 * diagonal approaches singular values.
		 * */
		lse_qrstep(ls, &um, &im, m);
	}

	m = um.m;

	/* We are looking for the largest and smallest diagonal elements of \Rx.
	 * */
	for (i = 0; i < ls->n_len_of_x; ++i) {

#if LSE_FAST_TRANSFORM != 0
		u = lse_fabsf(m[0] / lse_sqrtf(um.d[i] * im.d[i]));
#else /* LSE_FAST_TRANSFORM */
		u = lse_fabsf(m[0]);
#endif

		if (i != 0) {

			ls->esv.max = (ls->esv.max < u) ? u : ls->esv.max;
			ls->esv.min = (ls->esv.min > u) ? u : ls->esv.min;
		}
		else {
			ls->esv.max = u;
			ls->esv.min = u;
		}

		m += um.len - i;
	}
}

