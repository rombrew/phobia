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

#define lse_fabsf(x)		fabs(x)
#define lse_sqrtf(x)		sqrt(x)

static lse_float_t
lse_hypotf(lse_float_t a, lse_float_t b)
{
	lse_float_t		u;

	/* NOTE: We use naive implementation because it is the fastest and
	 * quite ulp-accurate. Beware of overflow and underflow.
	 * */
	u = lse_sqrtf(a * a + b * b);

	return u;
}

static lse_float_t
lse_l1normf(lse_float_t a, lse_float_t b)
{
	return lse_fabsf(a) + lse_fabsf(b);
}

static void
lse_qrupdate(lse_upper_t *rm, lse_row_t *xz)
{
	lse_float_t	*m = rm->m;
	lse_float_t	*v = xz->m;
	lse_float_t	sg, cg, t, u;

	int		n, lz, i, j;

	n = (rm->len < rm->keep) ? rm->len : rm->keep;
	lz = rm->len - xz->len;

	if (lz > 0) {

		/* We skip leading zeros.
		 * */
		m += lz * rm->len - lz * (lz - 1) / 2;
	}

	for (i = lz; i < n; ++i) {

		m += - i;

		/* Build the orthogonal transformation.
		 * */
		sg = - v[i];
		cg = m[i];

		u = lse_hypotf(sg, cg);

		if (u > (lse_float_t) 0.) {

			m[i] = u;

			sg /= u;
			cg /= u;

			/* Apply the transformation.
			 * */
			for (j = i + 1; j < rm->len; ++j) {

				t = cg * m[j] - sg * v[j];
				u = sg * m[j] + cg * v[j];

				v[j] = u;
				m[j] = t;
			}
		}

		m += rm->len;
	}

	m += - n;

	/* Copy the tail.
	 * */
	for (i = n; i < rm->len; ++i)
		m[i] = v[i];

	rm->keep += 1;
}

static void
lse_qrmerge(lse_upper_t *rm, lse_upper_t *em)
{
	lse_float_t	*m, *v = em->m;
	lse_float_t	sg, cg, t, u;

	int		k, n, i, j;

	n = (rm->len < em->keep) ? rm->len : em->keep;

	if (rm->keep != 0) {

		for (k = 0; k < n; ++k) {

			/* We extract one by one the row-vectors from \em
			 * matrix and embed them into the \rm matrix.
			 * */
			m = rm->m + k * rm->len - k * (k - 1) / 2;
			v += - k;

			for (i = k; i < rm->len; ++i) {

				m += - i;

				/* Build the orthogonal transformation.
				 * */
				sg = - v[i];
				cg = m[i];

				u = lse_hypotf(sg, cg);

				if (u > (lse_float_t) 0.) {

					m[i] = u;

					sg /= u;
					cg /= u;

					/* Apply the transformation.
					 * */
					for (j = i + 1; j < rm->len; ++j) {

						t = cg * m[j] - sg * v[j];
						u = sg * m[j] + cg * v[j];

						v[j] = u;
						m[j] = t;
					}
				}

				m += rm->len;
			}

			v += rm->len;
		}
	}
	else {
		/* We copy all row-vectors from \em matrix to the \rm matrix as
		 * it does not contain any data for now.
		 * */
		m = rm->m;
		k = rm->len * (rm->len + 1) / 2;

		if (n == rm->len) {

			for (i = 0; i < k; ++i)
				m[i] = v[i];
		}
		else {
			n = n * rm->len - n * (n - 1) / 2;

			for (i = 0; i < n; ++i)
				m[i] = v[i];

			for (i = n; i < k; ++i)
				m[i] = (lse_float_t) 0.;
		}
	}

	rm->keep += rm->len;
	em->keep = 0;
}

static void
lse_rowreduce(lse_upper_t *rm, lse_row_t *xz)
{
	lse_float_t	*m = rm->m;
	lse_float_t	*v = xz->m;
	lse_float_t	sg, cg, t, u;

	int		n, i, j;

	n = (rm->len < rm->keep) ? rm->len : rm->keep;

	for (i = 0; i < n; ++i) {

		m += - i;

		/* We build the pseudo-orthogonal l1-norm based transformation.
		 * */
		sg = - v[i];
		cg = m[i];

		u = lse_l1normf(sg, cg);

		if (u > (lse_float_t) 0.) {

			sg /= u;
			cg /= u;

			m[i] = cg * m[i] - sg * v[i];

			/* Apply the transformation.
			 * */
			for (j = i + 1; j < rm->len; ++j) {

				t = cg * m[j] - sg * v[j];
				u = sg * m[j] + cg * v[j];

				v[j] = u;
				m[j] = t;
			}
		}

		m += rm->len;
	}

	m += - n;

	/* Copy the tail.
	 * */
	for (i = n; i < rm->len; ++i)
		m[i] = v[i];

	rm->keep += 1;
}

int lse_getsize(int n_cascades, int n_full)
{
	int		n_lse_len, n_vm_len;

	n_lse_len = sizeof(lse_t) - sizeof(((lse_t *) 0)->vm);

	n_vm_len = n_cascades * n_full * (n_full + 1) / 2
		+ n_full * n_full / 4 + n_full / 2 + 1;

	return n_lse_len + sizeof(lse_float_t) * n_vm_len;
}

void lse_construct(lse_t *ls, int n_cascades, int n_len_of_x, int n_len_of_z)
{
	lse_float_t	*vm = ls->vm;

	int		i, n_full;

	ls->n_cascades = n_cascades;
	ls->n_len_of_x = n_len_of_x;
	ls->n_len_of_z = n_len_of_z;

	n_full = n_len_of_x + n_len_of_z;

	ls->n_threshold = n_full * 2;
	ls->n_total = 0;

	for (i = 0; i < ls->n_cascades; ++i) {

		ls->rm[i].len = n_full;
		ls->rm[i].keep = 0;
		ls->rm[i].m = vm;

		vm += n_full * (n_full + 1) / 2;
	}

	ls->sol.len = ls->n_len_of_x * ls->n_len_of_z;
	ls->sol.m = vm;

	ls->std.len = ls->n_len_of_z;
	ls->std.m = vm + ls->sol.len;

	ls->l_max = (lse_float_t) 0.;
	ls->l_min = (lse_float_t) 0.;
}

void lse_insert(lse_t *ls, lse_float_t *v)
{
	lse_row_t	xz = { ls->rm[0].len, v };

	int		i, keep;

	lse_qrupdate(&ls->rm[0], &xz);

	for (i = 1; i < ls->n_cascades; ++i) {

		if (ls->rm[i - 1].keep >= ls->n_threshold) {

			/* If data rows is too many collected we merge it into
			 * higher cascade.
			 * */
			lse_qrmerge(&ls->rm[i], &ls->rm[i - 1]);

			if (i == ls->n_cascades - 1) {

				/* Update the threshold value based on amount
				 * of data in top cascade.
				 * */
				keep = ls->rm[ls->n_cascades - 1].keep;
				ls->n_threshold = (keep > ls->n_threshold)
					? keep : ls->n_threshold;
			}

			break;
		}
	}

	ls->n_total += 1;
}

void lse_reduce(lse_t *ls, lse_float_t *v)
{
	lse_row_t	xz = { ls->rm[0].len, v };

	lse_rowreduce(&ls->rm[0], &xz);

	ls->n_total += 1;
}

void lse_ridge(lse_t *ls, lse_float_t la)
{
	lse_row_t	xz = { 0, ls->sol.m };

	int		i, j;

	/* Add bias with the unit matrix multiplied by \la.
	 * */
	for (i = 0; i < ls->n_len_of_x; ++i) {

		xz.len = ls->rm[0].len - i;
		xz.m[i] = la;

		for (j = i + 1; j < ls->rm[0].len; ++j)
			xz.m[j] = (lse_float_t) 0.;

		lse_qrupdate(&ls->rm[0], &xz);
	}
}

void lse_forget(lse_t *ls, lse_float_t la)
{
	lse_upper_t	*rm;

	int		i, j, n_len;

	for (i = 0; i < ls->n_cascades; ++i) {

		rm = &ls->rm[i];

		if (rm->keep != 0) {

			n_len = rm->len * (rm->len + 1) / 2;

			/* We just scale the \R matrix with factor \la.
			 * */
			for (j = 0; j < n_len; ++j)
				rm->m[j] *= la;
		}
	}
}

static void
lse_collapse(lse_t *ls)
{
	int		i;

	for (i = 1; i < ls->n_cascades; ++i) {

		if (ls->rm[i - 1].keep != 0) {

			/* We merge all cascades into the top \R matrix.
			 * */
			lse_qrmerge(&ls->rm[i], &ls->rm[i - 1]);
		}
	}
}

void lse_solve(lse_t *ls)
{
	lse_upper_t	*rm;

	lse_float_t	*sol = ls->sol.m;
	lse_float_t	*mq, *m, u;

	int		k, i, j;

	lse_collapse(ls);

	rm = &ls->rm[ls->n_cascades - 1];
	mq = rm->m + (ls->n_len_of_x - 1) * (rm->len - 1)
		- (ls->n_len_of_x - 1) * (ls->n_len_of_x - 2) / 2;

	/* We calculate the backward substitution.
	 * */
	for (k = 0; k < ls->n_len_of_z; ++k) {

		m = mq;

		for (i = ls->n_len_of_x - 1; i >= 0; --i) {

			u = (lse_float_t) 0.;

			for (j = i + 1; j < ls->n_len_of_x; ++j)
				u += sol[j] * m[j];

			sol[i] = (m[ls->n_len_of_x + k] - u) / m[i];

			m += i - rm->len;
		}

		sol += ls->n_len_of_x;
	}
}

void lse_std(lse_t *ls)
{
	lse_upper_t	*rm;

	lse_float_t	*std = ls->std.m;
	lse_float_t	*mq, *m, ratio, u;

	int		i, j;

	lse_collapse(ls);

	rm = &ls->rm[ls->n_cascades - 1];
	mq = rm->m + ls->n_len_of_x * rm->len
		- ls->n_len_of_x * (ls->n_len_of_x - 1) / 2;

	ratio = lse_sqrtf((lse_float_t) (ls->n_total - 1));

	/* We calculate l2 norm over \Rz columns.
	 * */
	for (i = 0; i < ls->n_len_of_z; ++i) {

		m = mq;
		u = lse_fabsf(m[0]);

		for (j = 1; j < i + 1; ++j) {

			m += rm->len - (ls->n_len_of_x + j);
			u = lse_hypotf(u, m[0]);
		}

		std[i] = u / ratio;

		mq += 1;
	}
}

static void
lse_qrstep(lse_upper_t *rm, lse_upper_t *em, lse_row_t *qq)
{
	lse_float_t	*mq = em->m;
	lse_float_t	*qm = qq->m;
	lse_float_t	*m;

	int		i, j;

	rm->keep = 0;

	/* Here we transpose the \em matrix and bring it to the \rm
	 * upper-triangular form again.
	 * */
	for (i = 0; i < rm->len; ++i) {

		m = mq;

		for (j = 0; j < i + 1; ++j) {

			qm[j] = m[0];
			m += em->len - (j + 1);
		}

		for (j = i + 1; j < rm->len; ++j)
			qm[j] = (lse_float_t) 0.;

		lse_qrupdate(rm, qq);

		mq += 1;
	}
}

void lse_cond(lse_t *ls, int n_approx)
{
	lse_upper_t	um, im, *rm;
	lse_row_t	qq;

	lse_float_t	*m, u;

	int		i, n;

	lse_collapse(ls);

	if (n_approx > 0 && ls->n_cascades > 2) {

		/* NOTE: We allocate two new \Rx matrices instead of \R
		 * cascades that are empty for now.
		 * */
		m = ls->rm[0].m;

		um.len = ls->n_len_of_x;
		um.m = m;

		m += ls->n_len_of_x * (ls->n_len_of_x + 1) / 2;

		qq.len = ls->n_len_of_x;
		qq.m = m;

		m += ls->n_len_of_x;

		im.len = ls->n_len_of_x;
		im.m = m;

		/* First QR step.
		 * */
		lse_qrstep(&um, &ls->rm[ls->n_cascades - 1], &qq);

		n = 1;

		while (n < n_approx) {

			/* Swap the matrices content.
			 * */
			m = um.m; um.m = im.m; im.m = m;

			/* We run the reduced form of QR algorithm. With each
			 * iteration off-diagonal elements tend to zero, so the
			 * diagonal approaches singular values.
			 * */
			lse_qrstep(&um, &im, &qq);

			n++;
		}

		rm = &um;
	}
	else {
		/* Take the \R matrix diagonal.
		 * */
		rm = &ls->rm[ls->n_cascades - 1];
	}

	m = rm->m;

	/* We are looking for the largest and smallest diagonal element.
	 * */
	for (i = 0; i < ls->n_len_of_x; ++i) {

		u = lse_fabsf(m[0]);

		if (i != 0) {

			ls->l_max = (ls->l_max < u) ? u : ls->l_max;
			ls->l_min = (ls->l_min > u) ? u : ls->l_min;
		}
		else {
			ls->l_max = u;
			ls->l_min = u;
		}

		m += rm->len - i;
	}
}

