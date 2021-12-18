#include <stddef.h>

#include "min.h"
#include "phobia/libm.h"

static float
invoke_pfun(min_t *m, const float *xarg)
{
	float		fval;
	int		j, flag = 0;

	for (j = 0; j < m->n_size_of_x; ++j) {

		if (m_fabsf(xarg[j]) > m->x_maximal) {

			flag = 1;
			break;
		}
	}

	fval = (flag == 0) ? m->pfun(m->link, xarg) : 1000000000.f;

	return fval;
}

static void
linop(min_t *m, float *y, const float *x0, const float *x1, const float *x2, float xk)
{
	int		j;

	for (j = 0; j < m->n_size_of_x; ++j) {

		y[j] = x0[j] + (x1[j] - x2[j]) * xk;
	}
}

void minsearch(min_t *m)
{
	float		refval, conval, xtol;
	float		*xbest, *xworst;
	int		i, j;

	/* Construct the initial SIMPLEX.
	 * */
	for (i = 0; i < m->n_size_of_x + 1; ++i) {

		xbest = &m->xarg[i * m->n_size_of_x];

		for (j = 0; j < m->n_size_of_x; ++j)
			xbest[j] = 0.f;

		if (i != 0) {

			xbest[i - 1] += (i & 1) ? m->x0_range : - m->x0_range;
		}
	}

	/* Initial evaluation of the target FUNCTION.
	 * */
	for (i = 0; i < m->n_size_of_x + 1; ++i) {

		xbest = &m->xarg[i * m->n_size_of_x];

		m->fval[i] = invoke_pfun(m, xbest);
	}

	m->n_best = 0;
	m->n_worst = 0;
	m->n_step = 0;

	for (; m->n_step < m->n_final_step; ++m->n_step) {

		/* Obtain BEST and WORST simplex point indexes.
		 * */
		for (i = 0; i < m->n_size_of_x + 1; ++i) {

			m->n_best = (m->fval[i] < m->fval[m->n_best]) ? i : m->n_best;
			m->n_worst = (m->fval[i] > m->fval[m->n_worst]) ? i : m->n_worst;
		}

		if (m->n_best == m->n_worst)
			break;

		m->n_last = m->n_best;

		for (i = 0; i < m->n_size_of_x + 1; ++i) {

			if (i != m->n_worst) {

				m->n_last = (m->fval[i] > m->fval[m->n_last])
					? i : m->n_last;
			}
		}

		/* Compute CENTROID point.
		 * */
		for (j = 0; j < m->n_size_of_x; ++j)
			m->xnul[j] = 0.f;

		for (i = 0; i < m->n_size_of_x + 1; ++i) {

			xworst = &m->xarg[i * m->n_size_of_x];

			if (i != m->n_worst) {

				for (j = 0; j < m->n_size_of_x; ++j)
					m->xnul[j] += xworst[j];
			}
		}

		xworst = &m->xarg[m->n_worst * m->n_size_of_x];

		for (j = 0; j < m->n_size_of_x; ++j)
			m->xnul[j] /= (float) m->n_size_of_x;

		/* Compute REFLECTED point.
		 * */
		linop(m, m->xref, m->xnul, m->xnul, xworst, 1.f);

		refval = invoke_pfun(m, m->xref);

		if (refval < m->fval[m->n_last]) {

			if (refval > m->fval[m->n_best]) {

				for (j = 0; j < m->n_size_of_x; ++j)
					xworst[j] = m->xref[j];

				m->fval[m->n_worst] = refval;
			}
			else {
				/* Compute EXPANDED point.
				 * */
				linop(m, m->xcon, m->xnul, m->xref, m->xnul, 2.f);

				conval = invoke_pfun(m, m->xcon);

				if (conval < refval) {

					for (j = 0; j < m->n_size_of_x; ++j)
						xworst[j] = m->xcon[j];

					m->fval[m->n_worst] = conval;
				}
				else {
					for (j = 0; j < m->n_size_of_x; ++j)
						xworst[j] = m->xref[j];

					m->fval[m->n_worst] = refval;
				}
			}
		}
		else {
			if (refval < m->fval[m->n_worst]) {

				/* Compute CONTRACTED OUTSIDE point.
				 * */
				linop(m, m->xcon, m->xnul, m->xref, m->xnul, .5f);
			}
			else {
				/* Compute CONTRACTED INSIDE point.
				 * */
				linop(m, m->xcon, m->xnul, xworst, m->xnul, .5f);
			}

			conval = invoke_pfun(m, m->xcon);

			if (conval < m->fval[m->n_worst]) {

				for (j = 0; j < m->n_size_of_x; ++j)
					xworst[j] = m->xcon[j];

				m->fval[m->n_worst] = conval;
			}
			else {
				/* SHRINK the simplex.
				 * */
				xbest = &m->xarg[m->n_best * m->n_size_of_x];

				for (i = 0; i < m->n_size_of_x + 1; ++i) {

					if (i != m->n_best) {

						xworst = &m->xarg[i * m->n_size_of_x];

						linop(m, xworst, xbest, xworst, xbest, .5f);

						m->fval[m->n_worst] = invoke_pfun(m, xworst);
					}
				}
			}
		}

		/* Compute TOLERANCE of the simplex.
		 * */
		xbest = &m->xarg[m->n_best * m->n_size_of_x];
		m->x_tol = 0.f;

		for (i = 0; i < m->n_size_of_x + 1; ++i) {

			if (i != m->n_best) {

				for (j = 0; j < m->n_size_of_x; ++j) {

					xtol = m_fabsf(m->xarg[i * m->n_size_of_x + j] - xbest[j]);
					m->x_tol = (xtol > m->x_tol) ? xtol : m->x_tol;
				}
			}
		}

		/* Invoke STEP function.
		 * */
		if (m->pstep != NULL)
			m->pstep(m->link);

		/* Check if the tolerance is small enough.
		 * */
		if (m->x_tol < m->x_final_tol)
			break;
	}
}

void minsolution(min_t *m, float *xsol)
{
	float		*xbest;
	int		i, j;

	for (i = 0; i < m->n_size_of_x + 1; ++i)
		m->n_best = (m->fval[i] < m->fval[m->n_best]) ? i : m->n_best;

	xbest = &m->xarg[m->n_best * m->n_size_of_x];

	for (j = 0; j < m->n_size_of_x; ++j)
		xsol[j] = xbest[j];
}

