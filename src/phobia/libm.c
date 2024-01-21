#include <stdint.h>

#include "libm.h"

int m_isfinitef(float x)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	return ((0xFFU & (u.i >> 23)) != 0xFFU) ? 1 : 0;
}

static float
m_fast_rsqrtf(float x)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	u.i = 0x5F380000U - (u.i >> 1);

	return u.f;
}

void m_rotatef(float x[2], float r)
{
	float           q, s, c, y[2];

	q = r * r;

	s = r * (1.f - q * (1.6666667E-1f - 8.3333338E-3f * q));
	c = 1.f - q * (.5f - 4.1666668E-2f * q);

	y[0] = c * x[0] - s * x[1];
	y[1] = s * x[0] + c * x[1];

	q = y[0] * y[0] + y[1] * y[1];

	s = likely(q < 2.f) ? (3.f - q) * .5f : m_fast_rsqrtf(q);

	x[0] = y[0] * s;
	x[1] = y[1] * s;
}

void m_rsumf(float *sum, float *rem, float x)
{
	float		y, m;

	/* Kahan compensated summation.
	 * */

	y = x - *rem;
	m = *sum + y;

	*rem = (m - *sum) - y;
	*sum = m;
}

float m_wrapf(float x)
{
	int		revol;

	revol = (int) (x * (1.f / M_2_PI_F));
	x += - (float) revol * M_2_PI_F;

	if (unlikely(x < - M_PI_F)) {

		x += M_2_PI_F;
	}
	else if (unlikely(x > M_PI_F)) {

		x += - M_2_PI_F;
	}

	return x;
}

static float
m_atanf(float x)
{
	static const float	lt_atanf[] = {

		-8.9550074E-3f,
		 4.5295657E-2f,
		-1.0939535E-1f,
		 1.9063993E-1f,
		-3.3214334E-1f,
		 9.9995555E-1f,
	};

	float		u, x2;

	x2 = x * x;

	u = lt_atanf[0];
	u = lt_atanf[1] + u * x2;
	u = lt_atanf[2] + u * x2;
	u = lt_atanf[3] + u * x2;
	u = lt_atanf[4] + u * x2;
	u = lt_atanf[5] + u * x2;

	return u * x;
}

float m_atan2f(float y, float x)
{
	float		u;

	if (m_fabsf(x) > m_fabsf(y)) {

		u = m_atanf(y / x);
		u += (x < 0.f) ? (y < 0.f) ? - M_PI_F : M_PI_F : 0.f;
	}
	else {
		u = - m_atanf(x / y);
		u += (y < 0.f) ? - M_PI_F / 2.f : M_PI_F / 2.f;
	}

	return u;
}

static float
m_sincosf(float x)
{
	static const float	lt_sincosf[] = {

		-1.3772950E-4f,
		-2.0450985E-4f,
		 8.6392885E-3f,
		-2.4328724E-4f,
		-1.6656229E-1f,
		-2.2378746E-5f,
		 1.0000019E+0f,
		-3.5525078E-8f,
	};

	float		u;

	u = lt_sincosf[0];
        u = lt_sincosf[1] + u * x;
        u = lt_sincosf[2] + u * x;
        u = lt_sincosf[3] + u * x;
	u = lt_sincosf[4] + u * x;
	u = lt_sincosf[5] + u * x;
	u = lt_sincosf[6] + u * x;
	u = lt_sincosf[7] + u * x;

	return u;
}

float m_sinf(float x)
{
	float           y, u;

	y = m_fabsf(x);

	if (y > M_PI_F / 2.f)
		y = M_PI_F - y;

	u = m_sincosf(y);
	u = (x < 0.f) ? - u : u;

	return u;
}

float m_cosf(float x)
{
        float           u;

	x = M_PI_F / 2.f - m_fabsf(x);
	u = (x < 0.f) ? - m_sincosf(- x) : m_sincosf(x);

	return u;
}

float m_log2f(float x)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x }, m;

	static const float	lt_log2f[] = {

		-1.1632429E-2f,
		 1.4237092E-1f,
		-8.2841748E-1f,
		 4.0229212E-2f,
		 3.9032760E+0f,
		-3.2458262E+0f,
		 3.5400000E+0f,
	};

	float		q;

	m.i = (u.i & 0x7FFFFF) | (0x7F << 23);

	q = lt_log2f[0];
	q = lt_log2f[1] + q * m.f;
	q = lt_log2f[2] + q * m.f;
	q = lt_log2f[3] + q * m.f;
	q = lt_log2f[4] + q * m.f;
	q = lt_log2f[5] + q * m.f;
	q /= (1.f + lt_log2f[6] * m.f);

	q += (float) u.i * (1.f / (float) (1U << 23)) - 127.f;

	return q;
}

float m_log10f(float x)
{
	return m_log2f(x) * M_LOG_10;
}

float m_logf(float x)
{
	return m_log2f(x) * M_LOG_E;
}

float m_exp2f(float x)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x }, m;

	static const float	lt_exp2f[] = {

		1.8948823E-3f,
		8.9472998E-3f,
		5.5861779E-2f,
		2.4014193E-1f,
		6.9315410E-1f,
		1.0000000E+0f,
	};

	float		y, q;

	m.i = (int) ((float) (1U << 23) * (x + 127.f));

	y = x - (int) x + (int) (u.i >> 31);

	q = lt_exp2f[0];
	q = lt_exp2f[1] + q * y;
	q = lt_exp2f[2] + q * y;
	q = lt_exp2f[3] + q * y;
	q = lt_exp2f[4] + q * y;
	q = lt_exp2f[5] + q * y;

	q *= m.f / (1.f + y);

	return q;
}

float m_exp10f(float x)
{
	return m_exp2f(x / M_LOG_10);
}

float m_expf(float x)
{
	return m_exp2f(x / M_LOG_E);
}

void m_la_eigf(const float a[3], float v[4], int m)
{
	float           b, d, la;

	/* Get the eigenvalues \v of quadratic form \a.
	 *
	 * R * [a(0) a(1)] * R' = [v(2) 0   ]
	 *     [a(1) a(2)]        [0    v(3)]
	 *
	 * R = [v(0) -v(1)]
	 *     [v(1)  v(0)].
	 * */

	b = a[0] + a[2];
	d = a[0] * a[0] + a[2] * a[2] - 2.f * a[0] * a[2] + 4.f * a[1] * a[1];

	la = (d > 0.f) ? m_sqrtf(d) : 0.f;

	if (m == 0) {

		v[2] = (b - la) * .5f;
		v[3] = (b + la) * .5f;
	}
	else {
		v[2] = (b + la) * .5f;
		v[3] = (b - la) * .5f;
	}

	if (a[0] >= a[2]) {

		la = a[0] - v[3];
		b = m_sqrtf(la * la + a[1] * a[1]);

		v[0] = la / b;
		v[1] = - a[1] / b;
	}
	else {
		la = a[2] - v[3];
		b = m_sqrtf(la * la + a[1] * a[1]);

		v[0] = a[1] / b;
		v[1] = - la / b;
	}

	if (v[0] < 0.f) {

		v[0] = - v[0];
		v[1] = - v[1];
	}
}

static uint32_t
m_lf_lcgu(uint32_t rseed)
{
	return rseed * 17317U + 1U;
}

void m_lf_randseed(m_seed_t *lf, int seed)
{
	uint32_t	lcgu;

	lcgu = m_lf_lcgu(seed);
	lcgu = m_lf_lcgu(lcgu);

	lf->seed[0] = (float) (lcgu = m_lf_lcgu(lcgu)) / 4294967300.f;
	lf->seed[1] = (float) (lcgu = m_lf_lcgu(lcgu)) / 4294967300.f;
	lf->seed[2] = (float) (lcgu = m_lf_lcgu(lcgu)) / 4294967300.f;
	lf->seed[3] = (float) (lcgu = m_lf_lcgu(lcgu)) / 4294967300.f;
	lf->nb = 0;
}

float m_lf_urandf(m_seed_t *lf)
{
	float		y, a, b;
	int		n, k;

	/* Lagged Fibonacci generator.
	 * */

	n = lf->nb;
	k = (n < 3) ? n + 1 : 0;

	a = lf->seed[k];
	b = lf->seed[n];

	y = (a < b) ? a - b + 1.f : a - b - 1.f;

	lf->seed[k] = y;
	lf->nb = k;

	return y;
}

float m_lf_gaussf(m_seed_t *lf)
{
	float		x;

	/* Normal distribution fast approximation.
	 * */

	x = m_lf_urandf(lf) + m_lf_urandf(lf) + m_lf_urandf(lf);

	return x;
}

