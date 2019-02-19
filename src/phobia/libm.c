#include "libm.h"

static const float	lt_atanf[] = {

	-8.9550074E-3f,
	 4.5295657E-2f,
	-1.0939535E-1f,
	 1.9063993E-1f,
	-3.3214334E-1f,
	 9.9995555E-1f,
};

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

static const float	lt_log2f[] = {

	-1.1632429E-2f,
	 1.4237092E-1f,
	-8.2841748E-1f,
	 4.0229212E-2f,
	 3.9032760E+0f,
	-3.2458262E+0f,
	 3.5400000E+0f,
};

static const float	lt_exp2f[] = {

	1.8948823e-3f,
	8.9472998e-3f,
	5.5861779e-2f,
	2.4014193e-1f,
	6.9315410e-1f,
	1.0000000e+0f,
};

int m_isfinitef(float x)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x };

	return ((0xFFUL & (u.i >> 23)) != 0xFFUL) ? 1 : 0;
}

void m_rotf(float y[2], float r, const float x[2])
{
	float           q, s, c, a, b;

	q = r * r;
	s = r - q * r * (1.f / 6.f - (1.f / 120.f) * q);
	c = 1.f - q * (.5f - (1.f / 24.f) * q);

	a = c * x[0] - s * x[1];
	b = s * x[0] + c * x[1];

	q = (3.f - a * a - b * b) * .5f;

	y[0] = a * q;
	y[1] = b * q;
}

static float
m_atanf(float x)
{
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
        float           x_abs, u;

	x_abs = m_fabsf(x);

	if (x_abs > (M_PI_F / 2.f))
		x_abs = M_PI_F - x_abs;

	u = m_sincosf(x_abs);
	u = (x < 0.f) ? - u : u;

	return u;
}

float m_cosf(float x)
{
        float           u;

	x = (M_PI_F / 2.f) - m_fabsf(x);
	u = (x < 0.f) ? - m_sincosf(- x) : m_sincosf(x);

	return u;
}

float m_log2f(float x)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x }, m;

	float		q;

	m.i = (u.i & 0x7FFFFF) | (0x7F << 23);

	q = lt_log2f[0];
	q = lt_log2f[1] + q * m.f;
	q = lt_log2f[2] + q * m.f;
	q = lt_log2f[3] + q * m.f;
	q = lt_log2f[4] + q * m.f;
	q = lt_log2f[5] + q * m.f;
	q /= (1.f + lt_log2f[6] * m.f);

	q += (float) u.i * (1.f / (float) (1UL << 23)) - 127.f;

	return q;
}

float m_logf(float x)
{
	return m_log2f(x) * M_LOG2_F;
}

float m_exp2f(float x)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x }, m;

	float		r, q;

	m.i = (int) ((float) (1UL << 23) * (x + 127.f));
	r = x - (int) x + (int) (u.i >> 31);

	q = lt_exp2f[0];
        q = lt_exp2f[1] + q * r;
        q = lt_exp2f[2] + q * r;
        q = lt_exp2f[3] + q * r;
	q = lt_exp2f[4] + q * r;
	q = lt_exp2f[5] + q * r;

	q *= m.f / (1.f + r);

	return q;
}

float m_expf(float x)
{
	return m_exp2f(x * (1.f / M_LOG2_F));
}

