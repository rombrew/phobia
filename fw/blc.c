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

#include "blc.h"

#define KPI		3.14159265f

blc_t			bl;

inline float
clamp(float x, float h, float l)
{
	x = (x > h) ? h : (x < l) ? l : x;

	return x;
}

static void
abc2dq(const float abc[3], const float x[2], float dq[2])
{
	float		xy[2];

	xy[0] = abc[0];
	xy[1] = 0.57735027f * abc[0] + 1.1547005f * abc[1];

	dq[0] = x[1] * xy[0] + x[0] * xy[1];
	dq[1] = -x[0] * xy[0] + x[1] * xy[1];
}

static void
dq2abc(const float dq[2], const float x[2], float abc[3])
{
	float		xy[2];

	xy[0] = x[1] * dq[0] - x[0] * dq[1];
	xy[1] = x[0] * dq[0] + x[1] * dq[1];

	abc[0] = xy[0];
	abc[1] = -0.5f * xy[0] + 0.86602540f * xy[1];
	abc[2] = -(abc[0] + abc[1]);
}

static void
ksincosf(float a, float x[2])
{
	static float	l[8] = {

		-1.37729499e-04f,
		-2.04509846e-04f,
		8.63928854e-03f,
		-2.43287243e-04f,
		-1.66562291e-01f,
		-2.23787462e-05f,
		1.00000193e+00f,
		-3.55250780e-08f,
	};

	float		u;
	int		m = 0;

	if (a < 0.0) {

		m += 1;
		a = -a;
	}

	if (a > (KPI / 2.0f)) {

		m += 2;
		a = KPI - a;
	}

	u = l[1] + l[0] * a;
	u = l[2] + u * a;
	u = l[3] + u * a;
	u = l[4] + u * a;
	u = l[5] + u * a;
	u = l[6] + u * a;
	u = l[7] + u * a;
	x[0] = (m & 1) ? -u : u;

	a = (KPI / 2.0f) - a;
	u = l[1] + l[0] * a;
	u = l[2] + u * a;
	u = l[3] + u * a;
	u = l[4] + u * a;
	u = l[5] + u * a;
	u = l[6] + u * a;
	u = l[7] + u * a;
	x[1] = (m & 2) ? -u : u;
}

static void
mclr(float *c, int m, int n)
{
	int		i, j;

	for (i = 0; i < m; ++i)
		for (j = 0; j < n; ++j) {

			c[i * n + j] = 0.0f;
		}
}

static void
msub(float *a, const float *b, int m, int n)
{
	int		i, j;

	for (i = 0; i < m; ++i)
		for (j = 0; j < n; ++j) {

			a[i * n + j] += - b[i * n + j];
		}
}

static void
mtra(const float *a, float *c, int m, int n)
{
	int		i, j;

	for (i = 0; i < m; ++i)
		for (j = 0; j < n; ++j) {

			c[i * n + j] = a[j * m + i];
		}
}

static void
mmul(const float *a, const float *b, float *c, int m, int n, int k)
{
	int		i, j, l;

	for (i = 0; i < m; ++i)
		for (j = 0; j < n; ++j) {

			c[i * n + j] = 0.0f;
			for (l = 0; l < k; ++l)
				c[i * n + j] += a[i * k + l] * b[l * n + j];
		}
}

static void
mprt(const char *name, const float *a, int m, int n)
{
	int		i, j;

	printf("%s = \n", name);

	for (i = 0; i < m; ++i) {
		for (j = 0; j < n; ++j) {

			printf("%.9f ", a[i * n + j]);
		}

		printf("\n");
	}

	printf("\n\n");
}

static void
kccl_u(const float u[2]);

static void
kalf_kgain();

void blc_enable(float tdel)
{
	bl.tdel = tdel;

	/* Initial FSM.
	 * */
	bl.mode = BLC_MODE_IDLE;

	/* Initial control.
	 * */
	bl.u[0] = 0.0f;
	bl.u[1] = 0.0f;

	/* Post this control.
	 * */
	kccl_u(bl.u);

	/* Initial state.
	 * */
	bl.x[0] = 0.0f;
	bl.x[1] = 0.0f;
	bl.x[2] = 0.0f;
	bl.x[3] = 0.0f;
	bl.x[4] = 0.0f;

	/* Initial covariance.
	 * */
	mclr(bl.pp, 5, 5);
	bl.pp[0] = 1e+2f;
	bl.pp[6] = 1e+2f;
	bl.pp[12] = 2e+2f;
	bl.pp[18] = 3e-2f;
	bl.pp[24] = 3e-4f;

	/* Initial NoFT.
	 * */
	bl.noft = 0;

	/* Initial DQ.
	 * */
	bl.dq[0] = 0.0f;
	bl.dq[1] = 1.0f;

	/* Initial noise variance.
	 * */
	bl.qq[0] = 1e-4f;
	bl.qq[1] = 1e-4f;
	bl.qq[2] = 1e-2f;
	bl.qq[3] = 1e-2f;
	bl.qq[4] = 1e-2f;
	bl.rr[0] = 1e-4f;
	bl.rr[1] = 1e-4f;

	/* Setup Kalman gain.
	 * */
	kalf_kgain();

	/* Initial constants.
	 * */
	bl.c.aD = 0.0f;
	bl.c.cD = 0.0f;
	bl.c.R = 147e-3f;
	bl.c.iL = 1.0f / 44e-6f;
	bl.c.E = 1.2e-3f;
	bl.c.U = 12.0f;
	bl.c.Z = 11;
	bl.c.iJ = 1.0f / 10e-5f;

	/* Initial zero drift Kalman filter.
	 * */
	bl.i.zp = 1e+0f;
	bl.i.zq = 1e-10f;
	bl.i.zr = 1e-4f;

	/* Initial current control loop.
	 * */
	bl.ccl.sp = 10.0f;
	bl.ccl.k[0] = 0.03f;
	bl.ccl.k[1] = 0.005f;
	bl.ccl.x[0] = 0.0f;
	bl.ccl.x[1] = 0.0f;
}

static void
kequation(float dx[4], float x[4])
{
	dx[0] = (bl.u[0] * bl.c.U * 0.5f - x[0] * bl.c.R) * bl.c.iL
		+ x[2] * x[1];
	dx[1] = (bl.u[1] * bl.c.U * 0.5f - x[1] * bl.c.R - x[2] * bl.c.E)
		* bl.c.iL - x[2] * x[0];
	dx[2] = bl.c.Z * bl.c.iJ * (1.5f * bl.c.Z * bl.c.E * x[1] - x[3]);
	dx[3] = x[2];
}

static void
kalf_predict()
{
	float		s1[4], s2[4], x2[4], T;

	T = bl.tdel;

	/* Second-order ODE solver.
	 * */

	x2[0] = bl.x[0];
	x2[1] = bl.x[1];
	x2[2] = bl.x[2];
	x2[3] = bl.x[4];

	kequation(s1, x2);

	x2[0] += s1[0] * T;
	x2[1] += s1[1] * T;
	x2[2] += s1[2] * T;

	kequation(s2, x2);

	bl.x[0] += 0.5f * (s1[0] + s2[0]) * T;
	bl.x[1] += 0.5f * (s1[1] + s2[1]) * T;
	bl.x[2] += 0.5f * (s1[2] + s2[2]) * T;
	bl.x[3] += 0.5f * (s1[3] + s2[3]) * T;
}

static void
kalf_predict_cov()
{
	float		aa[25], aat[25], T;
	float		aapp[25];

	T = bl.tdel;

	mclr(aa, 5, 5);
	aa[0] = 1.0f - bl.c.iL * bl.c.R * T;
	aa[1] = bl.x[2] * T;
	aa[2] = bl.x[1] * T;
	aa[5] = - bl.x[2] * T;
	aa[6] = 1.0f - bl.c.iL * bl.c.R * T;
	aa[7] = -(bl.c.iL * bl.c.E + bl.x[0]) * T;
	aa[11] = 1.5f * bl.c.E * bl.c.iJ * bl.c.Z * bl.c.Z * T;
	aa[12] = 1.0f;
	aa[14] = - bl.c.iJ * bl.c.Z * T;
	aa[17] = T;
	aa[18] = 1.0f;
	aa[24] = 1.0f;

	mmul(aa, bl.pp, aapp, 5, 5, 5);
	mtra(aa, aat, 5, 5);
	mmul(aapp, aat, bl.pp, 5, 5, 5);

	bl.pp[0] += bl.qq[0];
	bl.pp[6] += bl.qq[1];
	bl.pp[12] += bl.qq[2];
	bl.pp[18] += bl.qq[3];
	bl.pp[24] += bl.qq[4];
}

static void
kalf_kgain()
{
	float		cc[10], cct[10];
	float		ccpp[10], ss[4], inv, iss[4];
	float		ppcc[10], kcp[25];

	static int 	n;

	mclr(cc, 2, 5);
	cc[0] = 1.0f;
	cc[6] = 1.0f;

	mmul(cc, bl.pp, ccpp, 2, 5, 5);
	mtra(cc, cct, 5, 2);
	mmul(ccpp, cct, ss, 2, 2, 5);

	ss[0] += bl.rr[0];
	ss[3] += bl.rr[1];

	inv = 1.0f / (ss[0] * ss[3] - ss[1] * ss[2]);
	iss[0] = ss[3] * inv;
	iss[1] = -ss[2] * inv;
	iss[2] = -ss[1] * inv;
	iss[3] = ss[0] * inv;

	mtra(ccpp, ppcc, 5, 2);
	mmul(ppcc, iss, bl.kk, 5, 2, 2);

	mmul(bl.kk, ccpp, kcp, 5, 5, 2);
	msub(bl.pp, kcp, 5, 5);
}

#if 0
static void
kalf_predict_cov()
{
	float		aa[8], T;
	float		aapp[25];

	T = bl.tdel;

	/* Transition matrix.
	 * */
	aa[0] = 1.0f - bl.c.iL * bl.c.R * T;
	aa[1] = bl.x[2] * T;
	aa[2] = bl.x[1] * T;
	aa[3] = bl.c.iL * bl.x[2] * bl.c.E * T;
	aa[4] = - bl.c.iL * bl.c.E * T;
	aa[5] = 1.5f * bl.c.E * bl.c.iJ * bl.c.Z * bl.c.Z * T;
	aa[6] = T;
	aa[7] = - bl.c.iJ * bl.c.Z * T;

	/* Update covariance.
	 * */
	aapp[0] = aa[0] * bl.pp[0] + aa[1] * bl.pp[1]
		+ aa[2] * bl.pp[2] + aa[3] * bl.pp[3];
	aapp[1] = aa[0] * bl.pp[1] + aa[1] * bl.pp[5]
		+ aa[2] * bl.pp[6] + aa[3] * bl.pp[7];
	aapp[2] = aa[0] * bl.pp[2] + aa[1] * bl.pp[6]
		+ aa[2] * bl.pp[9] + aa[3] * bl.pp[10];
	aapp[3] = aa[0] * bl.pp[3] + aa[1] * bl.pp[7]
		+ aa[2] * bl.pp[10] + aa[3] * bl.pp[12];

	aapp[5] = - aa[1] * bl.pp[0] + aa[0] * bl.pp[1]
		+ aa[4] * bl.pp[2];
	aapp[6] = - aa[1] * bl.pp[1] + aa[0] * bl.pp[5]
		+ aa[4] * bl.pp[6];
	aapp[7] = - aa[1] * bl.pp[2] + aa[0] * bl.pp[6]
		+ aa[4] * bl.pp[9];
	aapp[8] = - aa[1] * bl.pp[3] + aa[0] * bl.pp[7]
		+ aa[4] * bl.pp[10];

	aapp[10] = aa[5] * bl.pp[1] + bl.pp[2] + aa[7] * bl.pp[4];
	aapp[11] = aa[5] * bl.pp[5] + bl.pp[6] + aa[7] * bl.pp[8];
	aapp[12] = aa[5] * bl.pp[6] + bl.pp[9] + aa[7] * bl.pp[11];
	aapp[13] = aa[5] * bl.pp[7] + bl.pp[10] + aa[7] * bl.pp[13];
	aapp[14] = aa[5] * bl.pp[8] + bl.pp[11] + aa[7] * bl.pp[14];

	aapp[15] = aa[6] * bl.pp[2] + bl.pp[3];
	aapp[16] = aa[6] * bl.pp[6] + bl.pp[7];
	aapp[17] = aa[6] * bl.pp[9] + bl.pp[10];
	aapp[18] = aa[6] * bl.pp[10] + bl.pp[12];
	aapp[19] = aa[6] * bl.pp[11] + bl.pp[13];

	aapp[20] = bl.pp[4];
	aapp[21] = bl.pp[8];
	aapp[22] = bl.pp[11];
	aapp[23] = bl.pp[13];
	aapp[24] = bl.pp[14];

	bl.pp[0] = aa[0] * aapp[0] + aa[1] * aapp[1]
		+ aa[2] * aapp[2] + aa[3] * aapp[3];
	bl.pp[1] = aa[0] * aapp[5] + aa[1] * aapp[6]
		+ aa[2] * aapp[7] + aa[3] * aapp[8];
	bl.pp[2] = aa[0] * aapp[10] + aa[1] * aapp[11]
		+ aa[2] * aapp[12] + aa[3] * aapp[13];
	bl.pp[3] = aa[0] * aapp[15] + aa[1] * aapp[16]
		+ aa[2] * aapp[17] + aa[3] * aapp[18];
	bl.pp[4] = aa[0] * aapp[20] + aa[1] * aapp[21]
		+ aa[2] * aapp[22] + aa[3] * aapp[23];

	bl.pp[5] = - aa[1] * aapp[5] + aa[0] * aapp[6]
		+ aa[4] * aapp[7];
	bl.pp[6] = - aa[1] * aapp[10] + aa[0] * aapp[11]
		+ aa[4] * aapp[12];
	bl.pp[7] = - aa[1] * aapp[15] + aa[0] * aapp[16]
		+ aa[4] * aapp[17];
	bl.pp[8] = - aa[1] * aapp[20] + aa[0] * aapp[21]
		+ aa[4] * aapp[22];

	bl.pp[9] = aa[5] * aapp[11] + aapp[12] + aa[7] * aapp[14];
	bl.pp[10] = aa[5] * aapp[16] + aapp[17] + aa[7] * aapp[19];
	bl.pp[11] = aa[5] * aapp[21] + aapp[22] + aa[7] * aapp[24];

	bl.pp[12] = aa[6] * aapp[17] + aapp[18];
	bl.pp[13] = aa[6] * aapp[22] + aapp[23];
	bl.pp[14] = aapp[24];

	bl.pp[0] += bl.qq[0];
	bl.pp[5] += bl.qq[1];
	bl.pp[9] += bl.qq[2];
	bl.pp[12] += bl.qq[3];
	bl.pp[14] += bl.qq[4];
}

static void
kalf_kgain()
{
	float		ss[3], inv, iss[3], kcp[15];

	/* Calculate Kalman gain.
	 * */
	ss[0] = bl.pp[0] + bl.rr[0];
	ss[1] = bl.pp[1];
	ss[2] = bl.pp[5] + bl.rr[1];

	inv = 1.0f / (ss[0] * ss[2] - ss[1] * ss[1]);
	iss[0] = ss[2] * inv;
	iss[1] = -ss[1] * inv;
	iss[2] = ss[0] * inv;

	bl.kk[0] = bl.pp[0] * iss[0] + bl.pp[1] * iss[1];
	bl.kk[1] = bl.pp[0] * iss[1] + bl.pp[1] * iss[2];
	bl.kk[2] = bl.pp[1] * iss[0] + bl.pp[5] * iss[1];
	bl.kk[3] = bl.pp[1] * iss[1] + bl.pp[5] * iss[2];
	bl.kk[4] = bl.pp[2] * iss[0] + bl.pp[6] * iss[1];
	bl.kk[5] = bl.pp[2] * iss[1] + bl.pp[6] * iss[2];
	bl.kk[6] = bl.pp[3] * iss[0] + bl.pp[7] * iss[1];
	bl.kk[7] = bl.pp[3] * iss[1] + bl.pp[7] * iss[2];
	bl.kk[8] = bl.pp[4] * iss[0] + bl.pp[8] * iss[1];
	bl.kk[9] = bl.pp[4] * iss[1] + bl.pp[8] * iss[2];

	/* Update covariance.
	 * */
	kcp[0] = bl.kk[0] * bl.pp[0] + bl.kk[1] * bl.pp[1];
	kcp[1] = bl.kk[0] * bl.pp[1] + bl.kk[1] * bl.pp[5];
	kcp[2] = bl.kk[0] * bl.pp[2] + bl.kk[1] * bl.pp[6];
	kcp[3] = bl.kk[0] * bl.pp[3] + bl.kk[1] * bl.pp[7];
	kcp[4] = bl.kk[0] * bl.pp[4] + bl.kk[1] * bl.pp[8];
	kcp[5] = bl.kk[2] * bl.pp[1] + bl.kk[3] * bl.pp[5];
	kcp[6] = bl.kk[2] * bl.pp[2] + bl.kk[3] * bl.pp[6];
	kcp[7] = bl.kk[2] * bl.pp[3] + bl.kk[3] * bl.pp[7];
	kcp[8] = bl.kk[2] * bl.pp[4] + bl.kk[3] * bl.pp[8];
	kcp[9] = bl.kk[4] * bl.pp[2] + bl.kk[5] * bl.pp[6];
	kcp[10] = bl.kk[4] * bl.pp[3] + bl.kk[5] * bl.pp[7];
	kcp[11] = bl.kk[4] * bl.pp[4] + bl.kk[5] * bl.pp[8];
	kcp[12] = bl.kk[6] * bl.pp[3] + bl.kk[7] * bl.pp[7];
	kcp[13] = bl.kk[6] * bl.pp[4] + bl.kk[7] * bl.pp[8];
	kcp[14] = bl.kk[8] * bl.pp[4] + bl.kk[9] * bl.pp[8];

	bl.pp[0] -= kcp[0];
	bl.pp[1] -= kcp[1];
	bl.pp[2] -= kcp[2];
	bl.pp[3] -= kcp[3];
	bl.pp[4] -= kcp[4];
	bl.pp[5] -= kcp[5];
	bl.pp[6] -= kcp[6];
	bl.pp[7] -= kcp[7];
	bl.pp[8] -= kcp[8];
	bl.pp[9] -= kcp[9];
	bl.pp[10] -= kcp[10];
	bl.pp[11] -= kcp[11];
	bl.pp[12] -= kcp[12];
	bl.pp[13] -= kcp[13];
	bl.pp[14] -= kcp[14];
}
#endif

static void
kali_zd(const float e[2], const float dq[2])
{
	float		eabc[3], k;

	/* Get error back to ABC axes.
	 * */
	dq2abc(e, dq, eabc);

	/* Kalman filter.
	 * */
	bl.i.zp += bl.i.zq;
	k = bl.i.zp / (bl.i.zp + bl.i.zr);
	bl.c.aD += k * eabc[0];
	bl.c.cD += k * eabc[2];
	bl.i.zp -= k * bl.i.zp;
}

static void
kali_plant(const float e[2], const float dq[2], const float x[5])
{
	float		cc[10], ppcc[10], ss[3];

	cc[0] = 0;
	cc[9] = 0;

	ppcc[0] = bl.i.pp[0] * cc[0] + bl.i.pp[1] * cc[1]
		+ bl.i.pp[2] * cc[2] + bl.i.pp[3] * cc[3]
		+ bl.i.pp[4] * cc[4];
	ppcc[1] = bl.i.pp[0] * cc[0] + bl.i.pp[1] * cc[1]
		+ bl.i.pp[2] * cc[2] + bl.i.pp[3] * cc[3]
		+ bl.i.pp[4] * cc[4];
}

static void
kccl_u(const float u[2])
{
	float		dc[3];

	/* Transform to ABC axes.
	 * */
	dq2abc(u, bl.dq, dc);

	/* Convert to DC.
	 * */
	dc[0] = dc[0] * 0.5f + 0.5f;
	dc[1] = dc[1] * 0.5f + 0.5f;
	dc[2] = dc[2] * 0.5f + 0.5f;

	/* Pass to the PWM driver.
	 * */
	bridge_dc(dc);
}

static void
kccl_update(const float x[2], float spd, float spq)
{
	float		e[2], u[2];

	/* Obtain the error of regulation.
	 * */
	e[0] = spd - x[0];
	e[1] = spq - x[1];

	/* DQ current regulator.
	 * */
	bl.ccl.x[0] += bl.ccl.k[1] * e[0];
	bl.ccl.x[0] = clamp(bl.ccl.x[0], 1.0f, -1.0f);
	u[0] = bl.ccl.k[0] * e[0] + bl.ccl.x[0];

	bl.ccl.x[1] += bl.ccl.k[1] * e[1];
	bl.ccl.x[1] = clamp(bl.ccl.x[1], 1.0f, -1.0f);
	u[1] = bl.ccl.k[0] * e[1] + bl.ccl.x[1];

	u[0] = clamp(u[0], 1.0f, -1.0f);
	u[1] = clamp(u[1], 1.0f, -1.0f);

	/* Store control signal.
	 * */
	bl.u[0] = u[0];
	bl.u[1] = u[1];

	/* Post this control.
	 * */
	kccl_u(bl.u);
}

void blc_update(const float z[2])
{
	float		zabc[3], zdq[2], e[2];

	/* Zero drift cancelation.
	 * */
	zabc[0] = z[0] - bl.c.aD;
	zabc[2] = z[1] - bl.c.cD;
	zabc[1] = -(zabc[0] + zabc[2]);

	/* Transform to DQ frame.
	 * */
	abc2dq(zabc, bl.dq, zdq);

	/* -------------------------------------------- */

	if (bl.mode == BLC_MODE_DRIFT) {

		/* Identify zero drift.
		 * */
		kali_zd(zdq, bl.dq);
	}
	else if (bl.mode == BLC_MODE_ALIGN) {

		/* Open loop control to align the rotor.
		 * */
		kccl_update(zdq, bl.ccl.sp, 0.0f);
	}
	else if (bl.mode == BLC_MODE_RUN) {

		/* Obtain the error of observation.
		 * */
		e[0] = zdq[0] - bl.x[0];
		e[1] = zdq[1] - bl.x[1];

		/* Observer innovation.
		 * */
		bl.e[0] = bl.kk[0] * e[0] + bl.kk[1] * e[1];
		bl.e[1] = bl.kk[2] * e[0] + bl.kk[3] * e[1];
		bl.e[2] = bl.kk[4] * e[0] + bl.kk[5] * e[1];
		bl.e[3] = bl.kk[6] * e[0] + bl.kk[7] * e[1];
		bl.e[4] = bl.kk[8] * e[0] + bl.kk[9] * e[1];

		/* Update state estimate.
		 * */
		bl.x[0] += bl.e[0];
		bl.x[1] += bl.e[1];
		bl.x[2] += bl.e[2];
		bl.x[3] += bl.e[3];
		bl.x[4] += bl.e[4];

		/* Predict state.
		 * */
		kalf_predict();

		/* Wrap angular position.
		 * */
		if (bl.x[3] < -KPI) {

			bl.x[3] += 2.0f * KPI;
			bl.noft += 1;
		}
		else if (bl.x[3] > KPI) {

			bl.x[3] -= 2.0f * KPI;
			bl.noft += 1;
		}

		/* Clamp angular position.
		 * */
		bl.x[3] = clamp(bl.x[3], KPI, -KPI);

		/* Update DQ frame axes.
		 * */
		ksincosf(bl.x[3], bl.dq);

		/* Update control.
		 * */
		kccl_update(bl.x, 0.0f, bl.ccl.sp);

		/* -------------------------------------------- */

		/* Calculate predicted covariance.
		 * */
		kalf_predict_cov();

		/* Calculate Kalman gain.
		 * */
		kalf_kgain();

		/* -------------------------------------------- */

		/* Update identifier.
		 * */
		//kali_zd(e, bl.dq);
		//kali_plant(e, bl.x);
	}
}

