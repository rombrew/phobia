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

blc_t			bl;

float sqrtf(float x);

inline float
satf(float x, float h, float l)
{
	x = (x > h) ? h : (x < l) ? l : x;

	return x;
}

static void
abc2dq(const float abc[3], const float e[2], float dq[2])
{
	float		xy[2];

	xy[0] = abc[0];
	xy[1] = 0.57735027f * abc[0] + 1.1547005f * abc[1];

	dq[0] = e[0] * xy[0] + e[1] * xy[1];
	dq[1] = -e[1] * xy[0] + e[0] * xy[1];
}

static void
dq2abc(const float dq[2], const float e[2], float abc[3])
{
	float		xy[2];

	xy[0] = e[0] * dq[0] - e[1] * dq[1];
	xy[1] = e[1] * dq[0] + e[0] * dq[1];

	abc[0] = xy[0];
	abc[1] = -0.5f * xy[0] + 0.86602540f * xy[1];
	abc[2] = -(abc[0] + abc[1]);
}

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

	/* Initial state.
	 * */
	bl.x[0] = 0.0f;
	bl.x[1] = 0.0f;
	bl.x[2] = 0.0f;
	bl.x[3] = 1.0f;
	bl.x[4] = 0.0f;
	bl.x[5] = 0.0f;

	/* Initial covariance.
	 * */
	bl.pp[0] = 5e+2f;
	bl.pp[1] = 0.0f;
	bl.pp[2] = 0.0f;
	bl.pp[3] = 0.0f;
	bl.pp[4] = 0.0f;
	bl.pp[5] = 5e+2f;
	bl.pp[6] = 0.0f;
	bl.pp[7] = 0.0f;
	bl.pp[8] = 0.0f;
	bl.pp[9] = 9e+6f;
	bl.pp[10] = 0.0f;
	bl.pp[11] = 0.0f;
	bl.pp[12] = 4e+1f;
	bl.pp[13] = 0.0f;
	bl.pp[14] = 5e+0f;

	/* Initial noise variance.
	 * */
	bl.qq[0] = 1e-2f;
	bl.qq[1] = 1e-2f;
	bl.qq[2] = 1e-4f;
	bl.qq[3] = 1e-5f;
	bl.qq[4] = 1e-5f;
	bl.rr[0] = 1e-4f;
	bl.rr[1] = 1e-4f;

	/* Setup Kalman gain.
	 * */
	kalf_kgain();

	/* Initial constants.
	 * */
	bl.c.gI = 0.014663f;
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
	bl.zdk[0] = 1.0f;
	bl.zdk[1] = 1e-9f;
	bl.zdk[2] = 1e-4f;

	/* Initial current control loop.
	 * */
	bl.ccl_sp = 1.0f;
	bl.ccl_k[0] = 0.03f;
	bl.ccl_k[1] = 0.005f;
}

static void
kadd(float x[2], float a)
{
	float		s1[2], s2[2];
	float		x2[2], il;

	a = satf(a, 1.0f, -1.0f);

	s1[0] = -x[1] * a;
	s1[1] = x[0] * a;

	x2[0] = x[0] + s1[0];
	x2[1] = x[1] + s1[1];

	s2[0] = -x2[1] * a;
	s2[1] = x2[0] * a;

	x[0] += 0.5f * (s1[0] + s2[0]);
	x[1] += 0.5f * (s1[1] + s2[1]);

	il = 1.0f / sqrtf(x[0] * x[0] + x[1] * x[1]);
	x[0] *= il;
	x[1] *= il;
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
	x2[3] = bl.x[5];

	kequation(s1, x2);

	x2[0] = bl.x[0] + s1[0] * T;
	x2[1] = bl.x[1] + s1[1] * T;
	x2[2] = bl.x[2] + s1[2] * T;

	kequation(s2, x2);

	bl.x[0] += 0.5f * (s1[0] + s2[0]) * T;
	bl.x[1] += 0.5f * (s1[1] + s2[1]) * T;
	bl.x[2] += 0.5f * (s1[2] + s2[2]) * T;
	kadd(bl.x + 3, (s1[3] + s2[3]) * T * 0.5f);
}

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

static void
kccl_u(const float u[2])
{
	float		dc[3];

	/* Transform to ABC axes.
	 * */
	dq2abc(u, bl.x + 3, dc);

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
	bl.ccl_x[0] += bl.ccl_k[1] * e[0];
	bl.ccl_x[0] = satf(bl.ccl_x[0], 1.0f, -1.0f);
	u[0] = bl.ccl_k[0] * e[0] + bl.ccl_x[0];

	bl.ccl_x[1] += bl.ccl_k[1] * e[1];
	bl.ccl_x[1] = satf(bl.ccl_x[1], 1.0f, -1.0f);
	u[1] = bl.ccl_k[0] * e[1] + bl.ccl_x[1];

	u[0] = satf(u[0], 1.0f, -1.0f);
	u[1] = satf(u[1], 1.0f, -1.0f);

	/* Store control signal.
	 * */
	bl.u[0] = u[0];
	bl.u[1] = u[1];

	/* Post this control.
	 * */
	kccl_u(u);
}

static void
kzdi_update(const float e[2], const float x[2])
{
	float		eabc[3], k;

	/* Get error back to ABC axes.
	 * */
	dq2abc(e, x, eabc);

	/* Kalman filter.
	 * */
	bl.zdk[0] += bl.zdk[1];
	k = bl.zdk[0] / (bl.zdk[0] + bl.zdk[2]);
	bl.c.aD -= k * eabc[0];
	bl.c.cD -= k * eabc[2];
	bl.zdk[0] -= k * bl.zdk[0];
}

void blc_update(const int i[2])
{
	float		z[3], zdq[2], e[2];

	/* Prepare measured signals.
	 * */
	z[0] = (float) (i[0] - 2048) * bl.c.gI + bl.c.aD;
	z[2] = (float) (i[1] - 2048) * bl.c.gI + bl.c.cD;
	z[1] = -(z[0] + z[2]);

	/* Transform to DQ frame.
	 * */
	abc2dq(z, bl.x + 3, zdq);

	/* -------------------------------------------- */

	if (bl.mode == BLC_MODE_IDLE) {

		/* Identify zero drift.
		 * */
		kzdi_update(zdq, bl.x + 3);
	}
	else if (bl.mode == BLC_MODE_ALIGN) {

		/* Open loop control to align the rotor.
		 * */
		kccl_update(zdq, bl.ccl_sp, 0.0f);
	}
	else if (bl.mode == BLC_MODE_RUN) {

		/* Obtain the error of observation.
		 * */
		e[0] = zdq[0] - bl.x[0];
		e[1] = zdq[1] - bl.x[1];

		/* Update state estimate.
		 * */
		bl.x[0] += bl.kk[0] * e[0] + bl.kk[1] * e[1];
		bl.x[1] += bl.kk[2] * e[0] + bl.kk[3] * e[1];
		bl.x[2] += bl.kk[4] * e[0] + bl.kk[5] * e[1];
		kadd(bl.x + 3, bl.kk[6] * e[0] + bl.kk[7] * e[1]);
		bl.x[5] += bl.kk[8] * e[0] + bl.kk[9] * e[1];

		/* Predict state.
		 * */
		kalf_predict();

		/* Update control.
		 * */
		kccl_update(bl.x, 0.0f, bl.ccl_sp);

		/*printf("-> %f %f %f %f %f %f \n",
			bl.x[0], bl.x[1], bl.x[2], bl.x[3], bl.x[4], bl.x[5]);
		printf("-> %f %f\n",
			bl.u[0], bl.u[1]);
		system("read");*/

		/* -------------------------------------------- */

		/* Calculate predicted covariance.
		 * */
		kalf_predict_cov();

		/* Calculate Kalman gain.
		 * */
		kalf_kgain();

		/* -------------------------------------------- */

		/* FIXME: Identify
		 * */
	}
}

