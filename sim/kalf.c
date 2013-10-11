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

#include "kalf.h"
#include "klib.h"

kalf_t		kf;

void kalf_enable(float tdel)
{
	kf.tdel = tdel;

	kf.x[0] = 0.0f;
	kf.x[1] = 0.0f;
	kf.x[2] = 0.0f;

	kf.x[3] = 1.0f;
	kf.x[4] = 0.0f;

	kf.x[5] = 0.0f;
}

static void
kadd(float x[2], float a)
{
	float		s1[2], s2[2];
	float		x2[2], il;

	a = (a > 1.0f) ? 1.0f : a;
	a = (a < -1.0f) ? -1.0f : a;

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
	dx[0] = (kf.u[0] * kf.c.U - x[0] * kf.c.R) * kf.c.iL
		+ x[2] * x[1];
	dx[1] = (kf.u[1] * kf.c.U - x[1] * kf.c.R - x[2] * kf.c.E) * kf.c.iL
		- x[2] * x[0];
	dx[2] = kf.c.Z * kf.c.iJ * (1.5f * kf.c.Z * kf.c.E * x[1] - x[3]);
	dx[3] = x[2];
}

static void
kalf_predict()
{
	float		s1[4], s2[4], x2[4], T;

	T = kf.tdel;

	/* Second-order ODE solver.
	 * */

	x2[0] = kf.x[0];
	x2[1] = kf.x[1];
	x2[2] = kf.x[2];
	x2[3] = kf.x[5];

	kequation(s1, x2);

	x2[0] = kf.x[0] + s1[0] * T;
	x2[1] = kf.x[1] + s1[1] * T;
	x2[2] = kf.x[2] + s1[2] * T;

	kequation(s2, x2);

	kf.x[0] += 0.5f * (s1[0] + s2[0]) * T;
	kf.x[1] += 0.5f * (s1[1] + s2[1]) * T;
	kf.x[2] += 0.5f * (s1[2] + s2[2]) * T;
	kadd(kf.x + 3, (s1[3] + s2[3]) * T * 0.5f);
}

static void
kalf_predict_cov()
{
	float		aa[8], T;
	float		aapp[25];

	T = kf.tdel;

	aa[0] = 1.0f - kf.c.iL * kf.c.R * T;
	aa[1] = kf.x[2] * T;
	aa[2] = kf.x[1] * T;
	aa[3] = kf.c.iL * kf.x[2] * kf.c.E * T;
	aa[4] = - kf.c.iL * kf.c.E * T;
	aa[5] = 1.5f * kf.c.E * kf.c.iJ * kf.c.Z * kf.c.Z * T;
	aa[6] = T;
	aa[7] = - kf.c.iJ * kf.c.Z * T;

	aapp[0] = aa[0] * kf.pp[0] + aa[1] * kf.pp[1]
		+ aa[2] * kf.pp[2] + aa[3] * kf.pp[3];
	aapp[1] = aa[0] * kf.pp[1] + aa[1] * kf.pp[5]
		+ aa[2] * kf.pp[6] + aa[3] * kf.pp[7];
	aapp[2] = aa[0] * kf.pp[2] + aa[1] * kf.pp[6]
		+ aa[2] * kf.pp[9] + aa[3] * kf.pp[10];
	aapp[3] = aa[0] * kf.pp[3] + aa[1] * kf.pp[7]
		+ aa[2] * kf.pp[10] + aa[3] * kf.pp[12];
	/*aapp[4] = aa[0] * kf.pp[4] + aa[1] * kf.pp[8]
		+ aa[2] * kf.pp[11] + aa[3] * kf.pp[13];*/

	aapp[5] = - aa[1] * kf.pp[0] + aa[0] * kf.pp[1]
		+ aa[4] * kf.pp[2];
	aapp[6] = - aa[1] * kf.pp[1] + aa[0] * kf.pp[5]
		+ aa[4] * kf.pp[6];
	aapp[7] = - aa[1] * kf.pp[2] + aa[0] * kf.pp[6]
		+ aa[4] * kf.pp[9];
	aapp[8] = - aa[1] * kf.pp[3] + aa[0] * kf.pp[7]
		+ aa[4] * kf.pp[10];
	/*aapp[9] = - aa[1] * kf.pp[4] + aa[0] * kf.pp[8]
		+ aa[4] * kf.pp[11];*/

	aapp[10] = aa[5] * kf.pp[1] + kf.pp[2] + aa[7] * kf.pp[4];
	aapp[11] = aa[5] * kf.pp[5] + kf.pp[6] + aa[7] * kf.pp[8];
	aapp[12] = aa[5] * kf.pp[6] + kf.pp[9] + aa[7] * kf.pp[11];
	aapp[13] = aa[5] * kf.pp[7] + kf.pp[10] + aa[7] * kf.pp[13];
	aapp[14] = aa[5] * kf.pp[8] + kf.pp[11] + aa[7] * kf.pp[14];

	aapp[15] = aa[6] * kf.pp[2] + kf.pp[3];
	aapp[16] = aa[6] * kf.pp[6] + kf.pp[7];
	aapp[17] = aa[6] * kf.pp[9] + kf.pp[10];
	aapp[18] = aa[6] * kf.pp[10] + kf.pp[12];
	aapp[19] = aa[6] * kf.pp[11] + kf.pp[13];

	aapp[20] = kf.pp[4];
	aapp[21] = kf.pp[8];
	aapp[22] = kf.pp[11];
	aapp[23] = kf.pp[13];
	aapp[24] = kf.pp[14];

	kf.pp[0] = aa[0] * aapp[0] + aa[1] * aapp[1]
		+ aa[2] * aapp[2] + aa[3] * aapp[3];
	kf.pp[1] = aa[0] * aapp[5] + aa[1] * aapp[6]
		+ aa[2] * aapp[7] + aa[3] * aapp[8];
	kf.pp[2] = aa[0] * aapp[10] + aa[1] * aapp[11]
		+ aa[2] * aapp[12] + aa[3] * aapp[13];
	kf.pp[3] = aa[0] * aapp[15] + aa[1] * aapp[16]
		+ aa[2] * aapp[17] + aa[3] * aapp[18];
	kf.pp[4] = aa[0] * aapp[20] + aa[1] * aapp[21]
		+ aa[2] * aapp[22] + aa[3] * aapp[23];

	kf.pp[5] = - aa[1] * aapp[5] + aa[0] * aapp[6]
		+ aa[4] * aapp[7];
	kf.pp[6] = - aa[1] * aapp[10] + aa[0] * aapp[11]
		+ aa[4] * aapp[12];
	kf.pp[7] = - aa[1] * aapp[15] + aa[0] * aapp[16]
		+ aa[4] * aapp[17];
	kf.pp[8] = - aa[1] * aapp[20] + aa[0] * aapp[21]
		+ aa[4] * aapp[22];

	kf.pp[9] = aa[5] * aapp[11] + aapp[12] + aa[7] * aapp[14];
	kf.pp[10] = aa[5] * aapp[16] + aapp[17] + aa[7] * aapp[19];
	kf.pp[11] = aa[5] * aapp[21] + aapp[22] + aa[7] * aapp[24];

	kf.pp[12] = aa[6] * aapp[17] + aapp[18];
	kf.pp[13] = aa[6] * aapp[22] + aapp[23];
	kf.pp[14] = aapp[24];

	kf.pp[0] += kf.qq[0];
	kf.pp[5] += kf.qq[1];
	kf.pp[9] += kf.qq[2];
	kf.pp[12] += kf.qq[3];
	kf.pp[14] += kf.qq[4];
}

static void
kalf_kgain()
{
	float		ss[3], inv, iss[3], kcp[15];

	ss[0] = kf.pp[0] + kf.rr[0];
	ss[1] = kf.pp[1];
	ss[2] = kf.pp[5] + kf.rr[1];

	inv = 1.0f / (ss[0] * ss[2] - ss[1] * ss[1]);
	iss[0] = ss[2] * inv;
	iss[1] = -ss[1] * inv;
	iss[2] = ss[0] * inv;

	kf.kk[0] = kf.pp[0] * iss[0] + kf.pp[1] * iss[1];
	kf.kk[1] = kf.pp[0] * iss[1] + kf.pp[1] * iss[2];
	kf.kk[2] = kf.pp[1] * iss[0] + kf.pp[5] * iss[1];
	kf.kk[3] = kf.pp[1] * iss[1] + kf.pp[5] * iss[2];
	kf.kk[4] = kf.pp[2] * iss[0] + kf.pp[6] * iss[1];
	kf.kk[5] = kf.pp[2] * iss[1] + kf.pp[6] * iss[2];
	kf.kk[6] = kf.pp[3] * iss[0] + kf.pp[7] * iss[1];
	kf.kk[7] = kf.pp[3] * iss[1] + kf.pp[7] * iss[2];
	kf.kk[8] = kf.pp[4] * iss[0] + kf.pp[8] * iss[1];
	kf.kk[9] = kf.pp[4] * iss[1] + kf.pp[8] * iss[2];

	/* Update covariance.
	 * */
	kcp[0] = kf.kk[0] * kf.pp[0] + kf.kk[1] * kf.pp[1];
	kcp[1] = kf.kk[0] * kf.pp[1] + kf.kk[1] * kf.pp[5];
	kcp[2] = kf.kk[0] * kf.pp[2] + kf.kk[1] * kf.pp[6];
	kcp[3] = kf.kk[0] * kf.pp[3] + kf.kk[1] * kf.pp[7];
	kcp[4] = kf.kk[0] * kf.pp[4] + kf.kk[1] * kf.pp[8];
	kcp[5] = kf.kk[2] * kf.pp[1] + kf.kk[3] * kf.pp[5];
	kcp[6] = kf.kk[2] * kf.pp[2] + kf.kk[3] * kf.pp[6];
	kcp[7] = kf.kk[2] * kf.pp[3] + kf.kk[3] * kf.pp[7];
	kcp[8] = kf.kk[2] * kf.pp[4] + kf.kk[3] * kf.pp[8];
	kcp[9] = kf.kk[4] * kf.pp[2] + kf.kk[5] * kf.pp[6];
	kcp[10] = kf.kk[4] * kf.pp[3] + kf.kk[5] * kf.pp[7];
	kcp[11] = kf.kk[4] * kf.pp[4] + kf.kk[5] * kf.pp[8];
	kcp[12] = kf.kk[6] * kf.pp[3] + kf.kk[7] * kf.pp[7];
	kcp[13] = kf.kk[6] * kf.pp[4] + kf.kk[7] * kf.pp[8];
	kcp[14] = kf.kk[8] * kf.pp[4] + kf.kk[9] * kf.pp[8];

	kf.pp[0] -= kcp[0];
	kf.pp[1] -= kcp[1];
	kf.pp[2] -= kcp[2];
	kf.pp[3] -= kcp[3];
	kf.pp[4] -= kcp[4];
	kf.pp[5] -= kcp[5];
	kf.pp[6] -= kcp[6];
	kf.pp[7] -= kcp[7];
	kf.pp[8] -= kcp[8];
	kf.pp[9] -= kcp[9];
	kf.pp[10] -= kcp[10];
	kf.pp[11] -= kcp[11];
	kf.pp[12] -= kcp[12];
	kf.pp[13] -= kcp[13];
	kf.pp[14] -= kcp[14];
}

void kalf_update(int i[2])
{
	float		z[3], zdq[2], e[2];

	/* Prepare measured signals.
	 * */
	z[0] = (float) (i[0] - 2048) * kf.c.gI + kf.c.aD;
	z[2] = (float) (i[1] - 2048) * kf.c.gI + kf.c.cD;
	z[1] = - z[0] - z[2];

	/* Transform to DQ frame.
	 * */
	abc2dq(z, kf.x + 3, zdq);

	/* ----------------------------------------------------- */

	/* Obtain the error.
	 * */
	e[0] = zdq[0] - kf.x[0];
	e[1] = zdq[1] - kf.x[1];

	/* Update state estimate.
	 * */
	kf.x[0] += kf.kk[0] * e[0] + kf.kk[1] * e[1];
	kf.x[1] += kf.kk[2] * e[0] + kf.kk[3] * e[1];
	kf.x[2] += kf.kk[4] * e[0] + kf.kk[5] * e[1];
	kadd(kf.x + 3, kf.kk[6] * e[0] + kf.kk[7] * e[1]);
	kf.x[5] += kf.kk[8] * e[0] + kf.kk[9] * e[1];

	/* Predict.
	 * */
	kalf_predict();

	/* ----------------------------------------------------- */

	// FIXME: control

	/* ----------------------------------------------------- */

	/* Calculate predicted covariance.
	 * */
	kalf_predict_cov();

	/* Calculate Kalman gain.
	 * */
	kalf_kgain();

	/* ----------------------------------------------------- */

	/* FIXME: Identify
	 * */
}

