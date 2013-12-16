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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "plant.h"
#include "blc.h"
#include "lib.h"

void bridge_dc(const float dc[3])
{
	plant.ub[0] = (double) dc[0];
	plant.ub[1] = (double) dc[1];
	plant.ub[2] = (double) dc[2];
}

static void
script(double t)
{
	if (t < 100e-3) {

		bl.mode = BLC_MODE_DRIFT;
	}
	else if (t < 900e-3) {

		bl.mode = BLC_MODE_ALIGN;
		bl.ccl.sp = (float) ((t - 100e-3) / 800e-3);
	}
	else if (t < 1.5) {

		bl.mode = BLC_MODE_RUN;
		bl.ccl.sp = 10.0f;
	}
	else if (t < 2.0) {

		bl.ccl.sp = 4.0f;
	}
	else {

		bl.ccl.sp = 15.0f;
	}
}

static void
abc2dq(const double ab[2], const double a, double dq[2])
{
	double		xy[2], x[2];

	xy[0] = ab[0];
	xy[1] = 0.57735027f * ab[0] + 1.1547005f * ab[1];

	x[0] = sin(a);
	x[1] = cos(a);

	dq[0] = x[1] * xy[0] + x[0] * xy[1];
	dq[1] = -x[0] * xy[0] + x[1] * xy[1];
}

static void
sim(double tend)
{
	const int	shsz = 50;
	double		shb[shsz][2], dsdx[2];
	int		shi = 0;
	double		xs[7], dq[2];
	float		blz[2];
	int		tel = 1, tl, ts = 0;
	FILE		*fd;

	plant_enable();
	blc_enable(plant.tdel);

	fd = fopen("TEL", "w");

	if (fd == NULL) {

		fprintf(stderr, "fopen: %s", strerror(errno));
		exit(1);
	}

	while (plant.tsim < tend) {

		/* Automate script update.
		 * */
		script(plant.tsim);

		/* Plant model update.
		 * */
		plant_update();

		/* ----------------------------- */

		/* Get estimated signals now.
		 * */
		xs[0] = (double) bl.x[0];
		xs[1] = (double) bl.x[1];
		xs[2] = (double) bl.x[2];
		xs[3] = (double) bl.x[3];
		xs[4] = (double) bl.x[4];
		xs[5] = (double) bl.u[0];
		xs[6] = (double) bl.u[1];

		/* BLC update.
		 * */
		blz[0] = (float) (plant.z[0] - 2048) * 0.014663f;
		blz[1] = (float) (plant.z[1] - 2048) * 0.014663f;
		blc_update(blz);

		/* ----------------------------- */

		if (tel) {

			/* Signal shift and differentiation.
			 * */
			dsdx[0] = plant.x[5] - shb[shi][0];
			dsdx[1] = plant.x[6] - shb[shi][1];
			dsdx[0] /= plant.tdel * (double) shsz;
			dsdx[1] /= plant.tdel * (double) shsz;
			shb[shi][0] = plant.x[5];
			shb[shi][1] = plant.x[6];
			shi = (shi < (shsz - 1)) ? shi + 1 : 0;

			/* Base plant telemetry.
			 * */
			fprintf(fd, "%.6lf ", plant.tsim);
			abc2dq(plant.x + 0, plant.x[3], dq);
			fprintf(fd, "%.4lf %.4lf %.4lf %.4lf "
					"%.2lf %.2lf %.2lf ",
					dq[0], dq[1],
					plant.x[2], plant.x[3],
					plant.x[4], dsdx[0], dsdx[1]);

			/* Estimated variables.
			 * */
			fprintf(fd, "%.4f %.4f %.4f %.4f %.4f "
					" %.4f %.4f %i ",
					xs[0], xs[1], xs[2],
					xs[3], xs[4], xs[5],
					xs[6], bl.noft);
			fprintf(fd, "%.4f %.4f %.4f %.6f %.4f "
					"%.4f %.6f ",
					bl.c.aD, bl.c.cD,
					bl.c.R, 1.0f / bl.c.iL, bl.c.E,
					bl.c.U, 1.0f / bl.c.iJ);
			fprintf(fd, "%.4f %.4f %.4f %.4f %.4f ",
					bl.e[0], bl.e[1], bl.e[2],
					bl.e[3], bl.e[4]);
			fputs("\n", fd);
		}

		/* ----------------------------- */

		tl = ts;
		ts = (int) (plant.tsim * 1e+1);

		if (tl < ts) {

			printf("\rtsim = %2.1lf", plant.tsim);
			fflush(stdout);
		}
	}

	fclose(fd);
	puts("\n");
}

int main(int argc, char *argv[])
{
	double		tend = 3.0;

	lib_enable();
	sim(tend);

	return 0;
}

