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
	plant.u[0] = (double) dc[0];
	plant.u[1] = (double) dc[1];
	plant.u[2] = (double) dc[2];
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
	else {

		bl.ccl.sp = 5.0f;
	}
}

static void
abc2dq(const double ab[2], const double x, double dq[2])
{
	double		xy[2], r[2];

	xy[0] = ab[0];
	xy[1] = 0.57735027f * ab[0] + 1.1547005f * ab[1];

	r[0] = sin(x);
	r[1] = cos(x);

	dq[0] = r[1] * xy[0] + r[0] * xy[1];
	dq[1] = r[1] * xy[1] - r[0] * xy[0];
}

static void
sim(double tend)
{
	double		dqx[2];
	float		blz[3];
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

		/* BLC update.
		 * */
		blz[0] = (float) (plant.z[0] - 2048) * 1.464843e-02;
		blz[1] = (float) (plant.z[1] - 2048) * 1.464843e-02;
		blz[2] = (float) plant.z[2] * 7.250976e-03;
		blc_update(blz, blz[2]);

		if (tel) {

			/* Base plant telemetry.
			 * */
			fprintf(fd, "%.6lf ", plant.tsim);
			abc2dq(plant.x, plant.x[3], dqx);
			fprintf(fd, "%.4lf %.4lf %.4lf %.4lf "
					"%.4lf %.4lf %.4lf ",
					dqx[0], dqx[1],
					plant.x[2], plant.x[3],
					plant.x[4], plant.x[5],
					plant.x[6]);

			/* Estimated variables.
			 * */
			fprintf(fd, "%.4f %.4f %.4f %.4f %.4f "
					" %.4f %.4f %i ",
					bl.x[0], bl.x[1], bl.x[2],
					bl.x[3], bl.x[4], bl.u[0],
					bl.u[1], bl.noft);
			fprintf(fd, "%.6f %.6f %.6f %.6f %.6f "
					"%.6f %.6f ",
					bl.c.aD, bl.c.cD,
					1e+3 * bl.c.R, 1e+6 / bl.c.iL,
					1e+6 * bl.c.E, bl.c.U,
					1e+6 / bl.c.iJ);

			/* External.
			 * */
			fprintf(fd, "%.4lf %.4lf %.4lf %.4lf ",
					plant.x[0],
					-(plant.x[0] + plant.x[1]),
					plant.u[0] - 0.5,
					plant.u[2] - 0.5);

			fputs("\n", fd);

		}

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
	double		tend = 5.0;

	lib_enable();
	sim(tend);

	return 0;
}

