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
test(double t)
{
	if (t < 0.1) {

		bl.mode = BLC_MODE_IDLE;
	}
	else if (t < 1.0) {

		bl.mode = BLC_MODE_ALIGN;
	}
	else {

		bl.mode = BLC_MODE_RUN;
	}
}

static void
sim(double tend)
{
	double		x5;
	int		tel = 1, tl, ts = 0;
	FILE		*fd;

	plant_enable();
	blc_enable(plant.tdel);

	fd = fopen("TEL", "w");

	if (fd == NULL) {

		printf("fopen: %s", strerror(errno));
		exit(1);
	}

	while (plant.tsim < tend) {

		/* Automate test update.
		 * */
		test(plant.tsim);

		/* Plant model update.
		 * */
		x5 = plant.x[5];
		plant_update();

		/* ----------------------------- */

		/* BLC update.
		 * */
		blc_update(plant.z);

		/* ----------------------------- */

		if (tel) {

			fprintf(fd, "%2.6lf ", plant.tsim);
			fprintf(fd, "%2.4lf %2.4lf %2.4lf %5.4lf %1.4lf %2.2lf %3.4lf ",
					plant.x[0], - plant.x[0] - plant.x[1],
					plant.x[1], plant.x[2], plant.x[3], plant.x[4],
					(plant.x[5] - x5) / plant.tdel);
			fprintf(fd, "%1.4lf %1.4lf %1.4lf ",
				plant.u[0], plant.u[1], plant.u[2]);
			fprintf(fd, "%i %i ",
					plant.z[0], plant.z[1]);
			fprintf(fd, "%f %f ",
					bl.c.aD, bl.c.cD);
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

