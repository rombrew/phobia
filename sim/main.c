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
#include "lib.h"

#include "kalf.h"

static void
sim(double tend)
{
	int		tel = 1, tl, ts = 0;
	FILE		*fd;

	plant_enable();
	kalf_enable(plant.tdel);

	fd = fopen("TEL", "w");

	if (fd == NULL) {

		printf("fopen: %s", strerror(errno));
		exit(1);
	}

	while (plant.tsim < tend) {

		/* Plant model update.
		 * */
		plant_update();

		/* ----------------------------- */

		kalf_update(plant.z);

		/* ----------------------------- */

		if (tel) {

			fprintf(fd, "%2.6lf ", plant.tsim);
			fprintf(fd, "%2.3lf %2.3lf %5.4lf %1.4lf %2.2lf %3.4lf ",
					plant.x[0], - plant.x[0] - plant.x[1],
					plant.x[2], plant.x[3], plant.x[4],
					plant.x[5] - plant.x5);
			fprintf(fd, "%i %i ",
					plant.z[0], plant.z[1]);
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
	double		tend = 5.0;

	lib_enable();
	sim(tend);

	return 0;
}

