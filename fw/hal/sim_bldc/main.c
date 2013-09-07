#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "lib.h"
#include "plant.h"

int main(int argc, char *argv[])
{
	double		tend;
	int		tl, ts = 0;
	FILE		*tel;

	tend = 5.0;

	lib_enable();
	plant_enable();

	tel = fopen("TEL", "w");

	while (plant.tsim < tend) {

		plant_update();

		/* ----------------------------- */

		// FIXME:
		plant.u[0] = 0.5 + 1e-2 * sin(4e+1 * plant.tsim);
		plant.u[1] = 0.5 + 1e-2 * sin(4e+1 * plant.tsim - 2.0 * M_PI / 3.0);
		plant.u[2] = 0.5 + 1e-2 * sin(4e+1 * plant.tsim + 2.0 * M_PI / 3.0);

		/* ----------------------------- */

		fprintf(tel, "%2.6lf ", plant.tsim);
		fprintf(tel, "%2.3lf %2.3lf %5.4lf %1.4lf %2.2lf ",
				plant.x[0], plant.x[1],
				plant.x[2], plant.x[3], plant.x[4]);
		fprintf(tel, "%i %i ",
				plant.z[0], plant.z[1]);
		fputs("\n", tel);

		/* ----------------------------- */

		tl = ts;
                ts = (int) (plant.tsim * 1e+1);

                if (tl < ts) {

                        printf("\rtsim = %2.1lf", plant.tsim);
                        fflush(stdout);
                }
	}

	fclose(tel);
	puts("\n");

	return 0;
}

