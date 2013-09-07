#include <stdlib.h>
#include <math.h>

#include "plant.h"
#include "lib.h"

plant_t			plant;

void abc2dq(const double abc[3], const double e[2], double dq[2])
{
	double		xy[2];

	xy[0] = abc[0];
	xy[1] = 0.577350269189626 * abc[0] + 1.15470053837925 * abc[1];

	dq[0] = e[0] * xy[0] + e[1] * xy[1];
	dq[1] = -e[1] * xy[0] + e[0] * xy[1];
}

void dq2abc(const double dq[2], const double e[2], double abc[3])
{
	double		xy[2];

	xy[0] = e[0] * dq[0] - e[1] * dq[1];
	xy[1] = e[1] * dq[0] + e[0] * dq[1];

	abc[0] = xy[0];
	abc[1] = -0.5 * xy[0] + 0.866025403784439 * xy[1];
	abc[2] = -0.5 * xy[0] - 0.866025403784439 * xy[1];
}

static double
plant_bemf_shape(double x)
{
	const double	PI3 = M_PI / 3.0;
	double		s1, s2;

	/* Sinusoidal shape.
	 * */
	s1 = sin(x - M_PI / 2.0);

	/* Trapezoidal shape.
	 * */
	s2 = (x < PI3) ? -1.0 :
		(x < 2.0 * PI3) ? 2.0 / PI3 * x - 3.0:
		(x < 4.0 * PI3) ? 1.0 :
		(x < 5.0 * PI3) ? -2.0 / PI3 * x + 9.0 : -1.0;

	/* Mixed output.
	 * */
	x = s1 + (s2 - s1) * 0.5;

	return x;
}

static double
plant_bemf_shape_lookup(double x)
{
	double		e, a;
	int		j;

	/* Obtain index and scalar.
	 * */
	x *= 180.0 / M_PI;
	j = (int) x;
	a = x - (double) j;

	/* Wrap the index.
	 * */
	j = (j < 0) ? j + 360 : (j > 360)
		? j - 360 : j;

	/* Linear interpolated.
	 * */
	e = plant.tab_E_shape[j];
	e += (plant.tab_E_shape[j + 1] - e) * a;

	return e;
}

void plant_enable()
{
	double		x;
	int		j;

	/* BEMF waveform shape table.
	 * */
	for (j = 0; j < 361; ++j) {

		x = plant_bemf_shape(j * M_PI / 180.0);
		plant.tab_E_shape[j] = x;
	}

	plant.tsim = 0.0; /* Simulation time (Second) */
        plant.tdel = 1.0 / 20e+3; /* Delta */
	plant.pwmf = 1000; /* PWM resolution */

        plant.x[0] = 0.0; /* Phase A current (Ampere) */
	plant.x[1] = 0.0; /* Phase B current (Ampere) */
        plant.x[2] = 0.0; /* Electrical speed (Radian/Sec) */
        plant.x[3] = 0.0; /* Electrical position (Radian) */
        plant.x[4] = 20.0; /* Temperature (Celsius) */

	/* Winding resistance at 20 C. (Ohm)
         * */
	plant.const_R = 15e-3;

	/* L - Winding inductance. (Henry)
         * */
	plant.const_L = 20e-6;

	/* BEMF and Torque constant.
         * */
        plant.const_E = 1.2e-3;

	/* Source voltage. (Volt)
	 * */
	plant.const_U = 12.0;

	/* Number of the rotor pole pairs.
	 * */
	plant.const_Z = 11.0;

	/* Moment of inertia. (Kg/m^2)
	 * */
	plant.const_J = 10e-5;

	/* Load torque constants.
	 * */
	plant.const_M[0] = 0.0;
	plant.const_M[1] = 0.0;
	plant.const_M[2] = 0.0;
	plant.const_M[3] = 0.0;
}

static void
plant_equation(double dx[PLANT_STATE_SIZE],
		const double x[PLANT_STATE_SIZE])
{
	double		R, L, E, U, Z, J;
	double		e[3], bemf[3], Uz, Mt, Ml, s;

	R = plant.const_R * (1.0 + 4.28e-3 * (x[4] - 20.0));
	L = plant.const_L;
	E = plant.const_E;
	U = plant.const_U;
	Z = plant.const_Z;
	J = plant.const_J;

	e[0] = plant_bemf_shape_lookup(x[3]);
	e[1] = plant_bemf_shape_lookup(x[3] - 2.0 * M_PI / 3.0);
	e[2] = plant_bemf_shape_lookup(x[3] + 2.0 * M_PI / 3.0);

	bemf[0] = x[2] * E * e[0];
	bemf[1] = x[2] * E * e[1];
	bemf[2] = x[2] * E * e[2];

	Uz = (plant.u[0] + plant.u[1] + plant.u[2]) * U / 3.0
		- (bemf[0] + bemf[1] + bemf[2]) / 3.0;

	/* Electrical equations.
	 * */
	dx[0] = ((plant.u[0] * U - Uz) - x[0] * R - bemf[0]) / L;
	dx[1] = ((plant.u[1] * U - Uz) - x[1] * R - bemf[1]) / L;

	Mt = Z * E * (x[0] * e[0] + x[1] * e[1]
			- (x[0] + x[1]) * e[2]);

	s = fabs(x[2] / Z);
	Ml = plant.const_M[1] * s
		+ plant.const_M[2] * s * s
		+ plant.const_M[3] * s * s * s;
	Ml = plant.const_M[0] + (x[2] < 0.0 ? Ml : -Ml);

	/* Mechanical equations.
	 * */
	dx[2] = Z * (Mt + Ml) / J;
	dx[3] = x[2];

	/* Thermal equation.
	 * */
	dx[4] = 0.0;
}

static void
plant_solve(double dt)
{
	double		s1[PLANT_STATE_SIZE];
	double		s2[PLANT_STATE_SIZE];
	double		x2[PLANT_STATE_SIZE];
	int		j;

	/* Second-order ODE solver.
	 * */

	plant_equation(s1, plant.x);

	for (j = 0; j < PLANT_STATE_SIZE; ++j)
		x2[j] = plant.x[j] + s1[j] * dt;

	plant_equation(s2, x2);

	for (j = 0; j < PLANT_STATE_SIZE; ++j)
		plant.x[j] += (s1[j] + s2[j]) * dt / 2.0;

	/* Wrap the angular position.
	 * */
	plant.x[3] = (plant.x[3] < -M_PI)
		? plant.x[3] + 2.0 * M_PI : (plant.x[3] > M_PI)
		? plant.x[3] - 2.0 * M_PI : plant.x[3];
}

void plant_update()
{
	int		j, ton[3], a, pm[3];
	double		pwmdt, dt, sa[2], u, uref;

	pwmdt = plant.tdel / plant.pwmf / 2.0;
	uref = 3.3;

	ton[0] = (int) (plant.u[0] * plant.pwmf);
	ton[1] = (int) (plant.u[1] * plant.pwmf);
	ton[2] = (int) (plant.u[2] * plant.pwmf);

	if (1) {

		/* Sort Ton values.
		 * */

		pm[0] = 0;
		pm[1] = 1;
		pm[2] = 2;

		if (ton[pm[0]] < ton[pm[2]]) {

			j = pm[2];
			pm[2] = pm[0];
			pm[0] = j;
		}

		if (ton[pm[0]] < ton[pm[1]]) {

			j = pm[1];
			pm[1] = pm[0];
			pm[0] = j;
		}

		if (ton[pm[1]] < ton[pm[2]]) {

			j = pm[2];
			pm[2] = pm[1];
			pm[1] = j;
		}

		/* Do large steps.
		 * */

		plant.i[pm[0]] = 1;
		plant.i[pm[1]] = 1;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[0]]);
		plant_solve(dt);

		plant.i[pm[0]] = 0;
		plant.i[pm[1]] = 1;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[1]] - ton[pm[0]]);
		plant_solve(dt);

		plant.i[pm[0]] = 0;
		plant.i[pm[1]] = 0;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[2]] - ton[pm[1]]);
		plant_solve(dt);

		plant.i[pm[0]] = 0;
		plant.i[pm[1]] = 0;
		plant.i[pm[2]] = 0;

		dt = pwmdt * (plant.pwmf - ton[pm[2]]);
		plant_solve(dt);

		/* Current sampling.
		 * */
		sa[0] = plant.x[0];
		sa[1] = - plant.x[0] - plant.x[1];

		dt = pwmdt * (plant.pwmf - ton[pm[2]]);
		plant_solve(dt);

		plant.i[pm[0]] = 0;
		plant.i[pm[1]] = 0;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[2]] - ton[pm[1]]);
		plant_solve(dt);

		plant.i[pm[0]] = 0;
		plant.i[pm[1]] = 1;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[1]] - ton[pm[0]]);
		plant_solve(dt);

		plant.i[pm[0]] = 1;
		plant.i[pm[1]] = 1;
		plant.i[pm[2]] = 1;

		dt = pwmdt * (ton[pm[0]]);
		plant_solve(dt);
	}
	else {
		/* Count Up.
		 * */
		for (j = 0; j < plant.pwmf; ++j) {

			plant.i[0] = j < ton[0] ? 1 : 0;
			plant.i[1] = j < ton[1] ? 1 : 0;
			plant.i[2] = j < ton[2] ? 1 : 0;

			plant_solve(pwmdt);
		}

		/* Current sampling.
		 * */
		sa[0] = plant.x[0];
		sa[1] = - plant.x[0] - plant.x[1];

		/* Count Down.
		 * */
		for (j = plant.pwmf; j > 0; --j) {

			plant.i[0] = j > ton[0] ? 0 : 1;
			plant.i[1] = j > ton[1] ? 0 : 1;
			plant.i[2] = j > ton[2] ? 0 : 1;

			plant_solve(pwmdt);
		}
	}

	plant.tsim += plant.tdel;

	/* ADC conversion.
	 * */
	u = sa[0] * 55e-3 + uref / 2.0 + gauss() * 3e-3 + 37e-3;
	a = (int) (u / uref * 4096);
	a = a < 0 ? 0 : a > 4095 ? 4095 : a;
	plant.z[0] = a;

	u = sa[1] * 55e-3 + uref / 2.0 + gauss() * 3e-3 - 11e-3;
	a = (int) (u / uref * 4096);
	a = a < 0 ? 0 : a > 4095 ? 4095 : a;
	plant.z[1] = a;
}

