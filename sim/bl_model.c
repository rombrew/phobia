/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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
#include <math.h>

#include "bl_model.h"
#include "lib.h"

void blm_AB_DQ(double R, double A, double B, double *D, double *Q)
{
	double		X, Y, rS, rC;

	X = A;
	Y = .577350269189626 * A + 1.15470053837925 * B;

	rS = sin(R);
	rC = cos(R);

	*D = rC * X + rS * Y;
	*Q = rC * Y - rS * X;
}

void blm_DQ_AB(double R, double D, double Q, double *A, double *B)
{
	double		X, Y, rS, rC;

	rS = sin(R);
	rC = cos(R);

	X = rC * D - rS * Q;
	Y = rS * D + rC * Q;

	*A = X;
	*B = - .5 * X + .866025403784439 * Y;
}

void blm_Enable(blm_t *m)
{
	double		Kv;

	m->Tsim = 0.; /* Simulation time (Second) */
        m->dT = 1. / 60E+3; /* Time delta */
	m->sT = 5E-6; /* Solver step */
	m->PWM_resolution = 1400; /* PWM resolution */

	m->sF[0] = 0;
	m->sF[1] = 0;
	m->sF[2] = 0;

        m->X[0] = 0.; /* Axis D current (Ampere) */
	m->X[1] = 0.; /* Axis Q current (Ampere) */
        m->X[2] = 0.; /* Electrical Speed (Radian/Sec) */
        m->X[3] = 0.; /* Electrical Position (Radian) */
        m->X[4] = 20.; /* Temperature (Celsius) */

	/* Winding resistance. (Ohm)
         * */
	m->R = 22E-3;

	/* Iron loss resistance. (Ohm)
	 * */
	m->Q = 11.;

	/* Winding inductance. (Henry)
         * */
	m->Ld = 7E-6;
	m->Lq = 11E-6;

	/* Source voltage. (Volt)
	 * */
	m->U = 28.;

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 14;

	/* BEMF constant. (Weber)
         * */
	Kv = 280.; /* Total RPM per Volt */
        m->E = 60. / 2. / M_PI / sqrt(3.) / (Kv * m->Zp);

	/* Moment of inertia.
	 * */
	m->J = 1E-4;

	/* Load torque constants.
	 * */
	m->M[0] = 2E-3;
	m->M[1] = 0E-5;
	m->M[2] = 5E-1;
	m->M[3] = 0E-3;
}

static void
blm_DQ_Equation(const blm_t *m, const double X[], double dX[])
{
	double		UA, UB, UD, UQ, Q;
	double		R1, E1, MT, ML, W;

	/* Thermal drift.
	 * */
	R1 = m->R  * (1. + 4.28E-3 * (X[4] - 20.));
	E1 = m->E  * (1. - 1.21E-3 * (X[4] - 20.));

	/* BEMF waveform.
	 * */
	E1 *= 1. + sin(X[3] * 2.) * 0E-2 + cos(X[3] * 1.) * 0E-2 + sin(X[3] * 6.) * 0E-2;

	/* Voltage from VSI.
	 * */
	Q = (m->sF[0] + m->sF[1] + m->sF[2]) / 3.;
	UA = (m->sF[0] - Q) * m->U;
	UB = (m->sF[1] - Q) * m->U;

	blm_AB_DQ(X[3], UA, UB, &UD, &UQ);

	/* Electrical equations.
	 * */
	UD += - R1 * X[0] + m->Lq * X[2] * X[1];
	UQ += - R1 * X[1] - m->Ld * X[2] * X[0] - E1 * X[2];

	dX[0] = UD / m->Ld;
	dX[1] = UQ / m->Lq;

	/* Torque production.
	 * */
	MT = 1.5 * m->Zp * (E1 - (m->Lq - m->Ld) * X[0]) * X[1];

	/* Load.
	 * */
	W = fabs(X[2] / m->Zp);
	ML = m->M[0] + W * (m->M[1] + W * m->M[2])
		+ m->M[3] * sin(X[3] * 6.);
	ML = (X[2] < 0.) ? ML : - ML;

	/* Mechanical equations.
	 * */
	dX[2] = m->Zp * (MT + ML) / m->J;
	dX[3] = X[2];

	/* Thermal equation.
	 * */
	dX[4] = 0.;
}

static void
blm_Solve(blm_t *m, double dT)
{
	double		s1[5], s2[5], x2[5];
	int		j;

	/* Second-order ODE solver.
	 * */

	blm_DQ_Equation(m, m->X, s1);

	for (j = 0; j < 5; ++j)
		x2[j] = m->X[j] + s1[j] * dT;

	blm_DQ_Equation(m, x2, s2);

	for (j = 0; j < 5; ++j)
		m->X[j] += (s1[j] + s2[j]) * dT / 2.;
}

static void
blm_Solve_Split(blm_t *m, double dT)
{
	double		sT = m->sT;

	/* Split the long interval.
	 * */
	while (dT > sT) {

		blm_Solve(m, sT);
		dT -= sT;
	}

	blm_Solve(m, dT);

	/* Wrap the angular position.
	 * */
	m->X[3] = (m->X[3] < - M_PI)
		? m->X[3] + 2. * M_PI : (m->X[3] > M_PI)
		? m->X[3] - 2. * M_PI : m->X[3];
}

static void
blm_Bridge_Sample(blm_t *m)
{
	double		S1, S2, U, Uref, dU;
	int		ADC;

	/* ADC reference voltage.
	 * */
	Uref = 3.3;

	/* Current sampling.
	 * */
	blm_DQ_AB(m->X[3], m->X[0], m->X[1], &S1, &S2);

	/* Output voltage of the current sensor A.
	 * */
	U = S1 * 30E-3 + Uref / 2.;
	dU = libGauss() * 3E-3 + 17E-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->sensor_A = (ADC - 2048) * 2.6855E-2;

	/* Output voltage of the current sensor B.
	 * */
	U = S2 * 30E-3 + Uref / 2.;
	dU = libGauss() * 3E-3 - 11E-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->sensor_B = (ADC - 2048) * 2.6855E-2;

	/* Voltage sampling.
	 * */
	S1 = m->U;

	U = S1 * 27. / (470. + 27.);
	dU = libGauss() * 3E-3 + 0E-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->supply_U = ADC * 1.4830E-2;
}

static void
blm_Bridge_Solve(blm_t *m)
{
	int		temp, Ton[3], pm[3];
	double		dPWM, dT;

	dPWM = m->dT / m->PWM_resolution / 2.;
	Ton[0] = (m->uA < 0) ? 0 : (m->uA > m->PWM_resolution) ? m->PWM_resolution : m->uA;
	Ton[1] = (m->uB < 0) ? 0 : (m->uB > m->PWM_resolution) ? m->PWM_resolution : m->uB;
	Ton[2] = (m->uC < 0) ? 0 : (m->uC > m->PWM_resolution) ? m->PWM_resolution : m->uC;

	/* Sort Ton values.
	 * */
	pm[0] = 0;
	pm[1] = 1;
	pm[2] = 2;

	if (Ton[pm[0]] < Ton[pm[2]]) {

		temp = pm[2];
		pm[2] = pm[0];
		pm[0] = temp;
	}

	if (Ton[pm[0]] < Ton[pm[1]]) {

		temp = pm[1];
		pm[1] = pm[0];
		pm[0] = temp;
	}

	if (Ton[pm[1]] < Ton[pm[2]]) {

		temp = pm[2];
		pm[2] = pm[1];
		pm[1] = temp;
	}

	/* Count Up.
	 * */
	dT = dPWM * (m->PWM_resolution - Ton[pm[0]]);
	blm_Solve_Split(m, dT);

	m->sF[pm[0]] = 1;

	dT = dPWM * (Ton[pm[0]] - Ton[pm[1]]);
	blm_Solve_Split(m, dT);

	m->sF[pm[1]] = 1;

	dT = dPWM * (Ton[pm[1]] - Ton[pm[2]]);
	blm_Solve_Split(m, dT);

	m->sF[pm[2]] = 1;

	dT = dPWM * (Ton[pm[2]]);
	blm_Solve_Split(m, dT);

	/* Count Down.
	 * */
	blm_Solve_Split(m, dT);

	m->sF[pm[2]] = 0;

	dT = dPWM * (Ton[pm[1]] - Ton[pm[2]]);
	blm_Solve_Split(m, dT);

	m->sF[pm[1]] = 0;

	dT = dPWM * (Ton[pm[0]] - Ton[pm[1]]);
	blm_Solve_Split(m, dT);

	m->sF[pm[0]] = 0;

	dT = dPWM * (m->PWM_resolution - Ton[pm[0]]);
	blm_Solve_Split(m, dT);
}

void blm_Update(blm_t *m)
{
	blm_Bridge_Sample(m);
	blm_Bridge_Solve(m);

	m->Tsim += m->dT;
}

