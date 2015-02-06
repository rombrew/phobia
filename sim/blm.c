/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#include "blm.h"
#include "lib.h"

static double
blmBEMFShape(double x)
{
	double		s1;

	/* Sinusoidal shape.
	 * */
	s1 = - (sin(x) + sin(x * 3.) * 4e-2);

	return s1;
}

void blmEnable(blm_t *m)
{
	double		Kv;

	m->Tsim = 0.; /* Simulation time (Second) */
        m->dT = 1. / 20e+3; /* Time delta */
	m->sT = 50e-6; /* Solver step */
	m->PWMR = 700; /* PWM resolution */

	m->sF[0] = 1;
	m->sF[1] = 1;
	m->sF[2] = 1;

        m->X[0] = 0.; /* Phase A current (Ampere) */
	m->X[1] = 0.; /* Phase B current (Ampere) */
        m->X[2] = 0.; /* Electrical Speed (Radian/Sec) */
        m->X[3] = -1.; /* Electrical Position (Radian) */
        m->X[4] = 20.; /* Temperature (Celsius) */

	/* Winding resistance. (Ohm)
         * */
	m->R = 74e-3;

	/* Iron loss resistance. (Ohm)
	 * */
	m->Q = 18.;

	/* Winding inductance. (Henry)
         * */
	m->L = 45e-6;

	/* Source voltage. (Volt)
	 * */
	m->U = 12.;

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 11;

	/* BEMF constant. (Weber)
         * */
	Kv = 650.; /* Total RPM per Volt */
        m->E = 60. / 4. / M_PI / (Kv * m->Zp);

	/* Moment of inertia.
	 * */
	m->J = 10e-5;

	/* Load torque constants.
	 * */
	m->M[0] = 1e-3;
	m->M[1] = 0e-0;
	m->M[2] = 1e-7;
	m->M[3] = 0e-0;
}

static void
blmEquation(const blm_t *m, const double X[7], double dX[7])
{
	double		EA, EB, EC, IA, IB, IC;
	double		BEMFA, BEMFB, BEMFC;
	double		R, L, E, Uz, Mt, Ml, w;

	R = m->R * (1. + 4.28e-3 * (X[4] - 20.));
	L = m->L * (1. - 0.11e-3 * (X[4] - 20.));
	E = m->E * (1. - 1.21e-3 * (X[4] - 20.));

	EA = blmBEMFShape(X[3]);
	EB = blmBEMFShape(X[3] - 2. * M_PI / 3.);
	EC = blmBEMFShape(X[3] + 2. * M_PI / 3.);

	BEMFA = X[2] * E * EA;
	BEMFB = X[2] * E * EB;
	BEMFC = X[2] * E * EC;

	Uz = (m->sF[0] + m->sF[1] + m->sF[2]) * m->U / 3.
		- (BEMFA + BEMFB + BEMFC) / 3.;

	/* Electrical equations.
	 * */
	dX[0] = ((m->sF[0] * m->U - Uz) - X[0] * R - BEMFA) / L;
	dX[1] = ((m->sF[1] * m->U - Uz) - X[1] * R - BEMFB) / L;

	IA = X[0] - BEMFA / m->Q;
	IB = X[1] - BEMFB / m->Q;
	IC = -(X[0] + X[1]) - BEMFC / m->Q;

	Mt = m->Zp * m->E * (EA * IA + EB * IB + EC * IC);

	w = fabs(X[2] / m->Zp);
	Ml = m->M[0] + m->M[1] * w
		+ m->M[2] * w * w
		+ m->M[3] * w * w * w;
	Ml = (X[2] < 0. ? Ml : -Ml);

	/* Mechanical equations.
	 * */
	dX[2] = m->Zp * (Mt + Ml) / m->J;
	dX[3] = X[2];

	/* Thermal equation.
	 * */
	dX[4] = 0.;
}

static void
blmSolve(blm_t *m, double dT)
{
	double		s1[7], s2[7], x2[7];
	int		j;

	/* Second-order ODE solver.
	 * */

	blmEquation(m, m->X, s1);

	for (j = 0; j < 7; ++j)
		x2[j] = m->X[j] + s1[j] * dT;

	blmEquation(m, x2, s2);

	for (j = 0; j < 7; ++j)
		m->X[j] += (s1[j] + s2[j]) * dT / 2.;
}

static void
blmSolveSplit(blm_t *m, double dT)
{
	double		sT = m->sT;

	/* Split the long interval.
	 * */
	while (dT > sT) {

		blmSolve(m, sT);
		dT -= sT;
	}

	blmSolve(m, dT);

	/* Wrap the angular position.
	 * */
	m->X[3] = (m->X[3] < -M_PI)
		? m->X[3] + 2. * M_PI : (m->X[3] > M_PI)
		? m->X[3] - 2. * M_PI : m->X[3];
}

static void
blmBridgeSample(blm_t *m)
{
	double		S1, S2, U, Uref, dU;
	int		ADC;

	/* ADC reference voltage.
	 * */
	Uref = 3.3;

	/* Current sampling.
	 * */
	S1 = m->X[0];
	S2 = m->X[1];

	/* Output voltage of the current sensor A.
	 * */
	U = S1 * 55e-3 + Uref / 2.;
	dU = libGauss() * 3e-3 + 27e-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->xA = ADC;

	/* Output voltage of the current sensor B.
	 * */
	U = S2 * 55e-3 + Uref / 2.;
	dU = libGauss() * 3e-3 - 11e-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->xB = ADC;

	/* Voltage sampling.
	 * */
	S1 = m->U;

	U = S1 / 9.;
	dU = libGauss() * 3e-3 + 0e-3;
	U += dU;

	/* ADC conversion.
	 * */
	ADC = (int) (U / Uref * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;
	m->xU = ADC;
}

static void
blmBridgeSolve(blm_t *m)
{
	int		j, Ton[3], pm[3];
	double		dPWM, dT;

	dPWM = m->dT / m->PWMR / 2.;
	Ton[0] = (m->uA < 0) ? 0 : (m->uA > m->PWMR) ? m->PWMR : m->uA;
	Ton[1] = (m->uB < 0) ? 0 : (m->uB > m->PWMR) ? m->PWMR : m->uB;
	Ton[2] = (m->uC < 0) ? 0 : (m->uC > m->PWMR) ? m->PWMR : m->uC;

	/* Sort Ton values.
	 * */
	pm[0] = 0;
	pm[1] = 1;
	pm[2] = 2;

	if (Ton[pm[0]] < Ton[pm[2]]) {

		j = pm[2];
		pm[2] = pm[0];
		pm[0] = j;
	}

	if (Ton[pm[0]] < Ton[pm[1]]) {

		j = pm[1];
		pm[1] = pm[0];
		pm[0] = j;
	}

	if (Ton[pm[1]] < Ton[pm[2]]) {

		j = pm[2];
		pm[2] = pm[1];
		pm[1] = j;
	}

	/* Count Up.
	 * */
	dT = dPWM * (Ton[pm[0]]);
	blmSolveSplit(m, dT);

	m->sF[pm[0]] = 0;

	dT = dPWM * (Ton[pm[1]] - Ton[pm[0]]);
	blmSolveSplit(m, dT);

	m->sF[pm[1]] = 0;

	dT = dPWM * (Ton[pm[2]] - Ton[pm[1]]);
	blmSolveSplit(m, dT);

	m->sF[pm[2]] = 0;

	dT = dPWM * (m->PWMR - Ton[pm[2]]);
	blmSolveSplit(m, dT);

	/* Count Down.
	 * */
	blmSolveSplit(m, dT);

	m->sF[pm[2]] = 1;

	dT = dPWM * (Ton[pm[2]] - Ton[pm[1]]);
	blmSolveSplit(m, dT);

	m->sF[pm[1]] = 1;

	dT = dPWM * (Ton[pm[1]] - Ton[pm[0]]);
	blmSolveSplit(m, dT);

	m->sF[pm[0]] = 1;

	dT = dPWM * (Ton[pm[0]]);
	blmSolveSplit(m, dT);
}

void blmUpdate(blm_t *m)
{
	blmBridgeSample(m);
	blmBridgeSolve(m);

	m->Tsim += m->dT;
}

