/*
   Phobia Motor Controller for RC and robotics.
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
blm_BEMF_Shape(double x)
{
	double		s1;

	/* Almost sinusoidal shape.
	 * */
	s1 = - (sin(x) + sin(x * 5.) * 1e-2);

	return s1;
}

void blm_Enable(blm_t *m)
{
	double		Kv;

	m->Tsim = 0.; /* Simulation time (Second) */
        m->dT = 1. / 40e+3; /* Time delta */
	m->sT = 5e-6; /* Solver step */
	m->PWMR = 2100; /* PWM resolution */
	m->mDQ = 1; /* Saliency model */

	m->sF[0] = 0;
	m->sF[1] = 0;
	m->sF[2] = 0;

        m->X[0] = 0.; /* Phase A/D current (Ampere) */
	m->X[1] = 0.; /* Phase B/Q current (Ampere) */
        m->X[2] = 0.; /* Electrical Speed (Radian/Sec) */
        m->X[3] = -1.; /* Electrical Position (Radian) */
        m->X[4] = 20.; /* Temperature (Celsius) */

	/* Winding resistance. (Ohm)
         * */
	m->R = 175e-3;

	/* Iron loss resistance. (Ohm)
	 * */
	m->Q = 18.;

	/* Winding inductance. (Henry)
         * */
	m->L = 25e-6;

	/* Source voltage. (Volt)
	 * */
	m->U = 12.;

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 11;

	/* BEMF constant. (Weber)
         * */
	Kv = 650.; /* Total RPM per Volt */
        m->E = 60. / 2. / M_PI / sqrt(3.) / (Kv * m->Zp);

	/* Moment of inertia.
	 * */
	m->J = 2e-5;

	/* Load torque constants.
	 * */
	m->M[0] = 1e-3;
	m->M[1] = 0e-0;
	m->M[2] = 2e-9;
	m->M[3] = 0e-0;

	/* D/Q inductance. (Henry)
         * */
	m->Ld = 19e-6;
	m->Lq = 22e-6;
}

static void
blm_AB_Equation(const blm_t *m, const double X[], double dX[])
{
	double		EA, EB, EC, IA, IB, IC;
	double		BEMFA, BEMFB, BEMFC;
	double		R, E, Q, Mt, Ml, w;

	R = m->R  * (1. + 4.28e-3 * (X[4] - 20.));
	E = m->E  * (1. - 1.21e-3 * (X[4] - 20.));

	EA = blm_BEMF_Shape(X[3]);
	EB = blm_BEMF_Shape(X[3] - 2. * M_PI / 3.);
	EC = blm_BEMF_Shape(X[3] + 2. * M_PI / 3.);

	BEMFA = X[2] * E * EA;
	BEMFB = X[2] * E * EB;
	BEMFC = X[2] * E * EC;

	Q = (m->sF[0] + m->sF[1] + m->sF[2]) * m->U / 3.
		- (BEMFA + BEMFB + BEMFC) / 3.;

	/* Electrical equations.
	 * */
	dX[0] = ((m->sF[0] * m->U - Q) - X[0] * R - BEMFA) / m->L;
	dX[1] = ((m->sF[1] * m->U - Q) - X[1] * R - BEMFB) / m->L;

	IA = X[0] - BEMFA / m->Q;
	IB = X[1] - BEMFB / m->Q;
	IC = -(X[0] + X[1]) - BEMFC / m->Q;

	Mt = m->Zp * E * (EA * IA + EB * IB + EC * IC);

	w = fabs(X[2] / m->Zp);
	Ml = m->M[0] + m->M[1] * w + m->M[2] * w * w + m->M[3] * sin(X[3] * 3.);
	Ml = (X[2] < 0. ? Ml : - Ml);

	/* Mechanical equations.
	 * */
	dX[2] = m->Zp * (Mt + Ml) / m->J;
	dX[3] = X[2];

	/* Thermal equation.
	 * */
	dX[4] = 0.;
}

static void
blm_DQ_Equation(const blm_t *m, const double X[], double dX[])
{
	double		uA, uB, uX, uY, uD, uQ;
	double		R, E, Q, Mt, Ml, w;

	R = m->R  * (1. + 4.28e-3 * (X[4] - 20.));
	E = m->E  * (1. - 1.21e-3 * (X[4] - 20.));

	Q = (m->sF[0] + m->sF[1] + m->sF[2]) / 3.;
	uA = (m->sF[0] - Q) * m->U;
	uB = (m->sF[1] - Q) * m->U;

	uX = uA;
	uY = .57735027f * uA + 1.1547005f * uB;

	uD = cos(X[3]) * uX + sin(X[3]) * uY;
	uQ = cos(X[3]) * uY - sin(X[3]) * uX;

	/* Electrical equations.
	 * */
	dX[0] = (uD - R * X[0] + m->Lq * X[2] * X[1]) / m->Ld;
	dX[1] = (uQ - R * X[1] - m->Ld * X[2] * X[0] - E * X[2]) / m->Lq;

	Mt = 1.5f * m->Zp * (E - (m->Lq - m->Ld) * X[0]) * X[1];

	w = fabs(X[2] / m->Zp);
	Ml = m->M[0] + m->M[1] * w + m->M[2] * w * w;
	Ml = (X[2] < 0. ? Ml : - Ml);

	/* Mechanical equations.
	 * */
	dX[2] = m->Zp * (Mt + Ml) / m->J;
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

	if (m->mDQ)

		blm_DQ_Equation(m, m->X, s1);
	else
		blm_AB_Equation(m, m->X, s1);

	for (j = 0; j < 5; ++j)
		x2[j] = m->X[j] + s1[j] * dT;

	if (m->mDQ)

		blm_DQ_Equation(m, x2, s2);
	else
		blm_AB_Equation(m, x2, s2);

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
	Uref = 3.;

	/* Current sampling.
	 * */
	if (m->mDQ) {

		S1 = cos(m->X[3]) * m->X[0] - sin(m->X[3]) * m->X[1];
		S2 = sin(m->X[3]) * m->X[0] + cos(m->X[3]) * m->X[1];
		S2 = - .5f * S1 + .8660254f * S2;
	}
	else {
		S1 = m->X[0];
		S2 = m->X[1];
	}

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
blm_Bridge_Solve(blm_t *m)
{
	int		temp, Ton[3], pm[3];
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
	dT = dPWM * (m->PWMR - Ton[pm[0]]);
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

	dT = dPWM * (m->PWMR - Ton[pm[0]]);
	blm_Solve_Split(m, dT);
}

void blm_Update(blm_t *m)
{
	blm_Bridge_Sample(m);
	blm_Bridge_Solve(m);

	m->Tsim += m->dT;
}

