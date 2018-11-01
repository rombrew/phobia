/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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
        m->dT = 1. / 30000.; /* PWM period */
	m->sT = 5E-6; /* Solver step */
	m->PWM_R = 2800; /* PWM resolution */

	m->VSI[0] = 0;
	m->VSI[1] = 0;
	m->VSI[2] = 0;

        m->X[0] = 0.; /* Axis D current (Ampere) */
	m->X[1] = 0.; /* Axis Q current (Ampere) */
        m->X[2] = 0.; /* Electrical Speed (Radian/Sec) */
        m->X[3] = 0.; /* Electrical Position (Radian) */
        m->X[4] = 20.; /* Temperature (Celsius) */

	m->X[5] = 0.; /* Current Sensor A */
	m->X[6] = 0.; /* Current Sensor B */
	m->X[7] = 0.; /* Voltage Sensor A */
	m->X[8] = 0.; /* Voltage Sensor B */
	m->X[9] = 0.; /* Voltage Sensor C */

	/* Winding resistance. (Ohm)
         * */
	m->R = 48E-3;

	/* Iron loss resistance. (Ohm)
	 * */
	m->Q = 11.;

	/* Winding inductance. (Henry)
         * */
	m->Ld = 25E-6;
	m->Lq = 35E-6;

	/* Source voltage. (Volt)
	 * */
	m->U = 42.;

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 7;

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
	m->M[2] = 1E-7;
	m->M[3] = 0E-3;

	/* ADC conversion time.
	 * */
	m->T_ADC = 0.714e-6;

	/* Sensor time constant.
	 * */
	m->tau_I = 0.6E-6;
	m->tau_U = 55E-6;
}

static void
blm_DQ_Equation(const blm_t *m, const double X[], double dX[])
{
	double		UA, UB, UD, UQ, Q;
	double		R1, E1, MT, ML, W;

	/* Thermal drift.
	 * */
	R1 = m->R  * (1. + 4E-3 * (X[4] - 20.));
	E1 = m->E  * (1. - 1E-3 * (X[4] - 20.));

	/* BEMF waveform distortion.
	 * */
	E1 *= 1. + sin(X[3] * 3.) * 0E-2;

	/* Voltage from VSI.
	 * */
	Q = (m->VSI[0] + m->VSI[1] + m->VSI[2]) / 3.;
	UA = (m->VSI[0] - Q) * m->U;
	UB = (m->VSI[1] - Q) * m->U;

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
	double		S1[5], S2[5], X2[5], A, B;
	int		j;

	/* Second-order ODE solver.
	 * */

	blm_DQ_Equation(m, m->X, S1);

	for (j = 0; j < 5; ++j)
		X2[j] = m->X[j] + S1[j] * dT;

	blm_DQ_Equation(m, X2, S2);

	for (j = 0; j < 5; ++j)
		m->X[j] += (S1[j] + S2[j]) * dT / 2.;

	/* Sensor transient (exact solution).
	 * */
	blm_DQ_AB(m->X[3], m->X[0], m->X[1], &A, &B);

	m->X[5] += (A - m->X[5]) * (1. - exp(- dT / m->tau_I));
	m->X[6] += (B - m->X[6]) * (1. - exp(- dT / m->tau_I));

	m->X[7] += (m->VSI[0] - m->X[7]) * (1. - exp(- dT / m->tau_U));
	m->X[8] += (m->VSI[1] - m->X[8]) * (1. - exp(- dT / m->tau_U));
	m->X[9] += (m->VSI[2] - m->X[9]) * (1. - exp(- dT / m->tau_U));
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
	m->X[3] = (m->X[3] < - M_PI) ? m->X[3] + 2. * M_PI :
		(m->X[3] > M_PI) ? m->X[3] - 2. * M_PI : m->X[3];
}

static int
blm_ADC(double u)
{
	int		ADC;

	u += lib_gauss() * 7E-4;

	ADC = (int) (u * 4096);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;

	return ADC;
}

static void
blm_VSI_Sample(blm_t *m, int N)
{
	const double	range_I = 55.;
	const double	range_U = 60.;

	int		ADC;

	if (N == 0) {

		ADC = blm_ADC(m->X[5] / 2. / range_I + .5);
		m->ADC_IA = (ADC - 2047) * range_I / 2048.;

		ADC = blm_ADC(m->X[6] / 2. / range_I + .5);
		m->ADC_IB = (ADC - 2047) * range_I / 2048.;
	}
	else if (N == 1) {

		ADC = blm_ADC(m->U / range_U);
		m->ADC_US = ADC * range_U / 4096.;

		ADC = blm_ADC(m->X[7] / range_U);
		m->ADC_UA = ADC * range_U / 4096.;
	}
	else if (N == 2) {

		ADC = blm_ADC(m->X[8] / range_U);
		m->ADC_UB = ADC * range_U / 4096.;

		ADC = blm_ADC(m->X[9] / range_U);
		m->ADC_UC = ADC * range_U / 4096.;
	}
}

static void
blm_VSI_Solve(blm_t *m)
{
	int		Tev[5], pm[5], n, k, tmp;
	double		tTIM, dT;

	tTIM = m->dT / m->PWM_R / 2.;

	/* ADC sampling.
	 * */
	Tev[0] = m->PWM_R - (int) (m->T_ADC / tTIM);
	Tev[1] = m->PWM_R - (int) (2. * m->T_ADC / tTIM);

	/* FETs switching.
	 * */
	Tev[2] = (m->PWM_A < 0) ? 0 : (m->PWM_A > m->PWM_R) ? m->PWM_R : m->PWM_A;
	Tev[3] = (m->PWM_B < 0) ? 0 : (m->PWM_B > m->PWM_R) ? m->PWM_R : m->PWM_B;
	Tev[4] = (m->PWM_C < 0) ? 0 : (m->PWM_C > m->PWM_R) ? m->PWM_R : m->PWM_C;

	for (n = 0; n < 5; ++n)
		pm[n] = n;

	/* Get sorted events.
	 * */
	for (n = 0; n < 5; ++n) {

		for (k = n + 1; k < 5; ++k) {

			if (Tev[pm[n]] < Tev[pm[k]]) {

				tmp = pm[n];
				pm[n] = pm[k];
				pm[k] = tmp;
			}
		}
	}

	/* Count Up.
	 * */
	blm_VSI_Sample(m, 0);

	tmp = m->PWM_R;

	for (n = 0; n < 5; ++n) {

		dT = tTIM * (tmp - Tev[pm[n]]);
		blm_Solve_Split(m, dT);

		if (pm[n] < 2) {

			blm_VSI_Sample(m, pm[n]);
		}
		else {
			m->VSI[pm[n] - 2] = 1;

			/* Distortion.
			 * */
			m->X[5] = 0.;
			m->X[6] = 0.;
		}

		tmp = Tev[pm[n]];
	}

	dT = tTIM * (Tev[pm[4]]);
	blm_Solve_Split(m, dT);

	/* Keep only three of them.
	 * */
	for (n = 0, k = 0; n < 5; ++n) {

		tmp = pm[n];

		if (tmp != 0 && tmp != 1) {

			pm[k++] = tmp;
		}
	}

	/* Count Down.
	 * */
	for (n = 2, tmp = 0; n >= 0; --n) {

		dT = tTIM * (Tev[pm[n]] - tmp);
		blm_Solve_Split(m, dT);

		m->VSI[pm[n] - 2] = 0;

		/* Distortion.
		 * */
		m->X[5] = 0.;
		m->X[6] = 0.;

		tmp = Tev[pm[n]];
	}

	dT = tTIM * (m->PWM_R - Tev[pm[0]]);
	blm_Solve_Split(m, dT);
}

void blm_Update(blm_t *m)
{
	blm_VSI_Solve(m);
	m->Tsim += m->dT;
}

