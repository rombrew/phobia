#include <stddef.h>
#include <math.h>

#include "blm.h"
#include "lfg.h"

void blm_AB_DQ(double theta, double A, double B, double *D, double *Q)
{
	double		X, Y, tS, tC;

	X = A;
	Y = 0.577350269189626 * A + 1.15470053837925 * B;

	tS = sin(theta);
	tC = cos(theta);

	*D = tC * X + tS * Y;
	*Q = tC * Y - tS * X;
}

void blm_DQ_ABC(double theta, double D, double Q, double *A, double *B, double *C)
{
	double		X, Y, tS, tC;

	tS = sin(theta);
	tC = cos(theta);

	X = tC * D - tS * Q;
	Y = tS * D + tC * Q;

	*A = X;
	*B = - 0.5 * X + 0.866025403784439 * Y;
	*C = - 0.5 * X - 0.866025403784439 * Y;
}

double blm_Kv_lambda(blm_t *m, double Kv)
{
	/* Convert the total machine Kv (rpm/v) to the flux linkage (Weber)
	 * */
	return (60. / 2. / M_PI) / sqrt(3.) / (Kv * m->Zp);
}

void blm_enable(blm_t *m)
{
	/* WARNING: All the following parameters are given for example and
	 * should be redefined from the outside in accordance with the task
	 * being solved.
	 */

	m->time = 0.;		/* Simulation TIME (Second) */
	m->sol_dT = 5.E-6;	/* ODE solver step (Second) */

	m->pwm_dT = 35.E-6;		/* PWM cycle (Second)    */
	m->pwm_deadtime = 90.E-9;	/* PWM deadtime (Second) */
	m->pwm_minimal = 50.E-9;	/* PWM minimal (Second)  */
	m->pwm_resolution = 2940;	/* PWM resolution        */

	/* Threshold about deadtime (Ampere).
	 * */
	m->Dtol = 0.1;

	/* Winding resistance (Ohm).
         * */
	m->Rs = 0.011;

	/* Winding inductance (Henry).
         * */
	m->Ld = 11.E-6;
	m->Lq = 16.E-6;

	/* Number of the rotor pole pairs.
	 * */
	m->Zp = 14;

	/* Flux linkage constant (Weber).
         * */
        m->lambda = blm_Kv_lambda(m, 270.);

	/* Ambient temperature (Celsius).
	 * */
	m->Ta = 25.;

	/* Thermal capacity (Joule/Kelvin).
	 * */
	m->Ct = 20.;

	/* Thermal resistance to ambient (Kelvin/Watt).
	 * */
	m->Rt = 0.5;

	/* DC link voltage (Volt).
	 * */
	m->Udc = 48.;

	/* DC link internal resistance (Ohm).
	 * */
	m->Rdc = 0.2;

	/* Decoupling capacitance (Farad).
	 * */
	m->Cdc = 940.E-6;

	/* Moment of inertia (Kilogram Metre squared).
	 * */
	m->Jm = 5.E-3;

	/* Load torque polynom.
	 * */
	m->Mq[0] = 0.E-3;
	m->Mq[1] = 5.E-4;
	m->Mq[2] = 5.E-7;
	m->Mq[3] = 5.E-2;

	/* ADC conversion time (Second).
	 * */
	m->adc_Tconv = 0.643E-6;

	/* ADC sample offset (Second).
	 * */
	m->adc_Toffset = 0.2E-6;

	/* Sensor time constant (Second).
	 * */
	m->tau_A = 0.636E-6;
	m->tau_B = 5.940E-6;

	/* Sensor measurement range.
	 * */
	m->range_A = 165.;	/* (Ampere) */
	m->range_B = 60.;	/* (Volt)   */

	/* Hall Sensor installation angles (Degree).
	 * */
	m->hall[0] = 30.7;
	m->hall[1] = 150.1;
	m->hall[2] = 270.5;

	/* EABI incremental (absolute) encoder.
	 * */
	m->eabi_ERES = 2400;	/* Mechanical resolution  */
	m->eabi_WRAP = 65536;	/* Wrap constant          */
	m->eabi_Zq = 1.0;	/* Reduction ratio        */

	/* Resolver SIN/COS.
	 * */
	m->analog_Zq = 1.0;	/* Reduction ratio        */
}

void blm_restart(blm_t *m)
{
	m->unsync_flag = 0;

	m->state[0] = 0.;	/* Axis D current (Ampere) */
	m->state[1] = 0.;	/* Axis Q current (Ampere) */
	m->state[2] = 0.;	/* Electrical Speed (Radian/Sec) */
	m->state[3] = 0.;	/* Electrical Position (Radian) */
	m->state[4] = m->Ta;	/* Temperature (Celsius) */
	m->state[5] = 0.;	/* Energy consumption (Joule) */
	m->state[6] = m->Udc;	/* DC link voltage (Volt) */

	m->state[7] = 0.;	/* Current Sensor iA */
	m->state[8] = 0.;	/* Current Sensor iB */
	m->state[9] = 0.;	/* Current Sensor iC */
	m->state[10] = m->Udc;	/* Voltage Sensor uS */
	m->state[11] = m->Udc;	/* Voltage Sensor uS */
	m->state[12] = 0.;	/* Voltage Sensor uA */
	m->state[13] = 0.;	/* Voltage Sensor uB */
	m->state[14] = 0.;	/* Voltage Sensor uC */

	m->xfet[0] = 0;
	m->xfet[1] = 0;
	m->xfet[2] = 0;
	m->xfet[3] = 0;
	m->xfet[4] = 0;
	m->xfet[5] = 0;

	m->xdtu[0] = 0;
	m->xdtu[1] = 0;
	m->xdtu[2] = 0;

	m->event[0].ev = 0;
	m->event[1].ev = 1;
	m->event[2].ev = 2;
	m->event[3].ev = 3;
	m->event[4].ev = 4;
	m->event[5].ev = 5;
	m->event[6].ev = 6;
	m->event[7].ev = 7;
	m->event[8].ev = 8;

	m->revol = 0;
}

static void
blm_equation(const blm_t *m, const double state[7], double y[7])
{
	double		uA, uB, uD, uQ, Rs, lambda, mP, mQ, mS;

	/* Thermal drift.
	 * */
	Rs = m->Rs * (1. + 3.93E-3 * (state[4] - m->Ta));
	lambda = m->lambda * (1. - 1.20E-3 * (state[4] - m->Ta));

	/* Voltage from VSI.
	 * */
	uQ = (m->xfet[0] + m->xfet[1] + m->xfet[2]) / 3.;
	uA = (m->xfet[0] - uQ) * state[6];
	uB = (m->xfet[1] - uQ) * state[6];

	blm_AB_DQ(state[3], uA, uB, &uD, &uQ);

	/* Energy consumption equation.
	 * */
	y[5] = 1.5 * (state[0] * uD + state[1] * uQ);

	/* DC link voltage equation.
	 * */
	y[6] = ((m->Udc - state[6]) / m->Rdc - y[5] / state[6]) / m->Cdc;

	/* Electrical equations of PMSM.
	 * */
	uD += - Rs * state[0] + m->Lq * state[2] * state[1];
	uQ += - Rs * state[1] - m->Ld * state[2] * state[0] - lambda * state[2];

	y[0] = uD / m->Ld;
	y[1] = uQ / m->Lq;

	/* Torque production.
	 * */
	mP = 1.5 * m->Zp * (lambda + (m->Ld - m->Lq) * state[0]) * state[1];

	/* Mechanical load torque.
	 * */
	mS = state[2] / m->Zp;
	mQ = m->Mq[0] - mS * (m->Mq[1] + fabs(mS) * m->Mq[2]);
	mQ += - mS / (1.f + fabs(mS)) * m->Mq[3];

	/* Mechanical equations.
	 * */
	y[2] = m->Zp * (mP + mQ) / m->Jm;
	y[3] = state[2];

	/* Thermal equation.
	 * */
	y[4] = (1.5f * Rs * (state[0] * state[0] + state[1] * state[1])
			+ (m->Ta - state[4]) / m->Rt) / m->Ct;
}

static void
blm_ode_step(blm_t *m, double dT)
{
	double		x0[7], y0[7], y1[7];

	double		iA, iB, iC, uA, uB, uC;
	double		kA, kB, uMIN;

	/* Second-order ODE solver.
	 * */

	blm_equation(m, m->state, y0);

	if (m->pwm_Z != BLM_Z_DETACHED) {

		x0[0] = m->state[0] + y0[0] * dT;
		x0[1] = m->state[1] + y0[1] * dT;
	}
	else {
		x0[0] = 0.;
		x0[1] = 0.;
	}

	x0[2] = m->state[2] + y0[2] * dT;
	x0[3] = m->state[3] + y0[3] * dT;
	x0[4] = m->state[4] + y0[4] * dT;
	x0[5] = m->state[5] + y0[5] * dT;
	x0[6] = m->state[6] + y0[6] * dT;

	blm_equation(m, x0, y1);

	if (m->pwm_Z != BLM_Z_DETACHED) {

		m->state[0] += (y0[0] + y1[0]) * dT / 2.;
		m->state[1] += (y0[1] + y1[1]) * dT / 2.;
	}
	else {
		m->state[0] = 0.;
		m->state[1] = 0.;
	}

	m->state[2] += (y0[2] + y1[2]) * dT / 2.;
	m->state[3] += (y0[3] + y1[3]) * dT / 2.;
	m->state[4] += (y0[4] + y1[4]) * dT / 2.;
	m->state[5] += (y0[5] + y1[5]) * dT / 2.;
	m->state[6] += (y0[6] + y1[6]) * dT / 2.;

	/* Sensor transient (FAST).
	 * */
	kA = 1.0 - exp(- dT / m->tau_A);
	kB = 1.0 - exp(- dT / m->tau_B);

	blm_DQ_ABC(m->state[3], m->state[0], m->state[1], &iA, &iB, &iC);

	m->state[7] += (iA - m->state[7]) * kA;
	m->state[8] += (iB - m->state[8]) * kA;
	m->state[9] += (iC - m->state[9]) * kA;

	if (m->pwm_Z != BLM_Z_DETACHED) {

		uA = m->xfet[0] * m->state[6];
		uB = m->xfet[1] * m->state[6];
		uC = m->xfet[2] * m->state[6];
	}
	else {
		blm_DQ_ABC(m->state[3], 0., m->lambda * m->state[2], &uA, &uB, &uC);

		uMIN = (uA < uB) ? uA : uB;
		uMIN = (uMIN < uC) ? uMIN : uC;

		uA += - uMIN;
		uB += - uMIN;
		uC += - uMIN;
	}

	m->state[10] += (m->state[6]  - m->state[10]) * kA;
	m->state[11] += (m->state[10] - m->state[11]) * kB;
	m->state[12] += (uA - m->state[12]) * kB;
	m->state[13] += (uB - m->state[13]) * kB;
	m->state[14] += (uC - m->state[14]) * kB;

	if (m->proc_step != NULL) {

		m->proc_step(dT);
	}
}

static void
blm_solve(blm_t *m, double dT)
{
	double		iA, iB, iC;

	blm_DQ_ABC(m->state[3], m->state[0], m->state[1], &iA, &iB, &iC);

	if (m->xdtu[0] != 0) {

		/* Dead-Time on A.
		 * */
		m->xfet[0] = (iA > m->Dtol) ? 0 : (iA < - m->Dtol) ? 1 : m->xfet[0];
	}

	if (m->xdtu[1] != 0) {

		/* Dead-Time on B.
		 * */
		m->xfet[1] = (iB > m->Dtol) ? 0 : (iB < - m->Dtol) ? 1 : m->xfet[1];
	}

	if (m->xdtu[2] != 0) {

		/* Dead-Time on C.
		 * */
		m->xfet[2] = (iC > m->Dtol) ? 0 : (iC < - m->Dtol) ? 1 : m->xfet[2];
	}

	if (m->xfet[0] != m->xfet[3]) {

		/* ADC surge on A.
		 * */
		m->state[7]  += lfg_gauss() * 5.;
		m->state[10] += lfg_gauss() * 2.;
	}

	if (m->xfet[1] != m->xfet[4]) {

		/* ADC surge on B.
		 * */
		m->state[8]  += lfg_gauss() * 5.;
		m->state[10] += lfg_gauss() * 2.;
	}

	if (m->xfet[2] != m->xfet[5]) {

		/* ADC surge on C.
		 * */
		m->state[9]  += lfg_gauss() * 5.;
		m->state[10] += lfg_gauss() * 2.;
	}

	/* Divide the long interval.
	 * */
	while (dT > m->sol_dT) {

		blm_ode_step(m, m->sol_dT);
		dT -= m->sol_dT;
	}

	blm_ode_step(m, dT);

	if (m->state[3] < - M_PI) {

		m->state[3] += 2. * M_PI;
		m->revol -= 1;
	}
	else if (m->state[3] > M_PI) {

		m->state[3] -= 2. * M_PI;
		m->revol += 1;
	}

	/* Keep the previous VSI state.
	 * */
	m->xfet[3] = m->xfet[0];
	m->xfet[4] = m->xfet[1];
	m->xfet[5] = m->xfet[2];
}

static double
blm_ADC(double vconv, double vmin, double vmax)
{
	double		rel;
	int		ADC;

	rel = (vconv - vmin) / (vmax - vmin);

	ADC = (int) (rel * 4096. + lfg_gauss() * 2.);
	ADC = ADC < 0 ? 0 : ADC > 4095 ? 4095 : ADC;

	return (double) ADC / 4096. * (vmax - vmin) + vmin;
}

static void
blm_sample_hall(blm_t *m)
{
	double		mX, mY, hX, hY;
	int		HS = 0;

	mX = cos(m->state[3]);
	mY = sin(m->state[3]);

	hX = cos(m->hall[0] * (M_PI / 180.));
	hY = sin(m->hall[0] * (M_PI / 180.));

	HS |= (mX * hX + mY * hY < 0.) ? 1 : 0;

	hX = cos(m->hall[1] * (M_PI / 180.));
	hY = sin(m->hall[1] * (M_PI / 180.));

	HS |= (mX * hX + mY * hY < 0.) ? 2 : 0;

	hX = cos(m->hall[2] * (M_PI / 180.));
	hY = sin(m->hall[2] * (M_PI / 180.));

	HS |= (mX * hX + mY * hY < 0.) ? 4 : 0;

	m->pulse_HS = HS;
}

static void
blm_sample_eabi(blm_t *m)
{
	double		location, angle;
	int		EP;

	location = m->state[3] + (2. * M_PI) * (double) m->revol;
	angle = location * m->eabi_Zq / m->Zp;

	EP = (int) (angle / (2. * M_PI) * (double) m->eabi_ERES);

	EP = EP - (EP / m->eabi_WRAP) * m->eabi_WRAP;
	EP += (EP < 0) ? m->eabi_WRAP : 0;

	m->pulse_EP = EP;
}

static void
blm_sample_analog(blm_t *m)
{
	double		location, angle;

	location = m->state[3] + (2. * M_PI) * (double) m->revol;
	angle = location * m->analog_Zq / m->Zp;

	m->analog_SIN = (float) blm_ADC(sin(angle), - 3., 3.);
	m->analog_COS = (float) blm_ADC(cos(angle), - 3., 3.);
}

static void
blm_pwm_bsort(blm_t *m)
{
	int		ebuf[2], i;

	do {
		ebuf[0] = 0;

		/* Bubble SORT.
		 * */
		for (i = 1; i < 9; ++i) {

			if (m->event[i - 1].comp < m->event[i].comp) {

				ebuf[0] = m->event[i].ev;
				ebuf[1] = m->event[i].comp;

				m->event[i].ev   = m->event[i - 1].ev;
				m->event[i].comp = m->event[i - 1].comp;
				m->event[i - 1].ev   = ebuf[0];
				m->event[i - 1].comp = ebuf[1];
			}
		}
	}
	while (ebuf[0] != 0);
}

static void
blm_pwm_up_event(blm_t *m, int ev)
{
	switch (ev) {

		case 1:
			m->analog_uS = (float) blm_ADC(m->state[11], 0., m->range_B);
			m->analog_uA = (float) blm_ADC(m->state[12], 0., m->range_B);
			m->analog_uB = (float) blm_ADC(m->state[13], 0., m->range_B);
			break;

		case 2:
			m->analog_uC = (float) blm_ADC(m->state[14], 0., m->range_B);

			m->analog_iA = m->hold_iA;
			m->analog_iB = m->hold_iB;
			m->analog_iC = m->hold_iC;

			blm_sample_analog(m);
			blm_sample_hall(m);
			blm_sample_eabi(m);
			break;

		case 3:
			m->xfet[0] = 1;
			m->xdtu[0] = 1;
			break;

		case 4:
			m->xfet[1] = 1;
			m->xdtu[1] = 1;
			break;

		case 5:
			m->xfet[2] = 1;
			m->xdtu[2] = 1;
			break;

		case 6:
			m->xfet[0] = 1;
			m->xdtu[0] = 0;
			break;

		case 7:
			m->xfet[1] = 1;
			m->xdtu[1] = 0;
			break;

		case 8:
			m->xfet[2] = 1;
			m->xdtu[2] = 0;
			break;

		default:
			break;
	}
}

static void
blm_pwm_down_event(blm_t *m, int ev)
{
	switch (ev) {

		case 0:
			m->hold_iA = (float) blm_ADC(m->state[7], - m->range_A, m->range_A);
			m->hold_iB = (float) blm_ADC(m->state[8], - m->range_A, m->range_A);
			m->hold_iC = (float) blm_ADC(m->state[9], - m->range_A, m->range_A);
			break;

		case 3:
			m->xfet[0] = 0;
			m->xdtu[0] = 0;
			break;

		case 4:
			m->xfet[1] = 0;
			m->xdtu[1] = 0;
			break;

		case 5:
			m->xfet[2] = 0;
			m->xdtu[2] = 0;
			break;

		case 6:
			m->xfet[0] = 0;
			m->xdtu[0] = 1;
			break;

		case 7:
			m->xfet[1] = 0;
			m->xdtu[1] = 1;
			break;

		case 8:
			m->xfet[2] = 0;
			m->xdtu[2] = 1;
			break;

		default:
			break;
	}
}

static void
blm_pwm_solve(blm_t *m)
{
	double		dTu;
	int		rev[9], xA, xB, xC, xMIN, xMAX, xAD, xCONV, level, i;

	for (i = 0; i < 9; ++i) {

		level = m->event[i].ev;
		rev[level] = i;
	}

	dTu = m->pwm_dT / (double) (m->pwm_resolution * 2);

	xMIN = (int) (m->pwm_minimal * (double) m->pwm_resolution / m->pwm_dT);
	xMAX = m->pwm_resolution;

	xAD = (int) (m->adc_Toffset / dTu);
	xCONV = (int) (m->adc_Tconv / dTu);

	/* ADC sampling.
	 * */
	m->event[rev[0]].comp = xMAX - xAD;
	m->event[rev[1]].comp = xMAX + xAD - xCONV;
	m->event[rev[2]].comp = xMAX + xAD - 2 * xCONV;

	xA = (m->pwm_A < xMIN) ? 0 : m->pwm_A + (int) (m->pwm_deadtime / dTu);
	xB = (m->pwm_B < xMIN) ? 0 : m->pwm_B + (int) (m->pwm_deadtime / dTu);
	xC = (m->pwm_C < xMIN) ? 0 : m->pwm_C + (int) (m->pwm_deadtime / dTu);

	/* FET low side.
	 * */
	m->event[rev[3]].comp = (xA < xMIN) ? 0 : (xA > xMAX - xMIN) ? xMAX : xA;
	m->event[rev[4]].comp = (xB < xMIN) ? 0 : (xB > xMAX - xMIN) ? xMAX : xB;
	m->event[rev[5]].comp = (xC < xMIN) ? 0 : (xC > xMAX - xMIN) ? xMAX : xC;

	xA = m->pwm_A;
	xB = m->pwm_B;
	xC = m->pwm_C;

	/* FET high side.
	 * */
	m->event[rev[6]].comp = (xA < xMIN) ? 0 : (xA > xMAX - xMIN) ? xMAX : xA;
	m->event[rev[7]].comp = (xB < xMIN) ? 0 : (xB > xMAX - xMIN) ? xMAX : xB;
	m->event[rev[8]].comp = (xC < xMIN) ? 0 : (xC > xMAX - xMIN) ? xMAX : xC;

	/* Get SORTED events.
	 * */
	blm_pwm_bsort(m);

	level = xMAX;

	/* PWM count up.
	 * */
	for (i = 0; i < 9; ++i) {

		if (level != m->event[i].comp) {

			blm_solve(m, dTu * (level - m->event[i].comp));

			level = m->event[i].comp;
		}

		blm_pwm_up_event(m, m->event[i].ev);
	}

	if (m->event[8].comp != 0) {

		blm_solve(m, dTu * m->event[8].comp);
	}

	level = 0;

	/* PWM count down.
	 * */
	for (i = 8; i >= 0; --i) {

		if (level != m->event[i].comp) {

			blm_solve(m, dTu * (m->event[i].comp - level));

			level = m->event[i].comp;
		}

		blm_pwm_down_event(m, m->event[i].ev);
	}

	if (level != xMAX) {

		blm_solve(m, dTu * (xMAX - level));
	}

	/* Get average POWER on PWM cycle.
	 * */
	m->drain_wP = m->state[5] / m->pwm_dT;
	m->state[5] = 0.;
}

void blm_update(blm_t *m)
{
	blm_pwm_solve(m);

	m->time += m->pwm_dT;
}

