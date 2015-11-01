/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

#include "hal/hal.h"
#include "lib.h"
#include "m.h"
#include "pmc.h"
#include "task.h"

int			ap_errno;

extern void pm_i_slew_rate_auto(const char *s);
extern void pm_i_gain_auto(const char *s);
extern void pm_const_E_wb(const char *s);

static void
irq_avg_value_4()
{
	if (td.avgN < td.avgMAX) {

		td.avgSUM[0] += *td.avgIN[0];
		td.avgSUM[1] += *td.avgIN[1];
		td.avgSUM[2] += *td.avgIN[2];
		td.avgSUM[3] += *td.avgIN[3];

		td.avgN++;
	}
	else
		td.pIRQ = NULL;
}

static int
ap_wait_for_idle()
{
	while (pm.m_state != PMC_STATE_IDLE)
		taskIOMUX();

	return pm.m_errno;
}

static const char *
ap_strerror(int errno)
{
	const char *list[] = {

		"No error",
		"Unbalanced resistance",
	};

	return (errno >= 0 && errno < 3) ? list[errno] : "";
}

#define AP_WAIT_FOR_IDLE()	\
{ if (ap_wait_for_idle() != PMC_OK) { printf("ERROR %i: %s" EOL, \
		pm.m_errno, pmc_strerror(pm.m_errno)); break; } }

#define AP_ERORR(errno)		\
{ ap_errno = errno; printf("AP ERROR %i: %s" EOL, \
		errno, ap_strerror(errno)); break; }

static void
ap_get_const_L(float *LDQ)
{
	pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);

	if (fabsf(IMP[2]) < 45.f) {

		LDQ[0] = IMP[0];
		LDQ[1] = IMP[1];
	}
	else {
		LDQ[0] = IMP[1];
		LDQ[1] = IMP[0];
	}
}

void ap_identify_base(const char *s)
{
	float			LDQ[4], temp;

	while (pm.lu_region == PMC_LU_DISABLED)
	{
		pmc_request(&pm, PMC_STATE_ZERO_DRIFT);
		AP_WAIT_FOR_IDLE();

		printf("scal_A0 %3f (A)" EOL, &pm.scal_A[0]);
		printf("scal_B0 %3f (A)" EOL, &pm.scal_B[0]);

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pm.const_R += pm.wave_temp[2];

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pm.const_R += pm.wave_temp[2];

		printf("R %4e (Ohm)" EOL, &pm.const_R);

		temp = pm.wave_freq_sine_hz;
		pm.wave_freq_sine_hz = 10.f;

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();

		pmc_impedance(pm.wave_DFT, pm.wave_freq_sine_hz, IMP);
		pm.wave_freq_sine_hz = temp;

		printf( "+ R1 %4e (Ohm)" EOL
			"+ R2 %4e (Ohm)" EOL, IMP + 3, IMP + 4);

		(IMP[3] / IMP[4] < 91e-2f) ? AP_ERORR(AP_ERROR_UNBALANCED_RESISTANCE) : 0;

		pmc_request(&pm, PMC_STATE_WAVE_HOLD);
		AP_WAIT_FOR_IDLE();

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();

		ap_get_const_L(LDQ + 0);

		temp = pm.wave_freq_sine_hz;
		pm.wave_freq_sine_hz /= 2.f;

		pmc_request(&pm, PMC_STATE_WAVE_SINE);
		AP_WAIT_FOR_IDLE();

		ap_get_const_L(LDQ + 2);
		pm.wave_freq_sine_hz = temp;

		printf(	"+ F %1f (Hz)" EOL
			"+ L1 %4e (H)" EOL
			"+ L2 %4e (H)" EOL,
			"+ F/2 %1f (Hz)" EOL
			"+ L1 %4e (H)" EOL,
			"+ L2 %4e (H)" EOL,
			LDQ + 0, LDQ + 1, LDQ + 2, LDQ + 3);

		temp = (LDQ[0] - LDQ[2]) / (LDQ[0] + LDQ[2]) * 2.f;
		(17e-2f < temp) ? AP_ERORR() : 0;

		/* TODO */

		pm.const_Ld_inversed = 1.f / pm.const_Ld;
		pm.const_Lq_inversed = 1.f / pm.const_Lq;

		printf("Ld %4e (H)" EOL, &pm.const_Ld);
		printf("Lq %4e (H)" EOL, &pm.const_Lq);

		pm_i_slew_rate_auto(EOL);
		pm_i_gain_auto(EOL);

		break;
	}
}

void ap_update_const_E(const char *s)
{
	if (td.pIRQ == NULL && pm.lu_region != PMC_LU_DISABLED) {

		td.avgIN[0] = &pm.lu_X[0];
		td.avgIN[1] = &pm.lu_X[1];
		td.avgIN[2] = &pm.lu_X[4];
		td.avgIN[3] = &pm.drift_Q;

		td.avgSUM[0] = 0.f;
		td.avgSUM[1] = 0.f;
		td.avgSUM[2] = 0.f;
		td.avgSUM[3] = 0.f;

		td.avgN = 0;
		td.avgMAX = pm.freq_hz * pm.T_measure;

		td.pIRQ = &irq_avg_value_4;

		while (td.pIRQ != NULL)
			taskIOMUX();

		td.avgSUM[0] /= (float) td.avgN;
		td.avgSUM[1] /= (float) td.avgN;
		td.avgSUM[2] /= (float) td.avgN;
		td.avgSUM[3] /= (float) td.avgN;

		pm.const_E -= td.avgSUM[3] / td.avgSUM[2];
		pm_const_E_wb(EOL);
	}
}

