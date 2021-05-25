#include "libm.h"
#include "pm.h"

void pm_default(pmc_t *pm)
{
	pm->dc_minimal = 0.25f;		/* (us) */
	pm->dc_clearance = 5.0f;	/* (us) */
	pm->dc_skip = 2.0f;		/* (us) */
	pm->dc_bootstrap = 100.f;	/* (ms) */
	pm->dc_clamped = 1.f;		/* (s)  */

	pm->config_NOP = PM_NOP_THREE_PHASE;
	pm->config_TVM = PM_ENABLED;
	pm->config_IFB = PM_IFB_AB_INLINE;
	pm->config_DEBUG = PM_DISABLED;

	pm->config_VSI_PRECISE = PM_DISABLED;
	pm->config_FORCED = PM_ENABLED;
	pm->config_FLUX = PM_ENABLED;
	pm->config_HFI = PM_DISABLED;
	pm->config_MAJOR_AXIS = PM_DISABLED;
	pm->config_SENSOR = PM_SENSOR_DISABLED;
	pm->config_ABI_REVERSED = PM_DISABLED;
	pm->config_ABI_DEBOUNCE = PM_ENABLED;
	pm->config_DRIVE = PM_DRIVE_SPEED;
	pm->config_MTPA = PM_ENABLED;
	pm->config_WEAK = PM_DISABLED;
	pm->config_BRAKE = PM_DISABLED;
	pm->config_LIMITED = PM_DISABLED;
	pm->config_INFO	= PM_ENABLED;
	pm->config_BOOST = PM_DISABLED;

	pm->tm_transient_slow = 50.f;	/* (ms) */
	pm->tm_transient_fast = 2.f;	/* (ms) */
	pm->tm_voltage_hold = 100.f;	/* (ms) */
	pm->tm_current_hold = 500.f;	/* (ms) */
	pm->tm_instant_probe = 2.f;	/* (ms) */
	pm->tm_average_probe = 500.f;	/* (ms) */
	pm->tm_average_drift = 100.f;	/* (ms) */
	pm->tm_average_inertia = 700.f;	/* (ms) */
	pm->tm_startup = 50.f;		/* (ms) */
	pm->tm_halt_pause = 1000.f;	/* (ms) */

	pm->ad_IA[0] = 0.f;
	pm->ad_IA[1] = 1.f;
	pm->ad_IB[0] = 0.f;
	pm->ad_IB[1] = 1.f;
	pm->ad_US[0] = 0.f;
	pm->ad_US[1] = 1.f;
	pm->ad_UA[0] = 0.f;
	pm->ad_UA[1] = 1.f;
	pm->ad_UB[0] = 0.f;
	pm->ad_UB[1] = 1.f;
	pm->ad_UC[0] = 0.f;
	pm->ad_UC[1] = 1.f;

	pm->probe_current_hold = 10.f;
	pm->probe_current_weak = 0.f;
	pm->probe_hold_angle = 0.f;
	pm->probe_current_sine = 2.f;
	pm->probe_freq_sine_hz = 2000.f;
	pm->probe_speed_maximal_pc = 70.f;
	pm->probe_speed_hold = 700.f;
	pm->probe_speed_detached = 50.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tol = 2.f;
	pm->fault_current_tol = 2.f;
	pm->fault_accuracy_tol = 1E-1f;
	pm->fault_current_halt = 156.f;
	pm->fault_voltage_halt = 57.f;

	pm->vsi_gain_LP_F = 5E-3f;

	pm->tvm_ALLOWED = PM_DISABLED;
	pm->tvm_range_DC = .1f;
	pm->tvm_FIR_A[0] = 0.f;
	pm->tvm_FIR_A[1] = 0.f;
	pm->tvm_FIR_A[2] = 0.f;
	pm->tvm_FIR_B[0] = 0.f;
	pm->tvm_FIR_B[1] = 0.f;
	pm->tvm_FIR_B[2] = 0.f;
	pm->tvm_FIR_C[0] = 0.f;
	pm->tvm_FIR_C[1] = 0.f;
	pm->tvm_FIR_C[2] = 0.f;

	pm->lu_gain_TF = 5E-4f;

	pm->forced_hold_D = 10.f;
	pm->forced_maximal = 700.f;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 200.f;

	pm->detach_take_U = .5f;
	pm->detach_gain_AD = 2E-1f;
	pm->detach_gain_SF = 5E-2f;

	pm->flux_gain_IN = 5E-4f;
	pm->flux_gain_LO = 2E-5f;
	pm->flux_gain_HI = 5E-5f;
	pm->flux_gain_AD = 2E-1f;
	pm->flux_gain_SF = 5E-2f;
	pm->flux_gain_IF = 2E-1f;

	pm->flux_MPPE = 50.f;
	pm->flux_gain_TAKE = 3.f;
	pm->flux_gain_GIVE = 2.f;
	pm->flux_gain_LEVE = 2E-1f;
	pm->flux_gain_LP_S = 2E-1f;

	pm->hfi_inject_sine = 2.f;
	pm->hfi_maximal = 700.f;
	pm->hfi_DIV = 10;
	pm->hfi_SKIP = 1;
	pm->hfi_SUM = 5;
	pm->hfi_FLUX = 0;
	pm->hfi_gain_SF = 5E-3f;
	pm->hfi_gain_IF = 5E-1f;

	pm->hall_ALLOWED = PM_DISABLED;
	pm->hall_prol_T = 100.f;	/* (ms) */
	pm->hall_gain_PF = 1E-0f;
	pm->hall_gain_SF = 5E-3f;
	pm->hall_gain_IF = 1E-0f;

	pm->abi_PPR = 600;
	pm->abi_Zq = 1.f;
	pm->abi_gain_PF = 1E-0f;
	pm->abi_gain_SF = 5E-2f;
	pm->abi_gain_IF = 1E-0f;

	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Zp = 1;
	pm->const_Ja = 0.f;
	pm->const_im_L1 = 0.f;
	pm->const_im_L2 = 0.f;

	pm->watt_wP_maximal = 4000.f;
	pm->watt_iDC_maximal = 80.f;
	pm->watt_wP_reverse = 4000.f;
	pm->watt_iDC_reverse = 80.f;
	pm->watt_dclink_HI = 56.f;
	pm->watt_dclink_LO = 7.f;
	pm->watt_gain_LP_F = 5E-2f;
	pm->watt_gain_LP_P = 5E-2f;

	pm->i_maximal = 120.f;
	pm->i_reverse = pm->i_maximal;
	pm->i_derated_HFI = 10.f;
	pm->i_slew_rate = 4000.f;
	pm->i_tol_Z = 0.f;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->weak_maximal = 30.f;
	pm->weak_gain_EU = 5E-3f;

	pm->v_maximal = 60.f;
	pm->v_reverse = pm->v_maximal;

	pm->s_maximal = 30000.f;
	pm->s_reverse = pm->s_maximal;
	pm->s_accel = 3000.f;
	pm->s_linspan = 100.f;
	pm->s_tol_Z = 0.f;
	pm->s_gain_P = 5E-2f;
	pm->s_gain_S = 5E-1f;

	pm->x_tol_N = 1.f;
	pm->x_tol_Z = 0.f;
	pm->x_gain_P = 35.f;
	pm->x_gain_N = 2E-1f;

	pm->boost_gain_P = 1E-1f;
	pm->boost_gain_I = 1E-3f;
}

void pm_build(pmc_t *pm)
{
	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		pm->k_UMAX = .66666667f;
		pm->k_EMAX = .57735027f;
		pm->k_KWAT = 1.5f;
		pm->k_ZNUL = 0;
	}
	else {
		pm->k_UMAX = 1.f;
		pm->k_EMAX = .70710678f;
		pm->k_KWAT = 1.f;
		pm->k_ZNUL = 0;
	}

	pm->ts_minimal = (int) (pm->dc_minimal * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_clearance = (int) (pm->dc_clearance * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_skip = (int) (pm->dc_skip * (1.f / 1000000.f)
			* pm->freq_hz * (float) pm->dc_resolution);
	pm->ts_bootstrap = PM_TSMS(pm, pm->dc_bootstrap);
	pm->ts_clamped = (int) (pm->freq_hz * pm->dc_clamped);
	pm->ts_inverted = 1.f / (float) pm->dc_resolution;

	if (pm->config_FLUX == PM_ENABLED) {

		pm->temp_const_iE = (pm->const_E > M_EPS_F) ? 1.f / pm->const_E : 0.f;
	}

	if (pm->config_HFI == PM_ENABLED) {

		pm->temp_HFI_wS = 2.f * M_PI_F * pm->freq_hz / (float) pm->hfi_DIV;
		pm->temp_HFI_HT[0] = m_cosf(pm->temp_HFI_wS * pm->dT * .5f);
		pm->temp_HFI_HT[1] = m_sinf(pm->temp_HFI_wS * pm->dT * .5f);
	}

	if (pm->config_SENSOR == PM_SENSOR_HALL) {

		pm->temp_prol_T = PM_TSMS(pm, pm->hall_prol_T);
	}
	else if (pm->config_SENSOR == PM_SENSOR_ABI) {

		pm->temp_ZiPPR = M_PI_F / 2.f * (float) pm->const_Zp
			* pm->abi_Zq / (float) pm->abi_PPR;
	}
}

void pm_tune_MPPE(pmc_t *pm)
{
	float		iSTD, vSTD, rU, wSTD;

	/* Current measurement accuracy (standard deviation).
	 * */
	iSTD = 0.2f;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		/* TVM measurement accuracy (standard deviation).
		 * */
		vSTD = 0.1f;
	}
	else {
		/* Terminal voltage accuracy based on PWM edges distortion.
		 * */
		vSTD = pm->dc_minimal * (1.f / 1000000.f) * pm->freq_hz * pm->const_fb_U;
	}

	/* Add some voltage uncertainty due to resistance thermal drift.
	 * */
	vSTD += 1E-1f * pm->i_maximal * pm->const_R;

	/* Calculate speed estimate STD from FLUX equations.
	 * */
	rU = iSTD * pm->const_R;
	wSTD = m_sqrtf(vSTD * vSTD + rU * rU) * pm->dT;

	rU = iSTD * pm->const_im_L2;
	wSTD = m_sqrtf(wSTD * wSTD + rU * rU) / pm->const_E;

	wSTD *= pm->freq_hz * pm->flux_gain_SF;

	/* Take the value of speed estimate error (peak to peak).
	 * */
	pm->flux_MPPE = wSTD * 5.f;
}

void pm_tune_forced(pmc_t *pm)
{
	/* Tune forced control based on motor constants.
	 * */
	pm->forced_maximal = 5E-1f * pm->k_EMAX * pm->const_fb_U / pm->const_E;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 1E-1f * pm->forced_hold_D / pm->const_Ja;
}

void pm_tune_loop_current(pmc_t *pm)
{
	float		im_Lm, Kp, Ki;

	im_Lm = (pm->const_im_L1 < pm->const_im_L2) ? pm->const_im_L1 : pm->const_im_L2;

	/* Tune current loop based on state-space model.
	 *
	 *          [1-R*T/L-Kp*T/L  -Ki*T/L]
	 * x(k+1) = [1                1     ] * x(k)
	 *
	 * */
	Kp = 2E-1f * im_Lm * pm->freq_hz - pm->const_R;
	Ki = 5E-3f * im_Lm * pm->freq_hz;

	pm->i_gain_P = (Kp > 0.f) ? Kp : 0.f;
	pm->i_gain_I = Ki;

	/* Get limit current slew rate.
	 * */
	pm->i_slew_rate = 2E-2f * pm->const_fb_U / im_Lm;
}

void pm_tune_loop_speed(pmc_t *pm)
{
	/* Tune load torque estimate.
	 * */
	pm->lu_gain_TF = 5E-0f / (pm->flux_MPPE * pm->freq_hz * pm->const_Ja);

	/* Tune speed loop based on MPPE value.
	 * */
	pm->s_gain_P = 2E-0f / pm->flux_MPPE;
}

static void
pm_forced(pmc_t *pm)
{
	float		wSP, dS;

	/* Get the SETPOINT of forced speed.
	 * */
	if (pm->config_DRIVE == PM_DRIVE_CURRENT) {

		wSP = (pm->i_setpoint_torque < - M_EPS_F) ? - PM_MAX_F
			: (pm->i_setpoint_torque > M_EPS_F) ? PM_MAX_F : 0.f;
	}
	else {
		wSP = pm->s_setpoint_speed;
	}

	/* Maximal forced speed constraint.
	 * */
	wSP = (wSP > pm->forced_maximal) ? pm->forced_maximal :
		(wSP < - pm->forced_reverse) ? - pm->forced_reverse : wSP;

	/* Update actual speed with specified acceleration.
	 * */
	dS = pm->forced_accel * pm->dT;
	pm->forced_wS = (pm->forced_wS < wSP - dS) ? pm->forced_wS + dS :
		(pm->forced_wS > wSP + dS) ? pm->forced_wS - dS : wSP;

	/* Update DQ-frame.
	 * */
	m_rotatef(pm->forced_F, pm->forced_wS * pm->dT);
	m_normalizef(pm->forced_F);
}

static void
pm_detached_FLUX(pmc_t *pm)
{
	float		uA, uB, uC, uX, uY, U, EX, EY, E, S;

	/* Get BEMF voltage.
	 * */
	uA = pm->fb_uA;
	uB = pm->fb_uB;
	uC = pm->fb_uC;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		U = (1.f / 3.f) * (uA + uB + uC);
		uA = uA - U;
		uB = uB - U;

		uX = uA;
		uY = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uX = uA - uC;
		uY = uB - uC;
	}

	pm->detach_X = uX;
	pm->detach_Y = uY;

	/* Absolute BEMF voltage.
	 * */
	U = m_sqrtf(uX * uX + uY * uY);

	if (U > pm->detach_take_U) {

		E = 1.f / U;

		uX *= E;
		uY *= E;

		if (pm->detach_TIM != 0) {

			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->detach_V, pm->flux_wS * pm->dT);
			m_normalizef(pm->detach_V);

			EX = uX * pm->detach_V[0] + uY * pm->detach_V[1];
			EY = uY * pm->detach_V[0] - uX * pm->detach_V[1];

			if (EX > M_EPS_F) {

				E = EY / EX * pm->freq_hz;

				S = U * pm->detach_gain_AD;
				S = (S > 1.f) ? 1.f : S;

				pm->flux_wS += E * S * pm->detach_gain_SF;
			}

			pm->flux_E = U / m_fabsf(pm->flux_wS);

			E = (pm->flux_wS < 0.f) ? - 1.f : 1.f;

			pm->flux_F[0] = uY * E;
			pm->flux_F[1] = - uX * E;

			E = (pm->const_E > M_EPS_F) ? pm->const_E : pm->flux_E;

			pm->flux_X = E * pm->flux_F[0];
			pm->flux_Y = E * pm->flux_F[1];
		}

		pm->detach_V[0] = uX;
		pm->detach_V[1] = uY;

		pm->detach_TIM++;
	}
	else {
		pm->detach_TIM = 0;
		pm->flux_wS = 0.f;
	}
}

static void
pm_mode_FLUX(pmc_t *pm)
{
	float			lev_wS, lev_E;
	int			lev_TIM;

	/* Get smooth speed passed through LPF.
	 * */
	pm->flux_lpf_wS += (pm->flux_wS - pm->flux_lpf_wS) * pm->flux_gain_LP_S;

	if (pm->flux_mode == PM_FLUX_UNCERTAIN) {

		lev_wS = pm->flux_gain_TAKE * pm->flux_MPPE;

		if (pm->lu_mode == PM_LU_DETACHED) {

			lev_TIM = PM_TSMS(pm, pm->tm_startup);

			if (		m_fabsf(pm->flux_lpf_wS) > lev_wS
					&& pm->detach_TIM > lev_TIM) {

				pm->flux_mode = PM_FLUX_HIGH;
			}
		}
		else {
			lev_E = pm->flux_gain_LEVE * pm->const_E;

			if (m_fabsf(pm->flux_E - pm->const_E) < lev_E) {

				if (		pm->flux_lpf_wS > lev_wS
						&& pm->lu_wS > lev_wS) {

					pm->flux_mode = PM_FLUX_HIGH;
				}
				else if (	pm->flux_lpf_wS < - lev_wS
						&& pm->lu_wS < - lev_wS) {

					pm->flux_mode = PM_FLUX_HIGH;
				}
			}
		}
	}
	else if (pm->flux_mode == PM_FLUX_HIGH) {

		lev_wS = pm->flux_gain_GIVE * pm->flux_MPPE;

		if (pm->lu_mode == PM_LU_DETACHED) {

			if (		m_fabsf(pm->flux_lpf_wS) < lev_wS
					|| pm->detach_TIM == 0) {

				pm->flux_mode = PM_FLUX_UNCERTAIN;
			}
		}
		else {
			if (m_fabsf(pm->flux_lpf_wS) < lev_wS) {

				pm->flux_mode = PM_FLUX_UNCERTAIN;
			}
		}
	}
}

static void
pm_estimate_FLUX(pmc_t *pm)
{
	float		uX, uY, lX, lY, iEE, EX, EY, DX, DY, E, gain_F;

	/* Get the actual voltage.
	 * */
	uX = pm->vsi_X - pm->const_R * pm->lu_iX;
	uY = pm->vsi_Y - pm->const_R * pm->lu_iY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		uX += pm->tvm_DX - pm->vsi_DX;
		uY += pm->tvm_DY - pm->vsi_DY;
	}

	/* Stator FLUX (linear model).
	 * */
	lX = pm->const_im_L2 * pm->lu_iX;
	lY = pm->const_im_L2 * pm->lu_iY;

	if (pm->const_E > M_EPS_F) {

		iEE = pm->temp_const_iE * pm->temp_const_iE;

		/* Adaptive GAIN.
		 * */
		E = m_fabsf(pm->flux_wS * pm->const_E) * pm->flux_gain_AD;
		E = (E > 1.f) ? 1.f : 0.f;

		gain_F = (pm->flux_gain_LO + E * pm->flux_gain_HI) * pm->temp_const_iE;

		/* FLUX observer equations.
		 * */
		pm->flux_X += uX * pm->dT;
		pm->flux_Y += uY * pm->dT;

		EX = pm->flux_X - lX;
		EY = pm->flux_Y - lY;

		gain_F *= 1.f - (EX * EX + EY * EY) * iEE;

		pm->flux_X += EX * gain_F;
		pm->flux_Y += EY * gain_F;
	}
	else {
		/* Startup estimation.
		 * */
		pm->flux_X += uX * pm->dT;
		pm->flux_Y += uY * pm->dT;

		EX = pm->flux_X - lX;
		EY = pm->flux_Y - lY;

		gain_F = - pm->flux_gain_IN;

		pm->flux_X += EX * gain_F;
		pm->flux_Y += EY * gain_F;
	}

	/* Extract rotor FLUX linkage.
	 * */
	EX = pm->flux_X - lX;
	EY = pm->flux_Y - lY;

	E = m_sqrtf(EX * EX + EY * EY);

	pm->flux_E = E;

	if (E > M_EPS_F) {

		E = 1.f / E;

		EX *= E;
		EY *= E;

		if (		pm->lu_mode == PM_LU_FORCED
				&& pm->const_E < M_EPS_F) {

			pm->flux_wS = pm->forced_wS;
		}
		else {
			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->flux_F, pm->flux_wS * pm->dT);
			m_normalizef(pm->flux_F);

			DX = EX * pm->flux_F[0] + EY * pm->flux_F[1];
			DY = EY * pm->flux_F[0] - EX * pm->flux_F[1];

			if (DX > M_EPS_F) {

				E = DY / DX * pm->freq_hz;
				pm->flux_wS += E * pm->flux_gain_SF;
			}

			if (		pm->flux_gain_IF > M_EPS_F
					&& pm->const_Ja > M_EPS_F) {

				pm->flux_wS += (pm->lu_iQ - pm->lu_lpf_torque)
					* pm->flux_gain_IF * pm->dT / pm->const_Ja;
			}
		}

		pm->flux_F[0] = EX;
		pm->flux_F[1] = EY;
	}

	/* Detect HIGH or UNCERTAIN speed mode.
	 * */
	pm_mode_FLUX(pm);
}

void pm_hfi_DFT(pmc_t *pm, float la[5])
{
	lse_t		*ls = &pm->probe_LS[0];
	lse_float_t	v[5];

	float		*DFT = pm->hfi_DFT;
	float		lz[3], iw;

	/* The primary impedance equation is \Z * \I = \U,
	 *
	 * [R - j*lz(0)      j*lz(1)] * [IX] = [UX]
	 * [    j*lz(1)  R - j*lz(2)]   [IY]   [UY], where
	 *
	 * IX = [DFT(0) + j*DFT(1)],  UX = [DFT(2) + j*DFT(3)],
	 * IY = [DFT(4) + j*DFT(5)],  UY = [DFT(6) + j*DFT(7)].
	 *
	 * We rewrite it with respect to the impedance components.
	 *
	 * [DFT(0)  DFT(1) -DFT(5)  0     ]   [R    ]   [DFT(2)]
	 * [DFT(1) -DFT(0)  DFT(4)  0     ] * [lz(0)] = [DFT(3)]
	 * [DFT(4)  0      -DFT(1)  DFT(5)]   [lz(1)]   [DFT(6)].
	 * [DFT(5)  0       DFT(0) -DFT(4)]   [lz(2)]   [DFT(7)]
	 *
	 * */

	lse_initiate(ls, 4, 1);

	v[0] = DFT[0];
	v[1] = DFT[1];
	v[2] = - DFT[5];
	v[3] = 0.f;
	v[4] = DFT[2];

	lse_insert(ls, v);

	v[0] = DFT[1];
	v[1] = - DFT[0];
	v[2] = DFT[4];
	v[3] = 0.f;
	v[4] = DFT[3];

	lse_insert(ls, v);

	v[0] = DFT[4];
	v[1] = 0.f;
	v[2] = - DFT[1];
	v[3] = DFT[5];
	v[4] = DFT[6];

	lse_insert(ls, v);

	v[0] = DFT[5];
	v[1] = 0.f;
	v[2] = DFT[0];
	v[3] = - DFT[4];
	v[4] = DFT[7];

	lse_insert(ls, v);
	lse_finalise(ls);

	iw = 1.f / pm->temp_HFI_wS;

	la[4] = ls->b[0];
	lz[0] = ls->b[1] * iw;
	lz[1] = ls->b[2] * iw;
	lz[2] = ls->b[3] * iw;

	m_la_eigf(lz, la, (pm->config_MAJOR_AXIS == PM_ENABLED) ? 1 : 0);
}

static void
pm_estimate_HFI(pmc_t *pm)
{
	float			iD, iQ, uD, uQ;
	float			la[5], F[2], E;

	if (pm->hfi_DFT_N == pm->hfi_DIV * pm->hfi_SKIP) {

		pm->hfi_DFT[0] = 0.f;
		pm->hfi_DFT[1] = 0.f;
		pm->hfi_DFT[2] = 0.f;
		pm->hfi_DFT[3] = 0.f;
		pm->hfi_DFT[4] = 0.f;
		pm->hfi_DFT[5] = 0.f;
		pm->hfi_DFT[6] = 0.f;
		pm->hfi_DFT[7] = 0.f;
		pm->hfi_DFT[8] = 0.f;

		pm->hfi_REM[0] = 0.f;
		pm->hfi_REM[1] = 0.f;
		pm->hfi_REM[2] = 0.f;
		pm->hfi_REM[3] = 0.f;
		pm->hfi_REM[4] = 0.f;
		pm->hfi_REM[5] = 0.f;
		pm->hfi_REM[6] = 0.f;
		pm->hfi_REM[7] = 0.f;
		pm->hfi_REM[8] = 0.f;
	}

	iD = pm->lu_F[0] * pm->hfi_REM[10] + pm->lu_F[1] * pm->hfi_REM[11];
	iQ = pm->lu_F[0] * pm->hfi_REM[11] - pm->lu_F[1] * pm->hfi_REM[10];

	/* Get corrected frame (half-period shift).
	 * */
	F[0] = pm->lu_F[0] * pm->temp_HFI_HT[0] - pm->lu_F[1] * pm->temp_HFI_HT[1];
	F[1] = pm->lu_F[1] * pm->temp_HFI_HT[0] + pm->lu_F[0] * pm->temp_HFI_HT[1];

	/* Get VSI voltages on DQ-axes.
	 * */
	uD = F[0] * pm->tvm_DX + F[1] * pm->tvm_DY;
	uQ = F[0] * pm->tvm_DY - F[1] * pm->tvm_DX;

	/* Discrete Fourier Transform (DFT).
	 * */
	m_rsumf(&pm->hfi_DFT[0], &pm->hfi_REM[0], iD * pm->hfi_wave[0]);
	m_rsumf(&pm->hfi_DFT[1], &pm->hfi_REM[1], iD * pm->hfi_wave[1]);
	m_rsumf(&pm->hfi_DFT[2], &pm->hfi_REM[2], uD * pm->hfi_wave[0]);
	m_rsumf(&pm->hfi_DFT[3], &pm->hfi_REM[3], uD * pm->hfi_wave[1]);
	m_rsumf(&pm->hfi_DFT[4], &pm->hfi_REM[4], iQ * pm->hfi_wave[0]);
	m_rsumf(&pm->hfi_DFT[5], &pm->hfi_REM[5], iQ * pm->hfi_wave[1]);
	m_rsumf(&pm->hfi_DFT[6], &pm->hfi_REM[6], uQ * pm->hfi_wave[0]);
	m_rsumf(&pm->hfi_DFT[7], &pm->hfi_REM[7], uQ * pm->hfi_wave[1]);

	if (pm->hfi_FLUX != 0) {

		/* Get D-axis polarity with doubled frequency cosine wave.
		 * */
		E = pm->hfi_wave[0] * pm->hfi_wave[0] - pm->hfi_wave[1] * pm->hfi_wave[1];
		m_rsumf(&pm->hfi_DFT[8], &pm->hfi_REM[8], iD * E);
	}

	pm->hfi_REM[10] = pm->lu_iX;
	pm->hfi_REM[11] = pm->lu_iY;

	pm->hfi_DFT_N += 1;

	if (pm->hfi_DFT_N >= pm->hfi_DIV * (pm->hfi_SUM + pm->hfi_SKIP)) {

		pm_hfi_DFT(pm, la);

		if (m_isfinitef(la[0]) != 0 && m_isfinitef(la[1]) != 0) {

			/* Speed estimation (PLL).
			 * */
			if (la[0] > M_EPS_F) {

				E = la[1] / la[0] * pm->freq_hz;
				pm->hfi_wS += E * pm->hfi_gain_SF;
			}

			/* Get actual saliency frame.
			 * */
			F[0] = pm->lu_F[0] * la[0] - pm->lu_F[1] * la[1];
			F[1] = pm->lu_F[1] * la[0] + pm->lu_F[0] * la[1];

			E = (3.f - F[0] * F[0] - F[1] * F[1]) * .5f;

			pm->hfi_F[0] = F[0] * E;
			pm->hfi_F[1] = F[1] * E;

			pm->hfi_im_L1 = la[2];
			pm->hfi_im_L2 = la[3];
			pm->hfi_im_R = la[4];

			if (pm->hfi_FLUX != 0) {

				/* Get accumulated FLUX polarity.
				 * */
				pm->hfi_im_FP += pm->hfi_DFT[8];
				pm->hfi_DFT_P += 1;

				if (pm->hfi_DFT_P >= pm->hfi_FLUX) {

					if (pm->hfi_im_FP > 0.f) {

						/* Flip DQ-frame.
						 * */
						pm->hfi_F[0] = - pm->hfi_F[0];
						pm->hfi_F[1] = - pm->hfi_F[1];
					}

					pm->hfi_im_FP = 0.f;
					pm->hfi_DFT_P = 0;
				}
			}
		}

		pm->hfi_DFT_N = 0;
	}

	if (		pm->hfi_gain_IF > M_EPS_F
			&& pm->const_Ja > M_EPS_F) {

		pm->hfi_wS += (pm->lu_iQ - pm->lu_lpf_torque)
			* pm->hfi_gain_IF * pm->dT / pm->const_Ja;
	}

	/* Maximal HFI speed constraint.
	 * */
	pm->hfi_wS = (pm->hfi_wS > pm->hfi_maximal) ? pm->hfi_maximal :
		(pm->hfi_wS < - pm->hfi_maximal) ? - pm->hfi_maximal : pm->hfi_wS;

	/* Extrapolate rotor position.
	 * */
	m_rotatef(pm->hfi_F, pm->hfi_wS * pm->dT);
	m_normalizef(pm->hfi_F);

	/* HF wave synthesis.
	 * */
	m_rotatef(pm->hfi_wave, pm->temp_HFI_wS * pm->dT);
	m_normalizef(pm->hfi_wave);
}

static void
pm_sensor_HALL(pmc_t *pm)
{
	float		hF[2], A, B, relE, gain_SF;
	int		HS;

	HS = pm->fb_HS;

	if (HS >= 1 && HS <= 6) {

		pm->hall_prolTIM++;

		if (HS == pm->hall_HS) {

			if (pm->hall_DIRF != 0) {

				relE = pm->hall_wS * pm->hall_gain_PF * pm->dT;
				B = PM_HALL_SPAN * pm->hall_gain_PF;

				if (		m_fabsf(relE) > M_EPS_F
						&& m_fabsf(pm->hall_prolS) < B) {

					m_rotatef(pm->hall_F, relE);
					m_normalizef(pm->hall_F);

					pm->hall_prolS += relE;
				}
				else {
					relE = 0.f;
				}

				if (pm->hall_prolTIM >= pm->temp_prol_T) {

					pm->hall_DIRF = 0;

					relE = 0.f;
				}
			}
			else {
				pm->hall_F[0] = pm->hall_ST[HS].X;
				pm->hall_F[1] = pm->hall_ST[HS].Y;

				relE = 0.f;
			}
		}
		else {
			hF[0] = pm->hall_ST[HS].X;
			hF[1] = pm->hall_ST[HS].Y;

			A = pm->hall_ST[pm->hall_HS].X;
			B = pm->hall_ST[pm->hall_HS].Y;

			relE = hF[1] * A - hF[0] * B;
			pm->hall_DIRF = (relE < 0.f) ? - 1 : 1;

			relE = - pm->hall_DIRF * (PM_HALL_SPAN / 2.f);

			m_rotatef(hF, relE * pm->hall_gain_PF);
			m_normalizef(hF);

			A = hF[0] * pm->hall_F[0] + hF[1] * pm->hall_F[1];
			B = hF[1] * pm->hall_F[0] - hF[0] * pm->hall_F[1];

			pm->hall_prolS = 0.f;
			pm->hall_F[0] = hF[0];
			pm->hall_F[1] = hF[1];

			relE = m_atan2f(B, A);
		}

		gain_SF = 1.f / (float) pm->hall_prolTIM;
		gain_SF = (gain_SF < pm->hall_gain_SF) ? gain_SF : pm->hall_gain_SF;

		pm->hall_wS += (relE * pm->freq_hz - pm->hall_wS) * gain_SF;

		B = PM_HALL_SPAN * pm->hall_gain_PF;

		if (		pm->hall_gain_IF > M_EPS_F
				&& pm->const_Ja > M_EPS_F
				&& m_fabsf(pm->hall_prolS) < B) {

			pm->hall_wS += (pm->lu_iQ - pm->lu_lpf_torque)
				* pm->hall_gain_IF * pm->dT / pm->const_Ja;
		}

		if (HS != pm->hall_HS) {

			pm->hall_HS = HS;
			pm->hall_prolTIM = 0;
		}
	}
	else {
		pm->hall_ALLOWED = PM_DISABLED;

		pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;
		pm->lu_mode = PM_LU_ESTIMATE_FLUX;
	}
}

static void
pm_sensor_ABI(pmc_t *pm)
{
	float			relR, rotR, rotF[2], gain_SF;
	int			relEP, WRAP, N;

	relEP = (short int) (pm->fb_EP - pm->abi_baseEP);
	relEP = (pm->config_ABI_REVERSED == PM_ENABLED) ? - relEP : relEP;

	pm->abi_baseEP = pm->fb_EP;
	pm->abi_prolTIM++;

	if (pm->config_ABI_DEBOUNCE == PM_ENABLED) {

		N =  (relEP > 0 && pm->abi_lastEP < 0) ? - 1 : 0;
		N += (relEP < 0 && pm->abi_lastEP > 0) ?   1 : 0;

		pm->abi_lastEP = (relEP != 0) ? relEP : pm->abi_lastEP;

		relEP += N;
	}

	if (relEP == 0) {

		relR = pm->abi_wS * pm->abi_gain_PF * pm->dT;
		relR = (m_fabsf(pm->abi_prolS) < pm->temp_ZiPPR) ? relR : 0.f;
		pm->abi_prolS += relR;

		rotR = pm->abi_prolS + (float) pm->abi_rotEP * pm->temp_ZiPPR;
	}
	else {
		relR = (float) relEP * pm->temp_ZiPPR - pm->abi_prolS;
		pm->abi_prolS = 0.f;

		WRAP = 4 * pm->abi_PPR;

		pm->abi_rotEP += relEP;
		pm->abi_rotEP += (pm->abi_rotEP < - WRAP) ? WRAP : 0;
		pm->abi_rotEP += (pm->abi_rotEP > WRAP) ? - WRAP : 0;

		rotR = (float) pm->abi_rotEP * pm->temp_ZiPPR;
	}

	N = (int) (rotR / (2.f * M_PI_F));
	rotR -= (float) (N * 2.f * M_PI_F);

	rotR += (rotR < - M_PI_F) ? 2.f * M_PI_F : 0.f;
	rotR += (rotR > M_PI_F) ? - 2.f * M_PI_F : 0.f;

	rotF[0] = m_cosf(rotR);
	rotF[1] = m_sinf(rotR);

	pm->abi_F[0] = rotF[0] * pm->abi_baseF[0] - rotF[1] * pm->abi_baseF[1];
	pm->abi_F[1] = rotF[1] * pm->abi_baseF[0] + rotF[0] * pm->abi_baseF[1];

	gain_SF = 1.f / (float) pm->abi_prolTIM;
	gain_SF = (gain_SF < pm->abi_gain_SF) ? gain_SF : pm->abi_gain_SF;

	pm->abi_wS += (relR * pm->freq_hz - pm->abi_wS) * gain_SF;

	if (		pm->abi_gain_IF > M_EPS_F
			&& pm->const_Ja > M_EPS_F
			&& m_fabsf(pm->abi_prolS) < pm->temp_ZiPPR) {

		pm->abi_wS += (pm->lu_iQ - pm->lu_lpf_torque)
			* pm->abi_gain_IF * pm->dT / pm->const_Ja;
	}

	if (relEP != 0) {

		pm->abi_prolTIM = 0;
	}
}

static void
pm_lu_estimate_torque(pmc_t *pm)
{
	float			wDIF, iLT;

	wDIF = pm->lu_wS - pm->lu_base_wS;

	/* Instant estimate of the load torque.
	 * */
	iLT = pm->lu_iQ - wDIF * pm->freq_hz * pm->const_Ja;

	pm->lu_lpf_torque += (iLT - pm->lu_lpf_torque) * pm->lu_gain_TF;
	pm->lu_base_wS = pm->lu_wS;
}

static void
pm_lu_revol_counter(pmc_t *pm)
{
	if (pm->lu_F[0] < 0.f) {

		if (pm->lu_F[1] < 0.f) {

			pm->lu_revol += (pm->lu_F1 >= 0.f) ? 1 : 0;
		}
		else {
			pm->lu_revol += (pm->lu_F1 < 0.f) ? - 1 : 0;
		}
	}

	pm->lu_F1 = pm->lu_F[1];
}

static void
pm_lu_FSM(pmc_t *pm)
{
	int			lev_SKIP, lev_HOLD;

	if (pm->lu_mode == PM_LU_DETACHED) {

		if (pm->config_FLUX == PM_ENABLED) {

			if (pm->detach_SKIP > 0) {

				pm_detached_FLUX(pm);
			}

			pm_mode_FLUX(pm);

			pm->lu_F[0] = pm->flux_F[0];
			pm->lu_F[1] = pm->flux_F[1];
			pm->lu_wS = pm->flux_wS;
		}

		pm->detach_SKIP++;

		lev_SKIP = 2.f * PM_TSMS(pm, pm->tm_startup);

		if (pm->flux_mode == PM_FLUX_LOCKED_IN_DETACH) {

			/* Lock in detached mode permanently.
			 * */
		}
		else if (pm->flux_mode == PM_FLUX_HIGH) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;

			pm->proc_set_Z(pm->k_ZNUL);
		}
		else if (pm->detach_SKIP < lev_SKIP) {

			/* Not enough time passed to go into low speed
			 * mode. Shut up here.
			 * */
		}
		else if (	pm->config_SENSOR == PM_SENSOR_HALL
				&& pm->hall_ALLOWED == PM_ENABLED) {

			pm->lu_mode = PM_LU_SENSOR_HALL;

			pm->hall_HS = pm->fb_HS;
			pm->hall_DIRF = 0;
			pm->hall_prolTIM = pm->temp_prol_T;

			pm->hall_F[0] = pm->lu_F[0];
			pm->hall_F[1] = pm->lu_F[1];
			pm->hall_wS = pm->lu_wS;

			pm->proc_set_Z(pm->k_ZNUL);
		}
		else if (pm->config_HFI == PM_ENABLED) {

			pm->lu_mode = PM_LU_ESTIMATE_HFI;

			pm->hfi_DFT_N = 0;
			pm->hfi_DFT_P = 0;
			pm->hfi_TIM = 0;

			pm->hfi_F[0] = pm->lu_F[0];
			pm->hfi_F[1] = pm->lu_F[1];
			pm->hfi_wS = pm->lu_wS;

			pm->proc_set_Z(pm->k_ZNUL);
		}
		else if (pm->config_FORCED == PM_ENABLED) {

			pm->lu_mode = PM_LU_FORCED;

			pm->forced_F[0] = pm->flux_F[0];
			pm->forced_F[1] = pm->flux_F[1];
			pm->forced_wS = pm->flux_wS;
			pm->forced_TIM = 0;

			pm->proc_set_Z(pm->k_ZNUL);
		}
	}
	else if (pm->lu_mode == PM_LU_FORCED) {

		pm_forced(pm);

		if (pm->config_FLUX == PM_ENABLED) {

			pm_estimate_FLUX(pm);
		}

		pm->forced_TIM++;

		pm->lu_F[0] = pm->forced_F[0];
		pm->lu_F[1] = pm->forced_F[1];
		pm->lu_wS = pm->forced_wS;

		if (		pm->flux_mode == PM_FLUX_HIGH
				&& pm->const_E > M_EPS_F) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_current_hold);

			if (		pm->config_SENSOR == PM_SENSOR_ABI
					&& pm->forced_TIM > lev_HOLD) {

				pm->lu_mode = PM_LU_SENSOR_ABI;

				pm->abi_baseEP = pm->fb_EP;
				pm->abi_baseF[0] = pm->lu_F[0];
				pm->abi_baseF[1] = pm->lu_F[1];
				pm->abi_lastEP = 0;
				pm->abi_rotEP = 0;
				pm->abi_prolTIM = 0;
				pm->abi_prolS = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_FLUX) {

		if (pm->config_FLUX == PM_ENABLED) {

			pm_estimate_FLUX(pm);

			pm->lu_F[0] = pm->flux_F[0];
			pm->lu_F[1] = pm->flux_F[1];
			pm->lu_wS = pm->flux_wS;
		}

		if (pm->flux_mode == PM_FLUX_UNCERTAIN) {

			if (		pm->config_SENSOR == PM_SENSOR_HALL
					&& pm->hall_ALLOWED == PM_ENABLED) {

				pm->lu_mode = PM_LU_SENSOR_HALL;

				pm->hall_HS = pm->fb_HS;
				pm->hall_DIRF = 0;
				pm->hall_prolTIM = pm->temp_prol_T;

				pm->hall_F[0] = pm->lu_F[0];
				pm->hall_F[1] = pm->lu_F[1];
				pm->hall_wS = pm->lu_wS;
			}
			else if (	pm->config_SENSOR == PM_SENSOR_ABI
					&& pm->flux_LOCKED == PM_FLUX_HIGH) {

				pm->lu_mode = PM_LU_SENSOR_ABI;

				pm->abi_baseEP = pm->fb_EP;
				pm->abi_baseF[0] = pm->lu_F[0];
				pm->abi_baseF[1] = pm->lu_F[1];
				pm->abi_lastEP = 0;
				pm->abi_rotEP = 0;
				pm->abi_prolTIM = 0;
				pm->abi_prolS = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
			else if (pm->config_HFI == PM_ENABLED) {

				pm->lu_mode = PM_LU_ESTIMATE_HFI;

				pm->hfi_DFT_N = 0;
				pm->hfi_DFT_P = 0;
				pm->hfi_TIM = 0;

				pm->hfi_F[0] = pm->lu_F[0];
				pm->hfi_F[1] = pm->lu_F[1];
				pm->hfi_wS = pm->lu_wS;
			}
			else if (pm->config_FORCED == PM_ENABLED) {

				pm->lu_mode = PM_LU_FORCED;

				pm->forced_F[0] = pm->flux_F[0];
				pm->forced_F[1] = pm->flux_F[1];
				pm->forced_wS = pm->flux_wS;
				pm->forced_TIM = 0;
			}
			else if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->lu_mode = PM_LU_DETACHED;

				pm->detach_TIM = 0;
				pm->detach_SKIP = - PM_TSMS(pm, pm->tm_transient_fast);

				pm->flux_F[0] = 1.f;
				pm->flux_F[1] = 0.f;
				pm->flux_wS = 0.f;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->proc_set_Z(7);
			}
		}
		else if (	pm->flux_mode == PM_FLUX_HIGH
				&& pm->flux_LOCKED != PM_FLUX_HIGH) {

			pm->flux_LOCKED = PM_FLUX_HIGH;
		}
	}
	else if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

		pm_estimate_HFI(pm);

		if (pm->config_FLUX == PM_ENABLED) {

			pm_estimate_FLUX(pm);
		}

		pm->hfi_TIM++;

		pm->lu_F[0] = pm->hfi_F[0];
		pm->lu_F[1] = pm->hfi_F[1];
		pm->lu_wS = pm->hfi_wS;

		if (pm->flux_mode == PM_FLUX_HIGH) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_startup);

			if (		pm->config_SENSOR == PM_SENSOR_ABI
					&& pm->hfi_TIM > lev_HOLD) {

				pm->lu_mode = PM_LU_SENSOR_ABI;

				pm->abi_baseEP = pm->fb_EP;
				pm->abi_baseF[0] = pm->lu_F[0];
				pm->abi_baseF[1] = pm->lu_F[1];
				pm->abi_lastEP = 0;
				pm->abi_rotEP = 0;
				pm->abi_prolTIM = 0;
				pm->abi_prolS = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
		}
	}
	else if (pm->lu_mode == PM_LU_SENSOR_HALL) {

		pm_sensor_HALL(pm);

		if (pm->config_FLUX == PM_ENABLED) {

			pm_estimate_FLUX(pm);
		}

		pm->lu_F[0] = pm->hall_F[0];
		pm->lu_F[1] = pm->hall_F[1];
		pm->lu_wS = pm->hall_wS;

		if (pm->flux_mode == PM_FLUX_HIGH) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_mode == PM_LU_SENSOR_ABI) {

		pm_sensor_ABI(pm);

		if (pm->config_FLUX == PM_ENABLED) {

			pm_estimate_FLUX(pm);
		}

		pm->lu_F[0] = pm->abi_F[0];
		pm->lu_F[1] = pm->abi_F[1];
		pm->lu_wS = pm->abi_wS;

		if (pm->flux_mode == PM_FLUX_HIGH) {

			pm->lu_mode = PM_LU_ESTIMATE_FLUX;
		}
	}

	if (PM_CONFIG_DEBUG(pm) == PM_ENABLED) {

		if (pm->debug_locked_HFI != PM_ENABLED) {

			if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

				pm->debug_locked_HFI = PM_ENABLED;
			}
			else if (pm->config_HFI == PM_ENABLED) {

				pm->debug_locked_HFI = PM_ENABLED;

				pm->hfi_DFT_N = 0;
				pm->hfi_DFT_P = 0;
				pm->hfi_TIM = 0;

				pm->hfi_F[0] = pm->lu_F[0];
				pm->hfi_F[1] = pm->lu_F[1];
				pm->hfi_wS = pm->lu_wS;
			}
		}

		if (		pm->debug_locked_HFI == PM_ENABLED
				&& pm->lu_mode != PM_LU_ESTIMATE_HFI) {

			pm_estimate_HFI(pm);
		}

		if (pm->debug_locked_SENSOR == PM_SENSOR_DISABLED) {

			if (pm->lu_mode == PM_LU_SENSOR_HALL) {

				pm->debug_locked_SENSOR = PM_SENSOR_HALL;
			}
			else if (	pm->config_SENSOR == PM_SENSOR_HALL
					&& pm->hall_ALLOWED == PM_ENABLED) {

				pm->debug_locked_SENSOR = PM_SENSOR_HALL;

				pm->hall_HS = pm->fb_HS;
				pm->hall_DIRF = 0;
				pm->hall_prolTIM = pm->temp_prol_T;

				pm->hall_F[0] = pm->lu_F[0];
				pm->hall_F[1] = pm->lu_F[1];
				pm->hall_wS = pm->lu_wS;
			}

			/*if (pm->lu_mode == PM_LU_SENSOR_ABI) {

				pm->debug_locked_SENSOR = PM_SENSOR_ABI;
			}
			else if (	pm->config_SENSOR == PM_SENSOR_ABI
					&& pm->flux_LOCKED == PM_FLUX_HIGH) {

				pm->debug_locked_SENSOR = PM_SENSOR_ABI;

				pm->abi_baseEP = pm->fb_EP;
				pm->abi_baseF[0] = pm->lu_F[0];
				pm->abi_baseF[1] = pm->lu_F[1];
				pm->abi_lastEP = 0;
				pm->abi_rotEP = 0;
				pm->abi_prolTIM = 0;
				pm->abi_prolS = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}*/
		}

		if (		pm->debug_locked_SENSOR == PM_SENSOR_HALL
				&& pm->lu_mode != PM_LU_SENSOR_HALL) {

			pm_sensor_HALL(pm);
		}
		else if (	pm->debug_locked_SENSOR == PM_SENSOR_ABI
				&& pm->lu_mode != PM_LU_SENSOR_ABI) {

			pm_sensor_ABI(pm);
		}
	}

	pm->lu_iD = pm->lu_F[0] * pm->lu_iX + pm->lu_F[1] * pm->lu_iY;
	pm->lu_iQ = pm->lu_F[0] * pm->lu_iY - pm->lu_F[1] * pm->lu_iX;

	/* Get an external load torque estimate in units of current.
	 * */
	pm_lu_estimate_torque(pm);

	/* Track the position to get full number of revolutions.
	 * */
	pm_lu_revol_counter(pm);
}

void pm_voltage(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, uMIN, uMAX, uDC;
	int		xA, xB, xC, xMIN, xMAX;

	uX *= pm->temp_const_ifbU;
	uY *= pm->temp_const_ifbU;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uA = uX;
		uB = - .5f * uX + .8660254f * uY;
		uC = - .5f * uX - .8660254f * uY;
	}
	else {
		uA = uX;
		uB = uY;
		uC = 0.f;
	}

	if (uA < uB) {

		uMIN = uA;
		uMAX = uB;
	}
	else {
		uMIN = uB;
		uMAX = uA;
	}

	if (uC < uMIN) {

		uMIN = uC;
	}
	else if (uMAX < uC) {

		uMAX = uC;
	}

	uDC = uMAX - uMIN;

	pm->vsi_DC = uDC;
	pm->vsi_lpf_DC += (pm->vsi_DC - pm->vsi_lpf_DC) * pm->vsi_gain_LP_F;

	if (uDC > 1.f) {

		uDC = 1.f / uDC;

		uA *= uDC;
		uB *= uDC;
		uC *= uDC;

		uMIN *= uDC;
		uMAX *= uDC;
	}

	if (pm->config_VSI_PRECISE == PM_ENABLED) {

		uDC = (uMAX - uMIN < .5f) ? .5f - uMAX : 0.f - uMIN;
	}
	else {
		uDC = 0.f - uMIN;
	}

	uA += uDC;
	uB += uDC;
	uC += uDC;

	xA = (int) (pm->dc_resolution * uA);
	xB = (int) (pm->dc_resolution * uB);
	xC = (int) (pm->dc_resolution * uC);

	if (pm->lu_mode != PM_LU_DISABLED) {

		if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA > xMAX || xB > xMAX) {

				xMAX = (xA < xB) ? xB : xA;
				xMAX = (xMAX < xC) ? xC : xMAX;
				xMIN = pm->dc_resolution - xMAX;

				xA += xMIN;
				xB += xMIN;
				xC += xMIN;
			}

			xMAX = pm->dc_resolution - pm->ts_minimal;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? pm->dc_resolution : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? pm->dc_resolution : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? pm->dc_resolution : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_LOW) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? xMAX : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? xMAX : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? xMAX : xC;
		}
	}
	else {
		xA += pm->ts_minimal;
		xB += pm->ts_minimal;
		xC += pm->ts_minimal;

		xMAX = pm->dc_resolution - pm->ts_clearance;

		xA = (xA > xMAX) ? xMAX : xA;
		xB = (xB > xMAX) ? xMAX : xB;
		xC = (xC > xMAX) ? xMAX : xC;
	}

	if (pm->ts_bootstrap != 0) {

		pm->vsi_SA = (xA == pm->dc_resolution) ? pm->vsi_SA + 1 : 0;
		pm->vsi_SB = (xB == pm->dc_resolution) ? pm->vsi_SB + 1 : 0;
		pm->vsi_SC = (xC == pm->dc_resolution) ? pm->vsi_SC + 1 : 0;

		if (		pm->vsi_SA > pm->ts_bootstrap
				|| pm->vsi_SB > pm->ts_bootstrap
				|| pm->vsi_SC > pm->ts_bootstrap) {

			/* Clamp voltage to a safe level if bootstrap retention
			 * time is running out.
			 * */
			pm->vsi_TIM = 1;
		}

		if (pm->vsi_TIM >= 1) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			xA = (xA > xMAX) ? xMAX : xA;
			xB = (xB > xMAX) ? xMAX : xB;
			xC = (xC > xMAX) ? xMAX : xC;

			pm->vsi_TIM++;

			if (pm->vsi_TIM >= pm->ts_clamped) {

				pm->vsi_TIM = 0;
			}
		}
	}

	/* Output DC values to PWM.
	 * */
	pm->proc_set_DC(xA, xB, xC);

	pm->vsi_DX = pm->vsi_X;
	pm->vsi_DY = pm->vsi_Y;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		uDC = (1.f / 3.f) * (xA + xB + xC);

		uA = (xA - uDC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - uDC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uA = (xA - xC) * pm->const_fb_U * pm->ts_inverted;
		uB = (xB - xC) * pm->const_fb_U * pm->ts_inverted;

		pm->vsi_X = uA;
		pm->vsi_Y = uB;
	}

	xA = pm->dc_resolution - xA;
	xB = pm->dc_resolution - xB;
	xC = pm->dc_resolution - xC;

	/* Check if there are PWM edges within clearance zone. The CURRENT
	 * measurements will be used or rejected based on this flags.
	 * */
	pm->vsi_AF = ((pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip)
			|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
	pm->vsi_BF = ((pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip)
			|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;

	/* Check if there are PWM edges within clearance zone. The DC link
	 * voltage measurement will be used or rejected based on this flag.
	 * */
	pm->vsi_SF = ((pm->vsi_AG > pm->ts_skip && xA > pm->ts_skip)
			|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
	pm->vsi_SF |= ((pm->vsi_BG > pm->ts_skip && xB > pm->ts_skip)
			|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;
	pm->vsi_SF |= ((pm->vsi_CG > pm->ts_skip && xC > pm->ts_skip)
			|| (pm->vsi_CG == 0 && xC == 0)) ? 0 : 1;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		xMIN = (int) (pm->dc_resolution * (1.f - pm->tvm_range_DC));

		/* Check if terminal voltages were sampled within acceptable
		 * zone. The VOLTAGE measurement will be used or rejected based
		 * on these flags.
		 * */
		pm->vsi_UF = (	   pm->vsi_AG > xMIN
				&& pm->vsi_BG > xMIN
				&& pm->vsi_CG > xMIN
				&& xA > xMIN
				&& xB > xMIN
				&& xC > xMIN) ? 0 : 1;

		/* Check if terminal voltages are exactly zero to get more
		 * accuracy.
		 * */
		pm->vsi_AZ = (pm->vsi_AG == 0) ? 0 : 1;
		pm->vsi_BZ = (pm->vsi_BG == 0) ? 0 : 1;
		pm->vsi_CZ = (pm->vsi_CG == 0) ? 0 : 1;
	}

	pm->vsi_AG = xA;
	pm->vsi_BG = xB;
	pm->vsi_CG = xC;
}

static float
pm_form_iSP(pmc_t *pm, float eS)
{
	float		iSP;

	/* There is a DEAD zone.
	 * */
	eS = (m_fabsf(eS) > pm->s_tol_Z) ? eS : 0.f;

	if (		pm->lu_mode == PM_LU_ESTIMATE_HFI
			|| pm->lu_mode == PM_LU_SENSOR_HALL) {

		/* Slow down in case of weak speed estimate.
		 * */
		eS *= pm->s_gain_S;
	}

	/* The speed regulator uses an load torque estimate as feed forward
	 * component. It also replaces the integral component.
	 * */
	iSP = pm->s_gain_P * eS + pm->lu_lpf_torque;

	/* Output clamp.
	 * */
	iSP = (iSP > pm->i_maximal) ? pm->i_maximal :
		(iSP < - pm->i_reverse) ? - pm->i_reverse : iSP;

	return iSP;
}

static void
pm_loop_current(pmc_t *pm)
{
	float		track_D, track_Q, eD, eQ, uD, uQ, uX, uY, wP, hfi_U;
	float		iMAX, iREV, uMAX, uREV, wMAX, wREV, E, dS, lerp_F;
	int		mode_HFI;

	if (pm->lu_mode == PM_LU_FORCED) {

		track_D = pm->forced_hold_D;
		track_Q = 0.f;
	}
	else if (	pm->lu_mode == PM_LU_ESTIMATE_FLUX
			&& pm->flux_mode == PM_FLUX_UNCERTAIN) {

		track_D = 0.f;
		track_Q = 0.f;
	}
	else {
		if (pm->config_DRIVE == PM_DRIVE_CURRENT) {

			if (		pm->config_BRAKE == PM_ENABLED
					&& pm->i_setpoint_torque < 0.f) {

				/* The holding BRAKE operates in both directions.
				 * */
				track_D = 0.f;
				track_Q = pm_form_iSP(pm, 0.f - pm->lu_wS);

				iMAX = m_fabsf(pm->i_setpoint_torque);

				track_Q = (track_Q > iMAX) ? iMAX
					: (track_Q < - iMAX) ? - iMAX : track_Q;
			}
			else {
				track_D = 0.f;
				track_Q = pm->i_setpoint_torque;
			}
		}
		else {
			track_D = 0.f;
			track_Q = pm->s_iSP;
		}

		if (pm->config_MTPA == PM_ENABLED) {

			/* TODO */
		}

		if (pm->config_WEAK == PM_ENABLED) {

			E = (1.f - pm->vsi_DC) * pm->const_fb_U;

			pm->weak_D += E * pm->weak_gain_EU;
			pm->weak_D = (pm->weak_D < - pm->weak_maximal) ? - pm->weak_maximal :
				(pm->weak_D > 0.f) ? 0.f : pm->weak_D;

			if (pm->weak_D < - M_EPS_F) {

				E = pm->k_EMAX * pm->const_fb_U;

				pm->i_derated_WEAK = E / (pm->lu_wS * pm->const_im_L2);

				/* Flux weakening control.
				 * */
				track_D += pm->weak_D;
			}
			else {
				pm->i_derated_WEAK = PM_MAX_F;
			}
		}
	}

	if (		pm->config_DRIVE == PM_DRIVE_CURRENT
			&& pm->config_LIMITED == PM_ENABLED) {

		/* Maximal SPEED limit.
		 * */
		if (pm->lu_wS < - pm->s_reverse) {

			E = - pm->s_reverse - pm->lu_wS;

			lerp_F = E / pm->s_linspan;
			lerp_F = (lerp_F > 1.f) ? 1.f : lerp_F;

			track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp_F;
		}
		else if (pm->lu_wS > pm->s_maximal) {

			E = pm->s_maximal - pm->lu_wS;

			lerp_F = - E / pm->s_linspan;
			lerp_F = (lerp_F > 1.f) ? 1.f : lerp_F;

			track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp_F;
		}

		/* Maximal acceleration limit.
		 * */
		dS = pm->s_accel * pm->dT;
		pm->s_track = (pm->s_track < pm->lu_wS - dS) ? pm->s_track + dS
			: (pm->s_track > pm->lu_wS + dS) ? pm->s_track - dS : pm->lu_wS;

		E = pm->s_track - pm->lu_wS;

		lerp_F = m_fabsf(E) / pm->s_linspan;
		lerp_F = (lerp_F > 1.f) ? 1.f : lerp_F;

		track_Q += (pm_form_iSP(pm, E) - track_Q) * lerp_F;
	}

	/* Get VSI voltages on DQ-axes.
	 * */
	uD = pm->lu_F[0] * pm->tvm_DX + pm->lu_F[1] * pm->tvm_DY;
	uQ = pm->lu_F[0] * pm->tvm_DY - pm->lu_F[1] * pm->tvm_DX;

	/* LPF is necessary to ensure the stability of POWER limiting loop.
	 * */
	pm->watt_lpf_D += (uD - pm->watt_lpf_D) * pm->watt_gain_LP_F;
	pm->watt_lpf_Q += (uQ - pm->watt_lpf_Q) * pm->watt_gain_LP_F;

	/* Operating POWER is a scalar product of voltage and current.
	 * */
	wP = pm->k_KWAT * (pm->lu_iD * pm->watt_lpf_D + pm->lu_iQ * pm->watt_lpf_Q);
	pm->watt_lpf_wP += (wP - pm->watt_lpf_wP) * pm->watt_gain_LP_P;

	/* Maximal CURRENT constraints.
	 * */
	iMAX = pm->i_maximal;
	iREV = - pm->i_reverse;

	if (		pm->config_WEAK == PM_ENABLED
			&& pm->weak_D < - M_EPS_F) {

		/* Flux weakening constraints.
		 * */
		track_Q = (track_Q > pm->i_derated_WEAK) ? pm->i_derated_WEAK
			: (track_Q < - pm->i_derated_WEAK) ? - pm->i_derated_WEAK : track_Q;
	}
	else {
		iMAX = (iMAX < pm->i_derated_PCB) ? iMAX : pm->i_derated_PCB;
		iREV = (iREV > - pm->i_derated_PCB) ? iREV : - pm->i_derated_PCB;

		if (pm->lu_mode == PM_LU_ESTIMATE_HFI) {

			iMAX = (iMAX < pm->i_derated_HFI) ? iMAX : pm->i_derated_HFI;
			iREV = (iREV > - pm->i_derated_HFI) ? iREV : - pm->i_derated_HFI;
		}
	}

	track_D = (track_D > iMAX) ? iMAX : (track_D < - iMAX) ? - iMAX : track_D;
	track_Q = (track_Q > iMAX) ? iMAX : (track_Q < iREV) ? iREV : track_Q;

	/* Prevent DC link OVERVOLTAGE.
	 * */
	track_Q = (pm->const_fb_U > pm->watt_dclink_HI) ? 0.f : track_Q;
	track_Q = (pm->const_fb_U < pm->watt_dclink_LO) ? 0.f : track_Q;

	/* Maximal POWER constraint.
	 * */
	wMAX = pm->watt_wP_maximal;
	wREV = - pm->watt_wP_reverse;

	/* Maximal DC link CURRENT constraint.
	 * */
	wP = pm->watt_iDC_maximal * pm->const_fb_U;
	wMAX = (wP < wMAX) ? wP : wMAX;
	wP = - pm->watt_iDC_reverse * pm->const_fb_U;
	wREV = (wP > wREV) ? wP : wREV;

	/* Apply POWER constraints (with D-axis priority).
	 * */
	wP = pm->k_KWAT * (track_D * pm->watt_lpf_D + track_Q * pm->watt_lpf_Q);

	if (wP > wMAX) {

		wP = pm->k_KWAT * track_D * pm->watt_lpf_D;

		if (wP > wMAX) {

			track_D *= wMAX / wP;
			track_Q = 0.f;
		}
		else {
			wMAX -= wP;
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP > M_EPS_F) {

				track_Q *= wMAX / wP;
			}
		}
	}
	else if (wP < wREV) {

		wP = pm->k_KWAT * track_D * pm->watt_lpf_D;

		if (wP < wREV) {

			track_D *= wREV / wP;
			track_Q = 0.f;
		}
		else {
			wREV -= wP;
			wP = pm->k_KWAT * track_Q * pm->watt_lpf_Q;

			if (wP < - M_EPS_F) {

				track_Q *= wREV / wP;
			}
		}
	}

	/* Now determine whether to inject HF.
	 * */
	mode_HFI = (pm->lu_mode == PM_LU_ESTIMATE_HFI) ? PM_ENABLED : PM_DISABLED;

	if (PM_CONFIG_DEBUG(pm) == PM_ENABLED) {

		mode_HFI = (pm->debug_locked_HFI == PM_ENABLED) ? PM_ENABLED : mode_HFI;
	}

	pm->i_slew_dT += pm->dT;

	if (mode_HFI != PM_ENABLED || pm->hfi_DFT_N == 0) {

		E = pm->i_slew_rate * pm->i_slew_dT;

		/* Update once per DFT block.
		 * */
		pm->i_track_D = (pm->i_track_D < track_D - E) ? pm->i_track_D + E
			: (pm->i_track_D > track_D + E) ? pm->i_track_D - E : track_D;
		pm->i_track_Q = (pm->i_track_Q < track_Q - E) ? pm->i_track_Q + E
			: (pm->i_track_Q > track_Q + E) ? pm->i_track_Q - E : track_Q;

		pm->i_slew_dT = 0.f;
	}

	/* Obtain discrepancy in DQ-axes.
	 * */
	eD = pm->i_track_D - pm->lu_iD;
	eQ = pm->i_track_Q - pm->lu_iQ;

	/* There is a DEAD zone.
	 * */
	eD = (m_fabsf(eD) > pm->i_tol_Z) ? eD : 0.f;
	eQ = (m_fabsf(eQ) > pm->i_tol_Z) ? eQ : 0.f;

	uD = pm->i_gain_P * eD;
	uQ = pm->i_gain_P * eQ;

	/* Feed forward compensation (R).
	 * */
	uD += pm->const_R * pm->i_track_D;
	uQ += pm->const_R * pm->i_track_Q;

	/* Feed forward compensation (L).
	 * */
	uD += - pm->lu_wS * pm->const_im_L2 * pm->i_track_Q;
	uQ += pm->lu_wS * (pm->const_im_L1 * pm->i_track_D + pm->const_E);

	uMAX = pm->k_UMAX * pm->const_fb_U;

	pm->i_integral_D += pm->i_gain_I * eD;
	pm->i_integral_D = (pm->i_integral_D > uMAX) ? uMAX :
		(pm->i_integral_D < - uMAX) ? - uMAX : pm->i_integral_D;
	uD += pm->i_integral_D;

	pm->i_integral_Q += pm->i_gain_I * eQ;
	pm->i_integral_Q = (pm->i_integral_Q > uMAX) ? uMAX :
		(pm->i_integral_Q < - uMAX) ? - uMAX : pm->i_integral_Q;
	uQ += pm->i_integral_Q;

	/* Output voltage CLAMP.
	 * */
	uD = (uD > uMAX) ? uMAX : (uD < - uMAX) ? - uMAX : uD;
	uQ = (uQ > uMAX) ? uMAX : (uQ < - uMAX) ? - uMAX : uQ;

	uMAX = pm->k_EMAX * pm->v_maximal;
	uREV = - pm->k_EMAX * pm->v_reverse;

	/* Output voltage (Q) specified constraint.
	 * */
	uQ = (uQ > uMAX) ? uMAX : (uQ < uREV) ? uREV : uQ;

	if (mode_HFI == PM_ENABLED) {

		/* HF injection.
		 * */
		hfi_U = pm->hfi_inject_sine * pm->temp_HFI_wS * pm->const_im_L1;

		uD += pm->hfi_wave[0] * hfi_U;
		uQ += pm->hfi_wave[1] * hfi_U;
	}

	/* Go to XY-axes.
	 * */
	uX = pm->lu_F[0] * uD - pm->lu_F[1] * uQ;
	uY = pm->lu_F[1] * uD + pm->lu_F[0] * uQ;

	pm_voltage(pm, uX, uY);
}

static void
pm_loop_speed(pmc_t *pm)
{
	float		wSP, eS, dS;

	wSP = pm->s_setpoint_speed;

	/* Maximal speed constraint.
	 * */
	wSP = (wSP > pm->s_maximal) ? pm->s_maximal :
		(wSP < - pm->s_reverse) ? - pm->s_reverse : wSP;

	if (pm->config_DRIVE == PM_DRIVE_SPEED) {

		/* Maximal acceleration constraint.
		 * */
		dS = pm->s_accel * pm->dT;
		pm->s_track = (pm->s_track < wSP - dS) ? pm->s_track + dS
			: (pm->s_track > wSP + dS) ? pm->s_track - dS : wSP;
	}
	else {
		/* No constraint required.
		 * */
		pm->s_track = wSP;
	}

	if (pm->lu_mode == PM_LU_FORCED) {

		pm->s_track = pm->forced_wS;
		pm->s_iSP = 0.f;
	}
	else {
		/* Get speed discrepancy.
		 * */
		eS = pm->s_track - pm->lu_wS;

		/* Update current loop SETPOINT.
		 * */
		pm->s_iSP = pm_form_iSP(pm, eS);
	}
}

static void
pm_loop_servo(pmc_t *pm)
{
	float		EX, EY, eP, eP_abs, wSP, lerp_S;

	/* Get position discrepancy.
	 * */
	EX = pm->x_setpoint_F[0] * pm->lu_F[0] + pm->x_setpoint_F[1] * pm->lu_F[1];
	EY = pm->x_setpoint_F[1] * pm->lu_F[0] - pm->x_setpoint_F[0] * pm->lu_F[1];

	eP = m_atan2f(EY, EX);

	if (EY < 0.f) {

		if (pm->lu_F[1] < 0.f && pm->x_setpoint_F[1] >= 0.f)
			eP += 2.f * M_PI_F;
	}
	else {
		if (pm->lu_F[1] >= 0.f && pm->x_setpoint_F[1] < 0.f)
			eP -= 2.f * M_PI_F;
	}

	eP += (pm->x_setpoint_revol - pm->lu_revol) * 2.f * M_PI_F;
	eP_abs = m_fabsf(eP);

	pm->x_residual = eP;

	/* Servo is based on constant acceleration formula.
	 * */
	eP = (eP < 0.f) ? - m_sqrtf(eP_abs) : m_sqrtf(eP_abs);

	/* There is a DEAD zone.
	 * */
	eP = (eP_abs > pm->x_tol_Z) ? eP : 0.f;

	/* Slow down in NEAR zone.
	 * */
	lerp_S = (eP_abs < pm->x_tol_N) ? eP_abs / pm->x_tol_N : 1.f;
	lerp_S = pm->x_gain_N + (1.f - pm->x_gain_N) * lerp_S;

	wSP = eP * pm->x_gain_P * lerp_S + pm->x_setpoint_speed;

	/* Update speed loop SETPOINT.
	 * */
	pm->s_setpoint_speed = wSP;
}

static void
pm_infometer(pmc_t *pm)
{
	float		Wh, Ah, fuel;
	int		rel, relqu = 4;

	rel = pm->lu_revol - pm->im_base_revol;

	if (rel < - relqu) {

		pm->im_total_revol += - rel;
		pm->im_base_revol = pm->lu_revol;
	}
	else if (rel > relqu) {

		pm->im_total_revol += rel;
		pm->im_base_revol = pm->lu_revol;
	}

	/* Traveled distance (m).
	 * */
	pm->im_distance = (float) pm->im_total_revol
		* pm->const_ld_S / (float) pm->const_Zp;

	/* Get WATT per HOUR.
	 * */
	Wh = pm->watt_lpf_wP * pm->dT * (1.f / 3600.f);
	Ah = Wh * pm->temp_const_ifbU;

	if (Wh > 0.f) {

		m_rsumf(&pm->im_consumed_Wh, &pm->im_REM[0], Wh);
		m_rsumf(&pm->im_consumed_Ah, &pm->im_REM[1], Ah);
	}
	else {
		m_rsumf(&pm->im_reverted_Wh, &pm->im_REM[2], - Wh);
		m_rsumf(&pm->im_reverted_Ah, &pm->im_REM[3], - Ah);
	}

	/* Fuel gauge.
	 * */
	if (pm->im_capacity_Ah > M_EPS_F) {

		fuel = (pm->im_capacity_Ah - pm->im_consumed_Ah
				+ pm->im_reverted_Ah) / pm->im_capacity_Ah;

		pm->im_fuel_pc = 100.f * fuel;
	}
}

void pm_feedback(pmc_t *pm, pmfb_t *fb)
{
	float		vA, vB, vC, Q;

	if (		pm->lu_mode != PM_LU_DISABLED
			&& (pm->vsi_AF | pm->vsi_BF) != 0) {

		m_rotatef(pm->lu_F, pm->lu_wS * pm->dT);
		m_normalizef(pm->lu_F);

		pm->lu_iX = pm->lu_F[0] * pm->lu_iD - pm->lu_F[1] * pm->lu_iQ;
		pm->lu_iY = pm->lu_F[1] * pm->lu_iD + pm->lu_F[0] * pm->lu_iQ;

		if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

			pm->fb_iA = pm->lu_iX;
			pm->fb_iB = - .5f * pm->lu_iX + .8660254f * pm->lu_iY;
		}
		else {
			pm->fb_iA = pm->lu_iX;
			pm->fb_iB = pm->lu_iY;
		}
	}

	if (pm->vsi_AF == 0) {

		/* Get inline current A.
		 * */
		pm->fb_iA = pm->ad_IA[1] * fb->current_A + pm->ad_IA[0];

		if (m_fabsf(pm->fb_iA) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INLINE_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (pm->vsi_BF == 0) {

		/* Get inline current B.
		 * */
		pm->fb_iB = pm->ad_IB[1] * fb->current_B + pm->ad_IB[0];

		if (m_fabsf(pm->fb_iB) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INLINE_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		pm->lu_iX = pm->fb_iA;
		pm->lu_iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;
	}
	else {
		pm->lu_iX = pm->fb_iA;
		pm->lu_iY = pm->fb_iB;
	}

	if (pm->vsi_SF == 0) {

		/* Get DC link voltage.
		 * */
		pm->const_fb_U = pm->ad_US[1] * fb->voltage_U + pm->ad_US[0];
		pm->temp_const_ifbU = 1.f / pm->const_fb_U;

		if (		pm->const_fb_U > pm->fault_voltage_halt
				&& pm->weak_D > - M_EPS_F) {

			pm->fsm_errno = PM_ERROR_DC_LINK_OVERVOLTAGE;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		vA = pm->tvm_FIR_A[1] * pm->fb_uA;
		vB = pm->tvm_FIR_B[1] * pm->fb_uB;
		vC = pm->tvm_FIR_C[1] * pm->fb_uC;

		/* Get instant terminal voltages.
		 * */
		pm->fb_uA = pm->ad_UA[1] * fb->voltage_A + pm->ad_UA[0];
		pm->fb_uB = pm->ad_UB[1] * fb->voltage_B + pm->ad_UB[0];
		pm->fb_uC = pm->ad_UC[1] * fb->voltage_C + pm->ad_UC[0];

		if (		pm->lu_mode != PM_LU_DETACHED
				&& pm->tvm_ALLOWED == PM_ENABLED
				&& pm->vsi_UF == 0) {

			/* Extract actual terminal voltages with FIR.
			 * */

			if (pm->vsi_AZ != 0) {

				vA += pm->tvm_FIR_A[0] * pm->fb_uA + pm->tvm_FIR_A[2];
			}
			else {
				vA = 0.f;
			}

			if (pm->vsi_BZ != 0) {

				vB += pm->tvm_FIR_B[0] * pm->fb_uB + pm->tvm_FIR_B[2];
			}
			else {
				vB = 0.f;
			}

			if (pm->vsi_CZ != 0) {

				vC += pm->tvm_FIR_C[0] * pm->fb_uC + pm->tvm_FIR_C[2];
			}
			else {
				vC = 0.f;
			}

			pm->tvm_A = vA;
			pm->tvm_B = vB;
			pm->tvm_C = vC;

			if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

				Q = (1.f / 3.f) * (vA + vB + vC);
				vA = vA - Q;
				vB = vB - Q;

				pm->tvm_DX = vA;
				pm->tvm_DY = .57735027f * vA + 1.1547005f * vB;
			}
			else {
				vA = vA - vC;
				vB = vB - vC;

				pm->tvm_DX = vA;
				pm->tvm_DY = vB;
			}
		}
		else {
			pm->tvm_DX = pm->vsi_DX;
			pm->tvm_DY = pm->vsi_DY;
		}
	}
	else {
		/* This fallback makes possible the use of TVM variables in
		 * case of no TVM configuration.
		 * */
		pm->tvm_DX = pm->vsi_DX;
		pm->tvm_DY = pm->vsi_DY;
	}

	/* Get SENSOR values.
	 * */
	pm->fb_HS = fb->pulse_HS;
	pm->fb_EP = fb->pulse_EP;

	/* Main FSM is used to execute external commands.
	 * */
	pm_FSM(pm);

	if (pm->lu_mode != PM_LU_DISABLED) {

		/* The observer FSM.
		 * */
		pm_lu_FSM(pm);

		if (pm->lu_mode == PM_LU_DETACHED) {

			pm_voltage(pm, pm->detach_X, pm->detach_Y);
		}
		else {
			if (pm->config_DRIVE == PM_DRIVE_SPEED) {

				pm_loop_speed(pm);
			}
			else if (pm->config_DRIVE == PM_DRIVE_SERVO) {

				pm_loop_servo(pm);
				pm_loop_speed(pm);
			}

			/* Current loop is always enabled.
			 * */
			pm_loop_current(pm);
		}

		if (pm->config_INFO == PM_ENABLED) {

			/* To collect information.
			 * */
			pm_infometer(pm);
		}

		if (m_isfinitef(pm->lu_F[0]) == 0) {

			pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			pm->fsm_state = PM_STATE_HALT;
		}
	}
}

