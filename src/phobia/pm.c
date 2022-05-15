#include "libm.h"
#include "pm.h"

void pm_build(pmc_t *pm)
{
	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		pm->k_UMAX = .66666667f;
		pm->k_EMAX = .57735027f;
		pm->k_KWAT = 1.5f;
	}
	else {
		pm->k_UMAX = 1.f;
		pm->k_EMAX = .70710678f;
		pm->k_KWAT = 1.f;
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

	pm->quick_iE = (pm->const_E > M_EPS_F) ? 1.f / pm->const_E : 0.f;
	pm->quick_iEq = pm->quick_iE * pm->quick_iE;
	pm->quick_iL1 = (pm->const_im_L1 > M_EPS_F) ? 1.f / pm->const_im_L1 : 0.f;
	pm->quick_iL2 = (pm->const_im_L2 > M_EPS_F) ? 1.f / pm->const_im_L2 : 0.f;

	pm->quick_hfwS = 2.f * M_PI_F * pm->freq_hz / (float) pm->hfi_INJS;
	pm->quick_hfSC[0] = m_cosf(pm->quick_hfwS * pm->dT * .5f);
	pm->quick_hfSC[1] = m_sinf(pm->quick_hfwS * pm->dT * .5f);

	pm->quick_ZiEP = 2.f * M_PI_F * (float) (pm->const_Zp * pm->abi_gear_ZS)
		/ (float) (pm->abi_gear_ZQ * pm->abi_EPPR);

	pm->quick_ZiSQ = (float) (pm->const_Zp * pm->abi_gear_ZS)
		/ (float) pm->abi_gear_ZQ;
}

static void
pm_tune_all_default(pmc_t *pm)
{
	pm->dc_minimal = 0.2f;		/* (us) */
	pm->dc_clearance = 5.0f;	/* (us) */
	pm->dc_skip = 2.0f;		/* (us) */
	pm->dc_bootstrap = 100.f;	/* (ms) */
	pm->dc_clamped = 1.f;		/* (s)  */

	pm->config_NOP = PM_NOP_THREE_PHASE;
	pm->config_IFB = PM_IFB_ABC_INLINE;
	pm->config_TVM = PM_ENABLED;
	pm->config_DEBUG = PM_DISABLED;

	pm->config_VSI_CIRCULAR = PM_DISABLED;
	pm->config_VSI_PRECISE = PM_DISABLED;
	pm->config_LU_FORCED = PM_ENABLED;
	pm->config_LU_ESTIMATE_FLUX = PM_ESTIMATE_ORTEGA;
	pm->config_LU_ESTIMATE_HFI = PM_DISABLED;
	pm->config_LU_SENSOR_HALL = PM_DISABLED;
	pm->config_LU_SENSOR_ABI = PM_DISABLED;
	pm->config_LU_SENSOR_SINCOS = PM_DISABLED;
	pm->config_LU_LOCATION = PM_LOCATION_INHERITED;
	pm->config_LU_DRIVE = PM_DRIVE_SPEED;
	pm->config_HFI_MAJOR_AXES = PM_DISABLED;
	pm->config_ABI_ABSOLUTE = PM_DISABLED;
	pm->config_SINCOS_FRONTEND = PM_SINCOS_PLAIN;
	pm->config_HOLDING_BRAKE = PM_DISABLED;
	pm->config_SPEED_LIMITED = PM_ENABLED;
	pm->config_MTPA_RELUCTANCE = PM_DISABLED;
	pm->config_WEAKENING = PM_DISABLED;
	pm->config_MILEAGE_INFO	= PM_ENABLED;
	pm->config_BOOST_CHARGE = PM_DISABLED;

	pm->tm_transient_slow = 50.f;	/* (ms) */
	pm->tm_transient_fast = 2.f;	/* (ms) */
	pm->tm_voltage_hold = 100.f;	/* (ms) */
	pm->tm_current_hold = 500.f;	/* (ms) */
	pm->tm_instant_probe = 2.f;	/* (ms) */
	pm->tm_average_probe = 500.f;	/* (ms) */
	pm->tm_average_drift = 100.f;	/* (ms) */
	pm->tm_average_inertia = 700.f;	/* (ms) */
	pm->tm_startup = 100.f;		/* (ms) */
	pm->tm_halt_pause = 1000.f;	/* (ms) */

	pm->ad_IA[0] = 0.f;
	pm->ad_IA[1] = 1.f;
	pm->ad_IB[0] = 0.f;
	pm->ad_IB[1] = 1.f;
	pm->ad_IC[0] = 0.f;
	pm->ad_IC[1] = 1.f;
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
	pm->probe_freq_sine_hz = 1000.f;
	pm->probe_speed_hold = 900.f;
	pm->probe_speed_detached = 50.f;
	pm->probe_gain_P = 1E-2f;
	pm->probe_gain_I = 1E-3f;

	pm->fault_voltage_tol = 4.f;
	pm->fault_current_tol = 4.f;
	pm->fault_accuracy_tol = .1f;
	pm->fault_terminal_tol = 0.090f;
	pm->fault_current_halt = 156.f;
	pm->fault_voltage_halt = 57.f;

	pm->vsi_gain_LP_F = 5E-3f;

	pm->tvm_INUSE = PM_DISABLED;
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

	pm->lu_transient = 800.f;
	pm->lu_gain_TQ = 5E-4f;

	pm->forced_hold_D = 10.f;
	pm->forced_maximal = 900.f;
	pm->forced_reverse = pm->forced_maximal;
	pm->forced_accel = 200.f;
	pm->forced_maximal_DC = .7f;

	pm->detach_level_U = 1.f;
	pm->detach_trip_AD = 2E-1f;
	pm->detach_gain_SF = 5E-2f;

	pm->flux_trip_AD = 2E-1f;
	pm->flux_gain_IN = 5E-4f;
	pm->flux_gain_LO = 2E-5f;
	pm->flux_gain_HI = 5E-5f;
	pm->flux_gain_SF = 5E-2f;
	pm->flux_gain_IF = 5E-1f;

	pm->flux_gain_DA = 5E-1f;
	pm->flux_gain_QA = 5E-1f;
	pm->flux_gain_DP = 5E-3f;
	pm->flux_gain_DS = 2E+0f;
	pm->flux_gain_QS = 2E+1f;
	pm->flux_gain_QZ = 5E-3f;

	pm->zone_MPPE = 40.f;
	pm->zone_MURE = 80.f;
	pm->zone_gain_TA = 4.f;
	pm->zone_gain_GI = 1.f;
	pm->zone_gain_ES = .2f;
	pm->zone_gain_LP_S = 5E-2f;

	pm->hfi_inject_sine = 2.f;
	pm->hfi_maximal = 900.f;
	pm->hfi_INJS = 9;
	pm->hfi_SKIP = 1;
	pm->hfi_ESTI = 5;
	pm->hfi_TORQ = 5;
	pm->hfi_POLA = 0;
	pm->hfi_gain_SF = 5E-3f;
	pm->hfi_gain_IF = 1E-0f;

	pm->hall_INUSE = PM_DISABLED;
	pm->hall_prol_ms = 100.f;	/* (ms) */
	pm->hall_gain_PF = 1E-0f;
	pm->hall_gain_SF = 5E-3f;
	pm->hall_gain_IF = 1E-0f;

	pm->abi_INUSE = PM_DISABLED;
	pm->abi_EPPR = 2400;
	pm->abi_gear_ZS = 1;
	pm->abi_gear_ZQ = 1;
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
	pm->watt_dclink_HI = 52.f;
	pm->watt_dclink_LO = 7.f;
	pm->watt_gain_LP_F = 5E-2f;
	pm->watt_gain_LP_P = 5E-2f;

	pm->i_maximal = 120.f;
	pm->i_reverse = pm->i_maximal;
	pm->i_slew_rate = 7000.f;
	pm->i_tol_Z = 0.f;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->weak_maximal = 30.f;
	pm->weak_gain_EU = 5E-2f;

	pm->v_maximal = 60.f;
	pm->v_reverse = pm->v_maximal;

	pm->s_maximal = 15000.f;
	pm->s_reverse = pm->s_maximal;
	pm->s_accel = 7000.f;
	pm->s_linspan = 100.f;
	pm->s_tol_Z = 0.f;
	pm->s_gain_P = 5E-2f;
	pm->s_gain_H = 5E-1f;

	pm->x_tol_NEAR = 1.f;
	pm->x_tol_Z = 0.f;
	pm->x_gain_P = 35.f;
	pm->x_gain_Z = 5.f;

	pm->boost_gain_P = 1E-1f;
	pm->boost_gain_I = 1E-3f;
}

static void
pm_tune_probe_default(pmc_t *pm)
{
	pm->forced_accel = 200.f;

	pm->zone_MPPE = 40.f;
	pm->zone_MURE = 80.f;

	pm->const_E = 0.f;
	pm->const_R = 0.f;
	pm->const_Ja = 0.f;
	pm->const_im_L1 = 0.f;
	pm->const_im_L2 = 0.f;

	pm->i_maximal = 120.f;
	pm->i_reverse = pm->i_maximal;
	pm->i_slew_rate = 4000.f;
	pm->i_gain_P = 2E-1f;
	pm->i_gain_I = 5E-3f;

	pm->lu_gain_TQ = 5E-4f;
	pm->s_gain_P = 5E-2f;
}

static void
pm_tune_maximal_current(pmc_t *pm)
{
	float			maximal_A, new_A;

	maximal_A = pm->fault_current_halt * .9f;

	if (pm->const_R > M_EPS_F) {

		/* Based on DC link voltage.
		 * */
		new_A = pm->k_UMAX * pm->const_fb_U / pm->const_R;
		maximal_A = (new_A < maximal_A) ? new_A : maximal_A;

		/* Based on resistive LOSSES.
		 * */
		new_A = m_sqrtf(.07f * pm->watt_wP_maximal / pm->const_R);
		maximal_A = (new_A < maximal_A) ? new_A : maximal_A;
	}

	if (maximal_A < pm->i_maximal) {

		/* Get the maximal inline current.
		 * */
		pm->i_maximal = (float) (int) maximal_A;
		pm->i_reverse = pm->i_maximal;
	}
}

static void
pm_tune_loop_current(pmc_t *pm)
{
	float		Lm, Kp, Ki;

	if (		   pm->const_im_L1 > M_EPS_F
			&& pm->const_im_L1 > M_EPS_F) {

		Lm = (pm->const_im_L1 < pm->const_im_L2)
			? pm->const_im_L1 : pm->const_im_L2;

		/* Tune current loop based on state-space model.
		 *
		 *          [1-R*T/L-Kp*T/L  -Ki*T/L]
		 * x(k+1) = [1                1     ] * x(k)
		 *
		 * */
		Kp = .5f * Lm * pm->freq_hz - pm->const_R;
		Ki = .02f * Lm * pm->freq_hz;

		pm->i_gain_P = (Kp > 0.f) ? Kp : 0.f;
		pm->i_gain_I = Ki;

		/* Get the current slew rate limit.
		 * */
		pm->i_slew_rate = .05f * pm->const_fb_U / Lm;
	}
}

static void
pm_tune_zone_threshold(pmc_t *pm)
{
	float		iSTD, vSTD, uTMP, wSTD, wMAX;

	if (		   pm->const_im_L1 > M_EPS_F
			&& pm->const_im_L1 > M_EPS_F
			&& pm->const_E > M_EPS_F) {

		/* Current measurement accuracy (standard deviation).
		 * */
		iSTD = .3f;

		if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

			/* TVM measurement accuracy (standard deviation).
			 * */
			vSTD = .2f;
		}
		else {
			/* Terminal voltage accuracy based on PWM edges distortion.
			 * */
			vSTD = pm->dc_minimal * (1.f / 1000000.f)
				* pm->freq_hz * pm->const_fb_U;
		}

		/* Calculate speed estimate STD from the FLUX equations.
		 * */
		uTMP = iSTD * pm->const_R;
		wSTD = m_sqrtf(vSTD * vSTD + uTMP * uTMP) * pm->dT;

		uTMP = iSTD * pm->const_im_L2;
		wSTD = m_sqrtf(wSTD * wSTD + uTMP * uTMP) / pm->const_E;

		wSTD *= pm->freq_hz * pm->flux_gain_SF;

		/* Take the value of random speed estimate error (peak to peak).
		 * */
		pm->zone_MPPE = wSTD * 5.f;

		/* Take the value of uncertainty due to resistance error.
		 * */
		pm->zone_MURE = .2f * pm->i_maximal * pm->const_R / pm->const_E;

		/* Maximal allowed MURE.
		 * */
		wMAX = .4f * pm->forced_maximal;

		if (pm->zone_MURE < 40.f) {

			/* We do not believe the motor is so GOOD.
			 * */
			pm->zone_MURE = 40.f;
		}
		else if (pm->zone_MURE > wMAX) {

			pm->zone_MURE = wMAX;
		}
	}
}

static void
pm_tune_loop_forced(pmc_t *pm)
{
	if (pm->const_Ja > M_EPS_F) {

		/* Tune forced control based on motor constants.
		 * */
		pm->forced_accel = .2f * pm->forced_hold_D / pm->const_Ja;
	}
}

static void
pm_tune_loop_speed(pmc_t *pm)
{
	if (		   pm->zone_MPPE > M_EPS_F
			&& pm->const_Ja > M_EPS_F) {

		/* Tune load torque estimate.
		 * */
		pm->lu_gain_TQ = 5.f / (pm->zone_MPPE * pm->freq_hz * pm->const_Ja);

		/* Tune speed loop based on MPPE value.
		 * */
		pm->s_gain_P = 2.f / pm->zone_MPPE;
	}
}

void pm_tune(pmc_t *pm, int req)
{
	switch (req) {

		case PM_TUNE_ALL_DEFAULT:
			pm_tune_all_default(pm);
			break;

		case PM_TUNE_PROBE_DEFAULT:
			pm_tune_probe_default(pm);
			break;

		case PM_TUNE_MAXIMAL_CURRENT:
			pm_tune_maximal_current(pm);
			break;

		case PM_TUNE_LOOP_CURRENT:
			pm_tune_loop_current(pm);
			break;

		case PM_TUNE_ZONE_THRESHOLD:
			pm_tune_zone_threshold(pm);
			break;

		case PM_TUNE_LOOP_FORCED:
			pm_tune_loop_forced(pm);
			break;

		case PM_TUNE_LOOP_SPEED:
			pm_tune_loop_speed(pm);
			break;

		default:
			break;
	}
}

static void
pm_forced(pmc_t *pm)
{
	float		wSP, dS;

	/* Get the SETPOINT of forced speed.
	 * */
	if (pm->config_LU_DRIVE == PM_DRIVE_CURRENT) {

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

	if (pm->vsi_lpf_DC > pm->forced_maximal_DC) {

		/* We are unable to keep such a high speed at
		 * this DC link voltage. Stop.
		 * */
		wSP = 0.f;
	}

	/* Update actual speed with specified acceleration.
	 * */
	dS = pm->forced_accel * pm->dT;
	pm->forced_wS = (pm->forced_wS < wSP - dS) ? pm->forced_wS + dS :
		(pm->forced_wS > wSP + dS) ? pm->forced_wS - dS : wSP;

	/* Update DQ-axes.
	 * */
	m_rotatef(pm->forced_F, pm->forced_wS * pm->dT);
}

static void
pm_detached_BEMF(pmc_t *pm)
{
	float		uA, uB, uC, uX, uY, U, EX, EY, E, S;

	/* Get BEMF voltage.
	 * */
	uA = pm->fb_uA;
	uB = pm->fb_uB;
	uC = pm->fb_uC;

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		U = .33333333f * (uA + uB + uC);
		uA = uA - U;
		uB = uB - U;

		uX = uA;
		uY = .57735027f * uA + 1.1547005f * uB;
	}
	else {
		uX = uA - uC;
		uY = uB - uC;
	}

	pm->vsi_X = uX;
	pm->vsi_Y = uY;

	/* Absolute BEMF voltage.
	 * */
	U = m_sqrtf(uX * uX + uY * uY);

	if (U > pm->detach_level_U) {

		E = 1.f / U;

		uX *= E;
		uY *= E;

		if (pm->detach_LOCK != 0) {

			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->flux_X, pm->flux_wS * pm->dT);

			EX = uX * pm->flux_X[0] + uY * pm->flux_X[1];
			EY = uY * pm->flux_X[0] - uX * pm->flux_X[1];

			if (EX > M_EPS_F) {

				E = EY * pm->freq_hz;

				S = U * pm->detach_trip_AD;
				S = (S > 1.f) ? 1.f : S;

				pm->flux_wS += E * S * pm->detach_gain_SF;
			}

			pm->flux_E = U / m_fabsf(pm->flux_wS);

			E = (pm->flux_wS < 0.f) ? - 1.f : 1.f;

			pm->flux_F[0] = uY * E;
			pm->flux_F[1] = - uX * E;
		}

		pm->flux_X[0] = uX;
		pm->flux_X[1] = uY;

		pm->detach_LOCK++;
	}
	else {
		pm->flux_wS = 0.f;

		pm->detach_LOCK = 0;
	}
}

static void
pm_flux_ORTEGA(pmc_t *pm)
{
	float		uX, uY, lX, lY, EX, EY, DX, DY, S, E;

	/* Get the actual voltage.
	 * */
	uX = pm->vsi_X - pm->const_R * pm->lu_iX;
	uY = pm->vsi_Y - pm->const_R * pm->lu_iY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		uX += pm->tvm_DX - pm->vsi_DX;
		uY += pm->tvm_DY - pm->vsi_DY;
	}

	/* Stator FLUX.
	 * */
	lX = pm->const_im_L2 * pm->lu_iX;
	lY = pm->const_im_L2 * pm->lu_iY;

	if (pm->const_E > M_EPS_F) {

		/* FLUX equations.
		 * */
		pm->flux_X[0] += uX * pm->dT;
		pm->flux_X[1] += uY * pm->dT;

		EX = pm->flux_X[0] - lX;
		EY = pm->flux_X[1] - lY;

		S = m_fabsf(pm->flux_wS * pm->const_E) * pm->flux_trip_AD;
		S = (S > 1.f) ? 1.f : S;

		/* Get the flux RESIDUE.
		 * */
		E = 1.f - (EX * EX + EY * EY) * pm->quick_iEq;

		/* Adaptive GAIN.
		 * */
		E *=	  pm->flux_gain_HI * S
			+ pm->flux_gain_LO * (1.f - S);

		pm->flux_X[0] += EX * E * pm->quick_iE;
		pm->flux_X[1] += EY * E * pm->quick_iE;
	}
	else {
		/* Startup estimation.
		 * */
		pm->flux_X[0] += uX * pm->dT;
		pm->flux_X[1] += uY * pm->dT;

		EX = pm->flux_X[0] - lX;
		EY = pm->flux_X[1] - lY;

		E = - pm->flux_gain_IN;

		pm->flux_X[0] += EX * E;
		pm->flux_X[1] += EY * E;
	}

	/* Extract the rotor FLUX linkage.
	 * */
	EX = pm->flux_X[0] - lX;
	EY = pm->flux_X[1] - lY;

	E = m_sqrtf(EX * EX + EY * EY);

	pm->flux_E = E;

	if (E > M_EPS_F) {

		E = 1.f / E;

		EX *= E;
		EY *= E;

		if (		pm->lu_MODE == PM_LU_FORCED
				&& pm->const_E < M_EPS_F) {

			pm->flux_wS = pm->forced_wS;
		}
		else {
			/* Speed estimation (PLL).
			 * */
			m_rotatef(pm->flux_F, pm->flux_wS * pm->dT);

			DX = EX * pm->flux_F[0] + EY * pm->flux_F[1];
			DY = EY * pm->flux_F[0] - EX * pm->flux_F[1];

			if (DX > M_EPS_F) {

				E = DY * pm->freq_hz;
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
}

static void
pm_flux_equation(pmc_t *pm, float Y[2], const float X[2], const float F[2])
{
        float           uD, uQ, R1, E1, flux_D, flux_Q;

        uD = F[0] * pm->vsi_X + F[1] * pm->vsi_Y;
        uQ = F[0] * pm->vsi_Y - F[1] * pm->vsi_X;

        R1 = pm->const_R;
        E1 = pm->const_E;

	uQ += pm->flux_QZ;

        flux_D = pm->const_im_L1 * X[0] + E1;
        flux_Q = pm->const_im_L2 * X[1];

        Y[0] = (uD - R1 * X[0] + flux_Q * pm->flux_wS) * pm->quick_iL1;
        Y[1] = (uQ - R1 * X[1] - flux_D * pm->flux_wS) * pm->quick_iL2;
}

static void
pm_flux_solve(pmc_t *pm, float X[2], float F[2], float wS)
{
        float           Y1[2], Y2[2];

        /* Second-order ODE solver.
         * */

        pm_flux_equation(pm, Y1, X, F);

        X[0] += Y1[0] * pm->dT;
        X[1] += Y1[1] * pm->dT;

        m_rotatef(F, wS * pm->dT);

        pm_flux_equation(pm, Y2, X, F);

        X[0] += (Y2[0] - Y1[0]) * pm->dT * .5f;
        X[1] += (Y2[1] - Y1[1]) * pm->dT * .5f;
}

static void
pm_flux_solve_TVM(pmc_t *pm, float X[2], float F[2])
{
        float           uX, uY, uD, uQ;

        /* First-order ODE solver.
         * */

        uX = pm->tvm_DX - pm->vsi_DX;
        uY = pm->tvm_DY - pm->vsi_DY;

        uD = F[0] * uX + F[1] * uY;
        uQ = F[0] * uY - F[1] * uX;

        X[0] += uD * pm->dT * pm->quick_iL1;
        X[1] += uQ * pm->dT * pm->quick_iL2;
}

static void
pm_flux_LUENBERGER(pmc_t *pm)
{
	float		eD, eQ, eR, E, S;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		pm_flux_solve_TVM(pm, pm->flux_X, pm->flux_F);
	}

	if (pm->vsi_XXF == 0) {

		/* Get the current RESIDUE in DQ-axes.
                 * */
		eD = pm->flux_F[0] * pm->lu_iX + pm->flux_F[1] * pm->lu_iY - pm->flux_X[0];
		eQ = pm->flux_F[0] * pm->lu_iY - pm->flux_F[1] * pm->lu_iX - pm->flux_X[1];

		/* Update internal current estimate.
		 * */
		pm->flux_X[0] += pm->flux_gain_DA * eD;
		pm->flux_X[1] += pm->flux_gain_QA * eQ;

		if (		pm->lu_MODE == PM_LU_FORCED
				&& pm->const_E < M_EPS_F) {

			pm->flux_F[0] = pm->forced_F[0];
			pm->flux_F[1] = pm->forced_F[1];
			pm->flux_wS = pm->forced_wS;

			/* Estimate QZ shift to get initial E constant.
			 * */
			pm->flux_QZ += pm->flux_gain_QZ * eQ;
		}
		else {
			eR = (pm->flux_wS < 0.f) ? - eD : eD;

			E = pm->flux_gain_DP * eR;
			E = (E < - 1.f) ? - 1.f : (E > 1.f) ? 1.f : E;

			/* Update DQ frame estimate with D residue.
			 * */
			m_rotatef(pm->flux_F, E);

			S = m_fabsf(pm->flux_wS * pm->const_E) * pm->flux_trip_AD;
			S = (S > 1.f) ? 1.f : S;

			/* We prefer the Q residue at low speed as it gives a
			 * fast response. At high speed it may inject a ripple
			 * if BEMF is not pure sinusoidal so we suppress it and
			 * more and more go to D residue.
			 * */
			E =	  pm->flux_gain_DS * eR * S
				- pm->flux_gain_QS * eQ * (1.f - S);

			pm->flux_wS += E;

			if (		pm->flux_gain_IF > M_EPS_F
					&& pm->const_Ja > M_EPS_F) {

				pm->flux_wS += (pm->lu_iQ - pm->lu_lpf_torque)
					* pm->flux_gain_IF * pm->dT / pm->const_Ja;
			}

			if (pm->flux_ZONE == PM_ZONE_HIGH) {

				/* Estimate QZ shift at high speed to get relaxed solution.
				 * */
				pm->flux_QZ += pm->flux_gain_QZ * (eR + eQ);
			}
			else {
				/* At the low speed QZ shift should not be estimated.
				 * */
				pm->flux_QZ = 0.f;
			}
		}

		/* Calculate E constant correction.
		 * */
		pm->flux_E = (m_fabsf(pm->flux_wS) > M_EPS_F)
			? pm->const_E - pm->flux_QZ / pm->flux_wS : 0.f;
	}

	/* Time update to the next cycle.
         * */
        pm_flux_solve(pm, pm->flux_X, pm->flux_F, pm->flux_wS);
}

static void
pm_flux_ZONE(pmc_t *pm)
{
	float			lev_wS, lev_E;
	int			lev_TIM;

	/* Get smooth speed passed through LPF.
	 * */
	pm->zone_lpf_wS += (pm->flux_wS - pm->zone_lpf_wS) * pm->zone_gain_LP_S;

	if (		   pm->flux_ZONE == PM_ZONE_NONE
			|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

		lev_wS = pm->zone_MURE + pm->zone_gain_TA * pm->zone_MPPE;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			lev_TIM = PM_TSMS(pm, pm->tm_startup);

			if (		m_fabsf(pm->zone_lpf_wS) > lev_wS
					&& pm->detach_LOCK > lev_TIM) {

				pm->flux_ZONE = PM_ZONE_HIGH;
			}
		}
		else {
			lev_E = pm->zone_gain_ES * pm->const_E;

			if (m_fabsf(pm->flux_E - pm->const_E) < lev_E) {

				if (		pm->zone_lpf_wS > lev_wS
						&& pm->lu_wS > lev_wS) {

					pm->flux_ZONE = PM_ZONE_HIGH;
				}
				else if (	pm->zone_lpf_wS < - lev_wS
						&& pm->lu_wS < - lev_wS) {

					pm->flux_ZONE = PM_ZONE_HIGH;
				}
			}
		}
	}
	else if (pm->flux_ZONE == PM_ZONE_HIGH) {

		lev_wS = pm->zone_MURE + pm->zone_gain_GI * pm->zone_MPPE;

		if (pm->lu_MODE == PM_LU_DETACHED) {

			if (		m_fabsf(pm->zone_lpf_wS) < lev_wS
					|| pm->detach_LOCK == 0) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
		else {
			if (m_fabsf(pm->zone_lpf_wS) < lev_wS) {

				pm->flux_ZONE = PM_ZONE_UNCERTAIN;
			}
		}
	}
}

static void
pm_estimate_FLUX(pmc_t *pm)
{
	if (pm->config_LU_ESTIMATE_FLUX == PM_ESTIMATE_ORTEGA) {

		if (pm->flux_ESTIMATE != PM_ESTIMATE_ORTEGA) {

			float			E1, L2;

			E1 = (pm->const_E > M_EPS_F) ? pm->const_E : pm->flux_E;
			L2 = pm->const_im_L2;

			pm->flux_X[0] = L2 * pm->lu_iX + E1 * pm->flux_F[0];
			pm->flux_X[1] = L2 * pm->lu_iY + E1 * pm->flux_F[1];

			pm->flux_ESTIMATE = PM_ESTIMATE_ORTEGA;
		}

		pm_flux_ORTEGA(pm);
	}
	else if (pm->config_LU_ESTIMATE_FLUX == PM_ESTIMATE_LUENBERGER) {

		if (pm->flux_ESTIMATE != PM_ESTIMATE_LUENBERGER) {

			pm->flux_X[0] = pm->lu_iD;
			pm->flux_X[1] = pm->lu_iQ;

			pm->flux_QZ = 0.f;

			pm->flux_ESTIMATE = PM_ESTIMATE_LUENBERGER;
		}

		pm_flux_LUENBERGER(pm);
	}
	else {
		/* NOTE: No sensorless observer selected. It is ok when you
		 * only need a SENSORED drive */

		if (pm->flux_ESTIMATE != PM_ESTIMATE_NONE) {

			pm->flux_ESTIMATE = PM_ESTIMATE_NONE;
		}
	}

	if (pm->config_LU_ESTIMATE_FLUX != PM_ESTIMATE_NONE) {

		pm_flux_ZONE(pm);
	}
}

void pm_hfi_DFT(pmc_t *pm, float la[5])
{
	lse_t		*ls = &pm->probe_LS[0];
	lse_float_t	v[5];

	float		*DFT = pm->hfi_DFT;
	float		lz[3], iW;

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

	lse_initiate(ls, LSE_CASCADE_MAX, 4, 1);

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

	iW = 1.f / pm->quick_hfwS;

	la[4] = ls->b[0];
	lz[0] = ls->b[1] * iW;
	lz[1] = ls->b[2] * iW;
	lz[2] = ls->b[3] * iW;

	m_la_eigf(lz, la, (pm->config_HFI_MAJOR_AXES == PM_ENABLED) ? 1 : 0);
}

static void
pm_estimate_HFI(pmc_t *pm)
{
	float			iD, iQ, uD, uQ;
	float			la[5], F[2], E;

	if (pm->hfi_IN == pm->hfi_INJS) {

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

	iD = pm->hfi_F[0] * pm->hfi_REM[10] + pm->hfi_F[1] * pm->hfi_REM[11];
	iQ = pm->hfi_F[0] * pm->hfi_REM[11] - pm->hfi_F[1] * pm->hfi_REM[10];

	/* Get corrected frame (half-period shift).
	 * */
	F[0] = pm->hfi_F[0] * pm->quick_hfSC[0] - pm->hfi_F[1] * pm->quick_hfSC[1];
	F[1] = pm->hfi_F[1] * pm->quick_hfSC[0] + pm->hfi_F[0] * pm->quick_hfSC[1];

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

	if (pm->hfi_POLA != 0) {

		E = pm->hfi_wave[0] * pm->hfi_wave[0] - pm->hfi_wave[1] * pm->hfi_wave[1];

		/* Get D-axis polarity with doubled frequency cosine wave.
		 * */
		m_rsumf(&pm->hfi_DFT[8], &pm->hfi_REM[8], iD * E);
	}

	pm->hfi_REM[10] = pm->lu_iX;
	pm->hfi_REM[11] = pm->lu_iY;

	pm->hfi_IN += 1;

	if (pm->hfi_IN == pm->hfi_INJS * (pm->hfi_SKIP + pm->hfi_ESTI)) {

		pm_hfi_DFT(pm, la);

		if (m_isfinitef(la[0]) != 0 && m_isfinitef(la[1]) != 0) {

			/* Speed estimation (PLL).
			 * */
			if (la[0] > M_EPS_F) {

				E = la[1] * pm->freq_hz;
				pm->hfi_wS += E * pm->hfi_gain_SF;
			}

			/* Get actual saliency frame.
			 * */
			F[0] = pm->hfi_F[0] * la[0] - pm->hfi_F[1] * la[1];
			F[1] = pm->hfi_F[1] * la[0] + pm->hfi_F[0] * la[1];

			E = (3.f - F[0] * F[0] - F[1] * F[1]) * .5f;

			pm->hfi_F[0] = F[0] * E;
			pm->hfi_F[1] = F[1] * E;

			pm->hfi_im_L1 = la[2];
			pm->hfi_im_L2 = la[3];
			pm->hfi_im_R = la[4];

			if (pm->hfi_POLA != 0) {

				if (pm->hfi_DFT[8] * (float) pm->hfi_POLA > 0.f) {

					/* Flip DQ-frame.
					 * */
					pm->hfi_F[0] = - pm->hfi_F[0];
					pm->hfi_F[1] = - pm->hfi_F[1];
				}
			}
		}
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

	/* Transfer the rotor position.
	 * */
	m_rotatef(pm->hfi_F, pm->hfi_wS * pm->dT);

	/* HF wave synthesis.
	 * */
	m_rotatef(pm->hfi_wave, pm->quick_hfwS * pm->dT);

	/* Enable to inject HF wave inside the current loop.
	 * */
	pm->hfi_INJECTED = PM_ENABLED;
}

static void
pm_sensor_HALL(pmc_t *pm)
{
	float		hF[2], A, B, relE, gain_SF;
	int		HS, lev_PROL;

	HS = pm->fb_HS;

	if (HS >= 1 && HS <= 6) {

		pm->hall_TIM++;
		pm->hall_ERR = 0;

		if (HS == pm->hall_HS) {

			if (pm->hall_DIRF != 0) {

				relE = pm->hall_wS * pm->hall_gain_PF * pm->dT;
				B = PM_HALL_SPAN * pm->hall_gain_PF;

				if (		m_fabsf(relE) > M_EPS_F
						&& m_fabsf(pm->hall_prol) < B) {

					m_rotatef(pm->hall_F, relE);

					pm->hall_prol += relE;
				}
				else {
					relE = 0.f;
				}

				lev_PROL = PM_TSMS(pm, pm->hall_prol_ms);

				if (pm->hall_TIM >= lev_PROL) {

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

			A = hF[0] * pm->hall_F[0] + hF[1] * pm->hall_F[1];
			B = hF[1] * pm->hall_F[0] - hF[0] * pm->hall_F[1];

			pm->hall_prol = 0.f;
			pm->hall_F[0] = hF[0];
			pm->hall_F[1] = hF[1];

			relE = m_atan2f(B, A);
		}

		gain_SF = 1.f / (float) pm->hall_TIM;
		gain_SF = (gain_SF < pm->hall_gain_SF) ? gain_SF : pm->hall_gain_SF;

		pm->hall_wS += (relE * pm->freq_hz - pm->hall_wS) * gain_SF;

		B = PM_HALL_SPAN * pm->hall_gain_PF;

		if (		pm->hall_gain_IF > M_EPS_F
				&& pm->const_Ja > M_EPS_F
				&& m_fabsf(pm->hall_prol) < B) {

			pm->hall_wS += (pm->lu_iQ - pm->lu_lpf_torque)
				* pm->hall_gain_IF * pm->dT / pm->const_Ja;
		}

		if (HS != pm->hall_HS) {

			pm->hall_HS = HS;
			pm->hall_TIM = 0;
		}
	}
	else {
		pm->hall_ERR++;

		lev_PROL = PM_TSMS(pm, pm->tm_transient_slow);

		if (pm->hall_ERR >= lev_PROL) {

			pm->hall_INUSE = PM_DISABLED;

			pm->fsm_errno = PM_ERROR_SENSOR_HALL_FAULT;

			if (pm->config_LU_ESTIMATE_FLUX != PM_ESTIMATE_NONE) {

				pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
			}
		}
	}
}

static void
pm_sensor_ABI(pmc_t *pm)
{
	float			relAN, locAN, luF[2], gain_SF;
	int			relEP, uEP, WRAP;

	relEP = (pm->fb_EP - pm->abi_in_EP) & 0xFFFFU;

	pm->abi_in_EP = pm->fb_EP;

	if (pm->config_ABI_ABSOLUTE == PM_ENABLED) {

		if (relEP < - pm->abi_EPPR / 2) {

			relEP += pm->abi_EPPR;
			pm->abi_revol += 1;
		}
		else if (relEP > pm->abi_EPPR / 2) {

			relEP += - pm->abi_EPPR;
			pm->abi_revol += - 1;
		}

		WRAP = pm->abi_gear_ZQ;

		if (pm->abi_revol < - WRAP) {

			pm->abi_unwrap += - WRAP;
			pm->abi_revol += WRAP;
		}
		else if (pm->abi_revol > WRAP) {

			pm->abi_unwrap += WRAP;
			pm->abi_revol += - WRAP;
		}

		/* Absolute position in EP units.
		 * */
		pm->abi_loc_EP = pm->abi_in_EP + pm->abi_revol * pm->abi_EPPR;
	}

	/* Debounce (around one EP).
	 * */
	uEP =  (relEP > 0 && pm->abi_rel_EP < 0) ? - 1 : 0;
	uEP += (relEP < 0 && pm->abi_rel_EP > 0) ?   1 : 0;

	pm->abi_rel_EP = (relEP != 0) ? relEP : pm->abi_rel_EP;

	relEP += uEP;

	if (pm->config_ABI_ABSOLUTE == PM_ENABLED) {

		pm->abi_loc_EP += uEP;
	}

	pm->abi_TIM++;

	if (relEP == 0) {

		relAN = pm->abi_wS * pm->abi_gain_PF * pm->dT;
		relAN = (m_fabsf(pm->abi_prol) < pm->quick_ZiEP) ? relAN : 0.f;

		pm->abi_prol += relAN;
	}
	else {
		relAN = (float) relEP * pm->quick_ZiEP - pm->abi_prol;

		pm->abi_prol = 0.f;

		if (pm->config_ABI_ABSOLUTE != PM_ENABLED) {

			pm->abi_loc_EP += relEP;

			WRAP = pm->abi_EPPR * pm->abi_gear_ZQ;

			if (pm->abi_loc_EP < - WRAP) {

				pm->abi_unwrap += - pm->abi_gear_ZQ;
				pm->abi_loc_EP += WRAP;
			}
			else if (pm->abi_loc_EP > WRAP) {

				pm->abi_unwrap += pm->abi_gear_ZQ;
				pm->abi_loc_EP += - WRAP;
			}
		}
	}

	if (pm->config_LU_LOCATION == PM_LOCATION_ABI) {

		float		locEP;

		/* Take the electrical absolute LOCATION.
		 * */
		locEP = (float) pm->abi_unwrap * (float) pm->abi_EPPR
			+ (float) pm->abi_loc_EP;

		pm->abi_location = locEP * pm->quick_ZiEP + pm->abi_prol;
	}

	/* Take the electrical position.
	 * */
	locAN = (float) pm->abi_loc_EP * pm->quick_ZiEP + pm->abi_prol;
	locAN = m_wrapf(locAN);

	luF[0] = m_cosf(locAN);
	luF[1] = m_sinf(locAN);

	pm->abi_F[0] = luF[0] * pm->abi_in_F[0] - luF[1] * pm->abi_in_F[1];
	pm->abi_F[1] = luF[1] * pm->abi_in_F[0] + luF[0] * pm->abi_in_F[1];

	gain_SF = 1.f / (float) pm->abi_TIM;
	gain_SF = (gain_SF < pm->abi_gain_SF) ? gain_SF : pm->abi_gain_SF;

	/* Speed estimation.
	 * */
	pm->abi_wS += (relAN * pm->freq_hz - pm->abi_wS) * gain_SF;

	if (		pm->abi_gain_IF > M_EPS_F
			&& pm->const_Ja > M_EPS_F
			&& m_fabsf(pm->abi_prol) < pm->quick_ZiEP) {

		pm->abi_wS += (pm->lu_iQ - pm->lu_lpf_torque)
			* pm->abi_gain_IF * pm->dT / pm->const_Ja;
	}

	if (relEP != 0) {

		pm->abi_TIM = 0;
	}
}

static void
pm_sensor_SINCOS(pmc_t *pm)
{
	float			*FIR = pm->sincos_FIR;
	float			scAN, locAN;
	int			WRAP;

	if (pm->config_SINCOS_FRONTEND == PM_SINCOS_ANALOG) {

		float		Q[9];

		Q[0] = pm->fb_COS;
		Q[1] = pm->fb_SIN;
		Q[2] = Q[0] * Q[1];
		Q[3] = Q[0] * Q[0];
		Q[4] = Q[1] * Q[1];
		Q[5] = Q[3] * Q[1];
		Q[6] = Q[4] * Q[0];
		Q[7] = Q[3] * Q[0];
		Q[8] = Q[4] * Q[1];

		pm->sincos_SC[0] = FIR[0] + FIR[2] * Q[0] + FIR[4]  * Q[1]
			+ FIR[6]  * Q[2] + FIR[8]  * Q[3] + FIR[10] * Q[4]
			+ FIR[12] * Q[5] + FIR[14] * Q[6] + FIR[16] * Q[7]
			+ FIR[18] * Q[8];

		pm->sincos_SC[0] = FIR[1] + FIR[3] * Q[0] + FIR[5]  * Q[1]
			+ FIR[7]  * Q[2] + FIR[9]  * Q[3] + FIR[11] * Q[4]
			+ FIR[13] * Q[5] + FIR[15] * Q[6] + FIR[17] * Q[7]
			+ FIR[19] * Q[8];
	}
	else if (pm->config_SINCOS_FRONTEND == PM_SINCOS_RESOLVER) {

		/* TODO */
	}

	/* Track the SIN/COS position.
	 * */
	if (pm->sincos_SC[0] < 0.f) {

		if (		   pm->sincos_SC[1] < 0.f
				&& pm->sincos_SC[2] >= 0.f) {

			pm->sincos_revol += 1;
		}
		else if (	   pm->sincos_SC[1] >= 0.f
				&& pm->sincos_SC[2] < 0.f) {

			pm->sincos_revol += - 1;
		}
	}

	pm->sincos_SC[2] = pm->sincos_SC[1];

	WRAP = pm->sincos_gear_ZQ;

	if (pm->sincos_revol < - WRAP) {

		pm->sincos_unwrap += - WRAP;
		pm->sincos_revol += WRAP;
	}
	else if (pm->sincos_revol > WRAP) {

		pm->sincos_unwrap += WRAP;
		pm->sincos_revol += - WRAP;
	}

	scAN = m_atan2f(pm->sincos_SC[1], pm->sincos_SC[0])
		+ (float) pm->sincos_revol * (2.f * M_PI_F);

	if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		float		scLOC;

		scLOC = scAN + (float) pm->sincos_unwrap * (2.f * M_PI_F);

		/* Take the electrical absolute LOCATION.
		 * */
		pm->sincos_location = scLOC * pm->quick_ZiSQ;
	}

	/* Take the electrical position.
	 * */
	locAN = m_wrapf(scAN * pm->quick_ZiSQ);

	pm->sincos_F[0] = m_cosf(locAN);
	pm->sincos_F[1] = m_sinf(locAN);

	/* TODO */
}

static void
pm_lu_FSM(pmc_t *pm)
{
	float			EX, EY, hTS, lu_F[2], wINC, inTQ;
	int			lev_SKIP, lev_HOLD;

	/* Get the current on DQ-axes.
	 * */
	pm->lu_iD = pm->lu_F[0] * pm->lu_iX + pm->lu_F[1] * pm->lu_iY;
	pm->lu_iQ = pm->lu_F[0] * pm->lu_iY - pm->lu_F[1] * pm->lu_iX;

	/* Transfer to the next apriori position.
	 * */
	m_rotatef(pm->lu_F, pm->lu_wS * pm->dT);

	if (pm->vsi_XXF != 0) {

		/* The current prediction in XY-axes will be used in case of
		 * current samples are discarded.
		 * */
		pm->lu_iX = pm->lu_F[0] * pm->lu_iD - pm->lu_F[1] * pm->lu_iQ;
		pm->lu_iY = pm->lu_F[1] * pm->lu_iD + pm->lu_F[0] * pm->lu_iQ;
	}

	if (pm->lu_MODE == PM_LU_DETACHED) {

		if (pm->detach_TIM > 0) {

			pm_detached_BEMF(pm);
		}

		pm_flux_ZONE(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		pm->detach_TIM++;

		lev_SKIP = PM_TSMS(pm, pm->tm_startup);

		if (pm->flux_ZONE == PM_ZONE_LOCKED_IN_DETACH) {

			/* Lock in detached mode permanently.
			 * */
		}
		else if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;

			pm->proc_set_Z(PM_Z_NUL);
		}
		else if (pm->detach_TIM < lev_SKIP) {

			/* Not enough time passed to go into low speed
			 * mode. Shut up here.
			 * */
		}
		else if (	pm->config_LU_SENSOR_ABI == PM_ENABLED
				&& pm->config_ABI_ABSOLUTE == PM_ENABLED
				&& pm->abi_INUSE == PM_ENABLED) {

			pm->lu_MODE = PM_LU_SENSOR_ABI;

			pm->abi_in_EP = pm->fb_EP;

			pm->abi_revol = 0;
			pm->abi_unwrap = 0;
			pm->abi_rel_EP = 0;
			pm->abi_loc_EP = 0;
			pm->abi_TIM = 0;
			pm->abi_prol = 0.f;

			pm->abi_F[0] = pm->lu_F[0];
			pm->abi_F[1] = pm->lu_F[1];
			pm->abi_wS = pm->lu_wS;
		}
		else if (	pm->config_LU_SENSOR_HALL == PM_ENABLED
				&& pm->hall_INUSE == PM_ENABLED) {

			pm->lu_MODE = PM_LU_SENSOR_HALL;

			pm->hall_HS = pm->fb_HS;
			pm->hall_DIRF = 0;
			pm->hall_TIM = PM_TSMS(pm, pm->hall_prol_ms);
			pm->hall_ERR = 0;

			pm->hall_F[0] = pm->lu_F[0];
			pm->hall_F[1] = pm->lu_F[1];
			pm->hall_wS = pm->lu_wS;

			pm->proc_set_Z(PM_Z_NUL);
		}
		else if (pm->config_LU_ESTIMATE_HFI == PM_ENABLED) {

			pm->lu_MODE = PM_LU_ESTIMATE_HFI;

			pm->hfi_IN = 0;
			pm->hfi_TIM = 0;

			pm->hfi_F[0] = pm->lu_F[0];
			pm->hfi_F[1] = pm->lu_F[1];
			pm->hfi_wS = pm->lu_wS;

			pm->proc_set_Z(PM_Z_NUL);
		}
		else if (pm->config_LU_FORCED == PM_ENABLED) {

			pm->lu_MODE = PM_LU_FORCED;

			pm->forced_F[0] = pm->flux_F[0];
			pm->forced_F[1] = pm->flux_F[1];
			pm->forced_wS = pm->flux_wS;
			pm->forced_TIM = 0;

			pm->proc_set_Z(PM_Z_NUL);
		}
	}
	else if (pm->lu_MODE == PM_LU_FORCED) {

		pm_estimate_FLUX(pm);
		pm_forced(pm);

		pm->forced_TIM++;

		lu_F[0] = pm->forced_F[0];
		lu_F[1] = pm->forced_F[1];

		pm->lu_wS = pm->forced_wS;

		if (		pm->flux_ZONE == PM_ZONE_HIGH
				&& pm->const_E > M_EPS_F) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;

			pm->lu_lpf_torque = (pm->forced_wS < 0.f)
				? - pm->forced_hold_D : pm->forced_hold_D;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_current_hold);

			if (		pm->config_LU_SENSOR_ABI == PM_ENABLED
					&& pm->config_ABI_ABSOLUTE != PM_ENABLED
					&& pm->abi_INUSE == PM_ENABLED
					&& pm->forced_TIM > lev_HOLD) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;

				pm->abi_in_EP = pm->fb_EP;
				pm->abi_in_F[0] = pm->lu_F[0];
				pm->abi_in_F[1] = pm->lu_F[1];

				pm->abi_revol = 0;
				pm->abi_unwrap = 0;
				pm->abi_rel_EP = 0;
				pm->abi_loc_EP = 0;
				pm->abi_TIM = 0;
				pm->abi_prol = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ESTIMATE_FLUX) {

		pm_estimate_FLUX(pm);

		lu_F[0] = pm->flux_F[0];
		lu_F[1] = pm->flux_F[1];

		pm->lu_wS = pm->flux_wS;

		if (		   pm->flux_ZONE == PM_ZONE_NONE
				|| pm->flux_ZONE == PM_ZONE_UNCERTAIN) {

			if (		pm->config_LU_SENSOR_HALL == PM_ENABLED
					&& pm->hall_INUSE == PM_ENABLED) {

				pm->lu_MODE = PM_LU_SENSOR_HALL;

				pm->hall_HS = pm->fb_HS;
				pm->hall_DIRF = 0;
				pm->hall_TIM = PM_TSMS(pm, pm->hall_prol_ms);
				pm->hall_ERR = 0;

				pm->hall_F[0] = pm->lu_F[0];
				pm->hall_F[1] = pm->lu_F[1];
				pm->hall_wS = pm->lu_wS;
			}
			else if (	pm->config_LU_SENSOR_ABI == PM_ENABLED
					&& pm->flux_ZONE != PM_ZONE_NONE) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;

				pm->abi_in_EP = pm->fb_EP;
				pm->abi_in_F[0] = pm->lu_F[0];
				pm->abi_in_F[1] = pm->lu_F[1];

				pm->abi_revol = 0;
				pm->abi_unwrap = 0;
				pm->abi_rel_EP = 0;
				pm->abi_loc_EP = 0;
				pm->abi_TIM = 0;
				pm->abi_prol = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
			else if (pm->config_LU_ESTIMATE_HFI == PM_ENABLED) {

				pm->lu_MODE = PM_LU_ESTIMATE_HFI;

				pm->hfi_IN = 0;
				pm->hfi_TIM = 0;

				pm->hfi_F[0] = pm->lu_F[0];
				pm->hfi_F[1] = pm->lu_F[1];
				pm->hfi_wS = pm->lu_wS;
			}
			else if (pm->config_LU_FORCED == PM_ENABLED) {

				pm->lu_MODE = PM_LU_FORCED;

				pm->forced_F[0] = pm->flux_F[0];
				pm->forced_F[1] = pm->flux_F[1];
				pm->forced_wS = pm->flux_wS;
				pm->forced_TIM = 0;
			}
			else if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

				pm->lu_MODE = PM_LU_DETACHED;

				pm->detach_LOCK = 0;
				pm->detach_TIM = - PM_TSMS(pm, pm->tm_transient_fast);

				pm->flux_wS = 0.f;

				pm->watt_lpf_D = 0.f;
				pm->watt_lpf_Q = 0.f;
				pm->watt_lpf_wP = 0.f;

				pm->proc_set_Z(PM_Z_ABC);
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_ESTIMATE_HFI) {

		pm_estimate_FLUX(pm);
		pm_estimate_HFI(pm);

		pm->hfi_TIM++;

		lu_F[0] = pm->hfi_F[0];
		lu_F[1] = pm->hfi_F[1];

		pm->lu_wS = pm->hfi_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
		else {
			lev_HOLD = PM_TSMS(pm, pm->tm_startup);

			if (		pm->config_LU_SENSOR_ABI == PM_ENABLED
					&& pm->config_ABI_ABSOLUTE != PM_ENABLED
					&& pm->abi_INUSE == PM_ENABLED
					&& pm->hfi_TIM > lev_HOLD) {

				pm->lu_MODE = PM_LU_SENSOR_ABI;

				pm->abi_in_EP = pm->fb_EP;
				pm->abi_in_F[0] = pm->lu_F[0];
				pm->abi_in_F[1] = pm->lu_F[1];

				pm->abi_revol = 0;
				pm->abi_unwrap = 0;
				pm->abi_rel_EP = 0;
				pm->abi_loc_EP = 0;
				pm->abi_TIM = 0;
				pm->abi_prol = 0.f;

				pm->abi_F[0] = pm->lu_F[0];
				pm->abi_F[1] = pm->lu_F[1];
				pm->abi_wS = pm->lu_wS;
			}
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_HALL) {

		pm_estimate_FLUX(pm);
		pm_sensor_HALL(pm);

		lu_F[0] = pm->hall_F[0];
		lu_F[1] = pm->hall_F[1];

		pm->lu_wS = pm->hall_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_ABI) {

		pm_estimate_FLUX(pm);
		pm_sensor_ABI(pm);

		if (		PM_CONFIG_DEBUG(pm) == PM_ENABLED
				&& pm->config_LU_ESTIMATE_HFI == PM_ENABLED) {

			pm_estimate_HFI(pm);
		}

		lu_F[0] = pm->abi_F[0];
		lu_F[1] = pm->abi_F[1];

		pm->lu_wS = pm->abi_wS;

		if (pm->flux_ZONE == PM_ZONE_HIGH) {

			pm->lu_MODE = PM_LU_ESTIMATE_FLUX;
		}
	}
	else if (pm->lu_MODE == PM_LU_SENSOR_SINCOS) {

		/* TODO */
	}

	/* Take the LU position estimate with TRANSIENT rate limited.
	 * */
	hTS = pm->lu_transient * pm->dT;

	EX = lu_F[0] * pm->lu_F[0] + lu_F[1] * pm->lu_F[1];
	EY = lu_F[1] * pm->lu_F[0] - lu_F[0] * pm->lu_F[1];

	if (EX > M_EPS_F && EY < - hTS) {

		m_rotatef(pm->lu_F, - hTS);
	}
	else if (EX > M_EPS_F && EY > hTS) {

		m_rotatef(pm->lu_F, hTS);
	}
	else {
		pm->lu_F[0] = lu_F[0];
		pm->lu_F[1] = lu_F[1];
	}

	/* Track the position to get full number of revolutions.
	 * */
	if (pm->lu_F[0] < 0.f) {

		if (		   pm->lu_F[1] < 0.f
				&& pm->lu_F[2] >= 0.f) {

			pm->lu_revol += 1;
		}
		else if (	   pm->lu_F[1] >= 0.f
				&& pm->lu_F[2] < 0.f) {

			pm->lu_revol += - 1;
		}
	}

	pm->lu_F[2] = pm->lu_F[1];

	if (pm->lu_revol - pm->lu_revob < - 4) {

		pm->lu_total_revol += pm->lu_revob - pm->lu_revol;
		pm->lu_revob = pm->lu_revol;
	}
	else if (pm->lu_revol - pm->lu_revob > 4) {

		pm->lu_total_revol += pm->lu_revol - pm->lu_revob;
		pm->lu_revob = pm->lu_revol;
	}

	/* Take the LOCATION according to the configuration.
	 * */
	if (pm->config_LU_LOCATION == PM_LOCATION_INHERITED) {

		pm->lu_location = m_atan2f(pm->lu_F[1], pm->lu_F[0])
			+ (float) pm->lu_revol * (2.f * M_PI_F);
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_ABI) {

		if (0) {

			/* TODO */

			/*pm->abi_baseEP = pm->fb_EP;
			pm->abi_baseF[0] = pm->lu_F[0];
			pm->abi_baseF[1] = pm->lu_F[1];
			pm->abi_lastEP = 0;
			pm->abi_rotEP = 0;
			pm->abi_prolTIM = 0;
			pm->abi_prolS = 0.f;

			pm->abi_F[0] = pm->lu_F[0];
			pm->abi_F[1] = pm->lu_F[1];
			pm->abi_wS = pm->lu_wS;*/
		}

		if (pm->lu_MODE != PM_LU_SENSOR_ABI) {

			pm_sensor_ABI(pm);
		}

		pm->lu_wS = pm->abi_wS;
		pm->lu_location = pm->abi_location;
	}
	else if (pm->config_LU_LOCATION == PM_LOCATION_SINCOS) {

		/* TODO */
	}

	if (		pm->config_LU_ESTIMATE_FLUX == PM_ESTIMATE_LUENBERGER
			&& pm->lu_MODE == PM_LU_ESTIMATE_FLUX) {

		/* Replace the current on DQ-axes with predicted one.
		 * */
		pm->lu_iD = pm->flux_X[0];
		pm->lu_iQ = pm->flux_X[1];
	}

	/* Get an external LOAD torque estimate in units of current.
	 * */
	wINC = pm->lu_wS - pm->lu_base_wS;
	pm->lu_base_wS = pm->lu_wS;

	if (pm->lu_MODE != PM_LU_FORCED) {

		inTQ = pm->lu_iQ - wINC * pm->freq_hz * pm->const_Ja;
		pm->lu_lpf_torque += (inTQ - pm->lu_lpf_torque) * pm->lu_gain_TQ;
	}
}

void pm_clearance(pmc_t *pm, int xA, int xB, int xC)
{
	int		xMIN;

	xA = pm->dc_resolution - xA;
	xB = pm->dc_resolution - xB;
	xC = pm->dc_resolution - xC;

	/* Check if there are PWM edges within clearance zone. The CURRENT
	 * measurements will be used or rejected based on this flags.
	 *
	 * NOTE: In case of INLINE current measurement it is possible to get
	 * sample at the TOP.
	 *
	 * NOTE: To get best result you should have a current sensor with a
	 * fast transient that allows you to specify narrow clearance zone.
	 *
	 *                   1 - sqrt(3) / 2
	 * 	clearance < -----------------
	 *                      pwm_freq
	 *
	 * */
	if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

		pm->vsi_AF = ((pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
		pm->vsi_BF = ((pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

		pm->vsi_AF = (pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip) ? 0 : 1;
		pm->vsi_CF = 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

		pm->vsi_AF = ((pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0)) ? 0 : 1;
		pm->vsi_BF = ((pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0)) ? 0 : 1;
		pm->vsi_CF = ((pm->vsi_CG >= pm->ts_clearance && xC > pm->ts_skip)
				|| (pm->vsi_CG == 0 && xC == 0)) ? 0 : 1;
	}
	else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

		pm->vsi_AF = (pm->vsi_AG >= pm->ts_clearance && xA > pm->ts_skip) ? 0 : 1;
		pm->vsi_BF = (pm->vsi_BG >= pm->ts_clearance && xB > pm->ts_skip) ? 0 : 1;
		pm->vsi_CF = (pm->vsi_CG >= pm->ts_clearance && xC > pm->ts_skip) ? 0 : 1;
	}

	/* Chech if at least TWO samples are clean so they can be used in
	 * control loops.
	 * */
	pm->vsi_XXF = (pm->vsi_AF + pm->vsi_BF + pm->vsi_CF < 2) ? 0 : 1;

	/* Check if there are PWM edges within clearance zone. The DC link
	 * voltage measurement will be used or rejected based on this flag.
	 * */
	pm->vsi_SF = (	   ((pm->vsi_AG > pm->ts_skip && xA > pm->ts_skip)
				|| (pm->vsi_AG == 0 && xA == 0))
			&& ((pm->vsi_BG > pm->ts_skip && xB > pm->ts_skip)
				|| (pm->vsi_BG == 0 && xB == 0))
			&& ((pm->vsi_CG > pm->ts_skip && xC > pm->ts_skip)
				|| (pm->vsi_CG == 0 && xC == 0))) ? 0 : 1;

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

		/* Check if terminal voltages are exactly ZERO to get more
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

void pm_voltage(pmc_t *pm, float uX, float uY)
{
	float		uA, uB, uC, uMIN, uMAX, uDC;
	int		xA, xB, xC, xMIN, xMAX;

	uX *= pm->quick_iUdc;
	uY *= pm->quick_iUdc;

	uDC = m_sqrtf(uX * uX + uY * uY);

	pm->vsi_DC = uDC / pm->k_EMAX;
	pm->vsi_lpf_DC += (pm->vsi_DC - pm->vsi_lpf_DC) * pm->vsi_gain_LP_F;

	if (		pm->config_VSI_CIRCULAR == PM_ENABLED
			&& uDC > pm->k_EMAX) {

		uDC = pm->k_EMAX / uDC;

		uX *= uDC;
		uY *= uDC;
	}

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

	if (uDC > 1.f) {

		uDC = 1.f / uDC;

		uA *= uDC;
		uB *= uDC;
		uC *= uDC;

		uMIN *= uDC;
		uMAX *= uDC;
	}

	if (pm->config_VSI_PRECISE == PM_ENABLED) {

		uDC = .5f - (uMAX + uMIN) * .5f;
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

	if (pm->lu_MODE != PM_LU_DISABLED) {

		if (PM_CONFIG_IFB(pm) == PM_IFB_AB_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA > xMAX || xB > xMAX) {

				xMAX = (xA > xB) ? xA : xB;
				xMAX = (xC > xMAX) ? xC : xMAX;
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
		else if (PM_CONFIG_IFB(pm) == PM_IFB_AB_GND) {

			xMAX = pm->dc_resolution - pm->ts_clearance;
			xMIN = pm->dc_resolution - pm->ts_minimal;

			xA = (xA < pm->ts_minimal) ? 0 : (xA > xMAX) ? xMAX : xA;
			xB = (xB < pm->ts_minimal) ? 0 : (xB > xMAX) ? xMAX : xB;
			xC = (xC < pm->ts_minimal) ? 0 : (xC > xMAX) ? xMAX : xC;
		}
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_INLINE) {

			xMAX = pm->dc_resolution - pm->ts_clearance;

			if (xA > xMAX || xB > xMAX || xC > xMAX) {

				xMAX = (xA > xB) ? xA : xB;
				xMAX = (xC > xMAX) ? xC : xMAX;
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
		else if (PM_CONFIG_IFB(pm) == PM_IFB_ABC_GND) {

			xA = (xA < pm->ts_minimal) ? 0 : xA;
                        xB = (xB < pm->ts_minimal) ? 0 : xB;
			xC = (xC < pm->ts_minimal) ? 0 : xC;

			xMAX = pm->dc_resolution - pm->ts_clearance;
			xMIN = pm->dc_resolution - pm->ts_minimal;

			if (xA > xMAX && xB > xMAX) {

				xC = (xC > xMIN) ? pm->dc_resolution : xC;

				if (xA < xB) {

					xA = (xA > xMAX) ? xMAX : xA;
					xB = (xB > xMIN) ? pm->dc_resolution : xB;
				}
				else {
					xA = (xA > xMIN) ? pm->dc_resolution : xA;
					xB = (xB > xMAX) ? xMAX : xB;
				}
			}
			else if (xB > xMAX && xC > xMAX) {

				xA = (xA > xMIN) ? pm->dc_resolution : xA;

				if (xB < xC) {

					xB = (xB > xMAX) ? xMAX : xB;
					xC = (xC > xMIN) ? pm->dc_resolution : xC;
				}
				else {
					xB = (xB > xMIN) ? pm->dc_resolution : xB;
					xC = (xC > xMAX) ? xMAX : xC;
				}
			}
			else if (xA > xMAX && xC > xMAX) {

				xB = (xB > xMIN) ? pm->dc_resolution : xB;

				if (xA < xC) {

					xA = (xA > xMAX) ? xMAX : xA;
					xC = (xC > xMIN) ? pm->dc_resolution : xC;
				}
				else {
					xA = (xA > xMIN) ? pm->dc_resolution : xA;
					xC = (xC > xMAX) ? xMAX : xC;
				}
			}
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

		if (		   pm->vsi_SA > pm->ts_bootstrap
				|| pm->vsi_SB > pm->ts_bootstrap
				|| pm->vsi_SC > pm->ts_bootstrap) {

			/* Clamp the output DC to a safe level if bootstrap
			 * retention time is running out.
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

		uDC = .33333333f * (xA + xB + xC);

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

	/* Update the measurement clearance flags according to the DC values.
	 * */
	pm_clearance(pm, xA, xB, xC);
}

static float
pm_form_iSP(pmc_t *pm, float eS)
{
	float		iSP;

	/* There is a DEAD zone.
	 * */
	eS = (m_fabsf(eS) > pm->s_tol_Z) ? eS : 0.f;

	/* The speed regulator uses an load torque estimate as feed forward
	 * component. It also replaces the integral component.
	 * */
	iSP = pm->s_gain_P * eS + pm->lu_lpf_torque;

	if (		   pm->lu_MODE == PM_LU_ESTIMATE_HFI
			&& pm->config_LU_LOCATION == PM_LOCATION_INHERITED) {

		/* Use HIGH proportional gain in HFI mode.
		 * */
		iSP += pm->s_gain_H * eS;
	}

	/* Output clamp.
	 * */
	iSP = (iSP > pm->i_maximal) ? pm->i_maximal :
		(iSP < - pm->i_reverse) ? - pm->i_reverse : iSP;

	return iSP;
}

static void
pm_loop_current(pmc_t *pm)
{
	float		track_D, track_Q, eD, eQ, uD, uQ, uX, uY, wP;
	float		iMAX, iREV, uMAX, uREV, wMAX, wREV, E, dS, lerp_F, hfi_NT;

	if (pm->lu_MODE == PM_LU_FORCED) {

		track_D = pm->forced_hold_D;
		track_Q = 0.f;
	}
	else if (	pm->lu_MODE == PM_LU_ESTIMATE_FLUX
			&& (	   pm->flux_ZONE == PM_ZONE_NONE
				|| pm->flux_ZONE == PM_ZONE_UNCERTAIN)) {

		track_D = 0.f;
		track_Q = 0.f;
	}
	else {
		if (pm->config_LU_DRIVE == PM_DRIVE_CURRENT) {

			if (		pm->config_HOLDING_BRAKE == PM_ENABLED
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

		if (pm->config_MTPA_RELUCTANCE == PM_ENABLED) {

			/* TODO */
		}

		if (pm->config_WEAKENING == PM_ENABLED) {

			E = (1.f - pm->vsi_DC) * pm->const_fb_U;

			if (pm->const_fb_U > pm->watt_dclink_HI) {

				/* Prevent the lack of weakening in case of overvoltage.
				 * */
				E = (E > 0.f) ? 0.f : E;
			}

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

		if (		pm->config_LU_DRIVE == PM_DRIVE_CURRENT
				&& pm->config_SPEED_LIMITED == PM_ENABLED) {

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

	if (		pm->config_WEAKENING == PM_ENABLED
			&& pm->weak_D < - M_EPS_F) {

		/* Flux weakening constraints.
		 * */
		track_Q = (track_Q > pm->i_derated_WEAK) ? pm->i_derated_WEAK
			: (track_Q < - pm->i_derated_WEAK) ? - pm->i_derated_WEAK : track_Q;
	}
	else {
		iMAX = (iMAX < pm->i_derated_PCB) ? iMAX : pm->i_derated_PCB;
		iREV = (iREV > - pm->i_derated_PCB) ? iREV : - pm->i_derated_PCB;
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

	/* Slew rate FACTOR.
	 * */
	hfi_NT = 1.f;

	if (pm->hfi_INJECTED == PM_ENABLED) {

		if (pm->hfi_TORQ != 0) {

			if (		pm->hfi_IN < pm->hfi_INJS * (pm->hfi_SKIP
						+ pm->hfi_ESTI)
					|| pm->hfi_IN > pm->hfi_INJS * (pm->hfi_SKIP
						+ pm->hfi_ESTI + pm->hfi_TORQ)) {

				/* ZERO current on HFI measurement window.
				 * */
				track_D = 0.f;
				track_Q = 0.f;
			}
		}
		else {
			if (pm->hfi_IN < pm->hfi_INJS * (pm->hfi_SKIP + pm->hfi_ESTI)) {

				/* Keep the current CONSTANT.
				 * */
				hfi_NT = 0.f;
			}
			else {
				hfi_NT = pm->hfi_IN;

				/* HFI cycle restart immediate.
				 * */
				pm->hfi_IN = 0;
			}
		}
	}

	/* Slew rate limited current tracking.
	 * */
	dS = pm->i_slew_rate * pm->dT * hfi_NT;
	pm->i_track_D = (pm->i_track_D < track_D - dS) ? pm->i_track_D + dS
		: (pm->i_track_D > track_D + dS) ? pm->i_track_D - dS : track_D;
	pm->i_track_Q = (pm->i_track_Q < track_Q - dS) ? pm->i_track_Q + dS
		: (pm->i_track_Q > track_Q + dS) ? pm->i_track_Q - dS : track_Q;

	if (pm->hfi_INJECTED == PM_ENABLED) {

		if (pm->hfi_TORQ != 0) {

			if (		pm->hfi_IN > pm->hfi_INJS * (pm->hfi_SKIP
						+ pm->hfi_ESTI + pm->hfi_TORQ)
					&& m_fabsf(pm->i_track_D) < M_EPS_F
					&& m_fabsf(pm->i_track_Q) < M_EPS_F) {

				/* HFI cycle restart after tracking reaches ZERO.
				 * */
				pm->hfi_IN = 0;
			}
		}
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

	if (pm->hfi_INJECTED == PM_ENABLED) {

		if (pm->hfi_IN < pm->hfi_INJS * (pm->hfi_SKIP + pm->hfi_ESTI)) {

			hfi_NT = pm->hfi_inject_sine * pm->quick_hfwS * pm->const_im_L1;

			/* HF injection.
			 * */
			uD += pm->hfi_wave[0] * hfi_NT;
			uQ += pm->hfi_wave[1] * hfi_NT;
		}
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

	if (pm->config_LU_DRIVE == PM_DRIVE_SPEED) {

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

	if (pm->lu_MODE == PM_LU_FORCED) {

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
	float		E, E_abs, wSP, lerp_S, gain_P;

	/* Get location discrepancy.
	 * */
	E = pm->x_setpoint_location - pm->lu_location;
	E_abs = m_fabsf(E);

	pm->x_discrepancy = E;

	/* Servo is based on constant acceleration formula.
	 * */
	E = (E < 0.f) ? - m_sqrtf(E_abs) : m_sqrtf(E_abs);

	/* There is a DEAD zone.
	 * */
	E = (E_abs > pm->x_tol_Z) ? E : 0.f;

	/* Slow down in NEAR zone.
	 * */
	lerp_S = (E_abs < pm->x_tol_NEAR) ? E_abs / pm->x_tol_NEAR : 1.f;
	gain_P = pm->x_gain_P * lerp_S + pm->x_gain_Z * (1.f - lerp_S);

	wSP = E * gain_P + pm->x_setpoint_speed;

	/* Update speed loop SETPOINT.
	 * */
	pm->s_setpoint_speed = wSP;
}

static void
pm_mileage_info(pmc_t *pm)
{
	float		Wh, Ah, fuel;

	/* Traveled distance (m).
	 * */
	pm->im_distance = (float) pm->lu_total_revol
		* pm->const_ld_S / (float) pm->const_Zp;

	/* Get WATT per HOUR.
	 * */
	Wh = pm->watt_lpf_wP * pm->dT * (1.f / 3600.f);
	Ah = Wh * pm->quick_iUdc;

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

	if (pm->vsi_AF == 0) {

		/* Get inline current A.
		 * */
		pm->fb_iA = pm->ad_IA[1] * fb->current_A + pm->ad_IA[0];

		if (m_fabsf(pm->fb_iA) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (pm->vsi_BF == 0) {

		/* Get inline current B.
		 * */
		pm->fb_iB = pm->ad_IB[1] * fb->current_B + pm->ad_IB[0];

		if (m_fabsf(pm->fb_iB) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (pm->vsi_CF == 0) {

		/* Get inline current C.
		 * */
		pm->fb_iC = pm->ad_IC[1] * fb->current_C + pm->ad_IC[0];

		if (m_fabsf(pm->fb_iC) > pm->fault_current_halt) {

			pm->fsm_errno = PM_ERROR_INSTANT_OVERCURRENT;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	if (PM_CONFIG_NOP(pm) == PM_NOP_THREE_PHASE) {

		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = .33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);
			vA = pm->fb_iA - Q;
			vB = pm->fb_iB - Q;

			pm->lu_iX = vA;
			pm->lu_iY = .57735027f * vA + 1.1547005f * vB;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_BF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = .57735027f * pm->fb_iA + 1.1547005f * pm->fb_iB;
		}
		else if (	   pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = - pm->fb_iB - pm->fb_iC;
			pm->lu_iY = .57735027f * pm->fb_iB - .57735027f * pm->fb_iC;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = - .57735027f * pm->fb_iA - 1.1547005f * pm->fb_iC;
		}
	}
	else {
		if (		   pm->vsi_AF == 0
				&& pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			Q = .33333333f * (pm->fb_iA + pm->fb_iB + pm->fb_iC);
			vA = pm->fb_iA - Q;
			vB = pm->fb_iB - Q;

			pm->lu_iX = vA;
			pm->lu_iY = vB;
		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_BF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = pm->fb_iB;
		}
		else if (	   pm->vsi_BF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = - pm->fb_iB - pm->fb_iC;
			pm->lu_iY = pm->fb_iB;

		}
		else if (	   pm->vsi_AF == 0
				&& pm->vsi_CF == 0) {

			pm->lu_iX = pm->fb_iA;
			pm->lu_iY = - pm->fb_iA - pm->fb_iC;
		}
	}

	if (pm->vsi_SF == 0) {

		/* Get DC link voltage.
		 * */
		pm->const_fb_U = pm->ad_US[1] * fb->voltage_U + pm->ad_US[0];
		pm->quick_iUdc = 1.f / pm->const_fb_U;

		if (		pm->const_fb_U > pm->fault_voltage_halt
				&& pm->weak_D > - M_EPS_F) {

			pm->fsm_errno = PM_ERROR_DC_LINK_OVERVOLTAGE;
			pm->fsm_req = PM_STATE_HALT;
		}
	}

	/* This fallback makes possible the use of TVM voltages in case of
	 * no TVM capable hardware.
	 * */
	pm->tvm_DX = pm->vsi_DX;
	pm->tvm_DY = pm->vsi_DY;

	if (PM_CONFIG_TVM(pm) == PM_ENABLED) {

		/* Save previous cycle voltages.
		 * */
		vA = pm->fb_uA;
		vB = pm->fb_uB;
		vC = pm->fb_uC;

		/* Get instant terminal voltages.
		 * */
		pm->fb_uA = pm->ad_UA[1] * fb->voltage_A + pm->ad_UA[0];
		pm->fb_uB = pm->ad_UB[1] * fb->voltage_B + pm->ad_UB[0];
		pm->fb_uC = pm->ad_UC[1] * fb->voltage_C + pm->ad_UC[0];

		if (		pm->lu_MODE != PM_LU_DETACHED
				&& pm->tvm_INUSE == PM_ENABLED
				&& pm->vsi_UF == 0) {

			/* Extract the average terminal voltages using FIR.
			 * */

			vA *= pm->tvm_FIR_A[1];
			vB *= pm->tvm_FIR_B[1];
			vC *= pm->tvm_FIR_C[1];

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

				Q = .33333333f * (vA + vB + vC);
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
	}

	/* Get SENSOR values.
	 * */
	pm->fb_HS = fb->pulse_HS;
	pm->fb_EP = fb->pulse_EP;
	pm->fb_SIN = fb->analog_SIN;
	pm->fb_COS = fb->analog_COS;

	/* Main FSM is used to execute external commands.
	 * */
	pm_FSM(pm);

	if (pm->lu_MODE != PM_LU_DISABLED) {

		pm->hfi_INJECTED = PM_DISABLED;

		/* The observer FSM.
		 * */
		pm_lu_FSM(pm);

		if (pm->lu_MODE == PM_LU_DETACHED) {

			pm_voltage(pm, pm->vsi_X, pm->vsi_Y);
		}
		else {
			if (pm->config_LU_DRIVE == PM_DRIVE_SPEED) {

				pm_loop_speed(pm);
			}
			else if (pm->config_LU_DRIVE == PM_DRIVE_SERVO) {

				pm_loop_servo(pm);
				pm_loop_speed(pm);
			}

			/* Current loop is always enabled.
			 * */
			pm_loop_current(pm);
		}

		if (pm->config_MILEAGE_INFO == PM_ENABLED) {

			/* To collect mileage information.
			 * */
			pm_mileage_info(pm);
		}

		if (m_isfinitef(pm->lu_F[0]) == 0) {

			pm->fsm_errno = PM_ERROR_INVALID_OPERATION;
			pm->fsm_state = PM_STATE_HALT;
		}
	}
}

