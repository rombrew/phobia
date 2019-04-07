#ifndef _H_PM_
#define _H_PM_

#define PM_CONFIG_ABC(pm)			(pm)->config_ABC
#define PM_CONFIG_VOLT(pm)			(pm)->config_VOLT

#define PM_UMAX(pm)	((PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) ? 2.f / 3.f : 1.f)
#define PM_EMAX(pm)	((PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) ? .57735027f : .70710678f)
#define PM_KWAT(pm)	((PM_CONFIG_ABC(pm) == PM_ABC_THREE_PHASE) ? 1.5f : 1.f)

#define PM_UNRESTRICTED				1E+35f
#define PM_SFI(s)				#s

enum {
	PM_ABC_THREE_PHASE			= 0,
	PM_ABC_TWO_PHASE,
};

enum {
	PM_LDQ_NON_SALIENT_POLE			= 0,	// assumes (Ld = Lq)
	PM_LDQ_SALIENT_POLE,				// assumes (Ld > Lq)
	PM_LDQ_SATURATION_SALIENCY			// assumes (Ld < Lq)
};

enum {
	PM_DISABLED				= 0,
	PM_ENABLED
};

enum {
	PM_LOOP_DRIVE_CURRENT			= 0,
	PM_LOOP_DRIVE_SPEED,
	PM_LOOP_DRIVE_SERVO,
	PM_LOOP_RECTIFIER_VOLTAGE,
};

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_DETACHED,
	PM_LU_FORCED,
	PM_LU_ESTIMATE_FLUX,
	PM_LU_ESTIMATE_HFI,
	PM_LU_SENSORED_HALL,
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_SELF_TEST_POWER_STAGE,
	PM_STATE_SELF_TEST_SAMPLING_ACCURACY,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_INITIATE,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_ADJUST_SENSOR_HALL,
	PM_STATE_HALT,
};

enum {
	PM_OK					= 0,
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERORR_POWER_STAGE_FAULT,
	PM_ERROR_ACCURACY_FAULT,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_OVER_CURRENT,
	PM_ERROR_RESIDUE_UNSTABLE,
	PM_ERROR_INVALID_OPERATION,
};

typedef struct {

	float		current_A;
	float		current_B;
	float		voltage_U;

	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		hall;
	float		sensor;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;

	int		dc_resolution;
	int		dc_minimal;
	int		dc_clearance;

	int		fail_reason;
	int		self_BM[8];
	float		self_RMS[2];

	int		config_ABC;
	int		config_LDQ;
	int		config_VOLT;
	int		config_HALL;
	int		config_HFI;
	int		config_LOOP;

	int		fsm_state;
	int		fsm_phase;
	int		fsm_phase_2;

	int		tm_value;
	int		tm_end;

	float		tm_transient_skip;
	float		tm_voltage_hold;
	float		tm_current_hold;
	float		tm_instant_probe;
	float		tm_average_drift;
	float		tm_average_probe;
	float		tm_startup;

	float		adjust_IA[2];
	float		adjust_IB[2];
	float		adjust_US[2];
	float		adjust_UA[2];
	float		adjust_UB[2];
	float		adjust_UC[2];

	float		fb_current_clamp;
	float		fb_current_A;
	float		fb_current_B;
	float		fb_voltage_A;
	float		fb_voltage_B;
	float		fb_voltage_C;
	int		fb_hall_A;
	int		fb_hall_B;
	int		fb_hall_C;

	float		probe_fb_X;
	float		probe_fb_Y;
	float		probe_current_hold_X;
	float		probe_current_hold_Y;
	float		probe_current_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_low;
	float		probe_speed_ramp;
	float		probe_gain_P;
	float		probe_gain_I;
	float		probe_DFT[8];
	float		probe_impedance_R;
	float		probe_rotation_DQ;
	float		probe_LSQ_A[9];
	float		probe_LSQ_B[9];
	float		probe_LSQ_C[9];

	float		FIX[27];

	float		fault_voltage_tolerance;
	float		fault_current_tolerance;
	float		fault_current_halt_level;
	float		fault_adjust_tolerance;
	float		fault_flux_residue_maximal;

	int		vsi_precise_MODE;
	int		vsi_clamp_to_GND;
	int		vsi_current_ZONE;
	int		vsi_voltage_ZONE;
	float		vsi_X;
	float		vsi_Y;
	float		vsi_DX;
	float		vsi_DY;
	float		vsi_lpf_D;
	float		vsi_lpf_Q;
	float		vsi_lpf_watt;
	float		vsi_gain_LP;
	float		vsi_gain_LW;

	float		volt_maximal;
	float		volt_A;
	float		volt_B;
	float		volt_C;
	float		volt_FIR_A[3];
	float		volt_FIR_B[3];
	float		volt_FIR_C[3];
	float		volt_residue_X;
	float		volt_residue_Y;

	float		lu_X[5];
	int		lu_mode;
	int		lu_revol;

	float		forced_X[5];
	float		forced_hold_D;
	float		forced_accel;
	float		forced_setpoint;

	float		flux_X[5];
	float		flux_drift_Q;
	float		flux_residue_D;
	float		flux_residue_Q;
	float		flux_residue_lpf;
	float		flux_gain_LP;
	float		flux_gain_DA;
	float		flux_gain_QA;
	float		flux_gain_DP;
	float		flux_gain_DS;
	float		flux_gain_QS;
	float		flux_gain_QZ;
	float		flux_bemf_unlock;
	float		flux_bemf_lock;
	float		flux_bemf_high;

	float		hfi_X[5];
	float		hfi_freq_hz;
	float		hfi_swing_D;
	float		hfi_wave[2];
	float		hfi_flux;
	float		hfi_gain_P;
	float		hfi_gain_S;
	float		hfi_gain_F;

	float		hall_X[5];
	float		hall_speed_maximal;

	float		const_lpf_U;
	float		const_gain_LP;
	float		const_E;
	float		const_R;
	float		const_Ld;
	float		const_Lq;
	int		const_Zp;
	float		const_J;

	float		i_maximal;
	float		i_derated;
	float		i_watt_maximal;
	float		i_watt_reverse;
	float		i_watt_derated;
	float		i_setpoint_D;
	float		i_setpoint_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_gain_PD;
	float		i_gain_ID;
	float		i_gain_PQ;
	float		i_gain_IQ;

	float		s_maximal;
	float		s_setpoint;
	float		s_accel;
	float		s_track;
	float		s_integral;
	float		s_gain_P;
	float		s_gain_I;

	float		p_setpoint_DQ[2];
	float		p_setpoint_s;
	int		p_setpoint_revol;
	float		p_gain_P;
	float		p_gain_I;

	void 		(* proc_set_DC) (int, int, int);
	void 		(* proc_set_Z) (int);
}
pmc_t;

void pm_config_default(pmc_t *pm);
void pm_config_tune_current_loop(pmc_t *pm);

void pm_voltage_control(pmc_t *pm, float uX, float uY);
void pm_voltage_residue(pmc_t *pm);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

void pm_FSM(pmc_t *pm);
void pm_fsm_req(pmc_t *pm, int req);

const char *pm_strerror(int n);

#endif /* _H_PM_ */

