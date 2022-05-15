#ifndef _H_PM_
#define _H_PM_

#include "lse.h"

#define PM_CONFIG_NOP(pm)	(pm)->config_NOP
#define PM_CONFIG_IFB(pm)	(pm)->config_IFB
#define PM_CONFIG_TVM(pm)	(pm)->config_TVM
#define PM_CONFIG_DEBUG(pm)	(pm)->config_DEBUG

#define PM_TSMS(pm, ms)		(int) (pm->freq_hz * (ms) * 0.001f)

#define PM_HALL_SPAN		1.05f
#define PM_MAX_F		1000000000000.f
#define PM_SFI(s)		#s

enum {
	PM_Z_NUL				= 0,
	PM_Z_A,
	PM_Z_B,
	PM_Z_AB,
	PM_Z_C,
	PM_Z_AC,
	PM_Z_BC,
	PM_Z_ABC
};

enum {
	PM_NOP_THREE_PHASE			= 0,
	PM_NOP_TWO_PHASE
};

enum {
	PM_IFB_AB_INLINE			= 0,
	PM_IFB_AB_GND,
	PM_IFB_ABC_INLINE,
	PM_IFB_ABC_GND
};

enum {
	PM_DISABLED				= 0,
	PM_ENABLED
};

enum {
	PM_ESTIMATE_NONE			= 0,
	PM_ESTIMATE_ORTEGA,
	PM_ESTIMATE_LUENBERGER
};

enum {
	PM_LOCATION_INHERITED			= 0,
	PM_LOCATION_ABI,
	PM_LOCATION_SINCOS
};

enum {
	PM_SINCOS_PLAIN				= 0,
	PM_SINCOS_ANALOG,
	PM_SINCOS_RESOLVER,
};

enum {
	PM_DRIVE_CURRENT			= 0,
	PM_DRIVE_SPEED,
	PM_DRIVE_SERVO
};

enum {
	PM_ZONE_NONE				= 0,
	PM_ZONE_UNCERTAIN,
	PM_ZONE_HIGH,
	PM_ZONE_LOCKED_IN_DETACH
};

enum {
	PM_LU_DISABLED				= 0,
	PM_LU_DETACHED,
	PM_LU_FORCED,
	PM_LU_ESTIMATE_FLUX,
	PM_LU_ESTIMATE_HFI,
	PM_LU_SENSOR_HALL,
	PM_LU_SENSOR_ABI,
	PM_LU_SENSOR_SINCOS
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_SELF_TEST_BOOTSTRAP,
	PM_STATE_SELF_TEST_POWER_STAGE,
	PM_STATE_SELF_TEST_CLEARANCE,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_DETACHED,
	PM_STATE_LU_STARTUP,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_ADJUST_SENSOR_HALL,
	PM_STATE_ADJUST_SENSOR_ABI,
	PM_STATE_ADJUST_SENSOR_SINCOS,
	PM_STATE_LOOP_BOOST,
	PM_STATE_HALT
};

enum {
	PM_TUNE_ALL_DEFAULT			= 0,
	PM_TUNE_PROBE_DEFAULT,
	PM_TUNE_MAXIMAL_CURRENT,
	PM_TUNE_LOOP_CURRENT,
	PM_TUNE_ZONE_THRESHOLD,
	PM_TUNE_LOOP_FORCED,
	PM_TUNE_LOOP_SPEED
};

enum {
	PM_OK					= 0,

	/* Internal.
	 * */
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERROR_BOOTSTRAP_FAULT,
	PM_ERROR_POWER_STAGE_DAMAGED,
	PM_ERROR_INSUFFICIENT_ACCURACY,
	PM_ERROR_CURRENT_LOOP_IS_OPEN,
	PM_ERROR_INSTANT_OVERCURRENT,
	PM_ERROR_DC_LINK_OVERVOLTAGE,
	PM_ERROR_UNCERTAIN_RESULT,
	PM_ERROR_INVALID_OPERATION,
	PM_ERROR_SENSOR_HALL_FAULT,

	/* Application level.
	 * */
	PM_ERROR_TIMEOUT,
	PM_ERROR_NO_FLUX_CAUGHT,
	PM_ERROR_LOSS_OF_SYNC,		/* BLM model only */

	/* Arise by hardware.
	 * */
	PM_ERROR_HW_OVERCURRENT,
	PM_ERROR_HW_STOP
};

typedef struct {

	float		current_A;
	float		current_B;
	float		current_C;
	float		voltage_U;
	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	int		pulse_HS;
	int		pulse_EP;

	float		analog_SIN;
	float		analog_COS;
}
pmfb_t;

typedef struct {

	float		freq_hz;
	float		dT;

	int		dc_resolution;
	float		dc_minimal;
	float		dc_clearance;
	float		dc_skip;
	float		dc_bootstrap;
	float		dc_clamped;

	float		k_UMAX;
	float		k_EMAX;
	float		k_KWAT;

	int		ts_minimal;
	int		ts_clearance;
	int		ts_skip;
	int		ts_bootstrap;
	int		ts_clamped;
	float		ts_inverted;

	float		self_BST[3];
	int		self_BM[8];
	float		self_RMSi[3];
	float		self_RMSu[4];

	int		config_NOP;
	int		config_IFB;
	int		config_TVM;
	int		config_DEBUG;

	int		config_VSI_CIRCULAR;
	int		config_VSI_PRECISE;
	int		config_LU_FORCED;
	int		config_LU_ESTIMATE_FLUX;
	int		config_LU_ESTIMATE_HFI;
	int		config_LU_SENSOR_HALL;
	int		config_LU_SENSOR_ABI;
	int		config_LU_SENSOR_SINCOS;
	int		config_LU_LOCATION;
	int		config_LU_DRIVE;
	int		config_HFI_MAJOR_AXES;
	int		config_ABI_ABSOLUTE;
	int		config_SINCOS_FRONTEND;
	int		config_HOLDING_BRAKE;
	int		config_SPEED_LIMITED;
	int		config_MTPA_RELUCTANCE;		/* TODO */
	int		config_WEAKENING;
	int		config_MILEAGE_INFO;
	int		config_BOOST_CHARGE;		/* TODO */

	int		fsm_req;
	int		fsm_state;
	int		fsm_phase;
	int		fsm_subi;
	int		fsm_errno;

	int		tm_value;
	int		tm_end;

	float		tm_transient_slow;
	float		tm_transient_fast;
	float		tm_voltage_hold;
	float		tm_current_hold;
	float		tm_instant_probe;
	float		tm_average_probe;
	float		tm_average_drift;
	float		tm_average_inertia;
	float		tm_startup;
	float		tm_halt_pause;

	float		ad_IA[2];
	float		ad_IB[2];
	float		ad_IC[2];
	float		ad_US[2];
	float		ad_UA[2];
	float		ad_UB[2];
	float		ad_UC[2];

	float		fb_iA;
	float		fb_iB;
	float		fb_iC;
	float		fb_uA;
	float		fb_uB;
	float		fb_uC;
	int		fb_HS;
	int		fb_EP;
	float		fb_SIN;
	float		fb_COS;

	float		probe_current_hold;
	float		probe_hold_angle;
	float		probe_current_weak;
	float		probe_current_sine;
	float		probe_freq_sine_hz;
	float		probe_speed_hold;
	float		probe_speed_detached;
	float		probe_gain_P;
	float		probe_gain_I;

	lse_t		probe_LS[3];

	float		fault_voltage_tol;
	float		fault_current_tol;
	float		fault_accuracy_tol;
	float		fault_terminal_tol;
	float		fault_current_halt;
	float		fault_voltage_halt;

	float		vsi_DC;
	float		vsi_X;
	float		vsi_Y;
	float		vsi_DX;
	float		vsi_DY;
	int		vsi_SA;
	int		vsi_SB;
	int		vsi_SC;
	int		vsi_TIM;
	int		vsi_AG;
	int		vsi_BG;
	int		vsi_CG;
	int		vsi_AF;
	int		vsi_BF;
	int		vsi_CF;
	int		vsi_XXF;
	int		vsi_SF;
	int		vsi_UF;
	int		vsi_AZ;
	int		vsi_BZ;
	int		vsi_CZ;
	float		vsi_lpf_DC;
	float		vsi_gain_LP_F;

	int		tvm_INUSE;
	float		tvm_range_DC;
	float		tvm_A;
	float		tvm_B;
	float		tvm_C;
	float		tvm_FIR_A[3];
	float		tvm_FIR_B[3];
	float		tvm_FIR_C[3];
	float		tvm_DX;
	float		tvm_DY;

	int		lu_MODE;
	float		lu_iX;
	float		lu_iY;
	float		lu_iD;
	float		lu_iQ;
	float		lu_F[3];
	float		lu_wS;
	float		lu_location;
	int		lu_revol;
	int		lu_revob;
	int		lu_total_revol;
	float		lu_transient;
	float		lu_lpf_torque;
	float		lu_base_wS;
	float		lu_gain_TQ;

	float		forced_F[2];
	float		forced_wS;
	float		forced_hold_D;
	float		forced_maximal;
	float		forced_reverse;
	float		forced_accel;
	float		forced_maximal_DC;
	int		forced_TIM;

	int		detach_LOCK;
	int		detach_TIM;
	float		detach_level_U;
	float		detach_trip_AD;
	float		detach_gain_SF;

	int		flux_ESTIMATE;
	int		flux_ZONE;

	float		flux_X[2];
	float		flux_E;
	float		flux_F[2];
	float		flux_wS;
	float		flux_trip_AD;
	float		flux_gain_IN;
	float		flux_gain_LO;
	float		flux_gain_HI;
	float		flux_gain_SF;
	float		flux_gain_IF;

	float		flux_QZ;
	float           flux_gain_DA;
        float           flux_gain_QA;
        float           flux_gain_DP;
        float           flux_gain_DS;
        float           flux_gain_QS;
        float           flux_gain_QZ;

	float		zone_lpf_wS;
	float		zone_MPPE;
	float		zone_MURE;
	float		zone_gain_TA;
	float		zone_gain_GI;
	float		zone_gain_ES;
	float		zone_gain_LP_S;

	int		hfi_INJECTED;
	float		hfi_F[2];
	float		hfi_wS;
	float		hfi_inject_sine;
	float		hfi_maximal;
	float		hfi_wave[2];
	float		hfi_im_L1;
	float		hfi_im_L2;
	float		hfi_im_R;
	int		hfi_INJS;
	int		hfi_SKIP;
	int		hfi_ESTI;
	int		hfi_TORQ;
	int		hfi_POLA;
	float		hfi_DFT[9];
	float		hfi_REM[12];
	int		hfi_IN;
	int		hfi_TIM;
	float		hfi_gain_SF;
	float		hfi_gain_IF;

	struct {

		float	X;
		float	Y;
	}
	hall_ST[8];

	int		hall_INUSE;
	int		hall_HS;
	int		hall_DIRF;
	float		hall_prol;
	int		hall_TIM;
	int		hall_ERR;
	float		hall_F[2];
	float		hall_wS;
	float		hall_prol_ms;
	float		hall_gain_PF;
	float		hall_gain_SF;
	float		hall_gain_IF;

	int		abi_INUSE;
	int		abi_in_EP;
	float		abi_in_F[2];
	int		abi_revol;
	int		abi_unwrap;
	int		abi_rel_EP;
	int		abi_loc_EP;
	int		abi_TIM;
	float		abi_prol;
	int		abi_EPPR;
	int		abi_gear_ZS;
	int		abi_gear_ZQ;
	float		abi_F[2];
	float		abi_wS;
	float		abi_location;
	float		abi_gain_PF;
	float		abi_gain_SF;
	float		abi_gain_IF;

	float		sincos_FIR[20];
	float		sincos_SC[3];
	int		sincos_revol;
	int		sincos_unwrap;
	int		sincos_gear_ZS;
	int		sincos_gear_ZQ;
	float		sincos_F[2];
	float		sincos_wS;
	float		sincos_location;
	float		sincos_gain_SF;
	float		sincos_gain_IF;

	float		const_fb_U;
	float		const_E;
	float		const_R;
	int		const_Zp;
	float		const_Ja;
	float		const_im_L1;
	float		const_im_L2;
	float		const_im_B;
	float		const_im_R;
	float		const_ld_S;

	float		quick_iUdc;
	float		quick_iE;
	float		quick_iEq;
	float		quick_iL1;
	float		quick_iL2;
	float		quick_hfwS;
	float		quick_hfSC[2];
	float		quick_ZiEP;
	float		quick_ZiSQ;

	float		watt_wP_maximal;
	float		watt_iDC_maximal;
	float		watt_wP_reverse;
	float		watt_iDC_reverse;
	float		watt_dclink_HI;
	float		watt_dclink_LO;
	float		watt_lpf_D;
	float		watt_lpf_Q;
	float		watt_lpf_wP;
	float		watt_gain_LP_F;
	float		watt_gain_LP_P;

	float		i_setpoint_torque;
	float		i_maximal;
	float		i_reverse;
	float		i_derated_PCB;
	float		i_derated_WEAK;
	float		i_track_D;
	float		i_track_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_slew_rate;
	float		i_tol_Z;
	float		i_gain_P;
	float		i_gain_I;

	float		weak_maximal;
	float		weak_D;
	float		weak_gain_EU;

	float		v_maximal;
	float		v_reverse;

	float		s_setpoint_speed;
	float		s_maximal;
	float		s_reverse;
	float		s_accel;
	float		s_linspan;
	float		s_track;
	float		s_tol_Z;
	float		s_gain_P;
	float		s_gain_H;
	float		s_iSP;

	float		x_setpoint_location;
	float		x_setpoint_speed;
	float		x_discrepancy;
	float		x_tol_NEAR;
	float		x_tol_Z;
	float		x_gain_P;
	float		x_gain_Z;

	float		im_distance;
	float		im_consumed_Wh;
	float		im_consumed_Ah;
	float		im_reverted_Wh;
	float		im_reverted_Ah;
	float		im_capacity_Ah;
	float		im_fuel_pc;
	float		im_REM[4];

	float		boost_iIN_HI;
	float		boost_uIN_LO;
	float		boost_iDC_HI;
	float		boost_uDC_HI;
	float		boost_gain_P;
	float		boost_gain_I;

	void 		(* proc_set_DC) (int, int, int);
	void 		(* proc_set_Z) (int);
}
pmc_t;

void pm_build(pmc_t *pm);
void pm_tune(pmc_t *pm, int req);

void pm_hfi_DFT(pmc_t *pm, float la[5]);

void pm_clearance(pmc_t *pm, int xA, int xB, int xC);
void pm_voltage(pmc_t *pm, float uX, float uY);

void pm_feedback(pmc_t *pm, pmfb_t *fb);

void pm_FSM(pmc_t *pm);

const char *pm_strerror(int errno);

#endif /* _H_PM_ */

