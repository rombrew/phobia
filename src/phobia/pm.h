#ifndef _H_PM_
#define _H_PM_

#include "libm.h"
#include "lse.h"

#define PM_CONFIG_NOP(pm)	(pm)->config_NOP
#define PM_CONFIG_IFB(pm)	(pm)->config_IFB
#define PM_CONFIG_TVM(pm)	(pm)->config_TVM
#define PM_CONFIG_DBG(pm)	(pm)->config_DBG

#define PM_TSMS(pm, ms)		(int) ((pm)->m_freq * (ms) * 0.001f)
#define PM_DTNS(pm, ns)		((ns) * (pm)->m_freq * 0.000000001f)

#define PM_MAX_F		1000000000000.f
#define PM_SFI(s)		#s

enum {
	PM_Z_NONE				= 0,
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
	PM_SALIENCY_NONE			= 0,
	PM_SALIENCY_NEGATIVE,
	PM_SALIENCY_POSITIVE
};

enum {
	PM_VSI_GND				= 0,
	PM_VSI_CENTER,
	PM_VSI_EXTREME
};

enum {
	PM_DISABLED				= 0,
	PM_ENABLED
};

enum {
	PM_FLUX_NONE				= 0,
	PM_FLUX_ORTEGA,
	PM_FLUX_KALMAN
};

enum {
	PM_SENSOR_NONE				= 0,
	PM_SENSOR_HALL,
	PM_SENSOR_EABI,
	PM_SENSOR_SINCOS
};

enum {
	PM_LOCATION_NONE			= 0,
	PM_LOCATION_INHERITED,
	PM_LOCATION_EABI,
	PM_LOCATION_SINCOS
};

enum {
	PM_DRIVE_CURRENT			= 0,
	PM_DRIVE_TORQUE,
	PM_DRIVE_SPEED,
	PM_DRIVE_LOCATION
};

enum {
	PM_HFI_NONE				= 0,
	PM_HFI_SINE,
	PM_HFI_SQUARE,
	PM_HFI_RANDOM
};

enum {
	PM_IRON_NONE				= 0,
	PM_IRON_SATURATION
};

enum {
	PM_MAGNET_NONE				= 0,
	PM_MAGNET_PERMANENT
};

enum {
	PM_EABI_INCREMENTAL			= 0,
	PM_EABI_ABSOLUTE
};

enum {
	PM_SINCOS_ANALOG			= 0,
	PM_SINCOS_RESOLVER,
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
	PM_LU_ESTIMATE,
	PM_LU_ON_HFI,
	PM_LU_SENSOR_HALL,
	PM_LU_SENSOR_EABI,
	PM_LU_SENSOR_SINCOS
};

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_SELF_TEST_BOOTSTRAP,
	PM_STATE_SELF_TEST_POWER_STAGE,
	PM_STATE_SELF_TEST_CLEARANCE,
	PM_STATE_ADJUST_ON_PCB_VOLTAGE,
	PM_STATE_ADJUST_ON_PCB_CURRENT,
	PM_STATE_ADJUST_DCU_VOLTAGE,
	PM_STATE_PROBE_CONST_RESISTANCE,
	PM_STATE_PROBE_CONST_INDUCTANCE,
	PM_STATE_LU_DETACHED,
	PM_STATE_LU_STARTUP,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_FLUX_LINKAGE,
	PM_STATE_PROBE_CONST_INERTIA,
	PM_STATE_PROBE_NOISE_THRESHOLD,
	PM_STATE_ADJUST_SENSOR_HALL,
	PM_STATE_ADJUST_SENSOR_EABI,
	PM_STATE_ADJUST_SENSOR_SINCOS,
	PM_STATE_HALT
};

enum {
	PM_AUTO_BASIC_DEFAULT			= 0,
	PM_AUTO_CONFIG_DEFAULT,
	PM_AUTO_MACHINE_DEFAULT,
	PM_AUTO_SCALE_DEFAULT,
	PM_AUTO_MAXIMAL_CURRENT,
	PM_AUTO_PROBE_SPEED_HOLD,
	PM_AUTO_FORCED_MAXIMAL,
	PM_AUTO_FORCED_ACCEL,
	PM_AUTO_ZONE_THRESHOLD,
	PM_AUTO_LOOP_CURRENT,
	PM_AUTO_LOOP_SPEED
};

enum {
	PM_OK					= 0,

	/* Internal.
	 * */
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERROR_BOOTSTRAP_FAULT,
	PM_ERROR_POWER_STAGE_BROKEN,
	PM_ERROR_LOW_CURRENT_ACCURACY,
	PM_ERROR_LOW_VOLTAGE_ACCURACY,
	PM_ERROR_LOW_DEADBAND_ACCURACY,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_INSTANT_OVERCURRENT,
	PM_ERROR_DC_LINK_OVERVOLTAGE,
	PM_ERROR_UNCERTAIN_RESULT,
	PM_ERROR_NAN_OPERATION,
	PM_ERROR_SENSOR_HALL_FAULT,
	PM_ERROR_SENSOR_EABI_FAULT,
	PM_ERROR_SENSOR_SINCOS_FAULT,

	/* Application level.
	 * */
	PM_ERROR_TIMEOUT,
	PM_ERROR_NO_FLUX_CAUGHT,
	PM_ERROR_NO_SYNC_FAULT,
	PM_ERROR_KNOB_SIGNAL_FAULT,
	PM_ERROR_SPI_DATA_FAULT,

	/* Arise by hardware.
	 * */
	PM_ERROR_HW_UNMANAGED_IRQ,
	PM_ERROR_HW_OVERCURRENT,
	PM_ERROR_HW_OVERTEMPERATURE,
	PM_ERROR_HW_EMERGENCY_STOP
};

typedef struct {

	float		current_A;
	float		current_B;
	float		current_C;
	float		voltage_U;
	float		voltage_A;
	float		voltage_B;
	float		voltage_C;

	float		analog_SIN;
	float		analog_COS;

	int		pulse_HS;
	int		pulse_EP;
}
pmfb_t;

typedef struct {

	float		m_freq;
	float		m_dT;

	int		dc_resolution;
	float		dc_minimal;
	float		dc_clearance;
	float		dc_skip;
	float		dc_bootstrap;

	float		k_UMAX;
	float		k_EMAX;
	float		k_KWAT;

	int		ts_minimal;
	int		ts_clearance;
	int		ts_skip;
	int		ts_bootstrap;
	float		ts_inverted;

	float		self_BST[3];
	int		self_IST[8];
	float		self_STDi[3];
	float		self_RMSi[3];
	float		self_RMSu;
	float		self_RMSt[3];
	float		self_DTu;

	int		config_NOP;
	int		config_IFB;
	int		config_TVM;
	int		config_DBG;

	int		config_VSI_ZERO;
	int		config_VSI_CLAMP;
	int		config_DCU_VOLTAGE;
	int		config_LU_FORCED;
	int		config_LU_FREEWHEEL;
	int		config_LU_ESTIMATE;
	int		config_LU_SENSOR;
	int		config_LU_LOCATION;
	int		config_LU_DRIVE;
	int		config_HFI_WAVETYPE;
	int		config_HFI_PERMANENT;
	int		config_SATURATION;
	int		config_EXCITATION;
	int		config_SALIENCY;
	int		config_RELUCTANCE;
	int		config_WEAKENING;
	int		config_CC_BRAKE_STOP;
	int		config_CC_SPEED_TRACK;
	int		config_EABI_FRONTEND;
	int		config_SINCOS_FRONTEND;

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
	float		tm_current_ramp;
	float		tm_instant_probe;
	float		tm_average_probe;
	float		tm_average_drift;
	float		tm_average_inertia;
	float		tm_average_outside;
	float		tm_pause_startup;
	float		tm_pause_forced;
	float		tm_pause_halt;

	float		scale_iA[2];
	float		scale_iB[2];
	float		scale_iC[2];
	float		scale_uS[2];
	float		scale_uA[2];
	float		scale_uB[2];
	float		scale_uC[2];

	float		fb_iA;
	float		fb_iB;
	float		fb_iC;
	float		fb_uA;
	float		fb_uB;
	float		fb_uC;
	float		fb_SIN;
	float		fb_COS;
	int		fb_HS;
	int		fb_EP;

	float		probe_current_hold;
	float		probe_weak_level;
	float		probe_hold_angle;
	float		probe_current_sine;
	float		probe_current_bias;
	float		probe_freq_sine;
	float		probe_speed_hold;
	float		probe_speed_tol;
	float		probe_location_tol;
	float		probe_loss_maximal;
	float		probe_gain_P;
	float		probe_gain_I;

	float		probe_DFT[8];
	float		probe_REM[8];
	float		probe_HF[2];
	float		probe_gain_LP;
	float		probe_HOLD[2];

	float		fault_voltage_tol;
	float		fault_current_tol;
	float		fault_accuracy_tol;
	float		fault_terminal_tol;
	float		fault_current_halt;
	float		fault_voltage_halt;

	float		vsi_DC;
	float		vsi_lpf_DC;
	float		vsi_X;
	float		vsi_Y;
	float		vsi_gain_LP;

	int		vsi_AF;
	int		vsi_BF;
	int		vsi_CF;
	int		vsi_IF;
	int		vsi_UF;
	int		vsi_AT;
	int		vsi_BT;
	int		vsi_CT;
	int		vsi_A0;
	int		vsi_B0;
	int		vsi_C0;

	float		dcu_deadband;
	float		dcu_tol;
	float		dcu_DX;
	float		dcu_DY;
	float		dcu_X;
	float		dcu_Y;

	int		lu_MODE;
	float		lu_iX;
	float		lu_iY;
	float		lu_iD;
	float		lu_iQ;
	float		lu_uD;
	float		lu_uQ;
	float		lu_F[3];
	float		lu_wS;
	float		lu_location;
	int		lu_revol;
	int		lu_revob;
	int		lu_total_revol;
	float		lu_transient;
	float		lu_mq_produce;
	float		lu_mq_load;
	float		lu_wS_prev;
	float		lu_gain_mq_LP;

	int		base_TIM;
	int		hold_TIM;

	float		forced_F[2];
	float		forced_wS;
	float		forced_hold_D;
	float		forced_weak_D;
	float		forced_maximal;
	float		forced_reverse;
	float		forced_accel;
	float		forced_slew_rate;
	float		forced_fall_rate;
	float		forced_track_D;
	float		forced_stop_DC;
	float		forced_gain_LS;

	int		flux_DETACH;
	int		flux_LINKAGE;

	int		detach_TIM;
	float		detach_threshold;
	float		detach_trip_tol;
	float		detach_gain_SF;

	int		flux_TYPE;
	int		flux_ZONE;

	float		flux_X[2];
	float		flux_F[2];
	float		flux_wS;
	float		flux_lambda;
	float		flux_trip_tol;
	float		flux_gain_IN;
	float		flux_gain_LO;
	float		flux_gain_HI;
	float		flux_gain_SF;
	float		flux_gain_IF;

	int		kalman_POSTPONED;

	float		kalman_P[15];
	float		kalman_A[10];
	float		kalman_K[10];
	float		kalman_rsu_D;
	float		kalman_rsu_Q;
	float		kalman_bias_Q;
	float		kalman_lpf_wS;
	float		kalman_gain_Q[4];
	float		kalman_gain_R;

	float		zone_noise;
	float		zone_threshold;
	float		zone_lpf_wS;
	float		zone_gain_TH;
	float		zone_gain_LP;

	float		hfi_maximal;
	float		hfi_freq;
	float		hfi_amplitude;
	float		hfi_wave[2];

	struct {

		float	X;
		float	Y;
	}
	hall_ST[8];

	int		hall_ERN;
	float		hall_F[2];
	float		hall_wS;
	float		hall_trip_tol;
	float		hall_gain_LO;
	float		hall_gain_SF;
	float		hall_gain_IF;

	int		eabi_RECENT;
	int		eabi_ADJUST;
	int		eabi_bEP;
	int		eabi_lEP;
	int		eabi_unwrap;
	float		eabi_interp;
	float		eabi_F0[2];
	int		eabi_const_EP;
	int		eabi_const_Zs;
	int		eabi_const_Zq;
	float		eabi_F[2];
	float		eabi_wS;
	float		eabi_location;
	float		eabi_trip_tol;
	float		eabi_gain_LO;
	float		eabi_gain_SF;
	float		eabi_gain_IF;

	int		sincos_RECENT;
	float		sincos_CONST[16];
	float		sincos_SC[3];
	int		sincos_revol;
	int		sincos_unwrap;
	int		sincos_const_Zs;
	int		sincos_const_Zq;
	float		sincos_F[2];
	float		sincos_wS;
	float		sincos_location;
	float		sincos_gain_PF;
	float		sincos_gain_SF;
	float		sincos_gain_IF;

	float		const_fb_U;
	float		const_lambda;
	float		const_Rs;
	int		const_Zp;
	float		const_Ja;
	float		const_im_Ld;
	float		const_im_Lq;
	float		const_im_A;
	float		const_im_Rz;
	float		const_ld_Sm;

	float		quick_iU;
	float		quick_iWb;
	float		quick_iWb2;
	float		quick_iLd;
	float		quick_iLq;
	float		quick_Lrel;
	float		quick_iL4rel;
	float		quick_TiLd;
	float		quick_TiLq;
	float		quick_TiLu[4];
	float		quick_HFwS;
	float		quick_HF[2];
	float		quick_ZiEP;
	float		quick_ZiSQ;

	int		watt_DC_MAX;
	int		watt_DC_MIN;

	float		watt_wP_maximal;
	float		watt_wA_maximal;
	float		watt_wP_reverse;
	float		watt_wA_reverse;
	float		watt_uDC_maximal;
	float		watt_uDC_minimal;
	float		watt_uDC_tol;
	float		watt_lpf_D;
	float		watt_lpf_Q;
	float		watt_drain_wP;
	float		watt_drain_wA;
	float		watt_traveled;
	float		watt_consumed_Wh;
	float		watt_consumed_Ah;
	float		watt_reverted_Wh;
	float		watt_reverted_Ah;
	float		watt_capacity_Ah;
	float		watt_fuel_gauge;
	float		watt_rem[4];
	float		watt_integral;
	float		watt_gain_P;
	float		watt_gain_I;
	float		watt_gain_LP;
	float		watt_gain_WF;

	float		i_setpoint_current;
	float		i_setpoint_torque;
	float		i_maximal;
	float		i_maximal_on_PCB;
	float		i_reverse;
	float		i_track_D;
	float		i_track_Q;
	float		i_integral_D;
	float		i_integral_Q;
	float		i_slew_rate;
	float		i_damping;
	float		i_gain_P;
	float		i_gain_I;

	float		mtpa_revstep;
	float		mtpa_setpoint_Q;
	float		mtpa_load_Q;
	float		mtpa_track_D;
	float		mtpa_gain_LP;

	float		weak_maximal;
	float		weak_track_D;
	float		weak_gain_EU;

	float		v_maximal;
	float		v_reverse;

	float		s_setpoint_speed;
	float		s_maximal;
	float		s_reverse;
	float		s_track;
	float		s_integral;
	float		s_accel_forward;
	float		s_accel_reverse;
	float		s_damping;
	float		s_gain_P;
	float		s_gain_I;
	float		s_gain_D;

	float		l_track;
	float		l_track_tol;
	float		l_blend;
	float		l_gain_LP;

	float		x_setpoint_location;
	float		x_setpoint_speed;
	float		x_maximal;
	float		x_minimal;
	float		x_boost_tol;
	float		x_track_tol;
	float		x_gain_P;
	float		x_gain_D;

	float		dbg_flux_rsu;

	void 		(* proc_set_DC) (int, int, int);
	void 		(* proc_set_Z) (int);

	lfseed_t	lfseed;
	lse_t		lse[2];
}
pmc_t;

void pm_quick_build(pmc_t *pm);
void pm_auto(pmc_t *pm, int req);

float pm_torque_equation(pmc_t *pm, float iD, float iQ);
float pm_torque_maximal(pmc_t *pm, float iQ);

void pm_clearance(pmc_t *pm, int xA, int xB, int xC);
void pm_voltage(pmc_t *pm, float uX, float uY);

void pm_FSM(pmc_t *pm);
void pm_feedback(pmc_t *pm, pmfb_t *fb);

const char *pm_strerror(int fsm_errno);

#endif /* _H_PM_ */

