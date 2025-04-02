#ifndef _H_BLM_
#define _H_BLM_

enum {
	BLM_Z_NONE		= 0,
	BLM_Z_DETACHED
};

typedef struct {

	double		time;
	double		sol_dT;

	int		unsync_flag;

	double		pwm_dT;
	double		pwm_deadtime;
	double		pwm_minimal;
	int		pwm_resolution;

	double		Dtol;

	int		pwm_A;
	int		pwm_B;
	int		pwm_C;
	int		pwm_Z;

	double		state[15];
	double		drain_wP;

	int		xfet[6];
	int		xdtu[3];
	int		revol;

	struct {

		int	ev;
		int	comp;
	}
	event[9];

	double		Rs;
	double		Ld;
	double		Lq;
	double		lambda;
	int		Zp;

	double		Ta;
	double		Ct;
	double		Rt;

	double		Udc;
	double		Rdc;
	double		Cdc;

	double		Jm;
	double		Mq[4];

	double		adc_Tconv;
	double		adc_Toffset;

	double		tau_A;
	double		tau_B;
	double		range_A;
	double		range_B;

	double		hall[3];

	int		eabi_ERES;
	int		eabi_WRAP;
	double		eabi_Zq;

	double		analog_Zq;

	float		hold_iA;
	float		hold_iB;
	float		hold_iC;

	float		analog_iA;
	float		analog_iB;
	float		analog_iC;
	float		analog_uS;
	float		analog_uA;
	float		analog_uB;
	float		analog_uC;

	int		pulse_HS;
	int		pulse_EP;

	float		analog_SIN;
	float		analog_COS;

	void 		(* proc_step) (double);
}
blm_t;

void blm_AB_DQ(double theta, double A, double B, double *D, double *Q);
void blm_DQ_ABC(double theta, double D, double Q, double *A, double *B, double *C);
double blm_Kv_lambda(blm_t *m, double Kv);

void blm_enable(blm_t *m);
void blm_restart(blm_t *m);
void blm_update(blm_t *m);

#endif /* _H_BLM_ */

