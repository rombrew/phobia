#ifndef _H_BLM_
#define _H_BLM_

typedef struct {

	double		Tsim;
	double		dT, sT;
	int		PWM_R;

	/* Duty Cycle (INPUT).
	 * */
	int		PWM_A;
	int		PWM_B;
	int		PWM_C;

	/* Detached (INPUT).
	 * */
	int		HI_Z;

	/* State variabes.
	 * */
	double		X[14];
	int		VSI[3];
	int		surge_I;

	/* Cycle Power.
	 * */
	double		iP;

	/* Motor constants.
	 * */
	double		R;
	double		Ld;
	double		Lq;
	double		E;
	int		Zp;

	/* Thermal constants.
	 * */
	double		Ct;
	double		Rt;

	/* Source contants.
	 * */
	double		U;
	double		Rs;
	double		Cb;

	/* Mechanical constants.
	 * */
	double		J;
	double		M[4];

	/* Sensor constants.
	 * */
	double		T_ADC;
	double		tau_I;
	double		tau_U;

	/* Hall Sensors.
	 * */
	double		HS[3];

	/* Quadrature Encoder.
	 * */
	int		EP_R;
	double		EP_Zq;

	/* ADC result (OUTPUT).
	 * */
	float		ADC_IA;
	float		ADC_IB;
	float		ADC_US;

	float		ADC_UA;
	float		ADC_UB;
	float		ADC_UC;

	/* Hall Sensors (OUTPUT).
	 * */
	int		pulse_HS;

	/* Encoder Pulse (OUTPUT).
	 * */
	int		pulse_EP;
}
blm_t;

void blm_Enable(blm_t *m);
void blm_Update(blm_t *m);

#endif /* _H_BLM_ */

