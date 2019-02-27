#ifndef _H_BLM_
#define _H_BLM_

typedef struct {

	double		Tsim;
	double		dT, sT;
	int		PWM_R;

	/* Duty Cycle (Input).
	 * */
	int		PWM_A;
	int		PWM_B;
	int		PWM_C;

	/* State of VSI.
	 * */
	int		VSI[3];
	int		surge_F;

	/* Satate variabes.
	 * */
	double		X[12];

	/* Cycle Power.
	 * */
	double		iP;

	/* Motor constants.
	 * */
	double		R;
	double		Ld;
	double		Lq;
	double		E;
	double		U;
	double		Rs;
	double		Cb;
	int		Zp;
	double		J;
	double		M[4];

	/* Sensor constants.
	 * */
	double		T_ADC;
	double		tau_I;
	double		tau_U;

	/* ADC result (Output).
	 * */
	float		ADC_IA;
	float		ADC_IB;
	float		ADC_US;

	float		ADC_UA;
	float		ADC_UB;
	float		ADC_UC;
}
blm_t;

void blm_Enable(blm_t *m);
void blm_Update(blm_t *m);

#endif /* _H_BLM_ */

