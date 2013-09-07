#ifndef _H_PLANT_
#define _H_PLANT_

#define PLANT_STATE_SIZE	5

typedef struct {

	double		tsim;
	double		tdel;
	int		pwmf;

	/* Input variables.
	 * */
	double		u[3];
	int		i[3];

	/* State variabes.
	 * */
	double		x[PLANT_STATE_SIZE];

	/* BEMF waveform shape table.
	 * */
	double		tab_E_shape[361];

	/* Constants.
	 * */
	double		const_R;
	double		const_L;
	double		const_E;
	double		const_U;
	double		const_Z;
	double		const_J;
	double		const_M[4];

	/* Output variables.
	 * */
	int		z[2];
}
plant_t;

extern plant_t		plant;

void plant_enable();
void plant_update();

#endif /* _H_PLANT_ */

