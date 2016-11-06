/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#ifndef _H_ADC_
#define _H_ADC_

#define ADC_THERMAL_FREQ_HZ			100UL

typedef struct {

	/* Current sensors.
	 * */
	float		sensor_A;
	float		sensor_B;

	/* Supply voltage.
	 * */
	float		supply_U;

	/* External voltage.
	 * */
	float		external_HVIN;

	/* NTC resistor.
	 * */
	int		thermal_xNTC;

	/* Thermal sensor.
	 * */
	int		thermal_xTEMP;

	/* Reference voltage.
	 * */
	int		in_xREF;
}
halADC_t;

typedef struct {

	float			A_1;
	float			B_1;
	float			U_1;
	float			HVIN_1;
	float			NTC[8];
	float			TEMP_0;
	float			TEMP_1;
	float			REF_1;
}
halADC_CONST_t;

extern halADC_t			halADC;
extern halADC_CONST_t		halADC_CONST;

void adcEnable();
void adcDisable();

extern void adcIRQ_feedback();
extern void adcIRQ_thermal();

#endif /* _H_ADC_ */

