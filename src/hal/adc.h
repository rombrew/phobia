/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

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

typedef struct {

	/* Current sensors.
	 * */
	int		xA;
	int		xB;

	/* Supply voltage.
	 * */
	int		xU;

	/* Thermal sensor.
	 * */
	int		xT;

	/* Reference voltage.
	 * */
	int		xR;
}
halADC_t;

extern halADC_t			halADC;

void adcEnable();
void adcDisable();

extern void adcIRQ();

#endif /* _H_ADC_ */

