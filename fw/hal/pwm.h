/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#ifndef _H_PWM_
#define _H_PWM_

enum {
	PWM_A			= 1,
	PWM_B			= 2,
	PWM_C			= 4,
};

typedef struct {

	/* Frequency (Hz).
	 * */
	int		hzF;

	/* Resolution.
	 * */
	int		R;

	/* Dead Time (ns).
	 * */
	int		nsD;
}
halPWM_TypeDef;

extern halPWM_TypeDef		halPWM;

void pwmEnable();
void pwmDisable();

void pwmDC(int uA, int uB, int uC);
void pwmZ(int Z);

#endif /* _H_BRIDGE_ */

