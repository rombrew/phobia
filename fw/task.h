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

#ifndef _H_TASK_
#define _H_TASK_

typedef struct {

	/* Time from power up (ms).
	 * */
	unsigned long int	mTIM;

	/* Pending flags.
	 * */
	unsigned char		xIN;
	unsigned char		xOUT;
	unsigned char		xSH;

	/* Use bxCAN transport.
	 * */
	unsigned char		xCAN;

	/* Busy mask.
	 * */
	int			mBUSY;

}
taskDATA_t;

extern taskDATA_t		td;

void taskYield();

#endif /* _H_TASK_ */

