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

#ifndef _H_CAN_
#define _H_CAN_

#define CAN_MASK_BASE_ID		0xFFE00000UL
#define CAN_MASK_EXTENDED_ID		0xFFFFFFF8UL

typedef struct {

	int		id;
	char		payload[8];
}
can_msg_t;

void canEnable();
void canDisable();

void canFilter(int nFilter, int bID, int bMask, int nFifo);
int canEmpty();
int canTransmit(int bID, int nBytes, const char bData[8]);

extern void canIRQ();

#endif /* _H_CAN_ */

