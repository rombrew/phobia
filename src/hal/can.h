/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

void CAN_startup();
void CAN_set_filter(int nfilt, int fifo, unsigned long ID, unsigned long mID);
void CAN_send_msg(unsigned long ID, int len, const unsigned char payload[8]);

extern void CAN_IRQ();

#endif /* _H_CAN_ */

