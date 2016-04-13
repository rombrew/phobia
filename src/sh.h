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

#ifndef _H_SH_
#define _H_SH_

#define SH_DEF(name)		void name(const char *s)
#define SH_ENTRY(name)		{ #name, &name}

typedef struct {

	const char		*iD;
	void			(* pF) (const char *);
}
shCMD_t;

int shRecv();
void shSend(int xC);

int shExRecv();
int shExSend(int xC);

void shTask();

#endif /* _H_SH_ */

