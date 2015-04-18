/*
   Phobia DC Motor Controller for RC and robotics.
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

#ifndef _H_TEL_
#define _H_TEL_

#define	TELSZ			40000

typedef struct {

	short int		in[8];
	int			sz, en;

	int			av[8];
	int			num, dec;

	short int		pD[TELSZ];
	short int		*pZ;
}
tel_t;

extern tel_t			tel;

void telTask();
void telShow();

#endif /* _H_TEL_ */

