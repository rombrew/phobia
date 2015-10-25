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

#include "lib.h"
#include "sh.h"
#include "tel.h"

tel_t				tel;

void telCapture()
{
	int			j, SZ;

	for (j = 0; j < tel.pSZ; ++j)
		tel.sAVG[j] += tel.pIN[j];

	tel.sCNT++;

	if (tel.sCNT >= tel.sDEC) {

		for (j = 0; j < tel.pSZ; ++j) {

			*tel.pZ++ = tel.sAVG[j] / tel.sCNT;
			tel.sAVG[j] = 0;
		}

		SZ = tel.pZ - tel.pD + tel.pSZ;
		tel.iEN = (SZ < TELSZ) ? tel.iEN : 0;

		tel.sCNT = 0;
	}
}

void telFlush()
{
	short int		*pZ;
	int			j;

	pZ = tel.pD;

	while (pZ < tel.pZ) {

		for (j = 0; j < tel.pSZ; ++j)
			printf("%i ", *pZ++);

		puts(EOL);
	}
}

