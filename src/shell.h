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

#ifndef _H_SHELL_
#define _H_SHELL_

#define K_ETX			0x03
#define K_EOT			0x04
#define K_SO			0x0E
#define K_DLE			0x10
#define K_ESC			0x1B

#define SH_DEF(name)		void name(const char *s)
#define SH_ENTRY(name)		{ #name, &name}

#define SH_ASSERT(x)		if ((x) == 0) { printf("ASSERT: %s" EOL, #x); return ; }

typedef struct {

	const char		*iD;
	void			(* pF) (const char *);
}
shCMD_t;

void taskSH(void *pvParameters);

#endif /* _H_SHELL_ */

