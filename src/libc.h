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

#ifndef _H_LIBC_
#define _H_LIBC_

#define EOL			"\r\n"

typedef struct {

	int		(* getc) ();
	void		(* putc) (int c);
}
io_ops_t;

extern io_ops_t		*iodef;

void *memset(void *d, int c, unsigned long sz);
void *memcpy(void *d, const void *s, unsigned long sz);

int strcmp(const char *s, const char *p);
int strcmpe(const char *s, const char *p);
int strcmpn(const char *s, const char *p, int x);
const char *strstr(const char *s, const char *p);
char *strcpy(char *d, const char *s);
char *strcpyn(char *d, const char *s, int n);
int strlen(const char *s);
const char *strchr(const char *s, int c);

void xputs(io_ops_t *_io, const char *s);
void xprintf(io_ops_t *_io, const char *fmt, ...);

void puts(const char *s);
void printf(const char *fmt, ...);

const char *stoi(int *x, const char *s);
const char *stof(float *x, const char *s);

unsigned long crc32b(const void *s, int sz);

#endif /* _H_LIBC_ */

