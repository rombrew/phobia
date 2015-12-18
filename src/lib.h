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

#ifndef _H_LIB_
#define _H_LIB_

#define NULL			((void *) 0L)
#define EOL			"\r\n"

void *memz(void *p, int sz);

int strcmp(const char *s, const char *p);
int strpcmp(const char *s, const char *p);
int strspl(const char *s, const char *p, int x);
char *strcpy(char *d, const char *s);
char *strncpy(char *d, const char *s, int n);
int strlen(const char *s);
const char *strchr(const char *s, int c);
const char *strtok(const char *s, const char *d);

void putc(char c);
void puts(const char *s);
void printf(const char *fmt, ...);

const char *stoi(int *x, const char *s);
const char *stof(float *x, const char *s);

#endif /* _H_LIB_ */

