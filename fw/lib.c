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

#include <stdarg.h>

#include "lib.h"
#include "sh.h"

#define PUTC(c)				shSend(c)

char strcmp(const char *s, const char *p)
{
	char		c;

	do {
		c = *s - *p;

		if (c || !*s)
			break;

		++s;
		++p;
	}
	while (1);

	return c;
}

char strpcmp(const char *s, const char *p)
{
	char		c;

	do {
		if (!*s)
			return 0;

		c = *s - *p;

		if (c)
			break;

		++s;
		++p;
	}
	while (1);

	return c;
}

char *strcpy(char *d, const char *s)
{
	do {
		if (!(*d = *s))
			break;

		++d;
		++s;
	}
	while (1);

	return d;
}

int strlen(const char *s)
{
	int		len = 0;

	do {
		if (!*s)
			break;

		++s;
		++len;
	}
	while (1);

	return len;
}

void putc(char c) { PUTC(c); }

void puts(const char *s)
{
	while (*s) PUTC(*s++);
}

static void
fmt_int(int x)
{
	char		s[16], *p;
	int		n;

	if (x < 0) {

		PUTC('-');
		x = -x;
	}

	p = s + 16;
	*--p = 0;

	do {
		n = x % 10;
		x /= 10;
		*--p = '0' + n;
	}
	while (x);

	while (*p) PUTC(*p++);
}

static void
fmt_float(float x)
{
}

void printf(const char *fmt, ...)
{
        va_list		ap;
        const char	*sarg;

        va_start(ap, fmt);

        while (*fmt) {

                if (*fmt == '%') {

                        ++fmt;

			switch (*fmt) {

				case '%':
					PUTC('%');
					break;

				case 'i':
					fmt_int(va_arg(ap, int));
					break;

				case 's':
					sarg = va_arg(ap, const char*);
					while (*sarg)
						PUTC(*sarg++);
					break;
			}
		}
                else
                        PUTC(*fmt);

                ++fmt;
        }

        va_end(ap);
}

void *malloc(unsigned long int usz)
{
	static char	*pHeap = (void *) 0x10000000;
	void		*p;

	usz = (usz + 7UL) & ~7UL;
	p = pHeap;
	pHeap += usz;

	return p;
}

