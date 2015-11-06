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

#include <stdarg.h>

#include "lib.h"
#include "sh.h"

#define PUTC(c)				shSend(c)

void *memz(void *p, int sz)
{
	int	*x = p;

	sz /= sizeof(int);

	while (sz > 0) {

		*x++ = 0;
		sz--;
	}

	return x;
}

int strcmp(const char *s, const char *p)
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

int strpcmp(const char *s, const char *p)
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

int strxcmp(const char *s, const char *p, int x)
{
	char		c;
	int		n = 0;

	do {
		if (n >= x)
			break;

		c = *s - *p;

		if (c || !*s)
			break;

		++s;
		++p;
		++n;
	}
	while (1);

	return n;
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

char *strncpy(char *d, const char *s, int n)
{
	do {
		if (n <= 0) {

			*d = 0;
			break;
		}

		if (!(*d = *s))
			break;

		++d;
		++s;
		--n;
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

const char *strtok(const char *s)
{
	while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;

	return s;
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

inline unsigned int
ftou(float x)
{
	union {
		float		f;
		unsigned int	i;
	} u;

	u.f = x;

	return u.i;
}

static void
fmt_float(float x, int n)
{
	int		i, be, ma;
	float		h;

	if (x < 0) {

		PUTC('-');
		x = - x;
	}

	be = (ftou(x) >> 23) & 0xFF;
	ma = ftou(x) & 0x7FFFFF;

	if (be == 0xFF) {

		if (ma != 0)

			puts("NaN");
		else
			puts("Inf");

		return ;
	}

	h = .5f;
	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;
	i = (int) x;
	fmt_int(i);
	x -= i;

	if (x < 1.f) {

		PUTC('.');

		while (n > 0) {

			x *= 10.f;
			i = (int) x;
			x -= i;

			PUTC('0' + i);
			n--;
		}
	}
}

static void
fmt_fexp(float x, int n)
{
	int		i, be, ma;
	int		de = 0;
	float		h;

	if (x < 0) {

		PUTC('-');
		x = - x;
	}

	be = (ftou(x) >> 23) & 0xFF;
	ma = ftou(x) & 0x7FFFFF;

	if (be == 0xFF) {

		if (ma != 0)

			puts("NaN");
		else
			puts("Inf");

		return ;
	}
	else if (be != 0) {

		do {
			be = ((ftou(x) >> 23) & 0xFF) - 127;

			if (be < 0) {

				x *= 10.f;
				de--;
			}
			else if (be > 3) {

				x /= 10.f;
				de++;
			}
			else
				break;
		}
		while (1);
	}

	h = .5f;
	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;
	i = (int) x;

	if (i > 9) {

		x /= 10.f;
		de++;
	}

	i = (int) x;
	x -= i;

	PUTC('0' + i);
	PUTC('.');

	while (n > 0) {

		x *= 10.f;
		i = (int) x;
		x -= i;

		PUTC('0' + i);
		n--;
	}

	PUTC('E');

	if (de >= 0)
		PUTC('+');

	fmt_int(de);
}

void printf(const char *fmt, ...)
{
        va_list		ap;
        const char	*s;
	int		n = 5;

        va_start(ap, fmt);

        while (*fmt) {

                if (*fmt == '%') {

                        ++fmt;

			if (*fmt >= '0' && *fmt <= '9')
				n = *fmt++ - '0';

			switch (*fmt) {

				case '%':
					PUTC('%');
					break;

				case 'i':
					fmt_int(va_arg(ap, int));
					break;

				case 'f':
					fmt_float(* va_arg(ap, float *), n);
					break;

				case 'e':
					fmt_fexp(* va_arg(ap, float *), n);
					break;

				case 's':
					s = va_arg(ap, const char *);
					puts(s);
					break;
			}
		}
                else
                        PUTC(*fmt);

                ++fmt;
        }

        va_end(ap);
}

const char *stoi(int *x, const char *s)
{
	int		n = 1, k = 0, i = 0;

	if (*s == '-') {

		n = -1;
		s++;
	}
	else if (*s == '+')
		s++;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		k++;

		if (i * n < 0)
			return NULL;
	}

	if (k == 0)
		return NULL;

	if (*s == 0 || *s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')

		*x = i;
	else
		return NULL;

	return s;
}

const char *stof(float *x, const char *s)
{
	int		n = 1, k = 0, de = 0, e;
	float		f = 0.f;

	if (*s == '-') {

		n = -1;
		s++;
	}
	else if (*s == '+')
		s++;

	while (*s >= '0' && *s <= '9') {

		f = 10.f * f + (*s++ - '0') * n;
		k++;
	}

	if (*s == '.') {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10.f * f + (*s++ - '0') * n;
			k++; de--;
		}
	}

	if (k == 0)
		return NULL;

	if (*s == 'n')
		de += -9, s++;
	else if (*s == 'u')
		de += -6, s++;
	else if (*s == 'm')
		de += -3, s++;
	else if (*s == 'K')
		de += 3, s++;
	else if (*s == 'M')
		de += 6, s++;
	else if (*s == 'G')
		de += 9, s++;
	else if (*s == 'e' || *s == 'E') {

		s = stoi(&e, s + 1);

		if (s != NULL)
			de += e;
		else
			return NULL;
	}

	if (*s == 0 || *s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {

		while (de < 0) {

			f /= 10.f;
			de++;
		}

		while (de > 0) {

			f *= 10.f;
			de--;
		}

		*x = f;
	}
	else
		return NULL;

	return s;
}

