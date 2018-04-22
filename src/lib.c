/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2017 Roman Belov <romblv@gmail.com>

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

#include <stddef.h>
#include <stdarg.h>

#include "lib.h"

io_ops_t		*iodef;

void *memset(void *d, int c, unsigned long sz)
{
	char		*cd = (char *) d;

	while (sz >= 1) {

		*cd++ = c;
		sz--;
	}

	return d;
}

void *memcpy(void *d, const void *s, unsigned long sz)
{
	long			*ld = (long *) d;
	const long		*ls = (const long *) s;

	if (!((size_t) d & (sizeof(long) - 1UL))
		&& !((size_t) s & (sizeof(long) - 1UL))) {

		while (sz >= sizeof(long)) {

			*ld++ = *ls++;
			sz -= sizeof(long);
		}
	}

	{
		char		*cd = (char *) ld;
		const char	*cs = (const char *) ls;

		while (sz >= 1) {

			*cd++ = *cs++;
			sz--;
		}
	}

	return d;
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

int strspl(const char *s, const char *p, int n)
{
	char		c;
	int		l = 0;

	do {
		if (l >= n)
			break;

		c = *s - *p;

		if (c || !*s)
			break;

		++s;
		++p;
		++l;
	}
	while (1);

	return l;
}

const char *strstr(const char *s, const char *p)
{
	const char		*a, *b;

	if (*p == 0)
		return s;

	for (b = p; *s != 0; ++s) {

		if (*s != *b)
			continue;

		a = s;

		do {
			if (*b == 0)
				return s;

			if (*a++ != *b++)
				break;
		}
		while (1);

		b = s;
	}

	return NULL;
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

const char *strchr(const char *s, int c)
{
	do {
		if (!*s)
			return NULL;

		if (*s == c)
			break;

		++s;
	}
	while (1);

	return s;
}

const char *strtok(const char *s, const char *d)
{
	while (*s != 0 && strchr(d, *s) == NULL) ++s;
	while (*s != 0 && strchr(d, *s) != NULL) ++s;

	return s;
}

void xputs(io_ops_t *_io, const char *s)
{
	while (*s) _io->putc(*s++);
}

static void
fmt_hexb(io_ops_t *_io, int x)
{
	int		n, c;

	n = (x & 0xF0) >> 8;
	c = (n < 10) ? '0' + n : 'A' + (n - 10);
	_io->putc(c);

	n = (x & 0x0F);
	c = (n < 10) ? '0' + n : 'A' + (n - 10);
	_io->putc(c);
}

static void
fmt_hexl(io_ops_t *_io, unsigned long x)
{
	fmt_hexb(_io, (x & 0xFF000000) >> 24);
	fmt_hexb(_io, (x & 0x00FF0000) >> 16);
	fmt_hexb(_io, (x & 0x0000FF00) >> 8);
	fmt_hexb(_io, (x & 0x000000FF));
}

static void
fmt_int(io_ops_t *_io, int x)
{
	char		s[16], *p;
	int		n;

	if (x < 0) {

		_io->putc('-');
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

	while (*p) _io->putc(*p++);
}

static void
fmt_float(io_ops_t *_io, float x, int n)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x };

	int		i;
	float		h;

	if (x < 0) {

		_io->putc('-');
		x = - x;
	}

	if ((0xFF & (u.i >> 23)) == 0xFF) {

		if ((u.i & 0x7FFFFF) != 0)

			xputs(_io, "NaN");
		else
			xputs(_io, "Inf");

		return ;
	}

	h = .5f;
	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;
	i = (int) x;
	fmt_int(_io, i);
	x -= i;

	if (x < 1.f) {

		_io->putc('.');

		while (n > 0) {

			x *= 10.f;
			i = (int) x;
			x -= i;

			_io->putc('0' + i);
			n--;
		}
	}
}

static void
fmt_fexp(io_ops_t *_io, float x, int n)
{
	union {
		float		f;
		unsigned long	i;
	}
	u = { x };

	int		i, e;
	float		h;

	if (x < 0) {

		_io->putc('-');
		x = - x;
	}

	if ((0xFF & (u.i >> 23)) == 0xFF) {

		if ((u.i & 0x7FFFFF) != 0)

			xputs(_io, "NaN");
		else
			xputs(_io, "Inf");

		return ;
	}
	else {
		e = 0;

		do {
			if (x < 1.f) {

				x *= 10.f;
				e--;
			}
			else if (x > 10.f) {

				x /= 10.f;
				e++;
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

	if (x > 10.f) {

		x /= 10.f;
		e++;
	}

	i = (int) x;
	x -= i;

	_io->putc('0' + i);
	_io->putc('.');

	while (n > 0) {

		x *= 10.f;
		i = (int) x;
		x -= i;

		_io->putc('0' + i);
		n--;
	}

	_io->putc('E');

	if (e >= 0)
		_io->putc('+');

	fmt_int(_io, e);
}

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap)
{
	const char	*s;
	int		n = 5;

	while (*fmt) {

                if (*fmt == '%') {

                        ++fmt;

			if (*fmt >= '0' && *fmt <= '9')
				n = *fmt++ - '0';

			switch (*fmt) {

				case '%':
					_io->putc('%');
					break;

				case 'x':
					if (n == 2) {

						fmt_hexb(_io, va_arg(ap, int));
					}
					else {
						fmt_hexl(_io, va_arg(ap, unsigned long));
					}
					break;

				case 'i':
					fmt_int(_io, va_arg(ap, int));
					break;

				case 'f':
					fmt_float(_io, * va_arg(ap, float *), n);
					break;

				case 'e':
					fmt_fexp(_io, * va_arg(ap, float *), n);
					break;

				case 'c':
					_io->putc(va_arg(ap, int));
					break;

				case 's':
					s = va_arg(ap, const char *);
					xputs(_io, s);
					break;
			}
		}
                else
                        _io->putc(*fmt);

                ++fmt;
        }
}

void xprintf(io_ops_t *_io, const char *fmt, ...)
{
        va_list		ap;

        va_start(ap, fmt);
	xvprintf(_io, fmt, ap);
        va_end(ap);
}

void puts(const char *s)
{
	xputs(iodef, s);
}

void printf(const char *fmt, ...)
{
        va_list		ap;

        va_start(ap, fmt);
	xvprintf(iodef, fmt, ap);
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

	if (*s == 0 || strchr(" ", *s) != NULL)

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

	if (*s == 0 || strchr(" ", *s) != NULL) {

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

unsigned long crc32b(const void *s, int sz)
{
	const char		*bs = (const char *) s;
	unsigned long		byte, crc, mask;
	int			j;

	crc = 0xFFFFFFFF;

	while (sz >= 1) {

		byte = *bs++;
		crc = crc ^ byte;

		for (j = 7; j >= 0; j--) {

			mask = -(crc & 1);
			crc = (crc >> 1) ^ (0xEDB88320 & mask);
		}

		sz--;
	}

	return ~crc;
}

