#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "libc.h"

io_ops_t		io_USART;
io_ops_t		io_USB;
io_ops_t		io_CAN;

io_ops_t		*iodef;

int			iodef_ECHO;
int			iodef_PRETTY;

uint32_t		rseed;

void *memset(void *d, int c, int n)
{
	uint32_t	fill, *long_d = (uint32_t *) d;

	if (((uint32_t) long_d & 3UL) == 0) {

		fill = (uint8_t) c;
		fill |= (fill << 8);
		fill |= (fill << 16);

		while (n >= 4) {

			*long_d++ = fill;
			n += - 4;
		}
	}

	{
		uint8_t		*tail_d = (uint8_t *) long_d;

		while (n >= 1) {

			*tail_d++ = (uint8_t) c;
			n += - 1;
		}
	}

	return d;
}

void *memcpy(void *restrict d, const void *restrict s, int n)
{
	uint32_t	*restrict long_d = (uint32_t * restrict) d;
	const uint32_t	*restrict long_s = (const uint32_t * restrict) s;

	if (((uint32_t) long_d & 3UL) == 0 && ((uint32_t) long_s & 3UL) == 0) {

		while (n >= 4) {

			*long_d++ = *long_s++;
			n += - 4;
		}
	}

	{
		uint8_t		*restrict tail_d = (uint8_t * restrict) long_d;
		const uint8_t	*restrict tail_s = (const uint8_t * restrict) long_s;

		while (n >= 1) {

			*tail_d++ = *tail_s++;
			n += - 1;
		}
	}

	return d;
}

int strcmp(const char *s, const char *p)
{
	char		c;

	do {
		c = *s - *p;

		if (c || *s == 0)
			break;

		++s;
		++p;
	}
	while (1);

	return c;
}

int strcmpe(const char *s, const char *p)
{
	char		c;

	do {
		if (*s == 0)
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

int strcmpn(const char *s, const char *p, int n)
{
	char		c;
	int		l = 0;

	do {
		if (l >= n)
			break;

		c = *s - *p;

		if (c || *s == 0)
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

		b = p;
	}

	return NULL;
}

char *strcpy(char *restrict d, const char *restrict s)
{
	do {
		if ((*d = *s) == 0)
			break;

		++d;
		++s;
	}
	while (1);

	return d;
}

char *strcpyn(char *restrict d, const char *restrict s, int n)
{
	do {
		if (n < 1) {

			*d = 0;
			break;
		}

		if ((*d = *s) == 0)
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
		if (*s == 0)
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
		if (*s == 0)
			return NULL;

		if (*s == c)
			break;

		++s;
	}
	while (1);

	return s;
}

void xputs(io_ops_t *_io, const char *s)
{
	while (*s) _io->putc(*s++);
}

void xputsp(io_ops_t *_io, const char *s, int nleft)
{
	while (*s) {

		_io->putc(*s++);
		nleft--;
	}

	while (nleft > 0) {

		_io->putc(' ');
		nleft--;
	}
}

static void
fmt_hexb(io_ops_t *_io, int x)
{
	int		n, c;

	n = (x & 0xF0UL) >> 4;
	c = (n < 10) ? '0' + n : 'A' + (n - 10);
	_io->putc(c);

	n = (x & 0x0FUL);
	c = (n < 10) ? '0' + n : 'A' + (n - 10);
	_io->putc(c);
}

static void
fmt_hexh(io_ops_t *_io, int x)
{
	fmt_hexb(_io, (x & 0xFF00U) >> 8);
	fmt_hexb(_io, (x & 0x00FFU));
}

static void
fmt_hexl(io_ops_t *_io, uint32_t x)
{
	fmt_hexb(_io, (x & 0xFF000000UL) >> 24);
	fmt_hexb(_io, (x & 0x00FF0000UL) >> 16);
	fmt_hexb(_io, (x & 0x0000FF00UL) >> 8);
	fmt_hexb(_io, (x & 0x000000FFUL));
}

static void
fmt_intp(io_ops_t *_io, int x, int nleft)
{
	char		s[16], *p;
	int		n;

	if (x < 0) {

		_io->putc('-');
		x = - x;
		nleft--;
	}

	p = s + 16;
	*--p = 0;

	do {
		n = x % 10;
		x /= 10;
		*--p = '0' + n;
	}
	while (x);

	while (*p) {

		_io->putc(*p++);
		nleft--;
	}

	while (nleft > 0) {

		_io->putc(' ');
		nleft--;
	}
}

static void
fmt_float(io_ops_t *_io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0.f) {

		_io->putc('-');
		x = - x;
	}

	if ((0xFFUL & (u.i >> 23)) == 0xFFUL) {

		if ((u.i & 0x7FFFFFUL) != 0) {

			xputs(_io, "NaN");
		}
		else {
			xputs(_io, "Inf");
		}

		return ;
	}

	v = 0;
	h = .5f;

	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;

	while (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	i = (int) x;
	x -= i;

	_io->putc('0' + i);

	while (v > 0) {

		x *= 10.f;
		v--;

		i = (int) x;
		x -= i;

		_io->putc('0' + i);
	}

	_io->putc('.');

	while (n > 0) {

		x *= 10.f;
		n--;

		i = (int) x;
		x -= i;

		_io->putc('0' + i);
	}
}

static void
fmt_fexp(io_ops_t *_io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0) {

		_io->putc('-');
		x = - x;
	}

	if ((0xFFUL & (u.i >> 23)) == 0xFFUL) {

		if ((u.i & 0x7FFFFFUL) != 0) {

			xputs(_io, "NaN");
		}
		else {
			xputs(_io, "Inf");
		}

		return ;
	}

	v = 0;
	h = .5f;

	while (x > 0.f && x < 1.f) {

		x *= 10.f;
		v--;
	}

	while (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;

	if (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	i = (int) x;
	x -= i;

	_io->putc('0' + i);
	_io->putc('.');

	while (n > 0) {

		x *= 10.f;
		n--;

		i = (int) x;
		x -= i;

		_io->putc('0' + i);
	}

	_io->putc('E');

	if (v >= 0) {

		_io->putc('+');
	}

	fmt_intp(_io, v, 0);
}

static void
fmt_pretty(io_ops_t *_io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0) {

		_io->putc('-');
		x = - x;
	}

	if ((0xFFUL & (u.i >> 23)) == 0xFFUL) {

		if ((u.i & 0x7FFFFFUL) != 0) {

			xputs(_io, "NaN");
		}
		else {
			xputs(_io, "Inf");
		}

		return ;
	}

	v = 0;
	h = .5f;

	while (x > 0.f && x < 1.f) {

		x *= 10.f;
		v--;
	}

	while (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	n--;

	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;

	if (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	i = (int) x;
	x -= i;

	_io->putc('0' + i);

	while (v % 3 != 0) {

		x *= 10.f;
		v--;
		n--;

		i = (int) x;
		x -= i;

		_io->putc('0' + i);
	}

	_io->putc('.');

	while (n > 0) {

		x *= 10.f;
		n--;

		i = (int) x;
		x -= i;

		_io->putc('0' + i);
	}

	if (v == - 9) _io->putc('n');
	else if (v == - 6) _io->putc('u');
	else if (v == - 3) _io->putc('m');
	else if (v == 3) _io->putc('K');
	else if (v == 6) _io->putc('M');
	else if (v == 9) _io->putc('G');
	else if (v != 0) {

		_io->putc('E');

		if (v >= 0) {

			_io->putc('+');
		}

		fmt_intp(_io, v, 0);
	}
}

void xvprintf(io_ops_t *_io, const char *fmt, va_list ap)
{
	const char	*s;
	int		n = 0;

	while (*fmt) {

                if (*fmt == '%') {

                        ++fmt;

			if (*fmt >= '0' && *fmt <= '9') {

				n = *fmt++ - '0';

				if (*fmt >= '0' && *fmt <= '9')
					n = 10 * n + (*fmt++ - '0');
			}

			switch (*fmt) {

				case '%':
					_io->putc('%');
					break;

				case 'x':
					if (n == 2) {

						fmt_hexb(_io, va_arg(ap, int));
					}
					else if (n == 4) {

						fmt_hexh(_io, va_arg(ap, int));
					}
					else if (n == 8) {

						fmt_hexl(_io, va_arg(ap, uint32_t));
					}
					break;

				case 'i':
					fmt_intp(_io, va_arg(ap, int), n);
					break;

				case 'f':
					fmt_float(_io, * va_arg(ap, float *), n);
					break;

				case 'e':
					fmt_fexp(_io, * va_arg(ap, float *), n);
					break;

				case 'g':
					if (iodef_PRETTY != 0) {

						fmt_pretty(_io, * va_arg(ap, float *), n);
					}
					else {
						fmt_fexp(_io, * va_arg(ap, float *), n);
					}
					break;

				case 'c':
					_io->putc(va_arg(ap, int));
					break;

				case 's':
					s = va_arg(ap, const char *);
					xputsp(_io, (s != NULL) ? s : "(null)", n);
					break;
			}
		}
                else {
                        _io->putc(*fmt);
		}

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

int getc() { return iodef->getc(); }
void putc(int c) { if (iodef_ECHO != 0) { iodef->putc(c); } }

void puts(const char *s)
{
	if (iodef_ECHO != 0) {

		xputs(iodef, s);
	}
}

void printf(const char *fmt, ...)
{
        va_list		ap;

	if (iodef_ECHO != 0) {

		va_start(ap, fmt);
		xvprintf(iodef, fmt, ap);
		va_end(ap);
	}
}

const char *stoi(int *x, const char *s)
{
	int		n, k, i;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	k = 0;
	i = 0;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		k++;

		if (k > 9) return NULL;
	}

	if (k == 0) return NULL;

	if (*s == 0 || *s == ' ') { *x = i; }
	else return NULL;

	return s;
}

const char *htoi(int *x, const char *s)
{
	int		i, k, h;

	k = 0;
	h = 0;

	if (*s == '0' && *(s + 1) == 'x') { s += 2; }

	do {
		if (*s >= '0' && *s <= '9') { i = *s++ - '0'; }
		else if (*s >= 'A' && *s <= 'F') { i = 10 + *s++ - 'A'; }
		else if (*s >= 'a' && *s <= 'f') { i = 10 + *s++ - 'a'; }
		else break;

		h = 16 * h + i;
		k++;

		if (k > 8) return NULL;
	}
	while (1);

	if (k == 0) return NULL;

	if (*s == 0 || *s == ' ') { *x = h; }
	else return NULL;

	return s;
}

const char *stof(float *x, const char *s)
{
	int		n, k, v, e;
	float		f;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	k = 0;
	v = 0;
	f = 0.f;

	while (*s >= '0' && *s <= '9') {

		f = 10.f * f + (*s++ - '0') * n;
		k++;
	}

	if (*s == '.') {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10.f * f + (*s++ - '0') * n;
			k++; v--;
		}
	}

	if (k == 0) return NULL;

	if (*s == 'n') { v += - 9; s++; }
	else if (*s == 'u') { v += - 6; s++; }
	else if (*s == 'm') { v += - 3; s++; }
	else if (*s == 'K') { v += 3; s++; }
	else if (*s == 'M') { v += 6; s++; }
	else if (*s == 'G') { v += 9; s++; }
	else if (*s == 'e' || *s == 'E') {

		s = stoi(&e, s + 1);

		if (s != NULL) { v += e; }
		else return NULL;
	}

	if (*s == 0 || *s == ' ') {

		while (v < 0) { f /= 10.f; v++; }
		while (v > 0) { f *= 10.f; v--; }

		*x = f;
	}
	else return NULL;

	return s;
}

uint32_t crc32b(const void *s, int n)
{
	const uint32_t		*ls = (const uint32_t *) s;
	uint32_t		crc, buf;

	const uint32_t		mask[16] = {

		0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
		0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
		0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
		0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL
	};

	crc = 0xFFFFFFFFUL;

	while (n >= 4) {

		buf = *ls++;
		n += - 4;

		crc = crc ^ buf;

		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
		crc = (crc >> 4) ^ mask[crc & 0x0FUL];
	}

	return crc ^ 0xFFFFFFFFUL;
}

uint32_t urand()
{
	uint32_t	rval;

	rseed = rseed * 17317UL + 1UL;
	rval = rseed >> 16;

	return rval;
}

