#include <stddef.h>
#include <stdarg.h>

#include "libc.h"

io_ops_t		io_USART;
io_ops_t		io_CAN;

io_ops_t		*iodef;
int			iodef_ECHO;

void __attribute__ ((used)) __aeabi_memclr4(void *d, int n)
{
	u32_t		*ld = (u32_t *) d;

	while (n >= 4) {

		*ld++ = 0UL;
		n += - 4;
	}
}

void *memset(void *d, int c, int n)
{
	u32_t		fill, *ld = (u32_t *) d;

	if (((u32_t) ld & 3UL) == 0) {

		fill = c;
		fill |= (fill << 8);
		fill |= (fill << 16);

		while (n >= 4) {

			*ld++ = fill;
			n += - 4;
		}
	}

	{
		u8_t		*xd = (u8_t *) ld;

		while (n >= 1) {

			*xd++ = c;
			n += - 1;
		}
	}

	return d;
}

void *memcpy(void *d, const void *s, int n)
{
	u32_t		*ld = (u32_t *) d;
	const u32_t	*ls = (const u32_t *) s;

	if (((u32_t) ld & 3UL) == 0 && ((u32_t) ls & 3UL) == 0) {

		while (n >= 4) {

			*ld++ = *ls++;
			n += - 4;
		}
	}

	{
		u8_t		*xd = (u8_t *) ld;
		const u8_t	*xs = (const u8_t *) ls;

		while (n >= 1) {

			*xd++ = *xs++;
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

		if (c || !*s)
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

int strcmpn(const char *s, const char *p, int n)
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

		b = p;
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

char *strcpyn(char *d, const char *s, int n)
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

void xputs(io_ops_t *_io, const char *s)
{
	while (*s) _io->putc(*s++);
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
fmt_hexl(io_ops_t *_io, u32_t x)
{
	fmt_hexb(_io, (x & 0xFF000000UL) >> 24);
	fmt_hexb(_io, (x & 0x00FF0000UL) >> 16);
	fmt_hexb(_io, (x & 0x0000FF00UL) >> 8);
	fmt_hexb(_io, (x & 0x000000FFUL));
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
		u32_t		i;
	}
	u = { x };

	int		i;
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

	h = .5f;
	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;

	if (x < (float) (1UL << 31)) {

		/* FIXME: Do not use conversion to INT.
		 * */

		i = (int) x;
		fmt_int(_io, i);
		x -= i;
	}
	else {
		xputs(_io, "MAX");

		return ;
	}

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
		u32_t		i;
	}
	u = { x };

	int		i, e;
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
	else {
		e = 0;

		do {
			if (x > 0.f && x < 1.f) {

				x *= 10.f;
				e--;
			}
			else if (x >= 10.f) {

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

	if (x >= 10.f) {

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
						fmt_hexl(_io, va_arg(ap, u32_t));
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
					xputs(_io, (s != NULL) ? s : "(null)");
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
void putc(int c) { iodef->putc(c); }

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
	else if (*s == '+') {

		s++;
	}

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		k++;

		if (i * n < 0) {

			return NULL;
		}
	}

	if (k == 0) {

		return NULL;
	}

	if (*s == 0 || strchr(" ", *s) != NULL) {

		*x = i;
	}
	else {
		return NULL;
	}

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
	else if (*s == '+') {

		s++;
	}
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

	if (k == 0) {

		return NULL;
	}

	if (*s == 'n') {

		de += -9, s++;
	}
	else if (*s == 'u') {

		de += -6, s++;
	}
	else if (*s == 'm') {

		de += -3, s++;
	}
	else if (*s == 'K') {

		de += 3, s++;
	}
	else if (*s == 'M') {

		de += 6, s++;
	}
	else if (*s == 'G') {

		de += 9, s++;
	}
	else if (*s == 'e' || *s == 'E') {

		s = stoi(&e, s + 1);

		if (s != NULL) {

			de += e;
		}
		else {
			return NULL;
		}
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
	else {
		return NULL;
	}

	return s;
}

u32_t crc32b(const void *s, int n)
{
	const u32_t		*ls = (const u32_t *) s;
	u32_t			crc, buf;

	const u32_t		mask[16] = {

		0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
		0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
		0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
		0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
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

