#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "libc.h"

io_ops_t		io_USART;
io_ops_t		io_USB;
io_ops_t		io_CAN;

io_ops_t		*iodef;

uint32_t		rseed;

void *memset(void *d, int c, size_t len)
{
	uint32_t	fill, *ld = (uint32_t *) d;

	if (likely(((uint32_t) ld & 3U) == 0)) {

		fill = (uint8_t) c;
		fill |= (fill << 8);
		fill |= (fill << 16);

		while (len >= 4U) {

			*ld++ = fill;
			len -= 4U;
		}
	}

	{
		uint8_t		*xd = (uint8_t *) ld;

		while (len >= 1U) {

			*xd++ = (uint8_t) c;
			len -= 1U;
		}
	}

	return d;
}

void *memcpy(void *restrict d, const void *restrict s, size_t len)
{
	uint32_t	*restrict ld = (uint32_t * restrict) d;
	const uint32_t	*restrict ls = (const uint32_t * restrict) s;

	if (likely(((uint32_t) ld & 3U) == 0 && ((uint32_t) ls & 3U) == 0)) {

		while (len >= 4U) {

			*ld++ = *ls++;
			len -= 4U;
		}
	}

	{
		uint8_t		*restrict xd = (uint8_t * restrict) ld;
		const uint8_t	*restrict xs = (const uint8_t * restrict) ls;

		while (len >= 1U) {

			*xd++ = *xs++;
			len -= 1U;
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

		if (c != 0)
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

		if (c != 0 || *s == 0)
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

char *strcpyn(char *restrict d, const char *restrict s, int len)
{
	do {
		if (len < 1) {

			*d = 0;
			break;
		}

		if ((*d = *s) == 0)
			break;

		++d;
		++s;
		--len;
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

void xputs(io_ops_t *io, const char *s)
{
	while (*s) io->putc(*s++);
}

void xputs_aligned(io_ops_t *io, const char *s, int len)
{
	while (*s) {

		io->putc(*s++);
		len--;
	}

	for (; len > 0; --len) {

		io->putc(' ');
	}
}

static void
fmt_hex_byte(io_ops_t *io, int x)
{
	int		n, c;

	n = (x & 0xF0U) >> 4;
	c = (n < 10) ? '0' + n : 'A' + (n - 10);

	io->putc(c);

	n = (x & 0x0FU);
	c = (n < 10) ? '0' + n : 'A' + (n - 10);

	io->putc(c);
}

static void
fmt_hex_short(io_ops_t *io, uint16_t x)
{
	union {
		uint16_t	x;
		uint8_t		b[2];
	}
	u = { x };

	fmt_hex_byte(io, u.b[1]);
	fmt_hex_byte(io, u.b[0]);
}

static void
fmt_hex_long(io_ops_t *io, uint32_t x)
{
	union {
		uint32_t	x;
		uint8_t		b[4];
	}
	u = { x };

	fmt_hex_byte(io, u.b[3]);
	fmt_hex_byte(io, u.b[2]);
	fmt_hex_byte(io, u.b[1]);
	fmt_hex_byte(io, u.b[0]);
}

static void
fmt_int_aligned(io_ops_t *io, int x, int len)
{
	char		s[16], *p;
	int		n;

	if (x < 0) {

		io->putc('-');

		x = - x;
		len--;
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

		io->putc(*p++);
		len--;
	}

	for (; len > 0; --len) {

		io->putc(' ');
	}
}

static void
fmt_fp_fixed(io_ops_t *io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0.f) {

		io->putc('-');

		x = - x;
	}

	if ((0xFFU & (u.i >> 23)) == 0xFFU) {

		if ((u.i & 0x7FFFFFU) != 0) {

			xputs(io, "NaN");
		}
		else {
			xputs(io, "Inf");
		}

		return ;
	}

	v = 0;
	h = 0.5f;

	for (i = 0; i < n; ++i)
		h /= 10.f;

	x += h;

	while (x >= 10.f) {

		x /= 10.f;
		v++;
	}

	i = (int) x;
	x -= (float) i;

	io->putc('0' + i);

	for (; v > 0; --v) {

		x *= 10.f;

		i = (int) x;
		x -= (float) i;

		io->putc('0' + i);
	}

	io->putc('.');

	for (; n > 0; --n) {

		x *= 10.f;

		i = (int) x;
		x -= (float) i;

		io->putc('0' + i);
	}
}

static void
fmt_fp_normal(io_ops_t *io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0.f) {

		io->putc('-');

		x = - x;
	}

	if ((0xFFU & (u.i >> 23)) == 0xFFU) {

		if ((u.i & 0x7FFFFFU) != 0) {

			xputs(io, "NaN");
		}
		else {
			xputs(io, "Inf");
		}

		return ;
	}

	v = 0;
	h = 0.5f;

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
	x -= (float) i;

	io->putc('0' + i);
	io->putc('.');

	for (; n > 0; --n) {

		x *= 10.f;

		i = (int) x;
		x -= (float) i;

		io->putc('0' + i);
	}

	io->putc('E');

	if (v >= 0) {

		io->putc('+');
	}

	fmt_int_aligned(io, v, 0);
}

static void
fmt_fp_pretty(io_ops_t *io, float x, int n)
{
	union {
		float		f;
		uint32_t	i;
	}
	u = { x };

	int		i, v;
	float		h;

	if (x < 0.f) {

		io->putc('-');

		x = - x;
	}

	if ((0xFFU & (u.i >> 23)) == 0xFFU) {

		if ((u.i & 0x7FFFFFU) != 0) {

			xputs(io, "NaN");
		}
		else {
			xputs(io, "Inf");
		}

		return ;
	}

	v = 0;
	h = 0.5f;

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
	x -= (float) i;

	io->putc('0' + i);

	for (; v % 3 != 0; --v, --n) {

		x *= 10.f;

		i = (int) x;
		x -= (float) i;

		io->putc('0' + i);
	}

	io->putc('.');

	for (; n > 0; --n) {

		x *= 10.f;

		i = (int) x;
		x -= (float) i;

		io->putc('0' + i);
	}

	if (v == - 9) io->putc('n');
	else if (v == - 6) io->putc('u');
	else if (v == - 3) io->putc('m');
	else if (v == 3) io->putc('K');
	else if (v == 6) io->putc('M');
	else if (v == 9) io->putc('G');
	else if (v != 0) {

		io->putc('E');

		if (v >= 0) {

			io->putc('+');
		}

		fmt_int_aligned(io, v, 0);
	}
}

void xvprintf(io_ops_t *io, const char *fmt, va_list ap)
{
	const char	*s;
	int		n;

	while (*fmt) {

                if (*fmt == '%') {

			n = 0;

			++fmt;

			if (*fmt == '*') {

				n = va_arg(ap, int);

				++fmt;
			}
			else {
				while (*fmt >= '0' && *fmt <= '9') {

					n = 10 * n + (*fmt - '0');

					++fmt;
				}
			}

			switch (*fmt) {

				case '%':
					io->putc('%');
					break;

				case 'x':
					if (n == 2) {

						fmt_hex_byte(io, va_arg(ap, int));
					}
					else if (n == 4) {

						fmt_hex_short(io, va_arg(ap, int));
					}
					else if (n == 8) {

						fmt_hex_long(io, va_arg(ap, uint32_t));
					}
					break;

				case 'i':
					fmt_int_aligned(io, va_arg(ap, int), n);
					break;

				case 'f':
					fmt_fp_fixed(io, * va_arg(ap, float *), n);
					break;

				case 'e':
					fmt_fp_normal(io, * va_arg(ap, float *), n);
					break;

				case 'g':
					fmt_fp_pretty(io, * va_arg(ap, float *), n);
					break;

				case 'c':
					io->putc(va_arg(ap, int));
					break;

				case 's':
					s = va_arg(ap, const char *);
					xputs_aligned(io, (s != NULL) ? s : "(null)", n);
					break;
			}
		}
                else {
                        io->putc(*fmt);
		}

                ++fmt;
        }
}

void xprintf(io_ops_t *io, const char *fmt, ...)
{
        va_list		ap;

        va_start(ap, fmt);
	xvprintf(io, fmt, ap);
        va_end(ap);
}

int getc() { return iodef->getc(); }
int poll() { return iodef->poll(); }
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
	int		n, d, i;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	i = 0;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		d += 1;
	}

	if (d == 0 || d > 9) { return NULL; }

	if (*s == 0 || *s == ' ') { *x = i; }
	else { return NULL; }

	return s;
}

const char *htoi(int *x, const char *s)
{
	int		i, d, h;

	d = 0;
	h = 0;

	if (*s == '0' && *(s + 1) == 'x') { s += 2; }

	do {
		if (*s >= '0' && *s <= '9') { i = *s++ - '0'; }
		else if (*s >= 'A' && *s <= 'F') { i = 10 + *s++ - 'A'; }
		else if (*s >= 'a' && *s <= 'f') { i = 10 + *s++ - 'a'; }
		else break;

		h = 16 * h + i;
		d += 1;
	}
	while (1);

	if (d == 0 || d > 8) { return NULL; }

	if (*s == 0 || *s == ' ') { *x = h; }
	else { return NULL; }

	return s;
}

const char *stof(float *x, const char *s)
{
	int		n, d, v, e;
	float		f;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	v = 0;
	f = 0.f;

	while (*s >= '0' && *s <= '9') {

		f = 10.f * f + (*s++ - '0') * n;
		d += 1;
	}

	if (*s == '.') {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10.f * f + (*s++ - '0') * n;
			d += 1; v -= 1;
		}
	}

	if (d == 0) { return NULL; }

	if (*s == 'n') { v += - 9; s++; }
	else if (*s == 'u') { v += - 6; s++; }
	else if (*s == 'm') { v += - 3; s++; }
	else if (*s == 'K') { v += 3; s++; }
	else if (*s == 'M') { v += 6; s++; }
	else if (*s == 'G') { v += 9; s++; }
	else if (*s == 'e' || *s == 'E') {

		s = stoi(&e, s + 1);

		if (s != NULL) { v += e; }
		else { return NULL; }
	}

	if (*s == 0 || *s == ' ') {

		while (v < 0) { f /= 10.f; v += 1; }
		while (v > 0) { f *= 10.f; v -= 1; }

		*x = f;
	}
	else { return NULL; }

	return s;
}

uint32_t crc32u(const void *raw, size_t len)
{
	const uint32_t		*ip = (const uint32_t *) raw;
	uint32_t		crcsum, seq;

	static const uint32_t	lt[16] = {

		0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU,
		0x76DC4190U, 0x6B6B51F4U, 0x4DB26158U, 0x5005713CU,
		0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU,
		0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU
	};

	crcsum = 0xFFFFFFFFU;

	while (len >= 4U) {

		seq = *ip++;
		len += - 4U;

		crcsum = crcsum ^ seq;

		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
		crcsum = (crcsum >> 4) ^ lt[crcsum & 0x0FU];
	}

	return crcsum ^ 0xFFFFFFFFU;
}

uint32_t urand()
{
	rseed = rseed * 17317U + 1U;

	return rseed >> 16;
}

