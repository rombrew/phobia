#ifndef _H_LIBC_
#define _H_LIBC_

#include <stddef.h>
#include <stdint.h>

#define EOL			"\r\n"

#define	IODEF_TO_USART()	if (iodef != &io_USART) { iodef = &io_USART; }
#define	IODEF_TO_USB()		if (iodef != &io_USB) { iodef = &io_USB; }
#define	IODEF_TO_CAN()		if (iodef != &io_CAN) { iodef = &io_CAN; }

#define URAND_MAX		65535U

typedef struct {

	int		(* getc) ();
	void		(* putc) (int c);
}
io_ops_t;

/* Serial IO interfaces.
 * */
extern io_ops_t		io_USART;
extern io_ops_t		io_USB;
extern io_ops_t		io_CAN;

/* Currently used.
 * */
extern io_ops_t		*iodef;

/* EHCO mode.
 * */
extern int		iodef_ECHO;

/* Random SEED.
 * */
extern uint32_t		rseed;

void *memset(void *d, int c, int n);
void *memcpy(void *restrict d, const void *restrict s, int n);

int strcmp(const char *s, const char *p);
int strcmpe(const char *s, const char *p);
int strcmpn(const char *s, const char *p, int x);
const char *strstr(const char *s, const char *p);
char *strcpy(char *restrict d, const char *restrict s);
char *strcpyn(char *restrict d, const char *restrict s, int n);
int strlen(const char *s);
const char *strchr(const char *s, int c);

void xputs(io_ops_t *_io, const char *s);
void xprintf(io_ops_t *_io, const char *fmt, ...);

int getc();
void putc(int c);
void puts(const char *s);
void printf(const char *fmt, ...);

const char *stoi(int *x, const char *s);
const char *htoi(int *x, const char *s);
const char *stof(float *x, const char *s);

uint32_t crc32b(const void *s, int n);
uint32_t urand();

#endif /* _H_LIBC_ */

