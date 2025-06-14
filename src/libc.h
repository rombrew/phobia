#ifndef _H_LIBC_
#define _H_LIBC_

#include <stddef.h>
#include <stdint.h>

#ifndef likely
#define likely(x)		__builtin_expect(!!(x), 1)
#endif

#ifndef unlikely
#define unlikely(x)		__builtin_expect(!!(x), 0)
#endif

#define LD_LIBC			__attribute__ ((noinline, used))
#define LD_TASK			__attribute__ ((noinline))

#define EOL			"\r\n"

#define	IODEF_TO_USART()	if (unlikely(iodef != &io_USART)) { iodef = &io_USART; }
#define	IODEF_TO_USB()		if (unlikely(iodef != &io_USB)) { iodef = &io_USB; }
#define	IODEF_TO_CAN()		if (unlikely(iodef != &io_CAN)) { iodef = &io_CAN; }

#define URAND_MAX		65535U

typedef struct {

	int		(* getc) ();
	int		(* poll) ();
	void		(* putc) (int c);
}
io_ops_t;

/* Serial IO interfaces.
 * */
extern io_ops_t		io_USART;
extern io_ops_t		io_USB;
extern io_ops_t		io_CAN;

/* Currently used serial.
 * */
extern io_ops_t		*iodef;

/* Random SEED.
 * */
extern uint32_t		rseed;

void *memset(void *d, int c, size_t len) LD_LIBC;
void *memcpy(void *restrict d, const void *restrict s, size_t len) LD_LIBC;

int strcmp(const char *s, const char *p);
int strcmps(const char *s, const char *p);
int strclen(const char *s, const char *p, int len);
const char *strstr(const char *s, const char *p);
char *strcpy(char *restrict d, const char *restrict s);
char *strncpy(char *restrict d, const char *restrict s, int len);
int strlen(const char *s);
const char *strchr(const char *s, int c);

void xputs(io_ops_t *io, const char *s);
void xprintf(io_ops_t *io, const char *fmt, ...);

int getc();
int poll();
void putc(int c);

void puts(const char *s);
void printf(const char *fmt, ...);

const char *stoi(int *x, const char *s);
const char *htoi(int *x, const char *s);
const char *stof(float *x, const char *s);

uint32_t crc32u(const void *raw, size_t len);
uint32_t urand();

#endif /* _H_LIBC_ */

