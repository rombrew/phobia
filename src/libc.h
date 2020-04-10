#ifndef _H_LIBC_
#define _H_LIBC_

#define EOL			"\r\n"

typedef unsigned long		u32_t;
typedef unsigned char		u8_t;

typedef struct {

	int		(* getc) ();
	void		(* putc) (int c);
}
io_ops_t;

extern io_ops_t		*iodef;

void *memset(void *d, int c, int n);
void *memcpy(void *d, const void *s, int n);

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

int getc();
void putc(int c);
void puts(const char *s);
void printf(const char *fmt, ...);

const char *stoi(int *x, const char *s);
const char *stof(float *x, const char *s);

u32_t crc32b(const void *s, int n);

#endif /* _H_LIBC_ */

