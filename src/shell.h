#ifndef _H_SHELL_
#define _H_SHELL_

#define K_ETX			0x03	/* Ctrl + C */
#define K_EOT			0x04	/* Ctrl + D */
#define K_BS			0x08	/* Ctrl + H */
#define K_TAB			0x09	/* Ctrl + I */
#define K_LF			0x0A	/* Ctrl + J */
#define K_CR			0x0D	/* Ctrl + M */
#define K_SO			0x0E	/* Ctrl + N */
#define K_DLE			0x10	/* Ctrl + P */
#define K_ESC			0x1B	/* Ctrl + [ */
#define K_DEL			0x7F

#undef SH_DEF
#define SH_DEF(name)		void name(const char *s)

typedef struct {

	const char		*sym;
	void			(* proc) (const char *);
}
sh_cmd_t;

const char *sh_next_arg(const char *s);

void task_SH(void *);

#endif /* _H_SHELL_ */

