#ifndef _H_SHELL_
#define _H_SHELL_

#define K_ETX			0x03
#define K_EOT			0x04
#define K_SO			0x0E
#define K_DLE			0x10
#define K_ESC			0x1B

#define SH_DEF(name)		void name(const char *s)

typedef struct {

	const char		*sym;
	void			(* proc) (const char *);
}
sh_cmd_t;

const char *sh_next_arg(const char *s);

void taskSH(void *); 

#endif /* _H_SHELL_ */

