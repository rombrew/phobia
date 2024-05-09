#include <stddef.h>

#include "hal/hal.h"

#include "shell.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "epcan.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "libc.h"

#define SH_CLINE_MAX			84
#define SH_HISTORY_MAX			240

#define SH_HIST_INC(n)                 	(((n) < (SH_HISTORY_MAX - 1)) ? (n) + 1 : 0)
#define SH_HIST_DEC(n)                 	(((n) > 0) ? (n) - 1 : SH_HISTORY_MAX - 1)

static const char
SH_ALLOWED[] = " +-_.",
SH_PROMPT[] = "(pmc) ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Command evaluate.
	 * */
	char		cline[SH_CLINE_MAX];
	int		ceol, xESC;
	char		*cargs;

	/* Completion feature.
	 * */
	int		mcomp, ceon, cnum;

	/* History feature.
	 * */
	char		cprev[SH_HISTORY_MAX];
	int		mprev, head, tail, pnum;
}
priv_sh_t;

#undef SH_DEF
#define SH_DEF(name)	void name(const char *s);
#include "shdefs.h"

const sh_cmd_t		cmLIST[] = {

#undef SH_DEF
#define SH_DEF(name)	{ #name, &name},
#include "shdefs.h"

	{NULL, NULL}
};

#define cmLIST_END	(cmLIST + sizeof(cmLIST) / sizeof(sh_cmd_t) - 2)

static int
sh_byte_is_letter(int c)
{
	return ((c >= 'a') && (c <= 'z'))
		|| ((c >= 'A') && (c <= 'Z'));
}

static int
sh_byte_is_digit(int c)
{
	return (c >= '0') && (c <= '9');
}

static void
sh_puts_erase(int n)
{
	while (n > 0) {

		puts(SH_BACKSPACE);
		--n;
	}
}

static void
sh_exact_match_call(priv_sh_t *sh)
{
	const sh_cmd_t		*cmd;
	const char		*id;

	cmd = cmLIST;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (strcmp(sh->cline, id) == 0) {

			/* Call the function.
			 * */
			cmd->proc(sh->cargs);

			break;
		}

		++cmd;
	}
	while (1);
}

static void
sh_cyclic_match(priv_sh_t *sh, int xd)
{
	const sh_cmd_t		*cmd;
	const char		*id;
	int			N = 0;

	cmd = cmLIST + sh->cnum;
	sh->cline[sh->ceon] = 0;

	do {
		cmd += (xd == DIR_UP) ? 1 : - 1;

		cmd = (cmd < cmLIST) ? cmLIST_END
			: (cmd > cmLIST_END) ? cmLIST : cmd;

		id = cmd->sym;

		if (strcmpe(sh->cline, id) == 0) {

			/* Copy the command name.
			 * */
			strcpyn(sh->cline, id, SH_CLINE_MAX - 2);

			break;
		}

		++N;

		if (N > (int) (cmLIST_END - cmLIST))
			break;
	}
	while (1);

	sh->cnum = cmd - cmLIST;
}

static void
sh_common_match(priv_sh_t *sh)
{
	const sh_cmd_t		*cmd;
	const char		*id, *sp;
	int			len;

	sp = NULL;
	cmd = cmLIST;

	sh->cnum = 0;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (strcmpe(sh->cline, id) == 0) {

			len = (sp != NULL) ? strcmpn(sp, id, len) : strlen(id);
			sp = id;

			sh->cnum++;
		}

		++cmd;
	}
	while (1);

	if (sp != NULL) {

		len = (len > SH_CLINE_MAX - 2) ? SH_CLINE_MAX - 2 : len;

		strcpyn(sh->cline, sp, len);
		sh->ceon = len;
	}
	else {
		sh->ceon = 0;
	}
}

static int
sh_history_move(priv_sh_t *sh, int xnum, int xd)
{
	if (xd == DIR_UP) {

		if (xnum != sh->head) {

			/* Get previous line.
			 * */
			xnum = SH_HIST_DEC(xnum);

			do {
				xnum = SH_HIST_DEC(xnum);

				if (sh->cprev[xnum] == 0)
					break;
			}
			while (1);

			xnum = SH_HIST_INC(xnum);
		}
	}
	else {
		if (xnum != sh->tail) {

			/* Get next line.
			 * */
			do {
				xnum = SH_HIST_INC(xnum);

				if (sh->cprev[xnum] == 0)
					break;
			}
			while (1);

			xnum = SH_HIST_INC(xnum);
		}
	}

	return xnum;
}

static void
sh_history_put(priv_sh_t *sh, const char *s)
{
	int			xnum, r;
	const char		*q = s;

	if (sh->head != sh->tail) {

		xnum = sh_history_move(sh, sh->tail, DIR_UP);

		do {
			r = sh->cprev[xnum] - *q;

			if (r || !*q)
				break;

			xnum = SH_HIST_INC(xnum);
			++q;
		}
		while (1);

		if (r == 0) {

			/* Do not put the same line again.
			 * */
			return ;
		}
	}

	do {
		sh->cprev[sh->tail] = *s;
		sh->tail = SH_HIST_INC(sh->tail);

		if (sh->tail == sh->head) {

			/* Forget old lines.
			 * */
			do {
				sh->head = SH_HIST_INC(sh->head);

				if (sh->cprev[sh->head] == 0)
					break;
			}
			while (1);

			sh->head = SH_HIST_INC(sh->head);
		}

		if (*s == 0)
			break;
		else
			++s;
	}
	while (1);
}

static char *
sh_markup_args(char *s)
{
	const char		*delim = " ";
	char			*argv, *q;
	int			argn, n;

	argv = NULL;
	argn = 1;

	q = s;
	n = 1;

	while (*s != 0) {

		if (strchr(delim, *s) == NULL) {

			if (n == 2) {

				argv = q;
			}

			*q++ = *s;

			n = 0;
		}
		else {
			if (n == 0) {

				*q++ = 0;
				argn++;
			}

			n = argn;
		}

		++s;
	}

	if (argv == NULL) { argv = q; }

	*(q + 0) = 0;
	*(q + 1) = 0;

	return argv;
}

static void
sh_evaluate(priv_sh_t *sh)
{
	char			*s;

	s = sh->cline;

	if (*s != 0) {

		/* Put the line in history.
		 * */
		sh_history_put(sh, s);

		/* Get the command line arguments.
		 * */
		sh->cargs = sh_markup_args(s);

		/* Search for specific command to execute.
		 * */
		sh_exact_match_call(sh);
	}
}

static void
sh_complete(priv_sh_t *sh, int xd)
{
	const char		space = ' ';
	char			*s;

	if (sh->mcomp == 0) {

		s = sh->cline;

		/* Do not complete with trailing spaces.
		 * */
		if (strchr(s, space) != NULL)
			return ;

		/* Complete to the common substring.
		 * */
		sh_common_match(sh);
		puts(sh->cline + sh->ceol);

		if (sh->cnum == 1) {

			/* Exact match.
			 * */
			sh->ceol = sh->ceon;

			if (sh->ceol < SH_CLINE_MAX - 2) {

				/* Put trailing space since completion is done.
				 * */
				sh->cline[sh->ceol++] = space;
				sh->cline[sh->ceol] = 0;

				putc(space);
			}
		}
		else if (sh->ceol <= sh->ceon) {

			/* Enter completion mode.
			 * */
			sh->mcomp = 1;
			sh->cnum = (xd == DIR_UP) ? - 1 : 0;

			if (sh->ceol != sh->ceon) {

				sh->ceol = sh->ceon;
			}
			else {
				sh_complete(sh, xd);
			}
		}
	}
	else {
		/* Search for the next match.
		 * */
		sh_cyclic_match(sh, xd);

		/* Update the command line.
		 * */
		sh_puts_erase(sh->ceol - sh->ceon);
		sh->ceol = strlen(sh->cline);
		puts(sh->cline + sh->ceon);
	}

	sh->mprev = 0;
}

static void
sh_history(priv_sh_t *sh, int xd)
{
	int			xnum;
	char			*s;

	if (sh->mprev == 0) {

		/* Enter history mode.
		 * */
		sh->pnum = sh->tail;
		sh->mprev = 1;

		xnum = sh->tail;

		/* Save current line.
		 * */
		sh_history_put(sh, sh->cline);

		sh->tail = xnum;
	}

	xnum = sh_history_move(sh, sh->pnum, xd);

	if (xnum != sh->pnum) {

		sh->pnum = xnum;
		s = sh->cline;

		do {
			if ((*s = sh->cprev[xnum]) == 0)
				break;

			xnum = SH_HIST_INC(xnum);
			++s;
		}
		while (1);

		/* Update the command line.
		 * */
		sh_puts_erase(sh->ceol);
		sh->ceol = strlen(sh->cline);

		puts(sh->cline);
	}

	sh->mcomp = 0;
}

static void
sh_line_putc(priv_sh_t *sh, char c)
{
	if (sh->ceol < SH_CLINE_MAX - 2) {

		sh->cline[sh->ceol++] = c;
		sh->cline[sh->ceol] = 0;

		/* Echo.
		 * */
		putc(c);

		sh->mcomp = 0;
		sh->mprev = 0;
	}
}

static void
sh_line_bs(priv_sh_t *sh)
{
	if (sh->ceol > 0) {

		sh->cline[--sh->ceol] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		sh->mcomp = 0;
		sh->mprev = 0;
	}
}

static void
sh_line_null(priv_sh_t *sh)
{
	sh->cline[sh->ceol = 0] = 0;

#ifdef HW_HAVE_NETWORK_EPCAN
	if (iodef == &io_CAN) {

		/* Prompt with CAN node ID.
		 * */
		printf("(net/%i) ", net.node_ID);
	}
	else
#endif /* HW_HAVE_NETWORK_EPCAN */

	{
		/* Prompt (local).
		 * */
		puts(SH_PROMPT);
	}

	sh->mcomp = 0;
	sh->mprev = 0;
}

const char *sh_next_arg(const char *s)
{
	int			len;

	len = strlen(s);
	s += (len != 0) ? len + 1 : 0;

	return s;
}

static priv_sh_t		privsh;

LD_TASK void task_CMDSH(void *pData)
{
	priv_sh_t	*sh = &privsh;
	int		c;

	do {
		c = getc();

		if (sh->xESC == 0) {

			if (sh_byte_is_letter(c) || sh_byte_is_digit(c)
					|| strchr(SH_ALLOWED, c) != NULL) {

				sh_line_putc(sh, c);
			}
			else if (c == K_CR) {

				/* Return.
				 * */
				puts(EOL);

				sh_evaluate(sh);
				sh_line_null(sh);
			}
			else if (c == K_BS || c == K_DEL) {

				/* Backspace.
				 * */
				sh_line_bs(sh);
			}
			else if (c == K_TAB || c == '@') {

				/* Tab.
				 * */
				sh_complete(sh, DIR_UP);
			}
			else if (c == K_ETX || c == K_EOT) {

				/* Ctrl + C.
				 * */
				puts(EOL);

				sh_line_null(sh);
			}
			else if (c == K_DLE || c == '*') {

				/* Ctrl + P.
				 * */
				sh_history(sh, DIR_UP);
			}
			else if (c == K_SO || c == '!') {

				/* Ctrl + N.
				 * */
				sh_history(sh, DIR_DOWN);
			}
			else if (c == K_ESC) {

				sh->xESC = 1;
			}
		}
		else {
			switch (sh->xESC) {

				case 1:
					sh->xESC = (c == '[') ? 2 : 0;
					break;

				case 2:
					if (c == '3') {

						sh->xESC = 3;
					}
					else if (c == 'A') {

						/* Up.
						 * */
						sh_history(sh, DIR_UP);

						sh->xESC = 0;
					}
					else if (c == 'B') {

						/* Down.
						 * */
						sh_history(sh, DIR_DOWN);

						sh->xESC = 0;
					}
					else if (c == 'Z') {

						/* Shift + Tab.
						 * */
						sh_complete(sh, DIR_DOWN);

						sh->xESC = 0;
					}
					else {
						sh->xESC = 0;
					}
					break;

				case 3:
					if (c == '~') {

						/* Delete.
						 * */
						puts(EOL);
						sh_line_null(sh);

						sh->xESC = 0;
					}
					else {
						sh->xESC = 0;
					}
					break;

				default:
					sh->xESC = 0;
			}
		}
	}
	while (1);
}

#undef SH_DEF
#define SH_DEF(name)		void name(const char *s)

/* TODO */

