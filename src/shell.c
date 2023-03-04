#include <stddef.h>

#include "hal/hal.h"

#include "shell.h"
#ifdef HW_HAVE_NETWORK_EPCAN
#include "epcan.h"
#endif /* HW_HAVE_NETWORK_EPCAN */
#include "libc.h"

#define SH_CLINE_SZ			84
#define SH_HISTORY_SZ			440

#define SH_HIST_INC(n)                 	(((n) < (SH_HISTORY_SZ - 1)) ? (n) + 1 : 0)
#define SH_HIST_DEC(n)                 	(((n) > 0) ? (n) - 1 : SH_HISTORY_SZ - 1)

static const char
SH_ALLOWED[] = "+-_. ",
SH_PROMPT[] = "(pmc) ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Base SH data.
	 * */
	char		cline[SH_CLINE_SZ];
	int		ceol, xesc;
	char		*parg;

	/* Completion block.
	 * */
	int		mcomp, ceon, cnum;

	/* History block.
	 * */
	char		chist[SH_HISTORY_SZ];
	int		mhist, hhead, htail, hnum;
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

static char
sh_isdigit(char c)
{
	return (c >= '0') && (c <= '9');
}

static char
sh_ischar(char c)
{
	return ((c >= 'a') && (c <= 'z'))
		|| ((c >= 'A') && (c <= 'Z'));
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
			cmd->proc(sh->parg);

			break;
		}

		++cmd;
	}
	while (1);
}

static void
sh_cyclic_match(priv_sh_t *sh, int xdir)
{
	const sh_cmd_t		*cmd;
	const char		*id;
	int			N = 0;

	cmd = cmLIST + sh->cnum;
	sh->cline[sh->ceon] = 0;

	do {
		cmd += (xdir == DIR_UP) ? 1 : - 1;

		cmd = (cmd < cmLIST) ? cmLIST_END
			: (cmd > cmLIST_END) ? cmLIST : cmd;

		id = cmd->sym;

		if (strcmpe(sh->cline, id) == 0) {

			/* Copy the command name.
			 * */
			strcpyn(sh->cline, id, SH_CLINE_SZ - 2);

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
	int			n;

	sp = NULL;
	cmd = cmLIST;

	sh->cnum = 0;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (strcmpe(sh->cline, id) == 0) {

			n = (sp != NULL) ? strcmpn(sp, id, n) : strlen(id);
			sp = id;

			sh->cnum++;
		}

		++cmd;
	}
	while (1);

	if (sp != NULL) {

		n = (n > SH_CLINE_SZ - 2) ? SH_CLINE_SZ - 2 : n;

		strcpyn(sh->cline, sp, n);
		sh->ceon = n;
	}
	else {
		sh->ceon = 0;
	}
}

static int
sh_history_move(priv_sh_t *sh, int xnum, int xdir)
{
	if (xdir == DIR_UP) {

		if (xnum != sh->hhead) {

			/* Get previous line.
			 * */
			xnum = SH_HIST_DEC(xnum);

			do {
				xnum = SH_HIST_DEC(xnum);

				if (sh->chist[xnum] == 0)
					break;
			}
			while (1);

			xnum = SH_HIST_INC(xnum);
		}
	}
	else {
		if (xnum != sh->htail) {

			/* Get next line.
			 * */
			do {
				xnum = SH_HIST_INC(xnum);

				if (sh->chist[xnum] == 0)
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

	if (sh->hhead != sh->htail) {

		xnum = sh_history_move(sh, sh->htail, DIR_UP);

		do {
			r = sh->chist[xnum] - *q;

			if (r || !*q)
				break;

			xnum = SH_HIST_INC(xnum);
			++q;
		}
		while (1);

		if (r == 0)

			/* Do not put the same line again.
			 * */
			return ;
	}

	do {
		sh->chist[sh->htail] = *s;
		sh->htail = SH_HIST_INC(sh->htail);

		if (sh->htail == sh->hhead) {

			/* Forget old lines.
			 * */
			do {
				sh->hhead = SH_HIST_INC(sh->hhead);

				if (sh->chist[sh->hhead] == 0)
					break;
			}
			while (1);

			sh->hhead = SH_HIST_INC(sh->hhead);
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
		sh->parg = sh_markup_args(s);

		/* Search for specific command to execute.
		 * */
		sh_exact_match_call(sh);
	}
}

static void
sh_complete(priv_sh_t *sh, int xdir)
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

			if (sh->ceol < SH_CLINE_SZ - 2) {

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
			sh->cnum = (xdir == DIR_UP) ? - 1 : 0;

			if (sh->ceol != sh->ceon)
				sh->ceol = sh->ceon;
			else
				sh_complete(sh, xdir);
		}
	}
	else {
		/* Search for the next match.
		 * */
		sh_cyclic_match(sh, xdir);

		/* Update the command line.
		 * */
		sh_puts_erase(sh->ceol - sh->ceon);
		sh->ceol = strlen(sh->cline);
		puts(sh->cline + sh->ceon);
	}

	sh->mhist = 0;
}

static void
sh_history(priv_sh_t *sh, int xdir)
{
	int			xnum;
	char			*s;

	if (sh->mhist == 0) {

		/* Enter history mode.
		 * */
		sh->hnum = sh->htail;
		sh->mhist = 1;
		xnum = sh->htail;

		/* Save current line.
		 * */
		sh_history_put(sh, sh->cline);
		sh->htail = xnum;
	}

	if (xdir == DIR_UP) {

		xnum = sh_history_move(sh, sh->hnum, DIR_UP);
	}
	else {
		xnum = sh_history_move(sh, sh->hnum, DIR_DOWN);
	}

	if (xnum != sh->hnum) {

		sh->hnum = xnum;
		s = sh->cline;

		do {
			if ((*s = sh->chist[xnum]) == 0)
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
	if (sh->ceol < SH_CLINE_SZ - 2) {

		sh->cline[sh->ceol++] = c;
		sh->cline[sh->ceol] = 0;

		/* Echo.
		 * */
		putc(c);

		sh->mcomp = 0;
		sh->mhist = 0;
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
		sh->mhist = 0;
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
	sh->mhist = 0;
}

const char *sh_next_arg(const char *s)
{
	int			n;

	n = strlen(s);
	s += (n != 0) ? n + 1 : 0;

	return s;
}

static priv_sh_t		privsh;

void task_SH(void *pData)
{
	priv_sh_t	*sh = &privsh;
	int		c;

	do {
		c = getc();

		if (sh->xesc == 0) {

			if (sh_ischar(c) || sh_isdigit(c) || strchr(SH_ALLOWED, c) != NULL) {

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

				sh->xesc = 1;
			}
		}
		else {
			switch (sh->xesc) {

				case 1:
					sh->xesc = (c == '[') ? 2 : 0;
					break;

				case 2:
					if (c == '3') {

						sh->xesc = 3;
					}
					else if (c == 'A') {

						/* Up.
						 * */
						sh_history(sh, DIR_UP);

						sh->xesc = 0;
					}
					else if (c == 'B') {

						/* Down.
						 * */
						sh_history(sh, DIR_DOWN);

						sh->xesc = 0;
					}
					else if (c == 'Z') {

						/* Shift + Tab.
						 * */
						sh_complete(sh, DIR_DOWN);

						sh->xesc = 0;
					}
					else {
						sh->xesc = 0;
					}
					break;

				case 3:
					if (c == '~') {

						/* Delete.
						 * */
						puts(EOL);
						sh_line_null(sh);

						sh->xesc = 0;
					}
					else {
						sh->xesc = 0;
					}
					break;

				default:
					sh->xesc = 0;
			}
		}
	}
	while (1);
}

#undef SH_DEF
#define SH_DEF(name)		void name(const char *s)

SH_DEF(shell_keycodes)
{
	int		c;

	do {
		c = getc();

		printf("%2x" EOL, c);
	}
	while (c != K_EOT && c != 'D');
}

