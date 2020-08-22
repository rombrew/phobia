#include <stddef.h>

#include "shell.h"
#include "ifcan.h"
#include "libc.h"

#define SH_CLINE_SZ			84
#define SH_HISTORY_SZ			440

#define HIST_INC(n)                 	(((n) < (SH_HISTORY_SZ - 1)) ? (n) + 1 : 0)
#define HIST_DEC(n)                 	(((n) > 0) ? (n) - 1 : SH_HISTORY_SZ - 1)

static const char
SH_PROMPT[] = "(pmc) ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Base SH data.
	 * */
	char		cLINE[SH_CLINE_SZ];
	int		cEOL, xESC;
	char		*s_arg;

	/* Completion block.
	 * */
	int		mCOMP, cEON, cNUM;

	/* History block.
	 * */
	char		cHIST[SH_HISTORY_SZ];
	int		mHIST, hHEAD, hTAIL, hNUM;
}
sh_t;

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
sh_exact_match_call(sh_t *sh)
{
	const sh_cmd_t		*cmd;
	const char		*id;

	cmd = cmLIST;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (strcmp(sh->cLINE, id) == 0) {

			/* Call the function.
			 * */
			cmd->proc(sh->s_arg);

			break;
		}

		++cmd;
	}
	while (1);
}

static void
sh_cyclic_match(sh_t *sh, int xDIR)
{
	const sh_cmd_t		*cmd;
	const char		*id;
	int			N = 0;

	cmd = cmLIST + sh->cNUM;
	sh->cLINE[sh->cEON] = 0;

	do {
		cmd += (xDIR == DIR_UP) ? 1 : - 1;

		cmd = (cmd < cmLIST) ? cmLIST_END
			: (cmd > cmLIST_END) ? cmLIST : cmd;

		id = cmd->sym;

		if (strcmpe(sh->cLINE, id) == 0) {

			/* Copy the command name.
			 * */
			strcpyn(sh->cLINE, id, (SH_CLINE_SZ - 2));

			break;
		}

		++N;

		if (N > (int) (cmLIST_END - cmLIST))
			break;
	}
	while (1);

	sh->cNUM = cmd - cmLIST;
}

static void
sh_common_match(sh_t *sh)
{
	const sh_cmd_t		*cmd;
	const char		*id, *sp;
	int			n;

	sp = NULL;
	cmd = cmLIST;

	sh->cNUM = 0;

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (strcmpe(sh->cLINE, id) == 0) {

			n = (sp != NULL) ? strcmpn(sp, id, n) : strlen(id);
			sp = id;

			sh->cNUM++;
		}

		++cmd;
	}
	while (1);

	if (sp != NULL) {

		if (n > (SH_CLINE_SZ - 2))
			n = (SH_CLINE_SZ - 2);

		strcpyn(sh->cLINE, sp, n);
		sh->cEON = n;
	}
	else
		sh->cEON = 0;
}

static int
sh_history_move(sh_t *sh, int xNUM, int xDIR)
{
	if (xDIR == DIR_UP) {

		if (xNUM != sh->hHEAD) {

			/* Get previous line.
			 * */
			xNUM = HIST_DEC(xNUM);

			do {
				xNUM = HIST_DEC(xNUM);

				if (sh->cHIST[xNUM] == 0)
					break;
			}
			while (1);

			xNUM = HIST_INC(xNUM);
		}
	}
	else {
		if (xNUM != sh->hTAIL) {

			/* Get next line.
			 * */
			do {
				xNUM = HIST_INC(xNUM);

				if (sh->cHIST[xNUM] == 0)
					break;
			}
			while (1);

			xNUM = HIST_INC(xNUM);
		}
	}

	return xNUM;
}

static void
sh_history_put(sh_t *sh, const char *s)
{
	int			xNUM, r;
	const char		*q = s;

	if (sh->hHEAD != sh->hTAIL) {

		xNUM = sh_history_move(sh, sh->hTAIL, DIR_UP);

		do {
			r = sh->cHIST[xNUM] - *q;

			if (r || !*q)
				break;

			xNUM = HIST_INC(xNUM);
			++q;
		}
		while (1);

		if (r == 0)

			/* Do not put the same line again.
			 * */
			return ;
	}

	do {
		sh->cHIST[sh->hTAIL] = *s;
		sh->hTAIL = HIST_INC(sh->hTAIL);

		if (sh->hTAIL == sh->hHEAD) {

			/* Forget old lines.
			 * */
			do {
				sh->hHEAD = HIST_INC(sh->hHEAD);

				if (sh->cHIST[sh->hHEAD] == 0)
					break;
			}
			while (1);

			sh->hHEAD = HIST_INC(sh->hHEAD);
		}

		if (*s == 0)
			break;
		else
			++s;
	}
	while (1);
}

static char *
sh_get_args(char *s)
{
	const char		*delim = " ";
	char			*s_arg, *q;
	int			n_arg, n;

	s_arg = NULL;
	n_arg = 1;

	q = s;
	n = 1;

	while (*s != 0) {

		if (strchr(delim, *s) == NULL) {

			if (n == 2) {

				s_arg = q;
			}

			*q++ = *s;

			n = 0;
		}
		else {
			if (n == 0) {

				*q++ = 0;
				n_arg++;
			}

			n = n_arg;
		}

		++s;
	}

	if (s_arg == NULL) {

		s_arg = q;
	}

	*(q + 0) = 0;
	*(q + 1) = 0;

	return s_arg;
}

static void
sh_evaluate(sh_t *sh)
{
	char			*s;

	s = sh->cLINE;

	if (*s != 0) {

		/* Put the line in history.
		 * */
		sh_history_put(sh, s);

		/* Get the command line arguments.
		 * */
		sh->s_arg = sh_get_args(s);

		/* Search for specific command to execute.
		 * */
		sh_exact_match_call(sh);
	}
}

static void
sh_complete(sh_t *sh, int xDIR)
{
	const char		space = ' ';
	char			*s;

	if (sh->mCOMP == 0) {

		s = sh->cLINE;

		/* Do not complete with trailing spaces.
		 * */
		if (strchr(s, space) != NULL)
			return ;

		/* Complete to the common substring.
		 * */
		sh_common_match(sh);
		puts(sh->cLINE + sh->cEOL);

		if (sh->cNUM == 1) {

			/* Exact match.
			 * */
			sh->cEOL = sh->cEON;

			if (sh->cEOL < (SH_CLINE_SZ - 2)) {

				/* Put trailing space since completion is done.
				 * */
				sh->cLINE[sh->cEOL++] = space;
				sh->cLINE[sh->cEOL] = 0;

				putc(space);
			}
		}
		else if (sh->cEOL <= sh->cEON) {

			/* Enter completion mode.
			 * */
			sh->mCOMP = 1;
			sh->cNUM = (xDIR == DIR_UP) ? - 1 : 0;

			if (sh->cEOL != sh->cEON)
				sh->cEOL = sh->cEON;
			else
				sh_complete(sh, xDIR);
		}
	}
	else {
		/* Search for the next match.
		 * */
		sh_cyclic_match(sh, xDIR);

		/* Update the command line.
		 * */
		sh_puts_erase(sh->cEOL - sh->cEON);
		sh->cEOL = strlen(sh->cLINE);
		puts(sh->cLINE + sh->cEON);
	}

	sh->mHIST = 0;
}

static void
sh_history(sh_t *sh, int xDIR)
{
	int			xNUM;
	char			*s;

	if (sh->mHIST == 0) {

		/* Enter history mode.
		 * */
		sh->hNUM = sh->hTAIL;
		sh->mHIST = 1;
		xNUM = sh->hTAIL;

		/* Save current line.
		 * */
		sh_history_put(sh, sh->cLINE);
		sh->hTAIL = xNUM;
	}

	if (xDIR == DIR_UP)

		xNUM = sh_history_move(sh, sh->hNUM, DIR_UP);
	else
		xNUM = sh_history_move(sh, sh->hNUM, DIR_DOWN);

	if (xNUM != sh->hNUM) {

		sh->hNUM = xNUM;
		s = sh->cLINE;

		do {
			if ((*s = sh->cHIST[xNUM]) == 0)
				break;

			xNUM = HIST_INC(xNUM);
			++s;
		}
		while (1);

		/* Update the command line.
		 * */
		sh_puts_erase(sh->cEOL);
		sh->cEOL = strlen(sh->cLINE);
		puts(sh->cLINE);
	}

	sh->mCOMP = 0;
}

static void
sh_line_putc(sh_t *sh, char c)
{
	if (sh->cEOL < (SH_CLINE_SZ - 2)) {

		sh->cLINE[sh->cEOL++] = c;
		sh->cLINE[sh->cEOL] = 0;

		if (iodef_ECHO != 0) {

			/* Echo.
			 * */
			putc(c);
		}

		sh->mCOMP = 0;
		sh->mHIST = 0;
	}
}

static void
sh_line_bs(sh_t *sh)
{
	if (sh->cEOL > 0) {

		sh->cLINE[--sh->cEOL] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		sh->mCOMP = 0;
		sh->mHIST = 0;
	}
}

static void
sh_line_null(sh_t *sh)
{
	sh->cLINE[sh->cEOL = 0] = 0;

	if (iodef_ECHO != 0) {

		if (iodef == &io_CAN) {

			/* Prompt with CAN node ID.
			 * */
			printf("(can/%i) ", can.node_ID);
		}
		else {
			/* Prompt (local).
			 * */
			puts(SH_PROMPT);
		}
	}

	sh->mCOMP = 0;
	sh->mHIST = 0;
}

const char *sh_next_arg(const char *s)
{
	int			n;

	n = strlen(s);
	s += (n != 0) ? n + 1 : 0;

	return s;
}

static sh_t			shlocal;

void task_SH(void *pData)
{
	const char	*allowed = "+-_. ";
	sh_t		*sh = &shlocal;
	int		c;

	do {
		c = getc();

		if (sh->xESC == 0) {

			if (sh_ischar(c) || sh_isdigit(c) || strchr(allowed, c) != NULL) {

				sh_line_putc(sh, c);
			}
			else if (c == K_CR) {

				if (iodef_ECHO != 0) {

					/* Return.
					 * */
					puts(EOL);
				}

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

SH_DEF(shell_keycodes)
{
	int		c;

	do {
		c = getc();

		printf("%2x" EOL, c);
	}
	while (c != K_EOT && c != 'D');
}

