/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stddef.h>

#include "shell.h"
#include "lib.h"

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

		if (!strcmp(sh->cLINE, id)) {

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

		if (!strpcmp(sh->cLINE, id)) {

			/* Copy the command name.
			 * */
			strncpy(sh->cLINE, id, (SH_CLINE_SZ - 2));

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

	do {
		id = cmd->sym;

		if (id == NULL)
			break;

		if (!strpcmp(sh->cLINE, id)) {

			n = (sp != NULL) ? strspl(sp, id, n) : strlen(id);
			sp = id;
		}

		++cmd;
	}
	while (1);

	if (sp != NULL) {

		if (n > (SH_CLINE_SZ - 2))
			n = (SH_CLINE_SZ - 2);

		strncpy(sh->cLINE, sp, n);
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
	char			*s_arg;
	int			n_arg, q;

	s_arg = NULL;
	n_arg = 0;

	q = -1;

	while (*s != 0) {

		if (strchr(delim, *s) == NULL) {

			if (q == 1)
				s_arg = s;

			q = 0;
		}
		else {
			if (q == 0)
				n_arg++;

			*s = 0;
			q = n_arg;
		}

		++s;
	}

	if (s_arg == NULL)
		s_arg = s;

	*(s + 1) = 0;

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
	char			*s;

	if (sh->mCOMP == 0) {

		s = sh->cLINE;

		while (*s) {

			/* Do not complete with trailing spaces.
			 * */
			if (*s == ' ')
				return ;

			++s;
		}

		/* Complete to the common substring.
		 * */
		sh_common_match(sh);
		puts(sh->cLINE + sh->cEOL);

		if (sh->cEOL <= sh->cEON) {

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

		/* Echo.
		 * */
		iodef->putc(c);

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

	/* Prompt.
	 * */
	puts(SH_PROMPT);

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

void taskSH(void *pData)
{
	const char	*allowed = "+-_.[] ";
	sh_t		*sh = &shlocal;
	int		c;

	do {
		c = iodef->getc();

		if (sh->xESC == 0) {

			if (sh_ischar(c) || sh_isdigit(c) || strchr(allowed, c) != NULL) {

				sh_line_putc(sh, c);
			}
			else if (c == '\r') {

				/* Echo.
				 * */
				puts(EOL);

				sh_evaluate(sh);
				sh_line_null(sh);
			}
			else if (c == '\b') {

				sh_line_bs(sh);
			}
			else if (c == '\t') {

				sh_complete(sh, DIR_UP);
			}
			else if (c == K_ETX || c == K_EOT) {

				puts(EOL);
				sh_line_null(sh);
			}
			else if (c == K_DLE) {

				sh_history(sh, DIR_UP);
			}
			else if (c == K_SO) {

				sh_history(sh, DIR_DOWN);
			}
			else if (c == K_ESC) {

				sh->xESC = 1;
			}
		}
		else {
			if (sh->xESC == 1) {

				sh->xESC = (c == '[') ? 2 : 0;
			}
			else {
				if (c == 'A') {

					sh_history(sh, DIR_UP);
				}
				else if (c == 'B') {

					sh_history(sh, DIR_DOWN);
				}
				else if (c == 'Z') {

					sh_complete(sh, DIR_DOWN);
				}

				sh->xESC = 0;
			}
		}
	}
	while (1);
}

