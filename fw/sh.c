/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

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

#include "sh.h"
#include "task.h"
#include "lib.h"

#define SH_RXBUF_SZ			40
#define SH_TXBUF_SZ			80
#define SH_CLINE_SZ			80

#define FIFO_INC(I, SZ)			(((I) < ((SZ) - 1)) ? (I) + 1 : 0)

static const char SH_PROMPT[] = "# ";
static const char SH_BACKSPACE[] = "\b \b";

typedef struct {

	char		rBuf[SH_RXBUF_SZ];
	int		rR, rT;

	char		tBuf[SH_TXBUF_SZ];
	int		tR, tT;

	char		cLine[SH_CLINE_SZ];
	char		cHist[SH_CLINE_SZ];
	int		nEOL;

	char		*pARG;
	int		cMD, cEON, cIT;
}
shTASK_t;

static shTASK_t		sh;
extern shCMD_t		cmList[];

int shRecv()
{
	int		xC;

	if (sh.rR != sh.rT) {

		xC = sh.rBuf[sh.rR];
		sh.rR = FIFO_INC(sh.rR, SH_RXBUF_SZ);
	}
	else
		xC = -1;

	return xC;
}

void shSend(int xC)
{
	int	tT;

	tT = FIFO_INC(sh.tT, SH_TXBUF_SZ);

	while (sh.tR == tT)
		taskYield();

	sh.tBuf[sh.tT] = (char) xC;
	sh.tT = tT;

	td.xOUT = 1;
}

int shExRecv()
{
	int		xC;

	if (sh.tR != sh.tT) {

		xC = sh.tBuf[sh.tR];
		sh.tR = FIFO_INC(sh.tR, SH_TXBUF_SZ);
	}
	else
		xC = -1;

	return xC;
}

void shExSend(int xC)
{
	sh.rBuf[sh.rT] = (char) xC;
	sh.rT = FIFO_INC(sh.rT, SH_RXBUF_SZ);

	td.xSH = 1;
}

static inline char
isDigit(char xC)
{
	return (xC >= '0') && (xC <= '9');
}

static inline char
isChar(char xC)
{
	return ((xC >= 'a') && (xC <= 'z'))
		|| ((xC >= 'A') && (xC <= 'Z'));
}

static void
shErase(int N)
{
	while (N) {

		puts(SH_BACKSPACE);
		--N;
	}
}

static void
shExactMatch()
{
	const shCMD_t		*pCMD;
	const char		*iD;

	pCMD = cmList;

	do {
		iD = pCMD->iD;

		if (iD == NULL)
			break;

		if (!strcmp(sh.cLine, iD)) {

			/* Call the function.
			 * */
			pCMD->pF(sh.pARG);

			break;
		}

		++pCMD;
	}
	while (1);
}

static void
shCyclicMatch()
{
	const shCMD_t		*pCMD;
	const char		*iD;
	int			N = 0;

	pCMD = cmList + sh.cIT;
	sh.cLine[sh.cEON] = 0;

	do {
		iD = pCMD->iD;

		if (iD == NULL) {

			pCMD = cmList;

			if (N++)
				break;
			else
				continue;
		}

		if (!strpcmp(sh.cLine, iD)) {

			/* Copy command name.
			 * */
			strcpy(sh.cLine, iD);

			break;
		}

		++pCMD;
	}
	while (1);

	sh.cIT = (pCMD - cmList) + 1;
}

static void
shEval()
{
	char			*pC;

	pC = sh.cLine;

	/* Parse the command line.
	 * */
	while (*pC && *pC != ' ') ++pC;
	while (*pC && *pC == ' ') *pC++ = 0;
	sh.pARG = pC;

	/* Search for specific command to execute.
	 * */
	shExactMatch();
}

static void
shComplete()
{
	char			*pC;
	int			N;

	if (!sh.cMD) {

		pC = sh.cLine;

		while (*pC) {

			/* Do not complete with trailing spaces.
			 * */
			if (*pC == ' ')
				return ;

			++pC;
		}

		N = sh.nEOL;

		/* Enter the iteration mode.
		 * */
		sh.cMD = 1;
		sh.cEON = N;
		sh.cIT = 0;
	}
	else {
		N = sh.cEON;
	}

	/* Search for the next match.
	 * */
	shCyclicMatch();

	/* Update command line.
	 * */
	shErase(sh.nEOL - N);
	sh.nEOL = strlen(sh.cLine);
	puts(sh.cLine + N);
}

static void
shLinePutC(char xC)
{
	if (sh.nEOL < (SH_CLINE_SZ - 1)) {

		sh.cLine[sh.nEOL++] = xC;
		sh.cLine[sh.nEOL] = 0;

		/* Echo.
		 * */
		putc(xC);

		sh.cMD = 0;
	}
}

static void
shLineBS()
{
	if (sh.nEOL > 0) {

		sh.cLine[--sh.nEOL] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		sh.cMD = 0;
	}
}

static void
shLineNULL()
{
	sh.cLine[sh.nEOL = 0] = 0;

	/* Prompt.
	 * */
	puts(SH_PROMPT);

	sh.cMD = 0;
}

void shTask()
{
	int		xC;

	while ((xC = shRecv()) >= 0) {

		if (isChar(xC) || isDigit(xC)
				|| (xC == ' ')
				|| (xC == '_')
				|| (xC == '.')
				|| (xC == '-')
				|| (xC == '%')) {

			shLinePutC(xC);
		}
		else if (xC == '\r') {

			/* Echo.
			 * */
			puts(EOL);

			shEval();
			shLineNULL();
		}
		else if (xC == '\b') {

			shLineBS();
		}
		else if (xC == '\t') {

			shComplete();
		}
	}
}

