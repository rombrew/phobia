/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#include "freertos/FreeRTOS.h"

#include "sh.h"
#include "lib.h"

#define SH_CLINE_SZ			80
#define SH_HISTORY_SZ			440

#define FIFO_INC(I, SZ)                 (((I) < ((SZ) - 1)) ? (I) + 1 : 0)
#define FIFO_DEC(I, SZ)                 (((I) > 0) ? (I) - 1 : (SZ) - 1)

static const char
SH_PROMPT[] = "# ",
SH_BACKSPACE[] = "\b \b";

enum {
	DIR_UP,
	DIR_DOWN
};

typedef struct {

	/* Base SH data.
	 * */
	char		cLine[SH_CLINE_SZ];
	int		nEOL, xESC;
	char		*pARG;

	/* Completion block.
	 * */
	int		cMD, cEON, cIT;

	/* History block.
	 * */
	char		cHist[SH_HISTORY_SZ];
	int		hMD, hHEAD, hTAIL, hIT;
}
shTASK_t;

static shTASK_t		*pSH;
extern const shCMD_t	cmList[];

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
	while (N > 0) {

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

		if (!strcmp(pSH->cLine, iD)) {

			/* Call the function.
			 * */
			pCMD->pF(pSH->pARG);

			break;
		}

		++pCMD;
	}
	while (1);
}

static void
shCyclicMatch(int xDIR)
{
	const shCMD_t		*pCMD;
	const char		*iD;
	int			N = 0;

	pCMD = cmList + pSH->cIT;
	pSH->cLine[pSH->cEON] = 0;

	pCMD += (xDIR == DIR_UP) ? 1 : - 1;

	do {
		if (pCMD < cmList) {

			/* Jump to the end.
			 * */
			while (pCMD->iD != NULL) ++pCMD;
			--pCMD;

			if (N++)
				break;
		}

		iD = pCMD->iD;

		if (iD == NULL) {

			/* Jump to the begin.
			 * */
			pCMD = cmList;

			if (N++)
				break;
		}

		if (!strpcmp(pSH->cLine, iD)) {

			/* Copy the command name.
			 * */
			strcpy(pSH->cLine, iD);

			break;
		}

		if (xDIR == DIR_UP)
			++pCMD;
		else
			--pCMD;
	}
	while (1);

	pSH->cIT = pCMD - cmList;
}

static void
shCommonMatch()
{
	const shCMD_t		*pCMD;
	const char		*iD, *iLast;
	int			N;

	iLast = NULL;
	pCMD = cmList;

	do {
		iD = pCMD->iD;

		if (iD == NULL)
			break;

		if (!strpcmp(pSH->cLine, iD)) {

			if (iLast != NULL)
				N = strspl(iLast, iD, N);
			else
				N = strlen(iD);

			iLast = iD;
		}

		++pCMD;
	}
	while (1);

	if (iLast != NULL) {

		strncpy(pSH->cLine, iLast, N);
		pSH->cEON = N;
	}
	else
		pSH->cEON = 0;
}

static int
shHistoryMove(int xIT, int xDIR)
{
	if (xDIR == DIR_UP) {

		if (xIT != pSH->hHEAD) {

			/* Get previous line.
			 * */
			xIT = FIFO_DEC(xIT, SH_HISTORY_SZ);

			do {
				xIT = FIFO_DEC(xIT, SH_HISTORY_SZ);

				if (pSH->cHist[xIT] == 0)
					break;
			}
			while (1);

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
	}
	else {
		if (xIT != pSH->hTAIL) {

			/* Get next line.
			 * */
			do {
				xIT = FIFO_INC(xIT, SH_HISTORY_SZ);

				if (pSH->cHist[xIT] == 0)
					break;
			}
			while (1);

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
	}

	return xIT;
}

static void
shHistoryPut(const char *xS)
{
	int			xIT, R;
	const char		*xP = xS;

	if (pSH->hHEAD != pSH->hTAIL) {

		xIT = shHistoryMove(pSH->hTAIL, DIR_UP);

		do {
			R = pSH->cHist[xIT] - *xP;

			if (R || !*xP)
				break;

			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
			xP++;
		}
		while (1);

		if (R == 0)

			/* Do not put the same line again.
			 * */
			return ;
	}

	do {
		pSH->cHist[pSH->hTAIL] = *xS;
		pSH->hTAIL = FIFO_INC(pSH->hTAIL, SH_HISTORY_SZ);

		if (pSH->hTAIL == pSH->hHEAD) {

			/* Forget old lines.
			 * */
			do {
				pSH->hHEAD = FIFO_INC(pSH->hHEAD, SH_HISTORY_SZ);

				if (pSH->cHist[pSH->hHEAD] == 0)
					break;
			}
			while (1);

			pSH->hHEAD = FIFO_INC(pSH->hHEAD, SH_HISTORY_SZ);
		}

		if (*xS == 0)
			break;
		else
			xS++;
	}
	while (1);
}

static void
shEval()
{
	char			*pC;

	pC = pSH->cLine;

	if (*pC != 0) {

		/* Put the line in history.
		 * */
		shHistoryPut(pC);

		/* Parse the command line.
		 * */
		while (*pC && *pC != ' ') ++pC;
		while (*pC && *pC == ' ') *pC++ = 0;
		pSH->pARG = pC;

		/* Search for specific command to execute.
		 * */
		shExactMatch();
	}
}

static void
shComplete(int xDIR)
{
	char			*pC;

	if (pSH->cMD == 0) {

		pC = pSH->cLine;

		while (*pC) {

			/* Do not complete with trailing spaces.
			 * */
			if (*pC == ' ')
				return ;

			++pC;
		}

		/* Complete to the common substring.
		 * */
		shCommonMatch();
		puts(pSH->cLine + pSH->nEOL);

		if (pSH->nEOL <= pSH->cEON) {

			/* Enter completion mode.
			 * */
			pSH->cMD = 1;
			pSH->cIT = (xDIR == DIR_UP) ? - 1 : 0;

			if (pSH->nEOL != pSH->cEON)
				pSH->nEOL = pSH->cEON;
			else
				shComplete(xDIR);
		}
	}
	else {
		/* Search for the next match.
		 * */
		shCyclicMatch(xDIR);

		/* Update the command line.
		 * */
		shErase(pSH->nEOL - pSH->cEON);
		pSH->nEOL = strlen(pSH->cLine);
		puts(pSH->cLine + pSH->cEON);
	}

	pSH->hMD = 0;
}

static void
shHistory(int xDIR)
{
	int			xIT;
	char			*xD;

	if (pSH->hMD == 0) {

		/* Enter history mode.
		 * */
		pSH->hIT = pSH->hTAIL;
		pSH->hMD = 1;
		xIT = pSH->hTAIL;

		/* Save current line.
		 * */
		shHistoryPut(pSH->cLine);
		pSH->hTAIL = xIT;
	}

	if (xDIR == DIR_UP)

		xIT = shHistoryMove(pSH->hIT, DIR_UP);
	else
		xIT = shHistoryMove(pSH->hIT, DIR_DOWN);

	if (xIT != pSH->hIT) {

		pSH->hIT = xIT;
		xD = pSH->cLine;

		do {
			if (!(*xD = pSH->cHist[xIT]))
				break;

			xD++;
			xIT = FIFO_INC(xIT, SH_HISTORY_SZ);
		}
		while (1);

		/* Update the command line.
		 * */
		shErase(pSH->nEOL);
		pSH->nEOL = strlen(pSH->cLine);
		puts(pSH->cLine);
	}

	pSH->cMD = 0;
}

static void
shLinePutC(char xC)
{
	if (pSH->nEOL < (SH_CLINE_SZ - 1)) {

		pSH->cLine[pSH->nEOL++] = xC;
		pSH->cLine[pSH->nEOL] = 0;

		/* Echo.
		 * */
		iodef->putc(xC);

		pSH->cMD = 0;
		pSH->hMD = 0;
	}
}

static void
shLineBS()
{
	if (pSH->nEOL > 0) {

		pSH->cLine[--pSH->nEOL] = 0;

		/* Echo.
		 * */
		puts(SH_BACKSPACE);

		pSH->cMD = 0;
		pSH->hMD = 0;
	}
}

static void
shLineNULL()
{
	pSH->cLine[pSH->nEOL = 0] = 0;

	/* Prompt.
	 * */
	puts(SH_PROMPT);

	pSH->cMD = 0;
	pSH->hMD = 0;
}

void taskSH(void *pvParameters)
{
	int		xC;

	pSH = pvPortMalloc(sizeof(shTASK_t));
	memzero(pSH, sizeof(shTASK_t));

	do {
		xC = iodef->getc();

		if (pSH->xESC == 0) {

			if (isChar(xC) || isDigit(xC)
					|| (xC == ' ')
					|| (xC == '_')
					|| (xC == '.')
					|| (xC == '-')
					|| (xC == '+')
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

				shComplete(DIR_UP);
			}
			else if (xC == K_ETX || xC == K_EOT) {

				puts(EOL);
				shLineNULL();
			}
			else if (xC == K_DLE) {

				shHistory(DIR_UP);
			}
			else if (xC == K_SO) {

				shHistory(DIR_DOWN);
			}
			else if (xC == K_ESC) {

				pSH->xESC = 1;
			}
		}
		else {
			if (pSH->xESC == 1) {

				pSH->xESC = (xC == '[') ? 2 : 0;
			}
			else {
				if (xC == 'A') {

					shHistory(DIR_UP);
				}
				else if (xC == 'B') {

					shHistory(DIR_DOWN);
				}
				else if (xC == 'Z') {

					shComplete(DIR_DOWN);
				}

				pSH->xESC = 0;
			}
		}
	}
	while (1);
}

#include "sh.gen_h"

const shCMD_t		cmList[] = {

#include "sh.gen_list"

	{NULL, NULL}
};

