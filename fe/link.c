#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <SDL2/SDL.h>

#include "link.h"
#include "serial.h"

#define LINK_EOL			"\r\n"
#define LINK_SPACE			" \t"
#define LINK_EXTRA			"])"

struct link_priv {

	struct serial_fd	*fd;

	int			reg_push_ID;

	char			lbuf[240];

	char			hwinfo_revision[32];
	char			hwinfo_build[32];
	char			hwinfo_crc32[32];

	int			flash_page;

	int			hwinfo_N;
	int			flash_N;
	int			flash_info_N;

	char			mb[81920];
	char			*mbflow;
};

const char *lk_stoi(int *x, const char *s)
{
	int		n, k, i;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	k = 0;
	i = 0;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		k++;

		if (k > 9) return NULL;
	}

	if (k == 0) return NULL;

	if (*s == 0 || strchr(LINK_SPACE LINK_EXTRA LINK_EOL, *s) != NULL) {

		*x = i;
	}
	else return NULL;

	return s;
}

const char *lk_stod(double *x, const char *s)
{
	int		n, k, v, e;
	double		f = 0.;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	k = 0;
	v = 0;
	f = 0.;

	while (*s >= '0' && *s <= '9') {

		f = 10. * f + (*s++ - '0') * n;
		k++;
	}

	if (*s == '.') {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10. * f + (*s++ - '0') * n;
			k++; v--;
		}
	}

	if (k == 0) return NULL;

	if (*s == 'n') { v += - 9; s++; }
	else if (*s == 'u') { v += - 6; s++; }
	else if (*s == 'm') { v += - 3; s++; }
	else if (*s == 'K') { v += 3; s++; }
	else if (*s == 'M') { v += 6; s++; }
	else if (*s == 'G') { v += 9; s++; }
	else if (*s == 'e' || *s == 'E') {

		s = lk_stoi(&e, s + 1);

		if (s != NULL) { v += e; }
		else return NULL;
	}

	if (*s == 0 || strchr(LINK_SPACE LINK_EXTRA LINK_EOL, *s) != NULL) {

		while (v < 0) { f /= 10.; v++; }
		while (v > 0) { f *= 10.; v--; }

		*x = f;
	}
	else return NULL;

	return s;
}

static const char *
lk_space(char **sp)
{
	char		*s = *sp;

	while (*s != 0 && strchr(LINK_SPACE, *s) != 0) { ++s; }

	*sp = s;

	return s;
}

static const char *
lk_token(char **sp)
{
	char		*s = *sp;
	const char	*r;

	while (*s != 0 && strchr(LINK_SPACE, *s) != 0) { ++s; }

	r = s;

	if (r[0] == '"') {

		++r;
		++s;

		while (*s != 0 && *s != '"') { ++s; }
	}
	else if (r[0] == '(') {

		++r;
		++s;

		while (*s != 0 && *s != ')') { ++s; }
	}
	else {
		while (*s != 0 && strchr(LINK_SPACE, *s) == 0) { ++s; }
	}

	if (*s != 0) {

		*s = 0;
		++s;
	}

	*sp = s;

	return r;
}

static void
link_fetch_network(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;
	int			n;

	if (strstr(lbuf, "(pmc)") == lbuf) {

		sprintf(lp->network, "%.32s", "local");
	}

	if (strstr(lbuf, "(net/") == lbuf) {

		if (lk_stoi(&n, lbuf + 5) != NULL) {

			sprintf(lp->network, "remote/%i", n);
		}
		else {
			lp->network[0] = 0;
		}
	}
}

static void
link_reg_postproc(struct link_pmc *lp, struct link_reg *reg)
{
	struct link_priv	*priv = lp->priv;
	double			dval;

	if (lk_stoi(&reg->lval, reg->val) != NULL) {

		reg->mode |= LINK_REG_TYPE_INT;
	}

	if (lk_stod(&dval, reg->val) != NULL) {

		reg->mode |= LINK_REG_TYPE_FLOAT;
		reg->fval = (float) dval;

		if (reg->started != 0) {

			reg->fmin = (reg->fval < reg->fmin) ? reg->fval : reg->fmin;
			reg->fmax = (reg->fval > reg->fmax) ? reg->fval : reg->fmax;
		}
		else {
			reg->fmin = reg->fval;
			reg->fmax = reg->fval;

			reg->started = 1;
		}
	}

	if (		(reg->mode & 7U) == LINK_REG_CONFIG
			&& strstr(reg->um, "_") != NULL
			&& (reg->mode & LINK_REG_TYPE_INT) != 0
			&& reg->lval >= 0 && reg->lval < LINK_COMBO_MAX
			&& reg->combo[reg->lval] == NULL) {

		if ((int) (priv->mbflow - priv->mb) < sizeof(priv->mb) - 90U) {

			sprintf(priv->mbflow, "%.77s", reg->um);

			reg->combo[reg->lval] = priv->mbflow;
			priv->mbflow += strlen(priv->mbflow) + 1;
		}

		reg->lmax_combo = (reg->lval > reg->lmax_combo)
			? reg->lval : reg->lmax_combo;
	}
}

static void
link_fetch_reg(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			ldup[sizeof(priv->lbuf)], *sp = ldup;
	const char		*tok, *sym;
	int			reg_mode, reg_ID, n, eol;

	strcpy(ldup, priv->lbuf);

	reg_mode = 0;
	reg_ID = -1;

	while (*sp != 0) {

		if (*sp == '[') {

			if ((int) (sp - ldup) > 5)
				break;

			tok = lk_token(&sp);
			eol = strlen(tok) - 1;

			if (tok[eol] == ']') {

				if (lk_stoi(&n, tok + 1) != NULL) {

					reg_ID = n;
					break;
				}
			}
		}
		else if (*sp == 'C') {

			reg_mode |= LINK_REG_CONFIG;
		}
		else if (*sp == 'R') {

			reg_mode |= LINK_REG_READ_ONLY;
		}
		else if (*sp == 'L') {

			reg_mode |= LINK_REG_LINKED;
		}

		sp++;
	}

	if (reg_ID >= 0 && reg_ID < LINK_REGS_MAX) {

		struct link_reg		*reg = lp->reg + reg_ID;

		reg->mode = reg_mode;

		sym = lk_token(&sp);
		tok = lk_token(&sp);

		if (strcmp(tok, "=") == 0) {

			char		vbuf[80];

			sprintf(vbuf, "%.77s", lk_space(&sp));

			sprintf(reg->sym, "%.77s", sym);
			sprintf(reg->val, "%.77s", lk_token(&sp));
			sprintf(reg->um,  "%.77s", lk_token(&sp));

			tok = lk_space(&sp);

			if (tok[0] == 0) {

				link_reg_postproc(lp, reg);
			}
			else {
				sprintf(reg->val, "%.77s", vbuf);

				reg->um[0] = 0;
			}

			reg->fetched = lp->clock;
			reg->queued = 0;
		}
	}
}

static void
link_fetch_hwinfo(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			ldup[sizeof(priv->lbuf)], *sp = ldup;
	const char		*tok;

	strcpy(ldup, priv->lbuf);

	tok = lk_token(&sp);

	if (strcmp(tok, "HW_revision") == 0) {

		sprintf(priv->hwinfo_revision, "%.16s", lk_token(&sp));
	}
	else if (strcmp(tok, "FW_build") == 0) {

		sprintf(priv->hwinfo_build, "%.16s", lk_token(&sp));
	}
	else if (strcmp(tok, "FW_crc32") == 0) {

		sprintf(priv->hwinfo_crc32, "%.9s %.12s", lk_token(&sp), lk_token(&sp));

		sprintf(lp->hwinfo, "%.16s %.16s %.22s", priv->hwinfo_revision,
				priv->hwinfo_build, priv->hwinfo_crc32);
	}
}

static void
link_fetch_flash(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;

	if (strstr(lbuf, "Flash ...") == lbuf) {

		if (strstr(lbuf + 9, "Done") != NULL) {

			lp->flash_errno = 1;
		}
		else {
			lp->flash_errno = 2;
		}
	}
	else if (strstr(lbuf, "Unable when PM is running") == lbuf) {

		lp->flash_errno = 3;
	}
}

static void
link_fetch_flash_info(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;
	char			vbuf[40], *rsym;
	const char		*s;
	int			n, sm;

	s = lbuf;
	rsym = vbuf;

	n = 0;
	sm = 0;

	while (*s != 0) {

		if (sm == 0) {

			if (*s != ' ') {

				sm = -1;
				break;
			}

			sm = 1;
		}
		else {
			if (		   *s != 'x'
					&& *s != 'a'
					&& *s != '.') {

				sm = -1;
				break;
			}
			else {
				*rsym++ = *s;
				++n;

				if (n > 32) {

					sm = -1;
					break;
				}
			}

			sm = 0;
		}

		++s;
	}

	*rsym = 0;

	if (n > 7 && sm != -1) {

		if (priv->flash_page < 8) {

			rsym = lp->flash_map[priv->flash_page++];

			sprintf(rsym, "%.19s", vbuf);
		}
	}
}

void link_open(struct link_pmc *lp, const char *devname, int baudrate)
{
	struct link_priv	*priv;

	if (lp->linked != 0)
		return ;

	sprintf(lp->devname, "%.77s", devname);

	lp->baudrate = baudrate;

	if (lp->priv != NULL) {

		memset(lp->priv, 0, sizeof(struct link_priv));
	}
	else {
		lp->priv = calloc(1, sizeof(struct link_priv));
	}

	priv = lp->priv;
	priv->mbflow = priv->mb;

	priv->fd = serial_open(devname, baudrate, SERIAL_DEFAULT);

	if (priv->fd == NULL)
		return ;

	serial_async_fputs(priv->fd, LINK_EOL);

	sprintf(priv->lbuf, "rtos_version" LINK_EOL);
	serial_async_fputs(priv->fd, priv->lbuf);

	lp->linked = 1;
	lp->locked = lp->clock + 1000;

	sprintf(priv->lbuf, "reg" LINK_EOL);
	serial_async_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "flash_info" LINK_EOL);
	serial_async_fputs(priv->fd, priv->lbuf);
}

void link_close(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return ;

	if (priv != NULL) {

		serial_close(priv->fd);

		memset(priv, 0, sizeof(struct link_priv));
	}

	memset(lp, 0, sizeof(struct link_pmc));

	lp->priv = priv;
}

int link_fetch(struct link_pmc *lp, int clock)
{
	struct link_priv	*priv = lp->priv;
	int			N = 0;

	lp->clock = clock;

	if (lp->linked == 0)
		return 0;

	while (serial_async_fgets(priv->fd, priv->lbuf, sizeof(priv->lbuf)) == SERIAL_OK) {

		link_fetch_network(lp);
		link_fetch_reg(lp);

		if (strstr(priv->lbuf, "rtos_version") != NULL) {

			priv->hwinfo_N = 5;
		}
		else if (priv->hwinfo_N > 0) {

			link_fetch_hwinfo(lp);

			priv->hwinfo_N--;
		}

		if (strstr(priv->lbuf, "flash_prog") != NULL) {

			priv->flash_N = 2;
		}
		else if (priv->flash_N > 0) {

			link_fetch_flash(lp);

			priv->flash_N--;
		}

		if (strstr(priv->lbuf, "flash_info") != NULL) {

			priv->flash_page = 0;
			priv->flash_info_N = 10;
		}
		else if (priv->flash_info_N > 0) {

			link_fetch_flash_info(lp);

			priv->flash_info_N--;
		}

		N++;
	}

	lp->fetched_N += N;

	return N;
}

void link_push(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	struct link_reg		*reg;
	int			reg_ID;

	if (lp->linked == 0)
		return ;

	if (lp->locked > lp->clock)
		return ;

	reg_ID = priv->reg_push_ID;

	do {
		reg_ID++;

		if (reg_ID >= LINK_REGS_MAX)
			reg_ID = 0;

		if (lp->reg[reg_ID].sym[0] == 0)
			reg_ID = 0;

		reg = lp->reg + reg_ID;

		if (reg->queued == 0) {

			if (		(reg->update != 0
					&& reg->fetched + reg->update < reg->shown)
					|| reg->fetched == 0) {

				sprintf(priv->lbuf, "reg %i" LINK_EOL, reg_ID);

				if (serial_async_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock + 20;
				}
				break;
			}

			if (reg->modified > reg->fetched) {

				sprintf(priv->lbuf, "reg %i %.77s" LINK_EOL,
						reg_ID, reg->val);

				if (serial_async_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock + 20;
				}
				break;
			}
		}
		else {
			if (reg->queued + 500 < lp->clock)
				reg->queued = 0;
		}

		if (reg_ID == priv->reg_push_ID)
			break;
	}
	while (1);

	priv->reg_push_ID = reg_ID;
}

int link_command(struct link_pmc *lp, const char *command)
{
	struct link_priv	*priv = lp->priv;
	int			pushed = 0;

	if (lp->linked == 0)
		return 0;

	if (lp->locked > lp->clock + 50)
		return 0;

	sprintf(priv->lbuf, "%.190s" LINK_EOL, command);

	if (serial_async_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

		pushed = 1;

		lp->locked = lp->clock + 100;
	}

	return pushed;
}

struct link_reg *link_reg_lookup(struct link_pmc *lp, const char *sym)
{
	struct link_reg			*reg = NULL;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < LINK_REGS_MAX; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] == 0)
			break;

		if (strcmp(lp->reg[reg_ID].sym, sym) == 0) {

			reg = &lp->reg[reg_ID];
			break;
		}
	}

	return reg;
}

int link_reg_lookup_range(struct link_pmc *lp, const char *sym, int *min, int *max)
{
	int				n, rc, reg_ID;

	n = strlen(sym);
	rc = 0;

	for (reg_ID = 0; reg_ID < LINK_REGS_MAX; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] == 0)
			break;

		if (strncmp(lp->reg[reg_ID].sym, sym, n) == 0) {

			if (rc == 0) {

				*min = reg_ID;
				rc = 1;
			}

			*max = reg_ID;
		}
		else if (rc != 0)
			break;
	}

	return rc;
}

