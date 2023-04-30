#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "config.h"
#include "link.h"
#include "dirent.h"
#include "serial.h"

#define LINK_EOL			"\r\n"
#define LINK_SPACE			" \t"
#define LINK_EXTRA			"])"

enum {
	LINK_MODE_IDLE			= 0,
	LINK_MODE_DATA_GRAB,
	LINK_MODE_HWINFO,
	LINK_MODE_EPCAN_MAP,
	LINK_MODE_FLASH_MAP,
	LINK_MODE_UNABLE_WARNING,
};

struct link_priv {

	struct serial_fd	*fd;

	int			link_mode;
	int			reg_push_ID;

	char			lbuf[LINK_MESSAGE_MAX];
	char			pbuf[LINK_MESSAGE_MAX];

	FILE			*fd_log;
	FILE			*fd_tlm;
	FILE			*fd_grab;

	char			hw_revision[LINK_NAME_MAX];
	char			hw_build[LINK_NAME_MAX];
	char			hw_crc32[LINK_NAME_MAX];

	int			tlm_clock;

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

static int
link_fetch_network(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;
	int			n, rc = 0;

	if (strstr(lbuf, "(pmc)") == lbuf) {

		sprintf(lp->network, "local");

		rc = 1;
	}

	if (strstr(lbuf, "(net/") == lbuf) {

		if (lk_stoi(&n, lbuf + 5) != NULL) {

			sprintf(lp->network, "remote/%i", n);
		}
		else {
			lp->network[0] = 0;
		}

		rc = 2;
	}

	return rc;
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
			&& (reg->mode & LINK_REG_TYPE_INT) != 0
			&& strlen(reg->um) >= 7
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
link_fetch_reg_format(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			ldup[sizeof(priv->lbuf)], *sp = ldup;
	const char		*tok, *sym;
	int			reg_mode, reg_ID, eol, n;

	strcpy(ldup, priv->lbuf);

	reg_ID = -1;

	if (*sp >= '0' && *sp <= '7') {

		reg_mode = (int) (*sp - '0');

		if (		   *(sp + 1) == ' '
				&& *(sp + 2) == '[') {

			sp += 2;

			tok = lk_token(&sp);
			eol = strlen(tok) - 1;

			if (tok[eol] == ']') {

				if (lk_stoi(&n, tok + 1) != NULL) {

					reg_ID = n;
				}
			}
		}
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

			lp->reg_MAX_N = (reg_ID + 1 > lp->reg_MAX_N)
				? reg_ID + 1 : lp->reg_MAX_N;
		}
	}
}

static void
link_fetch_hwinfo(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*sp = priv->lbuf;
	const char		*tok;

	tok = lk_token(&sp);

	if (strcmp(tok, "HW_revision") == 0) {

		sprintf(priv->hw_revision, "%.16s", lk_token(&sp));
	}
	else if (strcmp(tok, "FW_build") == 0) {

		sprintf(priv->hw_build, "%.16s", lk_token(&sp));
	}
	else if (strcmp(tok, "FW_crc32") == 0) {

		sprintf(priv->hw_crc32, "%.9s %.12s", lk_token(&sp), lk_token(&sp));

		sprintf(lp->hwinfo, "%.16s %.16s %.22s", priv->hw_revision,
				priv->hw_build, priv->hw_crc32);
	}
}

static void
link_fetch_epcan_map(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*sp = priv->lbuf;
	const char		*tok, *eol;
	int			N;

	tok = lk_token(&sp);
	eol = tok;

	while (*eol != 0) {

		if (*eol >= '0' && *eol <= '9') ;
		else if (*eol >= 'A' && *eol <= 'F') ;
		else if (*eol >= 'a' && *eol <= 'f') ;
		else break;

		++eol;
	}

	if (*eol == 0) {

		for (N = 0; N < LINK_EPCAN_MAX; ++N) {

			if (lp->epcan[N].UID[0] == 0)
				break;
		}

		if (lp->epcan[N].UID[0] != 0)
			return ;

		sprintf(lp->epcan[N].UID, "%.15s", tok);

		tok = lk_token(&sp);

		sprintf(lp->epcan[N].node_ID, "%.23s", tok);
	}
}

static void
link_fetch_flash_map(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	const char		*sp = priv->lbuf;
	int			N, bN;

	for (N = 0; N < LINK_FLASH_MAX; ++N) {

		if (lp->flash[N].block[0] == 0)
			break;
	}

	if (lp->flash[N].block[0] != 0)
		return ;

	bN = 0;

	while (*sp != 0) {

		if (		   *sp == 'x'
				|| *sp == 'a'
				|| *sp == '.') {

			lp->flash[N].block[bN++] = *sp;

			if (bN >= sizeof(lp->flash[0].block))
				break ;
		}
		else if (*sp != ' ')
			break;

		++sp;
	}
}

static void
link_fetch_unable_warning(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;

	if (strstr(lbuf, "Unable ") == lbuf) {

		sprintf(lp->unable_warning, "%.200s", lbuf);
	}
}

void link_open(struct link_pmc *lp, struct config_phobia *fe,
		const char *devname, int baudrate, const char *mode)
{
	struct link_priv	*priv;

	if (lp->linked != 0)
		return ;

	lp->fe = fe;

	sprintf(lp->devname, "%.77s", devname);

	lp->baudrate = baudrate;
	lp->quantum = 10;

	if (lp->priv != NULL) {

		memset(lp->priv, 0, sizeof(struct link_priv));
	}
	else {
		lp->priv = calloc(1, sizeof(struct link_priv));
	}

	priv = lp->priv;
	priv->fd = serial_open(devname, baudrate, mode);

	if (priv->fd == NULL)
		return ;

	lp->linked = 1;

	link_remote(lp);
}

void link_close(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return ;

	if (priv != NULL) {

		if (priv->fd != NULL) {

			serial_close(priv->fd);
		}

		if (priv->fd_log != NULL) {

			fclose(priv->fd_log);
		}

		if (priv->fd_tlm != NULL) {

			fclose(priv->fd_tlm);
		}

		if (priv->fd_grab != NULL) {

			fclose(priv->fd_grab);
		}

		memset(priv, 0, sizeof(struct link_priv));
	}

	memset(lp, 0, sizeof(struct link_pmc));

	lp->priv = priv;
}

void link_remote(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return ;

	priv->link_mode = LINK_MODE_IDLE;
	priv->reg_push_ID = 0;
	priv->mbflow = priv->mb;

	if (priv->fd_tlm != NULL) {

		fclose(priv->fd_tlm);
		priv->fd_tlm = NULL;
	}

	if (priv->fd_grab != NULL) {

		fclose(priv->fd_grab);
		priv->fd_grab = NULL;
	}

	lp->fetched_N = 0;

	lp->locked = lp->clock + 1000;
	lp->active = lp->clock;

	lp->grab_N = 0;
	lp->tlm_N = 0;

	memset(lp->reg, 0, sizeof(lp->reg));

	lp->reg_MAX_N = 0;

	serial_fputs(priv->fd, LINK_EOL);

	sprintf(priv->lbuf, "rtos_version" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "flash_info" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "reg" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);
}

static void
link_tlm_label(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd = priv->fd_tlm;
	struct link_reg		*reg;

	char			sym[40];
	int			N, reg_ID;

	fprintf(fd, "time@s;");

	for (N = 0; N < 10; ++N) {

		sprintf(sym, "tlm.reg_ID%i", N);

		reg = link_reg_lookup(lp, sym);
		reg_ID = (reg != NULL) ? reg->lval : 0;

		if (reg_ID > 0 && reg_ID < lp->reg_MAX_N) {

			reg = &lp->reg[reg_ID];

			fprintf(fd, "%s", reg->sym);

			if (reg->um[0] != 0) {

				fprintf(fd, "@%s", reg->um);
			}

			fprintf(fd, ";");
		}
	}

	fprintf(fd, "\n");
	fflush(fd);
}

static void
link_tlm_flush(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	struct config_phobia	*fe = lp->fe;
	FILE			*fd = priv->fd_tlm;
	struct link_reg		*reg;

	char			sym[40];
	int			N, reg_ID;

	fprintf(fd, "%.3f;", (double) lp->tlm_N * (fe->tlmrate / 1000.));

	for (N = 0; N < 10; ++N) {

		sprintf(sym, "tlm.reg_ID%i", N);

		reg = link_reg_lookup(lp, sym);
		reg_ID = (reg != NULL) ? reg->lval : 0;

		if (reg_ID > 0 && reg_ID < lp->reg_MAX_N) {

			reg = &lp->reg[reg_ID];

			fprintf(fd, "%s;", reg->val);
		}
	}

	fprintf(fd, "\n");
	fflush(fd);

	lp->tlm_N++;
}

int link_fetch(struct link_pmc *lp, int clock)
{
	struct link_priv	*priv = lp->priv;
	struct config_phobia	*fe = lp->fe;
	int			rc_local, N = 0;

	struct {

		const char	*command;
		int		mode;
	}
	const link_map[] = {

		{ "rtos_version",	LINK_MODE_HWINFO },
		{ "rtos_reboot",	LINK_MODE_UNABLE_WARNING },
		{ "flash_info",		LINK_MODE_FLASH_MAP },
		{ "flash_prog",		LINK_MODE_UNABLE_WARNING },
		{ "flash_wipe",		LINK_MODE_UNABLE_WARNING },
		{ "pm_self_",		LINK_MODE_UNABLE_WARNING },
		{ "pm_probe_",		LINK_MODE_UNABLE_WARNING },
		{ "pm_adjust_",		LINK_MODE_UNABLE_WARNING },
		{ "tlm_flush_sync",	LINK_MODE_DATA_GRAB },
		{ "tlm_live_sync",	LINK_MODE_DATA_GRAB },
		{ "net_survey",		LINK_MODE_EPCAN_MAP },
		{ "net_assign",		LINK_MODE_UNABLE_WARNING },
		{ "net_revoke",		LINK_MODE_UNABLE_WARNING },

		{ NULL, 0 }	/* END */
	},
	*mp;

	lp->clock = clock;

	if (lp->linked == 0)
		return 0;

	while (serial_fgets(priv->fd, priv->lbuf, sizeof(priv->lbuf)) == SERIAL_OK) {

		lp->active = lp->clock;

		if (priv->fd_log != NULL) {

			fprintf(priv->fd_log, "%s\n", priv->lbuf);
			fflush(priv->fd_log);
		}

		rc_local = link_fetch_network(lp);

		if (rc_local != 0) {

			if (priv->link_mode == LINK_MODE_DATA_GRAB) {

				if (priv->fd_grab != NULL) {

					fclose(priv->fd_grab);
					priv->fd_grab = NULL;
				}

				lp->grab_N = 0;
			}

			priv->link_mode = LINK_MODE_IDLE;
		}
		else {
			link_fetch_reg_format(lp);
		}

		switch (priv->link_mode) {

			case LINK_MODE_IDLE:

				if (rc_local == 0)
					break;

				mp = link_map;

				while (mp->command != NULL) {

					if (strstr(priv->lbuf, mp->command)) {

						priv->link_mode = mp->mode;
						break;
					}

					mp++;
				}

				if (priv->link_mode == LINK_MODE_FLASH_MAP)
					memset(lp->flash, 0, sizeof(lp->flash));
				else if (priv->link_mode == LINK_MODE_EPCAN_MAP)
					memset(lp->epcan, 0, sizeof(lp->epcan));

				break;

			case LINK_MODE_DATA_GRAB:

				if (priv->fd_grab == NULL)
					break;

				fprintf(priv->fd_grab, "%s\n", priv->lbuf);
				fflush(priv->fd_grab);

				lp->grab_N++;
				break;

			case LINK_MODE_HWINFO:
				link_fetch_hwinfo(lp);
				break;

			case LINK_MODE_EPCAN_MAP:
				link_fetch_epcan_map(lp);
				break;

			case LINK_MODE_FLASH_MAP:
				link_fetch_flash_map(lp);
				break;

			case LINK_MODE_UNABLE_WARNING:
				link_fetch_unable_warning(lp);
				break;
		}

		N++;
	}

	if (		priv->link_mode == LINK_MODE_DATA_GRAB
			&& lp->active + 500 < lp->clock) {

		link_grab_file_close(lp);
	}

	if (		lp->active + 1000 < lp->clock
			&& lp->keep + 1000 < lp->clock) {

		lp->keep = lp->clock;

		if (lp->active + 10000 > lp->clock) {

			serial_fputs(priv->fd, LINK_EOL);
		}
		else {
			link_close(lp);
		}
	}

	if (		priv->fd_tlm != NULL
			&& priv->tlm_clock < lp->clock) {

		priv->tlm_clock += fe->tlmrate;

		link_tlm_flush(lp);
	}

	lp->fetched_N += N;

	return N;
}

void link_push(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	struct link_reg		*reg;
	int			reg_ID, dofetch;

	if (lp->linked == 0)
		return ;

	if (priv->link_mode == LINK_MODE_DATA_GRAB)
		return ;

	if (lp->locked > lp->clock)
		return ;

	reg_ID = priv->reg_push_ID;

	do {
		reg = lp->reg + reg_ID;

		if (reg->queued == 0) {

			dofetch = 0;

			if (reg->update != 0) {

				if (reg->fetched + reg->update < reg->shown)
					dofetch = 1;
			}

			if (reg->always != 0) {

				if (reg->fetched + reg->always < lp->clock)
					dofetch = 1;
			}

			if (reg->onefetch != 0) {

				if (reg->fetched < reg->shown) {

					reg->onefetch = 0;
					dofetch = 1;
				}
			}

			if (dofetch != 0) {

				sprintf(priv->lbuf, "reg %i" LINK_EOL, reg_ID);

				if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock + lp->quantum;
				}

				break;
			}

			if (reg->modified > reg->fetched) {

				sprintf(priv->lbuf, "reg %i %.77s" LINK_EOL,
						reg_ID, reg->val);

				if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock + lp->quantum;
				}

				break;
			}
		}
		else {
			if (reg->queued + 500 < lp->clock)
				reg->queued = 0;
		}

		reg_ID++;

		if (reg_ID >= lp->reg_MAX_N)
			reg_ID = 0;

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

	sprintf(priv->lbuf, "%.90s" LINK_EOL, command);

	if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

		pushed = 1;

		lp->locked = lp->clock + 100;
	}

	return pushed;
}

struct link_reg *link_reg_lookup(struct link_pmc *lp, const char *sym)
{
	struct link_reg			*reg = NULL;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			if (strcmp(lp->reg[reg_ID].sym, sym) == 0) {

				reg = &lp->reg[reg_ID];
				break;
			}
		}
	}

	return reg;
}

int link_reg_lookup_range(struct link_pmc *lp, const char *sym, int *min, int *max)
{
	int				n, rc, reg_ID;

	n = strlen(sym);
	rc = 0;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

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
	}

	return rc;
}

void link_reg_fetch_all_shown(struct link_pmc *lp)
{
	struct link_reg			*reg = NULL;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			reg = &lp->reg[reg_ID];

			reg->shown = 0;
			reg->update = 0;

			reg->onefetch = 1;
		}
	}
}

void link_reg_clean_all_always(struct link_pmc *lp)
{
	struct link_reg			*reg = NULL;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			reg = &lp->reg[reg_ID];

			reg->always = 0;
		}
	}
}

int link_log_file_open(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	time_t			tm;
	int			rc = 0;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_log == NULL) {

		fd = fopen_from_UTF8(file, "a");

		if (fd != NULL) {

			time(&tm);

			fprintf(fd, "# log opened %s\n", ctime(&tm));

			priv->fd_log = fd;

			rc = 1;
		}
	}

	return rc;
}

int link_tlm_file_open(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	int			rc = 0;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_tlm == NULL) {

		fd = fopen_from_UTF8(file, "w");

		if (fd != NULL) {

			priv->fd_tlm = fd;
			priv->tlm_clock = lp->clock;

			lp->tlm_N = 0;

			link_tlm_label(lp);
			link_tlm_flush(lp);
			link_tlm_flush(lp);
			link_tlm_flush(lp);

			rc = 1;
		}
	}

	return rc;
}

void link_tlm_file_close(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return;

	if (priv->fd_tlm != NULL) {

		fclose(priv->fd_tlm);
		priv->fd_tlm = NULL;
	}

	lp->tlm_N = 0;
}

int link_grab_file_open(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	int			rc = 0;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_tlm != NULL)
		return 0;

	if (priv->fd_grab == NULL) {

		fd = fopen_from_UTF8(file, "w");

		if (fd != NULL) {

			priv->fd_grab = fd;

			lp->grab_N = 0;

			rc = 1;
		}
	}

	return rc;
}

void link_grab_file_close(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return;

	if (priv->fd_grab != NULL) {

		fclose(priv->fd_grab);
		priv->fd_grab = NULL;
	}

	if (priv->link_mode == LINK_MODE_DATA_GRAB) {

		lp->grab_N = 0;

		priv->link_mode = LINK_MODE_IDLE;
	}
}

