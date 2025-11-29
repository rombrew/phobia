#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "gp/dirent.h"
#include "config.h"
#include "link.h"
#include "serial.h"

#define LINK_EOL			"\r\n"
#define LINK_SPACE			" \t"
#define LINK_EXTRA			" \t])"

#define LINK_ALLOC_MAX			92160U
#define LINK_CACHE_MAX			4096U

enum {
	LINK_MODE_IDLE			= 0,
	LINK_MODE_HWINFO,
	LINK_MODE_GETTICK,
	LINK_MODE_DATA_GRAB,
	LINK_MODE_EPCAN_MAP,
	LINK_MODE_FLASH_MAP,
	LINK_MODE_COMMAND,
};

struct link_priv {

	struct serial_fd	*fd;

	int			link_mode;
	int			reg_push_ID;

	char			lbuf[LINK_LINE_MAX];

	FILE			*fd_log;
	FILE			*fd_grab;

	char			mb[LINK_ALLOC_MAX];
	char			*mbflow;

	int			cache[LINK_CACHE_MAX];
};

const char *lk_stoi(int *x, const char *s)
{
	int		n, d, i;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	i = 0;

	while (*s >= '0' && *s <= '9') {

		i = 10 * i + (*s++ - '0') * n;
		d += 1;
	}

	if (d == 0 || d > 9) { return NULL; }

	if (*s == 0 || strchr(LINK_EXTRA, *s) != NULL
			|| strchr(LINK_EOL, *s) != NULL) {

		*x = i;
	}
	else { return NULL; }

	return s;
}

const char *lk_stod(double *x, const char *s)
{
	int		n, d, v, e;
	double		f;

	if (*s == '-') { n = - 1; s++; }
	else if (*s == '+') { n = 1; s++; }
	else { n = 1; }

	d = 0;
	v = 0;
	f = 0.;

	while (*s >= '0' && *s <= '9') {

		f = 10. * f + (*s++ - '0') * n;
		d += 1;
	}

	if (*s == '.') {

		s++;

		while (*s >= '0' && *s <= '9') {

			f = 10. * f + (*s++ - '0') * n;
			d += 1; v -= 1;
		}
	}

	if (d == 0) { return NULL; }

	if (*s == 'p') { v += - 12; s++; }
	else if (*s == 'n') { v += - 9; s++; }
	else if (*s == 'u') { v += - 6; s++; }
	else if (*s == 'm') { v += - 3; s++; }
	else if (*s == 'K') { v += 3; s++; }
	else if (*s == 'M') { v += 6; s++; }
	else if (*s == 'G') { v += 9; s++; }
	else if (*s == 'T') { v += 12; s++; }
	else if (*s == 'e' || *s == 'E') {

		s = lk_stoi(&e, s + 1);

		if (s != NULL) { v += e; }
		else { return NULL; }
	}

	if (*s == 0 || strchr(LINK_EXTRA, *s) != NULL
			|| strchr(LINK_EOL, *s) != NULL) {

		while (v < 0) { f /= 10.; v += 1; }
		while (v > 0) { f *= 10.; v -= 1; }

		*x = f;
	}
	else { return NULL; }

	return s;
}

static const char *
lk_space(const char *s)
{
	while (*s != 0 && strchr(LINK_SPACE, *s) != 0) { ++s; }

	return s;
}

static const char *
lk_eol(const char *s)
{
	while (*s != 0 && strchr(LINK_EOL, *s) == 0) { ++s; }

	return s;
}

static const char *
lk_token(char **sp)
{
	const char	*tok;
	char		*s;

	tok = lk_space(*sp);
	s = (char *) tok;

	if (tok[0] == '"') {

		++tok;
		++s;

		while (*s != 0 && *s != '"') { ++s; }
	}
	else if (tok[0] == '[') {

		++tok;
		++s;

		while (*s != 0 && *s != ']') { ++s; }
	}
	else if (tok[0] == '(') {

		++tok;
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

	return tok;
}

static unsigned int
lk_hash(const char *sym)
{
	unsigned long		hash = 0U;

	while (*sym != 0) {

		hash = (hash + *sym) * 1149773U;
		hash ^= (hash << 1) + (hash >> 4);

		++sym;
	}

	hash ^= (hash << 15);
	hash = (hash >> 16) & (LINK_CACHE_MAX - 1U);

	return hash;
}

static char *
link_mballoc(struct link_pmc *lp, int len)
{
	struct link_priv	*priv = lp->priv;
	char			*mb = NULL;

	if ((int) (priv->mbflow - priv->mb) < LINK_ALLOC_MAX - len) {

		mb = priv->mbflow;
		priv->mbflow += len;
	}

	return mb;
}

static int
link_fetch_network(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;
	int			net_ID;

	if (strstr(lbuf, "(pmc)") == lbuf) {

		sprintf(lp->network, "SERIAL");

		return 1;
	}

	if (strstr(lbuf, "(net/") == lbuf) {

		if (lk_stoi(&net_ID, lbuf + 5) != NULL) {

			sprintf(lp->network, "REMOTE/%i", net_ID);
		}
		else {
			lp->network[0] = 0;
		}

		return 2;
	}

	return 0;
}

static void
link_reg_postproc(struct link_pmc *lp, struct link_reg *reg)
{
	double			dval;

	if (lk_stoi(&reg->lval, reg->val) != NULL) {

		reg->mode |= LINK_REG_TYPE_INT;
	}

	if (lk_stod(&dval, reg->val) != NULL) {

		reg->mode |= LINK_REG_TYPE_FLOAT;

		reg->fval = (float) dval;

		if (reg->started != 0) {

			if (reg->fval < reg->fmin) {

				reg->fmin = reg->fval;
				strcpy(reg->vmin, reg->val);
			}

			if (reg->fval > reg->fmax) {

				reg->fmax = reg->fval;
				strcpy(reg->vmax, reg->val);
			}
		}
		else {
			reg->fmin = reg->fval;
			reg->fmax = reg->fval;

			strcpy(reg->vmin, reg->val);
			strcpy(reg->vmax, reg->val);

			reg->started = 1;
		}
	}

	if (		(reg->mode & 7U) == LINK_REG_CONFIG
			&& (reg->mode & LINK_REG_TYPE_INT) != 0
			&& strlen(reg->um) >= 7
			&& strlen(reg->um) < LINK_NAME_MAX
			&& reg->lval >= 0 && reg->lval < LINK_COMBO_MAX) {

		reg->mode |= LINK_REG_TYPE_ENUMERATE;

		if (reg->combo[reg->lval] == NULL) {

			reg->combo[reg->lval] = link_mballoc(lp, LINK_NAME_MAX);
		}

		if (reg->combo[reg->lval] != NULL) {

			sprintf(reg->combo[reg->lval], "%.79s", reg->um);

			reg->lmax_combo = (reg->lval > reg->lmax_combo)
				? reg->lval : reg->lmax_combo;
		}
	}
	else if (reg->enumerated != 0) {

		reg->mode |= LINK_REG_TYPE_ENUMERATE;
	}
}

static void
link_fetch_reg_format(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			ldup[LINK_LINE_MAX], *sp = ldup;
	const char		*tok, *sym;
	int			reg_mode, reg_ID, N;

	strcpy(ldup, priv->lbuf);

	reg_ID = -1;

	tok = lk_token(&sp);

	if (strlen(tok) <= 2) {

		if (lk_stoi(&N, tok) != NULL) {

			reg_mode = N;

			tok = lk_token(&sp);

			if (		strlen(tok) <= 3
					&& tok[-1] == '[') {

				if (lk_stoi(&N, lk_space(tok)) != NULL) {

					reg_ID = N;
				}
			}
		}
	}

	if (reg_ID >= 0 && reg_ID < LINK_REGS_MAX) {

		struct link_reg		*reg = lp->reg + reg_ID;

		sym = lk_token(&sp);
		tok = lk_token(&sp);

		if (strcmp(tok, "=") == 0) {

			char			text[80];
			struct link_reg		local;

			sprintf(text, "%.79s", sp = (char *) lk_space(sp));

			reg->mode = reg_mode;

			if (reg->mode & LINK_REG_HIDDEN) {

				local = *reg;
				reg = &local;
			}

			sprintf(reg->sym, "%.79s", sym);
			sprintf(reg->val, "%.79s", lk_token(&sp));
			sprintf(reg->um,  "%.79s", lk_token(&sp));

			tok = lk_space(sp);

			if (tok[0] == 0) {

				link_reg_postproc(lp, reg);
			}
			else {
				strcpy(reg->val, text);

				reg->um[0] = 0;
			}

			if (reg->mode & LINK_REG_HIDDEN) {

				reg = lp->reg + reg_ID;

				reg->mode = local.mode;
				reg->lmax_combo = local.lmax_combo;

				for (N = 0; N <= reg->lmax_combo; ++N)
					reg->combo[N] = local.combo[N];
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

	if (strcmp(tok, "Hardware") == 0) {

		sprintf(lp->hw.hardware, "%.64s", lk_token(&sp));
	}
	else if (strcmp(tok, "Revision") == 0) {

		sprintf(lp->hw.revision, "%.64s", lk_token(&sp));
	}
	else if (strcmp(tok, "Build") == 0) {

		sprintf(lp->hw.build, "%.16s", lk_token(&sp));
	}
	else if (strcmp(tok, "CRC32") == 0) {

		sprintf(lp->hw.crc32, "%.16s", lk_token(&sp));

		tok = lk_token(&sp);

		sprintf(lp->hw.crc32 + strlen(lp->hw.crc32), " (%.16s)", tok);

		sprintf(lp->hwinfo, "%.64s / %.32s", lp->hw.hardware, lp->hw.crc32);
	}
}

static void
link_fetch_gettick(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*sp = priv->lbuf;
	int			time;

	if (strcmp(lk_token(&sp), "TN") == 0) {

		lk_token(&sp);

		if (lk_stoi(&time, lk_token(&sp)) != NULL) {

			if (time < lp->time) {

				lp->time_warning = 1;
			}

			lp->time = time;
		}
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
		sprintf(lp->epcan[N].node_ID, "%.23s", lk_token(&sp));
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
link_fetch_command(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	char			*lbuf = priv->lbuf;
	int			len;

	if (strstr(lbuf, "Unable ") == lbuf) {

		lp->unable_warning = 1;
	}

	len = strlen(lp->command_grab);

	if (strlen(lbuf) < LINK_DATA_MAX - len) {

		sprintf(lp->command_grab + len, "%s\n", lbuf);
	}
}

void link_open(struct link_pmc *lp, struct config_phobia *fe,
		const char *devname, int baudrate, const char *mode)
{
	struct link_priv	*priv;

	if (lp->linked != 0)
		return ;

	lp->fe = fe;

	sprintf(lp->devname, "%.79s", devname);

	lp->baudrate = baudrate;

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

	priv->link_mode = LINK_MODE_IDLE;
	priv->mbflow = priv->mb;

	lp->locked = lp->clock + 1000;
	lp->active = lp->clock;
	lp->keep = lp->clock;

	strcpy(lp->reg[0].sym, "null");

	lp->reg_MAX_N = 1;

	sprintf(priv->lbuf, "\x04\x04" LINK_EOL LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "ap_version" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "flash_info" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "reg" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	lp->linked = 1;
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

	if (priv->fd_grab != NULL) {

		fclose(priv->fd_grab);
		priv->fd_grab = NULL;
	}

	lp->time = 0;

	lp->locked = lp->clock + 1000;
	lp->active = lp->clock;
	lp->keep = lp->clock;

	lp->grab_N = 0;

	memset(lp->reg, 0, sizeof(lp->reg));

	strcpy(lp->reg[0].sym, "null");

	lp->reg_MAX_N = 1;

	sprintf(priv->lbuf, LINK_EOL LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "ap_version" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "flash_info" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);

	sprintf(priv->lbuf, "reg" LINK_EOL);
	serial_fputs(priv->fd, priv->lbuf);
}

int link_fetch(struct link_pmc *lp, int clock)
{
	struct link_priv	*priv = lp->priv;
	int			rc_local, N = 0;

	struct {

		const char	*command;
		int		mode;
	}
	const	link_map[] = {

		{ "ap_version",		LINK_MODE_HWINFO },
		{ "ap_gettick",		LINK_MODE_GETTICK },
		{ "ap_log_flush",	LINK_MODE_DATA_GRAB },
		{ "ap_reboot",		LINK_MODE_COMMAND },
		{ "ap_bootload",	LINK_MODE_COMMAND },
		{ "flash_info",		LINK_MODE_FLASH_MAP },
		{ "flash_prog",		LINK_MODE_COMMAND },
		{ "flash_wipe",		LINK_MODE_COMMAND },
		{ "pm_self",		LINK_MODE_COMMAND },
		{ "pm_probe",		LINK_MODE_COMMAND },
		{ "pm_adjust",		LINK_MODE_COMMAND },
		{ "tlm_flush_sync",	LINK_MODE_DATA_GRAB },
		{ "tlm_stream_sync",	LINK_MODE_DATA_GRAB },
		{ "pm_scan_impedance",	LINK_MODE_DATA_GRAB },
		{ "net_survey",		LINK_MODE_EPCAN_MAP },
		{ "net_assign",		LINK_MODE_COMMAND },
		{ "net_revoke",		LINK_MODE_COMMAND },

		{ NULL, 0 }		/* END */
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

			if (		   lp->command_state == LINK_COMMAND_RUNING
					|| lp->command_state == LINK_COMMAND_WAITING) {

				lp->command_state = LINK_COMMAND_NONE;
			}

			if (priv->link_mode == LINK_MODE_DATA_GRAB) {

				link_grab_file_close(lp);
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

					if (strstr(priv->lbuf, mp->command) != NULL) {

						priv->link_mode = mp->mode;
						break;
					}

					mp++;
				}

				if (priv->link_mode == LINK_MODE_FLASH_MAP) {

					memset(lp->flash, 0, sizeof(lp->flash));
				}
				else if (priv->link_mode == LINK_MODE_EPCAN_MAP) {

					memset(lp->epcan, 0, sizeof(lp->epcan));
				}
				else if (priv->link_mode == LINK_MODE_COMMAND) {

					lp->command_grab[0] = 0;

					if (lp->command_state == LINK_COMMAND_PENDING) {

						lp->command_state = LINK_COMMAND_RUNING;
					}
				}
				break;

			case LINK_MODE_HWINFO:
				link_fetch_hwinfo(lp);
				break;

			case LINK_MODE_GETTICK:
				link_fetch_gettick(lp);
				break;

			case LINK_MODE_DATA_GRAB:

				if (priv->fd_grab == NULL)
					break;

				fprintf(priv->fd_grab, "%s\n", priv->lbuf);
				fflush(priv->fd_grab);

				lp->locked = lp->clock;
				lp->grab_N++;
				break;

			case LINK_MODE_EPCAN_MAP:
				link_fetch_epcan_map(lp);
				break;

			case LINK_MODE_FLASH_MAP:
				link_fetch_flash_map(lp);
				break;

			case LINK_MODE_COMMAND:
				link_fetch_command(lp);
				break;
		}

		N++;
	}

	if (priv->link_mode == LINK_MODE_DATA_GRAB) {

		if (lp->active + 1000 < lp->clock) {

			link_grab_file_close(lp);
		}
	}
	else {
		if (lp->active + 12000 < lp->clock) {

			link_close(lp);

			return N;
		}
		else if (lp->active + 1000 < lp->clock) {

			if (lp->keep + 1000 < lp->clock) {

				sprintf(priv->lbuf, "ap_gettick" LINK_EOL);
				serial_fputs(priv->fd, priv->lbuf);

				lp->keep = lp->clock;
			}
		}
		else {
			if (lp->keep + 5000 < lp->clock) {

				sprintf(priv->lbuf, "ap_gettick" LINK_EOL);
				serial_fputs(priv->fd, priv->lbuf);

				lp->keep = lp->clock;
			}
		}
	}

	lp->line_N += N;

	return N;
}

static int
link_reg_all_queued(struct link_pmc *lp)
{
	struct link_reg			*reg;
	int				reg_ID;
	int				busy_N = 0;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			reg = &lp->reg[reg_ID];

			if (reg->queued != 0)
				busy_N++;
		}
	}

	return busy_N;
}

void link_push(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	struct link_reg		*reg;
	int			reg_ID, dofetch, busy_N;

	if (lp->linked == 0)
		return ;

	if (lp->locked > lp->clock)
		return ;

	if (priv->link_mode == LINK_MODE_DATA_GRAB)
		return ;

	busy_N = link_reg_all_queued(lp);

	if (busy_N > 10)
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

			if (reg->onefetch != 0) {

				if (reg->fetched < reg->shown) {

					reg->onefetch = 0;
					dofetch = 1;
				}
			}

			if (reg->mode & LINK_REG_READ_ONLY) {

				if (reg->fetched + 10000 < reg->shown)
					dofetch = 1;
			}

			if (reg->modified > reg->fetched) {

				sprintf(priv->lbuf, "reg %i %.79s" LINK_EOL,
						reg_ID, reg->val);

				if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock;

					busy_N++;
				}
			}
			else if (dofetch != 0) {

				sprintf(priv->lbuf, "reg %i" LINK_EOL, reg_ID);

				if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock;

					busy_N++;
				}
			}

			if (		(reg->mode & LINK_REG_TYPE_ENUMERATE) != 0
					&& (reg->enumerated + 10000 < reg->fetched
						|| reg->enumerated == 0)) {

				sprintf(priv->lbuf, "enum_reg %i" LINK_EOL, reg_ID);

				if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->enumerated = lp->clock;
					reg->queued = lp->clock;

					lp->locked = lp->clock;

					busy_N++;
				}
			}
		}
		else {
			if (reg->queued + 1000 < lp->clock)
				reg->queued = 0;
		}

		reg_ID++;

		if (reg_ID >= lp->reg_MAX_N)
			reg_ID = 0;

		if (busy_N > 10)
			break;

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

	if (lp->locked > lp->clock)
		return 0;

	sprintf(priv->lbuf, "%.90s" LINK_EOL, command);

	if (serial_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

		pushed = 1;

		lp->locked = lp->clock + 200;
	}

	return pushed;
}

struct link_reg *link_reg_lookup(struct link_pmc *lp, const char *sym)
{
	struct link_priv	*priv = lp->priv;
	struct link_reg		*reg = NULL;
	int			hash, reg_ID;

	if (lp->linked == 0)
		return NULL;

	hash = lk_hash(sym);
	reg_ID = priv->cache[hash];

	if (strcmp(lp->reg[reg_ID].sym, sym) == 0) {

		reg = &lp->reg[reg_ID];
	}
	else {
		for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

			if (lp->reg[reg_ID].sym[0] != 0) {

				if (strcmp(lp->reg[reg_ID].sym, sym) == 0) {

					priv->cache[hash] = reg_ID;
					reg = &lp->reg[reg_ID];

					break;
				}
			}
		}
	}

	return reg;
}

int link_reg_lookup_range(struct link_pmc *lp, const char *sym, int *min, int *max)
{
	struct link_priv	*priv = lp->priv;
	int			len, hash, reg_ID, found = 0;

	if (lp->linked == 0)
		return 0;

	len = strlen(sym);

	hash = lk_hash(sym);
	reg_ID = priv->cache[hash];

	if (strncmp(lp->reg[reg_ID].sym, sym, len) == 0) {

		*min = reg_ID;
		*max = reg_ID++;

		found = 1;
	}
	else {
		reg_ID = 0;
	}

	for (; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			if (strncmp(lp->reg[reg_ID].sym, sym, len) == 0) {

				if (found == 0) {

					priv->cache[hash] = reg_ID;

					*min = reg_ID;
					found = 1;
				}

				*max = reg_ID;
			}
			else if (found != 0)
				break;
		}
	}

	return found;
}

void link_reg_fetch_all_shown(struct link_pmc *lp)
{
	struct link_reg			*reg;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < lp->reg_MAX_N; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] != 0) {

			reg = &lp->reg[reg_ID];

			reg->update = 0;
			reg->onefetch = 1;
		}
	}
}

void link_config_write(struct link_pmc *lp, const char *file)
{
	struct link_reg		*reg;
	FILE			*fd;
	int			reg_ID;

	if (lp->linked == 0)
		return ;

	fd = fopen_from_UTF8(file, "w");

	if (fd != NULL) {

		for (reg_ID = 1; reg_ID < lp->reg_MAX_N; ++reg_ID) {

			if (lp->reg[reg_ID].sym[0] != 0) {

				reg = &lp->reg[reg_ID];

				if (reg->mode & LINK_REG_CONFIG) {

					if (reg->mode & LINK_REG_LINKED) {

						fprintf(fd, "%s %s\n", reg->sym, reg->um);
					}
					else {
						fprintf(fd, "%s %s\n", reg->sym, reg->val);
					}
				}
			}
		}

		fclose(fd);
	}
}

void link_config_read(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	struct link_reg		*reg;
	const char		*sym, *val;

	FILE			*fd;

	if (lp->linked == 0)
		return ;

	fd = fopen_from_UTF8(file, "r");

	if (fd != NULL) {

		while (fgets(priv->lbuf, sizeof(priv->lbuf), fd) != NULL) {

			char	*sp = priv->lbuf;

			sym = lk_token(&sp);
			val = lk_token(&sp);

			reg = link_reg_lookup(lp, sym);

			if (reg != NULL) {

				* (char *) lk_eol(val) = 0;

				sprintf(reg->val, "%.70s", val);

				reg->modified = lp->clock;
			}
		}

		fclose(fd);
	}
}

int link_log_file_open(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	time_t			tm;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_log == NULL) {

		fd = fopen_from_UTF8(file, "a");

		if (fd != NULL) {

			time(&tm);

			fprintf(fd, "# log opened %s\n", ctime(&tm));

			priv->fd_log = fd;

			return 1;
		}
	}

	return 0;
}

int link_grab_file_open(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_grab == NULL) {

		fd = fopen_from_UTF8(file, "w");

		if (fd != NULL) {

			priv->fd_grab = fd;

			lp->grab_N = 1;

			return 1;
		}
	}

	return 0;
}

void link_grab_file_close(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;

	if (lp->linked == 0)
		return ;

	if (priv->fd_grab != NULL) {

		fclose(priv->fd_grab);
		priv->fd_grab = NULL;
	}

	if (priv->link_mode == LINK_MODE_DATA_GRAB) {

		priv->link_mode = LINK_MODE_IDLE;

		lp->grab_N = 0;
	}
}

