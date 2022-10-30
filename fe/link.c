#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "link.h"
#include "dirent.h"
#include "serial.h"

#define LINK_EOL			"\r\n"
#define LINK_SPACE			" \t"
#define LINK_EXTRA			"])"

struct link_priv {

	struct serial_fd	*fd;

	int			reg_push_ID;

	char			lbuf[LINK_MESSAGE_MAX];
	char			pbuf[LINK_MESSAGE_MAX];

	char			hwinfo_revision[LINK_NAME_MAX];
	char			hwinfo_build[LINK_NAME_MAX];
	char			hwinfo_crc32[LINK_NAME_MAX];

	FILE			*fd_grab;
	FILE			*fd_push;
	FILE			*fd_log;

	int			push_flag;

	int			fetch_hwinfo;
	int			fetch_flash_info;
	int			fetch_unable;

	char			*flash_map;

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
link_fetch_reg_format(struct link_pmc *lp)
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

		sprintf(lp->hwinfo, "%.16s %.16s %.22s",
				priv->hwinfo_revision,
				priv->hwinfo_build,
				priv->hwinfo_crc32);
	}
}

static void
link_fetch_flash_info(struct link_pmc *lp)
{
	struct link_priv	*priv = lp->priv;
	const char		*s = priv->lbuf;
	char			*map;
	int			N = 0;

	map = priv->flash_map;

	while (*s != 0) {

		if (		   *s == 'x'
				|| *s == 'a'
				|| *s == '.') {

			*map++ = *s;

			++N;

			if (N >= 32)
				break;
		}
		else if (*s != ' ') {

			N = 0;
			break;
		}

		++s;
	}

	if (N > 0) {

		*map++ = '\n';

		priv->flash_map = map;
	}
	else {
		*(priv->flash_map) = 0;
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

void link_open(struct link_pmc *lp, const char *devname, int baudrate)
{
	struct link_priv	*priv;

	if (lp->linked != 0)
		return ;

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

		if (priv->fd_grab != NULL) {

			fclose(priv->fd_grab);
		}

		if (priv->fd_log != NULL) {

			fclose(priv->fd_log);
		}

		memset(priv, 0, sizeof(struct link_priv));
	}

	memset(lp, 0, sizeof(struct link_pmc));

	lp->priv = priv;
}

int link_fetch(struct link_pmc *lp, int clock)
{
	struct link_priv	*priv = lp->priv;
	int			rc_local, N = 0;

	lp->clock = clock;

	if (lp->linked == 0)
		return 0;

	while (serial_async_fgets(priv->fd, priv->lbuf, sizeof(priv->lbuf)) == SERIAL_OK) {

		if (priv->fd_log != NULL) {

			fprintf(priv->fd_log, "%s\n", priv->lbuf);
			fflush(priv->fd_log);
		}

		rc_local = link_fetch_network(lp);

		if (		lp->grab_mode == LINK_GRAB_TRANSFER
				&& priv->fd_grab != NULL) {

			if (rc_local != 0) {

				fclose(priv->fd_grab);
				priv->fd_grab = NULL;

				lp->grab_mode = LINK_GRAB_FINISHED;
			}
			else {
				/* Grab the next LINE.
				 * */
				fprintf(priv->fd_grab, "%s\n", priv->lbuf);
				fflush(priv->fd_grab);

				lp->grabed = lp->clock;
				lp->grab_fetched_N++;
			}
		}
		else {
			if (rc_local != 0) {

				if (lp->push_mode == LINK_PUSH_QUEUED) {

					lp->push_mode = LINK_PUSH_FINISHED;
				}

				if (strstr(priv->lbuf, "rtos_version") != NULL) {

					priv->fetch_hwinfo = 4;
				}
				else if (strstr(priv->lbuf, "rtos_reboot") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "flash_info") != NULL) {

					priv->fetch_flash_info = 8;
					priv->flash_map = lp->flash_info_map;
				}
				else if (strstr(priv->lbuf, "flash_prog") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "flash_wipe") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_self_test") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_self_adjust") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_base") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_spinup") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_detached") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_const_E") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_const_J") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (strstr(priv->lbuf, "pm_probe_noise_") != NULL) {

					priv->fetch_unable = 1;
				}
				else if (	lp->grab_mode != LINK_GRAB_WAIT
						|| priv->fd_grab == NULL) {

					/* Shut up here */
				}
				else if (strstr(priv->lbuf, "tlm_flush_sync") != NULL) {

					lp->grab_mode = LINK_GRAB_TRANSFER;
				}
				else if (strstr(priv->lbuf, "tlm_live_sync") != NULL) {

					lp->grab_mode = LINK_GRAB_TRANSFER;
				}
				else if (strstr(priv->lbuf, "export_reg") != NULL) {

					lp->grab_mode = LINK_GRAB_TRANSFER;
				}
			}
			else {
				/* Always check for reg printouts.
				 * */
				link_fetch_reg_format(lp);

				if (priv->fetch_hwinfo > 0) {

					link_fetch_hwinfo(lp);
					priv->fetch_hwinfo--;
				}

				if (priv->fetch_flash_info > 0) {

					link_fetch_flash_info(lp);
					priv->fetch_flash_info--;
				}

				if (priv->fetch_unable > 0) {

					link_fetch_unable_warning(lp);
					priv->fetch_unable = 0;
				}
			}
		}

		N++;
	}

	if (		lp->grabed + 1000 < lp->clock
			&& priv->fd_grab != NULL) {

		fclose(priv->fd_grab);
		priv->fd_grab = NULL;

		lp->grab_mode = LINK_GRAB_FINISHED;
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

	if (		lp->push_mode == LINK_PUSH_TRANSFER
			&& priv->fd_push != NULL) {

		char		*eol;

		do {
			if (priv->push_flag != 0) {

				if (serial_async_fputs(priv->fd, priv->pbuf) != SERIAL_OK)
					break;

				if (priv->push_flag < 0) {

					fclose(priv->fd_push);
					priv->fd_push = NULL;

					lp->locked = lp->clock + 100;
					lp->push_mode = LINK_PUSH_QUEUED;
					break;
				}

				priv->push_flag = 0;
			}

			if (fgets(priv->pbuf, sizeof(priv->pbuf) - 2U,
						priv->fd_push) != NULL) {

				eol = strchr(priv->pbuf, '\r');
				if (eol != NULL) *eol = 0;

				eol = strchr(priv->pbuf, '\n');
				if (eol != NULL) *eol = 0;

				strcat(priv->pbuf, LINK_EOL);

				lp->push_queued_N++;

				if (serial_async_fputs(priv->fd, priv->pbuf) != SERIAL_OK) {

					priv->push_flag = 1;
					break;
				}
			}
			else {
				strcpy(priv->pbuf, "reg iodef_ECHO 1" LINK_EOL LINK_EOL);

				priv->push_flag = -1;
			}
		}
		while (1);

		return ;
	}

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

			dofetch = 0;

			if (reg->onefetch != 0) {

				if (reg->fetched < reg->shown) {

					dofetch = 1;
					reg->onefetch = 0;
				}
			}
			else if (reg->update != 0) {

				if (reg->fetched + reg->update < reg->shown)
					dofetch = 1;
			}

			if (dofetch != 0) {

				sprintf(priv->lbuf, "reg %i" LINK_EOL, reg_ID);

				if (serial_async_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

					reg->queued = lp->clock;
					lp->locked = lp->clock + lp->quantum;
				}

				break;
			}

			if (reg->modified > reg->fetched) {

				sprintf(priv->lbuf, "reg %i %.77s" LINK_EOL,
						reg_ID, reg->val);

				if (serial_async_fputs(priv->fd, priv->lbuf) == SERIAL_OK) {

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

void link_reg_fetch_all_shown(struct link_pmc *lp)
{
	struct link_reg			*reg = NULL;
	int				reg_ID;

	for (reg_ID = 0; reg_ID < LINK_REGS_MAX; ++reg_ID) {

		if (lp->reg[reg_ID].sym[0] == 0)
			break;

		reg = &lp->reg[reg_ID];

		reg->shown = 0;
		reg->update = 0;
		reg->onefetch = 1;
	}
}

int link_grab_file(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	int			rc = 0;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_grab == NULL) {

		fd = fopen_from_UTF8(file, "w");

		if (fd != NULL) {

			lp->grabed = lp->clock;
			lp->grab_mode = LINK_GRAB_WAIT;
			lp->grab_fetched_N = 0;

			priv->fd_grab = fd;

			rc = 1;
		}
	}

	return rc;
}

int link_push_file(struct link_pmc *lp, const char *file)
{
	struct link_priv	*priv = lp->priv;
	FILE			*fd;
	int			rc = 0;

	if (lp->linked == 0)
		return 0;

	if (priv->fd_grab == NULL) {

		fd = fopen_from_UTF8(file, "r");

		if (fd != NULL) {

			lp->push_mode = LINK_PUSH_TRANSFER;
			lp->push_queued_N = 0;

			strcpy(priv->pbuf, "reg iodef_ECHO 0" LINK_EOL);

			priv->fd_push = fd;
			priv->push_flag = 1;

			rc = 1;
		}
	}

	return rc;
}

int link_log_file(struct link_pmc *lp, const char *file)
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

