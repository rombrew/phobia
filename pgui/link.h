#ifndef _H_LINK_
#define _H_LINK_

#include "config.h"

#define LINK_REGS_MAX		900
#define LINK_NAME_MAX		80
#define LINK_MESSAGE_MAX	500
#define LINK_COMBO_MAX		40
#define LINK_EPCAN_MAX		32
#define LINK_FLASH_MAX		10

enum {
	LINK_REG_CONFIG		= 1U,
	LINK_REG_READ_ONLY	= 2U,
	LINK_REG_LINKED		= 4U,
	LINK_REG_HIDDEN		= 8U,

	LINK_REG_TYPE_INT		= (1U << 8),
	LINK_REG_TYPE_FLOAT		= (1U << 9),

	LINK_REG_TYPE_ENUMERATE		= (1U << 10)
};

enum {
	LINK_PRIMAL_UNDEFINED	= 0,
	LINK_PRIMAL_NONE,
	LINK_PRIMAL_ENABLED
};

enum {
	LINK_COMMAND_NONE	= 0,
	LINK_COMMAND_PENDING,
	LINK_COMMAND_RUNING,
	LINK_COMMAND_WAITING
};

struct link_priv;

struct link_reg {

	int		mode;

	int		shown;
	int		modified;
	int		fetched;
	int		queued;
	int		enumerated;

	char		sym[LINK_NAME_MAX];
	char		val[LINK_NAME_MAX];
	char		um[LINK_NAME_MAX];

	int		lval;
	float		fval;

	float		fmin;
	float		fmax;

	char		vmin[LINK_NAME_MAX];
	char		vmax[LINK_NAME_MAX];

	int		started;
	int		update;
	int		onefetch;

	char		*combo[LINK_COMBO_MAX];
	int		lmax_combo;

	int		um_sel;
	int		primal;
};

struct link_pmc {

	struct link_priv	*priv;
	struct config_phobia	*fe;

	char			devname[LINK_NAME_MAX];
	int			baudrate;

	int			linked;
	int			uptime;

	int			clock;
	int			locked;
	int			active;
	int			keep;

	struct {

		char		revision[LINK_NAME_MAX];
		char		identify[LINK_NAME_MAX];
		char		build[LINK_NAME_MAX];
		char		crc32[LINK_NAME_MAX];
	}
	hw;

	char			hwinfo[LINK_NAME_MAX];
	char			network[LINK_NAME_MAX];

	struct {

		char		UID[16];
		char		node_ID[24];
	}
	epcan[LINK_EPCAN_MAX];

	struct {

		char		block[32];
	}
	flash[LINK_FLASH_MAX];

	char			unable_warning[LINK_MESSAGE_MAX];
	int			uptime_warning;

	int			command_state;

	int			line_N;
	int			grab_N;

	struct link_reg		reg[LINK_REGS_MAX];

	int			reg_MAX_N;
};

const char *lk_stoi(int *x, const char *s);
const char *lk_stod(double *x, const char *s);

void link_open(struct link_pmc *lp, struct config_phobia *fe,
		const char *devname, int baudrate, const char *mode);
void link_close(struct link_pmc *lp);
void link_remote(struct link_pmc *lp);

int link_fetch(struct link_pmc *lp, int clock);
void link_push(struct link_pmc *lp);
int link_command(struct link_pmc *lp, const char *command);

struct link_reg *link_reg_lookup(struct link_pmc *lp, const char *sym);
int link_reg_lookup_range(struct link_pmc *lp, const char *sym, int *min, int *max);
void link_reg_fetch_all_shown(struct link_pmc *lp);

void link_config_write(struct link_pmc *lp, const char *file);
void link_config_read(struct link_pmc *lp, const char *file);
int link_log_file_open(struct link_pmc *lp, const char *file);
int link_grab_file_open(struct link_pmc *lp, const char *file);
void link_grab_file_close(struct link_pmc *lp);

#endif /* _H_LINK_ */

