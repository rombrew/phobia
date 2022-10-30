#ifndef _H_LINK_
#define _H_LINK_

#define LINK_REGS_MAX		900
#define LINK_NAME_MAX		80
#define LINK_MESSAGE_MAX	220
#define LINK_COMBO_MAX		40
#define LINK_FLASH_MAX		512

enum {
	LINK_REG_CONFIG		= 1U,
	LINK_REG_READ_ONLY	= 2U,
	LINK_REG_LINKED		= 4U,

	LINK_REG_TYPE_INT	= (1UL << 8),
	LINK_REG_TYPE_FLOAT	= (1UL << 9),
};

enum {
	LINK_GRAB_WAIT		= 0,
	LINK_GRAB_TRANSFER,
	LINK_GRAB_FINISHED
};

enum {
	LINK_PUSH_WAIT		= 0,
	LINK_PUSH_TRANSFER,
	LINK_PUSH_QUEUED,
	LINK_PUSH_FINISHED
};

struct link_priv;

struct link_reg {

	int		mode;

	int		shown;
	int		modified;
	int		fetched;
	int		queued;

	char		sym[LINK_NAME_MAX];
	char		val[LINK_NAME_MAX];
	char		um[LINK_NAME_MAX];

	int		lval;
	float		fval;

	float		fmin;
	float		fmax;

	int		started;
	int		update;
	int		onefetch;

	char		*combo[LINK_COMBO_MAX];
	int		lmax_combo;

	int		um_sel;
};

struct link_pmc {

	char			devname[LINK_NAME_MAX];
	int			baudrate;
	int			quantum;

	int			linked;
	int			fetched_N;

	int			clock;
	int			locked;
	int			grabed;

	char			hwinfo[LINK_NAME_MAX];
	char			network[LINK_NAME_MAX];

	char			flash_info_map[LINK_FLASH_MAX];
	char			unable_warning[LINK_MESSAGE_MAX];

	int			grab_mode;
	int			grab_fetched_N;

	int			push_mode;
	int			push_queued_N;

	struct link_priv	*priv;
	struct link_reg		reg[LINK_REGS_MAX];
};

const char *lk_stoi(int *x, const char *s);
const char *lk_stod(double *x, const char *s);

void link_open(struct link_pmc *lp, const char *devname, int baudrate);
void link_close(struct link_pmc *lp);

int link_fetch(struct link_pmc *lp, int clock);
void link_push(struct link_pmc *lp);
int link_command(struct link_pmc *lp, const char *command);

struct link_reg *link_reg_lookup(struct link_pmc *lp, const char *sym);
int link_reg_lookup_range(struct link_pmc *lp, const char *sym, int *min, int *max);
void link_reg_fetch_all_shown(struct link_pmc *lp);

int link_grab_file(struct link_pmc *lp, const char *file);
int link_push_file(struct link_pmc *lp, const char *file);
int link_log_file(struct link_pmc *lp, const char *file);

#endif /* _H_LINK_ */

