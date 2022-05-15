#ifndef _H_LINK_
#define _H_LINK_

#define LINK_REGS_MAX		900
#define LINK_COMBO_MAX		40

enum {
	LINK_REG_CONFIG		= 1U,
	LINK_REG_READ_ONLY	= 2U,
	LINK_REG_LINKED		= 4U,

	LINK_REG_TYPE_INT	= (1UL << 8),
	LINK_REG_TYPE_FLOAT	= (1UL << 9),
};

struct link_priv;

struct link_reg {

	int		mode;

	int		shown;
	int		modified;
	int		fetched;
	int		queued;

	char		sym[80];
	char		val[80];
	char		um[80];

	int		lval;
	float		fval;

	float		fmin;
	float		fmax;

	int		started;
	int		update;

	char		*combo[LINK_COMBO_MAX];
	int		lmax_combo;

	int		um_sel;
};

struct link_pmc {

	char			devname[80];
	int			baudrate;

	int			linked;
	int			fetched_N;

	int			clock;
	int			locked;

	char			hwinfo[80];
	char			network[40];

	const char		*tlm_file;

	char			flash_map[8][20];
	int			flash_errno;

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

#endif /* _H_LINK_ */

