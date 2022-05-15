#ifndef _H_CONFIG_
#define _H_CONFIG_

struct config_pmcfe {

	char			rcfile[250];

	int			windowsize;
	char			path[250];
};

void config_open(struct config_pmcfe *fe);
void config_write(struct config_pmcfe *fe);
void config_default(struct config_pmcfe *fe);

#endif /* _H_CONFIG_ */

