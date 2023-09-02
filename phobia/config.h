#ifndef _H_CONFIG_
#define _H_CONFIG_

#include <stdlib.h>
#include <stdio.h>

#define CONFIG_VERSION		4

#define FILE_HOME_CONFIG	"phobia"
#define FILE_LINK_LOG		"phobia.log"
#define FILE_TELEMETRY_GRAB	"tlmgrab.csv"
#define FILE_TELEMETRY_LOG	"tlmlog.csv"
#define FILE_TELEMETRY_FILTER	".csv"
#define FILE_CONFIG_DEFAULT	"config.txt"
#define FILE_CONFIG_FILTER	".txt"

#define PHOBIA_PATH_MAX		400
#define PHOBIA_NAME_MAX		80

struct config_phobia {

	int			version;

	char			rcfile[PHOBIA_PATH_MAX];
	int			local;

	char			serialport[PHOBIA_NAME_MAX];
	int			baudrate;
	int			parity;

	int			windowsize;
	char			storage[PHOBIA_PATH_MAX];
	char			fuzzy[PHOBIA_NAME_MAX];
	int			lograte;
	char			lograte_lbuf[PHOBIA_NAME_MAX];
	int			regfile;
};

FILE *fopen_from_UTF8(const char *file, const char *mode);

void config_open(struct config_phobia *fe);
void config_write(struct config_phobia *fe);
void config_default(struct config_phobia *fe);

#endif /* _H_CONFIG_ */

