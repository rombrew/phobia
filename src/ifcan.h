#ifndef _H_IFCAN_
#define _H_IFCAN_

#include "libc.h"

enum {
	IFCAN_LOG_DISABLED		= 0,
	IFCAN_LOG_FILTERED,
	IFCAN_LOG_PROMISCUOUS
};

enum {
	IFCAN_STREAMS_MAX		= 8,
};

typedef struct {

	int			node_ID;
	int			log_MODE;

	struct {

		/* TODO */
		float		reg_VAL;
		int		reg_ID;
		float		reg_range[2];
		int		f_func;
	}
	stream[IFCAN_STREAMS_MAX];
}
ifcan_t;

extern ifcan_t			can;

int IFCAN_getc();
void IFCAN_putc(int c);

void IFCAN_startup();
void IFCAN_filter_ID();

#endif /* _H_IFCAN_ */

