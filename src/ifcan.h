#ifndef _H_IFCAN_
#define _H_IFCAN_

enum {
	IFCAN_LOG_DISABLED		= 0,
	IFCAN_LOG_FILTERED,
	IFCAN_LOG_PROMISCUOUS
};

enum {
	IFCAN_PIPE_DISABLED		= 0,
	IFCAN_PIPE_INCOMING,
	IFCAN_PIPE_OUTGOING_REGULAR,
	IFCAN_PIPE_OUTGOING_TRIGGERED
};

enum {
	IFCAN_PAYLOAD_FLOAT		= 0,
	IFCAN_PAYLOAD_INT_16,
	IFCAN_PAYLOAD_PACKED_INT_16_0,
	IFCAN_PAYLOAD_PACKED_INT_16_1,
	IFCAN_PAYLOAD_PACKED_INT_16_2,
	IFCAN_PAYLOAD_PACKED_INT_16_3,
};

enum {
	IFCAN_PIPES_MAX		= 8,
};

typedef struct {

	int		ID;

	float		reg_DATA;
	int		reg_ID;

	int		MODE;
	int		STARTUP;
	int		tim;
	int		trigger_ID;
	int		PAYLOAD;
	float		range[2];

	int		tim_N;
	int		flag_TX;
}
ifcan_pipe_t;

typedef struct {

	int			node_ID;
	int			log_MODE;
	int			flash_MODE;

	int			startup_LOST;

	ifcan_pipe_t		pipe[IFCAN_PIPES_MAX];
}
ifcan_t;

extern ifcan_t			net;

void IFCAN_pipes_REGULAR();

int IFCAN_getc();
void IFCAN_putc(int c);

void IFCAN_startup();
void IFCAN_filter_ID();

#endif /* _H_IFCAN_ */

