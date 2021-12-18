#ifndef _H_IFCAN_
#define _H_IFCAN_

#define IFCAN_ID(node_ID, func)		(1024U | (node_ID) << 5 | (func))

#define IFCAN_GET_NODE(ID)		(((ID) >> 5) & 31U)
#define IFCAN_GET_FUNC(ID)		((ID) & 31U)

/* Network functions.
 * */
#define IFCAN_ID_NET_SURVEY		IFCAN_ID(31U, 31U)
#define IFCAN_ID_NET_ASSIGN		IFCAN_ID(31U, 29U)

/* Flash functions.
 * */
#define IFCAN_ID_FLASH_REVISION1	IFCAN_ID(31U, 27U)
#define IFCAN_ID_FLASH_REVISION2	IFCAN_ID(31U, 26U)
#define IFCAN_ID_FLASH_INIT		IFCAN_ID(31U, 25U)
#define IFCAN_ID_FLASH_DATA		IFCAN_ID(31U, 24U)

/* Lower IDs is for DATA streaming.
 * */
#define	IFCAN_ID_NODE_BASE		IFCAN_ID(0U, 0U)

#define IFCAN_FILTER_MATCH		IFCAN_ID(31U, 31U)
#define IFCAN_FILTER_NETWORK		IFCAN_ID(31U, 0U)

/* Maximal number of nodes in network.
 * */
#define IFCAN_NODES_MAX			30

enum {
	/* Node functions (offset).
	 * */
	IFCAN_NODE_REQ			= 0,
	IFCAN_NODE_ACK,
	IFCAN_NODE_RX,
	IFCAN_NODE_TX,
};

enum {
	IFCAN_REQ_NOTHING		= 0,

	/* Node flow control.
	 * */
	IFCAN_REQ_FLOW_TX_PAUSE,
};

enum {
	IFCAN_ACK_NOTHING		= 0,

	/* Network ACKs.
	 * */
	IFCAN_ACK_NETWORK_REPLY,
	IFCAN_ACK_NETWORK_RESERVED1,
	IFCAN_ACK_NETWORK_RESERVED2,

	/* Flash ACKs.
	 * */
	IFCAN_ACK_FLASH_UP_TO_DATE,
	IFCAN_ACK_FLASH_REJECT,
	IFCAN_ACK_FLASH_WAIT_FOR_ERASE,
	IFCAN_ACK_FLASH_DATA_ACCEPT,
	IFCAN_ACK_FLASH_DATA_PAUSE,
	IFCAN_ACK_FLASH_CRC32_INVALID,
	IFCAN_ACK_FLASH_SELFUPDATE_DONE,
};

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

