#ifndef _H_EPCAN_
#define _H_EPCAN_

#define EPCAN_ID_UNMASK			(0x1FFFFE00U)

#define EPCAN_ID_OFFSET(numb)		(net.offset_ID + (numb))
#define EPCAN_ID_NODE(node, func)	(((node) << 3) | (func))

#define EPCAN_ID_CAN(node, func)	(EPCAN_ID_NODE(node, func) + EPCAN_ID_OFFSET(256U))

#define EPCAN_GET_NODE(ID)		((((ID) - EPCAN_ID_OFFSET(256U)) >> 3) & 31U)
#define EPCAN_GET_FUNC(ID)		(((ID)  - EPCAN_ID_OFFSET(256U)) & 7U)

#define EPCAN_ID_NET_SURVEY		EPCAN_ID_CAN(31U, 7U)
#define EPCAN_ID_NET_ASSIGN		EPCAN_ID_CAN(31U, 6U)

#define EPCAN_MATCH_ID_CAN		(EPCAN_ID_UNMASK | EPCAN_ID_NODE(31U, 7U))
#define EPCAN_MATCH_NET			(EPCAN_ID_UNMASK | EPCAN_ID_NODE(31U, 0U))
#define EPCAN_MATCH_FUNC		(EPCAN_ID_UNMASK | EPCAN_ID_NODE(0U,  7U))

#define EPCAN_OFFSET_DEFAULT		1024U
#define EPCAN_TLM_ID_DEFAULT		192

enum {
	EPCAN_NODE_REQ			= 0,
	EPCAN_NODE_ACK,
	EPCAN_NODE_RX,				/* receive from remote node */
	EPCAN_NODE_TX				/* send to remote node */
};

enum {
	EPCAN_REQ_NOTHING		= 0,
	EPCAN_REQ_FLOW_TX_PAUSE,		/* node flow control */
	EPCAN_REQ_REG_GET,			/* request to GET register */
	EPCAN_REQ_REG_SET			/* request to SET register */
};

enum {
	EPCAN_ACK_NOTHING		= 0,
	EPCAN_ACK_NETWORK_REPLY,		/* reply to network survey */
	EPCAN_ACK_REG_DATA			/* reply to the GET request */
};

enum {
	EPCAN_LOG_DISABLED		= 0,
	EPCAN_LOG_FILTERED,
	EPCAN_LOG_PROMISCUOUS
};

enum {
	EPCAN_PIPE_DISABLED		= 0,
	EPCAN_PIPE_INCOMING,
	EPCAN_PIPE_OUTGOING_REGULAR,
	EPCAN_PIPE_OUTGOING_INJECTED
};

enum {
	EPCAN_PAYLOAD_FLOAT		= 0,
	EPCAN_PAYLOAD_INT_16
};

enum {
	EPCAN_EP_MAX			= 4
};

typedef struct {

	int		MODE;		/* operation mode of endpoint (EP) */
	int		ID;		/* ID of the endpoint (EP) */
	int		inject_ID;	/* injected EP ID */

	float		reg_DATA;	/* actual DATA */
	int		reg_ID;		/* linked register ID */

	int		PAYLOAD;	/* packet payload type */
	int		STARTUP;	/* motor startup behaviour */
	int		ACTIVE;

	int		rate;		/* transfer rate */
	float		range[2];	/* natural data range */

	int		tx_clock;
	int		tx_flag;
}
epcan_pipe_t;

typedef struct {

	uint32_t		offset_ID;	/* CAN ID network offset */
	int			node_ID;	/* ID of this node */

	int			log_MSG;
	int			timeout_EP;

	int			tlm_ID;		/* EP ID of telemetry */

	epcan_pipe_t		ep[EPCAN_EP_MAX];
}
epcan_t;

extern epcan_t			net;

void EPCAN_pipe_REGULAR();

void EPCAN_send_msg(CAN_msg_t *msg);
void EPCAN_putc(int c);

void EPCAN_startup();
void EPCAN_bind();

#endif /* _H_EPCAN_ */

