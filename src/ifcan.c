#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "ifcan.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

enum {
	/* Network functions.
	 * */
	IFCAN_ID_NET_SURVEY		= 2047,
	IFCAN_ID_NET_REPLY		= 2046,
	IFCAN_ID_NET_ASSIGN		= 2045,

	/* Flash functions.
	 * */
	IFCAN_ID_FLASH_INIT		= 2044,
	IFCAN_ID_FLASH_DATA		= 2043,
	IFCAN_ID_FLASH_ACK		= 2042,

	/* Node functions (offset).
	 * */
	IFCAN_ID_NODE_CTL		= 0,
	IFCAN_ID_NODE_RX		= 1,
	IFCAN_ID_NODE_TX		= 2,
	IFCAN_ID_NODE_END_OF		= 32,

	/* Lower IDs is for DATA streaming.
	 * */
	IFCAN_ID_NODE_BASE		= 1056,
};

enum {
	IFCAN_ACK_NOTHING		= 0,

	/* Flash ACKs.
	 * */
	IFCAN_ACK_FLASH_UP_TO_DATE,
	IFCAN_ACK_FLASH_WAIT_FOR_ERASE,
	IFCAN_ACK_FLASH_INIT_FAILURE,
	IFCAN_ACK_FLASH_DATA_ACCEPT,
	IFCAN_ACK_FLASH_DATA_PAUSE,
	IFCAN_ACK_FLASH_CRC32_INVALID,
	IFCAN_ACK_FLASH_SELFUPDATE,
};

enum {
	IFCAN_NODE_CTL_NOTHING		= 0,

	/* Node control.
	 * */
	IFCAN_NODE_CTL_FLOW_TX_PAUSE,
};

enum {
	IFCAN_FILTER_MATCH		= 2047,
	IFCAN_FILTER_NETWORK		= 2016,

	/* Maximal number of nodes.
	 * */
	IFCAN_NODES_MAX			= 30,
};

typedef struct {

	u32_t			UID;

	/* Input CAN messages.
	 * */
	QueueHandle_t		queue_IN;

	/* Serial IO.
	 * */
	QueueHandle_t		queue_RX;
	QueueHandle_t		queue_TX;
	QueueHandle_t		queue_EX;

	/* Serial LOG.
	 * */
	QueueHandle_t		queue_LOG;
	int			log_skipped;

	/* FLASH update across network.
	 * */
	u32_t			flash_INIT_sizeof;
	u32_t			flash_INIT_crc32;
	u32_t			*flash_long;
	u32_t			flash_DATA_accepted;
	int			flash_DATA_paused;

	/* Remote node ID we connected to.
	 * */
	int			remote_node_ID;

	/* TX pause notice (flow control).
	 * */
	int			flow_TX_pause;

	/* Remote nodes TABLE.
	 * */
	struct {

		u32_t		UID;

		u16_t		node_ID;
		u16_t		node_ACK;
	}
	node[IFCAN_NODES_MAX];
}
ifcan_local_t;

ifcan_t				can;
static ifcan_local_t		local;

static void
IFCAN_pipe_INCOMING(ifcan_pipe_t *pp, const CAN_msg_t *msg)
{
	float			lerp;

	switch (pp->PAYLOAD) {

		case IFCAN_PAYLOAD_FLOAT:

			if (msg->len == 4) {

				pp->reg_DATA = * (float *) &msg->payload[0];
			}
			break;

		case IFCAN_PAYLOAD_INT_16:

			if (msg->len == 2) {

				lerp = (float) * (u16_t *) &msg->payload[0] * (1.f / 65535.f);

				pp->reg_DATA = pp->range[0] + lerp * (pp->range[1] - pp->range[0]);
			}
			break;

		case IFCAN_PAYLOAD_INT_32:

			if (msg->len == 4) {

				lerp = (float) * (u32_t *) &msg->payload[0] * (1.f / 4294967295.f);

				pp->reg_DATA = pp->range[0] + lerp * (pp->range[1] - pp->range[0]);
			}
			break;

		default: break;
	}

	if (pp->reg_ID != ID_NULL) {

		reg_SET_F(pp->reg_ID, pp->reg_DATA);
	}
}

static void
IFCAN_pipe_OUTGOING(ifcan_pipe_t *pp)
{
	CAN_msg_t		msg;
	float			lerp;

	if (pp->reg_ID != ID_NULL) {

		pp->reg_DATA = reg_GET_F(pp->reg_ID);
	}

	msg.ID = pp->ID;

	switch (pp->PAYLOAD) {

		case IFCAN_PAYLOAD_FLOAT:

			msg.len = 4;
			* (float *) &msg.payload[0] = pp->reg_DATA;
			break;

		case IFCAN_PAYLOAD_INT_16:

			msg.len = 2;

			lerp = (pp->reg_DATA - pp->range[0]) / (pp->range[1] - pp->range[0]);
			lerp = (lerp < 0.f) ? 0.f : (lerp > 1.f) ? 1.f : lerp;

			* (u16_t *) &msg.payload[0] = (u16_t) (lerp * 65535.f);
			break;

		case IFCAN_PAYLOAD_INT_32:

			msg.len = 4;

			lerp = (pp->reg_DATA - pp->range[0]) / (pp->range[1] - pp->range[0]);
			lerp = (lerp < 0.f) ? 0.f : (lerp > 1.f) ? 1.f : lerp;

			* (u32_t *) &msg.payload[0] = (u32_t) (lerp * 4294967295.f);
			break;

		default: break;
	}

	if (CAN_send_msg(&msg) == CAN_TX_OK) {

		pp->flag_TX = 0;
	}
}

static void
IFCAN_pipes_message_IN(const CAN_msg_t *msg)
{
	ifcan_pipe_t		*pp;
	int			N;

	for (N = 0; N < IFCAN_PIPES_MAX; ++N) {

		pp = &can.pipe[N];

		if (pp->MODE == IFCAN_PIPE_INCOMING
				&& pp->ID == msg->ID) {

			pp->tim_N = 0;

			if (		pp->STARTUP == PM_ENABLED
					&& pm.lu_mode == PM_LU_DISABLED) {

				pm.fsm_req = PM_STATE_LU_STARTUP;
			}

			IFCAN_pipe_INCOMING(pp, msg);
		}
		else if (pp->MODE == IFCAN_PIPE_OUTGOING_TRIGGERED
				&& pp->trigger_ID == msg->ID) {

			pp->flag_TX = 1;

			IFCAN_pipe_OUTGOING(pp);
		}
	}
}

void IFCAN_pipes_REGULAR()
{
	ifcan_pipe_t		*pp;
	int			N;

	for (N = 0; N < IFCAN_PIPES_MAX; ++N) {

		pp = &can.pipe[N];

		if (pp->MODE == IFCAN_PIPE_INCOMING
				&& pp->STARTUP == PM_ENABLED
				&& pm.lu_mode != PM_LU_DISABLED) {

			pp->tim_N++;

			if (pp->tim_N >= can.startup_LOST) {

				pp->tim_N = 0;
				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}
		}
		else if (pp->MODE == IFCAN_PIPE_OUTGOING_REGULAR) {

			pp->tim_N++;

			if (pp->tim_N >= pp->tim) {

				pp->tim_N = 0;

				if (		pp->STARTUP != PM_ENABLED
						|| pm.lu_mode != PM_LU_DISABLED) {

					pp->flag_TX = 1;
				}
			}

			if (pp->flag_TX != 0) {

				IFCAN_pipe_OUTGOING(pp);
			}
		}
		else if (pp->MODE == IFCAN_PIPE_OUTGOING_TRIGGERED
				&& pp->flag_TX != 0) {

			IFCAN_pipe_OUTGOING(pp);
		}
	}
}

void CAN_IRQ()
{
	BaseType_t		xWoken = pdFALSE;

	if (hal.CAN_msg.ID < IFCAN_ID_NODE_BASE) {

		IFCAN_pipes_message_IN(&hal.CAN_msg);
	}

	if (hal.CAN_msg.ID >= IFCAN_ID_NODE_BASE
			|| can.log_MODE == IFCAN_LOG_PROMISCUOUS) {

		xQueueSendToBackFromISR(local.queue_IN, &hal.CAN_msg, &xWoken);
	}

	portYIELD_FROM_ISR(xWoken);
}

static void
local_node_discard()
{
	int			N;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		local.node[N].UID = 0;
		local.node[N].node_ID = 0;
	}
}

static void
local_node_discard_ACK()
{
	int			N;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		local.node[N].node_ACK = 0;
	}
}

static void
local_node_insert(u32_t UID, int node_ID)
{
	int			N, found = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].UID == UID) {

			found = 1;

			/* Update node ID.
			 * */
			local.node[N].node_ID = node_ID;
			break;
		}
	}

	if (found == 0) {

		for (N = 0; N < IFCAN_NODES_MAX; ++N) {

			if (local.node[N].UID == 0) {

				/* Insert NODE.
				 * */
				local.node[N].UID = UID;
				local.node[N].node_ID = node_ID;
				break;
			}
		}
	}
}

static int
local_nodes_N()
{
	int			N, nodes_N = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].node_ID >= IFCAN_ID_NODE_BASE) {

			nodes_N += 1;
		}
	}

	return nodes_N;
}

static int
local_nodes_by_ACK(int ack)
{
	int		N, nodes_N = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].node_ID >= IFCAN_ID_NODE_BASE
				&& local.node[N].node_ACK == ack) {

			nodes_N += 1;
		}
	}

	return nodes_N;
}

static void
local_node_update_ACK(int node_ID, int ack)
{
	int			N;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].node_ID == node_ID) {

			/* Update node ACK.
			 * */
			local.node[N].node_ACK = ack;
			break;
		}
	}
}

static int
local_node_is_valid_remote(int node_ID)
{
	int		N, rc = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (node_ID == local.node[N].node_ID) {

			rc = 1;
			break;
		}
	}

	if (node_ID == can.node_ID) {

		rc = 0;
	}

	return rc;
}

static int
local_node_assign_ID()
{
	int		N, node_ID, found_ID = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		node_ID = IFCAN_ID_NODE_BASE + N * IFCAN_ID_NODE_END_OF;

		if (local_node_is_valid_remote(node_ID) == 0
				&& node_ID != can.node_ID) {

			found_ID = node_ID;
			break;
		}
	}

	return found_ID;
}

void task_IFCAN_LOG(void *pData)
{
	char			xC;

	do {
		while (xQueueReceive(local.queue_LOG, &xC, portMAX_DELAY) == pdTRUE) {

			/* We need a decoupling queue to output the LOG as not
			 * to block regular messages processing. If we block
			 * here that some part of LOG may be lost.
			 * */
			putc(xC);
		}
	}
	while (1);
}

void IFCAN_log_putc(int c)
{
	char		xC = (char) c;

	xQueueSendToBack(local.queue_LOG, &xC, (TickType_t) 0);
}

static void
IFCAN_log_msg(CAN_msg_t *msg, const char *sym)
{
	int		N;

	io_ops_t	ops = {

		.getc = NULL,
		.putc = &IFCAN_log_putc
	};

	if (uxQueueSpacesAvailable(local.queue_LOG) >= 60) {

		if (local.log_skipped != 0) {

			xprintf(&ops, "(skipped %i)" EOL, local.log_skipped);

			local.log_skipped = 0;
		}

		xprintf(&ops, "%s %i %i", sym, msg->ID, msg->len);

		for (N = 0; N < msg->len; ++N) {

			xprintf(&ops, " %2x", msg->payload[N]);
		}

		xputs(&ops, EOL);
	}
	else {
		local.log_skipped += 1;
	}
}

static int
IFCAN_send_msg(CAN_msg_t *msg)
{
	int			rc, N = 0;

	do {
		rc = CAN_send_msg(msg);

		if (rc == CAN_TX_OK) {

			if (can.log_MODE != IFCAN_LOG_DISABLED) {

				IFCAN_log_msg(msg, "TX");
			}

			break;
		}

		N++;

		if (N >= 20) {

			break;
		}

		/* Wait until one of TX mailboxes is free.
		 * */
		hal_delay_ns(50000);
	}
	while (1);

	return rc;
}

static void
IFCAN_net_REPLY()
{
	CAN_msg_t		msg;

	msg.ID = IFCAN_ID_NET_REPLY;
	msg.len = 6;

	* (u32_t *) &msg.payload[0] = local.UID;
	* (u16_t *) &msg.payload[4] = can.node_ID;

	vTaskDelay((TickType_t) 1);

	IFCAN_send_msg(&msg);
}

static int
IFCAN_flash_is_up_to_date()
{
	u32_t			*flash = (u32_t *) &ld_begin_vectors;
	u32_t			flash_sizeof;

	flash_sizeof = * (flash + 8) - * (flash + 7);

	return (flash_sizeof == local.flash_INIT_sizeof
			&& crc32b(flash, flash_sizeof)
			== local.flash_INIT_crc32) ? 1 : 0;
}

static int
IFCAN_flash_erase()
{
	void			*flash_end;
	int			N, rc = 0;

	if (local.flash_INIT_sizeof >= 1024UL) {

		flash_end = (void *) (flash_ram_map[0] + local.flash_INIT_sizeof);

		if ((u32_t) flash_end > flash_ram_map[FLASH_SECTOR_MAX - 2]) {

			rc = 0;
		}
		else {
			/* Relocate configuration block.
			 * */
			rc = flash_block_relocate();

			if (rc != 0) {

				for (N = 0; N < FLASH_SECTOR_MAX - 2; ++N) {

					if ((u32_t) flash_end > flash_ram_map[N]) {

						FLASH_erase((void *) flash_ram_map[N]);
					}
				}
			}
		}
	}

	return rc;
}

static void
IFCAN_flash_ACK(int ack)
{
	CAN_msg_t		msg;

	msg.ID = IFCAN_ID_FLASH_ACK;
	msg.len = 4;

	* (u16_t *) &msg.payload[0] = can.node_ID;
	* (u16_t *) &msg.payload[2] = ack;

	IFCAN_send_msg(&msg);
}

static void
IFCAN_remote_node_CTL(int ctl)
{
	CAN_msg_t		msg;

	msg.ID = local.remote_node_ID + IFCAN_ID_NODE_CTL;
	msg.len = 2;

	* (u16_t *) &msg.payload[0] = ctl;

	IFCAN_send_msg(&msg);
}

static void
IFCAN_message_IN(const CAN_msg_t *msg)
{
	int			N;

	/* Network functions.
	 * */
	if (msg->ID == IFCAN_ID_NET_SURVEY) {

		IFCAN_net_REPLY();
	}
	else if (msg->ID == IFCAN_ID_NET_REPLY
			&& msg->len == 6) {

		local_node_insert(* (u32_t *) &msg->payload[0],
				* (u16_t *) &msg->payload[4]);
	}
	else if (msg->ID == IFCAN_ID_NET_ASSIGN
			&& msg->len == 6) {

		if (* (u32_t *) &msg->payload[0] == local.UID) {

			/* Accept new node ID.
			 * */
			can.node_ID = * (u16_t *) &msg->payload[4];

			IFCAN_filter_ID();
		}
	}

	/* Flash functions.
	 * */
	else if (msg->ID == IFCAN_ID_FLASH_INIT
			&& msg->len == 8
			&& can.node_ID >= IFCAN_ID_NODE_BASE
			&& pm.lu_mode == PM_LU_DISABLED) {

		local.flash_INIT_sizeof = * (u32_t *) &msg->payload[0];
		local.flash_INIT_crc32 = * (u32_t *) &msg->payload[4];

		if (IFCAN_flash_is_up_to_date() != 0) {

			IFCAN_flash_ACK(IFCAN_ACK_FLASH_UP_TO_DATE);
		}
		else {
			IFCAN_flash_ACK(IFCAN_ACK_FLASH_WAIT_FOR_ERASE);

			if (IFCAN_flash_erase() != 0) {

				local.flash_long = (u32_t *) flash_ram_map[0];
				local.flash_DATA_accepted = 0;
				local.flash_DATA_paused = 0;

				IFCAN_flash_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT);
			}
			else {
				IFCAN_flash_ACK(IFCAN_ACK_FLASH_INIT_FAILURE);
			}
		}
	}
	else if (msg->ID == IFCAN_ID_FLASH_DATA
			&& msg->len == 8
			&& can.node_ID >= IFCAN_ID_NODE_BASE
			&& pm.lu_mode == PM_LU_DISABLED
			&& (u32_t) local.flash_long >= flash_ram_map[0]) {

		FLASH_prog(local.flash_long, &msg->payload[0], msg->len);

		local.flash_long += 2UL;
		local.flash_DATA_accepted += msg->len;

		if (local.flash_DATA_paused == 0) {

			if (uxQueueSpacesAvailable(local.queue_IN) < 5) {

				IFCAN_flash_ACK(IFCAN_ACK_FLASH_DATA_PAUSE);
				local.flash_DATA_paused = 1;
			}
		}
		else {
			if (uxQueueMessagesWaiting(local.queue_IN) < 5) {

				IFCAN_flash_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT);
				local.flash_DATA_paused = 0;
			}
		}

		if (local.flash_DATA_accepted >= local.flash_INIT_sizeof) {

			if (crc32b((u32_t *) flash_ram_map[0], local.flash_INIT_sizeof)
					== local.flash_INIT_crc32) {

				IFCAN_flash_ACK(IFCAN_ACK_FLASH_SELFUPDATE);

				GPIO_set_LOW(GPIO_BOOST_12V);
				GPIO_set_HIGH(GPIO_LED);

				/* Go into the dark.
				 * */
				FLASH_selfupdate();
			}
			else {
				IFCAN_flash_ACK(IFCAN_ACK_FLASH_CRC32_INVALID);
			}

			local.flash_long = NULL;
		}
	}
	else if (msg->ID == IFCAN_ID_FLASH_ACK
			&& msg->len == 4
			&& can.node_ID >= IFCAN_ID_NODE_BASE
			&& pm.lu_mode == PM_LU_DISABLED) {

		local_node_update_ACK(* (u16_t *) &msg->payload[0],
				* (u16_t *) &msg->payload[2]);
	}

	/* Node functions.
	 * */
	else if (msg->ID == can.node_ID + IFCAN_ID_NODE_CTL) {

		if (* (u16_t *) &msg->payload[0] == IFCAN_NODE_CTL_FLOW_TX_PAUSE
				&& msg->len == 2) {

			local.flow_TX_pause = 1;
		}
	}
	else if (msg->ID == can.node_ID + IFCAN_ID_NODE_RX) {

		for (N = 0; N < msg->len; ++N) {

			xQueueSendToBack(local.queue_RX, &msg->payload[N], (TickType_t) 0);
		}

		IODEF_TO_CAN();
	}
	else if (msg->ID == local.remote_node_ID + IFCAN_ID_NODE_TX) {

		for (N = 0; N < msg->len; ++N) {

			/* Remote NODE output via LOG.
			 * */
			IFCAN_log_putc(msg->payload[N]);
		}

		if (uxQueueSpacesAvailable(local.queue_LOG) < 20) {

			/* Notify remote node about overflow.
			 * */
			IFCAN_remote_node_CTL(IFCAN_NODE_CTL_FLOW_TX_PAUSE);
		}
	}
}

void task_IFCAN_IN(void *pData)
{
	CAN_msg_t		msg;

	do {
		while (xQueueReceive(local.queue_IN, &msg, portMAX_DELAY) == pdTRUE) {

			if (can.log_MODE != IFCAN_LOG_DISABLED) {

				IFCAN_log_msg(&msg, "IN");
			}

			IFCAN_message_IN(&msg);
		}
	}
	while (1);
}

void task_IFCAN_TX(void *pData)
{
	CAN_msg_t		msg;
	char			xC;

	do {
		msg.len = 0;

		while (xQueueReceive(local.queue_TX, &xC, (TickType_t) 10) == pdTRUE) {

			msg.payload[msg.len++] = xC;

			if (msg.len >= 8) {

				break;
			}
		}

		if (msg.len != 0) {

			msg.ID = can.node_ID + IFCAN_ID_NODE_TX;

			IFCAN_send_msg(&msg);

			/* Do not send messages too frequently especially if
			 * you were asked to.
			 * */
			if (local.flow_TX_pause != 0) {

				vTaskDelay((TickType_t) 10);

				local.flow_TX_pause = 0;
			}
			else {
				vTaskDelay((TickType_t) 1);
			}
		}
	}
	while (1);
}

int IFCAN_getc()
{
	char		xC;

	xQueueReceive(local.queue_RX, &xC, portMAX_DELAY);

	return (int) xC;
}

void IFCAN_putc(int c)
{
	char		xC = (char) c;

	GPIO_set_HIGH(GPIO_LED);

	xQueueSendToBack(local.queue_TX, &xC, portMAX_DELAY);

	GPIO_set_LOW(GPIO_LED);
}

extern QueueHandle_t USART_queue_RX();

void IFCAN_startup()
{
	/* Produce UNIQUE ID.
	 * */
	local.UID = crc32b((void *) 0x1FFF7A10, 12);

	/* Allocate queues.
	 * */
	local.queue_IN = xQueueCreate(30, sizeof(CAN_msg_t));
	local.queue_RX = USART_queue_RX();
	local.queue_TX = xQueueCreate(80, sizeof(char));
	local.queue_EX = xQueueCreate(40, sizeof(char));
	local.queue_LOG = xQueueCreate(400, sizeof(char));

	/* Create IFCAN tasks.
	 * */
	xTaskCreate(task_IFCAN_IN, "IFCAN_IN", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_IFCAN_TX, "IFCAN_TX", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_IFCAN_LOG, "IFCAN_LOG", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	CAN_startup();

	IFCAN_filter_ID();
}

void IFCAN_filter_ID()
{
	int			N;

	if (can.log_MODE == IFCAN_LOG_PROMISCUOUS) {

		CAN_filter_ID(0, 0, 1, 0);
	}
	else {
		CAN_filter_ID(0, 0, IFCAN_FILTER_NETWORK, IFCAN_FILTER_NETWORK);
	}

	if (can.node_ID >= IFCAN_ID_NODE_BASE) {

		CAN_filter_ID(1, 0, can.node_ID + IFCAN_ID_NODE_CTL, IFCAN_FILTER_MATCH);
		CAN_filter_ID(2, 0, can.node_ID + IFCAN_ID_NODE_RX, IFCAN_FILTER_MATCH);
		CAN_filter_ID(3, 0, 0, 0);
	}
	else {
		CAN_filter_ID(1, 0, 0, 0);
		CAN_filter_ID(2, 0, 0, 0);
		CAN_filter_ID(3, 0, 0, 0);
	}

	for (N = 0; N < IFCAN_PIPES_MAX; ++N) {

		if (can.pipe[N].MODE == IFCAN_PIPE_INCOMING) {

			CAN_filter_ID(20 + N, 1, can.pipe[N].ID, IFCAN_FILTER_MATCH);
		}
		else if (can.pipe[N].MODE == IFCAN_PIPE_OUTGOING_TRIGGERED) {

			CAN_filter_ID(20 + N, 1, can.pipe[N].trigger_ID, IFCAN_FILTER_MATCH);
		}
		else {
			CAN_filter_ID(20 + N, 1, 0, 0);
		}
	}
}

SH_DEF(can_net_survey)
{
	CAN_msg_t		msg;
	int			N;

	local_node_discard();

	msg.ID = IFCAN_ID_NET_SURVEY;
	msg.len = 0;

	IFCAN_send_msg(&msg);

	/* Wait for all nodes to reply.
	 * */
	vTaskDelay((TickType_t) 50);

	if (local.node[0].UID != 0) {

		printf("UID         node_ID" EOL);

		for (N = 0; N < IFCAN_NODES_MAX; ++N) {

			if (local.node[N].UID != 0) {

				printf("%8x    ", local.node[N].UID);

				if (local.node[N].node_ID >= IFCAN_ID_NODE_BASE) {

					printf("%4i    ", local.node[N].node_ID);
				}
				else {
					printf("(not assigned)");
				}

				puts(EOL);
			}
		}
	}
	else {
		printf("No remote nodes found" EOL);
	}
}

SH_DEF(can_net_assign)
{
	CAN_msg_t		msg;
	int			N;

	if (can.node_ID == 0) {

		can.node_ID = local_node_assign_ID();

		IFCAN_filter_ID();

		printf("local assigned to %i" EOL, can.node_ID);
	}

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID == 0) {

			local.node[N].node_ID = local_node_assign_ID();

			if (local.node[N].node_ID != 0) {

				msg.ID = IFCAN_ID_NET_ASSIGN;
				msg.len = 6;

				* (u32_t *) &msg.payload[0] = local.node[N].UID;
				* (u16_t *) &msg.payload[4] = local.node[N].node_ID;

				IFCAN_send_msg(&msg);

				printf("UID %8x assigned to %i" EOL,
						local.node[N].UID,
						local.node[N].node_ID);
			}
		}
	}
}

SH_DEF(can_net_revoke)
{
	CAN_msg_t		msg;
	int			N, node_ID;

	if (stoi(&node_ID, s) == NULL) {

		return ;
	}

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID != 0) {

			if (local.node[N].node_ID == node_ID
					|| node_ID == 0) {

				local.node[N].node_ID = 0;

				msg.ID = IFCAN_ID_NET_ASSIGN;
				msg.len = 6;

				* (u32_t *) &msg.payload[0] = local.node[N].UID;
				* (u16_t *) &msg.payload[4] = 0;

				IFCAN_send_msg(&msg);

				printf("UID %8x revoked" EOL, local.node[N].UID);
			}
		}
	}
}

static void
IFCAN_flash_DATA(u32_t *flash, u32_t flash_sizeof)
{
	u32_t			*flash_end;
	CAN_msg_t		msg;
	int			rc, N, blk_N, fail_N;

	msg.ID = IFCAN_ID_FLASH_DATA;
	msg.len = 8;

	flash_end = (u32_t *) ((u32_t) flash + flash_sizeof);

	blk_N = 0;
	fail_N = 0;

	puts("Flash ");

	while (flash < flash_end) {

		N = 0;

		/* Note that bandwidth is limited because of it could take a
		 * time to program flash on remote side. So we should check if
		 * remote side signals us to pause transmission.
		 * */
		while (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_PAUSE) != 0) {

			N++;

			if (N >= 5) {

				local_node_discard_ACK();
				break;
			}

			vTaskDelay((TickType_t) 1);
		}

		* (u32_t *) &msg.payload[0] = *(flash + 0);
		* (u32_t *) &msg.payload[4] = *(flash + 1);

		rc = IFCAN_send_msg(&msg);

		if (rc == CAN_TX_OK) {

			flash += 2UL;
			blk_N += 8;

			if (blk_N >= 8192) {

				blk_N = 0;

				/* Display uploading progress.
				 * */
				putc('.');
			}
		}
		else {
			fail_N += 8;

			if (fail_N >= 8192) {

				puts(" Fail");
				break;
			}
		}
	}

	puts(EOL);
}

SH_DEF(can_flash_update)
{
	CAN_msg_t		msg;
	u32_t			*flash, flash_sizeof, flash_crc32;
	int			N, nodes_N;

	local_node_discard();

	msg.ID = IFCAN_ID_NET_SURVEY;
	msg.len = 0;

	IFCAN_send_msg(&msg);

	/* Wait for all nodes to reply.
	 * */
	vTaskDelay((TickType_t) 50);

	nodes_N = local_nodes_N();

	if (nodes_N == 0) {

		printf("No remote nodes were found" EOL);
	}
	else {
		local_node_discard_ACK();

		flash = (u32_t *) &ld_begin_vectors;

		flash_sizeof = * (flash + 8) - * (flash + 7);
		flash_crc32 = crc32b(flash, flash_sizeof);

		msg.ID = IFCAN_ID_FLASH_INIT;
		msg.len = 8;

		* (u32_t *) &msg.payload[0] = flash_sizeof;
		* (u32_t *) &msg.payload[4] = flash_crc32;

		IFCAN_send_msg(&msg);

		printf("Wait for %i nodes ..." EOL, nodes_N);

		/* Wait for ACK from all nodes.
		 * */
		vTaskDelay((TickType_t) 50);

		for (N = 0; N < 90; ++N) {

			if (local_nodes_by_ACK(IFCAN_ACK_FLASH_WAIT_FOR_ERASE) == 0) {

				break;
			}

			/* Wait a bit more.
			 * */
			vTaskDelay((TickType_t) 100);
		}

		nodes_N = local_nodes_by_ACK(IFCAN_ACK_FLASH_UP_TO_DATE);

		if (nodes_N != 0) {

			printf("%i nodes are up to date" EOL, nodes_N);
		}

		nodes_N = local_nodes_by_ACK(IFCAN_ACK_FLASH_INIT_FAILURE);

		if (nodes_N != 0) {

			printf("%i nodes are failed" EOL, nodes_N);
		}

		if (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT) != 0) {

			IFCAN_flash_DATA(flash, flash_sizeof);

			/* Wait for ACK from all nodes.
			 * */
			vTaskDelay((TickType_t) 50);

			nodes_N = local_nodes_by_ACK(IFCAN_ACK_FLASH_SELFUPDATE);

			printf("%i nodes were updated" EOL, nodes_N);
		}
	}
}

void task_REMOTE(void *pData)
{
	CAN_msg_t		msg;
	int			xC;

	do {
		msg.len = 0;

		while (xQueueReceive(local.queue_EX, &xC, (TickType_t) 10) == pdTRUE) {

			msg.payload[msg.len++] = xC;

			if (msg.len >= 8) {

				break;
			}
		}

		if (msg.len != 0) {

			msg.ID = local.remote_node_ID + IFCAN_ID_NODE_RX;

			IFCAN_send_msg(&msg);
		}
	}
	while (1);
}

void IFCAN_remote_putc(int c)
{
	char		xC = (char) c;

	xQueueSendToBack(local.queue_EX, &xC, portMAX_DELAY);
}

SH_DEF(can_node_remote)
{
	TaskHandle_t		xHandle;
	int			node_ID;
	char			xC;

	io_ops_t		ops = {

		.getc = NULL,
		.putc = &IFCAN_remote_putc
	};

	if (stoi(&node_ID, s) != NULL) {

		if (local_node_is_valid_remote(node_ID) != 0) {

			local.remote_node_ID = node_ID;
		}
		else {
			printf("No valid remote node ID" EOL);
		}
	}

	if (local.remote_node_ID != 0) {

		CAN_filter_ID(3, 0, local.remote_node_ID + IFCAN_ID_NODE_TX, IFCAN_FILTER_MATCH);

		xTaskCreate(task_REMOTE, "REMOTE", configMINIMAL_STACK_SIZE, NULL, 2, &xHandle);

		xputs(&ops, EOL);

		do {
			xC = getc();

			IFCAN_remote_putc(xC);

			if (xC == K_EOT)
				break;
		}
		while (1);

		vTaskDelete(xHandle);

		CAN_filter_ID(3, 0, 0, 0);

		local.remote_node_ID = 0;

		puts(EOL EOL);
	}
}

