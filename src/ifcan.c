#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "ifcan.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

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
	SemaphoreHandle_t	log_MUX;
	int			log_skipped;

	/* Network service.
	 * */
	QueueHandle_t		queue_NET;

	/* FLASH update across network.
	 * */
	u32_t			flash_INIT_sizeof;
	u32_t			flash_INIT_crc32;
	char			flash_INIT_revision[16];

	/* Remote node ID we connected to.
	 * */
	int			remote_node_ID;

	/* TX pause notice (flow control).
	 * */
	int			flow_TX_paused;

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

ifcan_t				net;

static ifcan_local_t		local;

static void
IFCAN_pipe_INCOMING(ifcan_pipe_t *pp, const CAN_msg_t *msg)
{
	float			lerp;

	switch (pp->PAYLOAD) {

		case IFCAN_PAYLOAD_FLOAT:

			if (msg->len == 4) {

				lerp = * (float *) &msg->payload[0];

				pp->reg_DATA = (pp->range[1] == 0.f) ? lerp
					: pp->range[1] * lerp + pp->range[0];
			}
			break;

		case IFCAN_PAYLOAD_INT_16:

			if (msg->len == 2) {

				lerp = (float) * (u16_t *) &msg->payload[0] * (1.f / 65535.f);

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

			lerp = (pp->range[1] == 0.f) ? pp->reg_DATA
				: pp->range[1] * pp->reg_DATA + pp->range[0];

			* (float *) &msg.payload[0] = lerp;
			break;

		case IFCAN_PAYLOAD_INT_16:

			msg.len = 2;

			lerp = (pp->reg_DATA - pp->range[0]) / (pp->range[1] - pp->range[0]);
			lerp = (lerp < 0.f) ? 0.f : (lerp > 1.f) ? 1.f : lerp;

			* (u16_t *) &msg.payload[0] = (u16_t) (lerp * 65535.f);
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

		pp = &net.pipe[N];

		if (		pp->MODE == IFCAN_PIPE_INCOMING
				&& pp->ID == msg->ID) {

			pp->tim_N = 0;

			if (		pp->STARTUP == PM_ENABLED
					&& pm.lu_MODE == PM_LU_DISABLED) {

				pp->ACTIVE = PM_ENABLED;
				pm.fsm_req = PM_STATE_LU_STARTUP;
			}

			IFCAN_pipe_INCOMING(pp, msg);
		}
		else if (	pp->MODE == IFCAN_PIPE_OUTGOING_INJECTED
				&& pp->inject_ID == msg->ID) {

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

		pp = &net.pipe[N];

		if (		pp->MODE == IFCAN_PIPE_INCOMING
				&& pp->ACTIVE == PM_ENABLED
				&& pm.lu_MODE != PM_LU_DISABLED) {

			pp->tim_N++;

			if (pp->tim_N >= net.startup_LOST) {

				pp->ACTIVE = PM_DISABLED;
				pp->tim_N = 0;

				pm.fsm_req = PM_STATE_LU_SHUTDOWN;
			}
		}
		else if (pp->MODE == IFCAN_PIPE_OUTGOING_REGULAR) {

			pp->tim_N++;

			if (pp->tim_N >= pp->TIM) {

				pp->tim_N = 0;

				if (		pp->STARTUP != PM_ENABLED
						|| pm.lu_MODE != PM_LU_DISABLED) {

					pp->flag_TX = 1;
				}
			}

			if (pp->flag_TX != 0) {

				IFCAN_pipe_OUTGOING(pp);
			}
		}
		else if (	pp->MODE == IFCAN_PIPE_OUTGOING_INJECTED
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

	if (		hal.CAN_msg.ID >= IFCAN_ID_NODE_BASE
			|| net.log_MODE == IFCAN_LOG_PROMISCUOUS) {

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
local_node_replace_ACK(int node_ACK, int node_NACK)
{
	int			N;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].node_ACK == node_ACK) {

			local.node[N].node_ACK = node_NACK;
		}
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

		if (local.node[N].node_ID != 0) {

			nodes_N += 1;
		}
	}

	return nodes_N;
}

static int
local_nodes_by_ACK(int node_ACK)
{
	int		N, nodes_N = 0;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (		   local.node[N].node_ID != 0
				&& local.node[N].node_ACK == node_ACK) {

			nodes_N += 1;
		}
	}

	return nodes_N;
}

static void
local_node_update_ACK(int node_ID, int node_ACK)
{
	int			N;

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].node_ID == node_ID) {

			/* Update node ACK.
			 * */
			local.node[N].node_ACK = node_ACK;
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

	rc = (node_ID == net.node_ID) ? 0 : rc;

	return rc;
}

static int
local_node_assign_ID()
{
	int		node_ID, found_ID = 0;

	for (node_ID = 1; node_ID <= IFCAN_NODES_MAX; ++node_ID) {

		if (local_node_is_valid_remote(node_ID) == 0
				&& node_ID != net.node_ID) {

			found_ID = node_ID;
			break;
		}
	}

	return found_ID;
}

static int
local_node_random_ID()
{
	int		list_ID[IFCAN_NODES_MAX];

	int		node_ID, found_ID = 0;
	int		N = 0;

	for (node_ID = 1; node_ID <= IFCAN_NODES_MAX; ++node_ID) {

		if (local_node_is_valid_remote(node_ID) == 0
				&& node_ID != net.node_ID) {

			list_ID[N++] = node_ID;
		}
	}

	if (N > 0) {

		found_ID = list_ID[urand() % N];
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
IFCAN_log_msg(CAN_msg_t *msg, const char *label)
{
	int		N;

	io_ops_t	ops = {

		.getc = NULL,
		.putc = &IFCAN_log_putc
	};

	if (		uxQueueSpacesAvailable(local.queue_LOG) >= 60
			&& xSemaphoreTake(local.log_MUX, (TickType_t) 10) == pdTRUE) {

		if (local.log_skipped != 0) {

			xprintf(&ops, "(skipped %i)" EOL, local.log_skipped);

			local.log_skipped = 0;
		}

		xprintf(&ops, "%s %i %i", label, msg->ID, msg->len);

		for (N = 0; N < msg->len; ++N) {

			xprintf(&ops, " %2x", msg->payload[N]);
		}

		xputs(&ops, EOL);

		xSemaphoreGive(local.log_MUX);
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

			if (net.log_MODE != IFCAN_LOG_DISABLED) {

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
		TIM_wait_ns(50000);
	}
	while (1);

	return rc;
}

void task_IFCAN_NET(void *pData)
{
	CAN_msg_t		msg;
	int			node_ACK, node_ID;

	do {
		while (xQueueReceive(local.queue_NET, &node_ACK, portMAX_DELAY) == pdTRUE) {

			if (node_ACK == IFCAN_ACK_NETWORK_REPLY) {

				if (net.node_ID != 0) {

					/* Case of assigned node ID.
					 * */
					node_ID = net.node_ID;
				}
				else {
					/* Wait for assigned nodes have replied.
					 * */
					vTaskDelay((TickType_t) 10);

					/* Get a random node ID from list of
					 * free ones.
					 * */
					node_ID = local_node_random_ID();

					/* Also we delay the transmission by
					 * random value to reduce the chance of
					 * a data collision.
					 * */
					vTaskDelay((TickType_t) (urand() % 30U));
				}

				msg.ID = IFCAN_ID(node_ID, IFCAN_NODE_ACK);
				msg.len = 6;

				* (u32_t *) &msg.payload[0] = local.UID;

				msg.payload[4] = net.node_ID;
				msg.payload[5] = node_ACK;

				IFCAN_send_msg(&msg);
			}
		}
	}
	while (1);
}

static void
IFCAN_node_ACK(int node_ACK)
{
	CAN_msg_t		msg;

	if (node_ACK == IFCAN_ACK_NETWORK_REPLY) {

		xQueueSendToBack(local.queue_NET, &node_ACK, (TickType_t) 0);
	}
	else {
		msg.ID = IFCAN_ID(net.node_ID, IFCAN_NODE_ACK);
		msg.len = 2;

		msg.payload[0] = net.node_ID;
		msg.payload[1] = node_ACK;

		IFCAN_send_msg(&msg);
	}
}

static void
IFCAN_remote_node_REQ(int node_REQ)
{
	CAN_msg_t		msg;

	msg.ID = IFCAN_ID(local.remote_node_ID, IFCAN_NODE_REQ);
	msg.len = 1;

	msg.payload[0] = node_REQ;

	IFCAN_send_msg(&msg);
}

static int
IFCAN_flash_is_up_to_date()
{
	u32_t			*flash = (u32_t *) fw.ld_begin;
	u32_t			flash_sizeof;

	flash_sizeof = fw.ld_end - fw.ld_begin;

	return (flash_sizeof == local.flash_INIT_sizeof
			&& crc32b(flash, flash_sizeof)
			== local.flash_INIT_crc32) ? 1 : 0;
}

static void
IFCAN_message_IN(const CAN_msg_t *msg)
{
	int			N;

	/* Network functions.
	 * */
	if (msg->ID == IFCAN_ID_NET_SURVEY) {

		local_node_discard();

		IFCAN_node_ACK(IFCAN_ACK_NETWORK_REPLY);
	}
	else if (msg->ID == IFCAN_ID_NET_ASSIGN
			&& msg->len == 5) {

		if (* (u32_t *) &msg->payload[0] == local.UID) {

			/* Accept new node ID.
			 * */
			net.node_ID = msg->payload[4];

			IFCAN_filter_ID();
		}
		else {
			local_node_insert(* (u32_t *) &msg->payload[0], msg->payload[4]);
		}
	}

	/* Flash functions.
	 * */
	else if (msg->ID == IFCAN_ID_FLASH_REVISION1
			&& msg->len == 8
			&& net.node_ID != 0) {

		local.flash_INIT_revision[0] = msg->payload[0];
		local.flash_INIT_revision[1] = msg->payload[1];
		local.flash_INIT_revision[2] = msg->payload[2];
		local.flash_INIT_revision[3] = msg->payload[3];
		local.flash_INIT_revision[4] = msg->payload[4];
		local.flash_INIT_revision[5] = msg->payload[5];
		local.flash_INIT_revision[6] = msg->payload[6];
		local.flash_INIT_revision[7] = msg->payload[7];
		local.flash_INIT_revision[8] = 0;
	}
	else if (msg->ID == IFCAN_ID_FLASH_REVISION2
			&& msg->len == 8
			&& net.node_ID != 0) {

		local.flash_INIT_revision[8] = msg->payload[0];
		local.flash_INIT_revision[9] = msg->payload[1];
		local.flash_INIT_revision[10] = msg->payload[2];
		local.flash_INIT_revision[11] = msg->payload[3];
		local.flash_INIT_revision[12] = msg->payload[4];
		local.flash_INIT_revision[13] = msg->payload[5];
		local.flash_INIT_revision[14] = msg->payload[6];
		local.flash_INIT_revision[15] = 0;
	}
	else if (msg->ID == IFCAN_ID_FLASH_INIT
			&& msg->len == 8
			&& net.node_ID != 0) {

		local.flash_INIT_sizeof = * (u32_t *) &msg->payload[0];
		local.flash_INIT_crc32 = * (u32_t *) &msg->payload[4];

		if (		pm.lu_MODE != PM_LU_DISABLED
				|| net.upgrade_MODE != PM_ENABLED) {

			IFCAN_node_ACK(IFCAN_ACK_FLASH_REJECT);
		}
		else if (strcmp(local.flash_INIT_revision, fw.hwrevision) != 0) {

			IFCAN_node_ACK(IFCAN_ACK_FLASH_REJECT);
		}
		else if (	local.flash_INIT_sizeof == 0UL
				|| local.flash_INIT_sizeof > FLASH_map[0] - fw.ld_begin) {

			IFCAN_node_ACK(IFCAN_ACK_FLASH_REJECT);
		}
		else if (IFCAN_flash_is_up_to_date() != 0) {

			IFCAN_node_ACK(IFCAN_ACK_FLASH_UP_TO_DATE);
		}
		else {
			app_halt();

			taskDISABLE_INTERRUPTS();

			GPIO_set_HIGH(GPIO_LED);

			/* Go into the selfupdate routine (run from RAM).
			 * */
			FLASH_selfupdate_CAN(local.flash_INIT_sizeof, local.flash_INIT_crc32);
		}
	}

	/* Node functions.
	 * */
	else if (msg->ID == IFCAN_ID(net.node_ID, IFCAN_NODE_REQ)) {

		if (msg->payload[0] == IFCAN_REQ_FLOW_TX_PAUSE
				&& msg->len == 1) {

			local.flow_TX_paused = 1;
		}
	}
	else if ((msg->ID & IFCAN_ID(0, 31)) == IFCAN_ID(0, IFCAN_NODE_ACK)
			&& IFCAN_GET_NODE(msg->ID) != 31U
			&& IFCAN_GET_NODE(msg->ID) != 0U) {

		if (msg->payload[5] == IFCAN_ACK_NETWORK_REPLY
				&& msg->len == 6) {

			local_node_insert(* (u32_t *) &msg->payload[0], msg->payload[4]);
		}
		else if (msg->len == 2) {

			local_node_update_ACK(msg->payload[0], msg->payload[1]);
		}
	}
	else if (msg->ID == IFCAN_ID(net.node_ID, IFCAN_NODE_RX)) {

		for (N = 0; N < msg->len; ++N) {

			xQueueSendToBack(local.queue_RX, &msg->payload[N], (TickType_t) 0);
		}

		IODEF_TO_CAN();
	}
	else if (msg->ID == IFCAN_ID(local.remote_node_ID, IFCAN_NODE_TX)) {

		for (N = 0; N < msg->len; ++N) {

			/* Remote NODE output via LOG.
			 * */
			IFCAN_log_putc(msg->payload[N]);
		}

		if (uxQueueSpacesAvailable(local.queue_LOG) < 20) {

			/* Notify remote node about overflow.
			 * */
			IFCAN_remote_node_REQ(IFCAN_REQ_FLOW_TX_PAUSE);
		}
	}
}

void task_IFCAN_IN(void *pData)
{
	CAN_msg_t		msg;

	do {
		while (xQueueReceive(local.queue_IN, &msg, portMAX_DELAY) == pdTRUE) {

			if (net.log_MODE != IFCAN_LOG_DISABLED) {

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

		if (msg.len > 0) {

			msg.ID = IFCAN_ID(net.node_ID, IFCAN_NODE_TX);

			IFCAN_send_msg(&msg);

			/* Do not send messages too frequently especially if
			 * you were asked to.
			 * */
			if (local.flow_TX_paused != 0) {

				vTaskDelay((TickType_t) 10);

				local.flow_TX_paused = 0;
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
	local.UID = RNG_make_UID();

	/* Allocate queues.
	 * */
	local.queue_IN = xQueueCreate(10, sizeof(CAN_msg_t));
	local.queue_RX = USART_queue_RX();
	local.queue_TX = xQueueCreate(80, sizeof(char));
	local.queue_EX = xQueueCreate(40, sizeof(char));
	local.queue_NET = xQueueCreate(1, sizeof(int));
	local.queue_LOG = xQueueCreate(400, sizeof(char));

	/* Allocate log semaphore.
	 * */
	local.log_MUX = xSemaphoreCreateMutex();

	/* Create IFCAN tasks.
	 * */
	xTaskCreate(task_IFCAN_IN, "IFCAN_IN", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_IFCAN_TX, "IFCAN_TX", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_IFCAN_NET, "IFCAN_NET", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_IFCAN_LOG, "IFCAN_LOG", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	CAN_startup();

	IFCAN_filter_ID();
}

void IFCAN_filter_ID()
{
	int			N;

	if (net.log_MODE == IFCAN_LOG_PROMISCUOUS) {

		CAN_filter_ID(0, 0, 1, 0);
	}
	else {
		CAN_filter_ID(0, 0, IFCAN_FILTER_NETWORK, IFCAN_FILTER_NETWORK);
	}

	if (net.node_ID != 0) {

		CAN_filter_ID(1, 0, IFCAN_ID(net.node_ID, IFCAN_NODE_REQ), IFCAN_FILTER_MATCH);
		CAN_filter_ID(2, 0, IFCAN_ID(0, IFCAN_NODE_ACK), IFCAN_ID(0, 31));
		CAN_filter_ID(3, 0, IFCAN_ID(net.node_ID, IFCAN_NODE_RX), IFCAN_FILTER_MATCH);
		CAN_filter_ID(4, 0, 0, 0);
	}
	else {
		CAN_filter_ID(1, 0, 0, 0);
		CAN_filter_ID(2, 0, IFCAN_ID(0, IFCAN_NODE_ACK), IFCAN_ID(0, 31));
		CAN_filter_ID(3, 0, 0, 0);
		CAN_filter_ID(4, 0, 0, 0);
	}

	for (N = 0; N < IFCAN_PIPES_MAX; ++N) {

		if (net.pipe[N].MODE == IFCAN_PIPE_INCOMING) {

			CAN_filter_ID(20 + N, 1, net.pipe[N].ID, IFCAN_FILTER_MATCH);
		}
		else if (net.pipe[N].MODE == IFCAN_PIPE_OUTGOING_INJECTED) {

			CAN_filter_ID(20 + N, 1, net.pipe[N].inject_ID, IFCAN_FILTER_MATCH);
		}
		else {
			CAN_filter_ID(20 + N, 1, 0, 0);
		}
	}
}

SH_DEF(net_survey)
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

				if (local.node[N].node_ID != 0) {

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

SH_DEF(net_assign)
{
	CAN_msg_t		msg;
	int			N;

	if (net.node_ID == 0) {

		net.node_ID = local_node_assign_ID();

		IFCAN_filter_ID();

		printf("local assigned to %i" EOL, net.node_ID);
	}

	for (N = 0; N < IFCAN_NODES_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID == 0) {

			local.node[N].node_ID = local_node_assign_ID();

			if (local.node[N].node_ID != 0) {

				msg.ID = IFCAN_ID_NET_ASSIGN;
				msg.len = 5;

				* (u32_t *) &msg.payload[0] = local.node[N].UID;
				msg.payload[4] = local.node[N].node_ID;

				IFCAN_send_msg(&msg);

				printf("UID %8x assigned to %i" EOL,
						local.node[N].UID,
						local.node[N].node_ID);
			}
		}
	}
}

SH_DEF(net_revoke)
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
				msg.len = 5;

				* (u32_t *) &msg.payload[0] = local.node[N].UID;
				msg.payload[4] = 0;

				IFCAN_send_msg(&msg);

				printf("UID %8x revoked" EOL, local.node[N].UID);
			}
		}
	}
}

static void
IFCAN_flash_DATA(const u32_t *flash, u32_t flash_sizeof)
{
	CAN_msg_t		msg;
	int			blk_N, blk_END, blk_DATA;
	int			ack_N, fail_N;

	msg.ID = IFCAN_ID_FLASH_DATA;
	msg.len = 8;

	blk_N = 0;
	blk_END = flash_sizeof / 4U;

	fail_N = 0;

	puts("Flash ");

	do {
		ack_N = 0;

		/* Note that bandwidth is limited because of it could take a
		 * time to program flash on remote side. So we should check if
		 * remote side signals us to pause transmission.
		 * */
		while (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_PAUSE) != 0) {

			ack_N++;

			if (ack_N >= 5) {

				local_node_replace_ACK(IFCAN_ACK_FLASH_DATA_PAUSE,
						IFCAN_ACK_FLASH_REJECT);
				break;
			}

			vTaskDelay((TickType_t) 1);
		}

		/* Check if there are nodes ready to accept data.
		 * */
		if (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT) == 0) {

			puts(" Done");
			break;
		}

		/* Check if enough data has already been transferred.
		 * */
		if (blk_N >= blk_END * 2U) {

			puts(" Done");
			break;
		}

		blk_DATA = blk_N % blk_END;

		/* Fill the packet payload.
		 * */
		* (u32_t *) &msg.payload[0] = (u32_t) blk_DATA;
		* (u32_t *) &msg.payload[4] = * (flash + blk_DATA);

		if (IFCAN_send_msg(&msg) == CAN_TX_OK) {

			blk_N += 1;

			if (blk_N % 2048 == 0) {

				/* Display uploading progress.
				 * */
				putc('.');
			}
		}
		else {
			fail_N += 1;

			if (fail_N >= 1000) {

				puts(" Fail");
				break;
			}
		}
	}
	while (1);

	puts(EOL);
}

SH_DEF(net_flash_update)
{
	const FW_info_t		*info;
	CAN_msg_t		msg;
	u32_t			flash_sizeof, flash_crc32;
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

		/* Get firmware information structure from the flash content.
		 * */
		info = (const FW_info_t *) * ((u32_t * ) fw.ld_begin + 7);

		/* Send REVISION messages (hardware revision).
		 * */
		msg.ID = IFCAN_ID_FLASH_REVISION1;
		msg.len = 8;

		msg.payload[0] = info->hwrevision[0];
		msg.payload[1] = info->hwrevision[1];
		msg.payload[2] = info->hwrevision[2];
		msg.payload[3] = info->hwrevision[3];
		msg.payload[4] = info->hwrevision[4];
		msg.payload[5] = info->hwrevision[5];
		msg.payload[6] = info->hwrevision[6];
		msg.payload[7] = info->hwrevision[7];

		IFCAN_send_msg(&msg);

		if (strlen(info->hwrevision) > 8) {

			msg.ID = IFCAN_ID_FLASH_REVISION2;
			msg.len = 8;

			msg.payload[0] = info->hwrevision[8];
			msg.payload[1] = info->hwrevision[9];
			msg.payload[2] = info->hwrevision[10];
			msg.payload[3] = info->hwrevision[11];
			msg.payload[4] = info->hwrevision[12];
			msg.payload[5] = info->hwrevision[13];
			msg.payload[6] = info->hwrevision[14];
			msg.payload[7] = info->hwrevision[15];

			IFCAN_send_msg(&msg);
		}

		/* Send INIT message (sizeof and crc32).
		 * */
		flash_sizeof = info->ld_end - info->ld_begin;
		flash_crc32 = crc32b((const void *) info->ld_begin, flash_sizeof);

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

		nodes_N = local_nodes_by_ACK(IFCAN_ACK_FLASH_REJECT);

		if (nodes_N != 0) {

			printf("%i nodes are reject" EOL, nodes_N);
		}

		if (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT) != 0) {

			IFCAN_flash_DATA((const void *) info->ld_begin, flash_sizeof);

			/* Wait for ACK from all nodes.
			 * */
			vTaskDelay((TickType_t) 50);

			nodes_N = local_nodes_by_ACK(IFCAN_ACK_FLASH_FINISHED);

			printf("%i nodes were updated" EOL, nodes_N);
		}
		else {
			printf("No remote nodes were updated" EOL);
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

		if (msg.len > 0) {

			msg.ID = IFCAN_ID(local.remote_node_ID, IFCAN_NODE_RX);

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

SH_DEF(net_node_remote)
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

		/* Do listen to incoming messages from remote node.
		 * */
		CAN_filter_ID(4, 0, IFCAN_ID(local.remote_node_ID, IFCAN_NODE_TX), IFCAN_FILTER_MATCH);

		/* Create task to outgoing message packaging.
		 * */
		xTaskCreate(task_REMOTE, "REMOTE", configMINIMAL_STACK_SIZE, NULL, 2, &xHandle);

		xputs(&ops, EOL);

		do {
			xC = getc();

			IFCAN_remote_putc(xC);

			if (xC == K_EOT)
				break;
		}
		while (1);

		/* Drop incoming connection immediately.
		 * */
		CAN_filter_ID(4, 0, 0, 0);

		local.remote_node_ID = 0;

		/* Wait for task_REMOTE to transmit the last message.
		 * */
		vTaskDelay((TickType_t) 50);

		vTaskDelete(xHandle);

		/* Do pretty line feed.
		 * */
		puts(EOL);
	}
}

