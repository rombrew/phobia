#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "epcan.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

typedef struct {

	uint32_t		UID;

	/* Input CAN messages.
	 * */
	QueueHandle_t		in_queue;

	/* Serial IO.
	 * */
	QueueHandle_t		rx_queue;
	QueueHandle_t		tx_queue;
	QueueHandle_t		remote_queue;

	/* Serial LOG.
	 * */
	QueueHandle_t		log_queue;
	SemaphoreHandle_t	log_mutex;
	int			log_skipped;

	/* Network service.
	 * */
	QueueHandle_t		net_queue;

	/* Remote node ID we connected to.
	 * */
	int			remote_node_ID;

	/* TX pause notice (flow control).
	 * */
	int			flow_tx_paused;

	/* Remote nodes TABLE.
	 * */
	struct {

		uint32_t	UID;

		uint16_t	node_ID;
		uint16_t	node_ACK;
	}
	node[EPCAN_NODES_MAX];
}
epcan_local_t;

epcan_t				net;

static epcan_local_t		local;

static void
EPCAN_pipe_INCOMING(epcan_pipe_t *ep, const CAN_msg_t *msg)
{
	float			lerp;

	switch (ep->PAYLOAD) {

		case EPCAN_PAYLOAD_FLOAT:

			if (msg->len == 4) {

				lerp = * (float *) &msg->payload[0];

				ep->reg_DATA = (ep->range[1] == 0.f) ? lerp
					: ep->range[1] * lerp + ep->range[0];
			}
			break;

		case EPCAN_PAYLOAD_INT_16:

			if (msg->len == 2) {

				lerp = (float) * (uint16_t *) &msg->payload[0] * (1.f / 65535.f);

				ep->reg_DATA = ep->range[0] + lerp * (ep->range[1] - ep->range[0]);
			}
			break;

		default: break;
	}

	if (ep->reg_ID != ID_NULL) {

		reg_SET_F(ep->reg_ID, ep->reg_DATA);
	}
}

static void
EPCAN_pipe_OUTGOING(epcan_pipe_t *ep)
{
	CAN_msg_t		msg;
	float			lerp;

	if (ep->reg_ID != ID_NULL) {

		ep->reg_DATA = reg_GET_F(ep->reg_ID);
	}

	msg.ID = ep->ID;

	switch (ep->PAYLOAD) {

		case EPCAN_PAYLOAD_FLOAT:

			msg.len = 4;

			lerp = (ep->range[1] == 0.f) ? ep->reg_DATA
				: ep->range[1] * ep->reg_DATA + ep->range[0];

			* (float *) &msg.payload[0] = lerp;
			break;

		case EPCAN_PAYLOAD_INT_16:

			msg.len = 2;

			lerp = (ep->reg_DATA - ep->range[0]) / (ep->range[1] - ep->range[0]);
			lerp = (lerp < 0.f) ? 0.f : (lerp > 1.f) ? 1.f : lerp;

			* (uint16_t *) &msg.payload[0] = (uint16_t) (lerp * 65535.f);
			break;

		default: break;
	}

	if (CAN_send_msg(&msg) == CAN_TX_OK) {

		ep->tx_flag = 0;
	}
}

static void
EPCAN_pipe_message_IN(const CAN_msg_t *msg)
{
	epcan_pipe_t		*ep;
	int			N;

	for (N = 0; N < EPCAN_PIPE_MAX; ++N) {

		ep = &net.ep[N];

		if (		ep->MODE == EPCAN_PIPE_INCOMING
				&& ep->ID == msg->ID) {

			ep->tx_N = 0;

			if (		ep->ACTIVE == PM_ENABLED
					&& pm.fsm_errno != PM_OK) {

				ep->ACTIVE = PM_DISABLED;
			}

			if (		ep->STARTUP == PM_ENABLED
					&& ep->ACTIVE != PM_ENABLED
					&& pm.lu_MODE == PM_LU_DISABLED) {

				pm.fsm_errno = PM_OK;
				pm.fsm_req = PM_STATE_LU_STARTUP;

				ep->ACTIVE = PM_ENABLED;
			}

			EPCAN_pipe_INCOMING(ep, msg);
		}
		else if (	ep->MODE == EPCAN_PIPE_OUTGOING_INJECTED
				&& ep->clock_ID == msg->ID) {

			ep->tx_flag = 1;

			EPCAN_pipe_OUTGOING(ep);
		}
	}
}

void EPCAN_pipe_REGULAR()
{
	epcan_pipe_t		*ep;
	int			N;

	for (N = 0; N < EPCAN_PIPE_MAX; ++N) {

		ep = &net.ep[N];

		if (		ep->MODE == EPCAN_PIPE_INCOMING
				&& ep->ACTIVE == PM_ENABLED) {

			ep->tx_N++;

			if (ep->tx_N >= net.timeout_EP) {

				if (pm.lu_MODE != PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				}

				ep->ACTIVE = PM_DISABLED;
				ep->tx_N = 0;
			}
		}
		else if (ep->MODE == EPCAN_PIPE_OUTGOING_REGULAR) {

			ep->tx_N++;

			if (ep->tx_N >= ep->rate) {

				ep->tx_N = 0;

				if (		ep->STARTUP != PM_ENABLED
						|| pm.lu_MODE != PM_LU_DISABLED) {

					ep->tx_flag = 1;
				}
			}

			if (ep->tx_flag != 0) {

				EPCAN_pipe_OUTGOING(ep);
			}
		}
		else if (	ep->MODE == EPCAN_PIPE_OUTGOING_INJECTED
				&& ep->tx_flag != 0) {

			EPCAN_pipe_OUTGOING(ep);
		}
	}
}

void CAN_IRQ()
{
	BaseType_t		xWoken = pdFALSE;

	if (hal.CAN_msg.ID < EPCAN_ID_NODE_BASE) {

		EPCAN_pipe_message_IN(&hal.CAN_msg);
	}

	if (		hal.CAN_msg.ID >= EPCAN_ID_NODE_BASE
			|| net.log_MODE == EPCAN_LOG_PROMISCUOUS) {

		xQueueSendToBackFromISR(local.in_queue, &hal.CAN_msg, &xWoken);
	}

	portYIELD_FROM_ISR(xWoken);
}

static void
local_node_discard()
{
	int			N;

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

		local.node[N].UID = 0;
		local.node[N].node_ID = 0;
	}
}

static void
local_node_insert(uint32_t UID, int node_ID)
{
	int			N, found = 0;

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

		if (local.node[N].UID == UID) {

			found = 1;

			/* Update node ID.
			 * */
			local.node[N].node_ID = node_ID;
			break;
		}
	}

	if (found == 0) {

		for (N = 0; N < EPCAN_NODES_MAX; ++N) {

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

static void
local_node_update_ACK(int node_ID, int node_ACK)
{
	int			N;

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

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

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

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

	for (node_ID = 1; node_ID <= EPCAN_NODES_MAX; ++node_ID) {

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
	int		list_ID[EPCAN_NODES_MAX];

	int		node_ID, found_ID = 0;
	int		N = 0;

	for (node_ID = 1; node_ID <= EPCAN_NODES_MAX; ++node_ID) {

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

LD_TASK void task_EPCAN_LOG(void *pData)
{
	char			xbyte;

	do {
		while (xQueueReceive(local.log_queue, &xbyte, portMAX_DELAY) == pdTRUE) {

			/* We need a decoupling queue to output the LOG as not
			 * to block regular messages processing. If we block
			 * here that some part of LOG may be lost.
			 * */
			putc(xbyte);
		}
	}
	while (1);
}

void EPCAN_log_putc(int c)
{
	char		xbyte = (char) c;

	xQueueSendToBack(local.log_queue, &xbyte, (TickType_t) 0);
}

static void
EPCAN_log_msg(CAN_msg_t *msg, const char *label)
{
	int		N;

	io_ops_t	ops = {

		.getc = NULL,
		.putc = &EPCAN_log_putc
	};

	if (		uxQueueSpacesAvailable(local.log_queue) >= 60
			&& xSemaphoreTake(local.log_mutex, (TickType_t) 10) == pdTRUE) {

		if (local.log_skipped != 0) {

			xprintf(&ops, "(skipped %i)" EOL, local.log_skipped);

			local.log_skipped = 0;
		}

		xprintf(&ops, "%s %i %i", label, msg->ID, msg->len);

		for (N = 0; N < msg->len; ++N) {

			xprintf(&ops, " %2x", msg->payload[N]);
		}

		xputs(&ops, EOL);

		xSemaphoreGive(local.log_mutex);
	}
	else {
		local.log_skipped += 1;
	}
}

static int
EPCAN_send_msg(CAN_msg_t *msg)
{
	int			rc, N = 0;

	do {
		rc = CAN_send_msg(msg);

		if (rc == CAN_TX_OK) {

			if (net.log_MODE != EPCAN_LOG_DISABLED) {

				EPCAN_log_msg(msg, "TX");
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

LD_TASK void task_EPCAN_NET(void *pData)
{
	CAN_msg_t		msg;
	int			node_ACK, node_ID;

	do {
		while (xQueueReceive(local.net_queue, &node_ACK, portMAX_DELAY) == pdTRUE) {

			if (node_ACK == EPCAN_ACK_NETWORK_REPLY) {

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

				msg.ID = EPCAN_ID(node_ID, EPCAN_NODE_ACK);
				msg.len = 6;

				* (uint32_t *) &msg.payload[0] = local.UID;

				msg.payload[4] = net.node_ID;
				msg.payload[5] = node_ACK;

				EPCAN_send_msg(&msg);
			}
		}
	}
	while (1);
}

static void
EPCAN_node_ACK(int node_ACK)
{
	CAN_msg_t		msg;

	if (node_ACK == EPCAN_ACK_NETWORK_REPLY) {

		xQueueSendToBack(local.net_queue, &node_ACK, (TickType_t) 0);
	}
	else {
		msg.ID = EPCAN_ID(net.node_ID, EPCAN_NODE_ACK);
		msg.len = 2;

		msg.payload[0] = net.node_ID;
		msg.payload[1] = node_ACK;

		EPCAN_send_msg(&msg);
	}
}

static void
EPCAN_remote_node_REQ(int node_REQ)
{
	CAN_msg_t		msg;

	msg.ID = EPCAN_ID(local.remote_node_ID, EPCAN_NODE_REQ);
	msg.len = 1;

	msg.payload[0] = node_REQ;

	EPCAN_send_msg(&msg);
}

static void
EPCAN_message_IN(const CAN_msg_t *msg)
{
	int			N;

	/* Network functions.
	 * */
	if (msg->ID == EPCAN_ID_NET_SURVEY) {

		local_node_discard();

		EPCAN_node_ACK(EPCAN_ACK_NETWORK_REPLY);
	}
	else if (msg->ID == EPCAN_ID_NET_ASSIGN
			&& msg->len == 5) {

		if (* (uint32_t *) &msg->payload[0] == local.UID) {

			/* Accept new node ID.
			 * */
			net.node_ID = msg->payload[4];

			EPCAN_filter_ID();
		}
		else {
			local_node_insert(* (uint32_t *) &msg->payload[0], msg->payload[4]);
		}
	}

	/* Node functions.
	 * */
	else if (msg->ID == EPCAN_ID(net.node_ID, EPCAN_NODE_REQ)) {

		if (msg->payload[0] == EPCAN_REQ_FLOW_TX_PAUSE
				&& msg->len == 1) {

			local.flow_tx_paused = 1;
		}
	}
	else if ((msg->ID & EPCAN_ID(0, 31)) == EPCAN_ID(0, EPCAN_NODE_ACK)
			&& EPCAN_GET_NODE(msg->ID) != 31U
			&& EPCAN_GET_NODE(msg->ID) != 0U) {

		if (msg->payload[5] == EPCAN_ACK_NETWORK_REPLY
				&& msg->len == 6) {

			local_node_insert(* (uint32_t *) &msg->payload[0], msg->payload[4]);
		}
		else if (msg->len == 2) {

			local_node_update_ACK(msg->payload[0], msg->payload[1]);
		}
	}
	else if (msg->ID == EPCAN_ID(net.node_ID, EPCAN_NODE_RX)) {

		for (N = 0; N < msg->len; ++N) {

			xQueueSendToBack(local.rx_queue, &msg->payload[N], (TickType_t) 0);
		}

		IODEF_TO_CAN();
	}
	else if (msg->ID == EPCAN_ID(local.remote_node_ID, EPCAN_NODE_TX)) {

		for (N = 0; N < msg->len; ++N) {

			/* Remote NODE output via LOG.
			 * */
			EPCAN_log_putc(msg->payload[N]);
		}

		if (uxQueueSpacesAvailable(local.log_queue) < 20) {

			/* Notify remote node about overflow.
			 * */
			EPCAN_remote_node_REQ(EPCAN_REQ_FLOW_TX_PAUSE);
		}
	}
}

LD_TASK void task_EPCAN_IN(void *pData)
{
	CAN_msg_t		msg;

	do {
		while (xQueueReceive(local.in_queue, &msg, portMAX_DELAY) == pdTRUE) {

			if (net.log_MODE != EPCAN_LOG_DISABLED) {

				EPCAN_log_msg(&msg, "IN");
			}

			EPCAN_message_IN(&msg);
		}
	}
	while (1);
}

LD_TASK void task_EPCAN_TX(void *pData)
{
	CAN_msg_t		msg;
	char			xbyte;

	do {
		msg.len = 0;

		while (xQueueReceive(local.tx_queue, &xbyte, (TickType_t) 10) == pdTRUE) {

			msg.payload[msg.len++] = xbyte;

			if (msg.len >= 8) {

				break;
			}
		}

		if (msg.len > 0) {

			msg.ID = EPCAN_ID(net.node_ID, EPCAN_NODE_TX);

			EPCAN_send_msg(&msg);

			/* Do not send messages too frequently especially if
			 * you were asked to.
			 * */
			if (local.flow_tx_paused != 0) {

				vTaskDelay((TickType_t) 10);

				local.flow_tx_paused = 0;
			}
			else {
				vTaskDelay((TickType_t) 1);
			}
		}
	}
	while (1);
}

void EPCAN_putc(int c)
{
	char		xbyte = (char) c;

	GPIO_set_HIGH(GPIO_LED_ALERT);

	xQueueSendToBack(local.tx_queue, &xbyte, portMAX_DELAY);

	GPIO_set_LOW(GPIO_LED_ALERT);
}

extern QueueHandle_t USART_public_rx_queue();

void EPCAN_startup()
{
	/* Produce UNIQUE ID.
	 * */
	local.UID = RNG_make_UID();

	/* Allocate queues.
	 * */
	local.in_queue = xQueueCreate(10, sizeof(CAN_msg_t));
	local.rx_queue = USART_public_rx_queue();
	local.tx_queue = xQueueCreate(80, sizeof(char));
	local.remote_queue = xQueueCreate(40, sizeof(char));
	local.log_queue = xQueueCreate(320, sizeof(char));
	local.net_queue = xQueueCreate(1, sizeof(int));

	/* Allocate semaphore.
	 * */
	local.log_mutex = xSemaphoreCreateMutex();

	/* Create EPCAN tasks.
	 * */
	xTaskCreate(task_EPCAN_IN, "EPCAN_IN", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_EPCAN_TX, "EPCAN_TX", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_EPCAN_NET, "EPCAN_NET", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_EPCAN_LOG, "EPCAN_LOG", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	CAN_startup();

	EPCAN_filter_ID();
}

void EPCAN_filter_ID()
{
	int			N;

	if (net.log_MODE == EPCAN_LOG_PROMISCUOUS) {

		CAN_filter_ID(0, 0, 1, 0);
	}
	else {
		CAN_filter_ID(0, 0, EPCAN_FILTER_NETWORK, EPCAN_FILTER_NETWORK);
	}

	if (net.node_ID != 0) {

		CAN_filter_ID(1, 0, EPCAN_ID(net.node_ID, EPCAN_NODE_REQ), EPCAN_FILTER_MATCH);
		CAN_filter_ID(2, 0, EPCAN_ID(0, EPCAN_NODE_ACK), EPCAN_ID(0, 31));
		CAN_filter_ID(3, 0, EPCAN_ID(net.node_ID, EPCAN_NODE_RX), EPCAN_FILTER_MATCH);
		CAN_filter_ID(4, 0, 0, 0);
	}
	else {
		CAN_filter_ID(1, 0, 0, 0);
		CAN_filter_ID(2, 0, EPCAN_ID(0, EPCAN_NODE_ACK), EPCAN_ID(0, 31));
		CAN_filter_ID(3, 0, 0, 0);
		CAN_filter_ID(4, 0, 0, 0);
	}

	for (N = 0; N < EPCAN_PIPE_MAX; ++N) {

		if (net.ep[N].MODE == EPCAN_PIPE_INCOMING) {

			CAN_filter_ID(20 + N, 1, net.ep[N].ID, EPCAN_FILTER_MATCH);
		}
		else if (net.ep[N].MODE == EPCAN_PIPE_OUTGOING_INJECTED) {

			CAN_filter_ID(20 + N, 1, net.ep[N].clock_ID, EPCAN_FILTER_MATCH);
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

	msg.ID = EPCAN_ID_NET_SURVEY;
	msg.len = 0;

	EPCAN_send_msg(&msg);

	/* Wait for all nodes to reply.
	 * */
	vTaskDelay((TickType_t) 50);

	if (local.node[0].UID != 0) {

		printf("UID         node_ID" EOL);

		for (N = 0; N < EPCAN_NODES_MAX; ++N) {

			if (local.node[N].UID != 0) {

				printf("%8x    ", local.node[N].UID);

				if (local.node[N].node_ID != 0) {

					printf("%i", local.node[N].node_ID);
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

		EPCAN_filter_ID();

		printf("local assigned to %i" EOL, net.node_ID);
	}

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID == 0) {

			local.node[N].node_ID = local_node_assign_ID();

			if (local.node[N].node_ID != 0) {

				msg.ID = EPCAN_ID_NET_ASSIGN;
				msg.len = 5;

				* (uint32_t *) &msg.payload[0] = local.node[N].UID;
				msg.payload[4] = local.node[N].node_ID;

				EPCAN_send_msg(&msg);

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

	for (N = 0; N < EPCAN_NODES_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID != 0) {

			if (local.node[N].node_ID == node_ID
					|| node_ID == 0) {

				local.node[N].node_ID = 0;

				msg.ID = EPCAN_ID_NET_ASSIGN;
				msg.len = 5;

				* (uint32_t *) &msg.payload[0] = local.node[N].UID;
				msg.payload[4] = 0;

				EPCAN_send_msg(&msg);

				printf("UID %8x revoked" EOL, local.node[N].UID);
			}
		}
	}
}

LD_TASK void task_epcan_REMOTE(void *pData)
{
	CAN_msg_t		msg;
	int			xbyte;

	do {
		msg.len = 0;

		while (xQueueReceive(local.remote_queue, &xbyte, (TickType_t) 10) == pdTRUE) {

			msg.payload[msg.len++] = xbyte;

			if (msg.len >= 8) {

				break;
			}
		}

		if (msg.len > 0) {

			msg.ID = EPCAN_ID(local.remote_node_ID, EPCAN_NODE_RX);

			EPCAN_send_msg(&msg);
		}
	}
	while (1);
}

void EPCAN_remote_putc(int c)
{
	char		xbyte = (char) c;

	xQueueSendToBack(local.remote_queue, &xbyte, portMAX_DELAY);
}

SH_DEF(net_node_remote)
{
	TaskHandle_t		xHandle;
	int			node_ID;
	char			xbyte;

	io_ops_t		ops = {

		.getc = NULL,
		.putc = &EPCAN_remote_putc
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
		CAN_filter_ID(4, 0, EPCAN_ID(local.remote_node_ID, EPCAN_NODE_TX), EPCAN_FILTER_MATCH);

		/* Create task to outgoing message packaging.
		 * */
		xTaskCreate(task_epcan_REMOTE, "REMOTE", configMINIMAL_STACK_SIZE,
				NULL, 2, &xHandle);

		xputs(&ops, EOL);

		do {
			xbyte = getc();

			EPCAN_remote_putc(xbyte);

			if (xbyte == K_EOT)
				break;
		}
		while (1);

		/* Drop incoming connection immediately.
		 * */
		CAN_filter_ID(4, 0, 0, 0);

		local.remote_node_ID = 0;

		/* Wait for task_epcan_REMOTE to transmit the last message.
		 * */
		vTaskDelay((TickType_t) 50);

		vTaskDelete(xHandle);

		/* Do pretty line feed.
		 * */
		puts(EOL);
	}
}

