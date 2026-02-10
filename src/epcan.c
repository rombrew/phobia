#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "hal/hal.h"

#include "epcan.h"
#include "libc.h"
#include "main.h"
#include "regfile.h"
#include "shell.h"

#define EPCAN_NODE_MAX			30

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

	/* Remote node ID we connected to.
	 * */
	int			remote_node_ID;

	/* TX pause notice (flow control).
	 * */
	int			flow_tx_paused;

	/* Remote nodes MAP.
	 * */
	struct {

		uint32_t	UID;

		int		node_ID;
	}
	node[EPCAN_NODE_MAX];

	/* Waiting for ACK from this node ID.
	 * */
	int			node_ACK_ID;

	/* Data payload from the ACK.
	 * */
	int			reg_ID;
	rval_t			reg_DATA;
}
epcan_local_t;

epcan_t				net;

static epcan_local_t		local;

static int
EPCAN_pipe_INCOMING(epcan_pipe_t *ep, const CAN_msg_t *msg)
{
	int		result = HAL_FAULT;

	switch (ep->PAYLOAD) {

		case EPCAN_PAYLOAD_FLOAT:

			if (msg->len == 4U) {

				ep->reg_DATA[0] = ep->range[0] + msg->payload.f[0] * (ep->range[1] - ep->range[0]);

				if (ep->reg_ID[0] != ID_NULL) {

					reg_SET_F(ep->reg_ID[0], ep->reg_DATA[0]);
				}

				result = HAL_OK;
			}
			break;

		case EPCAN_PAYLOAD_INT16:

			if (msg->len == 2U) {

				ep->reg_DATA[0] = ep->range[0] + (float) msg->payload.s[0]
					* (ep->range[1] - ep->range[0]) * (1.f / 65535.f);

				if (ep->reg_ID[0] != ID_NULL) {

					reg_SET_F(ep->reg_ID[0], ep->reg_DATA[0]);
				}

				result = HAL_OK;
			}
			break;

		case EPCAN_PAYLOAD_TWO_FLOAT:

			if (msg->len == 8U) {

				ep->reg_DATA[0] = ep->range[0] + msg->payload.f[0] * (ep->range[1] - ep->range[0]);
				ep->reg_DATA[1] = ep->range[0] + msg->payload.f[1] * (ep->range[1] - ep->range[0]);

				if (ep->reg_ID[0] != ID_NULL) {

					reg_SET_F(ep->reg_ID[0], ep->reg_DATA[0]);
				}

				if (ep->reg_ID[1] != ID_NULL) {

					reg_SET_F(ep->reg_ID[1], ep->reg_DATA[1]);
				}

				result = HAL_OK;
			}
			break;

		case EPCAN_PAYLOAD_TWO_INT16:

			if (msg->len == 4U) {

				ep->reg_DATA[0] = ep->range[0] + (float) msg->payload.s[0]
					* (ep->range[1] - ep->range[0]) * (1.f / 65535.f);
				ep->reg_DATA[1] = ep->range[0] + (float) msg->payload.s[1]
					* (ep->range[1] - ep->range[0]) * (1.f / 65535.f);

				if (ep->reg_ID[0] != ID_NULL) {

					reg_SET_F(ep->reg_ID[0], ep->reg_DATA[0]);
				}

				if (ep->reg_ID[1] != ID_NULL) {

					reg_SET_F(ep->reg_ID[1], ep->reg_DATA[1]);
				}

				result = HAL_OK;
			}
			break;

		default: break;
	}

	return result;
}

static void
EPCAN_pipe_OUTGOING(epcan_pipe_t *ep)
{
	CAN_msg_t		msg;
	float			fpay;

	if (ep->reg_ID[0] != ID_NULL) {

		ep->reg_DATA[0] = reg_GET_F(ep->reg_ID[0]);
	}

	msg.ID = EPCAN_ID_OFFSET(ep->ID);

	switch (ep->PAYLOAD) {

		case EPCAN_PAYLOAD_FLOAT:

			msg.len = 4U;
			msg.payload.f[0] = (ep->reg_DATA[0] - ep->range[0])
					/ (ep->range[1] - ep->range[0]);
			break;

		case EPCAN_PAYLOAD_INT16:

			msg.len = 2U;

			fpay = (ep->reg_DATA[0] - ep->range[0]) / (ep->range[1] - ep->range[0]);
			fpay = (fpay < 0.f) ? 0.f : (fpay > 1.f) ? 1.f : fpay;

			msg.payload.s[0] = (uint16_t) (fpay * 65535.f);
			break;

		case EPCAN_PAYLOAD_TWO_FLOAT:

			if (ep->reg_ID[1] != ID_NULL) {

				ep->reg_DATA[1] = reg_GET_F(ep->reg_ID[1]);
			}

			msg.len = 8U;
			msg.payload.f[0] = (ep->reg_DATA[0] - ep->range[0])
					/ (ep->range[1] - ep->range[0]);
			msg.payload.f[1] = (ep->reg_DATA[1] - ep->range[0])
					/ (ep->range[1] - ep->range[0]);
			break;

		case EPCAN_PAYLOAD_TWO_INT16:

			if (ep->reg_ID[1] != ID_NULL) {

				ep->reg_DATA[1] = reg_GET_F(ep->reg_ID[1]);
			}

			msg.len = 4U;

			fpay = (ep->reg_DATA[0] - ep->range[0]) / (ep->range[1] - ep->range[0]);
			fpay = (fpay < 0.f) ? 0.f : (fpay > 1.f) ? 1.f : fpay;

			msg.payload.s[0] = (uint16_t) (fpay * 65535.f);

			fpay = (ep->reg_DATA[1] - ep->range[0]) / (ep->range[1] - ep->range[0]);
			fpay = (fpay < 0.f) ? 0.f : (fpay > 1.f) ? 1.f : fpay;

			msg.payload.s[1] = (uint16_t) (fpay * 65535.f);
			break;

		default: break;
	}

	if (CAN_send_msg(&msg) == HAL_OK) {

		ep->tx_flag = 0;
	}
}

static void
EPCAN_pipe_message_IN(const CAN_msg_t *msg)
{
	epcan_pipe_t		*ep;
	int			N;

	for (N = 0; N < EPCAN_EP_MAX; ++N) {

		ep = &net.ep[N];

		if (		ep->MODE == EPCAN_PIPE_INCOMING
				&& EPCAN_ID_OFFSET(ep->ID) == msg->ID) {

			if (EPCAN_pipe_INCOMING(ep, msg) == HAL_OK) {

				ep->tx_clock = 0;

				if (		ep->ACTIVE == PM_ENABLED
						&& pm.lu_MODE == PM_LU_DISABLED) {

					ep->ACTIVE = PM_DISABLED;
				}

				if (		ep->STARTUP == PM_ENABLED
						&& ep->ACTIVE != PM_ENABLED
						&& pm.lu_MODE == PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_STARTUP;

					ep->ACTIVE = PM_ENABLED;
				}
			}
		}
		else if (	ep->MODE == EPCAN_PIPE_OUTGOING_INJECTED
				&& EPCAN_ID_OFFSET(net.inject_ID) == msg->ID) {

			ep->tx_flag = 1;

			EPCAN_pipe_OUTGOING(ep);
		}
	}
}

void EPCAN_pipe_PERIODIC()
{
	epcan_pipe_t		*ep;
	int			N;

	for (N = 0; N < EPCAN_EP_MAX; ++N) {

		ep = &net.ep[N];

		if (		ep->MODE == EPCAN_PIPE_INCOMING
				&& ep->ACTIVE == PM_ENABLED) {

			ep->tx_clock++;

			if (ep->tx_clock >= net.timeout_EP) {

				if (pm.lu_MODE != PM_LU_DISABLED) {

					pm.fsm_req = PM_STATE_LU_SHUTDOWN;
				}

				ep->ACTIVE = PM_DISABLED;
			}
		}
		else if (ep->MODE == EPCAN_PIPE_OUTGOING_PERIODIC) {

			ep->tx_clock++;

			if (ep->tx_clock >= ep->rate) {

				ep->tx_clock = 0;

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

	if (		hal.CAN_msg.ID >= EPCAN_ID_OFFSET(0U)
			&& hal.CAN_msg.ID < EPCAN_ID_CAN(1U, 0U)) {

		EPCAN_pipe_message_IN(&hal.CAN_msg);
	}

	if (		hal.CAN_msg.ID >= EPCAN_ID_CAN(1U, 0U)
			&& hal.CAN_msg.ID < EPCAN_ID_CAN(32U, 0U)) {

		xQueueSendToBackFromISR(local.in_queue, &hal.CAN_msg, &xWoken);
	}
	else if (net.log_MSG == EPCAN_LOG_PROMISCUOUS) {

		xQueueSendToBackFromISR(local.in_queue, &hal.CAN_msg, &xWoken);
	}

	portYIELD_FROM_ISR(xWoken);
}

static void
local_node_discard()
{
	int			N;

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		local.node[N].UID = 0U;
		local.node[N].node_ID = 0;
	}
}

static void
local_node_insert(uint32_t UID, int node_ID)
{
	int			N;

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		if (local.node[N].UID == UID) {

			/* Update node ID.
			 * */
			local.node[N].node_ID = node_ID;

			return ;
		}
	}

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		if (local.node[N].UID == 0U) {

			/* Insert NODE.
			 * */
			local.node[N].UID = UID;
			local.node[N].node_ID = node_ID;

			return ;
		}
	}
}

static int
local_node_valid_remote(int node_ID)
{
	int		N;

	if (node_ID == 0) {

		return 0;
	}

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		if (		node_ID == local.node[N].node_ID
				&& node_ID != net.node_ID) {

			return 1;
		}
	}

	return 0;
}

static int
local_node_assign_ID()
{
	int		node_ID, found_ID = 0;

	for (node_ID = 1; node_ID < EPCAN_NODE_MAX; ++node_ID) {

		if (		local_node_valid_remote(node_ID) == 0
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
	int		list_ID[EPCAN_NODE_MAX];
	int		node_ID, found_ID = 0, N = 0;

	for (node_ID = 1; node_ID <= EPCAN_NODE_MAX; ++node_ID) {

		if (		local_node_valid_remote(node_ID) == 0
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

	if (		uxQueueSpacesAvailable(local.log_queue) >= 70
			&& xSemaphoreTake(local.log_mutex, (TickType_t) 10) == pdTRUE) {

		if (local.log_skipped != 0) {

			xprintf(&ops, "(skipped %i)" EOL, local.log_skipped);

			local.log_skipped = 0;
		}

		if (msg->ID >= CAN_EXTENID_MIN) {

			xprintf(&ops, "%s %8x %i", label, msg->ID, msg->len);
		}
		else {
			xprintf(&ops, "%s %4x %i", label, msg->ID, msg->len);
		}

		for (N = 0; N < msg->len; ++N) {

			xprintf(&ops, " %2x", msg->payload.b[N]);
		}

		xputs(&ops, EOL);

		xSemaphoreGive(local.log_mutex);
	}
	else {
		local.log_skipped += 1;
	}
}

void EPCAN_send_msg(CAN_msg_t *msg)
{
	int			N = 0;

	do {
		if (CAN_send_msg(msg) == HAL_OK) {

			if (net.log_MSG != EPCAN_LOG_DISABLED) {

				EPCAN_log_msg(msg, "OUT");
			}

			break;
		}

		N++;

		if (N >= 20) {

			if (net.log_MSG != EPCAN_LOG_DISABLED) {

				EPCAN_log_msg(msg, "DROP");
			}

			break;
		}

		/* Wait until one of the mailboxes becomes free.
		 * */
		taskYIELD();
	}
	while (1);
}

static void
EPCAN_node_ACK_NET(int node_ACK)
{
	CAN_msg_t		msg;
	int			node_ID;

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

			/* Get a random node ID from the list
			 * of free ones.
			 * */
			node_ID = local_node_random_ID();

			/* Also we delay the transmission by
			 * random value to reduce the chance of
			 * a data collision.
			 * */
			vTaskDelay((TickType_t) (urand() % 27U));
		}

		msg.ID = EPCAN_ID_CAN(node_ID, EPCAN_NODE_ACK);
		msg.len = 8U;

		msg.payload.b[0] = node_ACK;
		msg.payload.b[1] = net.node_ID;
		msg.payload.b[2] = 0U;
		msg.payload.b[3] = node_ID;

		msg.payload.l[1] = local.UID;

		EPCAN_send_msg(&msg);
	}
}

static void
EPCAN_node_ACK_DATA(int node_ACK, int reg_ID)
{
	CAN_msg_t		msg;

	if (reg_ID > 0 && reg_ID < ID_MAX) {

		msg.ID = EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_ACK);
		msg.len = 8U;

		msg.payload.b[0] = node_ACK;
		msg.payload.b[1] = net.node_ID;

		msg.payload.s[1] = reg_ID;

		reg_GET(reg_ID, (rval_t *) &msg.payload.l[1]);

		EPCAN_send_msg(&msg);
	}
}

static void
EPCAN_node_REQ_remote(int node_REQ)
{
	CAN_msg_t		msg;

	msg.ID = EPCAN_ID_CAN(local.remote_node_ID, EPCAN_NODE_REQ);
	msg.len = 1U;

	msg.payload.b[0] = node_REQ;

	EPCAN_send_msg(&msg);
}

static void
EPCAN_message_IN(const CAN_msg_t *msg)
{
	int			N;

	/* Network functions.
	 * */
	if (		msg->ID == EPCAN_ID_NET_SURVEY
			&& msg->len == 5U) {

		local_node_discard();
		local_node_insert(msg->payload.l[0], msg->payload.b[4]);

		EPCAN_node_ACK_NET(EPCAN_ACK_NETWORK_REPLY);
	}
	else if (	msg->ID == EPCAN_ID_NET_ASSIGN
			&& msg->len == 5U) {

		if (msg->payload.l[0] == local.UID) {

			/* Accept new node ID.
			 * */
			net.node_ID = msg->payload.b[4];

			EPCAN_bind();
		}
		else {
			local_node_insert(msg->payload.l[0], msg->payload.b[4]);
		}
	}

	/* Node functions.
	 * */
	else if (msg->ID == EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_REQ)) {

		if (		msg->payload.b[0] == EPCAN_REQ_FLOW_TX_PAUSE
				&& msg->len == 1U) {

			local.flow_tx_paused = 1;
		}
		else if (	msg->payload.b[0] == EPCAN_REQ_REG_GET
				&& msg->len == 4U) {

			EPCAN_node_ACK_DATA(EPCAN_ACK_REG_DATA, msg->payload.s[1]);
		}
		else if (	msg->payload.b[0] == EPCAN_REQ_REG_SET
				&& msg->len == 8U) {

			reg_SET(msg->payload.s[1], (rval_t *) &msg->payload.l[1]);
		}
	}
	else if (EPCAN_GET_FUNC(msg->ID) == EPCAN_NODE_ACK) {

		if (            msg->payload.b[0] == EPCAN_ACK_NETWORK_REPLY
				&& msg->len == 8U) {

			local_node_insert(msg->payload.l[1], msg->payload.b[1]);
		}
		else if (	msg->payload.b[0] == EPCAN_ACK_REG_DATA
				&& msg->len == 8U) {

			if (		msg->payload.b[1] == local.node_ACK_ID
					&& local.node_ACK_ID != 0) {

				local.reg_ID = msg->payload.s[1];
				local.reg_DATA.f = msg->payload.f[1];

				hal_memory_fence();

				local.node_ACK_ID = 0;
			}
		}
	}
	else if (msg->ID == EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_RX)) {

		for (N = 0; N < msg->len; ++N) {

			xQueueSendToBack(local.rx_queue, &msg->payload.b[N], (TickType_t) 0);
		}

		IODEF_TO_CAN();
	}
	else if (msg->ID == EPCAN_ID_CAN(local.remote_node_ID, EPCAN_NODE_TX)) {

		for (N = 0; N < msg->len; ++N) {

			/* Remote NODE output via LOG.
			 * */
			EPCAN_log_putc(msg->payload.b[N]);
		}

		if (uxQueueSpacesAvailable(local.log_queue) < 24) {

			/* Notify remote node about overflow.
			 * */
			EPCAN_node_REQ_remote(EPCAN_REQ_FLOW_TX_PAUSE);
		}
	}
}

LD_TASK void task_EPCAN_IN(void *pData)
{
	CAN_msg_t		msg;

	do {
		while (xQueueReceive(local.in_queue, &msg, portMAX_DELAY) == pdTRUE) {

			if (net.log_MSG != EPCAN_LOG_DISABLED) {

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
		msg.len = 0U;

		while (xQueueReceive(local.tx_queue, &xbyte, (TickType_t) 10) == pdTRUE) {

			msg.payload.b[msg.len++] = xbyte;

			if (msg.len >= 8U) {

				break;
			}
		}

		if (msg.len > 0U) {

			msg.ID = EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_TX);

			EPCAN_send_msg(&msg);

			/* Do not send messages too frequently
			 * especially if you were asked to.
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

	/* Allocate semaphore.
	 * */
	local.log_mutex = xSemaphoreCreateMutex();

	/* Create EPCAN tasks.
	 * */
	xTaskCreate(task_EPCAN_IN, "EPCAN_IN", configDEFAULT_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(task_EPCAN_TX, "EPCAN_TX", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(task_EPCAN_LOG, "EPCAN_LOG", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	CAN_startup();

	EPCAN_bind();
}

void EPCAN_bind()
{
	int			N;

	if (net.log_MSG == EPCAN_LOG_PROMISCUOUS) {

		CAN_bind_ID(0, 0, 1U, 0U);
	}
	else {
		CAN_bind_ID(0, 0, EPCAN_ID_CAN(31U, 0U), EPCAN_MATCH_NET);
	}

	if (net.node_ID != 0) {

		CAN_bind_ID(1, 0, EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_REQ), EPCAN_MATCH_ID_CAN);
		CAN_bind_ID(2, 0, EPCAN_ID_CAN(0, EPCAN_NODE_ACK), EPCAN_MATCH_FUNC);
		CAN_bind_ID(3, 0, EPCAN_ID_CAN(net.node_ID, EPCAN_NODE_RX), EPCAN_MATCH_ID_CAN);
		CAN_bind_ID(4, 0, 0U, 0U);
	}
	else {
		CAN_bind_ID(1, 0, 0U, 0U);
		CAN_bind_ID(2, 0, EPCAN_ID_CAN(0, EPCAN_NODE_ACK), EPCAN_MATCH_FUNC);
		CAN_bind_ID(3, 0, 0U, 0U);
		CAN_bind_ID(4, 0, 0U, 0U);
	}

	for (N = 0; N < EPCAN_EP_MAX; ++N) {

		if (net.ep[N].MODE == EPCAN_PIPE_INCOMING) {

			CAN_bind_ID(10 + N, 1, EPCAN_ID_OFFSET(net.ep[N].ID), EPCAN_MATCH_ID_CAN);
		}
		else if (net.ep[N].MODE == EPCAN_PIPE_OUTGOING_INJECTED) {

			CAN_bind_ID(10 + N, 1, EPCAN_ID_OFFSET(net.inject_ID), EPCAN_MATCH_ID_CAN);
		}
		else {
			CAN_bind_ID(10 + N, 1, 0U, 0U);
		}
	}
}

#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_survey)
{
	CAN_msg_t		msg;
	int			N;

	local_node_discard();

	msg.ID = EPCAN_ID_NET_SURVEY;
	msg.len = 5U;

	msg.payload.l[0] = local.UID;
	msg.payload.b[4] = net.node_ID;

	EPCAN_send_msg(&msg);

	/* Wait for all nodes to reply.
	 * */
	vTaskDelay((TickType_t) 50);

	if (local.node[0].UID != 0) {

		printf("UID         node_ID" EOL);

		for (N = 0; N < EPCAN_NODE_MAX; ++N) {

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
#endif /* HW_HAVE_NETWORK_EPCAN */

#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_assign)
{
	CAN_msg_t		msg;
	int			N;

	if (net.node_ID == 0) {

		net.node_ID = local_node_assign_ID();

		EPCAN_bind();

		printf("local assigned to %i" EOL, net.node_ID);
	}

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID == 0) {

			local.node[N].node_ID = local_node_assign_ID();

			if (local.node[N].node_ID != 0) {

				msg.ID = EPCAN_ID_NET_ASSIGN;
				msg.len = 5U;

				msg.payload.l[0] = local.node[N].UID;
				msg.payload.b[4] = local.node[N].node_ID;

				EPCAN_send_msg(&msg);

				printf("UID %8x assigned to %i" EOL,
						local.node[N].UID,
						local.node[N].node_ID);
			}
		}
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_revoke)
{
	CAN_msg_t		msg;
	int			N, node_ID;

	if (stoi(&node_ID, s) == NULL) {

		return ;
	}

	for (N = 0; N < EPCAN_NODE_MAX; ++N) {

		if (local.node[N].UID != 0 && local.node[N].node_ID != 0) {

			if (		local.node[N].node_ID == node_ID
					|| node_ID == 0) {

				local.node[N].node_ID = 0;

				msg.ID = EPCAN_ID_NET_ASSIGN;
				msg.len = 5U;

				msg.payload.l[0] = local.node[N].UID;
				msg.payload.b[4] = 0;

				EPCAN_send_msg(&msg);

				printf("UID %8x revoked" EOL, local.node[N].UID);
			}
		}
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

LD_TASK void task_EPCAN_REMOTE(void *pData)
{
	CAN_msg_t		msg;
	char			xbyte;

	do {
		msg.len = 0U;

		while (xQueueReceive(local.remote_queue, &xbyte, (TickType_t) 10) == pdTRUE) {

			msg.payload.b[msg.len++] = xbyte;

			if (msg.len >= 8U) {

				break;
			}
		}

		if (msg.len > 0U) {

			msg.ID = EPCAN_ID_CAN(local.remote_node_ID, EPCAN_NODE_RX);

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

#ifdef HW_HAVE_NETWORK_EPCAN
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

		if (local_node_valid_remote(node_ID) != 0) {

			local.remote_node_ID = node_ID;
		}
		else {
			printf("No valid remote node ID" EOL);
		}
	}

	if (local.remote_node_ID != 0) {

		/* Do listen to incoming messages from remote node.
		 * */
		CAN_bind_ID(4, 0, EPCAN_ID_CAN(local.remote_node_ID, EPCAN_NODE_TX), EPCAN_MATCH_ID_CAN);

		/* Create task for outgoing message packaging.
		 * */
		xTaskCreate(task_EPCAN_REMOTE, "EPCAN_REMOTE", configMINIMAL_STACK_SIZE, NULL, 2, &xHandle);

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
		CAN_bind_ID(4, 0, 0U, 0U);

		local.remote_node_ID = 0;

		/* Wait for task_EPCAN_REMOTE to transmit the last message.
		 * */
		vTaskDelay((TickType_t) 50);

		vTaskDelete(xHandle);

		/* Do pretty line feed.
		 * */
		puts(EOL);
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

#ifdef HW_HAVE_NETWORK_EPCAN
SH_DEF(net_node_data)
{
	CAN_msg_t		msg;

	int			reg_ID = 0, node_ID, N;

	if (stoi(&node_ID, s) != NULL) {

		if (local_node_valid_remote(node_ID) != 0) {

			s = sh_next_arg(s);

			if (stoi(&reg_ID, s) != NULL) {

				/* This ID will only be validated
				 * on the remote side. */
			}
		}
		else {
			printf("No valid remote node ID" EOL);
		}
	}

	if (reg_ID != 0) {

		rval_t			rval;

		s = sh_next_arg(s);

		if (stoi(&rval.i, s) != NULL) {

			local.reg_ID = reg_ID;
			local.reg_DATA = rval;
		}
		else if (htoi(&rval.i, s) != NULL) {

			local.reg_ID = reg_ID;
			local.reg_DATA = rval;
		}
		else if (stof(&rval.f, s) != NULL) {

			local.reg_ID = reg_ID;
			local.reg_DATA = rval;
		}
		else {
			local.reg_ID = 0;
		}

		msg.ID = EPCAN_ID_CAN(node_ID, EPCAN_NODE_REQ);

		if (local.reg_ID == reg_ID) {

			msg.len = 8U;

			msg.payload.b[0] = EPCAN_REQ_REG_SET;
			msg.payload.b[1] = node_ID;

			msg.payload.s[1] = local.reg_ID;
			msg.payload.f[1] = local.reg_DATA.f;

			EPCAN_send_msg(&msg);
		}

		msg.len = 4U;

		msg.payload.b[0] = EPCAN_REQ_REG_GET;
		msg.payload.b[1] = node_ID;

		msg.payload.s[1] = reg_ID;

		local.node_ACK_ID = node_ID;
		local.reg_ID = 0;

		hal_memory_fence();

		EPCAN_send_msg(&msg);

		N = 0;

		do {
			vTaskDelay((TickType_t) 10);

			N++;

			if (N >= 10)
				break;
		}
		while (local.node_ACK_ID != 0);

		if (		local.node_ACK_ID == 0
				&& local.reg_ID == reg_ID) {

			printf("node/%i [%-3i] = %4g (%8x)" EOL, node_ID, reg_ID,
					&local.reg_DATA.f, local.reg_DATA.i);
		}
		else {
			printf("No remote ACK" EOL);
		}
	}
}
#endif /* HW_HAVE_NETWORK_EPCAN */

