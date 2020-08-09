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
	IFCAN_ID_NODE_RX		= 0,
	IFCAN_ID_NODE_TX		= 1,
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
	IFCAN_ACK_FLASH_CRC32_INVALID,
	IFCAN_ACK_FLASH_SELFUPDATE,
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

	/* FLASH update across network.
	 * */
	u32_t			flash_sizeof;
	u32_t			flash_crc32;
	u32_t			*flash_long;
	u32_t			flash_accepted;

	/* Remote node ID we connected to.
	 * */
	int			remote_node_ID;

	struct {

		u32_t		UID;
		u16_t		node_ID;
		u16_t		node_ACK;
	}
	node[IFCAN_NODES_MAX];
}
ifcan_local_t;

extern long			ld_begin_vectors;

ifcan_t				can;
static ifcan_local_t		local;

void CAN_IRQ()
{
	BaseType_t		xWoken = pdFALSE;

	if (hal.CAN_msg.ID >= IFCAN_ID_NODE_BASE) {

		xQueueSendToBackFromISR(local.queue_IN, &hal.CAN_msg, &xWoken);
	}
	else {
		// TODO: data stream handling
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
		local.node[N].node_ACK = 0;
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
IFCAN_log_msg(CAN_msg_t *msg)
{
	int		N;

	io_ops_t	ops = {

		.getc = NULL,
		.putc = &IFCAN_log_putc
	};

	xprintf(&ops, "%i %i", msg->ID, msg->len);

	for (N = 0; N < msg->len; ++N) {

		xprintf(&ops, " %2x", msg->payload[N]);
	}

	xputs(&ops, EOL);
}

static void
IFCAN_send_msg(CAN_msg_t *msg)
{
	int			N = 0;

	while (CAN_send_msg(msg) != CAN_TX_OK) {

		N++;

		if (N >= 3) {

			break;
		}

		/* Try again later.
		 * */
		vTaskDelay((TickType_t) 1);
	}

	if (can.log_MODE != IFCAN_LOG_DISABLED) {

		IFCAN_log_msg(msg);
	}
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

	return (flash_sizeof == local.flash_sizeof
			&& crc32b(flash, flash_sizeof)
			== local.flash_crc32) ? 1 : 0;
}

static int
IFCAN_flash_erase()
{
	void			*flash_end;
	int			N, rc = 0;

	if (local.flash_sizeof >= 1024UL) {

		flash_end = (void *) (flash_ram_map[0] + local.flash_sizeof);

		if ((u32_t) flash_end > flash_ram_map[FLASH_SECTOR_MAX - 1]) {

			rc = 0;
		}
		else {
			/* Relocate configuration block.
			 * */
			rc = flash_block_relocate();

			if (rc != 0) {

				for (N = 0; N < FLASH_SECTOR_MAX - 1; ++N) {

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
IFCAN_message_IN(const CAN_msg_t *msg)
{
	int			N;
	char			xC;

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

		local.flash_sizeof = * (u32_t *) &msg->payload[0];
		local.flash_crc32 = * (u32_t *) &msg->payload[4];

		if (IFCAN_flash_is_up_to_date() != 0) {

			IFCAN_flash_ACK(IFCAN_ACK_FLASH_UP_TO_DATE);
		}
		else {
			IFCAN_flash_ACK(IFCAN_ACK_FLASH_WAIT_FOR_ERASE);

			if (IFCAN_flash_erase() != 0) {

				local.flash_long = (u32_t *) flash_ram_map[0];
				local.flash_accepted = 0;

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
		local.flash_accepted += msg->len;

		if (local.flash_accepted >= local.flash_sizeof) {

			if (crc32b((u32_t *) flash_ram_map[0], local.flash_sizeof) == local.flash_crc32) {

				IFCAN_flash_ACK(IFCAN_ACK_FLASH_SELFUPDATE);

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
	else if (msg->ID == can.node_ID + IFCAN_ID_NODE_RX) {

		for (N = 0; N < msg->len; ++N) {

			xC = msg->payload[N];

			xQueueSendToBack(local.queue_RX, &xC, (TickType_t) 0);
		}

		IODEF_TO_CAN();
	}
	else if (msg->ID == local.remote_node_ID + IFCAN_ID_NODE_TX) {

		for (N = 0; N < msg->len; ++N) {

			/* Remote NODE output via LOG.
			 * */
			IFCAN_log_putc(msg->payload[N]);
		}
	}
}

void task_IFCAN_IN(void *pData)
{
	CAN_msg_t		msg;

	do {
		while (xQueueReceive(local.queue_IN, &msg, portMAX_DELAY) == pdTRUE) {

			if (can.log_MODE != IFCAN_LOG_DISABLED) {

				IFCAN_log_msg(&msg);
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

			/* Do not send messages too frequently.
			 * */
			vTaskDelay((TickType_t) 2);
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
	local.queue_IN = xQueueCreate(20, sizeof(CAN_msg_t));
	local.queue_RX = USART_queue_RX();
	local.queue_TX = xQueueCreate(80, sizeof(char));
	local.queue_EX = xQueueCreate(40, sizeof(char));
	local.queue_LOG = xQueueCreate(800, sizeof(char));

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

		CAN_filter_ID(0, 0, 0, 0);
	}
	else {
		CAN_filter_ID(0, 0, IFCAN_FILTER_NETWORK, IFCAN_FILTER_NETWORK);
	}

	if (can.node_ID >= IFCAN_ID_NODE_BASE) {

		CAN_filter_ID(1, 0, can.node_ID + IFCAN_ID_NODE_RX, IFCAN_FILTER_MATCH);
		CAN_filter_ID(2, 0, 0, 0);
	}

	N = 3;
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
	vTaskDelay((TickType_t) 10);

	if (local.node[0].UID != 0) {

		printf("UID         node_ID" EOL);

		for (N = 0; N < IFCAN_NODES_MAX; ++N) {

			if (local.node[N].UID != 0) {

				printf("%8x    ", local.node[N].UID);

				if (local.node[N].node_ID >= IFCAN_ID_NODE_BASE) {

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

static void
IFCAN_flash_DATA(u32_t *flash, u32_t flash_sizeof)
{
	u32_t			*flash_end;
	CAN_msg_t		msg;
	int			blk_N;

	msg.ID = IFCAN_ID_FLASH_DATA;
	msg.len = 8;

	flash_end = (u32_t *) ((u32_t) flash + flash_sizeof);
	blk_N = 0;

	puts("Flash ");

	while (flash < flash_end) {

		* (u32_t *) &msg.payload[0] = *flash++;
		* (u32_t *) &msg.payload[4] = *flash++;

		/* Note that bandwidth is limited because of pause before
		 * retransmission is quite large. Also we should keep in mind
		 * that it could take a time to program flash on remote side.
		 * */
		IFCAN_send_msg(&msg);

		blk_N += 8;

		if (blk_N >= 8192) {

			blk_N = 0;

			/* Display progress.
			 * */
			putc('.');
		}
	}

	puts(" Done" EOL);
}

SH_DEF(can_flash_update)
{
	CAN_msg_t		msg;
	u32_t			*flash, flash_sizeof, flash_crc32;
	int			N, nodes_N;

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
		vTaskDelay((TickType_t) 10);

		for (N = 0; N < 90; ++N) {

			if (local_nodes_by_ACK(IFCAN_ACK_FLASH_WAIT_FOR_ERASE) == 0) {

				break;
			}

			/* Wait a bit more.
			 * */
			vTaskDelay((TickType_t) 100);
		}

		if (local_nodes_by_ACK(IFCAN_ACK_FLASH_DATA_ACCEPT) == 0) {

			printf("There is nothing to update" EOL);
		}
		else {
			IFCAN_flash_DATA(flash, flash_sizeof);

			/* Wait for ACK from all nodes.
			 * */
			vTaskDelay((TickType_t) 10);

			printf("Updated %i nodes" EOL, local_nodes_by_ACK(IFCAN_ACK_FLASH_SELFUPDATE));
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

			/* Do not send messages too frequently.
			 * */
			vTaskDelay((TickType_t) 1);
		}
	}
	while (1);
}

SH_DEF(can_node_remote)
{
	TaskHandle_t		xHandle;
	int			node_ID;
	char			xC;

	if (stoi(&node_ID, s) != NULL) {

		if (local_node_is_valid_remote(node_ID) != 0) {

			local.remote_node_ID = node_ID;
		}
		else {
			printf("No valid remote node ID" EOL);
		}
	}

	if (local.remote_node_ID != 0) {

		CAN_filter_ID(2, 0, local.remote_node_ID + IFCAN_ID_NODE_TX, IFCAN_FILTER_MATCH);

		xTaskCreate(task_REMOTE, "REMOTE", configMINIMAL_STACK_SIZE, NULL, 2, &xHandle);

		do {
			xC = getc();

			if (xC == K_EOT)
				break;

			xQueueSendToBack(local.queue_EX, &xC, portMAX_DELAY);
		}
		while (1);

		vTaskDelete(xHandle);

		CAN_filter_ID(2, 0, 0, 0);

		local.remote_node_ID = 0;

		puts(EOL);
	}
}

