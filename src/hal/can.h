#ifndef _H_CAN_
#define _H_CAN_

#include "libc.h"

#define CAN_BITFREQ_HZ		1000000U

typedef struct {

	uint16_t		ID;
	uint16_t		len;
	uint8_t			payload[8];
}
CAN_msg_t;

void CAN_startup();
void CAN_configure();

void CAN_bind_ID(int fs, int mb, int ID, int mask_ID);
int CAN_send_msg(const CAN_msg_t *msg);

int CAN_errate();

extern void CAN_IRQ();

#endif /* _H_CAN_ */

