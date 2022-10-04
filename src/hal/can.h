#ifndef _H_CAN_
#define _H_CAN_

#include "libc.h"

enum {
	CAN_TX_OK			= 0,
	CAN_TX_FAILED,
};

typedef struct {

	u16_t			ID;
	u16_t			len;
	u8_t			payload[8];
}
CAN_msg_t;

void CAN_startup();
void CAN_configure();

void CAN_filter_ID(int fs, int mb, int ID, int mID);
int CAN_send_msg(const CAN_msg_t *msg);

extern void CAN_IRQ();

#endif /* _H_CAN_ */

