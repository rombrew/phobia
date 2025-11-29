#ifndef _H_CAN_
#define _H_CAN_

#include "libc.h"

#define CAN_BITFREQ_HZ		1000000U
#define CAN_EXTENID_MIN		2048U

typedef struct {

	uint32_t		ID;
	uint16_t		len;

	union {

		uint8_t		b[8];
		uint16_t	s[4];
		uint32_t	l[2];
		float		f[2];
	}
	payload;
}
CAN_msg_t;

void CAN_startup();
void CAN_configure();

void CAN_bind_ID(int fn, int mb, uint32_t ID, uint32_t mask_ID);
int CAN_send_msg(const CAN_msg_t *msg);

int CAN_errate();

extern void CAN_IRQ();

#endif /* _H_CAN_ */

