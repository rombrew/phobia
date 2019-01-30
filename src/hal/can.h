#ifndef _H_CAN_
#define _H_CAN_

void CAN_startup();
void CAN_set_filter(int nfilt, int fifo, unsigned long ID, unsigned long mID);
void CAN_send_msg(unsigned long ID, int len, const unsigned char payload[8]);

extern void CAN_IRQ();

#endif /* _H_CAN_ */

