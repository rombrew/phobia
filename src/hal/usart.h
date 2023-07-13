#ifndef _H_USART_
#define _H_USART_

void USART_startup();

int USART_getc();
int USART_poll();
void USART_putc(int c);

#endif /* _H_USART_ */

