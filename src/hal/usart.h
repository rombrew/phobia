/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2015 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_USART_
#define _H_USART_

#define USART_RXBUF_SZ		80
#define USART_TXBUF_SZ		80

typedef struct {

	int		baudRate;

	char		RX[USART_RXBUF_SZ];
	char		TX[USART_TXBUF_SZ];
	int		rN, tN;
}
halUSART_t;

extern halUSART_t		halUSART;

void usartEnable();
void usartDisable();

int usartRecv();
int usartSend(int xC);
void usartFlush();

#endif /* _H_USART_ */

