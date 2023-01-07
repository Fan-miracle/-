#ifndef __Emm42_USART_H
#define __Emm42_USART_H

#include "Emm42_board.h"


extern __IO uint16_t rxReceiveCount;
extern __IO uint8_t rxFrameFlag;
extern __IO uint8_t rxBuffer[20];

void usartSend(uint16_t Data);

#endif
