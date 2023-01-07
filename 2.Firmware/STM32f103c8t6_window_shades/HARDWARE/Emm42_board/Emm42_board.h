#ifndef __Emm42_BOARD_H
#define __Emm42_BOARD_H

#include "stm32f10x.h"
#include "Emm42_usart.h"

#define		Emm42_RCC_GPIOA		RCC_APB2Periph_GPIOA
#define		Emm42_RCC_UART		RCC_APB2Periph_USART1

#define		Emm42_UART				USART1
#define		Emm42_UART_PORT		GPIOA
#define		Emm42_TX_PIN			GPIO_Pin_9
#define		Emm42_RX_PIN			GPIO_Pin_10

void Emm42_nvic_init(void);
void Emm42_clock_init(void);
void Emm42_usart_init(void);
void Emm42_board_init(void);

#endif

