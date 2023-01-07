#include "Emm42_usart.h"

__IO uint8_t rxBuffer[20] = {0};
__IO uint16_t rxReceiveCount = 0;
__IO uint8_t rxFrameFlag = 0;	//表示一帧数据接收完成

/**
	* @brief   Emm42_UART中断函数
	* @param   无
	* @retval  无
	*/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(Emm42_UART, USART_IT_RXNE) != RESET)
	{
		if(rxReceiveCount < 20)
		{
			rxBuffer[rxReceiveCount] = (uint8_t)Emm42_UART->DR;

			++rxReceiveCount;
		}
		else
		{
			rxReceiveCount = 0;
		}

		USART_ClearITPendingBit(Emm42_UART, USART_IT_RXNE);
	}
	else if(USART_GetITStatus(Emm42_UART, USART_IT_IDLE) != RESET)
	{
/**********************************************************
***	先读SR再读DR，清除IDLE中断
**********************************************************/
		Emm42_UART->SR; Emm42_UART->DR;

/**********************************************************
***	释放互斥锁
**********************************************************/
		rxFrameFlag = 1;
	}
}

/**
	* @brief   Emm42_UART发送函数
	* @param   无
	* @retval  无
	*/
void usartSend(uint16_t Data)
{
	uint16_t t0 = 0;
	
	Emm42_UART->DR = (Data & (uint16_t)0x01FF);

	while( !(Emm42_UART->SR & USART_FLAG_TXE) )
	{
		++t0; if(t0 > 6000)	{	return;}
	}
}


