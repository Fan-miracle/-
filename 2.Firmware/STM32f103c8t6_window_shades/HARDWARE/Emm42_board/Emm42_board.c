#include "Emm42_board.h"

/**
	* @brief   配置NVIC控制器
	* @param   无
	* @retval  无
	*/
void Emm42_nvic_init(void)
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);						///< 4bit抢占优先级位

/**********************************************************
***	配置USART1的抢占优先级为0
**********************************************************/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				///< 响应优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	///< 抢占优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}


/**
	*	@brief		外设时钟初始化
	*	@param		无
	*	@retval		无
	*/
void Emm42_clock_init(void)
{
/**********************************************************
***	开启GPIOA、AFIO外设时钟
**********************************************************/
	RCC_APB2PeriphClockCmd(Emm42_RCC_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

/**********************************************************
***	开启USART外设时钟
**********************************************************/
	RCC_APB2PeriphClockCmd(Emm42_RCC_UART, ENABLE);

/**********************************************************
***	禁用JTAG
**********************************************************/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/**
	* @brief   初始化USART
	* @param   无
	* @retval  无
	*/
void Emm42_usart_init(void)
{
/**********************************************************
***	初始化USART引脚
**********************************************************/
	/* PA9 - USART1_TX */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = Emm42_TX_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(Emm42_UART_PORT, &GPIO_InitStructure);
	/* PA10 - USART1_RX */
	GPIO_InitStructure.GPIO_Pin = Emm42_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	/* 浮空输入 */
	GPIO_Init(Emm42_UART_PORT, &GPIO_InitStructure);

/**********************************************************
***	USART初始化
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(Emm42_UART, &USART_InitStructure);

/**********************************************************
***	使能USART中断
**********************************************************/	
	USART_ITConfig(Emm42_UART, USART_IT_RXNE, ENABLE);
	USART_ITConfig(Emm42_UART, USART_IT_IDLE, ENABLE);

/**********************************************************
***	使能USART
**********************************************************/
	USART_Cmd(Emm42_UART, ENABLE);
}

/**
	*	@brief		板载初始化
	*	@param		无
	*	@retval		无
	*/
void Emm42_board_init(void)
{
	Emm42_nvic_init();	/* 初始化NVIC */
	Emm42_clock_init();	/* 开启外设时钟 */
	Emm42_usart_init();	/* 初始化UART */
}
