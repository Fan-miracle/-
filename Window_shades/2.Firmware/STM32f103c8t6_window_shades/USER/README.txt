*	如果你的STM32控制板是STM32最小系统，使用USB进行供电的话，注意上电顺序：
*	上电时，先通闭环驱动板的7~28V供电，再通STM32控制板的USB供电，避免一些效应造成损坏！！！
*	断电时，先断STM32控制板的USB供电，再断闭环驱动板的7~28V供电。
*   Emm42_V4.0步进闭环驱动板的V+和Gnd接7~28V供电


//脉冲控制：***********************************************************************************************************************
//步进电机与STM32接线
*	1. 使能GPIOA端口时钟
*	2. 初始化PA5（接到闭环驱动板的En引脚） 为推挽输出，输出低电平（0V）		（确保闭环驱动屏幕上的En选项选择Hold或者L，默认是Hold）
*	3. 初始化PA6（接到闭环驱动板的Stp引脚）为推挽输出，输出低电平（0V）
*	4. 初始化PA7（接到闭环驱动板的Dir引脚）为推挽输出，输出低电平（0V）


//程序
int main(void)
{	
  __IO int32_t i = 0, j = 0;	u8 cntDir = 0;
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 															//串口初始化为115200
	delay_init();																			//延时初始化 
	LED_Init();																				//LED初始化
	OLED_Init();																			//初始化OLED  
	OLED_Clear(); 																		//OLED清屏
	 
	Emm42_board_init(); 															//初始化板载外设
	Emm42_delay_ms(1200);															//延时1.2秒等待Emm闭环驱动板上电初始化完毕
	 
	OLED_ShowString(20,0,"MPU6050",12);
	OLED_Refresh_Gram();
	//	WHILE循环发脉冲
	//		* 1. 异或取反PA6（Stp引脚）6400次，也就是发送3200个脉冲给闭环驱动板
	//		* 2. 延时1秒并改变PA7（Dir引脚）电平
	//		* 3. 再发送3200个脉冲
	//		*	现象：顺时针转一圈 -> 延时1秒 -> 逆时针转一圈 -> 延时1秒 -> 顺时针转一圈 -> 如此循环...
	while(1)
	{
		j = 3200;														
		while(j--);																										//高低电平的时间间隔，即脉冲时间的一半(控制电机转动速度)
		Emm42_GPIO->ODR ^= Emm42_Stp_Pin;															//异或取反PA6（Stp引脚）
		if(cntDir)	{--i;}	else	{++i;}															//记录IO取反次数（IO取反次数 = 2倍的脉冲数）
		//控制PA6（Stp引脚）取反了6400次，即发送了3200个脉冲
		//16细分下，发送3200个脉冲电机转动一圈（1.8度电机）
		//所以计数到6400即电机旋转了一圈后，现在开始切换方向
		if(i >= 6400)
		{
			Emm42_delay_ms(1000);																				/* 延时1秒 */
			GPIO_SetBits(Emm42_GPIO, Emm42_Dir_Pin);		cntDir = 1;			/* 改变PA7（Dir引脚）电平，切换到逆时针方向转动 */
		}
		else if(i == 0)
		{

			Emm42_delay_ms(1000);																				/* 延时1秒 */
			GPIO_ResetBits(Emm42_GPIO, Emm42_Dir_Pin);	cntDir = 0;			/* 改变PA7（Dir引脚）电平切换到顺时针方向转动 */
		}
	} 	
}





//串口控制**********************************************************************************************************
//STM32和Emm42_V4.0步进闭环的接线：
*	1. 确保闭环驱动屏幕上的Mode选项选择CR_UART模式
*	2. STM32的PA9接Emm闭环驱动的RX引脚
*	3. STM32的PA10接Emm闭环驱动的TX引脚
*   4. STM32的Gnd接Emm闭环驱动的Gnd引脚



//程序
	uint8_t cmd[10] = {0}; uint16_t i = 0;
	Emm42_board_init();
	Emm42_delay_ms(1200);
	while(1)
	{
		cmd[0] = 0x01;	/* 地址 */
		cmd[1] = 0xFD;	/* 功能码 */
		cmd[2] = 0x12;	/* 方向半字节 + 速度高半字节 */
		cmd[3] = 0xFF;	/* 速度字节,( (uint16_t)(cmd[2] & 0x0F) << 8 | (uint16_t)cmd[3] )组成速度 */
		cmd[4] = 0x40;	/* 加速度 */
		cmd[5] = 0x00;	/* 脉冲数高字节 */
		cmd[6] = 0x0C;	/* 脉冲数中字节 */
		cmd[7] = 0x80;	/* 脉冲数低字节 */
		cmd[8] = 0x6B;;	/* 固定校验字节 */
				/* 发送命令 */
		for(i=0; i < 9; i++) { usartSend(cmd[i]); }

		/*
			等待接收返回命令
				* Emm42_V4.0闭环接收到命令后将会返回命令：
						1.如果接收到命令是错误命令，则返回：01（地址） EE 6B（校验码）
						2.如果接收到命令是正确命令，则返回：01（地址） 02 6B（校验码）
						3.如果位置控制命令执行完成，则返回：01（地址） 9F 6B（校验码）
		*/
		rxReceiveCount = 0; while(rxFrameFlag == 0); rxFrameFlag = 0;
		cmd[0] = 0x01;	/* 地址 */
		cmd[1] = 0x36;	/* 功能码 */
		cmd[2] = 0x6B;	/* 固定校验字节 */
		for(i=0; i < 3; i++) { usartSend(cmd[i]); }		/* 发送命令 */
	
		rxReceiveCount = 0; while(rxFrameFlag == 0);/* 等待接收返回数据 */
		
		if(rxBuffer[0] == cmd[0])			///* 校验地址 */
		{
			MotorPosition = (int32_t)rxBuffer[1] << 24 | (int32_t)rxBuffer[2] << 16 | (int32_t)rxBuffer[3] << 8 | (int32_t)rxBuffer[4];			/* 拼接成电机位置 */
			MotorAngle = (float)( ( (int64_t)MotorPosition * (int64_t)225) / (int64_t)4096 ) / (float)10.0;				///* 转换成角度单位 */
		}
		rxFrameFlag = 0;
	} 












