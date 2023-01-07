#include "Emm42_me.h"

uint8_t cmd[10] = {0}; uint16_t i = 0;



//控制闭环电机的使能状态
void Motor_Enabled(void)
{
	cmd[0] = 0x01;	/* 地址 */
	cmd[1] = 0xF3;	/* 功能码 */
	cmd[2] = 0x00;	/* 发送01 F3 00 6B可以控制闭环电机处于不使能状态；发送01 F3 01 6B可以控制闭环电机处于使能状态；*/
	cmd[3] = 0x6B;	/* 固定校验字节 */
	for(i=0; i < 4; i++) { usartSend(cmd[i]); }									/* 发送命令 */
	rxReceiveCount = 0; while(rxFrameFlag == 0);								/* 等待接收返回数据 */
	rxFrameFlag = 0;
}

//控制闭环电机的正反转，即速度模式控制
void Motor_speed(void)
{
		cmd[0] = 0x01;	/* 地址 */
		cmd[1] = 0xF6;	/* 功能码 */
		cmd[2] = 0x15;	/* 方向半字节 + 速度高半字节 */
		cmd[3] = 0xFF;	/* 速度字节,( (uint16_t)(cmd[2] & 0x0F) << 8 | (uint16_t)cmd[3] )组成速度 */
		cmd[4] = 0x00;	/* 加速度 */
		cmd[5] = 0x6B;;	/* 固定校验字节 */
				
		for(i=0; i < 6; i++) 		/* 发送命令 */
		{ 
			usartSend(cmd[i]); 
		}				
		rxReceiveCount = 0; 
		while(rxFrameFlag == 0); //等待接收返回命令，* Emm42_V4.0闭环接收到命令后将会返回命令：
		rxFrameFlag = 0;
}

//Motor_speed_control(0x14,0xFF,0x40,0x00,0x0C,0x80)		//正转一周
//Motor_speed_control(0x04,0xFF,0x40,0x00,0x0C,0x80)		//反转一周
//Motor_speed_control(0x10,0x00,0xFF,0x00,0x19,0x6B)			//可以使闭环电机缓慢停止（速度档位为0）
//Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00)			//可以使闭环电机立即停止（脉冲为0，加速度为255）
void Motor_speed_control(u8 dir_speed_H,u8 speed_L,u8 a_sp,u8 pulse_G,u8 pulse_Z,u8 pulse_D)
{
		cmd[0] = 0x01;	/* 地址 */
		cmd[1] = 0xFD;	/* 功能码 */
		cmd[2] = dir_speed_H;	/* 方向半字节 + 速度高半字节 */
		cmd[3] = speed_L;	/* 速度字节,( (uint16_t)(cmd[2] & 0x0F) << 8 | (uint16_t)cmd[3] )组成速度 */
		cmd[4] = a_sp;	/* 加速度 */
		cmd[5] = pulse_G;	/* 脉冲数高字节 */
		cmd[6] = pulse_Z;	/* 脉冲数中字节 */
		cmd[7] = pulse_D;	/* 脉冲数低字节 */
		cmd[8] = 0x6B;;	/* 固定校验字节 */
				
		for(i=0; i < 9; i++) 		/* 发送命令 */
		{ 
			usartSend(cmd[i]); 
		}				
		rxReceiveCount = 0; 
		while(rxFrameFlag == 0); //等待接收返回命令
		rxFrameFlag = 0;
}


//控制闭环电机相对运动的角度，即位置模式控制
void Motor_speed_pos(void)
{
		/**********************************************************
		***	发送命令；串口位置控制命令格式：
			地址 0xFD  2字节方向+速度  加速度   3字节脉冲   CRC
			[0]  [1]     [2]  [3]       [4]   [5] [6] [7]  [8]
		**********************************************************/	
		cmd[0] = 0x01;	/* 地址 */
		cmd[1] = 0xFD;	/* 功能码 */
		cmd[2] = 0x14;	/* 方向半字节 + 速度高半字节 */
		cmd[3] = 0xFF;	/* 速度字节,( (uint16_t)(cmd[2] & 0x0F) << 8 | (uint16_t)cmd[3] )组成速度 */
		cmd[4] = 0x40;	/* 加速度 */
		cmd[5] = 0x00;	/* 脉冲数高字节 */
		cmd[6] = 0x0C;	/* 脉冲数中字节 */
		cmd[7] = 0x80;	/* 脉冲数低字节 */
		cmd[8] = 0x6B;;	/* 固定校验字节 */
				
		for(i=0; i < 9; i++) 		/* 发送命令 */
		{ 
			usartSend(cmd[i]); 
		}				
		rxReceiveCount = 0; 
		while(rxFrameFlag == 0); //等待接收返回命令，* Emm42_V4.0闭环接收到命令后将会返回命令：
		//1.如果接收到命令是错误命令，则返回：01（地址） EE 6B（校验码）
		//2.如果接收到命令是正确命令，则返回：01（地址） 02 6B（校验码）
		//3.如果位置控制命令执行完成，则返回：01（地址） 9F 6B（校验码）
		rxFrameFlag = 0;
}

//读取电机实时位置
float Emm42_MotorAngle(void)
{
	int32_t motorPosition;  float motorAngle;
	cmd[0] = 0x01;	/* 地址 */
	cmd[1] = 0x36;	/* 功能码 */
	cmd[2] = 0x6B;	/* 固定校验字节 */
	for(i=0; i < 3; i++) { usartSend(cmd[i]); }									/* 发送命令 */
	rxReceiveCount = 0; while(rxFrameFlag == 0);								/* 等待接收返回数据 */
	if(rxBuffer[0] == cmd[0])		/* 校验地址 */
	{
		motorPosition = (int32_t)rxBuffer[1] << 24 | (int32_t)rxBuffer[2] << 16 | (int32_t)rxBuffer[3] << 8 | (int32_t)rxBuffer[4];		/* 拼接成电机位置 */
		motorAngle = (float)( ( (int64_t)motorPosition * (int64_t)225) / (int64_t)4096 ) / (float)10.0;																/* 转换成角度单位 */
	}
	rxFrameFlag = 0;
	return motorAngle;
}

//读取位置误差
float Emm42_MotorError(void)
{
	int16 posError; float angleError;
	cmd[0] = 0x01;	/* 地址 */
	cmd[1] = 0x39;	/* 功能码 */
	cmd[2] = 0x6B;	/* 固定校验字节 */
	for(i=0; i < 3; i++) { usartSend(cmd[i]); }									/* 发送命令 */
	rxReceiveCount = 0; while(rxFrameFlag == 0);								/* 等待接收返回数据 */
	if(rxBuffer[0] == cmd[0])		/* 校验地址 */
	{
		posError = (int32_t)rxBuffer[1] << 8 | (int32_t)rxBuffer[2] ;	/* 拼接成电机位置 */
		angleError = (float)( ( (int64_t)posError * (int64_t)225) / (int64_t)4096 ) / (float)10.0;																/* 转换成角度单位 */
	}
	rxFrameFlag = 0;
	return angleError*100;
}



//读取输入脉冲数
int32 Emm42_Pulse_count(void)
{
	int32 pulse_num;
	cmd[0] = 0x01;	/* 地址 */
	cmd[1] = 0x33;	/* 功能码 */
	cmd[2] = 0x6B;	/* 固定校验字节 */
	for(i=0; i < 3; i++) { usartSend(cmd[i]); }									/* 发送命令 */
	rxReceiveCount = 0; while(rxFrameFlag == 0);								/* 等待接收返回数据 */
	if(rxBuffer[0] == cmd[0])		/* 校验地址 */
	{
		pulse_num = (int32_t)rxBuffer[1] << 24 | (int32_t)rxBuffer[2] << 16 | (int32_t)rxBuffer[3] << 8 | (int32_t)rxBuffer[4];		/* 拼接成电机位置 */
	}
	rxFrameFlag = 0;
	return pulse_num;
}



