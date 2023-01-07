#ifndef _EMM42_ME_H
#define _EMM42_ME_H

#include "stm32f10x.h"
#include "Emm42_board.h"
#include "Emm42_delay.h"
#include "Emm42_usart.h"



extern float Emm42_MotorAngle(void);	//读取电机实时位置
extern float Emm42_MotorError(void);	//读取位置误差
extern int32 Emm42_Pulse_count(void);	//读取输入脉冲数
extern void Motor_speed_pos(void);		//控制闭环电机相对运动的角度，即位置模式控制
extern void Motor_Enabled(void);			//控制闭环电机的使能状态
extern void Motor_speed(void);				//控制闭环电机的正反转，即速度模式控制

extern void Motor_speed_control(u8 dir_speed_H,u8 speed_L,u8 a_sp,u8 pulse_G,u8 pulse_Z,u8 pulse_D);
#endif


