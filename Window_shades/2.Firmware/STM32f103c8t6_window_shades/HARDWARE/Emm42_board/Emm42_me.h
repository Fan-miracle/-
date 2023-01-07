#ifndef _EMM42_ME_H
#define _EMM42_ME_H

#include "stm32f10x.h"
#include "Emm42_board.h"
#include "Emm42_delay.h"
#include "Emm42_usart.h"



extern float Emm42_MotorAngle(void);	//��ȡ���ʵʱλ��
extern float Emm42_MotorError(void);	//��ȡλ�����
extern int32 Emm42_Pulse_count(void);	//��ȡ����������
extern void Motor_speed_pos(void);		//���Ʊջ��������˶��ĽǶȣ���λ��ģʽ����
extern void Motor_Enabled(void);			//���Ʊջ������ʹ��״̬
extern void Motor_speed(void);				//���Ʊջ����������ת�����ٶ�ģʽ����

extern void Motor_speed_control(u8 dir_speed_H,u8 speed_L,u8 a_sp,u8 pulse_G,u8 pulse_Z,u8 pulse_D);
#endif


