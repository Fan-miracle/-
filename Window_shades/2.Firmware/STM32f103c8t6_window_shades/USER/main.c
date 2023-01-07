//  #include "stm32f10x_conf.h"
#include "delay.h"
#include "sys.h"
#include "Time.h"
#include "oled.h"
#include "led.h"
#include "key.h"
#include "usart.h"	
#include <math.h>
//#include "mpu6050.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h" 

#include "Emm42_board.h"
#include "Emm42_delay.h"
#include "Emm42_usart.h"
#include "Emm42_me.h"

#include "PWM.h"

#include "iic.h"
#include "BH1750.h"
#include "stmflash.h"

float MotorAngle;					//���λ��
float AngleError;					//λ�����
int32 Pulse_num;					//������

int UP=0;					//�Ϸ��򰴼�
int DWN = 0;			//�·��򰴼�
int LFT =0;				//���򰴼�
int RHT =0;				//�ҷ��򰴼�
int MID =0;				//�м䰴��
int SET_A =0;			//���ð���
int RST_B =0;			//��λ����

int32 light_Set = 35;					//��������
float light_data = 0;					//��ǿ��
int32 light_max =2000;				//��ǿ�����ֵ
int light_min = 0;						//��ǿ����Сֵ

int32 Servo_Angle = 1850;			//1750=2.5ms=90��			1800=2ms=45��;			1850=1.5s=0��;		1900=1ms=-45��;			1950=0.5ms=-90��


//�����������
static int Motor_num=0;  //���ת��Ȧ������̬���ݣ����溯��������������
static int Motor_angle_old=0;  //���ת���Ƕȣ�
static u32 Motor_pulse_old=0;  //���ת����������
int Motor_num_buff=0;		//���ת��Ȧ���ݴ棬
u32 Motor_pulse_buff=0;  //���ת���������ݴ棬
int Motor_angle_buff=0; 
u8 Motor_dir_flag = 0;
u8 Motor_state = 0;


u8 Motor_for = 1;						//����
u8 Motor_re = 1;						//����


//
extern u16 A_Parameter[10],B_Parameter[10],Flash_Parameter[10];  //Flash�������

extern u32 Sec;
extern u32 Sec_buf;
extern u8 Servo_flag;
extern u32 Servo_ms;


extern void Servo_Control(int data);
extern void Motor_control(void);

extern void Motor_data_treat(void);
extern void Motor_data_init(void);				//������ǰ�����ݣ���ȡ����
extern void Motor_data_save(void); 				//�����ݴ洢��Flash

//LD-2701����ת�Ƕ�Ϊ270�ȣ���1750~1950��Ӧ0~270�ȣ�
int main(void)
{	
	u8 key;   										//�������
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 															//���ڳ�ʼ��Ϊ115200
	delay_init();																			//��ʱ��ʼ�� 
	TIM2_Int_Init(999,71,2,0);				                //��ʱ1ms			������⣬����
	TIM3_Int_Init(999,719,2,1);				                //��ʱ10ms			�����¼�ִ��
	Key_Init();																				//������ʼ��
	LED_Init();																				//LED��ʼ��
	OLED_Init();																			//��ʼ��OLED  
	OLED_Clear(); 																		//OLED����
	 
	Emm42_board_init(); 															//��ʼ����������
	Emm42_delay_ms(1200);															//��ʱ1.2��ȴ�Emm�ջ��������ϵ��ʼ�����
	TIM4_PWM_Init(1999,719);   //PWMƵ��=72000000/��719+1��/��1999+1��=50hz=20ms
	bh1750_init();
	delay_ms(1000);
	
	OLED_ShowString(0,0,"light:",12); 
	OLED_ShowString(0,14,"light_Set:",12); 
	OLED_ShowString(0,28,"Servo_Angle:",12); 
	OLED_ShowString(0,50,"UP:",12); 
	OLED_ShowString(64,50,"DWN:",12); 
	
	OLED_Refresh_Gram();
	while(1)
	{
		key=KEY_Scan(0); 
		if(key)
		{
			switch(key)
			{
				case KEY_UP_PRES:  UP = 1; DWN = 0; 	 			 break;
				case KEY_DWN_PRES: DWN = 1;UP = 0;  				 break;
				case KEY_LFT_PRES: light_Set = light_Set-1;  break;
				case KEY_RHT_PRES: light_Set = light_Set+1;  break;
				case KEY_SET_PRES: break;
				case KEY_RST_PRES: break;
				default: break;
			}
		}
		if(Sec > 250)
		{
			Sec = 0;
			light_data = bh1750_get_data(); //��ȡ��������
			if(light_data>light_max)light_data = light_max;
			else if(light_data<light_min)light_data = light_min;
		}
		Servo_Control(light_data);
		Motor_control();
		
//		OLED_ShowNum(64,0,MotorAngle,4,12);		//��ʾ��������	
//		OLED_ShowNum(64,14,AngleError,4,12);
//		OLED_ShowNum(64,28,Pulse_num,4,12); 
		OLED_ShowNum(64,0,light_data,4,12);		//��ʾ��������	
		OLED_ShowNum(64,14,light_Set,4,12);
		OLED_ShowNum(64,28,Servo_Angle,4,12);   	

		OLED_ShowNum(30,50,UP,3,12);		//��ʾ��������	 
		OLED_ShowNum(94,50,DWN,3,12);		//��ʾ��������			
		OLED_Refresh_Gram();
	} 	
}

void Motor_control(void)
{
		if(UP == 1 && DWN == 0 && Motor_for==1)
		{		Motor_re = 0;
				Motor_speed_control(0x14,0xFF,0x80,0x00,0x00,0x19);					//��ת
		}
		else if(UP == 0 && DWN == 1 &&Motor_re ==1)
		{
			  Motor_for = 0;
				Motor_speed_control(0x04,0xFF,0x80,0x00,0x00,0x19);					//��ת
		}
		if((UP == 0 && DWN == 1 &&Motor_for==1 &&Motor_re ==0)||(UP == 1 && DWN == 0 &&Motor_for==0 &&Motor_re ==1))
		{		UP = 0;			DWN = 0;			Motor_re=1;			Motor_for =1;		}
	
		MotorAngle = Emm42_MotorAngle();					//���λ��
		AngleError = Emm42_MotorError();					//λ�����
		Pulse_num = Emm42_Pulse_count();					//����
		
		if(abs((int)AngleError)>230 && UP == 1 && DWN == 0)
		{
			Motor_for = 0;
			Motor_re = 1;
			Motor_speed_control(0x04,0xFF,0x40,0x00,0x00,0x0A);						//��ת1������
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	  				//����ֹͣ
		}
		else if(abs((int)AngleError)>230 && UP == 0 && DWN == 1)
		{
			Motor_for = 1;
			Motor_re = 0;
			Motor_speed_control(0x14,0xFF,0x40,0x00,0x00,0x0A);						//��ת1������
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);				    //����ֹͣ
		}
		else if(UP == 0&& DWN==0)
		{
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);				    //����ֹͣ
		}
}


void Motor_data_treat(void)
{
	Motor_num = Motor_num_buff + (Motor_angle_buff + MotorAngle)/360;
	Motor_angle_old = (int)(Motor_angle_buff + MotorAngle)%360;
	Motor_pulse_old = Motor_pulse_buff +Pulse_num;
}

void Motor_data_init(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)A_Parameter,10);
	//Flashδд�����ݵ�ʱ������Ϊ65535
	if(A_Parameter[0]==65535&&A_Parameter[1]==65535&&A_Parameter[2]==65535&&A_Parameter[3]==65535)
	{

	}
	else//Flash�������ݣ����ж�ȡ
	{		
		Motor_num = A_Parameter[0];
		Motor_angle_old = A_Parameter[1];
		Motor_pulse_old = A_Parameter[2];
	}
}

void Motor_data_save(void)
{
	B_Parameter[0]=Motor_num;		
	B_Parameter[1]=Motor_angle_old;	
	B_Parameter[2]=Motor_pulse_old;
	STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)B_Parameter,3);	
}



void Servo_Control(int data)
{
	if(Servo_ms > 1000)
	{
		Servo_flag =0;  //ʹ��TIMx	
		Servo_ms = 0;
	}
	if(light_Set-data>10)
	{
		Servo_flag = 1;
		if(Servo_ms > 100)
		{
			Servo_ms = 0;
			if(Servo_Angle<1900) Servo_Angle = Servo_Angle+2;
			TIM_SetCompare1(TIM4,Servo_Angle);
		}
	}
	else if(data-light_Set>10)
	{
		Servo_flag = 1;  
		if(Servo_ms > 100)
		{
			Servo_ms = 0;
			if(Servo_Angle>1750)Servo_Angle = Servo_Angle-2;
			TIM_SetCompare1(TIM4,Servo_Angle);
		}
	}
}




//		key=KEY_Scan(0); 
//		if(key)
//		{
//			switch(key)
//			{
//				case KEY_UP_PRES:  UP = UP+1;  	 break;
//				case KEY_DWN_PRES: DWN = DWN+1;  break;
//				case KEY_LFT_PRES: LFT = LFT+1;  break;
//				case KEY_RHT_PRES: RHT = RHT+1;  break;
//				case KEY_MID_PRES: MID = MID+1;  break;
//				case KEY_SET_PRES: light_Set = light_Set+1;  break;
//				case KEY_RST_PRES: light_Set = light_Set-1;  break;
//				default: break;
//			}
//		}
//		
//		if(Sec > 200)
//		{
//			Sec = 0;
//			light_data = bh1750_get_data(); //��ȡ��������
//			if(light_data>light_max)light_data = light_max;
//			else if(light_data<light_min)light_data = light_min;
//		}
//		Servo_Control(light_data);
//		OLED_ShowNum(64,0,light_data,4,12);		//��ʾ��������	
//		OLED_ShowNum(64,14,light_Set,4,12);
//		OLED_ShowNum(64,28,Servo_Angle,4,12);    
//		OLED_Refresh_Gram();


//int main(void)
//{	
//	u8 key;   										//�������
//	 
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
//	uart_init(115200);	 															//���ڳ�ʼ��Ϊ115200
//	delay_init();																			//��ʱ��ʼ�� 
//	TIM2_Int_Init(999,71,2,0);				                //��ʱ1ms			������⣬����
//	TIM3_Int_Init(999,719,2,1);				                //��ʱ10ms			�����¼�ִ��
//	Key_Init();																				//������ʼ��
//	LED_Init();																				//LED��ʼ��
//	OLED_Init();																			//��ʼ��OLED  
//	OLED_Clear(); 																		//OLED����
//	 
//	Emm42_board_init(); 															//��ʼ����������
//	Emm42_delay_ms(1200);															//��ʱ1.2��ȴ�Emm�ջ��������ϵ��ʼ�����
//	TIM4_PWM_Init(1999,719);   //PWMƵ��=72000000/��719+1��/��1999+1��=50hz=20ms
//	bh1750_init();
//	delay_ms(750);
//	Motor_data_init();					//������ǰ�����ݣ���ȡ����
//	Motor_pulse_buff = Motor_pulse_old;
//	Motor_num_buff = Motor_num;
//	Motor_angle_buff = Motor_angle_old;
//	
////	OLED_ShowString(0,0,"light:",12); 
////	OLED_ShowString(0,14,"light_Set:",12); 
////	OLED_ShowString(0,28,"Servo_Angle:",12); 
////	OLED_Refresh_Gram();
//	while(1)
//	{
//		key=KEY_Scan(0); 
//		if(key)
//		{
//			switch(key)
//			{
//				case KEY_UP_PRES:  UP = 1; DWN = 0; 	 break;
//				case KEY_DWN_PRES: DWN = 1;UP = 0;  break;
//				case KEY_LFT_PRES: LFT = LFT+1;  break;
//				case KEY_RHT_PRES: RHT = RHT+1;  break;
//				case KEY_MID_PRES: MID = MID+1;  break;
//				case KEY_SET_PRES: light_Set = light_Set+1;  break;
//				case KEY_RST_PRES: light_Set = light_Set-1;  break;
//				default: break;
//			}
//		}
////		if(UP == 1 && DWN == 0&& Motor_dir_flag == 1)
////				Motor_speed_control(0x14,0xFF,0x40,0x00,0x0C,0x80);					//��ת
////		else if(UP == 0 && DWN == 1 && Motor_dir_flag == 0)
////				Motor_speed_control(0x04,0xFF,0x40,0x00,0x0C,0x80);					//��ת
////		MotorAngle = Emm42_MotorAngle();					//���λ��
////		AngleError = Emm42_MotorError();					//λ�����
////		Pulse_num = Emm42_Pulse_count();					//������
////		Motor_data_treat();												//���ݴ���
////		Motor_data_save();												//�����ݴ洢��Flash
////		if(Motor_num>10)													//��ת�ر�
////		{
////			Motor_dir_flag = 0;											//����ת
////			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	
////		}
////		else if(Motor_num < 0)
////		{
////			Motor_dir_flag = 1;											//������ת
////			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	
////		}
//		
//		OLED_ShowNum(0,0,Motor_num,4,12);
//		OLED_ShowNum(0,14,Motor_angle_old,4,12);
//		OLED_ShowNum(64,14,MotorAngle,4,12);		//��ʾ��������	  
//		OLED_ShowNum(0,28,AngleError,4,12);		//��ʾ��������	 
//		OLED_ShowNum(64,28,Pulse_num,4,12);		//��ʾ��������
//		OLED_ShowNum(0,42,UP,4,12);		//��ʾ��������	 
//		OLED_ShowNum(64,42,DWN,4,12);		//��ʾ��������			
//		OLED_Refresh_Gram();
//	} 	
//}

