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

float MotorAngle;					//电机位置
float AngleError;					//位置误差
int32 Pulse_num;					//脉冲数

int UP=0;					//上方向按键
int DWN = 0;			//下方向按键
int LFT =0;				//左方向按键
int RHT =0;				//右方向按键
int MID =0;				//中间按键
int SET_A =0;			//设置按键
int RST_B =0;			//复位按键

int32 light_Set = 35;					//光照设置
float light_data = 0;					//光强度
int32 light_max =2000;				//光强度最大值
int light_min = 0;						//光强度最小值

int32 Servo_Angle = 1850;			//1750=2.5ms=90度			1800=2ms=45度;			1850=1.5s=0度;		1900=1ms=-45度;			1950=0.5ms=-90度


//步进电机参数
static int Motor_num=0;  //电机转动圈数，静态数据，不随函数进入把数据清除
static int Motor_angle_old=0;  //电机转动角度，
static u32 Motor_pulse_old=0;  //电机转动脉冲数，
int Motor_num_buff=0;		//电机转动圈数暂存，
u32 Motor_pulse_buff=0;  //电机转动脉冲数暂存，
int Motor_angle_buff=0; 
u8 Motor_dir_flag = 0;
u8 Motor_state = 0;


u8 Motor_for = 1;						//正向
u8 Motor_re = 1;						//反向


//
extern u16 A_Parameter[10],B_Parameter[10],Flash_Parameter[10];  //Flash相关数组

extern u32 Sec;
extern u32 Sec_buf;
extern u8 Servo_flag;
extern u32 Servo_ms;


extern void Servo_Control(int data);
extern void Motor_control(void);

extern void Motor_data_treat(void);
extern void Motor_data_init(void);				//将掉电前的数据，读取储存
extern void Motor_data_save(void); 				//将数据存储到Flash

//LD-2701的旋转角度为270度，从1750~1950对应0~270度；
int main(void)
{	
	u8 key;   										//按键检测
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 															//串口初始化为115200
	delay_init();																			//延时初始化 
	TIM2_Int_Init(999,71,2,0);				                //定时1ms			按键检测，计秒
	TIM3_Int_Init(999,719,2,1);				                //定时10ms			用于事件执行
	Key_Init();																				//按键初始化
	LED_Init();																				//LED初始化
	OLED_Init();																			//初始化OLED  
	OLED_Clear(); 																		//OLED清屏
	 
	Emm42_board_init(); 															//初始化板载外设
	Emm42_delay_ms(1200);															//延时1.2秒等待Emm闭环驱动板上电初始化完毕
	TIM4_PWM_Init(1999,719);   //PWM频率=72000000/（719+1）/（1999+1）=50hz=20ms
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
			light_data = bh1750_get_data(); //获取光照数据
			if(light_data>light_max)light_data = light_max;
			else if(light_data<light_min)light_data = light_min;
		}
		Servo_Control(light_data);
		Motor_control();
		
//		OLED_ShowNum(64,0,MotorAngle,4,12);		//显示整数部分	
//		OLED_ShowNum(64,14,AngleError,4,12);
//		OLED_ShowNum(64,28,Pulse_num,4,12); 
		OLED_ShowNum(64,0,light_data,4,12);		//显示整数部分	
		OLED_ShowNum(64,14,light_Set,4,12);
		OLED_ShowNum(64,28,Servo_Angle,4,12);   	

		OLED_ShowNum(30,50,UP,3,12);		//显示整数部分	 
		OLED_ShowNum(94,50,DWN,3,12);		//显示整数部分			
		OLED_Refresh_Gram();
	} 	
}

void Motor_control(void)
{
		if(UP == 1 && DWN == 0 && Motor_for==1)
		{		Motor_re = 0;
				Motor_speed_control(0x14,0xFF,0x80,0x00,0x00,0x19);					//正转
		}
		else if(UP == 0 && DWN == 1 &&Motor_re ==1)
		{
			  Motor_for = 0;
				Motor_speed_control(0x04,0xFF,0x80,0x00,0x00,0x19);					//反转
		}
		if((UP == 0 && DWN == 1 &&Motor_for==1 &&Motor_re ==0)||(UP == 1 && DWN == 0 &&Motor_for==0 &&Motor_re ==1))
		{		UP = 0;			DWN = 0;			Motor_re=1;			Motor_for =1;		}
	
		MotorAngle = Emm42_MotorAngle();					//电机位置
		AngleError = Emm42_MotorError();					//位置误差
		Pulse_num = Emm42_Pulse_count();					//脉冲
		
		if(abs((int)AngleError)>230 && UP == 1 && DWN == 0)
		{
			Motor_for = 0;
			Motor_re = 1;
			Motor_speed_control(0x04,0xFF,0x40,0x00,0x00,0x0A);						//正转1个脉冲
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	  				//立即停止
		}
		else if(abs((int)AngleError)>230 && UP == 0 && DWN == 1)
		{
			Motor_for = 1;
			Motor_re = 0;
			Motor_speed_control(0x14,0xFF,0x40,0x00,0x00,0x0A);						//反转1个脉冲
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);				    //立即停止
		}
		else if(UP == 0&& DWN==0)
		{
			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);				    //立即停止
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
	//Flash未写入数据的时候，数据为65535
	if(A_Parameter[0]==65535&&A_Parameter[1]==65535&&A_Parameter[2]==65535&&A_Parameter[3]==65535)
	{

	}
	else//Flash内有数据，进行读取
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
		Servo_flag =0;  //使能TIMx	
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
//			light_data = bh1750_get_data(); //获取光照数据
//			if(light_data>light_max)light_data = light_max;
//			else if(light_data<light_min)light_data = light_min;
//		}
//		Servo_Control(light_data);
//		OLED_ShowNum(64,0,light_data,4,12);		//显示整数部分	
//		OLED_ShowNum(64,14,light_Set,4,12);
//		OLED_ShowNum(64,28,Servo_Angle,4,12);    
//		OLED_Refresh_Gram();


//int main(void)
//{	
//	u8 key;   										//按键检测
//	 
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
//	uart_init(115200);	 															//串口初始化为115200
//	delay_init();																			//延时初始化 
//	TIM2_Int_Init(999,71,2,0);				                //定时1ms			按键检测，计秒
//	TIM3_Int_Init(999,719,2,1);				                //定时10ms			用于事件执行
//	Key_Init();																				//按键初始化
//	LED_Init();																				//LED初始化
//	OLED_Init();																			//初始化OLED  
//	OLED_Clear(); 																		//OLED清屏
//	 
//	Emm42_board_init(); 															//初始化板载外设
//	Emm42_delay_ms(1200);															//延时1.2秒等待Emm闭环驱动板上电初始化完毕
//	TIM4_PWM_Init(1999,719);   //PWM频率=72000000/（719+1）/（1999+1）=50hz=20ms
//	bh1750_init();
//	delay_ms(750);
//	Motor_data_init();					//将掉电前的数据，读取储存
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
////				Motor_speed_control(0x14,0xFF,0x40,0x00,0x0C,0x80);					//正转
////		else if(UP == 0 && DWN == 1 && Motor_dir_flag == 0)
////				Motor_speed_control(0x04,0xFF,0x40,0x00,0x0C,0x80);					//反转
////		MotorAngle = Emm42_MotorAngle();					//电机位置
////		AngleError = Emm42_MotorError();					//位置误差
////		Pulse_num = Emm42_Pulse_count();					//脉冲数
////		Motor_data_treat();												//数据处理
////		Motor_data_save();												//将数据存储到Flash
////		if(Motor_num>10)													//正转关闭
////		{
////			Motor_dir_flag = 0;											//允许反转
////			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	
////		}
////		else if(Motor_num < 0)
////		{
////			Motor_dir_flag = 1;											//允许正转
////			Motor_speed_control(0x12,0xFF,0xFF,0x00,0x00,0x00);	
////		}
//		
//		OLED_ShowNum(0,0,Motor_num,4,12);
//		OLED_ShowNum(0,14,Motor_angle_old,4,12);
//		OLED_ShowNum(64,14,MotorAngle,4,12);		//显示整数部分	  
//		OLED_ShowNum(0,28,AngleError,4,12);		//显示整数部分	 
//		OLED_ShowNum(64,28,Pulse_num,4,12);		//显示整数部分
//		OLED_ShowNum(0,42,UP,4,12);		//显示整数部分	 
//		OLED_ShowNum(64,42,DWN,4,12);		//显示整数部分			
//		OLED_Refresh_Gram();
//	} 	
//}

