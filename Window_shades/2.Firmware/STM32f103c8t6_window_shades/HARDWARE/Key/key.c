#include "key.h"


extern u32 Sec_buf;						//定时器3


void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_Str;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能PORTA,PORTE时钟
	GPIO_Str.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
	GPIO_Str.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Str.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Str);
	
//	GPIO_Str.GPIO_Mode = GPIO_Mode_IPU;//设置成上拉输入
//	GPIO_Str.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
//	GPIO_Init(GPIOB,&GPIO_Str);
}

//GPIO_Mode_IPD： 另一端接3.3V			
//GPIO_Mode_IPU： 另一端接GND


//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//注意此函数有响应优先级,				KEY_SET>KEY_RST>KEY_UP
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按	
	if(key_up == 0 && Sec_buf>100)
	{
		key_up=1;							//按键按下一秒后未回正，则支持连按
	}
	if(key_up&&(KEY_UP==0||KEY_DWN==0||KEY_LFT==0||KEY_RHT==0||KEY_SET==0||KEY_RST==0))		//按键按下
	{
		delay_ms(20);//去抖动 
		key_up=0;
		TIM_Cmd(TIM3, ENABLE);  //使能TIMx								
		if(KEY_SET==0)					return KEY_SET_PRES;								//设置按键
		else if(KEY_RST==0)			return KEY_RST_PRES;
		else if(KEY_UP==0)			return KEY_UP_PRES;									//向上
		else if(KEY_DWN==0)			return KEY_DWN_PRES;								//向下
		else if(KEY_LFT==0)			return KEY_LFT_PRES;	
		else if(KEY_RHT==0)			return KEY_RHT_PRES;		
//		else if(KEY_MID==0)			return KEY_MID_PRES;	
	}
	else if(KEY_UP==1&&KEY_DWN==1&&KEY_LFT==1&&KEY_RHT==1&&KEY_SET==1&&KEY_RST==1)				//按键回正
	{
		TIM_Cmd(TIM3, DISABLE);  //使能TIMx	
		Sec_buf = 0;
		key_up=1; 	    
	}
 	return 0;// 无按键按下
}














