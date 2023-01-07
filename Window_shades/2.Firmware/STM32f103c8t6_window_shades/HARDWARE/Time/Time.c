#include "Time.h"
#include "stm32f10x_tim.h"

u32 Sec=0;
u32 Sec_buf=0;
u8 Servo_flag = 0;
u32 Servo_ms=0;

uint8 Flag_1ms = 0;
uint8 Flag_10ms = 0;

/******************************		通用定时器		**********************************/
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数

//定时器TIM2初始化
void TIM2_Int_Init(u16 arr,u16 psc, uint8 Preem, uint8 Resq)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStr;
	NVIC_InitTypeDef NVIC_InitStr;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	TIM_TimeBaseStr.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStr.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStr.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStr.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStr); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStr.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = Preem;  //先占优先级0级
	NVIC_InitStr.NVIC_IRQChannelSubPriority = Resq;  //从优先级3级
	NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStr);  //初始化NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  //使能TIMx
}

void TIM3_Int_Init(u16 arr,u16 psc,uint8 Preem, uint8 Resq)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStr;
	NVIC_InitTypeDef NVIC_InitStr;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStr.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStr.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStr.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStr.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStr); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStr.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = Preem;  //先占优先级0级
	NVIC_InitStr.NVIC_IRQChannelSubPriority = Resq;  //从优先级3级
	NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStr);  //初始化NVIC寄存器

	//TIM_Cmd(TIM3, ENABLE);  //使能TIMx					 
}



void TIM4_Int_Init(u16 arr,u16 psc, uint8 Preem, uint8 Resq)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStr;
	NVIC_InitTypeDef NVIC_InitStr;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM3初始化
	TIM_TimeBaseStr.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStr.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStr.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStr.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStr); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStr.NVIC_IRQChannel = TIM4_IRQn;  //TIM3中断
	NVIC_InitStr.NVIC_IRQChannelPreemptionPriority = Preem;  //先占优先级0级
	NVIC_InitStr.NVIC_IRQChannelSubPriority = Resq;  //从优先级3级
	NVIC_InitStr.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStr);  //初始化NVIC寄存器

	TIM_Cmd(TIM4, ENABLE);  //使能TIMx
}



//定时器2中断服务程序
void TIM2_IRQHandler(void)   //TIM2中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		Sec++;
		if(Sec>1000000)Sec=0;
		if(Servo_flag) Servo_ms++;
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}


//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
	{
		Sec_buf++;
		if(Sec_buf>1000000)
			Sec_buf=0;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}


/////////////////////  usmart.c文件使用	 ///////////
////定时器4中断服务程序
//void TIM4_IRQHandler(void)   //TIM4中断
//{
//	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
//	{
//		//中断逻辑····
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIMx更新中断标志 
//	}
//}

//#include "stm32f103_capture.h"  用于输入捕获
//定时器5中断服务程序
//void TIM5_IRQHandler(void)   //TIM5中断
//{
//	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
//	{
//		//中断逻辑····
//		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //清除TIMx更新中断标志 
//	}
//}








