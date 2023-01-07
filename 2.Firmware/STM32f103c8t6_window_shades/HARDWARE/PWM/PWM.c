/*****************************************************************************************************************************************************														
基础知识储备：
TIM1_ETR     PA12            
TIM1_CH1     PA8            TIM2_CH1_ETR PA0            TIM3_CH1     PA6            TIM4_CH1     PB6 
TIM1_CH2     PA9            TIM2_CH2     PA1            TIM3_CH2     PA7            TIM4_CH2     PB7
TIM1_CH3     PA10           TIM2_CH3     PA2            TIM3_CH3     PB0            TIM4_CH3     PB8 
TIM1_CH4     PA11           TIM2_CH4     PA3            TIM3_CH4     PB1            TIM4_CH4     PB9  

 串口1： PA9-TX  PA10-RX      串口2：PA2-TX  PA3-RX      串口3： PB10-TX PB11-RX
 
		 SG90舵机控制方法：
                         --高电平时间为T1--                          总周期20ms:
                            —— —— —— ——
                           |           |                              高电平时间T1为 0.5ms     舵机转动0°
													 |           |                              高电平时间T1为   1ms     舵机转动45°
													 |           |                              高电平时间T1为 1.5ms     舵机转动90°
													 |           |                              高电平时间T1为   2ms     舵机转动135°
														             —— —— —— —— —— ——            高电平时间T1为 2.5ms     舵机转动180°
                           --------总周期 T2为20ms--------
												 
			代码思路：1、Ctrl + c  然后  Ctrl + v     哈哈哈哈！				

											|                
											|
											|
								--------------
                
								以180度舵机为例定义：高电平时间T1为 1.5ms     舵机转动90°    为     0°
								
								           相对而言：高电平时间T1为 0.5ms     舵机转动0°     为   -90°
                                     高电平时间T1为   1ms     舵机转动45°    为   -45°
																		 高电平时间T1为 1.5ms     舵机转动90°    为     0°
                                     高电平时间T1为   2ms     舵机转动135°   为   +45°
																		 高电平时间T1为 2.5ms     舵机转动180°   为   +90°
																		 
																		 
		//-90°    舵机转动0°
		//TIM_SetCompare1(TIM4,1945);//占空比(2000-1945)/2000*20ms=0.5ms 
		
		//-45°    舵机转动45°
		//TIM_SetCompare1(TIM4,1900);//占空比(2000-1900)/2000*20ms=1ms
		
		//0°      舵机转动90°
		//TIM_SetCompare1(TIM4,1850);//占空比(2000-1850)/2000*20ms=1.5ms
		
		//+45°    舵机转动135°
		//TIM_SetCompare1(TIM4,1800);//占空比(2000-1800)/2000*20ms=2ms
		
		//+90°    舵机转动180°
		//TIM_SetCompare1(TIM4,1750);//占空比(2000-1750)/2000*20ms=2.5ms
	
	
*************************************************************************************************************************************************/

#include "PWM.h"

//TIM4 PWM部分初始?
//PWM输出初始化
//arr:自动重装载值        psc:时钟预分频数


/******************************************************************************
* 模块描述
* 项目代号或名称 ：  8路PWM输出
* 创建日期       ：  2022/05/14
* 创建人         ：  志城
* 模块功能       ：  
* 修改日期       ：
* 参考文档       ：  精通STM32F4库函数版
* 项目运行平台   ：  STM32F103C8T6
* 其它           ：  PWM的一般步骤
										 实例要求：TIM4来产生PWM输出,并使能TIM4的通道1、2、3、4,逐个重映射到PB6\PB7\PB8\PB9,产生PWM来控制舵机

											1、使能定时器和相关IO时钟，调用函数:RCC_APB1PeriphClockCmd();RCC_APB2PeriphClockCmd();
											2、初始化IO为复用功能输出，调用函数:GPIO_Init();这里我们把PB6\PB7\PB8\PB9用作定时器的PWM输出引脚,要重映射配置即GPIO_Mode_AF_PP;
											   复用推挽输出；所以需要开启AFIO时钟。
												 
											3、初始化定时器，调用函数:TIM_TimeBaseInit();
											4、初始化输出比较参数，调用函数:TIM_OCInitStructure();
											5、使能预装载寄存器，调用函数:TIM_OC1PreloadConfig();
											6、使能定时器，调用函数:TIM_Cmd();											
											7、设置舵机初始角度，调用函数：TIM_SetCompare1(TIM4,1945);
															
基础知识储备：
TIM1_ETR     PA12            
TIM1_CH1     PA8            TIM2_CH1_ETR PA0            TIM3_CH1     PA6            TIM4_CH1     PB6 
TIM1_CH2     PA9            TIM2_CH2     PA1            TIM3_CH2     PA7            TIM4_CH2     PB7
TIM1_CH3     PA10           TIM2_CH3     PA2            TIM3_CH3     PB0            TIM4_CH3     PB8 
TIM1_CH4     PA11           TIM2_CH4     PA3            TIM3_CH4     PB1            TIM4_CH4     PB9  

					
//TIM3部分重映射
	
	*查看数据手册，引脚的定时器通道是完全映射，还是部分映射
	*二者调用参数不相同
	*完全映射 ：GPIO_FullRemap_TIM4
	*部分映射 ：GPIO_PartialRemap_TIM4
	*
   
	//设置TIM4_CH1重映射到PB6上
  //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM4,ENABLE);
*******************************************************************************/


void TIM4_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;   					 				//初始化TIM4设置ARR,PSC控制输出PWM的周期
	TIM_OCInitTypeDef  TIM_OCInitStructure;                         //PWM通道设置
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);     			  //使能定时器4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);      				//AFIO复用功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  ,ENABLE);   				//GPIOB使能
	
	
	//设置该引脚为复用输出功能
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;     //				GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	       //TIM4_CH1/CH2/CH3/CH4
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;                                   //复用推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);                                             //初始化GPIO
	
	
	//初始化TIM4,设置TIM4的ARR和PSC
	TIM_TimeBaseStruct.TIM_Period = arr;                   					 //设置自动重装载周期值   //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStruct.TIM_Prescaler = psc;                					 //设置预分频值           //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;               				 //设置时钟分割TDTS = Tck_tim	
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;         //TIM向上计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStruct);                      //根据指定参数初始化TIMx  //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	
	
	
	//输出初始化函数，选择四个通道，初始化PWM输出模式、比较输出极性、 比较输出使能、 比较值CCRx的值
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                                 //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);                            //根据TIMX的参数设定初始化外设 TIM1 ch1 ch2 ch3 ch4	
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);
  TIM_OC4Init(TIM4,&TIM_OCInitStructure);

	
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);  //使能预装载寄存器
  TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能TIMx在ARR上的预装载寄存器	
	
	TIM_Cmd(TIM4,ENABLE);     //使能TIM4
	
	
	//参数设置函数
	//作用：在四个通道中选择一个，设置比较值。通常在初始化函数中已经设置了比较值，此函数用于除初始化之外的修改
	//TIM_SetCompare1(TIM4,1850);
	//TIM_SetCompare2(TIM4,1850);
	//TIM_SetCompare3(TIM4,1850);
	//TIM_SetCompare4(TIM4,1850);
	
	

}








