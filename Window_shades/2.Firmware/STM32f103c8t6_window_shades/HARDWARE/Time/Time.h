#ifndef _TIME_H
#define _TIME_H

#include "stm32f10x.h"


extern u32 Sec;
extern u32 Sec_buf;

extern u8 Servo_flag;
extern u32 Servo_ms;


//通用定时器
void TIM2_Int_Init(u16 arr,u16 psc, uint8 Preem, uint8 Resq);
void TIM3_Int_Init(u16 arr,u16 psc, uint8 Preem, uint8 Resq);
void TIM4_Int_Init(u16 arr,u16 psc, uint8 Preem, uint8 Resq);

#endif


