#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"
#include "system_stm32f1xx.h"

void stm32_SysClock72M(uint32_t PLLMultiplier);//
void delay_init(u8 SYSCLK);//
void delay_us(u32 nus);//
void delay_ms(u32 nms);//
void System_Clock_Init(void);



#endif















