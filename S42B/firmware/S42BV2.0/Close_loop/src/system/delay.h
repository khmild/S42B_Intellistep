#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "system_stm32f1xx.h"

void overclock(uint32_t PLLMultiplier);//
void delayInit(void);//
void delayUs(u32 nus);//
void delayMs(u32 nms);//
void System_Clock_Init(void);



#endif















