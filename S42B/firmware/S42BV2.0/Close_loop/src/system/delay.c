#include "delay.h"
unsigned int fac_us,fac_ms;


void overclock(uint32_t PLLMultiplier) {

	// Use PLL as the system clock instead of the HSE (the board's oscillator)
	RCC -> CFGR |= RCC_CFGR_PLLSRC;

	// Activate the HSE (board's oscillator)
	RCC -> CR |= RCC_CR_HSEON;

	// Set the multiplier to 6x
	RCC -> CFGR |= PLLMultiplier;

	// Set the HSE to half speed before the PLL (so effectively mult./2 x speed overall)
	RCC -> CFGR |= RCC_CFGR_PLLXTPRE_HSE_Div2;

	// Activate the PLL
	RCC -> CR |= RCC_CR_PLLON;

	// Wait until the PLL is configured
	while(!(RCC_CR_PLLRDY & RCC -> CR));

	// Use the PLL as the system clock
	RCC -> CFGR |= RCC_CFGR_SW_PLL;

	// Update the system clock with the new speed
	SystemCoreClockUpdate();

}
/*
void delayInit(u8 SYSCLK)//
{
	
	SysTick->CTRL&=0xfffffffb;//
	fac_us=SYSCLK/8;
	fac_ms=fac_us*1000;
}
*/

void delayInit(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;
    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

	/*
    // Check if clock cycle counter has started
    if(DWT->CYCCNT)
    {
       return 0; //clock cycle counter started
    }
    else
    {
      return 1; // clock cycle counter not started
    }
	*/
}

/*
void delayUs(u32 nus)//
{
	u32 temp;
//	delay_Init(72);//
	SysTick->LOAD=nus*fac_us;
	SysTick->VAL=0x00;//
	SysTick->CTRL=0x01;//
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));
	SysTick->CTRL=0x00;
	SysTick->VAL=0x00;
}
*/
void delayUs(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (SystemCoreClock / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds - au32_ticks);
}

void delayMs(volatile uint32_t au32_milliseconds)
{
  delayUs(au32_milliseconds * 1000);
}

/*
void delayMs(u32 nms)//
{
	u32 temp;
//	delay_Init(72);//
	SysTick->LOAD=nms*fac_ms;
	SysTick->VAL=0x00;//
	SysTick->CTRL=0x01;//
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));
	SysTick->CTRL=0x00;
	SysTick->VAL=0x00;
}
*/
void System_Clock_Init(void)
{
    SystemInit();
	overclock(RCC_CFGR_PLLMULL12);
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
 	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
}







