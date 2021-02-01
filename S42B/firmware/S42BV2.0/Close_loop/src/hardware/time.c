#include "time.h"
#include "gpio.h"
#include "iwdg.h"
#include "oled.h"


void TIM2_Cap_Init(u16 timerPeriod,u16 prescalar)
{	 
  TIM_DeInit(TIM2);
    
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;  //PA0   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = timerPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = prescalar; 	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //
  
  TIM_ARRPreloadConfig(TIM2,DISABLE);
  TIM2 -> SMCR |= TIM_AutomaticOutput_Enable; // ! Not fully sure about this one
    
  TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 7);
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Disable);
    
  TIM_SetCounter(TIM2, 0);
  TIM_Cmd(TIM2, ENABLE);
}

void SetT1Pwm1(uint16_t Duty) {
  TIM1 -> CCR4 = Duty; 
}

void TIM3_Init(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef  GPIO_InitStruct;
//    GPIO_AFIODeInit();
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;			
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

    
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 1;
	TIM_TimeBaseInitStruct.TIM_Period = 256;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);

	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Disable);

  TIM_ARRPreloadConfig(TIM3, DISABLE);
    
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_Pulse = 3;
    
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);
  TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);

	TIM_OC2Init(TIM3,&TIM_OCInitStruct);
  TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
    
//    TIM_ARRPreloadConfig(TIM3,ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
//
void TIM4_Init(u16 arr,u16 psc)
{	 
//	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//
		
	// TIM4	setup
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = arr; // 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //
  TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	// Interrupt setup
 	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //
	NVIC_Init(&NVIC_InitStructure);  // 

  // Register the interrupt handler
  // ! Throws errors (internal errors, not shown on the compiler)
  //NVIC_SetVector(TIM4_IRQn, (uint32_t) &TIM4_IRQHandler);
  
  NVIC_EnableIRQ(TIM4_IRQn);
}

// Handles the update of the stepper motor 
void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
//            led1=!led1;
        //IWDG_Feed();//
        if(enmode==1)
        {
          //SET_BIT(TIM3->CR1, TIM_CR1_CEN);
          if(closedLoopMode) {
            y = *(volatile uint16_t*)((ReadValue(CMD_READ_ANGLE_VALUE)>>1)*2+0x08008000);//
            s = TIM_GetCounter(TIM2);//
            if(s-s_1<-32768)
              s_sum+=stepangle*65536;
            else if(s-s_1>32768)
              s_sum-=stepangle*65536;
            r=s_sum+stepangle*s;//
            s_1=s;
            
            if(y-y_1>8192) 
              wrap_count--;      
            else if(y-y_1<-8192) 
              wrap_count++; 
            yw=y+16384*wrap_count;//            
            error=r-yw;//
            if(error>1638)//
              error=1638;
            else if(error<-1638)
              error=-1638;
            iterm+=ki*error/32;//
            if(iterm>UMAXSUM)//
              iterm=UMAXSUM;
            else if(iterm<-UMAXSUM) 
              iterm=-UMAXSUM; 
        
               
            dterm=LPFA*dterm/128-LPFB*kd*(yw-yw_1)/8;//
            u=(kp*error+iterm+dterm)/128;//
            
            advance=(yw-yw_1)*1.5f;//
            y_1=y;  
            yw_1=yw;
        
            if(u>0)            
            {
              if(advance>68)//
                advance=68;
              else if(advance<0)
                advance=0; 		  
              y+=(82+advance);//
            }
            else if(u<0)
            {
              if(advance<-68)
                advance=-68; 
              else if(advance>0)
                advance=0; 
              y-=(82-advance);
              u=-u;
            }
            //
            #if 1
            if(u>Currents){
                u=Currents;//
                led1 = LED_ON;
            }
            #else 
            if(u>UMAXCL)     
            {
              u=UMAXCL;
              LED_H;
            }
            #endif
            else
              led1 = LED_OFF;
            Output(y,u);    
          }          
          else 
          {		
            s=TIM_GetCounter(TIM2);
            if(s-s_1<-32768)
              s_sum+=stepangle*65536;
            else if(s-s_1>32768)
              s_sum-=stepangle*65536;
            r=s_sum+stepangle*s;
            s_1=s;
            
            if(r==r_1)
            {
              hccount++;
              if(hccount>=1000)
                hccount=1000;
            }
            else
              hccount=0;
            
            if(hccount>=1000)//
              Output(r,UMAXOP/2);
            else
              Output(r,UMAXOP);
            r_1=r;
          }
         
          Data_update_Count++;
          if(Data_update_Count>=5000){       
              Data_update_Count-=5000;
              Motor_speed_flag++;
              if(Motor_speed_flag>=2){
                Motor_speed_flag=0;
                  
                  wap1=wap2;
                  wap2=wrap_count;
    //              Motor_speed = wap2-wap1;//
              }
            dataUpdateFlag=1;           
          }
        }
      }
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //
}


void Pulse_Control(u16 cycle, u16 pulse_num) {
	u16 arr3 = 0;
	u32 time = 0;
	if(pulse_num)
	{ 
		time = cycle * pulse_num / 100;
		arr3 = cycle / 10;             
		TIM_SetAutoreload(TIM2, time + 19);
		TIM_SetAutoreload(TIM3, arr3 - 1);
		TIM_SetCompare1(TIM3,arr3 / 2); 
		TIM_Cmd(TIM2,ENABLE);
       
	}
}
