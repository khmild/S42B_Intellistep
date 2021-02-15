/**
  ******************************************************************************
  * @file    main.c
  * @author  Vsion yang
  * @version V1.0.0
  * @date    2019.8.5
  * @brief   
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  ******************************************************************************
 */
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "time.h"
#include "exit.h"
#include "buttons.h"
#include "can.h"
#include "display.h"
#include "flash.h"
#include "encoder.h"
#include "iwdg.h"
#include "oled.h"
#include "stm32yyxx_ll_rcc.h"
#include "SSD1306.h"

// Setup the table for parameters
uint16_t savedParameters[] = {'1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','0'};

// Create a new motor instance
StepperMotor motor = StepperMotor();

// Run the setup
void setup() {

    uint8_t canbufTx[]="12345678";
    
    // Setup the system clock (includes overclocking)
	overclock(RCC_CFGR_PLLMULL12);

    // Load all of the values saved in flash
    loadSavedValues();

    // Initialize the LED
    pinMode(LED_PIN, OUTPUT);
    
    // Initialize the OLED
	SSD1306_Init();

    // Give the user feeback that OLED starting has been successful
    writeOLEDString(0, 0, "Oled Init...OK");

    // Wait for .1 seconds so everything can boot
    delay(100);

    // Initialize the buttons (for menu)
    initButtons();

    /*
    // Test the flash if specified
    #ifdef TEST_FLASH
        flash_test(); 
    #endif
    */
    // Setup the closed loop information if it is enabled 
    #ifdef CLOSED_LOOP_CONFIG

        // Clear the display, then write that we're using the closed loop model
        clearOLED();
        writeOLEDString(0, 0, "Close Loop Model");

        // Initialize the encoder, motor, and the PWM timer
        //encoderInit();
        //motorInit();
        //TIM3_Init();
        
        // ! if(isCalibrated()){

            // Let the user know that the calibration was successfully loaded
            writeOLEDString(0, 25, "  Calibration  ");
            writeOLEDString(40, 45, "  OK!");
            delay(500);

            // Start displaying the motor data
            clearOLED();
            writeOLEDString(0, 2, "Simp:  0000 RPM");//
            writeOLEDString(0, 22, " Err:  000.00 ");
            writeOLEDString(0, 42, " Deg:  0000.0");
            //Menu_Num_item=2;
            //Second_Menu=1;
            //menuUpdateFlag = false;
            //Menu_Num=1;  
            //resetStatusFlag=1;
            //Menu_Num2_item=8;
            //Motor_Enable = enmode;
            // *****************************************************
            /*
            flashRead(DATA_STORE_ADDRESS, parameterTable);            //
            SetModeCheck();                 //

            Currents=table1[1];             //
            Menu_Num2_item =table1[2];
            
            stepangle=table1[3];             //
            Menu_Num3_item=table1[4];        //
            
            enableModeFlag = table1[5];
            Menu_Num4_item= table1[6];  
            
            positiveDirection =table1[7];            //
            Menu_Num5_item =table1[8];
            
    //        Motor_Dir =table1[9];           //
    //        Menu_Num5_item =table1[10];
            */

            motor.setPValue(savedParameters[11]); // P term of PID
            motor.setIValue(savedParameters[12]); // I term of PID
            motor.setDValue(savedParameters[13]); // D term of PID
        // ! }
        // ! else {

            // Display that the motor is not calibrated
            writeOLEDString(48, 16, "NOT");
            writeOLEDString(16, 32, "Calibrated!");
            writeOLEDString(0, 48, "Please calibrate");
            delay(500);

            // ! Comment
            clearOLED();
            writeOLEDString(0, 0, "->");

            // Loop forever, checking the keys and updating the display
            while(true) {
                checkButtons();
                updateDisplay();
            }            
        // ! }
        /*
        EXTIX_Init();                       //
        NVIC_EnableIRQ(EXTI1_IRQn);         //
        UART_Configuration(USART1, UART1_DEFAULT_BAUDRATE);     //
        CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_5tq, 6, CAN_Mode_LoopBack);// 
        res = CAN1_Send_Msg(canbufTx, 8);                      //
        delayMs(100);
        if(res){                                            //
            printf("CAN1 Transport fail!\r\n");
        }
        else{
            printf("CAN1 Transport OK!\r\n");
        }
        TIM2_Cap_Init(0xFFFF,0);          //

        // ! Add logic checking to make sure that this is always a whole number 
        //TIM4_Init(10000 / STEPPER_UPDATE_FREQ, 7200);
        // Calculate the values for the timer given the interrupt frequency (tests the maximum prescalar available)
        for(int prescalarDivisor = 1; SystemCoreClock / (STEPPER_UPDATE_FREQ * prescalarDivisor) > 65536; prescalarDivisor = prescalarDivisor * 10) {

            // Check if the timer can be initialized with the values before the loop exits
            if (SystemCoreClock / (STEPPER_UPDATE_FREQ * prescalarDivisor) < 65536) {
                TIM4_Init(prescalarDivisor-1, SystemCoreClock / (STEPPER_UPDATE_FREQ * prescalarDivisor));
            }
        }

        */
    #endif
    /*
//    IWDG_Init(4,625);                 //
 	while(1) {
        #ifdef CLOSED_LOOP 
            if(enableModeFlag){
                if(getEnablePin()) {
                    restart_init();              
                } 
                else{
                    resetStatusFlag++;
                    enmode = 0;
                }    
            }
            else {
                if(!getEnablePin()){
                    restart_init();
                }
                else{
                    resetStatusFlag++;
                    enmode=0; //
                }
            }
            if(resetStatusFlag == 1){       //
                enmode = 0;
                resetStatusFlag++;           //1++            
                PID_Cal_value_init();           //
                    
                wap1 = 0;
                wap2 = 0;
                dataUpdateFlag = true;
            }
            else{
                if(resetStatusFlag > 3)
                    resetStatusFlag--;
            }
            
            usart_Receive_Process();                        //
            data_Process();
            test_uart();
            
            if(frameErrorFlag){
                frameErrorFlag =0;
                USART1_SendStr("Frame Err!\r\n");   
            }
            if(flashStoreFlag){
                flashStoreFlag = false;

                //USART_Cmd(USART1, DISABLE);
                USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
                
                flashWrite(DATA_STORE_ADDRESS, table1);//
                
                //USART_Cmd(USART1, ENABLE);
                USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
                USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
                
                //Reset_status_flag=1;
                //restart_init();
            }
            KeyScan();                                      //
            Oled_display();                                 //
            Motor_data_dis();                               //
            //TIM4_IRQHandler();
    #endif
#if 0
        if(KEY_Select==0){
			delay_ms(10);
			if(KEY_Select==0){
                if(k3_flag == 0){
                    k3_flag =1;
                    led1=!led1;			//
                    res=CAN1_Send_Msg(canbufTx,8);//
                    if(res)
                        printf("CAN1 Transport fail!\r\n");
                    else
                        printf("OK!\r\n");
                }
			}
		}
        else if(KEY_Confirm==0){
			delay_ms(10);
			if(KEY_Confirm==0){
                if(k4_flag == 0){
                    k4_flag =1; 
                    mode=!mode;
                    CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_5tq,6,mode);	//
                     
                    if(mode==0)//
                    {
                        printf("Nnormal Mode \r\n");	    
                    }else //
                    {
                        printf("LoopBack Mode\r\n");
                    }                    
                    led1=1;			//off
                    delay_ms(200);
                    led1=0;			//on
                    delay_ms(200);
                    led1=1;			//off
                    delay_ms(200);
                    led1=0;			//on
                    delay_ms(200);
                }
            }
		}
        else{
            k3_flag = 0;
            k4_flag = 0;
        }
        key=CAN1_Receive_Msg(canbufRx);
		if(key)//
		{	
            printf("CAN Receive: ");	
 			for(i=0;i<key;i++){		
                printf("%x ",canbufRx[i]);
 			}
            printf("\r\n");
		}
        key=0;
#endif
	}
*/
}
/*
//
void restart_init(void)
{
    //
    if(resetStatusFlag !=0 ){
        CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); 
        PID_Cal_value_init();           //
        SET_BIT(TIM2->CR1, TIM_CR1_CEN);
    }
    enmode=1;
    resetStatusFlag=0;
}

#ifdef TEST_FLASH 
void flash_test(void)
{
    static char t=0;
    STMFLASH_Write(Data_Store_Arrdess,table,16);
    delay_ms(10);
    FLASH_Unlock();						//
    STMFLASH_Read(Data_Store_Arrdess,table1,16);
    FLASH_Lock();//
     printf("flash Read: ");	
    for(t=0;t<16;t++){		
        printf("%c ",table1[t]);
    }
    while(1);
}
#endif

*/
void loop() {

}


void overclock(uint32_t PLLMultiplier) {

	// Use PLL as the system clock instead of the HSE (the board's oscillator)
	RCC -> CFGR |= RCC_CFGR_PLLSRC;

	// Activate the HSE (board's oscillator)
	RCC -> CR |= RCC_CR_HSEON;

	// Set the multiplier to 6x
	RCC -> CFGR |= PLLMultiplier;

	// Set the HSE to half speed before the PLL (so effectively multiplier/2 speed overall)
	RCC -> CFGR |= RCC_CFGR_PLLXTPRE_HSE_DIV2;

	// Activate the PLL
	RCC -> CR |= RCC_CR_PLLON;

	// Wait until the PLL is configured
	while(!(RCC_CR_PLLRDY & RCC -> CR));

	// Use the PLL as the system clock
	RCC -> CFGR |= RCC_CFGR_SW_PLL;

	// Update the system clock with the new speed
	SystemCoreClockUpdate();
}


















/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
