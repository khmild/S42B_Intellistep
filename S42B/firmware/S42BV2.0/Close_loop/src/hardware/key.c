#include "key.h"
#include "gpio.h"
#include "oled.h"
#include "tle5012.h"
#include "flash.h"

bool KEY_Select_flag = false;
bool KEY_Back_flag = false;
bool KEY_Confirm_flag = false; 

uint8_t Enter_exit_flag = 0;
uint8_t enter_num = 0; 


void Key_init(void)
{
  // ! Check out
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

  // Back pin (0), set the pin for input at 50Mhz sampling
  GPIO_InitTypeDef GPIO_0_InitStruct;
  GPIO_0_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_0_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_0_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_0_InitStruct);

  // Confirm pin (1)
  GPIO_InitTypeDef GPIO_1_InitStruct;
  GPIO_1_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_1_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_1_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_1_InitStruct);

  // Select pin (3) (on GPIOA)
  GPIO_InitTypeDef GPIO_3_InitStruct;
  GPIO_3_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_3_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_3_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_3_InitStruct);

}

// Test the keys
void Key_test(void)
{
  // Check to see if the select key is clicked
  if(checkSelectKey()){

    // Trigger the select key flag if it isn't
    if(!KEY_Select_flag){

      // Set the flag to true if it wasn't already
      KEY_Select_flag = true;

      // Invert the LED output (flash it)
      led1 = !led1;
    }
  }

  // Check the back key
  else if(checkBackKey()){

    // Trigger the back key flag if it isn't
    if(!KEY_Back_flag){

      // Set the flag to true if it wasn't already
      KEY_Back_flag = true;

      // Invert the LED output (flash it)
      led1 = !led1;
    }
  }

  // Check the confirm key
  else if(checkConfirmKey()){

    // Make sure that we're not interrupted
    if(!KEY_Confirm_flag){

      // Set the flag to true if it wasn't already
      KEY_Confirm_flag = true;

      // Invert the LED output (flash it)
      led1 = !led1;
    }
  }

  // Otherwise, set all of them to false
  else {
    KEY_Select_flag = false;
    KEY_Back_flag = false;
    KEY_Confirm_flag = false;
  }
}

// Scan each of the keys
void KeyScan(void) {

  // Check the select key
  if(checkSelectKey()){

    if(!KEY_Select_flag){
      PHASE_MULTIPLIER--;
      dataUpdateFlag = true;
      delayMs(100);
    }
    /*

    // Check if we're in the menu
    if(Menu_Num == 0){
      
      // Make sure that the key click wasn't processed yet
      if(!KEY_Select_flag){

        // Set the update flags
        KEY_Select_flag = true;
        menuUpdateFlag = true;
        
        if(!isCalibrated()){
            Menu_Num_item++;
            if(Menu_Num_item>1)
                Menu_Num_item=0;
        }
        else if(Second_Menu==1){
                Menu_Num_item++;
                if(Menu_Num_item > MENU_NUM){
                    Menu_Num_item =2;
            }
        }
        else if(Second_Menu==2){
                Menu_Num2_item++;
                if(Menu_Num2_item>33)
                    Menu_Num2_item=0;
        }
        else if(Second_Menu==3){
                Menu_Num3_item++;
                if(Menu_Num3_item>4)
                    Menu_Num3_item=0;
        }
        else if(Second_Menu==4){
                Menu_Num4_item++;
                if(Menu_Num4_item>1)
                    Menu_Num4_item=0;
        }
        else if(Second_Menu==5){
                Menu_Num5_item++;
                if(Menu_Num5_item>1)
                    Menu_Num5_item=0;
        }
      }
    }
    */
  }
  // Check the back key
  else if(checkBackKey()){

    // Make sure that the back key hasn't been dealt with yet
    if(!KEY_Back_flag){

      // Update the state flags
      KEY_Back_flag = true;
      menuUpdateFlag = true;

      // Clear the OLED for writing
      OLED_Clear();

      // ! Check
      if(Enter_exit_flag == 1){
          Enter_exit_flag=0;
          Menu_Num=0;
          Menu_Num_item=2;
          Second_Menu=1;
      }else {
          Menu_Num++;
          if(Menu_Num >1 ){
              Menu_Num=0;
          }
          dataUpdateFlag = true;
      }
    }
  }

  // Check the confirm key
  else if(checkConfirmKey()){

    // Make sure that we're not already dealing with a key
    if(!KEY_Confirm_flag){
      PHASE_MULTIPLIER++;
      dataUpdateFlag = true;
      delayMs(100);

      /*

      // Set the flags to the correct state
      KEY_Confirm_flag = true;

      // Check the current menu
      switch(Menu_Num){
        case 0:     
          switch(Menu_Num_item){
              case 0:
                SetModeCheck();
                break;
              case 1: 
                Enter_exit_flag = 1;
                Menu_Num = 1;
                menuUpdateFlag = true;
                OLED_Clear();
              break ;
              case 2: 
                      TIM_Cmd(TIM2,DISABLE);
                      enmode=0;
                      NVIC_DisableIRQ(EXTI1_IRQn);
                      Second_Calibrate_flag=1;
                      SetModeCheck();
                      NVIC_EnableIRQ(EXTI1_IRQn);
                      TIM_Cmd(TIM2,ENABLE);
                      Second_Menu=1;
                  break ;
              case 3:
                      OLED_Clear();   //
                      menuUpdateFlag = true;   //
                      enter_num++;
                      if(enter_num==1)
                          Second_Menu=2;        //
                      if(enter_num==2){
                          enter_num=0;
                          Second_Menu=1;
                          //
                          //
                          Currents= Currents_Set;//
                          flashStoreFlag = true;
                          table1[1]=Currents;
                          table1[2]=Menu_Num2_item;
                      }
                  break ;
              case 4: OLED_Clear();   //
                      menuUpdateFlag = true;   //
                      enter_num++;
                      if(enter_num==1)
                          Second_Menu=3;        //
                      if(enter_num==2){
                          enter_num=0;
                          Second_Menu=1;
                          enmode =0;
                          PID_Cal_value_init();

                          stepangle = Microstep_Set;
                          
                          flashStoreFlag = true;
                          table1[3]=stepangle;
                          table1[4]=Menu_Num3_item;
                      }
                  break ;
              case 5: OLED_Clear();   //
                      menuUpdateFlag = true;   //
                      enter_num++;
                      if(enter_num==1)
                          Second_Menu=4;        //
                      if(enter_num==2){
                          enter_num=0;
                          Second_Menu=1;
                          enmode =0;
                          PID_Cal_value_init();
                          
                          if(Dir_Enable == 0xaa ){
                              //enmode =1;
                            motorEnabled = false;
                          }else if(Dir_Enable == 0x55  ){
                              motorEnabled = true;
                              //enmode =0;
                          }
                          if(Dir_Enable == 0x55 || Dir_Enable == 0xAA){
                              Dir_Enable =0;
                              flashStoreFlag = true;//
                              table1[5] = motorEnabled;
                              table1[6]=Menu_Num4_item;
                              
                          }
                      }
                  break ;
              case 6: OLED_Clear();   //
                      menuUpdateFlag = true;   //
                      enter_num++;
                      if(enter_num==1)
                          Second_Menu=5;        //
                      if(enter_num==2){
                          enter_num=0;
                          Second_Menu=1;
                          
                          //
                          //Dir_Enable  ;
                          if(Dir_Enable ==0x11){
                              positiveDirection = true;
                          }else if(Dir_Enable == 0x22){
                              positiveDirection = false;
                          }
                          Dir_Enable=0;
                          flashStoreFlag = true;
                          table1[7] = positiveDirection;
                          table1[8] = Menu_Num5_item;
                      }
                  break ;
              case 7:Enter_exit_flag=1;    //
                      Menu_Num=1;           //  
                      menuUpdateFlag = true;   //
                      OLED_Clear();         //
                  break;
              default:break;
            }
        break;
        }
        */
      }
    }
    else {
        KEY_Select_flag = false;       //
        KEY_Back_flag = false;         //
        KEY_Confirm_flag = false;
    }
}

// Check select key function
bool checkSelectKey() {

  // Check to see if the select key is clicked
  if(KEY_Select == 0){

    // Wait to make sure it wasn't a misread
    delayMs(10);

    // Check again
    if(KEY_Select == 0){

      // The key is still clicked, it's valid
      return true;

    }
    else {

      // The key is no longer clicked, probably just a misread
      return false;
    }

  }
  else {

    // Key was never clicked, it's not clicked
    return false;
  }
}

// Check back key function
bool checkBackKey() {

  // Check to see if the back key is clicked
  if(KEY_Back == 0){

    // Wait to make sure it wasn't a misread
    delayMs(10);

    // Check again
    if(KEY_Back == 0){

      // The key is still clicked, it's valid
      return true;

    }
    else {

      // The key is no longer clicked, probably just a misread
      return false;
    }

  }
  else {

    // Key was never clicked, it's not clicked
    return false;
  }
}

// Check confirm key function
bool checkConfirmKey() {

  // Check to see if the confirm key is clicked
  if(KEY_Confirm == 0){

    // Wait to make sure it wasn't a misread
    delayMs(10);

    // Check again
    if(KEY_Confirm == 0){

      // The key is still clicked, it's valid
      return true;

    }
    else {

      // The key is no longer clicked, probably just a misread
      return false;
    }

  }
  else {

    // Key was never clicked, it's not clicked
    return false;
  }
}