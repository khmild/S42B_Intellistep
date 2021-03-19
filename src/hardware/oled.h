#ifndef __OLED_H__
#define __OLED_H__

// Imports
#include "config.h"
//#include "SSD1306.h"
#include "cstring"
#include "motor.h"
#include "encoder.h"
#include "main.h"
#include <algorithm>
#include "stm32f1xx_ll_gpio.h"

// Variables
extern String topLevelMenuItems[];
extern const int topLevelMenuLength;

void initOLED();
void writeOLEDString(uint8_t x, uint8_t y, String string);
void clearOLED();
void updateDisplay();
void displayMotorData();
void selectMenuItem();
void moveCursor();
void exitCurrentMenu();
int getMenuDepth();

// A large amount of low level commands for handling the useage of fast GPIO manipulation
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C //
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808  
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))


#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // 


#define OLED_CS  PBout(12)		//
#define OLED_RST PAout(8) 		//
#define OLED_RS  PBout(13)		//

//#define OLED_SCLK  PAout(11)		///(D0)
//#define OLED_SDIN  PAout(12)	    ////(D1)
////PC0~7
//#define DATAOUT(x) GPIOC->ODR=(GPIOC->ODR&0xff00)|(x&0x00FF); //

//
#define OLED_SCLK PBout(15)		//(D0)
#define OLED_SDIN PBout(14)		//(D1)


// BTT Display definitions

#define OLED_CMD  0	//
#define OLED_DATA 1	//
//
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	//

void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);							//

void OLED_Init(void);									//
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);					//

void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);			//
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);	//
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);	//
void OLED_ShowString(uint8_t x,uint8_t y,const char *p);	 		//

uint32_t oled_pow(uint8_t m,uint8_t n);//
void OLED_Showword(uint8_t x,uint8_t y,uint8_t *num,uint8_t mode);

#endif