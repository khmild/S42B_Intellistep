#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "usart.h"
#include "math.h"
#include "stdbool.h"

// Motor constants
#define PHASE_MULTIPLIER 12.5f
#define STEP_ANGLE 81.92f // ! Check to see for .9 deg motors as well

/* SPI command for TLE5012 */
#define CMD_READ_STATUS				0x8001			//8000
#define CMD_READ_ANGLE_VALUE		0x8021			//8020
#define CMD_READ_SPEED_VALUE		0x8031			//8030

#define WRITE_MOD1_VALUE		0x5060			//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001

#define WRITE_MOD2_VALUE		0x5080			//0_1010_0_001000_0001
#define MOD2_VALUE	0x0800

#define WRITE_MOD3_VALUE		0x5091			//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000

#define WRITE_MOD4_VALUE		0x50E0			//0_1010_0_001110_0001
#define MOD4_VALUE	0x0098						//9bit 512

#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 0x000D

/* Functionality mode */
#define REFERESH_ANGLE		0

#define UMAXCL   62    //
#define UMAXOP   160    //
#define UMAXSUM  25600  //


extern int16_t kp;     
extern int16_t ki;
extern int16_t kd;

extern const uint8_t LPFA; 
extern const uint8_t LPFB;


extern int32_t s;
extern int32_t s_1;
extern int32_t s_sum;
extern int32_t r;   
extern int32_t r_1;   
extern bool positiveDir; 
extern int16_t y;   
extern int16_t y_1;
extern int32_t yw;  
extern int32_t yw_1;
extern int16_t advance;
extern int32_t wrap_count; 
extern int32_t error;  
extern int32_t iterm;
extern int32_t dterm;
extern int16_t u;     
extern int32_t stepNum;
extern uint8_t stepangle;

extern uint16_t hccount;
extern bool closedLoopMode;
extern uint8_t enmode;

extern uint8_t Currents;
extern uint8_t Microstep;
extern bool positiveDirection;

extern volatile bool menuUpdateFlag;      //
extern volatile bool dataUpdateFlag;  
extern volatile uint16_t Data_update_Count;
extern uint8_t Menu_Num;             //
extern uint8_t Menu_Num_item;         //
extern uint8_t Menu_move_num;         //

extern uint8_t CalibrateEncoder_finish_flag; //
extern uint8_t Second_Calibrate_flag; //
extern uint8_t Second_Menu;           //
extern uint8_t Menu_Num2_item;        //  
extern uint8_t Menu_Num3_item;        //  
extern uint8_t Menu_Num4_item;        //  
extern uint8_t Menu_Num5_item;        //  
extern uint8_t Menu_Num6_item;        //  
extern int16_t Motor_speed;
extern int16_t wap1;
extern int16_t wap2;
extern uint8_t Motor_speed_flag;
extern volatile bool enableModeFlag;
extern volatile uint8_t resetStatusFlag;

extern volatile bool flashStoreFlag;
extern volatile uint8_t info_report_flag;
extern uint16_t table1[14];                    //

extern uint8_t Rx1_buff[9];
extern uint8_t Receive_Count;
extern uint8_t Rx1_temp_num;                       
extern uint8_t Receive_finish_flag;          //
extern volatile uint8_t Communications_Process_flag;     //
extern volatile uint8_t uartCRCFlag;                   //  
extern volatile uint8_t frameErrorFlag;                //
extern volatile uint8_t uartCRCCorrectFlag;           //
extern volatile int16_t temp; // ! Fix later
extern uint8_t Currents_Set;
extern uint8_t Microstep_Set;            //

extern uint8_t Dir_Enable;           // 
extern bool positiveDirection;                  //
extern bool motorEnabled;               //
extern uint8_t receivePulse;

extern volatile uint8_t enter1_once_flag;    //
extern volatile uint8_t enter2_once_flag;
extern volatile bool dir1_once_flag;      //
extern volatile bool dir2_once_flag;

extern void Output(int32_t theta,uint8_t effort);
extern uint16_t ReadValue(uint16_t RegValue);
extern uint16_t ReadAngle(void);



#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif




#endif



