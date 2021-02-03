#ifndef __TLE5012_H
#define __TLE5012_H


#include "main.h"

#define SPI_TX_OFF   {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0x40000000;}
#define SPI_TX_ON    {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xB0000000;}   

#define	TLE012_CS   PAout(4) //	

#define CAL         PAin(15)
#define CLOSE       PBin(3)
#define SET1        PBin(10)
#define SET2        PBin(11)

#define DRIVER1_DIR_1         PBout(6)
#define DRIVER1_DIR_2         PBout(7)
#define DRIVER2_DIR_1         PBout(8)
#define DRIVER2_DIR_2         PBout(9)

#define ON    true
#define OFF   false

extern uint32_t PHASE_MULTIPLIER;
    
void encoderInit(void);
void motorInit(void);
void SetModeCheck(void);
void Output(int32_t theta,uint8_t effort);
int16_t Mod(int32_t xMod,int16_t mMod);
void OneStep(void);
int fputc(int c,FILE *stream); 
int fgetc(FILE *stream); 
void SerialCheck(void);
uint16_t ReadValue(uint16_t RegValue);
void WriteValue(uint16_t RegAdd,uint16_t RegValue);
uint16_t ReadState(void);
uint16_t ReadAngle(void);
void CalibrateEncoder(void);




#endif

