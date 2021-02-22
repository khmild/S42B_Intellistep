#ifndef __TLE5012_H
#define __TLE5012_H

#include "Arduino.h"
#include "stm32yyxx_ll_spi.h"
#include "stm32yyxx_ll_gpio.h"
#include "pinMapping.h"
#include "SPI.h"

#define SPI_TX_OFF   {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0x40000000;}
#define SPI_TX_ON    {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xB0000000;}   

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

/*
#define CAL         PAin(15)
#define CLOSE       PBin(3)
#define SET1        PBin(10)
#define SET2        PBin(11)

#define DRIVER1_DIR_1         PBout(6)
#define DRIVER1_DIR_2         PBout(7)
#define DRIVER2_DIR_1         PBout(8)
#define DRIVER2_DIR_2         PBout(9)


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
void WriteValue(uint16_t RegAdd,uint16_t RegValue);
uint16_t ReadState(void);
void CalibrateEncoder(void);
*/

extern SPIClass encoderSPI;

uint16_t readEncoderValue(uint16_t registerValue);
uint16_t readEncoderState();
float getEncoderAngle();


#endif

