#ifndef __TLE5012_H
#define __TLE5012_H

#include "Arduino.h"
#include "stm32yyxx_ll_spi.h"
#include "stm32yyxx_ll_gpio.h"
#include "pinMapping.h"
#include "SPI.h"
#include "TLE5012-ino.hpp"

/*
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

//extern SPIClass3W encoderSPI;
extern Tle5012b encoder;

uint16_t readEncoderValue(uint16_t registerValue);
uint16_t readEncoderState();
double getEncoderAngle();
double getEncoderTemp();

#endif

