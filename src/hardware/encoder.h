#ifndef __TLE5012_H
#define __TLE5012_H

#include "Arduino.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio_ex.h"
#include "stm32f1xx_hal_rcc.h"
#include "config.h"
//#include "SPI.h"
//#include "TLE5012-ino.hpp"

#define SPI_TX_OFF   {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0x40000000;}
#define SPI_TX_ON    {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xB0000000;}

#define ENCODER_READ_REGISTER_BASE           0x8000
#define ENCODER_STATUS_REGISTER              (0x0000U)
#define ENCODER_ACTIVATION_REGISTER          (0x0010U)
#define ENCODER_ANGLE_REGISTER               (0x0020U) // (ENCODER_READ_REGISTER_BASE | (0x0020U)) //0x8021			//8020
#define ENCODER_SPEED_REGISTER		         (0x0030U) // (ENCODER_READ_REGISTER_BASE | (0x0030U))			//8030

#define ENCODER_WRITE_REGISTER_BASE          0x5000
#define MAX_REGISTER_MEM                     0x0030

// Constants for encoder calculations
#define DELETE_BIT_15 0x7FFF
#define CHECK_BIT_14 0x4000
#define CHANGE_UINT_TO_INT_15 0x8000
#define POW_2_15 32768


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
//extern Tle5012b encoder;

void initEncoder();

FlagStatus getSPIFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG);
void readSingleEncoderRegister(uint16_t address, uint16_t &data);
void readMultipleEncoderRegisters(uint16_t command, uint16_t data[]);
void writeToEncoderRegister(uint16_t command, uint16_t dataToWrite);
void SPISendReceive(uint16_t* sent_data, uint16_t size_of_sent_data, uint16_t* received_data, uint16_t size_of_received_data);
uint16_t readEncoderState();
int16_t getEncoderAngle();

#ifdef ENCODER_ESTIMATION
    double estimateEncoderSpeed();
#else
    double getEncoderSpeed();
#endif

//double getEncoderTemp();

#endif

