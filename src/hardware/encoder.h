#ifndef __TLE5012_H
#define __TLE5012_H

// Libraries
#include "Arduino.h"
#include "config.h"

// Defines
#define SPI_TX_OFF   {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0x40000000;}
#define SPI_TX_ON    {GPIOA->CRL&=0x0FFFFFFF;GPIOA->CRL|=0xB0000000;}

// Register locations
#define ENCODER_READ_COMMAND   0x8000 // 8000
#define ENCODER_WRITE_COMMAND  0x5000 // 5000
#define ENCODER_STATUS_REGISTER (0x0000U) // Same as base
#define ENCODER_ANGLE_REGISTER  (0x0020U)
#define ENCODER_SPEED_REGISTER  (0x0030U)
#define ENCODER_TEMP_REGISTER   (0x0150U)

// Calculation constants
#define POW_2_16                    65536   // 2^16
#define DELETE_BIT_15               0x7FFF  // Used to delete everything except the first 15 bits
#define CHANGE_UINT_TO_INT_15       0x8000  // Used to change unsigned 16 bit integer into signed
#define CHECK_BIT_14                0x4000  // Used to check the 14th bit
#define TEMP_OFFSET                 152.0   // Used to offset the temp reading
#define TEMP_DIV                    2.776   // Used to divide the temperature
#define DELETE_7BITS                0x01FF  // Used to calculate 9 bit signed integer sent by the sensor
#define CHANGE_UNIT_TO_INT_9        0x0200  // Used to change an unsigned 9 bit integer into signed
#define CHECK_BIT_9                 0x0100  // Used to check the 9th bit

// Functions
void initEncoder();
uint16_t readEncoderRegister(uint16_t registerAddress);
void writeToEncoderRegister(uint16_t registerAddress, uint16_t data);
uint16_t readEncoderState();
double getEncoderAngle();
double getEncoderSpeed();
double getEncoderTemp();

#endif

