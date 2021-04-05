#ifndef __TLE5012_H
#define __TLE5012_H

#include "Arduino.h"
#include "config.h"
#include "SPI.h"
#include "TLE5012-ino.hpp"

//extern SPIClass3W encoderSPI;
//extern Tle5012Ino encoder;

errorTypes initEncoder();
uint16_t readEncoderState();
double getEncoderAngle();
double getEncoderSpeed();
double getEncoderTemp();

#endif

