#ifndef __FAST_SINE_H__
#define __FAST_SINE_H__

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Public variables
extern const int16_t sineTable[SINE_VAL_COUNT];

// Main functions
/*
int16_t fastSin(uint16_t angle);
int16_t fastCos(uint16_t angle);
*/
#define fastSin(angle) (sineTable[angle])
#define fastCos(angle) (sineTable[(angle + (SINE_VAL_COUNT / 4)) & (SINE_VAL_COUNT - 1)])

#endif // !__FAST_SINE_H__