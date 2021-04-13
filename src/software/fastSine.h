#ifndef __FAST_SINE_H__
#define __FAST_SINE_H__

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Public variables
extern float sineValues[];

// Main functions
float fastSin(uint16_t angle);
float fastCos(uint16_t angle);

#endif // !__FAST_SINE_H__