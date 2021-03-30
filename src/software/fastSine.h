#ifndef __FAST_SINE_H__
#define __FAST_SINE_H__

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Main functions
int16_t fastSine(uint16_t angle);
int16_t fastCosine(uint16_t angle);

#endif // !__FAST_SINE_H__