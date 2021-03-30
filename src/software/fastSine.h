#ifndef FAST_SINE_H
#define FAST_SINE_H

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Main functions
int16_t fastSine(uint16_t angle);
int16_t fastCosine(uint16_t angle);

#endif // !FAST_SINE_H