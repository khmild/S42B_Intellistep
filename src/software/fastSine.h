#ifndef __FAST_SINE_H__
#define __FAST_SINE_H__

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Public variables
//extern float sineValues[];

// Array direction enum
typedef enum {
    FRONT,
    BACK
} ARRAY_SCAN_DIR;

// Main functions
int16_t fastSin(uint16_t angle);
int16_t fastCos(uint16_t angle);

#endif // !__FAST_SINE_H__