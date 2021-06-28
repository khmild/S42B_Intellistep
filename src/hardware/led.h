#ifndef __LED_H__
#define __LED_H__

// Imports
#include "Arduino.h"
#include "config.h"

// Main LED object
#define LED_PIN_OBJ output(GPIOC_BASE, 13)

// Functions
void initLED();
void setLED(uint8_t state);

#endif
