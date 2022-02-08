#ifndef __KEY_H
#define __KEY_H

#include "main.h"

// Need the timer library
#include "timers.h"

// Need the OLED library (for getting the menu level)
#include "oled.h"

// Function definitions
void initButtons();
void checkButtons();
bool checkButtonState(PinName buttonPin);
void readDipMicrostepping();
void checkDips();
void setDipInverted(bool inverted);
bool getDipInverted();

#endif


