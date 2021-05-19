#ifndef __KEY_H
#define __KEY_H

#include "main.h"

// Variable definitions
// Boolean for storing if the dip switches were installed the wrong way
extern bool dipInverted;

// Function definitions
void initButtons();
void checkButtons(bool updateScreen);
bool checkButtonState(PinName buttonPin);
void readDipMicrostepping();
void setDipInverted(bool inverted);
bool getDipInverted();

#endif


