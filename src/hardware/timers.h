#ifndef __TIMERS_H__
#define __TIMERS_H__

// Imports
#include "Arduino.h"
#include "config.h"
#include "main.h"

// Functions
void setupMotorTimers();
void disableInterrupts();
void enableInterrupts();
void stepMotor();
void updateMotor();

#endif