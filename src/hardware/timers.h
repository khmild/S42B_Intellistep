#ifndef __TIMERS_H__
#define __TIMERS_H__

// Imports
#include "Arduino.h"
#include "config.h"
#include "main.h"
#include "led.h"

// Functions

// Tiny little function, just gets the time that the current program has been running
uint32_t sec();

// Sets up the motor timers and interrupts
void setupMotorTimers();

// Pauses the timers, temporarily disabling them
void disableInterrupts();

// Re-enables interrupts, resuming them
void enableInterrupts();

// Enables step correction
void enableStepCorrection();

// Disables step correction
void disableStepCorrection();

// Updates the step correction frequency (called when microstepping is changed)
void updateCorrectionTimer();

// Function that steps the motor
void stepMotor();

// Function to correct motor position if it is out of place
void correctMotor();

#endif