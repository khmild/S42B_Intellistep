#ifndef __TIMERS_H__
#define __TIMERS_H__

// Imports
#include "Arduino.h"
#include "config.h"
#include "main.h"
#include "led.h"
#include "pid.h"
#include "serial.h"
#include "stm32f1xx_ll_tim.h"

// Variables
// Expose the StepperPID instance to other files
// (such as the flash for loading or saving parameters)
#ifdef ENABLE_PID
    extern StepperPID pid;
#endif

// Functions

// Tiny little function, just gets the time that the current program has been running
uint32_t sec();

// Sets up the motor timers and interrupts
void setupMotorTimers();

// Disables all motor timers (used when the motor cannot be messed with)
void disableMotorTimers();

// Enables the motor timers (used to reset the motor after the timers have been disabled)
void enableMotorTimers();

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

// Function for handling enable pin changes (when using interrupts)
void enablePinISR();

// Function that steps the motor
void stepMotor();

// Function that steps motor without adjusting the desired angle
void stepMotorNoDesiredAngle();

void generateSCurve(uint32_t accelTime, uint16_t sampleTime, uint16_t maxSpeed);

// Function to correct motor position if it is out of place
void correctMotor();

// Direct stepping
#ifdef ENABLE_DIRECT_STEPPING
// Schedule steps for the motor to execute (rate is in Hz)
#ifdef ENABLE_ACCELERATION
    void scheduleSteps(int64_t count, int32_t rate, uint16_t acceleration, STEP_DIR stepDir);
#else
    void scheduleSteps(int64_t count, int32_t rate, STEP_DIR stepDir);
#endif

#endif // ! ENABLE_DIRECT_STEPPING

#if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))
// Step schedule handler (runs when the interrupt is triggered)
void stepScheduleHandler();

// Convenience function to handle enabling the step schedule timer
void enableStepScheduleTimer();

// Convenience function to handle disabling the step schedule timer
void disableStepScheduleTimer();
#endif // ! ENABLE_DIRECT_STEPPING || ENABLE_PID

// Makes sure that all cached calls respect the current config
void syncInstructions();

#endif // ! __TIMERS_H__