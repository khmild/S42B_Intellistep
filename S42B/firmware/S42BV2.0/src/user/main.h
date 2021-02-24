#ifndef __MAIN_H
#define __MAIN_H

#include "motor.h"
#include "Arduino.h"
#include "stdint.h"
#include "pinMapping.h"

#define CLOSED_LOOP
#define CLOSED_LOOP_CONFIG
//#define TEST_FLASH
#define STEPPER_UPDATE_FREQ 10

// Motor constants
#define STEP_ANGLE 1.8 // ! Check to see for .9 deg motors as well
#define MAX_CURRENT 3500 // Maximum current in mA
#define MAX_MICROSTEP_DIVISOR 32 // The maximum microstepping divisor

extern void Output(int32_t theta,uint8_t effort);
extern uint16_t ReadAngle(void);
extern void overclock(uint32_t PLLMultiplier);

extern StepperMotor motor;

#endif