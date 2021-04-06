#ifndef __MAIN_H__
#define __MAIN_H__

#include "motor.h"
#include "Arduino.h"
#include "stdint.h"
#include "config.h"

//extern void Output(int32_t theta,uint8_t effort);
//extern uint16_t ReadAngle(void);
extern void overclock(uint32_t PLLMultiplier);

// Motor interrupts
extern void setupMotorTimers();
extern void updateMotor();

extern StepperMotor motor;

extern void setup();
extern void loop();

extern void blink();

#endif