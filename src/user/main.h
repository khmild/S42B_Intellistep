#ifndef __MAIN_H__
#define __MAIN_H__

#include "Arduino.h"
#include "stdint.h"
#include "config.h"
#include "motor.h"

//extern void Output(int32_t theta,uint8_t effort);
//extern uint16_t ReadAngle(void);
extern void overclock(uint32_t PLLMultiplier);

extern StepperMotor motor;

extern void setup();
extern void loop();

extern void blink();

#endif