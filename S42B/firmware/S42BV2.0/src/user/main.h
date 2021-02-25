#ifndef __MAIN_H
#define __MAIN_H

#include "motor.h"
#include "Arduino.h"
#include "stdint.h"
#include "config.h"

extern void Output(int32_t theta,uint8_t effort);
extern uint16_t ReadAngle(void);
extern void overclock(uint32_t PLLMultiplier);
extern void incrementMotor();

extern StepperMotor motor;

#endif