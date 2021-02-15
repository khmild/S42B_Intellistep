// Make sure that the motor header has only been used once
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

float getMotorRPM();
float getPIDError();
int getMicrosteps();
void setMicrosteps(int setMicrostepping);


#endif