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
//#define PHASE_MULTIPLIER 12.5f // ! Tune this
#define STEP_ANGLE 0.9 // ! Check to see for .9 deg motors as well

/* SPI command for TLE5012 */
#define CMD_READ_STATUS				0x8001			//8000
#define CMD_READ_ANGLE_VALUE		0x8021			//8020
#define CMD_READ_SPEED_VALUE		0x8031			//8030

#define WRITE_MOD1_VALUE		0x5060			//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001

#define WRITE_MOD2_VALUE		0x5080			//0_1010_0_001000_0001
#define MOD2_VALUE	0x0800

#define WRITE_MOD3_VALUE		0x5091			//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000

#define WRITE_MOD4_VALUE		0x50E0			//0_1010_0_001110_0001
#define MOD4_VALUE	0x0098						//9bit 512

#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 0x000D

extern void Output(int32_t theta,uint8_t effort);
extern uint16_t ReadAngle(void);
extern void overclock(uint32_t PLLMultiplier);

extern StepperMotor motor;

#endif



