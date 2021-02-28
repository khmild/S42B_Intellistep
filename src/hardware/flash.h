#ifndef __FLASH_H
#define __FLASH_H

#include "main.h"
#include "stm32f103xb.h"
#include "buttons.h"
#include "canMessaging.h"

// Where the parameters are saved
#define CALIBRATION_ADDRESS      0x08017C00

// Parameter for locking the flash
#define CR_LOCK_SET              ((uint32_t)0x00000080)

// Flash control register bits
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE)

// Flash timeout period
#define FLASH_PROGRAM_TIMEOUT    ((uint32_t)0x00002000)

// Where the data samples are stored
#define SAMPLING_DATA_ADDR      0x08018000
#define STM32_memory_size       128 // Kb

bool isCalibrated();
uint16_t flashReadHalfWord(uint32_t address);
uint8_t flashWriteHalfWord(uint32_t address, uint16_t data);
void flashRead(uint32_t address, uint16_t *pBuffer);
void flashWriteNoCheck(uint32_t address, uint16_t *pBuffer);
bool FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
void flashWrite(uint32_t address, uint16_t *pBuffer);
void flashErase32K();

void unlockFlash();
void lockFlash();

void loadSavedValues();
void saveParametersToFlash();

#endif


