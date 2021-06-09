#include "flash.h"


// Check if the stepper is calibrated
bool isCalibrated() {

    // Unlock the flash for reading
    unlockFlash();

    // Get the calibration flag
    uint16_t calibrationFlag = flashReadHalfWord(CALIBRATION_ADDRESS);

    // Lock the flash again
    lockFlash();

    // Return if the unit is calibrated
    return ((calibrationFlag >> 8) == 0xAA);
}


// Check if the flash is busy
uint8_t flashGetStatus(void) {

    // Check the flash state register
    uint32_t state = FLASH -> SR;

    if (state & (1<<0)) {
        // Flash busy
        return 1;
    }
    else if (state & (1<<2)) {
        // Programming error
        return 2;
    }
    else if (state & (1<<4)) {
        // ! Fix, unknown
        return 3;
    }
    // No result found
    return 0;
}


// Function to delay execution until the flash is able to to be read again
bool flashWaitDone(uint16_t microseconds) {

    // Create an accumulator for the result
    uint8_t result;

    // Mark the start time
    uint32_t startTime = micros();

    // Loop forever, checking the value each time
    do {
        // Get the status of the flash
        result = flashGetStatus();

        // If the flash isn't busy or time is up, exit the loop
        if(result != 1 || (micros() - startTime) > microseconds) {
            return false;
        }

        // Delay for a microsecond
        delayMicroseconds(1);

    } while(true);

    // Return true if we made it through the time and flash state check
    return true;
}


// Erases a full page of flash
bool flashErasePage(uint32_t address) {

    // Flash reads and writes should NOT have any interrupts allowed as it could cause major problems
    disableInterrupts();

    // Wait until the flash is available,  if so, begin writing
    if(flashWaitDone(0X5FFF)) {
        FLASH -> CR |= 1 << 1;
        FLASH -> AR = address;
        FLASH -> CR |= 1 << 6;

        // If the writing operation succeeded 
        if(flashWaitDone(0X5FFF)) {
            FLASH -> CR &= ~(1 << 1);

            // All done, time to re-enable interrupts
            enableInterrupts();

            // Operation successful
            return true;
        }

        // All done, time to re-enable interrupts
        enableInterrupts();

        // Failed to actually write the data
        return false;
    }
    else {
        // All done, time to re-enable interrupts
        enableInterrupts();
        
        // Flash wasn't available, couldn't write at all
        return false;
    }
}


uint16_t flashReadHalfWord(uint32_t address) {
    return *(volatile uint16_t*) address;
}


void flashRead(uint32_t address, uint16_t *pBuffer, uint16_t arrayLength) {
    for(uint16_t i = 0; i < arrayLength; i++) {
        pBuffer[i] = flashReadHalfWord(address);
        address +=2;
    }
}

// ! Maybe add a return to see if the function completed successfully
bool flashWriteNoCheck(uint32_t address, uint16_t *pBuffer, uint16_t arrayLength) {

    // Result accumulator, so the boolean doesn't have to be redeclared with each loop
    bool flashOpSuccess;

    // Loop through the buffer, writing each of the values
    for(uint16_t i = 0; i < arrayLength; i++) {

        // Perform the write operation
        flashOpSuccess = flashProgramHalfWord(address, pBuffer[i]);

        // Check if it failed. If so, return a fail
        if (!flashOpSuccess) {
            return false;
        }

        // Increment the address that we're writing to by 2
        address += 2;
    }

    // If we made it this far, all the of the writes were successful
    return true;
}

// Function from the stm32f10x_flash library
/**
  * @brief  Programs a half word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: true for operation success, false for fail 
  */
bool flashProgramHalfWord(uint32_t Address, uint16_t Data) {

    // Flash reads and writes should NOT have any interrupts allowed as it could cause major problems
    disableInterrupts();

    // Declare an accumulator
    bool flashAvailable = false;

    /* Wait for last operation to be completed */
    flashAvailable = flashWaitDone(FLASH_PROGRAM_TIMEOUT);
    
    if(flashAvailable) {
        /* if the previous operation is completed, proceed to program the new data */
        FLASH->CR |= CR_PG_Set;
    
        *(__IO uint16_t*)Address = Data;

        /* Wait for last operation to be completed */
        flashAvailable = flashWaitDone(FLASH_PROGRAM_TIMEOUT);
        
        /* Disable the PG Bit */
        FLASH->CR &= CR_PG_Reset;
    }

    // Re-enable the interrupts, the flash write is completed
    enableInterrupts();
    
    /* Return the Program Status */
    return flashAvailable;
}

#define STM32_FLASH_SIZE STM32_memory_size
#if STM32_FLASH_SIZE < 256
    #define STM_SECTOR_SIZE 1024
#else
    #define STM_SECTOR_SIZE	2048
#endif

uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2];


void flashWrite(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t arrayLength) {

    // Flash reads and writes should NOT have any interrupts allowed as it could cause major problems
    disableInterrupts();

    uint32_t secpos;	   // Sector position
    uint16_t secoff;	   // Sector offset
    uint16_t secremain;    //
    uint16_t i;
    uint32_t offaddr;      // Offset address

    if(WriteAddr < FLASH_BASE||(WriteAddr>=(FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//
    unlockFlash();
    offaddr = WriteAddr - FLASH_BASE;
    secpos = offaddr / STM_SECTOR_SIZE; //  0~63 for STM32F030
    secoff=(offaddr%STM_SECTOR_SIZE)/2;
    secremain = STM_SECTOR_SIZE / 2 - secoff;
    if(arrayLength<=secremain)secremain=arrayLength;
    while(1) {
        flashRead(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
        for(i=0;i<secremain;i++)
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;
        }
        if(i<secremain)
        {
            flashErasePage(secpos*STM_SECTOR_SIZE+FLASH_BASE);
            for(i=0;i<secremain;i++)
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            flashWriteNoCheck(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
        } else flashWriteNoCheck(WriteAddr, pBuffer,secremain);
        if(arrayLength==secremain)break;
        else
        {
            secpos++;
            secoff=0;
                 pBuffer+=secremain;
            WriteAddr+=secremain*2;
                 arrayLength-=secremain;
            if(arrayLength>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;
            else secremain=arrayLength;
        }
    };
    lockFlash();

    // Re-enable the interrupts
    enableInterrupts();
}


// Erase all of the data stored
void flashErase32K(void) {
    //for (uint8_t i = 0, i <= )
    flashErasePage(CALIBRATION_ADDRESS);
    flashErasePage(0x08018000); // CALIBRATION_ADDRESS + 1024
    flashErasePage(0x08018400);
    flashErasePage(0x08018800);
    flashErasePage(0x08018C00);
    flashErasePage(0x08019000);
    flashErasePage(0x08019400);
    flashErasePage(0x08019800);
    flashErasePage(0x08019C00);
    flashErasePage(0x0801A000);
    flashErasePage(0x0801A400);
    flashErasePage(0x0801A800);
    flashErasePage(0x0801AC00);
    flashErasePage(0x0801B000);
    flashErasePage(0x0801B400);
    flashErasePage(0x0801B800);
    flashErasePage(0x0801BC00);
    flashErasePage(0x0801C000);
    flashErasePage(0x0801C400);
    flashErasePage(0x0801C800);
    flashErasePage(0x0801CC00);
    flashErasePage(0x0801D000);
    flashErasePage(0x0801D400);
    flashErasePage(0x0801D800);
    flashErasePage(0x0801DC00);
    flashErasePage(0x0801E000);
    flashErasePage(0x0801E400);
    flashErasePage(0x0801E800);
    flashErasePage(0x0801EC00);
    flashErasePage(0x0801F000);
    flashErasePage(0x0801F400);
    flashErasePage(0x0801F800);
    flashErasePage(0x0801FC00);
}


// Function to allow flash to be tampered with
void unlockFlash() {

    // Directly from the STM32 flash library
    FLASH -> KEYR = FLASH_KEY1;
    FLASH -> KEYR = FLASH_KEY2;
}


// Function to lock flash, preventing it from being tampered with accidentally
void lockFlash() {

    // Directly from the STM32 flash library
    FLASH -> CR |= CR_LOCK_SET;

}


// Loads the saved parameters from flash and sets them
void loadSavedValues() {

    // Setup the table for parameters
    // Parameters are in form = (current, step angle, microstepping divisor, motor reversed, motor enable inversion, PID P value, PID I value, PID D value, CAN ID, dip switches inverted)
    uint16_t savedParameters[11];

    // Load the table from memory
    flashRead(CALIBRATION_ADDRESS, savedParameters, sizeof(savedParameters) / sizeof(savedParameters[0]));

    // Set the motor current
    motor.setRMSCurrent(savedParameters[0]);

    // Motor stepping angle
    motor.setFullStepAngle(savedParameters[1]);

    // Microstepping divisor
    motor.setMicrostepping(savedParameters[2]);

    // Motor Direction Reversed
    motor.setReversed(savedParameters[3]);

    // Motor Enable Inverted
    motor.setEnableInversion(savedParameters[4]);

    // Motor microstepping multiplier
    motor.setMicrostepMultiplier(savedParameters[5]);

    // P term of PID
    motor.setPValue(savedParameters[6]);

    // I term of PID
    motor.setIValue(savedParameters[7]);

    // D term of PID
    motor.setDValue(savedParameters[8]);

    // The CAN ID of the motor
    setCANID(AXIS_CAN_ID(savedParameters[9]));

    // If the dip switches were installed incorrectly
    setDipInverted(savedParameters[10]);
}


// Writes the currently saved parameters to flash memory for long term storage
void saveParametersToFlash() {

    // Setup the table for parameters
    // Parameters are in form = (current, step angle, microstepping divisor, motor reversed, motor enable inversion, PID P value, PID I value, PID D value, CAN ID, dip switches inverted)
    uint16_t savedParameters[11];

    // Get the motor current
    #ifndef ENABLE_DYNAMIC_CURRENT
        savedParameters[0] = motor.getRMSCurrent();
    #endif

    // Motor stepping angle
    savedParameters[1] = motor.getFullStepAngle();

    // Microstepping divisor
    savedParameters[2] = motor.getMicrostepping();

    // Motor Direction Reversed
    savedParameters[3] = motor.getReversed();

    // Motor Enable Inverted
    savedParameters[4] = motor.getEnableInversion();

    // Microstep multiplier
    savedParameters[5] = motor.getMicrostepMultiplier();

    // P term of PID
    savedParameters[6] = motor.getPValue();

    // I term of PID
    savedParameters[7] = motor.getIValue();

    // D term of PID
    savedParameters[8] = motor.getDValue();

    // CAN ID of the motor controller
    #ifdef ENABLE_CAN
        savedParameters[9] = getCANID();
    #endif

    // If the dip switches were installed incorrectly
    savedParameters[10] = getDipInverted();

    // Save the array to flash
    flashWrite(CALIBRATION_ADDRESS, savedParameters, sizeof(savedParameters) / sizeof(savedParameters[0]));
}