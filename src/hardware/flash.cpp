#include "flash.h"


// Check if the stepper is calibrated
bool isCalibrated() {

    // Disable interrupts (CANNOT BE INTERRUPTED)
    disableInterrupts();

    // Unlock the flash for reading
    HAL_FLASH_Unlock();

    // Get the calibration flag
    bool calibrationFlag = readFlash<bool>(CALIBRATED_INDEX);

    // Lock the flash again
    HAL_FLASH_Lock();

    // Flash is locked again, no need to worry
    enableInterrupts();

    // Return if the unit is calibrated
    return calibrationFlag;
}


// Raw read function. Reads raw bits into a set type
template<typename type>
type readFlash(uint32_t parameterIndex) {

    // Wait until the flash is free (continue if not done within 100ms)
    HAL_StatusTypeDef status = FLASH_WaitForLastOperation(100);

    // Check if the flash is free
    if (status == HAL_OK) {

        // Calculate the correct address (jump 4 because address is in bytes, byte is 8 bits, intervals are 32 bits)
        uint32_t address = CALIB_START_ADDR + (parameterIndex * 4);

        // Pull the data from said address
        return *(__IO type *)address;
    }
    else {
        // Flash timed out, just return 0
        return (type)0;
    }
}


// Raw write function. Writes out 16 bits to the flash
void writeToAddress(uint32_t address, uint16_t data) {

    // Disable interrupts (CANNOT BE INTERRUPTED)
    disableInterrupts();

    // Set the flash for writing
    HAL_FLASH_Unlock();

    // Wait for the flash to be ready
    HAL_StatusTypeDef status = FLASH_WaitForLastOperation(10);

    // Only continue if the flash is ready
    if (status == HAL_OK) {

        // Set the flash programming register
        SET_BIT(FLASH->CR, FLASH_CR_PG);

        // Check that the value isn't already stored (reduces wear on the flash)
        //if (readFlash<type>(parameterIndex) != data) {

            // Write the data
            *(__IO uint16_t*)address = data;
        //}

        // Wait for the operation to finish
        FLASH_WaitForLastOperation(10);

        // Clear the flash programming register
        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    }

    // Lock the flash (we finished writing)
    HAL_FLASH_Lock();

    // Re-enable interrupts
    enableInterrupts();
}

// Built on the address function, writes to a parameter index instead
void writeFlash(uint32_t parameterIndex, uint16_t data) {

    // Address is the start address plus 4 bytes for every parameter after
    writeToAddress(CALIB_START_ADDR + (4 * parameterIndex), data);
}

// Built on the address function, writes unsigned 32 bit ints instead
void writeFlash(uint32_t parameterIndex, uint32_t data) {

    // Write each part, but in separate parts. Second call moves up two bytes instead of one
    writeToAddress(CALIB_START_ADDR + (4 * parameterIndex),     (uint16_t)(data >> 16U));
    writeToAddress(CALIB_START_ADDR + (4 * parameterIndex) + 2, (uint16_t)(data >> 16U));
}

// Built on the address function, writes 64 bits to the flash


// Loads the saved parameters from flash and sets them
void loadParameters() {

    // Disable interrupts
    disableInterrupts();

    // Check to see if the data is valid
    if (readFlash<bool>(VALID_FLASH_CONTENTS)) {

        // Set the motor current
        #ifdef ENABLE_DYNAMIC_CURRENT
            motor.setDynamicAccelCurrent(readFlash<uint16_t>(DYNAMIC_ACCEL_CURRENT_INDEX));
            motor.setDynamicIdleCurrent(readFlash<uint16_t>(DYNAMIC_IDLE_CURRENT_INDEX));
            motor.setDynamicMaxCurrent(readFlash<uint16_t>(DYNAMIC_MAX_CURRENT_INDEX));
        #else
            // Just load the motor current from the idle index
            motor.setRMSCurrent(readFlash<uint16_t>(CURRENT_INDEX_1));
        #endif

        // Motor stepping angle
        motor.setFullStepAngle(readFlash<float>(FULL_STEP_ANGLE_INDEX));

        // Microstepping divisor
        motor.setMicrostepping(readFlash<uint16_t>(MICROSTEPPING_INDEX));

        // Motor Direction Reversed
        motor.setReversed(readFlash<bool>(MOTOR_REVERSED_INDEX));

        // Motor Enable Inverted
        motor.setEnableInversion(readFlash<bool>(ENABLE_INVERSION_INDEX));

        // Motor microstepping multiplier
        motor.setMicrostepMultiplier(readFlash<float>(MICROSTEP_MULTIPLIER_INDEX));

        // P term of PID
        motor.setPValue(readFlash<float>(P_TERM_INDEX));

        // I term of PID
        motor.setIValue(readFlash<float>(I_TERM_INDEX));

        // D term of PID
        motor.setDValue(readFlash<float>(D_TERM_INDEX));

        // The CAN ID of the motor
        setCANID(readFlash<AXIS_CAN_ID>(CAN_ID_INDEX));

        // If the dip switches were installed incorrectly
        setDipInverted(readFlash<bool>(INVERTED_DIPS_INDEX));
    }

    // All done, we can re-enable interrupts
    enableInterrupts();
}


// Writes the currently saved parameters to flash memory for long term storage
void saveParameters() {

    // Disable interrupts
    disableInterrupts();

    // Write that the data is valid
    // ! writeFlash<bool>(VALID_FLASH_CONTENTS, true);

    // No programming needed for the calibration state, that it written once calibrated

    // Get the motor current
    #ifdef ENABLE_DYNAMIC_CURRENT
        writeFlash(DYNAMIC_ACCEL_CURRENT_INDEX, motor.getDynamicAccelCurrent());
        writeFlash(DYNAMIC_IDLE_CURRENT_INDEX, motor.getDynamicIdleCurrent());
        writeFlash(DYNAMIC_MAX_CURRENT_INDEX, motor.getDynamicMaxCurrent());
    #else
        // Save the static motor current in the idle and max slot of the dynamic current setting, zeroing the acceleration
        writeFlash(CURRENT_INDEX_0, (uint16_t)0);
        writeFlash(CURRENT_INDEX_1, motor.getRMSCurrent());
        writeFlash(CURRENT_INDEX_2, motor.getRMSCurrent());
    #endif

    // Motor stepping angle
    // ! writeFlash<float>(FULL_STEP_ANGLE_INDEX, motor.getFullStepAngle());

    // Microstepping divisor
    writeFlash(MICROSTEPPING_INDEX, motor.getMicrostepping());

    // Motor Direction Reversed
    // ! writeFlash<bool>(MOTOR_REVERSED_INDEX, motor.getReversed());

    // Motor Enable Inverted
    // ! writeFlash<bool>(ENABLE_INVERSION_INDEX, motor.getEnableInversion());

    // Microstep multiplier
    // ! writeFlash<float>(MICROSTEP_MULTIPLIER_INDEX, motor.getMicrostepMultiplier());

    // P term of PID
    // ! writeFlash<float>(P_TERM_INDEX, motor.getPValue());

    // I term of PID
    // ! writeFlash<float>(I_TERM_INDEX, motor.getIValue());

    // D term of PID
    // ! writeFlash<float>(D_TERM_INDEX, motor.getDValue());

    // CAN ID of the motor controller
    #ifdef ENABLE_CAN
        writeFlash(CAN_ID_INDEX, (uint32_t)getCANID());
    #else
        writeFlash(CAN_ID_INDEX, (uint32_t)0);
    #endif

    // If the dip switches were installed incorrectly
    // ! writeFlash<bool>(INVERTED_DIPS_INDEX, getDipInverted());

    // Re-enable interrupts
    enableInterrupts();
}


// Wipes all parameters stored, returning to programming defaults
// !!! WARNING !!! Reboots processor!
void wipeParameters() {

    // Simpler method of just setting the contents to be invalid, rather than erasing everything
    // ! writeFlash<bool>(VALID_FLASH_CONTENTS, false);

    // Loop through all of the addresses, zeroing all of the bits
    //for (uint8_t index = 0; index < MAX_FLASH_PARAM_INDEX; index++) {
    //    writeFlash<uint32_t>(index, 0);
    //}

    // Reboot the processor
    NVIC_SystemReset();
}
