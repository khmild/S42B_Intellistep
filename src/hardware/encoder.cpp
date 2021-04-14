#include "encoder.h"

// SPI init structure
SPI_HandleTypeDef spiConfig;

// Main initialization structure
GPIO_InitTypeDef GPIO_InitStructure;

// A storage for the last angle sampled
double lastEncoderAngle = 0;

// A storage for the time of the last angle sampling
uint32_t lastAngleSampleTime = 0;

// Function to setup the encoder
void initEncoder() {

    // Setup pin A5, A6, and A7
    GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable the clock for the SPI bus
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Set the peripheral to be used
    spiConfig.Instance = SPI1;

    // Configure the settings for transactions
    spiConfig.Init.Direction = SPI_DIRECTION_2LINES;
    spiConfig.Init.Mode = SPI_MODE_MASTER;
    spiConfig.Init.DataSize = SPI_DATASIZE_8BIT;
    spiConfig.Init.CLKPolarity = SPI_POLARITY_LOW;
    spiConfig.Init.CLKPhase = SPI_PHASE_2EDGE;
    spiConfig.Init.NSS = SPI_NSS_SOFT;
    spiConfig.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spiConfig.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiConfig.Init.CRCPolynomial = 7;

    // Initialize the SPI bus with the parameters we set
    if (HAL_SPI_Init(&spiConfig) != HAL_OK) {
        Serial.println(F("SPI not initialized!"));
    }

    // Set the chip select pin high, disabling the encoder's communication
    pinMode(ENCODER_CS_PIN, OUTPUT);
    digitalWrite(ENCODER_CS_PIN, HIGH);

    // Reset the encoder's firmware
    writeToEncoderRegister(ENCODER_ACT_STATUS_REG, 0x401);
}


// Read the value of a register
uint16_t readEncoderRegister(uint16_t registerAddress) {

    // Pull CS low to select encoder
    digitalWrite(ENCODER_CS_PIN, LOW);

    // Add read bit to address
    registerAddress |= ENCODER_READ_COMMAND + 1;

    // Setup RX and TX buffers
    uint8_t rxbuf[2];
    uint8_t txbuf[2] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send 0xFFFF (like BTT code), this returns the wanted value
    txbuf[0] = 0xFF, txbuf[1] = 0xFF;
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWrite(ENCODER_CS_PIN, HIGH);

    // Return value as uint16
    return rxbuf[0] << 8 | rxbuf[1];
}


// Read multiple registers
// ! Flat out doesn't work
void readMultipleEncoderRegisters(uint16_t registerAddress, uint16_t* data, uint16_t dataLength) {

    // Pull CS low to select encoder
    digitalWriteFast(ENCODER_CS_PIN, LOW);

    // Setup TX and RX buffers
    registerAddress |= ENCODER_READ_COMMAND + dataLength;
    uint8_t txbuf[dataLength * 2] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };
    uint8_t rxbuf[dataLength * 2];

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send 0xFFFF (like BTT code), this returns the wanted value
    // Array length is doubled as we're using 8 bit values instead of 16
    for (uint8_t i = 0; i < dataLength * 2; i++) {
        txbuf[i] = 0xFF;
    }
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, dataLength * 2, 100);
    
    // Write the received data into the array
    for (uint8_t i = 0; i < dataLength; i++) {
        data[i] = rxbuf[i * 2] << 8 | rxbuf[i * 2 + 1];
    }

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWriteFast(ENCODER_CS_PIN, HIGH);
}


// Write a value to a register
// ! Untested
void writeToEncoderRegister(uint16_t registerAddress, uint16_t data) {

    // Pull CS low to select encoder
    digitalWriteFast(ENCODER_CS_PIN, LOW);

    // Setup TX and RX buffers
    registerAddress |= ENCODER_WRITE_COMMAND + 1;
    uint8_t txbuf[2] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };
    uint8_t rxbuf[2];

    // Send address we want to write, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 10);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send the data to be written
    txbuf[0] = uint8_t(data >> 8), txbuf[1] = uint8_t(data);
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 10);

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWriteFast(ENCODER_CS_PIN, HIGH);
}


// Reads the state of the encoder
uint16_t readEncoderState() {
    return (readEncoderRegister(ENCODER_STATUS_REG));
}


// Reads the value for the angle of the encoder (ranges from 0-360)
double getEncoderAngle() {

    // Get the value of the angle register
    uint16_t rawData = readEncoderRegister(ENCODER_ANGLE_REG);

    // Delete everything but the first 15 bits (others not needed)
    rawData = (rawData & (DELETE_BIT_15));

    // Return the value (equation from TLE5012 library)
    return (360.0 / POW_2_15) * ((double) rawData);
}

// For average velocity calculations instead of hardware readings from the TLE5012
#ifdef ENCODER_SPEED_ESTIMATION

// Reads the speed of the encoder (for later)
// ! Needs fixed yet, readings are off
double getEncoderSpeed() {

    // Get the newest angle
    double newAngle = getEncoderAngle();

    // Sample time
    uint32_t currentTime = millis();

    // Compute the average velocity
    double avgVelocity = (newAngle - lastEncoderAngle) / (currentTime - lastAngleSampleTime);

    // Correct the last angle and sample time
    lastEncoderAngle = newAngle;
    lastAngleSampleTime = currentTime;

    // Return the average velocity
    return avgVelocity;
}

#else // ENCODER_ESTIMATION

// Reads the speed of the encoder (for later)
double getEncoderSpeed() {
    // Prepare the variables to store data in
	uint16_t rawData[4];

    // Read the encoder, modifying the array
    readMultipleEncoderRegisters(ENCODER_SPEED_REG, rawData, sizeof(rawData) / sizeof(uint16_t));

	// Get raw speed reading
	int16_t rawSpeed = rawData[0];
	rawSpeed = rawSpeed & DELETE_BIT_15;

	// If bit 14 is set, the value is negative
	if (rawSpeed & CHECK_BIT_14) {
		rawSpeed = rawSpeed - CHANGE_UINT_TO_INT_15;
	}

	// Get FIR_MD from bits 15 and 16 of register 0x06
	uint16_t firMD = rawData[3] >> 14;

	// Determine sensor update rate from FIR_MD
	double firMDVal;
    switch (firMD) {
        case 0:
            firMDVal = 21.3;
            break;
        case 1:
            firMDVal = 42.7;
            break;
        case 2:
            firMDVal = 85.3;
            break;
        case 3:
            firMDVal = 170.6;
            break;
        default:
            firMDVal = 0.0;
            break;
    }

    // Calculate and return angle speed in degree per second
	return ((360.0 / POW_2_15) * rawSpeed) / (2.0 * firMDVal * 0.000001);
}

#endif // ! ENCODER_ESTIMATION

// Reads the temperature of the encoder
double getEncoderTemp() {

    // Create a variable to store the raw encoder data in
    uint16_t rawData = readEncoderRegister(ENCODER_TEMP_REG);

    // Delete everything but the first 7 bits
	rawData = (rawData & (DELETE_7BITS));

	// Check if the value received is positive or negative
	if (rawData & CHECK_BIT_9) {
		rawData = rawData - CHANGE_UNIT_TO_INT_9;
	}

    // Return the value (equation from TLE5012 library)
    int16_t rawTemp = rawData;
	return (rawTemp + TEMP_OFFSET) / (TEMP_DIV);
}

/*
// Runs through a bunch of motor movements to calibrate the encoder
void CalibrateEncoder(void) {
    // ! Also, need a mounting offset to make sure that the correct angles are read by motor.
    // Disable all interrupts so the calibration isn't interrupted
    __disable_irq();

    // Declare relevant variables
    int32_t encoderReading = 0;
    int32_t currentReading = 0;
    int32_t lastReading = 0;

    int32_t iStart = 0;
    int32_t jStart = 0;
    int32_t stepNo = 0;

    int32_t ticks = 0;
    uint16_t lookupAngle;
    int16_t x = 0;

    // Set that the motor should move in a positive direction
    positiveDir = true;

    // Energize the coils
    Output(0,80);

    // Flash LED on and off 4 times, waiting a quarter of a second between each change of the LED
    for(uint8_t m = 0; m < 4; m++) {
        led1 = LED_ON;
        delayMs(250);
        led1 = LED_OFF;
        delayMs(250);
    }

    // Loop through 200 motor movements (calibration should take around 2 seconds per direction)
    for(int16_t loopIndex = 0; loopIndex <= 199; loopIndex++) {

        // Zero the encoder's value
        encoderReading = 0;

        // ! Wait 20 ms (prevents loop from running too fast, not entirely sure about this)
        //delayMs(20);

        // Setup the last angle from the encoder
        lastReading = ReadAngle();

        // ! Loop through readings so they can stabilize (not entirely sure)
        for(uint8_t reading = 0; reading < 10; reading++) {

        // Read the encoder's angle
        currentReading = ReadAngle();

        // Make sure that the results are within the value limits of a 14 bit value
        if(currentReading - lastReading < -8192)
            currentReading += 16384;
        else if(currentReading - lastReading > 8192)
            currentReading -= 16384;

        encoderReading += currentReading;
        //delayMs(10);
        lastReading = currentReading;
        }

        // ! Divide the encoder value by 10 (not fully sure why)
        encoderReading = encoderReading / 10;

        // Constrain the encoder reading between 0 and 16384
        if(encoderReading > 16384) {
        encoderReading -= 16384;
        }
        else if(encoderReading < 0) {
        encoderReading += 16384;
        }

        // Save the value to the list of encoder reading
        fullStepReadings[loopIndex] = encoderReading;

        // Move the motor one step
        OneStep();

        // Delay so the motor can move
        //delayMs(10);
    }

    // Move in the negative direction
    positiveDir = false;

    // Give a little delay before the motor begins moving again
    delayMs(100);

    // Loop through 200 motor movements
    for(x = 199; x >= 0; x--) {

        // Zero the encoder reading
        encoderReading = 0;

        // ! Wait 20 ms (prevents loop from running too fast, not entirely sure about this)
        //delayMs(20);

        // Read the encoder's angle
        lastReading = ReadAngle();

        // ! Loop through readings so they can stabilize (not entirely sure)
        for(uint8_t reading = 0; reading < 10; reading++) {

        // Read the encoder's angle
        currentReading = ReadAngle();

        // Constrain value in the range of a 14-bit integer
        if(currentReading-lastReading < -8192) {
            currentReading += 16384;
        }
        else if(currentReading-lastReading > 8192) {
            currentReading -= 16384;
        }

        encoderReading += currentReading;
        //delayMs(10);
        lastReading = currentReading;
        }

        // ! Divide the encoder value by 10 (not fully sure why)
        encoderReading = encoderReading / 10;

        // Constrain the encoder's value between 0 and 16384
        if(encoderReading > 16384) {
        encoderReading -= 16384;
        }
        else if(encoderReading < 0) {
        encoderReading += 16384;
        }

        // Take the average of the encoder change. Basically (first reading + second reading)/2
        fullStepReadings[x]=(fullStepReadings[x]+encoderReading)/2;

        // Move the motor one step
        OneStep();

        // Allow the motor time to move
        //delayMs(10);
    }

    TIM_SetCompare1(TIM3,0);
    TIM_SetCompare2(TIM3,0);
    for(uint8_t i=0;i<200;i++)//
    {
        ticks=fullStepReadings[(i+1)%200]-fullStepReadings[i%200];
        if(ticks<-15000)
        ticks+=16384;
        else if(ticks>15000)
        ticks-=16384;
        for(int32_t j=0;j<ticks;j++)
        {
        stepNo=(fullStepReadings[i]+j)%16384;
        if(stepNo==0)
        {
            iStart=i;
            jStart=j;
        }
        }
    }
    FLASH_Unlock();
    flashErase32K();
    for(int32_t i=iStart;i<(iStart+200+1);i++)//
    {
        ticks=fullStepReadings[(i+1)%200]-fullStepReadings[i%200];
        if(ticks<-15000)
        ticks+=16384;
        if(i==iStart)
        {
        for(int32_t j=jStart;j<ticks;j++)
        {
            lookupAngle=(8192*i+8192*j/ticks)%1638400/100;
            flashWriteHalfWord(address,(uint16_t)lookupAngle);
            address+=2;
        }
        }
        else if(i==(iStart+200))
        {
        for(int32_t j=0;j<jStart;j++)
        {
            lookupAngle=((8192*i+8192*j/ticks)%1638400)/100;
            flashWriteHalfWord(address,(uint16_t)lookupAngle);
            address+=2;
        }
        }
        else
        {                        //this is the general case
        for(int32_t j=0;j<ticks;j++)
        {
            lookupAngle=((8192*i+8192*j/ticks)%1638400)/100;
            flashWriteHalfWord(address,(uint16_t)lookupAngle);
            address+=2;
        }
        }
    }
    if(Second_Calibrate_flag != 1) {
        // Create an array with the values that were obtained
        flashStoreFlag = true;
        table1[0] = 0xAACC;
        table1[1] = 128;
        table1[2] = 16;
        table1[3] = 4;
        table1[4] = 3;
        table1[5] = 0;
        table1[6] = 1;
        table1[7] = 1;
        table1[8] = 1;
        table1[11] = kp;
        table1[12] = ki;
        table1[13] = kd;

        // Write the array to flash
        flashWrite(DATA_STORE_ADDRESS, table1);
    }

    // Lock the flash against writing
    FLASH_Lock();

    CalibrateEncoder_finish_flag=1; //
}
*/