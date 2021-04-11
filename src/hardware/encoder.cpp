#include "encoder.h"

// SPI init structure
SPI_HandleTypeDef spiConfig;

// Main initialization structure
GPIO_InitTypeDef GPIO_InitStructure;

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
}


// Read the value of a register
uint16_t readEncoderRegister(uint16_t registerAddress) {

    // Pull CS low to select encoder
    digitalWrite(ENCODER_CS_PIN, LOW);

    // Setup TX and RX buffers
    registerAddress |= ENCODER_READ_COMMAND;
    uint8_t txbuf[] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };
    uint8_t rxbuf[] = { 0x00, 0x00 };

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send 0xFFFF (like BTT code), this returns the wanted value
    txbuf[0] = 0xFF, txbuf[1] = 0xFF;
    rxbuf[0] = 0x00, rxbuf[1] = 0x00;
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWrite(ENCODER_CS_PIN, HIGH);

    // Return angle value as uint16
    return rxbuf[0] << 8 | rxbuf[1];
}


// Write a value to a register
// ! Untested
void writeToEncoderRegister(uint16_t registerAddress, uint16_t data) {

    // Pull CS low to select encoder
    digitalWrite(ENCODER_CS_PIN, LOW);

    // Setup TX and RX buffers
    registerAddress |= ENCODER_WRITE_COMMAND;
    uint8_t txbuf[] = { uint8_t(registerAddress >> 8), uint8_t(registerAddress) };
    uint8_t rxbuf[] = { 0x00, 0x00 };

    // Send address we want to read, response seems to be equal to request
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 4, 100);

    // Set the MOSI pin to open drain
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Send the data to be written
    txbuf[0] = uint8_t(data >> 8), txbuf[1] = uint8_t(data);
    rxbuf[0] = 0x00, rxbuf[1] = 0x00;
    HAL_SPI_TransmitReceive(&spiConfig, txbuf, rxbuf, 2, 100);

    // Set MOSI back to Push/Pull
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Deselect encoder
    digitalWrite(ENCODER_CS_PIN, HIGH);
}


// Reads the state of the encoder
uint16_t readEncoderState() {
    return (readEncoderRegister(ENCODER_STATUS_REGISTER));
}


// Reads the value for the angle of the encoder
double getEncoderAngle() {

    // Get the value of the angle register
    uint16_t rawData = readEncoderRegister(ENCODER_ANGLE_REGISTER);

    // Delete everything but the first 15 bits (others not needed)
    rawData = (rawData & (DELETE_BIT_15));

    // Check if the value received is positive or negative
    if (rawData & CHECK_BIT_14) {
        rawData = rawData - CHANGE_UINT_TO_INT_15;
    }

    // Return the value (equation from TLE5012 library)
    return (360.0 / POW_2_16) * ((double) rawData);
}


// Reads the speed of the encoder (for later)
double getEncoderSpeed() {
    return 0;
    /*
    int8_t numOfData = 0x5;
	uint16_t rawData[numOfData] = {};

	errorTypes status = readMoreRegisters(reg.REG_ASPD + numOfData, rawData, upd, safe);
	if (status != NO_ERROR)
	{
		return (status);
	}

	// Prepare raw speed
	rawSpeed = rawData[0];
	rawSpeed = (rawSpeed & (DELETE_BIT_15));
	// check if the value received is positive or negative
	if (rawSpeed & CHECK_BIT_14)
	{
		rawSpeed = rawSpeed - CHANGE_UINT_TO_INT_15;
	}

	// Prepare firMDVal
	uint16_t firMDVal = rawData[3];
	firMDVal >>= 14;

	// Prepare intMode2Prediction
	uint16_t intMode2Prediction = rawData[5];
	if (intMode2Prediction & 0x0004)
	{
		intMode2Prediction = 3;
	}else{
		intMode2Prediction = 2;
	}

	// Prepare angle range
	uint16_t rawAngleRange = rawData[5];
	rawAngleRange &= GET_BIT_14_4;
	rawAngleRange >>= 4;
	double angleRange = 360 * (POW_2_7 / (double) (rawAngleRange));

	//checks the value of fir_MD according to which the value in the calculation of the speed will be determined
	//according to if prediction is enabled then, the formula for speed changes
	finalAngleSpeed = calculateAngleSpeed(angleRange, rawSpeed, firMDVal, intMode2Prediction);
	return (status);
    */
}


// Reads the temperature of the encoder
double getEncoderTemp() {

    // Create a variable to store the raw encoder data in
    uint16_t rawData = readEncoderRegister(readEncoderRegister(ENCODER_TEMP_REGISTER));

    // Delete everything but the first 7 bits
	rawData = (rawData & (DELETE_7BITS));

	// Check if the value received is positive or negative
	if (rawData & CHECK_BIT_9) {
		rawData = rawData - CHANGE_UNIT_TO_INT_9;
	}

    // Return the value (equation from TLE5012 library)
	return (rawData + TEMP_OFFSET) / (TEMP_DIV);
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