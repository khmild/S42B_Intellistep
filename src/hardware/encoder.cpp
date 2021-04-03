#include "encoder.h"
//#include "SSD1306.h"

// Main SPI interface for the encoder
//SPIClass3W encoderSPI = SPIClass3W();
//Tle5012b encoder = Tle5012Ino(ENCODER_SS);

SPI_HandleTypeDef SPI_HandleStructure;
GPIO_InitTypeDef GPIO_InitStructure;

// The command to send to the encoder
uint16_t _command[2];

// Angle estimation
#ifdef ENCODER_ESTIMATION
    double lastAngleReading = 0;
    uint32_t lastAngleReadingTime = 0;
#endif

// Function to setup the encoder
void initEncoder() {

    // Declare the SPI interface
    //encoderSPI.begin(ENCODER_MOSI, ENCODER_MISO, ENCODER_SCK, ENCODER_SS);
    //encoder.begin();

    // Enable the peripheral clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Disable the JTAG interface in favor of a SW interface
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    // Set up the SPI pins
    GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Set the GPIO bits
    GPIOA->BSRR = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;

    // Set up the SPI structure
    SPI_HandleStructure.Instance = SPI1;
    SPI_HandleStructure.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_HandleStructure.Init.Mode = SPI_MODE_MASTER;
	SPI_HandleStructure.Init.DataSize = SPI_DATASIZE_16BIT;
	SPI_HandleStructure.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_HandleStructure.Init.CLKPhase = SPI_PHASE_2EDGE;
	SPI_HandleStructure.Init.NSS = SPI_NSS_SOFT;
	SPI_HandleStructure.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Processor runs at 128MHz, so a divisor of 16 takes it to the spec of 8MHz
	SPI_HandleStructure.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_HandleStructure.Init.CRCPolynomial = 7;
	HAL_SPI_Init(&SPI_HandleStructure);

    // Enable the SPI bus
    SPI1->CR1 |= SPI_CR1_SPE;

    // Disable the chip select pin
    ENCODER_CS = 1;

    // Reset the firmware on the encoder
    writeToEncoderRegister(ENCODER_ACTIVATION_REGISTER, 0x401);

    // Write the device's slave number
    writeToEncoderRegister(ENCODER_WRITE_REGISTER_BASE, 0);
}


// Checks the status of a SPI flag
FlagStatus getSPIFlagStatus(SPI_TypeDef* SPIx, uint16_t SPI_I2S_FLAG) {
  FlagStatus bitstatus = RESET;

  /* Check the status of the specified SPI/I2S flag */
  if ((SPIx->SR & SPI_I2S_FLAG) != (uint16_t)RESET)
  {
    /* SPI_I2S_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_I2S_FLAG status */
  return bitstatus;
}


// Reads a specified register of the encoder
void readSingleEncoderRegister(uint16_t address, uint16_t &data) {

    _command[0] = ENCODER_READ_REGISTER_BASE | address | 0x0000 | 0x0000;
	uint16_t _received[MAX_REGISTER_MEM] = {0};
	SPISendReceive(_command, 1, _received, 2);
	data = _received[0];


    /*
    // Create an accumulator for the data collected
    uint16_t data;

    // Enable the chip select pin
    ENCODER_CS = 0;
    //digitalWrite(ENCODER_SS, LOW);

    // Wait for the chip to be ready
    while(getSPIFlagStatus(SPI1, SPI_FLAG_TXE) != SET);
   
    // Send the register address to be read
    SPI1->DR = address;

    // Wait for the chip to return the data
    while(getSPIFlagStatus(SPI1, SPI_FLAG_RXNE) != SET);

    // Read the data from the buffer
    data = SPI1->DR;

    // Disable the SPI tx
    SPI_TX_OFF;

    // Wait for the response
    while(getSPIFlagStatus(SPI1, SPI_FLAG_TXE) != SET);

    // Send a second message
    SPI1->DR = 0xFFFF;

    // Wait for the response
    while(getSPIFlagStatus(SPI1, SPI_FLAG_RXNE) != SET);

    // Read the data from the buffer
    data = SPI1->DR & 0x7FFF;

    // Disable the chip select
    ENCODER_CS = 1;
    //digitalWrite(ENCODER_SS, HIGH);

    // Turn SPI transactions back on
    SPI_TX_ON;

    // Return the data that was read
    return data;
    */

}


// Read multiple encoder registers
void readMultipleEncoderRegisters(uint16_t command, uint16_t data[]) {

	_command[0] = ENCODER_READ_REGISTER_BASE | command | 0x0000 | 0x0000;
	uint16_t _received[MAX_REGISTER_MEM] = {0};
	uint16_t _recDataLength = (_command[0] & (0x000F)); // Number of registers to read
	SPISendReceive(_command, 1, _received, _recDataLength);
	memcpy(data, _received, (_recDataLength)* sizeof(uint16_t));
}


// Write data to a specified register
void writeToEncoderRegister(uint16_t command, uint16_t dataToWrite) {
	uint16_t safety = 0;
	_command[0] = ENCODER_WRITE_REGISTER_BASE | command;
	_command[1] = dataToWrite;
	SPISendReceive(_command, 2, &safety, 1);
}


// Send / receive a set number of SPI data packets
void SPISendReceive(uint16_t* sent_data, uint16_t size_of_sent_data, uint16_t* received_data, uint16_t size_of_received_data) {

    // Enable the chip select pin
    ENCODER_CS = 0;

    // Define the counter used to index the sent and received data
    uint32_t data_index = 0;

    // Transmit each of the data, one by one
    for(data_index = 0; data_index < size_of_sent_data; data_index++) { 

        // Wait for the chip to be ready
        while(getSPIFlagStatus(SPI1, SPI_FLAG_TXE) != SET);

        // Send the specified data
        SPI1->DR = sent_data[data_index];
	}

	// Wait for 5 microseconds so the chip knows to send data back
	//delayMicroseconds(5);

    // Loop through, reading the data from the buffer
	for(data_index = 0; data_index < size_of_received_data; data_index++) {
        
        // Wait for the receieve register to populate
        while(getSPIFlagStatus(SPI1, SPI_FLAG_RXNE) != SET);

        // Read the data from the buffer
        received_data[data_index] = SPI1->DR;
	}

    // Disable the chip again
	ENCODER_CS = 1;
}



// Reads the state of the encoder
uint16_t readEncoderState() {

    // Create a variable where the state will be stored
    uint16_t state = 0;

    // Read the value of the encoder
    readSingleEncoderRegister(ENCODER_STATUS_REGISTER, state);

    // Return the state that was received
    return state;
}


// Reads the value for the angle of the encoder
int16_t getEncoderAngle() {

    // Create an accumulator for the read data
	uint16_t rawData = 0;

    // Read the register
	readSingleEncoderRegister(ENCODER_ANGLE_REGISTER, rawData);

    // Delete everything after the first 15 bits
	rawData = (rawData & (DELETE_BIT_15));

	// Check if the value received is positive or negative
	if (rawData & CHECK_BIT_14)
	{
		rawData = rawData - CHANGE_UINT_TO_INT_15;
	}

    // Convert the angle value to a regular degree value (equation from TLE5012 datasheet)
	return (360 / POW_2_15) * ((double) rawData);
}

#ifdef ENCODER_ESTIMATION

    // Function for estimating the speed of the encoder (takes much less flash memory)
    double estimateEncoderSpeed() {

        // Calculate the speed
        double angularSpeed = (getEncoderAngle() - lastAngleReading) / (millis() - lastAngleReadingTime);

        // Update the last variables for the next estimation
        lastAngleReading = getEncoderAngle();
        lastAngleReadingTime = millis();

        // Return the RPMs
        return (angularSpeed / 360);
    }

#else

    // Reads the speed of the encoder
    double getEncoderSpeed() {
        return 0;
        /*

        int8_t numOfData = 0x5;
        uint16_t rawData[numOfData] = {};
        rawData = readEncoderRegister(ENCODER_SPEED_REGISTER + numOfData);

        // Prepare raw speed
        int16_t rawSpeed = rawData[0];
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
        return (status);*/
    }

#endif

// Reads the temperature of the encoder
double getEncoderTemp() {

    // Declare a variable to store the temperature in
    double temperature = 0;

    // Read the temperature from the encoder
    //encoder.getTemperature(temperature);

    // Return the temperature
    return temperature;
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


//
static void Prompt_show(void) {
    OLED_Clear();
    //OLED_ShowString(0,0,"              ");
    OLED_ShowString(0,16,"   Finished!  ");
    OLED_ShowString(0,32,"  Please press ");
    OLED_ShowString(0,48,"Reset Key reboot");
}

*/