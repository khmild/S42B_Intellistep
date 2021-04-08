#include "encoder.h"

// Main SPI interface for the encoder
SPIClass3W encoderSPI;
SPISettings spiSettings;
Tle5012Ino encoder;

// The command to send to the encoder
uint16_t _command[2];


// Function to setup the encoder
errorTypes initEncoder() {

    // Create the SPI instance
    encoderSPI = SPIClass3W();

    // SPI settings to be used. Mainly done to correct the clock of the SPI bus. 
    // The clock should be 8MHz, with the SPI using the most significant bit first
    // CLK polarity set to 0 (low) and the phase set to 
    spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE1);

    // Initialize the bus with the settings selected
    encoderSPI.beginTransaction(ENCODER_CS_PIN, spiSettings);

    // End the SPI link. Seems dumb, but for some reason it allows the CPU to boot.
    encoderSPI.end();

    // Setup the encoder object to be used
    encoder = Tle5012Ino();

    // Start the communication with the encoder
    return encoder.begin();
    //encoderSPI.beginTransaction(ENCODER_CS, spiSettings);
    //encoder.writeSlaveNumber(encoder.mSlave);

    // Write that the SSC interface should be used
    //encoder.writeInterfaceType(Reg::interfaceType_t::SSC);

    // Set the encoder to use -180 to +180
    //encoder.reg.setAngleRange(Reg::angleRange_t::factor1);
}


// Reads the state of the encoder
uint16_t readEncoderState() {

    // Create a variable where the state will be stored
    uint16_t state = 0;

    // Read the value of the encoder
    encoder.readStatus(state);

    // Return the state that was received
    return state;
}


// Reads the value for the angle of the encoder
double getEncoderAngle() {

    // Create a value where the angle will be stored
    double angle = 0;

    // Poll the encoder for the angle
    encoder.getAngleValue(angle);

    // Return the received angle value
    return angle;
}


// Reads the speed of the encoder
double getEncoderSpeed() {

    // Declare a variable to store the encoder's speed in
    double speed = 0;

    // Get the speed from the encoder
    encoder.getAngleSpeed(speed);

    // Return the encoder's speed
    return speed;
}


// Reads the temperature of the encoder
double getEncoderTemp() {

    // Declare a variable to store the temperature in
    double temperature = 0;

    // Read the temperature from the encoder
    encoder.getTemperature(temperature);

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
*/