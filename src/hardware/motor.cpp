#include "motor.h"

// These imports must be here to prevent linking circles
#include "oled.h"
#include "flash.h"

// Optimize for speed
#pragma GCC optimize ("-Ofast")

// Main constructor (with PID terms)
StepperMotor::StepperMotor(float P, float I, float D) {

    // Variables for the input, setpoint, and output
    this -> pTerm = P;
    this -> iTerm = I;
    this -> dTerm = D;

    // Set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_DIR_1_PINS[A], OUTPUT);
    pinMode(COIL_DIR_2_PINS[A], OUTPUT);
    pinMode(COIL_DIR_1_PINS[B], OUTPUT);
    pinMode(COIL_DIR_2_PINS[B], OUTPUT);
    pinMode(COIL_POWER_OUTPUT_PINS[A], OUTPUT);
    pinMode(COIL_POWER_OUTPUT_PINS[B], OUTPUT);

    // Configure the PWM
    analogWriteResolution(12); // STM32's is 12 bits max (max 4096)
    analogWriteFrequency(MOTOR_PWM_FREQ * 1000);

    // Disable the motor
    setState(DISABLED, true);
}

// Constructor without PID terms
StepperMotor::StepperMotor() {

    // Just set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_DIR_1_PINS[A], OUTPUT);
    pinMode(COIL_DIR_2_PINS[A], OUTPUT);
    pinMode(COIL_DIR_1_PINS[B], OUTPUT);
    pinMode(COIL_DIR_2_PINS[B], OUTPUT);
    pinMode(COIL_POWER_OUTPUT_PINS[A], OUTPUT);
    pinMode(COIL_POWER_OUTPUT_PINS[B], OUTPUT);

    // Configure the PWM current output pins
    this -> PWMCurrentPinInfo[A] = analogSetup(COIL_POWER_OUTPUT_PINS[A], MOTOR_PWM_FREQ * 1000, 0);
    this -> PWMCurrentPinInfo[B] = analogSetup(COIL_POWER_OUTPUT_PINS[B], MOTOR_PWM_FREQ * 1000, 0);

    // Disable the motor
    setState(DISABLED, true);
}


// Returns the current RPM of the motor to two decimal places
float StepperMotor::getMotorRPM() const {
    return (getEncoderSpeed() / 360);
}


// Returns the deviation of the motor from the PID loop
float StepperMotor::getAngleError() const {
    return (getAbsoluteAngle() - (this -> desiredAngle));
}


// Returns the Proportional value of the PID loop
float StepperMotor::getPValue() const {
    return (this -> pTerm);
}


// Returns the Integral value fo the PID loop
float StepperMotor::getIValue() const {
    return (this -> iTerm);
}


// Returns the Derivative value for the PID loop
float StepperMotor::getDValue() const {
    return (this -> dTerm);
}


// Sets the Proportional term of the PID loop
void StepperMotor::setPValue(float newP) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (newP != -1) {
        this -> pTerm = newP;
    }
}


// Sets the Integral term of the PID loop
void StepperMotor::setIValue(float newI) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (newI != -1) {
        this -> iTerm = newI;
    }
}


// Sets the Derivative of the PID loop
void StepperMotor::setDValue(float newD) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (newD != -1) {
        this -> dTerm = newD;
    }
}


#ifdef ENABLE_DYNAMIC_CURRENT

// Gets the acceleration factor for dynamic current
uint16_t StepperMotor::getDynamicAccelCurrent() const {
    return (this -> dynamicAccelCurrent);
}

// Gets the idle factor for dynamic current
uint16_t StepperMotor::getDynamicIdleCurrent() const {
    return (this -> dynamicIdleCurrent);
}

// Gets the max current factor for dynamic current
uint16_t StepperMotor::getDynamicMaxCurrent() const {
    return (this -> dynamicMaxCurrent);
}

// Sets the acceleration factor for dynamic current
void StepperMotor::setDynamicAccelCurrent(uint16_t newAccelFactor) {

    // Make sure that the value being set is positive (no negatives allowed)
    if (newAccelFactor >= 0) {
        this -> dynamicAccelCurrent = newAccelFactor;
    }
}

// Sets the idle factor for dynamic current
void StepperMotor::setDynamicIdleCurrent(uint16_t newIdleFactor) {

    // Make sure that the value being set is positive (no negatives allowed)
    if (newIdleFactor >= 0) {
        this -> dynamicIdleCurrent = newIdleFactor;
    }
}

// Sets the max current factor for dynamic current
void StepperMotor::setDynamicMaxCurrent(uint16_t newMaxCurrent) {

    // Make sure that the value being set is positive (no negatives allowed)
    if (newMaxCurrent >= 0) {
        this -> dynamicMaxCurrent = newMaxCurrent;
    }
}

#else // ! ENABLE_DYNAMIC_CURRENT

// Gets the RMS current of the motor (in mA)
uint16_t StepperMotor::getRMSCurrent() const {
    return (this -> rmsCurrent);
}


// Gets the peak current of the motor (in mA)
uint16_t StepperMotor::getPeakCurrent() const {
    return (this -> peakCurrent);
}


// Sets the RMS current of the motor (in mA)
void StepperMotor::setRMSCurrent(uint16_t rmsCurrent) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (rmsCurrent != -1) {

        // Make sure that the RMS current is within the current bounds of the motor, if so set it
        this -> rmsCurrent = constrain(rmsCurrent, 0, (uint16_t)MAX_RMS_BOARD_CURRENT);

        // Also set the peak current
        this -> peakCurrent = constrain((uint16_t)(rmsCurrent * 1.414), 0, (uint16_t)MAX_PEAK_BOARD_CURRENT);
    }
}


// Sets the peak current of the motor (in mA)
void StepperMotor::setPeakCurrent(uint16_t peakCurrent) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (peakCurrent != -1) {

        // Make sure that the peak current is within the current bounds of the motor, if so set it
        this -> peakCurrent = constrain(peakCurrent, 0, (uint16_t)MAX_PEAK_BOARD_CURRENT);

        // Also set the RMS current
        this -> rmsCurrent = constrain((uint16_t)(peakCurrent * 0.707), 0, (uint16_t)MAX_RMS_BOARD_CURRENT);
    }
}
#endif // ! ENABLE_DYNAMIC_CURRENT

// Get the microstepping divisor of the motor
uint16_t StepperMotor::getMicrostepping() const {
    return (this -> microstepDivisor);
}


// Set the microstepping divisor of the motor
void StepperMotor::setMicrostepping(uint16_t setMicrostepping) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (setMicrostepping != -1) {

        // Set the microstepping divisor
        this -> microstepDivisor = setMicrostepping;

        // Fix the microstep angle
        this -> microstepAngle = (this -> fullStepAngle) / (this -> microstepDivisor);
    }
}


// Set the full step angle of the motor (in degrees)
void StepperMotor::setFullStepAngle(float newStepAngle) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (newStepAngle != -1) {

        // Make sure that the value is one of the 2 common types
        // ! Maybe remove later?
        if ((newStepAngle == (float)1.8) || (newStepAngle == (float)0.9)) {

            // Save the new full step angle
            this -> fullStepAngle = newStepAngle;

            // Fix the microstep angle
            this -> microstepAngle = (this -> fullStepAngle) / (this -> microstepDivisor);
        }
    }
}


// Get the full step angle of the motor object
float StepperMotor::getFullStepAngle() const {
    return (this -> fullStepAngle);
}


float StepperMotor::getMicrostepAngle() const {
    return (this -> microstepAngle);
}


// Set if the motor direction should be reversed or not
void StepperMotor::setReversed(bool reversed) {

    // Set if the motor should be reversed
    this -> reversed = reversed;
}


// Get if the motor direction is reversed
bool StepperMotor::getReversed() const {
    return (this -> reversed);
}


// Set if the motor enable should be inverted
void StepperMotor::setEnableInversion(bool inverted) {

    // Set the object's value
    this -> enableInverted = inverted;
}


// Get if the motor enable should be inverted
bool StepperMotor::getEnableInversion() const {

    // Return the object's value
    return (this -> enableInverted);
}


// Set the microstep multiplier
void StepperMotor::setMicrostepMultiplier(float newMultiplier) {

    // Set the object's value if it is valid
    if (newMultiplier != -1) {
        (this -> microstepMultiplier) = newMultiplier;
    }
}


// Get the microstep multiplier
float StepperMotor::getMicrostepMultiplier() const {

    // Return the object's value
    return (this -> microstepMultiplier);
}


// Set the desired angle of the motor
void StepperMotor::setDesiredAngle(float newDesiredAngle) {
    this -> desiredAngle = newDesiredAngle;
}


// Get the desired angle of the motor
float StepperMotor::getDesiredAngle() const {

    // Return the object's value
    return (this -> desiredAngle);
}


// Computes the coil values for the next step position and increments the set angle
void StepperMotor::step(STEP_DIR dir, bool useMultiplier, bool updateDesiredAngle) {

    // Main angle change (any inversions * angle of microstep)
    float angleChange = (this -> fullStepAngle) / (this -> microstepDivisor);

    // Factor in the multiplier if specified
    if (useMultiplier) {
        angleChange *= (this -> microstepMultiplier);
    }
    
    // Invert the change based on the direction
    if (dir == PIN) {

        // Use the DIR_PIN state
        angleChange *= StepperMotor::invertDirection(digitalReadFast(DIRECTION_PIN) == (this -> reversed));
    }
    //else if (dir == COUNTER_CLOCKWISE) {
        // Nothing to do here, the value is already positive
    //}
    else if (dir == CLOCKWISE) {
        // Make the angle change in the negative direction
        angleChange *= -1;
    }

    // Update the desired angle if specified
    if (updateDesiredAngle) {

        // Angles are basically just added to desired, not really much to do here
        this -> desiredAngle += angleChange;
    }

    // Motor's current angle must always be updated to correctly move the coils
    this -> currentAngle += angleChange;

    // Drive the coils to their destination
    this -> driveCoils(currentAngle, dir);
}


// Sets the coils of the motor based on the angle (angle should be in degrees)
void StepperMotor::driveCoils(float degAngle, STEP_DIR direction) {

    // Make sure that the angle range is valid
    while (degAngle < 0) {
        degAngle += 360;
    }

    // Convert the angle to microstep values (formula uses degAngle * full steps for rotation * microsteps)
    float microstepAngle = (degAngle / this -> fullStepAngle) * (this -> microstepDivisor);

    // Round the microstep angle, it has to be a whole value of the number of microsteps available
    // Also ensures that the coils are being driven to the major step positions (increases torque)
    uint16_t roundedMicrosteps = round(microstepAngle);

    // Make sure that the phase angle doesn't exceed the steps per phase rotation (4 full steps to a phase rotation * whatever microstepping)
    roundedMicrosteps = roundedMicrosteps % (4 * (this -> microstepDivisor));
    
    // Calculate the sine and cosine of the angle
    uint16_t arrayIndex = roundedMicrosteps * (MAX_MICROSTEP_DIVISOR / (this -> microstepDivisor));
    
    // Ensure the index is in the sineTable range
    arrayIndex &= (SINE_VAL_COUNT - 1);
    
    // Calculate the coil settings
    int16_t coilAPercent = fastSin(arrayIndex);
    int16_t coilBPercent = fastCos(arrayIndex);

    // Equation comes out to be (effort * -1 to 1) depending on the sine/cosine of the phase angle
    #ifdef ENABLE_DYNAMIC_CURRENT

        // Get the current acceleration
        double angAccel = abs(getEncoderAccel());

        // Compute the coil power
        int16_t coilAPower = ((int16_t)(((angAccel * (this -> dynamicAccelCurrent)) + (this -> dynamicIdleCurrent)) * 1.414) * coilAPercent) / SINE_MAX;
        int16_t coilBPower = ((int16_t)(((angAccel * (this -> dynamicAccelCurrent)) + (this -> dynamicIdleCurrent)) * 1.414) * coilBPercent) / SINE_MAX;
    #else
        // Just use static current multipiers
        int16_t coilAPower = ((int16_t)(this -> peakCurrent) * coilAPercent) / SINE_MAX;
        int16_t coilBPower = ((int16_t)(this -> peakCurrent) * coilBPercent) / SINE_MAX;
    #endif

    // Check the if the coil should be energized to move backward or forward
    if (coilAPower > 0) {

        // Set first channel for forward movement
        setCoil(A, COIL_STATE::FORWARD, coilAPower);
    }
    else if (coilAPower < 0) {

        // Set first channel for backward movement
        setCoil(A, COIL_STATE::BACKWARD, -coilAPower);
    }
    else {
        setCoil(A, BRAKE);
    }


    // Check the if the coil should be energized to move backward or forward
    if (coilBPower > 0) {

        // Set first channel for forward movement
        setCoil(B, COIL_STATE::FORWARD, coilBPower);
    }
    else if (coilBPower < 0) {

        // Set first channel for backward movement
        setCoil(B, BACKWARD, -coilBPower);
    }
    else {
        setCoil(B, BRAKE);
    }
}


// Function for setting a coil state and current
void StepperMotor::setCoil(COIL coil, COIL_STATE desiredState, uint16_t current) {

    // ! Maybe for later?
    // Check the current. If the current is 0, then this means that the motor should go to its idle mode
    //if (current == 0) {
    //    desiredState = IDLE_MODE;
    //}

    // Decide how to deal with the coil based on current, re-enabling it after setting the direction
    if (desiredState == FORWARD) {
        digitalWriteFaster(COIL_DIR_1_PINS[coil], HIGH);
        digitalWriteFaster(COIL_DIR_2_PINS[coil], LOW);
        analogWrite(COIL_POWER_OUTPUT_PINS[coil], currentToPWM(current));
    }
    else if (desiredState == BACKWARD) {
        digitalWriteFaster(COIL_DIR_1_PINS[coil], LOW);
        digitalWriteFaster(COIL_DIR_2_PINS[coil], HIGH);
        analogWrite(COIL_POWER_OUTPUT_PINS[coil], currentToPWM(current));
    }
    else if (desiredState == BRAKE) {
        digitalWriteFaster(COIL_DIR_1_PINS[coil], HIGH);
        digitalWriteFaster(COIL_DIR_2_PINS[coil], HIGH);
    }
    else if (desiredState == COAST) {
        digitalWriteFaster(COIL_DIR_1_PINS[coil], LOW);
        digitalWriteFaster(COIL_DIR_2_PINS[coil], LOW);
    }

    // Update the output pin with the correct current
    analogSet(&PWMCurrentPinInfo[coil], currentToPWM(current));
}


// Calculates the current of each of the coils (with mapping)(current in mA)
uint32_t StepperMotor::currentToPWM(uint16_t current) const {

    // Calculate the value to set the PWM interface to (based on algebraically manipulated equations from the datasheet)
    uint32_t PWMValue = abs((40.96 * CURRENT_SENSE_RESISTOR * current) / BOARD_VOLTAGE);

    // Constrain the PWM value, then return it
    return constrain(PWMValue, 0, 4095);
}


// Sets the speed of the motor (basically sets the speed at which the step function is called)
float StepperMotor::speedToHz(float angularSpeed) const {

    // Calculate the step angle (including microsteps)
    float stepAngle = (this -> fullStepAngle) / (this -> microstepDivisor);

    // Calculate the time between step calls
    return (angularSpeed / stepAngle);
}


// Sets a new motor state
void StepperMotor::setState(MOTOR_STATE newState, bool clearErrors) {

    // Check to make sure that the state is different from the current
    if ((this -> state) != newState) {

        // Check if we need to clear the errors
        if (clearErrors) {
            switch (newState) {

                // Need to clear the disabled state and start the coils
                case ENABLED:

                    // Drive the coils the current angle of the shaft (just locks the output in place)
                    driveCoils(getAngle() - startupAngleOffset, COUNTER_CLOCKWISE);
                    
                    // The motor's current angle needs corrected
                    currentAngle = getAngle() - startupAngleOffset;
                    this -> state = ENABLED;

                // Same as enabled, just forced
                case FORCED_ENABLED:

                    // Drive the coils the current angle of the shaft (just locks the output in place)
                    driveCoils(getAngle() - startupAngleOffset, COUNTER_CLOCKWISE);
                    
                    // The motor's current angle needs corrected
                    currentAngle = getAngle() - startupAngleOffset;
                    this -> state = FORCED_ENABLED;

                // No other special processing needed, just disable the coils and set the state
                default:
                    motor.setCoil(A, IDLE_MODE);
                    motor.setCoil(B, IDLE_MODE);
                    this -> state = newState;
            }
        }
        else {
            // Only change the state if the current state is either enabled or disabled
            if ((this -> state) == ENABLED || (this -> state) == DISABLED) {
                
                // Decide when needs to happen based on the new state
                switch (newState) {

                    // Need to clear the disabled state and start the coils
                    case ENABLED:

                        // Drive the coils the current angle of the shaft (just locks the output in place)
                        driveCoils(getAngle() - startupAngleOffset, COUNTER_CLOCKWISE);
                        
                        // The motor's current angle needs corrected
                        currentAngle = getAngle() - startupAngleOffset;
                        this -> state = ENABLED;

                    // No other special processing needed, just disable the coils and set the state
                    default:
                        motor.setCoil(A, IDLE_MODE);
                        motor.setCoil(B, IDLE_MODE);
                        this -> state = newState;
                }
            }
        }
    }
}


// Return the state of the motor
MOTOR_STATE StepperMotor::getState() const {
    return (this -> state);
}


/*
// Computes the speed of the motor
float StepperMotor::compute(float currentAngle) {

    // Update the current time
    this -> currentTime = (float)millis();

    // Calculate the elapsed time
    this -> elapsedTime = (float)((this -> currentTime) - (this -> previousTime));

    // Calculate the error
    this -> error = (float)((this -> desiredAngle) - currentAngle);

    // Calculate the cumulative error (used with I term)
    this -> cumulativeError += (this -> error) * (this -> elapsedTime);

    // Calculate the rate error
    this -> rateError = ((this -> error) - (this -> lastError)) / elapsedTime;

    // Calculate the output with the errors and the coefficients
    float output = ((this -> pTerm) * (this -> error)) + ((this -> iTerm) * (this -> cumulativeError)) + ((this -> dTerm) * (this -> rateError));

    // Constrain the output to the maximum set output
    if (abs(output) > MAX_MOTOR_SPEED) {

        // Set the new output to the maximum output with the sign of the original output
        output = MAX_MOTOR_SPEED * (output / abs(output));
    }

    // Update the last computation parameters
    this -> lastError = this -> error;
    this -> previousTime = this -> currentTime;

    // Return the output of the PID loop
    return output;
}
*/

// Calibrates the encoder and the PID loop
void StepperMotor::calibrate() {
    // ! Write yet

    // Only include if specified
    #ifdef ENABLE_OLED

        // Display that calibration is coming soon
        clearOLED();
        writeOLEDString(0, 0, "Calibration", false);
        writeOLEDString(0, 16, "coming soon", true);
        delay(5000);
    #endif

    // Calibrate encoder offset

    // Calibrate PID loop

    // Erase all of the written parameters
    // ! Just a quick fix, needs a better fix later
    eraseParameters();

    // Write that the module is configured
    writeFlash(CALIBRATED_INDEX, true);
}


// Returns a -1 for true and a 1 for false
float StepperMotor::invertDirection(bool invert) const {
    if (invert) {
        return -1;
    }
    else {
        return 1;
    }
}