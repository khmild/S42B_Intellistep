#include "motor.h"
#include "oled.h"

// Main constructor (with PID terms)
StepperMotor::StepperMotor(float P, float I, float D) {

    // Variables for the input, setpoint, and output
    this -> pTerm = P;
    this -> iTerm = I;
    this -> dTerm = D;

    // Set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_A_DIR_1_PIN, OUTPUT);
    pinMode(COIL_A_DIR_2_PIN, OUTPUT);
    pinMode(COIL_B_DIR_1_PIN, OUTPUT);
    pinMode(COIL_B_DIR_2_PIN, OUTPUT);
    pinMode(COIL_A_POWER_OUTPUT_PIN, OUTPUT);
    pinMode(COIL_B_POWER_OUTPUT_PIN, OUTPUT);
}

// Constructor without PID terms
StepperMotor::StepperMotor() {

    // Just set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_A_DIR_1_PIN, OUTPUT);
    pinMode(COIL_A_DIR_2_PIN, OUTPUT);
    pinMode(COIL_B_DIR_1_PIN, OUTPUT);
    pinMode(COIL_B_DIR_2_PIN, OUTPUT);
    pinMode(COIL_A_POWER_OUTPUT_PIN, OUTPUT);
    pinMode(COIL_B_POWER_OUTPUT_PIN, OUTPUT);
}


// Returns the current RPM of the motor to two decimal places
float StepperMotor::getMotorRPM() {
    return (getEncoderSpeed() / 360);
}


// Returns the deviation of the motor from the PID loop
float StepperMotor::getAngleError() {
    return ((this -> desiredAngle) - getEncoderAngle());
}


// Returns the Proportional value of the PID loop
float StepperMotor::getPValue() {
    return (this -> pTerm);
}


// Returns the Integral value fo the PID loop
float StepperMotor::getIValue() {
    return (this -> iTerm);
}


// Returns the Derivative value for the PID loop
float StepperMotor::getDValue() {
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


// Gets the current of the motor (in mA)
int StepperMotor::getCurrent() {
    return (this -> current);
}


// Sets the current of the motor (in mA)
void StepperMotor::setCurrent(int current) {

    // Make sure that the new value isn't a -1 (all functions that fail should return a -1)
    if (current != -1) {

        // Make sure that the current is within the current bounds of the motor, if so set it
        this -> current = constrain(current, 0, 3500);
    }
}


// Get the microstepping divisor of the motor
int StepperMotor::getMicrostepping() {
    return (this -> microstepDivisor);
}


// Set the microstepping divisor of the motor
void StepperMotor::setMicrostepping(int setMicrostepping) {

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
        if ((newStepAngle == 1.8) || (newStepAngle == 0.9)) {

            // Save the new full step angle
            this -> fullStepAngle = newStepAngle;

            // Fix the microstep angle
            this -> microstepAngle = (this -> fullStepAngle) / (this -> microstepDivisor);
        }
    }
}


// Get the full step angle of the motor object
float StepperMotor::getFullStepAngle() {
    return (this -> fullStepAngle);
}


float StepperMotor::getMicrostepAngle() {
    return (this -> microstepAngle);
}


// Set if the motor direction should be reversed or not
void StepperMotor::setReversed(bool reversed) {

    // Set if the motor should be reversed
    this -> reversed = reversed;
}


// Get if the motor direction is reversed
bool StepperMotor::getReversed() {
    return (this -> reversed);
}


// Set if the motor enable should be inverted
void StepperMotor::setEnableInversion(bool inverted) {

    // Set the object's value
    this -> enableInverted = inverted;
}


// Get if the motor enable should be inverted
bool StepperMotor::getEnableInversion() {

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
float StepperMotor::getMicrostepMultiplier() {

    // Return the object's value
    return (this -> microstepMultiplier);
}


// Computes the coil values for the next step position and increments the set angle
void StepperMotor::step(STEP_DIR dir = PIN, bool useMultiplier = true) {

    // Main angle change (any inversions * angle of microstep)
    float angleChange = StepperMotor::invertDirection(this -> reversed) * (this -> fullStepAngle) / (this -> microstepDivisor);

    // Factor in the multiplier if specified
    if (useMultiplier) {
        angleChange *= (this -> microstepMultiplier);
    }
    
    // Invert the change based on the direction
    if (dir == PIN) {

        // Use the DIR_PIN state
        angleChange *= StepperMotor::invertDirection(digitalRead(DIRECTION_PIN) == LOW);
    }
    //else if (dir == COUNTER_CLOCKWISE) {
        // Nothing to do here, the value is already positive
    //}
    else if (dir == CLOCKWISE) {
        // Make the angle change in the negative direction
        angleChange *= -1;
    }

    // Set the desired angle to itself + the change in angle
    this -> desiredAngle += angleChange;

    /* Explanation of how the stepping calculations work:
       1. Current angle is received from the encoder
       2. The mod of the current angle and the full step angle is found. This will return the progress through a step cycle for that angle (4 full steps for a step cycle).
       The sin and cos functions show the current needed at each coil at a given angle through the step cycle.
       3. Check to see if the motor need to move in the positive direction or the negative
       4. Since a phase angle's period is equal to 2pi, we can divide the angle by 4 (to get the full step angle) and then again by the microstepping divisor (to split the microsteps)
       5. Effectively round the phase angle to the closest "high torque" one. The motor is most powerful when closest to fractional radian values.
       6. Current is calculated using the new angle and the current in the motor
       7. Both coils currents are checked. If the current is reversed, then the respective driver is assigned to move backward. The current output is set using the PWM frequency of the power output pins
    */

    // Get the current angle of the motor
    //float currentAngle = getEncoderAngle();

    /*
    // Mod the current angle by total phase angle to estimate the phase angle of the motor
    float phaseAngle = fmod(currentAngle, ((this -> fullStepAngle) * 4));

    // Calculate if the motor should move in positive or negative direction
    if (currentAngle < (this -> desiredAngle)) {
        // Motor should move in the positive direction by the angle of a step (2pi / microstep divisor)
        phaseAngle += (2*PI / ((this -> microstepping) * 4));

        // Round the phase angle to be set to the closest perfect angles (increases torque on the output)
        phaseAngle -= fmod(phaseAngle, (2*PI / ((this -> microstepping) * 4)));
    }
    else {
        // Motor should move in the negative direction by the angle of a step (2pi / microstep divisor)
        phaseAngle -= (2*PI / ((this -> microstepping) * 4));

        // Round the phase angle to be set to the closest perfect angles (increases torque on the output)
        phaseAngle += fmod(-phaseAngle, (2*PI / ((this -> microstepping) * 4)));
    }
    */

    // Drive the coils to the new phase angle
    this -> driveCoils(desiredAngle);
}


// Sets the coils of the motor based on the angle (angle should be in degrees)
void StepperMotor::driveCoils(float degAngle) {

    // Convert the angle to microstep values (formula uses degAngle * full steps for rotation * microsteps)
    float microstepAngle = degAngle * (this -> fullStepAngle / 360) * (this -> microstepDivisor);

    // Round the microstep angle, it has to be a whole value of the number of microsteps available
    // Also ensures that the coils are being driven to the major step positions (increases torque)
    uint16_t roundedMicrosteps = round(microstepAngle);

    // Make sure that the phase angle doesn't exceed the maximum
    roundedMicrosteps = roundedMicrosteps % SINE_STEPS;

    // Calculate the sine and cosine of the angle
    float angleSin = fastSin(roundedMicrosteps);
    float angleCos = fastCos(roundedMicrosteps);

    // If in reverse, we swap the sign of one of the angles
    if ((bool)digitalRead(DIRECTION_PIN) != (this -> reversed)) {
        angleCos = -angleCos;
    }

    // Equation comes out to be (effort * -1 to 1) depending on the sine/cosine of the phase angle
    float coilAPower = ((this -> current) * abs(angleSin));
    float coilBPower = ((this -> current) * abs(angleCos));

    // Check the if the coil should be energized to move backward or forward
    if(angleSin > 0)  {

        // Set first channel for forward movement
        setADirection(FORWARD);
    }
    else if (angleSin < 0) {

        // Set first channel for backward movement
        setADirection(BACKWARD);
    }

    // Check the if the coil should be energized to move backward or forward
    if(angleCos > 0)  {

        // Set second channel for forward movement
        setBDirection(FORWARD);
    }
    else if (angleCos < 0) {

        // Set second channel for backward movement
        setBDirection(BACKWARD);
    }
    
    // Set the current of the coils
    setCoilCurrent(coilAPower, coilBPower);
}


// Function for setting the A coil state
// ! Maybe change to faster pin writing
void StepperMotor::setADirection(COIL_STATE desiredState) {
    if (desiredState == FORWARD) {
        digitalWrite(COIL_A_DIR_1_PIN, HIGH);
        digitalWrite(COIL_A_DIR_2_PIN, LOW);
    }
    else if (desiredState == BACKWARD) {
        digitalWrite(COIL_A_DIR_1_PIN, LOW);
        digitalWrite(COIL_A_DIR_2_PIN, HIGH);
    }
    else if (desiredState == BRAKE) {
        digitalWrite(COIL_A_DIR_1_PIN, HIGH);
        digitalWrite(COIL_A_DIR_2_PIN, HIGH);
        analogWrite(COIL_A_POWER_OUTPUT_PIN, 0);
    }
    else if (desiredState == COAST) {
        digitalWrite(COIL_A_DIR_1_PIN, LOW);
        digitalWrite(COIL_A_DIR_2_PIN, LOW);
        analogWrite(COIL_A_POWER_OUTPUT_PIN, 0);
    }
}


// Function for setting the B coil state
// ! Maybe change to faster pin writing
void StepperMotor::setBDirection(COIL_STATE desiredState) {
    if (desiredState == FORWARD) {
        digitalWrite(COIL_B_DIR_1_PIN, HIGH);
        digitalWrite(COIL_B_DIR_2_PIN, LOW);
    }
    else if (desiredState == BACKWARD) {
        digitalWrite(COIL_B_DIR_1_PIN, LOW);
        digitalWrite(COIL_B_DIR_2_PIN, HIGH);
    }
    else if (desiredState == BRAKE) {
        digitalWrite(COIL_B_DIR_1_PIN, HIGH);
        digitalWrite(COIL_B_DIR_2_PIN, HIGH);
        analogWrite(COIL_B_POWER_OUTPUT_PIN, 0);
    }
    else if (desiredState == COAST) {
        digitalWrite(COIL_B_DIR_1_PIN, LOW);
        digitalWrite(COIL_B_DIR_2_PIN, LOW);
        analogWrite(COIL_B_POWER_OUTPUT_PIN, 0);
    }
}


// Sets the current of each of the coils (with mapping)
void StepperMotor::setCoilCurrent(uint16_t ACurrent, uint16_t BCurrent) {

    // Calculate the value to set the PWM interface to (based on algebraically manipulated equations from the datasheet)
    uint32_t aPWMValue = 2550 * BOARD_VOLTAGE * CURRENT_SENSE_RESISTOR * ACurrent;
    uint32_t bPWMValue = 2550 * BOARD_VOLTAGE * CURRENT_SENSE_RESISTOR * BCurrent;

    // Constrain the PWM values
    aPWMValue = constrain(aPWMValue, 0, 255);
    bPWMValue = constrain(bPWMValue, 0, 255);

    // Set the values on the output pins
    analogWrite(COIL_A_POWER_OUTPUT_PIN, aPWMValue);
    analogWrite(COIL_B_POWER_OUTPUT_PIN, bPWMValue);
}


// Sets the speed of the motor (basically sets the speed at which the step function is called)
float StepperMotor::speedToHz(float angularSpeed) {

    // Calculate the step angle (including microsteps)
    float stepAngle = (this -> fullStepAngle) / (this -> microstepDivisor);

    // Calculate the time between step calls
    return (angularSpeed / stepAngle);
}


// Enables the motor, powering the coils
void StepperMotor::enable() {

    // Check if the motor is enabled yet. No need to set all of the coils if they're already set
    if (!(this -> enabled)) {

        // Mod the current angle by total phase angle to estimate the phase angle of the motor, then set the coils to the current position
        // ! Possibly make this round the angle to the nearest nice microstep
        this -> driveCoils(getEncoderAngle());

        // Set the motor to be enabled
        this -> enabled = true;
    }
}


// Disables the motor, freeing the coils
void StepperMotor::disable() {

    // Check if the motor is not enabled yet. No need to disable the coils if they're already off
    if (this -> enabled) {

        // Set the A driver to brake
        setADirection(COAST);

        // Set the B driver to brake
        setBDirection(COAST);

        // Set the coils
        setCoilCurrent(0, 0);

        // Set the enabled to false so we don't repeat
        this -> enabled = false;
    }
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

    // Display that calibration is coming soon
    clearOLED();
    writeOLEDString(0, 0, "Calibration");
    writeOLEDString(0, 16, "coming soon");
    delay(5000);

    // Calibrate encoder offset

    // Calibrate PID loop
}


// Returns a -1 for true and a 1 for false
float StepperMotor::invertDirection(bool invert) {
    if (invert) {
        return -1;
    }
    else {
        return 1;
    }
}