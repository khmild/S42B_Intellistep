#include "motor.h"

// Main constructor (with PID terms)
StepperMotor::StepperMotor(float P, float I, float D) {

    // Variables for the input, setpoint, and output
    this -> pTerm = P;
    this -> iTerm = I;
    this -> dTerm = D;

    // Set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_A_DIR_1, OUTPUT);
    pinMode(COIL_A_DIR_2, OUTPUT);
    pinMode(COIL_B_DIR_1, OUTPUT);
    pinMode(COIL_B_DIR_2, OUTPUT);
    pinMode(COIL_A_POWER_OUTPUT, OUTPUT);
    pinMode(COIL_B_POWER_OUTPUT, OUTPUT);
}

// Constructor without PID terms
StepperMotor::StepperMotor() {

    // Just set the previous time to the current system time
    this -> previousTime = millis();

    // Setup the pins as outputs
    pinMode(COIL_A_DIR_1, OUTPUT);
    pinMode(COIL_A_DIR_2, OUTPUT);
    pinMode(COIL_B_DIR_1, OUTPUT);
    pinMode(COIL_B_DIR_2, OUTPUT);
    pinMode(COIL_A_POWER_OUTPUT, OUTPUT);
    pinMode(COIL_B_POWER_OUTPUT, OUTPUT);
}


// Returns the current RPM of the motor to two decimal places
float StepperMotor::getMotorRPM() {
    // ! Write yet
    return 0;
}


// Returns the deviation of the motor from the PID loop
float StepperMotor::getPIDError() {
    return (this -> error);
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
    this -> pTerm = newP;
}


// Sets the Integral term of the PID loop
void StepperMotor::setIValue(float newI) {
    this -> iTerm = newI;
}


// Sets the Derivative of the PID loop
void StepperMotor::setDValue(float newD) {
    this -> dTerm = newD;
}


// Gets the current of the motor (in mA)
int StepperMotor::getCurrent() {
    return (this -> current);
}


// Sets the current of the motor (in mA)
void StepperMotor::setCurrent(int current) {

    // Make sure that the current is within the current bounds of the motor, if so set it
    this -> current = constrain(current, 0, 3500);
}


// Get the microstepping divisor of the motor
int StepperMotor::getMicrostepping() {
    return (this -> microstepping);
}


// Set the microstepping divisor of the motor
void StepperMotor::setMicrostepping(int setMicrostepping) {
    this -> microstepping = setMicrostepping;
}


// Set the full step angle of the motor (in degrees)
void StepperMotor::setFullStepAngle(float newStepAngle) {

    // Make sure that the value is one of the 2 common types (maybe remove later?)
    if ((newStepAngle == 1.8) || (newStepAngle == 0.9)) {
        this -> fullStepAngle = newStepAngle;
    }
}


// Set if the motor direction should be reversed or not
void StepperMotor::setReversed(bool reversed) {

    // Set if the motor should be reversed
    this -> reversed = reversed;
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


// Moves the set point one step in the respective direction
void StepperMotor::incrementAngle(bool positiveDirection) {

    // Main angle change (any inversions * angle of microstep)
    float angleChange = StepperMotor::invertDirection(!positiveDirection) * StepperMotor::invertDirection(this -> reversed) * (this -> fullStepAngle) / (this -> microstepping);

    // Set the desired angle to itself + the change in angle
    this -> desiredAngle += angleChange;
}

// Computes the coil values for the next step position
void StepperMotor::step() {

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
    float currentAngle = getEncoderAngle();

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

    // Equation comes out to be (effort * 0-1) depending on the sine/cosine of the phase angle
    int16_t coilAPower = round((this -> current) * sin(phaseAngle));
    int16_t coilBPower = round((this -> current) * cos(phaseAngle));

    // Check the if the coil should be energized to move backward or forward
    if(coilAPower > 0)  {

        // Set the power on the output pin
        analogWrite(COIL_A_POWER_OUTPUT, map(coilAPower, 0, 3500, 0, 255));

        // Set first channel for forward movement
        digitalWrite(COIL_A_DIR_1, HIGH);
        digitalWrite(COIL_A_DIR_2, LOW);
    }
    else if (coilAPower < 0) {

        // Set the power on the output pin
        analogWrite(COIL_A_POWER_OUTPUT, map(-coilAPower, 0, 3500, 0, 255));

        // Set first channel for forward movement
        digitalWrite(COIL_A_DIR_1, LOW);
        digitalWrite(COIL_A_DIR_2, HIGH);
    }
    else {

        // Turn off the coil, no current needed
        analogWrite(COIL_A_POWER_OUTPUT, 0);
    }

    // Check the if the coil should be energized to move backward or forward
    if(coilBPower > 0)  {

        // Set the power on the output pin
        analogWrite(COIL_B_POWER_OUTPUT, map(coilBPower, 0, 3500, 0, 255));

        // Set second channel for forward movement
        digitalWrite(COIL_B_DIR_1, HIGH);
        digitalWrite(COIL_B_DIR_2, LOW);
    }
    else if (coilBPower < 0) {

        // Set the power on the output pin
        analogWrite(COIL_B_POWER_OUTPUT, map(-coilBPower, 0, 3500, 0, 255));

        // Set second channel for forward movement
        digitalWrite(COIL_B_DIR_1, LOW);
        digitalWrite(COIL_B_DIR_2, HIGH);
    }
    else {

        // Turn off the coil, no current needed
        analogWrite(COIL_B_POWER_OUTPUT, 0);
    }
}


// Computes the output of the motor
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
    if (abs(output) > maxOutput) {

        // Set the new output to the maximum output with the sign of the original output
        output = maxOutput * (output / abs(output));
    }

    // Update the last computation parameters
    this -> lastError = this -> error;
    this -> previousTime = this -> currentTime;

    // Return the output of the PID loop
    return output;
}


// Calibrates the encoder and the PID loop
void StepperMotor::calibrate() {
    // ! Write yet

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