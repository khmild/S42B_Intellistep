#include "motor.h"

// Main constructor (with PID terms)
StepperMotor::StepperMotor(float P, float I, float D) {

    // Variables for the input, setpoint, and output
    this -> pTerm = P;
    this -> iTerm = I;
    this -> dTerm = D;

    // Set the previous time to the current system time
    this -> previousTime = millis();
}

// Constructor without PID terms
StepperMotor::StepperMotor() {

    // Just set the previous time to the current system time
    this -> previousTime = millis();
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
    this -> current = current;
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
void StepperMotor::step(bool positiveDirection) {

    // Main angle change (any inversions * angle of microstep)
    float angleChange = StepperMotor::invertDirection(!positiveDirection) * StepperMotor::invertDirection(this -> reversed) * (this -> fullStepAngle) / (this -> microstepping);

    // Set the desired angle to itself + the change in angle
    this -> desiredAngle += angleChange;
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


// Returns a -1 for true and a 1 for false
float StepperMotor::invertDirection(bool invert) {
    if (invert) {
        return -1;
    }
    else {
        return 1;
    }
}