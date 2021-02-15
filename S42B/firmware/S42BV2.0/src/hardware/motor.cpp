#include "motor.h"

// Main constructor (with PID terms)
StepperMotor::StepperMotor(float P, float I, float D) {

    // Variables for the input, setpoint, and output
    this -> pTerm = P;
    this -> iTerm = I;
    this -> dTerm = D;
}

// Constructor without PID terms
StepperMotor::StepperMotor() {
    // Nothing to do in here
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
int StepperMotor::getMicrostepping() {
    return (this -> microstepping);
}
void StepperMotor::setMicrostepping(int setMicrostepping) {
    this -> microstepping = setMicrostepping;
}

// Moves the set point one step in the respective direction
void StepperMotor::step(bool positiveDirection) {
    // ! Need to write yet
}

// Computes the output of the motor
void StepperMotor::compute(float feedback) {

}