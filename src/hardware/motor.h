// Make sure that the motor header has only been used once
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Arduino.h"
#include "HardwareTimer.h"
#include "encoder.h"

// For sin() and fmod() function
#include "cmath"
#include <math.h>

// Import the pin mapping
#include "config.h"

// Stepper motor class (defined to make life a bit easier when dealing with the motor)
class StepperMotor {

    // Everything is public, with the expection of some private variables
    public:

        // Initialize the motor with the PID constants
        StepperMotor(float P, float I, float D);

        // Initialize the motor without the PID constants. Needs constants before moving
        StepperMotor();

        // Returns the current RPM of the motor to two decimal places
        float getMotorRPM();

        // Returns the deviation of the motor from the PID loop
        float getPIDError();

        // Returns the Proportional value of the PID loop
        float getPValue();

        // Returns the Integral value fo the PID loop
        float getIValue();

        // Returns the Derivative value for the PID loop
        float getDValue();

        // Sets the Proportional term of the PID loop
        void setPValue(float newP);

        // Sets the Integral term of the PID loop
        void setIValue(float newI);

        // Sets the Derivative of the PID loop
        void setDValue(float newD);

        // Gets the current of the motor (in mA)
        int getCurrent();

        // Sets the current of the motor (in mA)
        void setCurrent(int current);

        // Gets the microstepping mode of the motor
        int getMicrostepping();

        // Sets the microstepping mode of the motor
        void setMicrostepping(int setMicrostepping);

        // Sets the angle of a full step of the motor
        void setFullStepAngle(float newStepAngle);

        // Gets the full step angle of the motor
        float getFullStepAngle();

        // Set if the motor should be reversed
        void setReversed(bool reversed);

        // Get if the motor direction is reversed
        bool getReversed();

        // Set if the motor enable pin should be inverted
        void setEnableInversion(bool inverted);

        // Get if the motor enable pin is inverted
        bool getEnableInversion();

        // Moves the set point one step in the respective direction
        void incrementAngle();

        // Calculates the coil values for the motor
        void step();

        // Sets the speed of the motor (angular speed is in deg/s)
        float speedToHz(float angularSpeed);

        // Computes the update frequency of the motor based on PID calculations
        float computeStepHz();

        // Releases the coils, allowing the motor to freewheel
        void disable();

        // Computes the next speed of the motor
        float compute(float feedback);

        // Calibrates the encoder and PID loop
        void calibrate();

    // Things that shouldn't be accessed by the outside
    private:

        // Function for turning booleans into -1 for true and 1 for false
        float invertDirection(bool invert);

        // Variable to keep the desired angle of the motor
        float desiredAngle = 0;

        // Motor PID controller values
        float pTerm = 0;
        float iTerm = 0;
        float dTerm = 0;

        // Motor PID variables (to help with computations)
        float currentTime;
        float previousTime;
        float elapsedTime;

        // Performance measurements (updated on compute)
        float error;
        float lastError;
        float cumulativeError;
        float rateError;

        // Maximum value for the output of the motor
        float maxOutput = 1;

        // Motor characteristics
        // Current (in mA)
        int current = 0;

        // Microstepping divisor
        int microstepping = 1;

        // Angle of a full step
        float fullStepAngle = 1.8;

        // If the motor direction is inverted
        bool reversed = false;

        // If the motor enable is inverted
        bool enableInverted = false;

        // The stepping interval (in millis)
        float stepInterval = 10;
};


#endif