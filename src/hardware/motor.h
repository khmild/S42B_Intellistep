// Make sure that the motor header has only been used once
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Arduino.h"
#include "HardwareTimer.h"
#include "encoder.h"

// For sin() and fmod() function
//#include "cmath"
//#include <math.h>
#include "fastSine.h"

// Import the pin mapping
#include "config.h"

// Enumeration for coil states
typedef enum {
    FORWARD,
    BACKWARD,
    BRAKE,
    COAST
} COIL_STATE;

// Enumeration for stepping direction
typedef enum {
    PIN,
    COUNTER_CLOCKWISE,
    CLOCKWISE
} STEP_DIR;

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
        float getAngleError();

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

        // Get the microstepping angle of the motor. This is the full steps divided by the microsteps. Used to speed up processing
        float getMicrostepAngle();

        // Set if the motor should be reversed
        void setReversed(bool reversed);

        // Get if the motor direction is reversed
        bool getReversed();

        // Set if the motor enable pin should be inverted
        void setEnableInversion(bool inverted);

        // Get if the motor enable pin is inverted
        bool getEnableInversion();

        // Set the microstep multiplier
        void setMicrostepMultiplier(float newMultiplier);

        // Get the microstep multiplier
        float getMicrostepMultiplier();

        // Calculates the coil values for the motor and updates the set angle. 
        void step(STEP_DIR dir = PIN, bool useMultiplier = true);

        // Sets the coils to hold the motor at the desired phase angle
        void driveCoils(float angle);

        // Sets the state of the A coil
        void setADirection(COIL_STATE desiredState);

        // Sets the state of the B coil
        void setBDirection(COIL_STATE desiredState);

        // Sets the current of the coils
        void setCoilCurrent(uint16_t ACurrent, uint16_t BCurrent);

        // Sets the speed of the motor (angular speed is in deg/s)
        float speedToHz(float angularSpeed);

        // Enables the coils, preventing motor movement
        void enable();

        // Releases the coils, allowing the motor to freewheel
        void disable();

        // Computes the next speed of the motor
        float compute(float feedback);

        // Calibrates the encoder and PID loop
        void calibrate();

        // Variable to keep the desired angle of the motor
        float desiredAngle = 0;

    // Things that shouldn't be accessed by the outside
    private:

        // Function for turning booleans into -1 for true and 1 for false
        float invertDirection(bool invert);

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

        // Motor characteristics
        // Current (in mA)
        int current = 750;

        // Microstepping divisor
        int microstepDivisor = 1;

        // Angle of a full step
        float fullStepAngle = 1.8;

        // Microstep angle (full step / microstepping divisor)
        float microstepAngle = 1.8;

        // If the motor is enabled or not (saves time so that the enable and disable pins are only set once)
        bool enabled = false;

        // If the motor direction is inverted
        bool reversed = false;

        // If the motor enable is inverted
        bool enableInverted = false;

        // The stepping interval (in millis)
        float stepInterval = 10;

        // Microstep multiplier (used to move a custom number of microsteps per step pulse)
        float microstepMultiplier = 1;
};


#endif