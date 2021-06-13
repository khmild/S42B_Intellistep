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

// Enumeration for coil letter
typedef enum {
    A,
    B
} COIL;

// Enumeration for stepping direction
typedef enum {
    PIN,
    COUNTER_CLOCKWISE,
    CLOCKWISE
} STEP_DIR;

// Enumeration for the enable state of the motor
typedef enum {
    ENABLED,
    DISABLED,
    FORCED_ENABLED,
    FORCED_DISABLED

    #ifdef ENABLE_OVERTEMP_PROTECTION
    , OVERTEMP
    #endif
} MOTOR_STATE;

// Stepper motor class (defined to make life a bit easier when dealing with the motor)
class StepperMotor {

    // Everything is public, with the expection of some private variables
    public:

        // Initialize the motor with the PID constants
        StepperMotor(float P, float I, float D);

        // Initialize the motor without the PID constants. Needs constants before moving
        StepperMotor();

        // Returns the current RPM of the motor to two decimal places
        float getMotorRPM() const;

        // Returns the deviation of the motor from the PID loop
        float getAngleError() const;

        // Returns the Proportional value of the PID loop
        float getPValue() const;

        // Returns the Integral value fo the PID loop
        float getIValue() const;

        // Returns the Derivative value for the PID loop
        float getDValue() const;

        // Sets the Proportional term of the PID loop
        void setPValue(float newP);

        // Sets the Integral term of the PID loop
        void setIValue(float newI);

        // Sets the Derivative of the PID loop
        void setDValue(float newD);


        // Dynamic current
        #ifdef ENABLE_DYNAMIC_CURRENT

        // Gets the acceleration factor for dynamic current
        uint16_t getDynamicAccelCurrent() const;

        // Gets the idle factor for dynamic current
        uint16_t getDynamicIdleCurrent() const;

        // Gets the max current factor for dynamic current
        uint16_t getDynamicMaxCurrent() const;

        // Sets the acceleration factor for dynamic current
        void setDynamicAccelCurrent(uint16_t newAccelFactor);

        // Sets the idle factor for dynamic current
        void setDynamicIdleCurrent(uint16_t newIdleFactor);

        // Sets the max current factor for dynamic current
        void setDynamicMaxCurrent(uint16_t newMaxCurrent);

        #else // ! ENABLE_DYNAMIC_CURRENT

        // Gets the RMS current of the motor (in mA)
        uint16_t getRMSCurrent() const;

        // Gets the peak current of the motor
        uint16_t getPeakCurrent() const;

        // Sets the RMS current of the motor (in mA)(Peak is adjusted to match)
        void setRMSCurrent(uint16_t rmsCurrent);

        // Sets the peak current of the motor (in mA)(RMS is adjusted to match)
        void setPeakCurrent(uint16_t peakCurrent);
        #endif


        // Gets the microstepping mode of the motor
        uint16_t getMicrostepping() const;

        // Sets the microstepping mode of the motor
        void setMicrostepping(uint16_t setMicrostepping);

        // Sets the angle of a full step of the motor
        void setFullStepAngle(float newStepAngle);

        // Gets the full step angle of the motor
        float getFullStepAngle() const;

        // Get the microstepping angle of the motor. This is the full steps divided by the microsteps. Used to speed up processing
        float getMicrostepAngle() const;

        // Set if the motor should be reversed
        void setReversed(bool reversed);

        // Get if the motor direction is reversed
        bool getReversed() const;

        // Set if the motor enable pin should be inverted
        void setEnableInversion(bool inverted);

        // Get if the motor enable pin is inverted
        bool getEnableInversion() const;

        // Set the microstep multiplier
        void setMicrostepMultiplier(float newMultiplier);

        // Get the microstep multiplier
        float getMicrostepMultiplier() const;

        // Set the desired motor angle
        void setDesiredAngle(float newDesiredAngle);

        // Gets the desired motor angle
        float getDesiredAngle() const;

        // Calculates the coil values for the motor and updates the set angle. 
        void step(STEP_DIR dir = PIN, bool useMultiplier = true, bool updateDesiredAngle = true);

        // Sets the coils to hold the motor at the desired phase angle
        void driveCoils(float angle, STEP_DIR direction);

        // Sets the state of a coil
        void setCoil(COIL coil, COIL_STATE desiredState, uint16_t current = 0);

        // Calculates the correct PWM setting based on an input current
        uint32_t currentToPWM(uint16_t current) const;

        // Sets the speed of the motor (angular speed is in deg/s)
        float speedToHz(float angularSpeed) const;
        
        // Sets the current state of the motor
        void setState(MOTOR_STATE newState, bool clearErrors = false);

        // Get the current state of the motor
        MOTOR_STATE getState() const;

        // Computes the next speed of the motor
        float compute(float feedback);

        // Calibrates the encoder and PID loop
        void calibrate();



    // Things that shouldn't be accessed by the outside
    private:

        // Function for turning booleans into -1 for true and 1 for false
        float invertDirection(bool invert) const;
        
        // Keeps the desired angle of the motor
        float desiredAngle = 0;

        // Keeps the current angle of the motor
        float currentAngle = 0;

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
        #ifdef ENABLE_DYNAMIC_CURRENT
            // Dynamic current settings
            uint16_t dynamicAccelCurrent = DYNAMIC_ACCEL_CURRENT;
            uint16_t dynamicIdleCurrent = DYNAMIC_IDLE_CURRENT;
            uint16_t dynamicMaxCurrent = DYNAMIC_MAX_CURRENT;
        #else
            // RMS Current (in mA)
            uint16_t rmsCurrent = STATIC_RMS_CURRENT;
            // Peak Current (in mA)
            uint16_t peakCurrent = (rmsCurrent * 1.414);
        #endif

        // Microstepping divisor
        uint16_t microstepDivisor = 1;

        // Angle of a full step
        float fullStepAngle = 1.8;

        // Microstep angle (full step / microstepping divisor)
        float microstepAngle = 1.8;

        // If the motor is enabled or not (saves time so that the enable and disable pins are only set once)
        MOTOR_STATE state = DISABLED;

        // If the motor direction is inverted
        bool reversed = false;

        // If the motor enable is inverted
        bool enableInverted = false;

        // Microstep multiplier (used to move a custom number of microsteps per step pulse)
        float microstepMultiplier = MICROSTEP_MULTIPLIER;
};


#endif