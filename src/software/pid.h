#ifndef __PID_H__
#define __PID_H__

// Include Arduino library
#include "Arduino.h"

// Include main PID library
#include "PID_v1.h"

// Encoder library (for getting current position)
#include "encoder.h"

// Main class for controlling the motor
// NOTE: This should be used for time increments between stepping
class StepperPID {

    // Public info (all of the functions to be used throughout the board)
    public:

        // Main constructor
        StepperPID(double min, double max);

        // Get functions for P, I, and D
        double getP() const;
        double getI() const;
        double getD() const;

        // Set functions for P, I, and D
        void setP(double newP);
        void setI(double newI);
        void setD(double newD);

        // Updates the PID loop with the P, I, and D variables
        void updateTunings();

        // Gets the desired motor angle
        double getDesiredPosition();

        // Set the desired motor angle (this is in absolute postioning,
        // meaning that something like 1500 is a valid position)
        void setDesiredPosition(double angle);

        // Runs the PID calculations and returns the output
        double update();

    // Private info (usually just variables)
    private:

        // Main variables for storing input, output, and setpoint
        double input = 0, output = 0, setpoint = 0;

        // P, I, and D terms for loop
        double P = 0, I = 0, D = 0;

        // PID object (modified)
        PID* pid;
};


#endif // ! __PID_H__