// Make sure that the motor header has only been used once
#ifndef __MOTOR_H__
#define __MOTOR_H__

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

        // Gets the microstepping mode of the motor
        int getMicrostepping();

        // Sets the microstepping mode of the motor
        void setMicrostepping(int setMicrostepping);

        // Moves the set point one step in the respective direction
        void step(bool positiveDirection);

        // Computes the next output of the motor
        void compute(float feedback);

    // Things that shouldn't be accessed by the outside
    private:

        // Variable to keep the desired angle of the motor
        float desiredAngle = 0;

        // Motor PID controller values
        float pTerm = 0;
        float iTerm = 0;
        float dTerm = 0;

        // Performance measurements (updated on compute)
        float error = 0;

        // Motor characteristics
        int current = 0;
        int microstepping = 1;
};


#endif