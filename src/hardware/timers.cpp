// Import the header file
#include "timers.h"

// Create a new timer instance
HardwareTimer *steppingTimer = new HardwareTimer(TIM1);

// A counter for the number of position faults (used for stall detection)
int outOfPosCount = 0;


// Sets up the motor update timer
void setupMotorTimers() {

    // Setup the step and stallfault pin
    pinMode(STEP_PIN, INPUT_PULLDOWN);
    pinMode(STALLFAULT_PIN, OUTPUT);

    // Attach the interupt to the step pin
    attachInterrupt(digitalPinToInterrupt(STEP_PIN), updateMotor, CHANGE);

    // Setup the timer for steps
    steppingTimer -> pause();
    steppingTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the interrupt
    steppingTimer -> setOverflow(STEP_UPDATE_FREQ, HERTZ_FORMAT);
    steppingTimer -> attachInterrupt(updateMotor);
    steppingTimer -> resume();
}


// Need to declare a function to power the motor coils for the step interrupt
void updateMotor() {

    // Check to see the state of the enable pin
    if (digitalRead(ENABLE_PIN) != motor.getEnableInversion()) {

        // The enable pin is off, the motor should be disabled
        motor.disable();

        // Shut off the StallGuard pin just in case
        digitalWrite(STALLFAULT_PIN, LOW);
    }
    else {

        // Enable the motor if it's not already (just energizes the coils to hold it in position)
        motor.enable();

        // Get the current angle of the motor (multiple reads take a longer time)
        double currentAngle = getEncoderAngle();

        // Calculate the angular deviation
        float angularDeviation = currentAngle - motor.desiredAngle;

        // Check to make sure that the motor is in range (it hasn't skipped steps)
        if (abs(angularDeviation) > motor.getMicrostepAngle()) {

            // Set the stepper to move in the correct direction
            if (angularDeviation > motor.getMicrostepAngle()) {

                // Motor is at a position larger than the desired one
                motor.driveCoils(currentAngle - motor.getMicrostepAngle());
            }
            else {
                // Motor is at a position smaller than the desired one
                motor.driveCoils(currentAngle + motor.getMicrostepAngle());
            }            

            // Check to see if the out of position faults have exceeded the maximum amounts
            if (outOfPosCount > (STEP_FAULT_TIME * (STEP_UPDATE_FREQ - 1))) {
                
                // The maximum count has been exceeded, trigger an endstop pulse
                digitalWrite(STALLFAULT_PIN, HIGH);
            }
            else {
                // Just count up, motor is out of position but not out of faults
                outOfPosCount++;
            }
        }
        else {
            // Reset the out of position count and the StallFault pin
            outOfPosCount = 0;
            digitalWrite(STALLFAULT_PIN, LOW);
        }
    }
}