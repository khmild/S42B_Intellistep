// Import the header file
#include "timers.h"

// Optimize for speed
#pragma GCC optimize ("-Ofast")

// Create a new timer instance
HardwareTimer *steppingTimer = new HardwareTimer(TIM1);

// A counter for the number of position faults (used for stall detection)
uint16_t outOfPosCount = 0;


// Sets up the motor update timer
void setupMotorTimers() {

    // Setup the step and stallfault pin
    pinMode(STEP_PIN, INPUT_PULLDOWN);
    //pinMode(STALLFAULT_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    // Attach the interupt to the step pin
    attachInterrupt(digitalPinToInterrupt(STEP_PIN), stepMotor, CHANGE);

    // Setup the timer for steps
    steppingTimer -> pause();
    steppingTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the interrupt
    steppingTimer -> setOverflow(STEP_UPDATE_FREQ, HERTZ_FORMAT);
    steppingTimer -> attachInterrupt(updateMotor);
    steppingTimer -> refresh();
    steppingTimer -> resume();
}


// Pauses the timers, essentially disabling them for the time being
void disableInterrupts() {
    __disable_irq();
}


// Resumes the timers, re-enabling them
void enableInterrupts() {
    __enable_irq();
}


// Just a simple stepping function. Interrupt functions can't be instance methods
void stepMotor() {
    motor.step();
}


// Need to declare a function to power the motor coils for the step interrupt
void updateMotor() {

    // Check to see the state of the enable pin
    if (digitalRead(ENABLE_PIN) == motor.getEnableInversion()) {

        // The enable pin is off, the motor should be disabled
        motor.disable();

        // Shut off the StallGuard pin just in case
        //digitalWriteFast(STALLFAULT_PIN, LOW);
        digitalWriteFast(LED_PIN, LOW);
    }
    else {

        // Enable the motor if it's not already (just energizes the coils to hold it in position)
        motor.enable();

        // Get the current angle of the motor (multiple reads take a longer time)
        double currentAngle = getAbsoluteAngle();

        // Calculate the angular deviation
        float angularDeviation = currentAngle - motor.getDesiredAngle();

        // Check to make sure that the motor is in range (it hasn't skipped steps)
        if (abs(angularDeviation) > 16 * motor.getMicrostepAngle()) {

            // Set the stepper to move in the correct direction
            if (angularDeviation > 16 * motor.getMicrostepAngle()) {

                // Motor is at a position larger than the desired one
                motor.step(CLOCKWISE, false, false);
            }
            else {
                // Motor is at a position smaller than the desired one
                motor.step(COUNTER_CLOCKWISE, false, false);
            }            

            // Check to see if the out of position faults have exceeded the maximum amounts
            if (outOfPosCount > (STEP_FAULT_TIME * (STEP_UPDATE_FREQ - 1)) || abs(angularDeviation) > STEP_FAULT_ANGLE) {
                
                // The maximum count has been exceeded, trigger an endstop pulse
                #if STALLFAULT_PIN != NC 
                    digitalWriteFast(STALLFAULT_PIN, HIGH);
                #endif
                digitalWriteFast(LED_PIN, HIGH);
            }
            else {
                // Just count up, motor is out of position but not out of faults
                outOfPosCount++;
            }
        }
        else {
            // Reset the out of position count and the StallFault pin
            outOfPosCount = 0;
            #if STALLFAULT_PIN != NC
                digitalWriteFast(STALLFAULT_PIN, LOW);
            #endif
            digitalWriteFast(LED_PIN, LOW);
        }
    }
}