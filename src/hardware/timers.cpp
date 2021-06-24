// Import the header file
#include "timers.h"

// Optimize for speed
#pragma GCC optimize ("-Ofast")

// Create a new timer instance
HardwareTimer *correctionTimer = new HardwareTimer(TIM1);

// If step correction is enabled (helps to prevent enabling the timer when it is already enabled)
bool stepCorrection = false;

// A counter for the number of position faults (used for stall detection)
uint16_t outOfPosCount = 0;

// The number of times the current blocks on the interrupts. All blocks must be cleared to allow the interrupts to start again
// A block count is needed for nested functions. This ensures that function 1 (cannot be interrupted) will not re-enable the
// interrupts before the uninterruptible function 2 that called the first function finishes.
uint8_t interruptBlockCount = 0;

// Create a boolean to store if the StallFault pin has been enabled.
// Pin is only setup after the first StallFault. This prevents programming interruptions
#ifdef ENABLE_STALLFAULT
    bool stallFaultPinSetup = false;
#endif


// Tiny little function, just gets the time that the current program has been running
uint32_t sec() {
    return (millis() / 1000);
}

// Sets up the motor update timer
void setupMotorTimers() {

    // Interupts are in order of importance as follows -
    // - 0 - step pin change
    // - 1 - position correction

    // Setup the step and stallfault pin
    pinMode(STEP_PIN, INPUT_PULLDOWN);

    // Check if StallFault is enabled
    #ifdef ENABLE_STALLFAULT

        // Make sure that StallFault is enabled
        #ifdef STALLFAULT_PIN
            //pinMode(STALLFAULT_PIN, OUTPUT);
        #endif

        // Reset the LED pin
        pinMode(LED_PIN, OUTPUT);
    #endif

    // Attach the interupt to the step pin (subpriority is set in platformio config file)
    attachInterrupt(digitalPinToInterrupt(STEP_PIN), stepMotor, RISING);

    // Setup the timer for steps
    correctionTimer -> pause();
    correctionTimer -> setInterruptPriority(1, 0);
    correctionTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the interrupt
    correctionTimer -> setOverflow(round(STEP_UPDATE_FREQ * motor.getMicrostepping()), HERTZ_FORMAT);
    correctionTimer -> attachInterrupt(correctMotor);
    correctionTimer -> refresh();
    correctionTimer -> resume();
}


// Pauses the timers, essentially disabling them for the time being
void disableInterrupts() {

    // Add one to the interrupt block counter
    interruptBlockCount++;

    // Disable the interrupts if this is the first block
    if (interruptBlockCount == 1) {
        __disable_irq();
    }
}


// Resumes the timers, re-enabling them
void enableInterrupts() {

    // Remove one of the blocks on the interrupts
    interruptBlockCount--;

    // If all of the blocks are gone, then re-enable the interrupts
    if (interruptBlockCount == 0) {
        __enable_irq();
    }
}


// Enables the step correction timer
void enableStepCorrection() {

    // Enable the timer if it isn't already, then set the variable
    if (!stepCorrection) {
        correctionTimer -> resume();
        stepCorrection = true;
    }
}


// Disables the step correction timer
void disableStepCorrection() {

    // Disable the timer if it isn't already, then set the variable
    if (stepCorrection) {
        correctionTimer -> pause();
        stepCorrection = false;
    }
}


// Set the speed of the step correction timer
void updateCorrectionTimer() {

    // Check the previous value of the timer, only changing if it is different
    if (correctionTimer -> getCount(HERTZ_FORMAT) != round(STEP_UPDATE_FREQ * motor.getMicrostepping())) {
        correctionTimer -> pause();
        correctionTimer -> setOverflow(round(STEP_UPDATE_FREQ * motor.getMicrostepping()), HERTZ_FORMAT);
        correctionTimer -> refresh();
        correctionTimer -> resume();
    }
}


// Just a simple stepping function. Interrupt functions can't be instance methods
void stepMotor() {
    motor.step();
}


// Need to declare a function to power the motor coils for the step interrupt
void correctMotor() {

    // Check to see the state of the enable pin
    if ((digitalReadFast(ENABLE_PIN) != motor.getEnableInversion()) && (motor.getState() != FORCED_ENABLED)) {

        // The enable pin is off, the motor should be disabled
        motor.setState(DISABLED);

        // Only include if StallFault is enabled
        #ifdef ENABLE_STALLFAULT

            // Shut off the StallGuard pin just in case
            // (No need to check if the pin is valid, the pin will never be set up if it isn't valid)
            #ifdef STALLFAULT_PIN
                digitalWriteFast(STALLFAULT_PIN, LOW);
            #endif

            // Fix the LED pin
            setLED(LOW);
        #endif
    }
    else {

        // Enable the motor if it's not already (just energizes the coils to hold it in position)
        motor.setState(ENABLED);

        // Get the angular deviation
        float angularDeviation = motor.getAngleError();

        // Check to make sure that the motor is in range (it hasn't skipped steps)
        if (abs(angularDeviation) > motor.getMicrostepAngle()) {

            // Set the stepper to move in the correct direction
            if (angularDeviation > 0) {

                // Motor is at a position larger than the desired one
                motor.step(CLOCKWISE, false, false);
            }
            else {
                // Motor is at a position smaller than the desired one
                motor.step(COUNTER_CLOCKWISE, false, false);
            }


            // Only use StallFault code if needed
            #ifdef ENABLE_STALLFAULT

                // Check to see if the out of position faults have exceeded the maximum amounts
                if (outOfPosCount > (STEP_FAULT_TIME * ((STEP_UPDATE_FREQ * motor.getMicrostepping()) - 1)) || abs(angularDeviation) > STEP_FAULT_ANGLE) {

                    // Setup the StallFault pin if it isn't already
                    // We need to wait for a fault because otherwise the programmer will be unable to program the board
                    #ifdef STALLFAULT_PIN
                    if (!stallFaultPinSetup) {

                        // Setup the StallFault pin
                        LL_GPIO_InitTypeDef GPIO_InitStruct;
                        GPIO_InitStruct.Pin = STM_LL_GPIO_PIN(STALLFAULT_PIN);
                        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
                        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
                        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
                        GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
                        LL_GPIO_Init(get_GPIO_Port(STM_PORT(STALLFAULT_PIN)), &GPIO_InitStruct);

                        // The StallFault pin is all set up
                        stallFaultPinSetup = true;
                    }

                    // The maximum count has been exceeded, trigger an endstop pulse
                    digitalWriteFast(STALLFAULT_PIN, HIGH);
                    #endif

                    // Also give an indicator on the LED
                    setLED(HIGH);
                }
                else {
                    // Just count up, motor is out of position but not out of faults
                    outOfPosCount++;
                }
            #endif
        }

        // Only if StallFault is enabled
        #ifdef ENABLE_STALLFAULT
        else {

            // Reset the out of position count and the StallFault pin
            outOfPosCount = 0;

            // Pull the StallFault low if it's setup
            // No need to check the validity of the pin here, it wouldn't be setup if it wasn't valid
            #ifdef STALLFAULT_PIN
            if (stallFaultPinSetup) {
                digitalWriteFast(STALLFAULT_PIN, LOW);
            }
            #endif

            // Also toggle the LED for visual purposes
            setLED(LOW);
        }
        #endif
    }
}