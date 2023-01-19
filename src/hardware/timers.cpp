// Import the header file
#include "timers.h"

// Optimize for speed
#pragma GCC optimize ("-Ofast")

// Timer uses:
// - TIM1 - Used to time correction calculations
// - TIM2 - Used to count steps (stores master record of steps)
// - TIM3 - Used to generate PWM signal for motor
// - TIM4 - Used to schedule steps for the motor (used by PID and direct stepping)

#ifndef DISABLE_CORRECTION_TIMER
// Create a new timer instance
HardwareTimer *correctionTimer = new HardwareTimer(TIM1);

// The frequency at which to update the correction timer
uint32_t correctionUpdateFreq = round(STEP_UPDATE_FREQ * motor.getMicrostepping());
#endif

// If step correction is enabled (helps to prevent enabling the timer when it is already enabled)
bool stepCorrection = false;

// A counter for the number of position faults (used for stall detection)
uint16_t outOfPosCount = 0;

// The number of times the current blocks on the interrupts. All blocks must be cleared to allow the interrupts to start again
// A block count is needed for nested functions. This ensures that function 1 (cannot be interrupted) will not re-enable the
// interrupts before the uninterruptible function 2 that called the first function finishes.
static uint8_t interruptBlockCount = 0;


// Setup everything related to the PID timer if needed
#ifdef ENABLE_PID
    // Create an instance of the PID class
    StepperPID pid = StepperPID();
#endif


// Setup everything related to step scheduling
#if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))

    // Main timer for scheduling steps
    HardwareTimer *stepScheduleTimer = new HardwareTimer(TIM4);

    // Direction of movement for direct steps
    STEP_DIR scheduledStepDir = POSITIVE;

    // Maximum stepping rate for the motor
    uint32_t maximumSteppingRate = 0;

    #ifdef ENABLE_ACCELERATION
        // Motor starts moving with this stepping rate(Hz)(before the acceleration)
        #define ACCEL_START_STEP_RATE 10

        // Actual stepping rate for the motor
        uint32_t actualSteppingRate = 0;

        uint32_t stepsToAccelerate = 0;

        // Variable for half way detection
        uint64_t halfStepsScheduled = 0;
        
        // Specifies acceleration status
        int8_t accelerate = 0;
        
        #ifdef RAMP_ACCELERATION
            // Acceleration per step
            uint16_t oneStepAcceleration = 0;
        #endif

        #ifdef S_CURVE_ACCELERATION
            uint16_t sCurveElementsNum = 0;
            uint16_t sCurveActualIndex = 0;
        #endif
    #endif
    
    // Remaining step count
    uint64_t remainingScheduledSteps = 0;

    // Stores if the timer is enabled
    // Saves large amounts of cycles as the timer only has to be toggled on a change
    bool stepScheduleTimerEnabled = false;

    // Stores if the timer should decrement the number of steps left
    bool decrementRemainingSteps = false;
#endif

// Tiny little function, just gets the time that the current program has been running
uint32_t sec() {
    return (millis() / 1000);
}

// Sets up the motor update timer
void setupMotorTimers() {

    // Interupts are in order of importance as follows -
    // - 5 - hardware step counter overflow handling
    // - 6.0 - enable pin change
    // - 6.1 - direction pin change
    // - 6.2 - step pin change
    // - 7.0 - position correction (or PID interval update)
    // - 7.1 - scheduled steps (if ENABLE_DIRECT_STEPPING or ENABLE_PID)

    // Attach the interupt to the enable pin
    attachInterrupt(ENABLE_PIN, enablePinISR, CHANGE); // input is pull-upped to VDD
    HAL_NVIC_SetPriority(EXTI2_IRQn, EN_PIN_PREMPT_PRIOR, EN_PIN_SUB_PRIOR); // Fix priority

    // Attach the interupt to the step pin
    // A normal step pin triggers on the rising edge. However, as explained here: https://github.com/CAP1Sup/Intellistep/pull/50#discussion_r663051004,
    // the optocoupler inverts the signal. Therefore, the falling edge is the correct value.
    attachInterrupt(STEP_PIN, stepMotor, FALLING); // input is pull-upped to VDD
    HAL_NVIC_SetPriority(EXTI0_IRQn, STEP_PIN_PREMPT_PRIOR, STEP_PIN_SUB_PRIOR); // Fix priority

    #ifndef DISABLE_CORRECTION_TIMER
    // Setup the timer for steps
    correctionTimer -> pause();
    correctionTimer -> setInterruptPriority(POS_CORRECTION_PREMPT_PRIOR, POS_CORRECTION_SUB_PRIOR);
    correctionTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the timed interrupt

    // Set the update rate and the variable that stores it
    correctionUpdateFreq = round(STEP_UPDATE_FREQ * motor.getMicrostepping());
    correctionTimer -> setOverflow(correctionUpdateFreq, HERTZ_FORMAT);

    // Finish setting up the correction timer
    #ifndef CHECK_STEPPING_RATE
        correctionTimer -> attachInterrupt(correctMotor);
    #endif
    #endif // ! DISABLE_CORRECTION_TIMER

    // Setup step schedule timer if it is enabled
    #if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))
        stepScheduleTimer -> pause();
        stepScheduleTimer -> setInterruptPriority(SCHED_STEPS_PREMPT_PRIOR, SCHED_STEPS_SUB_PRIOR);
        stepScheduleTimer -> setMode(1, TIMER_OUTPUT_COMPARE); // Disables the output, since we only need the timed interrupt
        stepScheduleTimer -> attachInterrupt(stepScheduleHandler);
        // Don't re-enable the motor, that will be done when the steps are scheduled
    #endif
}


// Disables all of the motor timers (in case the motor cannot be messed with)
void disableMotorTimers() {

    // Detach the step interrupt
    detachInterrupt(STEP_PIN);

    // Disable the correctional timer
    #ifndef DISABLE_CORRECTION_TIMER
    if (stepCorrection) {
        correctionTimer -> pause();
        syncInstructions();
    }
    #endif

    // Disable the stepping timer if it is enabled
    #if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))
    disableStepScheduleTimer();
    #endif
}


// Enable all of the motor timers
void enableMotorTimers() {

    // Attach the step interrupt
    attachInterrupt(STEP_PIN, stepMotor, FALLING); // input is pull-upped to VDD
    HAL_NVIC_SetPriority(EXTI0_IRQn, STEP_PIN_PREMPT_PRIOR, STEP_PIN_SUB_PRIOR); // Fix priority

    // Enable the correctional timer
    #ifndef DISABLE_CORRECTION_TIMER
    if (stepCorrection) {
        correctionTimer -> resume();
        syncInstructions();
    }
    #endif
}


// Pauses the timers, essentially disabling them for the time being
void disableInterrupts() {

    // Disable the interrupts if this is the first block
    if (interruptBlockCount == 0) {
        __disable_irq();
        syncInstructions();
    }

   // Add one to the interrupt block counter
    interruptBlockCount++;
}


// Resumes the timers, re-enabling them
void enableInterrupts() {

    // Remove one of the blocks on the interrupts
    interruptBlockCount--;

    // If all of the blocks are gone, then re-enable the interrupts
    if (interruptBlockCount == 0) {
        __enable_irq();
        syncInstructions();
    }
}


// Enables the step correction timer
void enableStepCorrection() {

    // Enable the timer if it isn't already, then set the variable
    if (!stepCorrection) {
        #ifndef DISABLE_CORRECTION_TIMER
        correctionTimer -> resume();
        #endif
        stepCorrection = true;
        syncInstructions();
    }
}


// Disables the step correction timer
void disableStepCorrection() {

    // Check if the timer is disabled
    if (stepCorrection) {

        #ifndef DISABLE_CORRECTION_TIMER
        // Disable the timer
        correctionTimer -> pause();
        #endif

        // Set that there will be no more step correction
        stepCorrection = false;
        syncInstructions();
    }

    // Disable the stepping timer if needed
    #if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))
    disableStepScheduleTimer();
    #endif
}


// Set the speed of the step correction timer
void updateCorrectionTimer() {

    #ifndef DISABLE_CORRECTION_TIMER
    // Check the previous value of the timer, only changing if it is different
    if (correctionUpdateFreq != (uint32_t)round(STEP_UPDATE_FREQ * (uint32_t)motor.getMicrostepping())) {

        // Compute the new freq, then set it
        correctionUpdateFreq = (uint32_t)round(STEP_UPDATE_FREQ * motor.getMicrostepping());
        correctionTimer -> setOverflow(correctionUpdateFreq, HERTZ_FORMAT);

        // Refresh the timer, then enable it if step correction is enabled
        correctionTimer -> refresh();
        if (stepCorrection) {
            correctionTimer -> resume();
        }

        // Sync the instruction barrier
        syncInstructions();
    }
    #endif
}


// Process a change in the enable pin
void enablePinISR() {
    // Motor should be disabled
    if (GPIO_READ(ENABLE_PIN) != motor.getEnableInversion()) {

        // Disable motor
        motor.setState(DISABLED);
    }
    else {
        // New motor state should be enabled
        motor.setState(ENABLED);
    }
}


// Just a simple stepping function. Interrupt functions can't be instance methods
void stepMotor() {

    // Hack from BTT, instead of using hardware to decide count, it can be done by software
    // This is really dumb, but otherwise the counter loses a pulse occasionally
    #ifdef USE_LEGACY_STEP_CNT_SETUP
    if (GPIO_READ(DIRECTION_PIN) == motor.getReversed()) {
        LL_TIM_SetCounterMode(TIM2, TIM_COUNTERMODE_DOWN);
    }
    else {
        LL_TIM_SetCounterMode(TIM2, TIM_COUNTERMODE_UP);
    }
    #endif

    #ifdef CHECK_STEPPING_RATE
        GPIO_WRITE(LED_PIN, HIGH);
    #endif

    // Step the motor
    motor.step((STEP_DIR)DIRECTION(GPIO_READ(DIRECTION_PIN)), motor.microstepMultiplier);

    #ifdef CHECK_STEPPING_RATE
        GPIO_WRITE(LED_PIN, LOW);
    #endif
}

#define S_CURVE_SIZE 2000
uint32_t S_curve[S_CURVE_SIZE];

void generateSCurve(uint16_t numOfElements, uint16_t maxSpeed)
{
    /* CONFIGURATION VARIABLES */
    float mn = 0;      // start point
    float mx = 10;     // stop point
    
    float a1 = 3;      // acceleration slope
    float a2 = 2;      // deceleration slope
    
    float c1 = 2;    // half point of slope
    float c2 = 8.5;    // half point of slope
    
    /* TEMP VARIABLES */
    float y1;          
    float y2;
    float temp;
    
    float f[numOfElements];
    int n = numOfElements; 
    int i;

    for (i = 0; i < n ; i++)  
    {
        temp = float((mx-mn)/(n));
        f[i] = mn + temp * i;
        
        y1 = 1/(1 + exp(-a1*(f[i]-c1)));        /* acceleration */
        y2 = 1/(1 + exp(-a2*(f[i]-c2)));        /* deceleration */
        S_curve[i] = abs((y1)- y2)*maxSpeed;    /* scale to max speed */          
    }
}


// Need to declare a function to power the motor coils for the step interrupt
void correctMotor() {
    #ifdef CHECK_CORRECT_MOTOR_RATE
        GPIO_WRITE(LED_PIN, HIGH);
    #endif

    // Make sure that the motor isn't disabled
    if (motor.getState() == ENABLED || motor.getState() == FORCED_ENABLED) {
        // "Smart" step correction using encoder counts
        #ifdef STEP_CORRECTION
        // Get the current angle of the motor
        float currentAbsAngle = motor.encoder.getAbsoluteAngleAvgFloat();

        // Get the angular deviation
        int32_t stepDeviation = motor.getStepError(currentAbsAngle);

        #else
        // Basic step correction using number of currently handled steps
        // ! NOT PROTECTED FROM SKIPPING!
        int32_t stepDeviation = motor.getUnhandledStepCNT();

        #endif // ! STEP_CORRECTION

        // Check to make sure that the motor is in range (it hasn't skipped steps)
        if ((stepDeviation > 1) || (stepDeviation < -1)) {
            // Run PID stepping if enabled
            #ifdef ENABLE_PID
                
                // Run the PID calculations
                int32_t pidOutput = round(pid.compute(currentAbsAngle, motor.getDesiredAngle()));
                uint32_t stepFreq = abs(pidOutput);

                #ifdef ENABLE_ACCELERATION
                    #ifdef RAMP_ACCELERATION
                        // ACCELERATION
                        if (accelerate == 1)
                        {
                            // Limit motor speed to the maximum value at this point
                            stepFreq = constrain(stepFreq, 0, actualSteppingRate);
                            // Increase speed step by step until maximum value is reached
                            actualSteppingRate += oneStepAcceleration;

                            // Stop acceleration if the speed is reached
                            if(actualSteppingRate >= maximumSteppingRate)
                            {
                                actualSteppingRate = maximumSteppingRate;
                                accelerate = 0;
                            }
                        }

                        // CONSTANT SPEED
                        if (accelerate == 0)
                        {
                            stepFreq = constrain(stepFreq, 0, maximumSteppingRate);
                        }

                        // DECELERATION
                        if (accelerate == (-1))
                        {
                            // Limit motor speed to the maximum value at this point
                            stepFreq = constrain(stepFreq, 0, actualSteppingRate);
                            // Decrease speed step by step until minimum value is reached
                            actualSteppingRate -= oneStepAcceleration;

                            // Stop acceleration if the speed is reached
                            if(actualSteppingRate < ACCEL_START_STEP_RATE+50)
                            {
                                actualSteppingRate = ACCEL_START_STEP_RATE+50;
                                accelerate = 0;
                            }
                        }

                        // DONE STEPPING TO THE POSITION
                        if (accelerate == (-99))
                        {
                            stepFreq = constrain(stepFreq, 0, maximumSteppingRate);
                            decrementRemainingSteps = false;
                        }
                    #endif

                    #ifdef S_CURVE_ACCELERATION
                        char buffer[32];
                        // ACCELERATION
                        if (accelerate == 1)
                        {
                            sprintf(buffer, "Accelerating %ld %ld\r\n", S_curve[sCurveActualIndex], stepFreq);
                            //sendSerialMessage(buffer);
                            stepFreq = constrain(stepFreq, 0, S_curve[sCurveActualIndex]);
                            sCurveActualIndex++;
                            if(sCurveActualIndex > sCurveElementsNum/2)
                            {
                                accelerate = 0;
                            }
                        }

                        // CONSTANT SPEED
                        if (accelerate == 0)
                        {
                            sprintf(buffer, "Const speed %ld %ld\r\n", S_curve[sCurveActualIndex], stepFreq);
                            //sendSerialMessage(buffer);
                            stepFreq = constrain(stepFreq, 0, maximumSteppingRate);
                        }

                        // DECELERATION
                        if (accelerate == -1)
                        {
                            sprintf(buffer, "Decelerating %ld %ld\r\n", S_curve[sCurveActualIndex], stepFreq);
                            //sendSerialMessage(buffer);
                            stepFreq = constrain(stepFreq, 0, S_curve[sCurveActualIndex]);
                            sCurveActualIndex++;
                            if (sCurveActualIndex >= sCurveElementsNum)
                            {
                                accelerate = 0;
                            }
                            
                        }

                        // DONE STEPPING TO THE POSITION
                        if (accelerate == -99)
                        {
                            stepFreq = constrain(stepFreq, 0, maximumSteppingRate);
                            decrementRemainingSteps = false;
                        }
                    #endif
                #else
                    stepFreq = constrain(stepFreq, 0, maximumSteppingRate);
                #endif
                

                // Notify the controller that motor is arrived to the desired position
                if(stepFreq < 40 && !motor.inPosition())
                {
                    sendSerialMessage("Motor in position\r\n");
                    motor.steppingDone();
                }

                // Check if the value is 0 (meaning that the timer needs disabled)
                if (stepFreq == 0) {
                    // The timer needs disabled
                    disableStepScheduleTimer();
                }
                else {
                    // Set the direction
                    if (pidOutput > 0) {
                        scheduledStepDir = POSITIVE;
                    }
                    else {
                        scheduledStepDir = NEGATIVE;
                    }

                    // Check if there's a movement threshold
                    #if (DEFAULT_PID_DISABLE_THRESHOLD > 0)

                        // Check to make sure that the movement threshold is exceeded, otherwise disable the motor
                        if (stepFreq > DEFAULT_PID_DISABLE_THRESHOLD) {

                            // Set the speed
                            stepScheduleTimer -> setOverflow(stepFreq, HERTZ_FORMAT);

                            // Enable the timer if it isn't already
                            enableStepScheduleTimer();

                            // Enable the motor
                            motor.setState(ENABLED);
                        }
                        else {
                            // No correction needed, pause the timer
                            disableStepScheduleTimer();
                            motor.setState(DISABLED);
                        }
                    #else
                        // Set the motor timer to call the stepping routine at specified time intervals
                        stepScheduleTimer -> setOverflow(stepFreq, HERTZ_FORMAT);

                        // Enable the timer if it isn't already
                        enableStepScheduleTimer();
                    #endif
                }

            #else // ! ENABLE_PID
                // Just "dumb" correction based on error direction
                // Set the stepper to move to reduce / eliminate the error
                if (stepDeviation > 0) {

                    // Motor is at a position smaller than the desired one
                    // Use the current angle to find the current step, then add 1
                    motor.step(POSITIVE, 1);

                }
                else {
                    // Motor is at a position larger than the desired one
                    // Use the current angle to find the current step, then subtract 1
                    motor.step(NEGATIVE, 1);
                }
            #endif // ! ENABLE_PID
        }
        else { // Motor is in correct position

            // Disable the PID correction timer if PID is enabled
            #ifdef ENABLE_PID
                disableStepScheduleTimer();
            #endif
        }
    }

    #ifdef CHECK_CORRECT_MOTOR_RATE
        GPIO_WRITE(LED_PIN, LOW);
    #endif
}


// Direct stepping
#ifdef ENABLE_DIRECT_STEPPING
// Configure a specific number of steps to execute at a set rate (rate is in Hz)
#ifdef ENABLE_ACCELERATION
    void scheduleSteps(int64_t count, int32_t rate, uint16_t acceleration, STEP_DIR stepDir)
#else
    void scheduleSteps(int64_t count, int32_t rate, STEP_DIR stepDir)
#endif
 {

    // Disable the correctional timer (needed to prevent both using the step timer at once)
    #ifndef DISABLE_CORRECTION_TIMER
    correctionTimer -> pause();
    #endif
    syncInstructions();

    // Set the count and step direction
    remainingScheduledSteps = abs(count);
    decrementRemainingSteps = true;
    scheduledStepDir = stepDir;
    maximumSteppingRate = rate;
    
    #ifdef ENABLE_ACCELERATION

        // Reset all the acceleration helping variables
        actualSteppingRate = ACCEL_START_STEP_RATE;
        stepsToAccelerate = 0;
        halfStepsScheduled = remainingScheduledSteps/2;
        
        #ifdef RAMP_ACCELERATION
            oneStepAcceleration = (uint32_t)acceleration/correctionUpdateFreq;
        #endif
        
        accelerate = 1;

        #ifdef S_CURVE_ACCELERATION
            sCurveElementsNum = (uint16_t)(2*acceleration/(1000/correctionUpdateFreq));
            char buffer[32];
            //sprintf(buffer, "Elem num %d\r\n", sCurveElementsNum);
            sendSerialMessage(buffer);
            sCurveActualIndex = 0;
            generateSCurve(sCurveElementsNum, rate);
        #endif

        // Configure the speed of the timer to the start rate
        stepScheduleTimer -> setOverflow(ACCEL_START_STEP_RATE, HERTZ_FORMAT);
    #else
        // Configure the speed of the timer
        stepScheduleTimer -> setOverflow(rate, HERTZ_FORMAT);
    #endif

    // Reenable the timer
    enableStepScheduleTimer();

    // Increase or decrease desired step counter
    if(stepDir == POSITIVE){
        motor.setDesiredStep(motor.getDesiredStep() + remainingScheduledSteps);
    }
    else{
        motor.setDesiredStep(motor.getDesiredStep() - remainingScheduledSteps);
    }

    // Set active stepping flag
    motor.steppingRequest();

    // Enable the motor
    motor.setState(ENABLED, true);
}
#endif

#if (defined(ENABLE_DIRECT_STEPPING) || defined(ENABLE_PID))
// Handles a step schedule event
void stepScheduleHandler() {
    // If we moving desired number of steps - count number of steps done
    if (decrementRemainingSteps) {
        // Make a step
        motor.step(scheduledStepDir, 1);

        // Decrement the counter
        remainingScheduledSteps--;

        #ifdef ENABLE_ACCELERATION
            
            if(accelerate == 1)
            {
                // If acceleration didnt stop halfway - start decelerating
                if (remainingScheduledSteps <= halfStepsScheduled)
                {
                    accelerate = -1;
                }
                // Count number of steps needed for acceleration
                stepsToAccelerate++;
            }

            // Start decelerating if the point of deceleration reached
            if(stepsToAccelerate >= remainingScheduledSteps)
            {
                accelerate = -1;
            }

            // Disable the timer if there are no remaining steps
            if (remainingScheduledSteps <= 0) { 
                // Pause the step timer (will be re-enabled by the PID loop)
                disableStepScheduleTimer();
                accelerate = -99;
            }

        #endif

        // Enable PID correction timer
        correctionTimer -> resume();
    }
    // if PID is just correcting the position
    // No need to worry about remaining steps
    else {
        // Just step the motor in the desired direction
        motor.step(scheduledStepDir, 1);
    }
}


// Convenience function to handle enabling the step schedule timer
void enableStepScheduleTimer() {
    if (!stepScheduleTimerEnabled) {
        stepScheduleTimer -> resume();
        stepScheduleTimerEnabled = true;
        syncInstructions();
    }
}


// Convenience function to handle disabling the step schedule timer
void disableStepScheduleTimer() {
    if (stepScheduleTimerEnabled) {
        stepScheduleTimer -> pause();
        stepScheduleTimerEnabled = false;
        syncInstructions();
    }
}
#endif // ! ENABLE_DIRECT_STEPPING


// Makes sure that all cached calls respect the current config
void syncInstructions() {

    // Make sure that the instruction cache is synced
    __DSB();
    __ISB();
}