#include "fastAnalogWrite.h"

// Sets up an analog pin with the specified value
// Starting value can be between 0 and 4095 (PWM_MAX_DUTY_CYCLE)
analogInfo analogSetup(PinName pin, uint32_t freq, uint32_t startingValue) {

    // Create a structure for the analogInfo
    analogInfo pinInfo;

    // Set the pin (not really used, but good to keep track of)
    pinInfo.pin = pin;

    // Setup the pin
    pinMode(pin, OUTPUT);
    
    // Start the PWM with the Arduino style function, using the highest possible resolution
    pwm_start(pin, freq, startingValue, RESOLUTION_12B_COMPARE_FORMAT);

    // Find the timer that is used with the pin, set it in the analogInfo
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    uint32_t index = get_timer_index(Instance);
    pinInfo.HTPointer = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

    // Get the channel for the pin, then set that in the analogInfo as well
    pinInfo.channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));

    // Return the analogInfo
    return pinInfo;
}

// Sets the value to output on the analog pin using the structure that was returned when setting it up
// Value should be in range of 0 and 4095 (PWM_MAX_DUTY_CYCLE)
void analogSet(analogInfo* pinInfo, uint32_t value) {

    // Check to make sure that the value is between 0 and 4095 (PWM_MAX_DUTY_CYCLE)
    // Very bad things could happen if this is not checked properly
    value = constrain(value, 0, PWM_MAX_DUTY_CYCLE);

    // Only need to set the capture compare
    pinInfo->HTPointer->setCaptureCompare(pinInfo->channel, value, RESOLUTION_12B_COMPARE_FORMAT);
}