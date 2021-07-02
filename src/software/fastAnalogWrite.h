#ifndef __FAST_ANALOG_WRITE__
#define __FAST_ANALOG_WRITE__

// For type definitions
#include "Arduino.h"

// For all of the config options
#include "config.h"

// Class to store analog info
class analogInfo {

    // Store the pin, HardwareTimer pointer, and the channel publicy so they can be accessed
    public:
        PinName pin;
        HardwareTimer *HTPointer;
        uint32_t channel;
};

// Functions
analogInfo analogSetup(PinName pin, uint32_t freq, uint16_t startingValue);
void analogSet(analogInfo* pinInfo, uint16_t value);

#endif // ! __FAST_ANALOG_WRITE__