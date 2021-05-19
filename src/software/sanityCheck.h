// Make sure that the file is only included once
#pragma once

// Import the config
#include "config.h"


// Check to make sure that at least a peak or RMS max current is defined
#if !(defined(MAX_PEAK_CURRENT) || defined(MAX_RMS_CURRENT))
    #error "A max peak or max RMS current must be defined!"

// Check to make sure both aren't defined already
#elif (defined(MAX_PEAK_CURRENT) && defined(MAX_RMS_CURRENT))
    #error "Only max peak or max RMS current can be defined, not both!"
#else
    // Calculate the other current maximum
    #ifdef MAX_PEAK_CURRENT
        #define MAX_RMS_CURRENT (uint16_t)(MAX_PEAK_CURRENT * 0.707)
    #else
        #define MAX_PEAK_CURRENT (uint16_t)(MAX_RMS_CURRENT * 1.414)
    #endif
#endif

// Warning settings only need checked when the OLED is enabled
#ifdef USE_OLED
    // Check to make sure that at least a peak or RMS wanring current is defined
    #if !(defined(WARNING_PEAK_CURRENT) || defined(WARNING_RMS_CURRENT))
        #error "A warning peak or warning RMS current must be defined!"

    // Check to make sure both aren't defined already
    #elif (defined(WARNING_PEAK_CURRENT) && defined(WARNING_RMS_CURRENT))
        #error "Only warning peak or warning RMS current can be defined, not both!"
    #else
        // Calculate the other current warning
        #ifdef WARNING_PEAK_CURRENT
            #define WARNING_RMS_CURRENT (uint16_t)(WARNING_PEAK_CURRENT * 0.707)
        #else
            #define WARNING_PEAK_CURRENT (uint16_t)(WARNING_RMS_CURRENT * 1.414)
        #endif
    #endif
#endif