// Make sure that the file is only included once
#pragma once

// Import the config
#include "config.h"


// Check to make sure that at least a peak or RMS max current is defined
#if !(defined(MAX_PEAK_BOARD_CURRENT) || defined(MAX_RMS_BOARD_CURRENT))
    #error "A max peak or max RMS current must be defined!"

// Check to make sure both aren't defined already
#elif (defined(MAX_PEAK_BOARD_CURRENT) && defined(MAX_RMS_BOARD_CURRENT))
    #error "Only max peak or max RMS current can be defined, not both!"
#else
    // Calculate the other current maximum
    #ifdef MAX_PEAK_BOARD_CURRENT
        #define MAX_RMS_BOARD_CURRENT (uint16_t)(MAX_PEAK_BOARD_CURRENT * 0.707)
    #else
        #define MAX_PEAK_BOARD_CURRENT (uint16_t)(MAX_RMS_BOARD_CURRENT * 1.414)
    #endif
#endif

// Warning settings only need checked when the OLED is enabled
#ifdef ENABLE_OLED
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

// Check to make sure that at least a peak or RMS default current is defined
#if !(defined(STATIC_PEAK_CURRENT) || defined(STATIC_RMS_CURRENT)) && !defined(ENABLE_DYNAMIC_CURRENT)
    #error "A static peak or static RMS current must be defined!"

// Check to make sure both aren't defined already
#elif (defined(STATIC_PEAK_CURRENT) && defined(STATIC_RMS_CURRENT)) && !defined(ENABLE_DYNAMIC_CURRENT)
    #error "Only max peak or max RMS current can be defined, not both!"
#elif !defined(ENABLE_DYNAMIC_CURRENT)
    // Calculate the other current maximum
    #ifdef STATIC_PEAK_CURRENT
        #define STATIC_RMS_CURRENT (uint16_t)(STATIC_PEAK_CURRENT * 0.707)
    #else
        #define STATIC_PEAK_CURRENT (uint16_t)(STATIC_RMS_CURRENT * 1.414)
    #endif
#endif