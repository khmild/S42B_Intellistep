/*
 *  Based on Smoothed.h
 *  Store and calculate smoothed values from sensors.
 *  Created by Matt Fryer on 2017-11-17, rewritten by Christian Piper (CAP1Sup) on 2020-5-10
 *  Licensed under LGPL
 */

#pragma once

// Standard naming conventions
#include "stdint.h"


// A class used to store and calculate the values to be smoothed.
template <typename T>
class MovingAverage {
  private:
    uint16_t readingsFactor = 10; // The smoothing factor. In average mode, this is the number of readings to average.
    uint16_t readingsPosition = 0; // Current position in the array
    uint16_t readingsNum = 0; // Number of readings currently being averaged
    T *readings; // Array of readings

    // ! Dynamically decide which type to use for the total
    double runningTotal; // A cache of the total of the array, speeds up getting the average

  public:
    MovingAverage();
    ~MovingAverage(); // Destructor to clean up when class instance killed
    bool begin(uint16_t smoothFactor = 10);
    bool add(T newReading);
    T get();
    T getLast();
    bool clear();
};


// Constructor
template <typename T>
MovingAverage<T>::MovingAverage () {}


// Destructor
template <typename T>
MovingAverage<T>::~MovingAverage () { // Destructor
    delete[] readings;
}


// Initialize the array for storing sensor values
template <typename T>
bool MovingAverage<T>::begin (uint16_t smoothFactor) {

    // Store the number of readings in the array
    readingsFactor = smoothFactor;

    // Create the actual array of the required size
    readings = new T[readingsFactor];

    // Initialise all the values in the array to zero
    for (uint16_t thisReading = 0; thisReading < readingsNum; thisReading++) {
        readings[thisReading] = 0;
    }

    // Set the running total to zero
    runningTotal = 0;

    return true;
}


// Add a value to the array
template <typename T>
bool MovingAverage<T>::add (T newReading) {

    // Keep record of the number of readings being averaged
    // This will count up to the array size then stay at that number
    if(readingsNum < readingsFactor) {
        readingsNum++;

        // Add the new value to the running total
        runningTotal += newReading;

        // Replace the zero in the array
        readings[readingsPosition] = newReading;
    }
    else {
        // Add the new value to the running total
        runningTotal += newReading;

        // Remove the old value from the running total
        runningTotal -= readings[readingsPosition];

        // Replace the old value in the array
        readings[readingsPosition] = newReading;
    }

    // If at the end of the array
    if (readingsPosition == (readingsFactor - 1)) {

        // Increment to the beginning of the array
        readingsPosition = 0;
    }
    else {
        // Increment to next array position position
        readingsPosition++;
    }

    // Return that we were able to successfully finish the process
    return true;
}


// Get the smoothed result
template <typename T>
T MovingAverage<T>::get() {
    return (runningTotal / readingsNum);
}


// Gets the last result stored
template <typename T>
T MovingAverage<T>::getLast() {

    // Just return the last reading
    if (readingsPosition == 0) {
        return readings[readingsFactor - 1];
    }
    else {
        return readings[readingsPosition - 1];
    }
}


// Clears all stored values
template <typename T>
bool MovingAverage<T>::clear () {

    // Reset the counters
    readingsPosition = 0;
    readingsNum = 0;
    runningTotal = 0;

    // Return true if successful
    return true;
}