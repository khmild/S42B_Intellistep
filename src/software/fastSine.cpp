#include "fastSine.h"

// Main sine lookup table
static const int16_t sineTable[SINE_VAL_COUNT] = {
     0,    490,    980,   1467,   1950,   2429,   2902,   3368,   3826,   4275,
  4713,   5141,   5555,   5956,   6343,   6715,   7071,   7409,   7730,   8032,
  8314,   8577,   8819,   9039,   9238,   9415,   9569,   9700,   9807,   9891,
  9951,   9987,  10000,   9987,   9951,   9891,   9807,   9700,   9569,   9415,
  9238,   9039,   8819,   8577,   8314,   8032,   7730,   7409,   7071,   6715,
  6343,   5956,   5555,   5141,   4713,   4275,   3826,   3368,   2902,   2429,
  1950,   1467,    980,    490,      0,   -490,   -980,  -1467,  -1950,  -2429,
 -2902,  -3368,  -3826,  -4275,  -4713,  -5141,  -5555,  -5956,  -6343,  -6715,
 -7071,  -7409,  -7730,  -8032,  -8314,  -8577,  -8819,  -9039,  -9238,  -9415,
 -9569,  -9700,  -9807,  -9891,  -9951,  -9987, -10000,  -9987,  -9951,  -9891,
 -9807,  -9700,  -9569,  -9415,  -9238,  -9039,  -8819,  -8577,  -8314,  -8032,
 -7730,  -7409,  -7071,  -6715,  -6343,  -5956,  -5555,  -5141,  -4713,  -4275,
 -3826,  -3368,  -2902,  -2429,  -1950,  -1467,   -980,   -490
};


// Fast sin function
int16_t fastSin(uint16_t angle) {

    // Nothing fancy, just return the angle
    return sineTable[angle];
}


// Fast cosine function
int16_t fastCos(uint16_t angle) {

    // Shift the angle 90 degrees forward
    angle += (SINE_VAL_COUNT / 4);

    // Check to make sure that the angle still within bounds. If not, then loop it back around
    if (angle >= SINE_VAL_COUNT) {
        angle -= SINE_VAL_COUNT;
    }
    else if (angle < 0) {
        angle += SINE_VAL_COUNT;
    }

    // Return the value in the table
	return sineTable[angle];
}