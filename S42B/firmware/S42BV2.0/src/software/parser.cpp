#include "parser.h"

// Parses an entire string for any commands
void parseString(String buffer) {
    // ! Add more gcodes to be able to set inversion

    // GCode Table
    //   - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
    //   - M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
    //   - M307 (ex M307) - Runs an autotune sequence for the PID loop
    //   - M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
    //   - M500 (ex M500) - Saves the currently loaded parameters into flash
    //   - M907 (ex M907 V3000) - Sets the current in mA

    // Change the string to uppercase (allows flexibility for upper and lower case characters)
    buffer.toUpperCase();

    // ! Check to see if the string contains another set of gcode, if so call the function recursively

    // Check to see if the letter is an M (only supported gcode at this point)
    if (parseValue(buffer, 'M') != "-1") {

        // Switch statement the command number
        switch (parseValue(buffer, 'M').toInt()) {

            case 93: {
                // M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
                motor.setFullStepAngle(parseValue(buffer, 'V').toFloat());
                break;
            }
            case 306: {
                // M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
                motor.setPValue(parseValue(buffer, 'P').toFloat());
                motor.setIValue(parseValue(buffer, 'I').toFloat());
                motor.setDValue(parseValue(buffer, 'D').toFloat());
                break;
            }
            case 307: {
                // M307 (ex M307) - Runs a calibration sequence for the PID loop and encoder
                motor.calibrate();
                break;
            }
            case 350: {
                // M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
                motor.setMicrostepping(parseValue(buffer, 'V').toInt());
                break;
            }
            case 500: {
                // M500 (ex M500) - Saves the currently loaded parameters into flash
                saveParametersToFlash();
                break;
            }
            case 907: {
                // M907 (ex M907 V3000) - Sets the current in mA
                motor.setCurrent(parseValue(buffer, 'V').toInt());
                break;
            }
        }
    }
    
}


// Returns the substring of the value after the letter parameter
String parseValue(String buffer, char letter) {

    // Search the input string for a V (for the value measurement)
    int charIndex = buffer.indexOf(toupper(letter));

    // If the index came back with a value, we can convert the value to an integer and set it
    if (charIndex != -1) {

        // Check to see if there is another space in the string before the end (for a string with multiple parameters)
        if (buffer.lastIndexOf(' ') > charIndex) {

            // Return only the substring between the last space and the letter
            return buffer.substring(charIndex, buffer.lastIndexOf(' '));
        }
        else {
            // Get the rest of the string after the letter (should only be a number)
            return buffer.substring(charIndex);
        }
    }
    else {
        // Index is invalid, V doesn't exist. Print an output message, then return a null
        Serial.println(noValue);
        return "-1";
    }
}