#include "parser.h"

// Parses an entire string for any commands
String parseString(String buffer) {
    // ! Add more gcodes to be able to set inversion

    // GCode Table
    //   - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
    //   - M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
    //   - M307 (ex M307) - Runs an autotune sequence for the PID loop
    //   - M308 (ex M308) - Runs the manual PID tuning interface. Serial is filled with encoder angles
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
                return FEEDBACK_OK;
            }
            case 306: {
                // M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
                motor.setPValue(parseValue(buffer, 'P').toFloat());
                motor.setIValue(parseValue(buffer, 'I').toFloat());
                motor.setDValue(parseValue(buffer, 'D').toFloat());
                return FEEDBACK_OK;
            }
            case 307: {
                // M307 (ex M307) - Runs a automatic calibration sequence for the PID loop and encoder
                motor.calibrate();
                return FEEDBACK_OK;
            }
            case 308: {
                // M308 (ex M308) - Runs the manual PID tuning interface. Serial is filled with encoder angles

                // Print a notice to the user that the PID tuning is starting
                Serial.println("Notice: The manual PID tuning is now starting. To exit, send any serial data.");

                // Wait for the user to read it
                delay(1000);

                // Loop forever, until a new value is sent
                while (!Serial.available()) {
                    Serial.println(getEncoderAngle());
                }
                return FEEDBACK_OK;
            }
            case 350: {
                // M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
                motor.setMicrostepping(parseValue(buffer, 'V').toInt());
                return FEEDBACK_OK;
            }
            case 500: {
                // M500 (ex M500) - Saves the currently loaded parameters into flash
                saveParametersToFlash();
                return FEEDBACK_OK;
            }
            case 907: {
                // M907 (ex M907 V3000) - Sets the current in mA
                motor.setCurrent(parseValue(buffer, 'V').toInt());
                return FEEDBACK_OK;
            }
        }
    }

    // Nothing here, nothing to do
    return "No command specified";
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
        Serial.println(FEEDBACK_NO_VALUE);
        return "-1";
    }
}