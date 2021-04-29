// Import the config (needed for the USE_SERIAL or USE_CAN defines)
#include "config.h"

// Only include if the serial or CAN bus is enabled
#if defined(USE_SERIAL) || defined(USE_CAN)

#include "parser.h"

// Parses an entire string for any commands
String parseString(String buffer) {

    // GCode Table
    //   - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
    //   - M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
    //   - M307 (ex M307) - Runs an autotune sequence for the PID loop
    //   - M308 (ex M308) - Runs the manual PID tuning interface. Serial is filled with encoder angles
    //   - M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
    //   - M352 (ex M320 S1) - Sets the direction pin inversion for the motor (0 is standard, 1 is inverted)
    //   - M353 (ex M353 S1) - Sets the enable pin inversion for the motor (0 is standard, 1 is inverted)
    //   - M354 (ex M354 S1) - Sets if the motor dip switches were installed incorrectly (reversed) (0 is standard, 1 is inverted)
    //   - M355 (ex M355 V1.34) - Sets the microstep multiplier for the board. Allows to use multiple motors connected to the same mainboard pin.
    //   - M356 (ex M356 V1 or M356 VX2) - Sets the CAN ID of the board. Can be set using the axis character or actual ID.
    //   - M500 (ex M500) - Saves the currently loaded parameters into flash
    //   - M907 (ex M907 V3000) - Sets the current in mA

    // Change the string to uppercase (allows flexibility for upper and lower case characters)
    buffer.toUpperCase();

    // ! Check to see if the string contains another set of gcode, if so call the function recursively

    // Check to see if the letter is an M (only supported gcode at this point)
    if (parseValue(buffer, 'M') != "-1") {

        // Switch statement the command number
        switch (parseValue(buffer, 'M').toInt()) {

            case 17:
                // M17 (ex M17) - Enables the motor (overrides enable pin)
                motor.enable(true);
                return FEEDBACK_OK;

            case 18:
                // M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
                motor.disable(true);
                return FEEDBACK_OK;

            case 84:
                // M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
                motor.disable(true);
                return FEEDBACK_OK;

            case 93:
                // M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
                motor.setFullStepAngle(parseValue(buffer, 'V').toFloat());
                return FEEDBACK_OK;

            case 115:
                // M115 (ex M115) - Prints out firmware information.
                return FIRMWARE_FEATURE_PRINT;

            case 306:
                // M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
                motor.setPValue(parseValue(buffer, 'P').toFloat());
                motor.setIValue(parseValue(buffer, 'I').toFloat());
                motor.setDValue(parseValue(buffer, 'D').toFloat());
                return FEEDBACK_OK;

            case 307:
                // M307 (ex M307) - Runs a automatic calibration sequence for the PID loop and encoder
                motor.calibrate();
                return FEEDBACK_OK;

            case 308:
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

            case 350:
                // M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
                motor.setMicrostepping(parseValue(buffer, 'V').toInt());
                return FEEDBACK_OK;

            case 352:
                // M352 (ex M320 S1) - Sets the direction pin inversion for the motor (0 is standard, 1 is inverted)
                motor.setReversed(parseValue(buffer, 'S').compareTo("1"));
                return FEEDBACK_OK;

            case 353:
                // M353 (ex M353 S1) - Sets the enable pin inversion for the motor (0 is standard, 1 is inverted)
                motor.setEnableInversion(parseValue(buffer, 'S').compareTo("1"));
                return FEEDBACK_OK;

            case 354:
                // M354 (ex M354 S1) - Sets if the motor dip switches were installed incorrectly (reversed) (0 is standard, 1 is inverted)
                setDipInverted(parseValue(buffer, 'S').compareTo("1"));
                return FEEDBACK_OK;

            case 355:
                // M355 (ex M355 V1.34) - Sets the microstep multiplier for the board. Allows to use multiple motors connected to the same mainboard pin.
                motor.setMicrostepMultiplier(parseValue(buffer, 'V').toFloat());
                return FEEDBACK_OK;

            case 356:

                // Only build in functionality if specified
                #ifdef USE_CAN
                    // M356 (ex M356 V1 or M356 VX2) - Sets the CAN ID of the board. Can be set using the axis character or actual ID.
                    if (parseValue(buffer, 'V').toInt() == 0) {

                        // Value is a character, process it once so that it can be used in the if statements
                        String axisValue = parseValue(buffer, 'V');

                        // Compare the values of the received value with the expected ones
                        if (axisValue == "X" || axisValue == "X1") {
                            setCANID(X);
                        }
                        else if (axisValue == "X2") {
                            setCANID(X2);
                        }
                        else if (axisValue == "X3") {
                            setCANID(X3);
                        }
                        else if (axisValue == "X4") {
                            setCANID(X4);
                        }
                        else if (axisValue == "X5") {
                            setCANID(X5);
                        }
                        else if (axisValue == "Y" || axisValue == "Y1") {
                            setCANID(Y);
                        }
                        else if (axisValue == "Y2") {
                            setCANID(Y2);
                        }
                        else if (axisValue == "Y3") {
                            setCANID(Y3);
                        }
                        else if (axisValue == "Y4") {
                            setCANID(Y4);
                        }
                        else if (axisValue == "Y5") {
                            setCANID(Y5);
                        }
                        else if (axisValue == "Z" || axisValue == "Z1") {
                            setCANID(Z);
                        }
                        else if (axisValue == "Z2") {
                            setCANID(Z2);
                        }
                        else if (axisValue == "Z3") {
                            setCANID(Z3);
                        }
                        else if (axisValue == "Z4") {
                            setCANID(Z4);
                        }
                        else if (axisValue == "Z5") {
                            setCANID(Z5);
                        }
                        else if (axisValue == "E" || axisValue == "E1") {
                            setCANID(E);
                        }
                        else if (axisValue == "E2") {
                            setCANID(E2);
                        }
                        else if (axisValue == "E3") {
                            setCANID(E3);
                        }
                        else if (axisValue == "E4") {
                            setCANID(E4);
                        }
                        else if (axisValue == "E5") {
                            setCANID(E5);
                        }
                        else {
                            return FEEDBACK_NO_VALUE;
                        }

                        // Return operation successful
                        return FEEDBACK_OK;
                    }
                    else {
                        // Value is a number
                        setCANID(AXIS_CAN_ID(parseValue(buffer, 'V').toInt()));

                        // Return that the operation is complete
                        return FEEDBACK_OK;
                    }

                #else
                    // Return that the feature is not enabled
                    return FEEDBACK_CAN_NOT_ENABLED;
                #endif

            case 500:
                // M500 (ex M500) - Saves the currently loaded parameters into flash
                saveParametersToFlash();
                return FEEDBACK_OK;

            case 907:
                // M907 (ex M907 V3000) - Sets the current in mA
                motor.setCurrent(parseValue(buffer, 'V').toInt());
                return FEEDBACK_OK;

        }
    }

    // Nothing here, nothing to do
    return F("No command specified\n");
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
            return buffer.substring(charIndex + 1, buffer.lastIndexOf(' '));
        }
        else {
            // Get the rest of the string after the letter (should only be a number)
            return buffer.substring(charIndex + 1);
        }
    }
    else {
        // Index is invalid, V doesn't exist. Print an output message, then return a null
        Serial.println(FEEDBACK_NO_VALUE);
        return "-1";
    }
}

#endif // (USE_SERIAL || USE_CAN)