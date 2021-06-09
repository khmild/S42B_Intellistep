// Import the config (needed for the ENABLE_SERIAL or ENABLE_CAN defines)
#include "config.h"

// Only include if the serial or CAN bus is enabled
#if defined(ENABLE_SERIAL) || defined(ENABLE_CAN)

#include "parser.h"

// Parses an entire string for any commands
String parseCommand(String buffer) {

    // GCode Table
    //   - M17 (ex M17) - Enables the motor (overrides enable pin)
    //   - M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
    //   - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
    //   - M115 (ex M115) - Prints out firmware information.
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
                motor.setState(ENABLED, true);
                return FEEDBACK_OK;

            case 18:
                // M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
                motor.setState(FORCED_DISABLED);
                return FEEDBACK_OK;

            case 84:
                // M18 / M84 (ex M18 or M84) - Disables the motor (overrides enable pin)
                motor.setState(FORCED_DISABLED);
                return FEEDBACK_OK;

            case 93:
                // M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
                motor.setFullStepAngle(parseValue(buffer, 'V').toFloat());
                return FEEDBACK_OK;

            case 115:
                // M115 (ex M115) - Prints out firmware information.
                return FIRMWARE_FEATURE_PRINT;

            #ifdef ENABLE_CAN
            case 116:
                // M116 (ex M116 S1) - Simple ping command that will send a message back to the sender. 
                // S value should be the CAN ID of the sender. If S is -1, then 
                txCANString(parseValue(buffer, 'S').toInt(), parseString(buffer, 'M'));
            #endif

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

                // Disable interrupts (so value printing is never stopped)
                disableInterrupts();

                // Print a notice to the user that the PID tuning is starting
                Serial.println("Notice: The manual PID tuning is now starting. To exit, send any serial data.");

                // Wait for the user to read it
                delay(1000);

                // Clear the serial buffer
                while (Serial.available() > 0) {
                    Serial.read();
                }

                // Loop forever, until a new value is sent
                while (!(Serial.available() > 0)) {
                    Serial.println(getAbsoluteAngle());
                }

                // Re-enable interrupts
                enableInterrupts();

                // When all done, the exit is acknowledged
                return FEEDBACK_OK;

            case 350:
                // M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
                motor.setMicrostepping(parseValue(buffer, 'V').toInt());
                updateCorrectionTimer();
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
                #ifdef ENABLE_CAN
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
                // M907 (ex M907 R3000 or M907 P3000) - Sets the RMS or Peak current in mA
                #ifndef ENABLE_DYNAMIC_CURRENT
                    motor.setRMSCurrent(parseValue(buffer, 'V').toInt());
                    motor.setPeakCurrent(parseValue(buffer, 'P').toInt());
                #endif
                return FEEDBACK_OK;

        }
    }

    // Nothing here, nothing to do
    return F("No command specified\n");
}


// Returns the substring of the value after the letter parameter
String parseValue(String buffer, char letter) {

    // Search the input string for the specified letter
    uint16_t charIndex = buffer.indexOf(toupper(letter));

    // If the index came back with a value, we can begin the process of extracting the raw value
    if (charIndex != -1) {

        // Get the next index of a space
        uint16_t nextSpaceIndex = buffer.substring(charIndex).indexOf(' ');

        // Check to see if there is a space between the letter and value
        if (nextSpaceIndex == charIndex + 1) {

            // We need to find out if there is another space after this parameter
            uint16_t endSpaceIndex = buffer.substring(nextSpaceIndex + 1).indexOf(' ');

            // Check to see if there is an ending space
            if (endSpaceIndex != -1) {

                // Return only the substring between the ending space and the space after the letter
                return buffer.substring(nextSpaceIndex + 1, endSpaceIndex - 1);
            }
            else {
                // That's the end of the string, we can start at the space and just include the rest
                return buffer.substring(nextSpaceIndex + 1);
            }
        }
        else if (nextSpaceIndex != -1) {
            // The next space index is after the value, so just include up to it
            return buffer.substring(charIndex + 1, nextSpaceIndex - 1);
        }
        else {
            // There is no more spaces in the string, therefore just return the rest of the string
            return buffer.substring(charIndex + 1);
        }
    }
    else {
        // Index is invalid, V doesn't exist. Print an output message, then return a null
        Serial.println(FEEDBACK_NO_VALUE);
        return "-1";
    }
}


// Returns the substring of the string after the letter parameter
String parseString(String buffer, char letter) {

    // Search the input string for the specified letter
    uint16_t charIndex = buffer.indexOf(toupper(letter));

    // If the index came back with a value, we can begin the process of extracting the raw value
    if (charIndex != -1) {

        // Time to check to see where the double quotations are
        uint16_t startQuotationIndex = buffer.substring(charIndex + 1).indexOf('"');
        uint16_t endQuotationIndex = buffer.substring(startQuotationIndex + 1).indexOf('"');

        // Make sure that there are quotations on each side
        if (startQuotationIndex != -1 && endQuotationIndex != -1) {
            
            // Return the string between them
            return buffer.substring(startQuotationIndex + 1, endQuotationIndex - 1);
        }
        else {
            // Throw an error, we can't find both the quotation marks
            Serial.println(FEEDBACK_INVALID_STRING);
            return "-1";
        }
    }
    else {
        // Index is invalid, letter doesn't exist. Print an output message, then return a null
        Serial.println(FEEDBACK_NO_VALUE);
        return "-1";
    }
}

#endif // (ENABLE_SERIAL || ENABLE_CAN)