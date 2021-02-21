// Import the header file
#include "serial.h"

// Arduino library
#include "Arduino.h"

// Initializes serial bus
void initSerial() {
    Serial.begin(115200);
}

// Sends a string to the host
void sendMessage(char* message) {
    Serial.write(message);
}

// Reads the string in the buffer
String readBuffer() {
    return Serial.readString();
}

// Returns if there is serial data to be read
bool checkSerialData() {
    return (Serial.available() > 0);
}

// Parse the buffer for commands
void runSerialParser() {
    // ! Write yet (use Marlin's parser for reference)

    // GCode Table
    //   - M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
    //   - M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
    //   - M307 (ex M307) - Runs an autotune sequence for the PID loop
    //   - M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
    //   - M500 (ex M500) - Saves the currently loaded parameters into flash
    //   - M907 (ex M907 V3000) - Sets the current in mA

    // Load the string for modification
    String buffer = readBuffer();

    // Make sure that the first value is the command letter
    if (buffer.charAt(0) == (char)"M" || buffer.charAt(0) == (char)"m") {

        // Create an accumulator for the number
        int commandNumber;

        // If it is, we can proceed by reading the number of the code
        if (buffer.charAt(3) == (char)" ") {

            // The command number only has two numbers (first index is the letter)
            commandNumber = atoi(buffer.substring(1, 2).c_str());
        }
        else {

            // The command number has 3 numbers
            commandNumber = atoi(buffer.substring(1, 3).c_str());
        }

        // Switch statement the command number
        switch (commandNumber) {

            case 93: 
                // M93 (ex M93 V1.8) - Sets the angle of a full step. This value should be 1.8° or 0.9°
                break;

            case 306:
                // M306 (ex M306 P1 I1 D1) - Sets the PID values for the motor
                break;

            case 307:
                // M307 (ex M307) - Runs an autotune sequence for the PID loop
                break;

            case 350:
                // M350 (ex M350 V16) - Sets the microstepping divisor for the motor. This value can be 1, 2, 4, 8, 16, or 32
                break;

            case 500:
                // M500 (ex M500) - Saves the currently loaded parameters into flash
                break;

            case 907:
                // M907 (ex M907 V3000) - Sets the current in mA
                break;

        }

    }
}
