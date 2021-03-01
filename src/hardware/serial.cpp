// Import the header file
#include "serial.h"

// Arduino library
#include "Arduino.h"

// The buffer for serial data between serial reads
String serialBuffer;

// Initializes serial bus
void initSerial() {
    Serial.begin(115200);
}

// Sends a string to the host
void sendMessage(String message) {
    Serial.write(message.c_str());
}

// Reads the string in the buffer
String readSerialBuffer() {

    // Local variables for storing relevant information
    char receivedChars[SERIAL_BUFFER_LENGTH];
    static int newIndex = 0;
    char readChar;

    // Check if there is serial data available to read
    if (Serial.available() > 0) {

        // Read the first character. If the first is the string start indicator, then we can continue with the read
        if (Serial.read() == STRING_START_MARKER) {

            // Loop forever, until the entire string is read
            while (Serial.available() > 0) {

                // Read the character out of the buffer
                readChar = Serial.read();

                // If the character isn't the end, add it to the received characters
                if (readChar != STRING_END_MARKER) {

                    // Add it to the character list
                    receivedChars[newIndex] = readChar;

                    // Increment the counter
                    newIndex++;

                    // Make sure that the serial buffer isn't overrun
                    if (newIndex >= SERIAL_BUFFER_LENGTH) {
                        newIndex = SERIAL_BUFFER_LENGTH - 1;
                    }
                }
                else {
                    // End character reached, terminate the string and return
                    receivedChars[newIndex] = '\0';
                    return String(receivedChars);
                }
            }
        }
    
    }

    // If we made it this far, the read failed and we should return a -1 (will be ignored)
    return String(-1);
}

// Parse the buffer for commands
void runSerialParser() {

    // Save the value as the result is resource intensive to produce
    String serialCommand = readSerialBuffer();

    // Check that it is an actual command, then proceed
    if (serialCommand != "-1") {

        // Send the feedback from the serial command
        sendMessage(parseString(serialCommand));
    }
}