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
void sendMessage(char* message) {
    Serial.write(message);
}

// Reads the string in the buffer
// ! Needs work, should clear buffer and only return if string end character is found
String readSerialBuffer() {

    // Check if there is serial data to be read
    if (Serial.available()) {

        // Add the latest serial data to the buffer
        serialBuffer += Serial.read();
    }

    // Return the buffer if it's full
    return serialBuffer;
}

// Parse the buffer for commands
// ! Fix to use start and end characters
void runSerialParser() {
    parseString(readSerialBuffer());
}