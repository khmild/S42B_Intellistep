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
String readSerialBuffer() {
    return Serial.readString();
}

// Returns if there is serial data to be read
bool checkSerialData() {
    return (Serial.available() > 0);
}

// Parse the buffer for commands
void runSerialParser() {
    parseString(readSerialBuffer());
}