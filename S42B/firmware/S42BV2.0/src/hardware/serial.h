#ifndef __SERIAL_H__
#define __SERIAL_H__

// Imports
#include "main.h"
#include "string.h"
#include <cstdlib>
#include "flash.h"
#include <cctype>

// Defines for strings that are used repeatedly
#define noValue "No value specified! Make sure to specify a value with a V before it"

void initSerial();
void sendMessage(String message);
String readBuffer();
bool checkSerialData();
void runSerialParser();
String parseValue(String buffer, char letter);

#endif